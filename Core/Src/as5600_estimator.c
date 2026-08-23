/**
 * @file    as5600_estimator.c
 * @brief   AS5600 multi-turn / velocity / fault estimator implementation.
 */
#include "as5600_estimator.h"

#define TWO_PI          (6.28318530717958647692f)
#define COUNTS_PER_REV  (4096)
#define HALF_REV        (2048)

/* Convert accumulated signed counts to a continuous angle in radians. */
static float counts_to_rad(int64_t counts)
{
    return ((float)counts) * (TWO_PI / (float)COUNTS_PER_REV);
}

void AS5600_Estimator_Init(AS5600_Estimator_t *est, const AS5600_EstConfig_t *cfg)
{
    est->dt_s           = cfg->dt_s;
    est->sign           = cfg->invert ? -1.0f : 1.0f;
    est->initialised    = false;
    est->raw_prev       = 0;
    est->total_counts   = 0;
    est->angle_rad      = 0.0f;
    est->turn_count     = 0;
    est->velocity_rad_s = 0.0f;
    est->fault          = AS5600_FAULT_NONE;
    est->status.magnet_detected   = false;
    est->status.magnet_too_weak   = false;
    est->status.magnet_too_strong = false;

    /* First-order IIR: alpha = dt / (dt + tau), tau = 1/(2*pi*fc).
     * fc <= 0 disables filtering (alpha = 1, pass-through). */
    if (cfg->vel_cutoff_hz > 0.0f && cfg->dt_s > 0.0f) {
        float tau = 1.0f / (TWO_PI * cfg->vel_cutoff_hz);
        est->vel_alpha = cfg->dt_s / (cfg->dt_s + tau);
    } else {
        est->vel_alpha = 1.0f;
    }
}

void AS5600_Estimator_Reset(AS5600_Estimator_t *est)
{
    est->initialised    = false;
    est->total_counts   = 0;
    est->angle_rad      = 0.0f;
    est->turn_count     = 0;
    est->velocity_rad_s = 0.0f;
}

/* Classify magnet/bus health into a fault code. */
static AS5600_Fault_t classify(bool i2c_ok, AS5600_Status_t s)
{
    if (!i2c_ok) {
        return AS5600_FAULT_I2C;
    }
    if (!s.magnet_detected) {
        return AS5600_FAULT_MAGNET_LOST;
    }
    if (s.magnet_too_weak) {
        return AS5600_FAULT_MAGNET_WEAK;   /* warning-grade */
    }
    if (s.magnet_too_strong) {
        return AS5600_FAULT_MAGNET_STRONG; /* warning-grade */
    }
    return AS5600_FAULT_NONE;
}

void AS5600_Estimator_Update(AS5600_Estimator_t *est, uint16_t raw12,
                             AS5600_Status_t status, bool i2c_ok)
{
    est->status = status;
    est->fault  = classify(i2c_ok, status);

    /* Hard faults (bad bus or no magnet): freeze outputs, don't advance state.
     * The control loop reads est->fault and decides how to react. Velocity is
     * forced to zero so a stale derivative can't drive the actuator. */
    if (est->fault == AS5600_FAULT_I2C || est->fault == AS5600_FAULT_MAGNET_LOST) {
        est->velocity_rad_s = 0.0f;
        return;
    }

    raw12 &= 0x0FFF;

    /* Seed on first good sample: no delta, no velocity yet. */
    if (!est->initialised) {
        est->raw_prev     = raw12;
        est->total_counts = 0;
        est->angle_rad    = 0.0f;
        est->turn_count   = 0;
        est->velocity_rad_s = 0.0f;
        est->initialised  = true;
        return;
    }

    /* Shortest-path delta in [-2048, +2047], resolving the 0/4095 wrap. */
    int32_t delta = (int32_t)raw12 - (int32_t)est->raw_prev;
    if (delta > HALF_REV) {
        delta -= COUNTS_PER_REV;
    } else if (delta < -HALF_REV) {
        delta += COUNTS_PER_REV;
    }

    /* Apply direction sign and accumulate. */
    if (est->sign < 0.0f) {
        delta = -delta;
    }
    est->total_counts += delta;
    est->raw_prev = raw12;

    /* Continuous angle + full-turn count. */
    float prev_angle = est->angle_rad;
    est->angle_rad   = counts_to_rad(est->total_counts);
    est->turn_count  = (int32_t)(est->total_counts / COUNTS_PER_REV);

    /* Velocity: backward difference, then 1st-order IIR low-pass. */
    float raw_vel = (est->angle_rad - prev_angle) / est->dt_s;
    est->velocity_rad_s += est->vel_alpha * (raw_vel - est->velocity_rad_s);
}
