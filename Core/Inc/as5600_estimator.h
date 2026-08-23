/**
 * @file    as5600_estimator.h
 * @brief   Application-level state estimator built on the AS5600 driver.
 *
 * The AS5600 is single-turn. This layer adds what a closed-loop joint needs:
 *   - continuous multi-turn angle (software turn counting / unwrap)
 *   - filtered angular velocity
 *   - a fault state machine so the control loop never trusts a bad reading
 *
 * It is HAL-independent: feed it raw counts + status (from the driver) and a
 * fixed sample period. That keeps it unit-testable on a host.
 */
#ifndef AS5600_ESTIMATOR_H
#define AS5600_ESTIMATOR_H

#include <stdint.h>
#include <stdbool.h>
#include "as5600_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Estimator configuration. Set once at init.
 */
typedef struct {
    float dt_s;            /* control-loop sample period [s] (e.g. 1/1000)     */
    float vel_cutoff_hz;   /* velocity low-pass cutoff [Hz]; 0 disables filter */
    bool  invert;          /* flip sign if mechanical + electrical dir differ  */
} AS5600_EstConfig_t;

/**
 * @brief Estimator internal state + latest outputs.
 */
typedef struct {
    /* configuration (copied in at init) */
    float   dt_s;
    float   vel_alpha;      /* precomputed IIR coefficient */
    float   sign;           /* +1.0f or -1.0f */

    /* raw tracking */
    bool    initialised;    /* false until the first valid sample seeds state  */
    uint16_t raw_prev;      /* previous raw 12-bit reading                      */
    int64_t total_counts;   /* accumulated signed counts across turns          */

    /* outputs (read by the control loop) */
    float   angle_rad;      /* continuous multi-turn angle [rad]                */
    int32_t turn_count;     /* signed completed full turns                      */
    float   velocity_rad_s; /* filtered angular velocity [rad/s]                */
    AS5600_Status_t status; /* last magnet health flags                         */
    AS5600_Fault_t  fault;  /* current fault classification                     */
} AS5600_Estimator_t;

/**
 * @brief Initialise the estimator. Does not read hardware.
 *
 * @note MAXIMUM SAMPLED SPEED CONSTRAINT
 *       The unwrap assumes the joint moves less than half a revolution
 *       (2048 counts) between two samples. Otherwise a wrap is missed and the
 *       multi-turn angle silently jumps. The safe limit is:
 *
 *           max_speed_rev_s < 0.5 / dt_s
 *           i.e. max_rpm    < 30 / dt_s
 *
 *       Verify your worst-case joint speed against this. Use
 *       AS5600_Estimator_MaxSpeedRadS() to compute the limit at runtime, or add
 *       a static assertion in your control code.
 */
void AS5600_Estimator_Init(AS5600_Estimator_t *est, const AS5600_EstConfig_t *cfg);

/**
 * @brief Feed one sample. Call once per control tick after reading the encoder.
 *
 * @param est     estimator instance
 * @param raw12   RAW ANGLE (0..4095) from the driver
 * @param status  magnet health flags from the driver
 * @param i2c_ok  false if the driver read failed this tick (I2C fault)
 *
 * On an I2C fault or lost magnet the position/velocity outputs are held at
 * their last-known-good values and @p fault is set accordingly; the control
 * loop must check @p fault before acting on the estimate.
 */
void AS5600_Estimator_Update(AS5600_Estimator_t *est, uint16_t raw12,
                             AS5600_Status_t status, bool i2c_ok);

/** @brief Seed/reset the origin to the current position (zeroes turn count). */
void AS5600_Estimator_Reset(AS5600_Estimator_t *est);

/** @brief Convenience: latest continuous angle in radians. */
static inline float AS5600_Estimator_AngleRad(const AS5600_Estimator_t *est)
{
    return est->angle_rad;
}

/** @brief Convenience: latest filtered velocity in rad/s. */
static inline float AS5600_Estimator_VelocityRadS(const AS5600_Estimator_t *est)
{
    return est->velocity_rad_s;
}

/** @brief True only when the latest estimate is trustworthy. */
static inline bool AS5600_Estimator_IsHealthy(const AS5600_Estimator_t *est)
{
    /* Weak/strong magnet are warnings, not hard faults: still usable. */
    return est->fault == AS5600_FAULT_NONE ||
           est->fault == AS5600_FAULT_MAGNET_WEAK ||
           est->fault == AS5600_FAULT_MAGNET_STRONG;
}

/**
 * @brief Maximum reliably-trackable speed for a given sample period [rad/s].
 *        Above this, the multi-turn unwrap can miss a revolution.
 */
static inline float AS5600_Estimator_MaxSpeedRadS(float dt_s)
{
    /* half a rev per sample: 0.5 * 2*pi / dt = pi / dt */
    return 3.14159265358979f / dt_s;
}

#ifdef __cplusplus
}
#endif
#endif /* AS5600_ESTIMATOR_H */
