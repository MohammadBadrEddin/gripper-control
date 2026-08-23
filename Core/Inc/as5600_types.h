/**
 * @file    as5600_types.h
 * @brief   Shared enums and status structs for the AS5600 library.
 *
 * HAL-independent on purpose so both the driver and the estimator (and host
 * unit tests) can share these definitions without pulling in STM32 headers.
 */
#ifndef AS5600_TYPES_H
#define AS5600_TYPES_H

#include <stdint.h>
#include <stdbool.h>

/* CONF: Power Mode (PM) */
typedef enum {
    AS5600_PM_NOM  = 0x0, /* always on   — use this for active closed-loop control */
    AS5600_PM_LPM1 = 0x1, /* polling 5ms   */
    AS5600_PM_LPM2 = 0x2, /* polling 20ms  */
    AS5600_PM_LPM3 = 0x3  /* polling 100ms */
} AS5600_PM_t;

/* CONF: Hysteresis (HYST), in LSB of the 12-bit output */
typedef enum {
    AS5600_HYST_OFF  = 0x0,
    AS5600_HYST_1LSB = 0x1,
    AS5600_HYST_2LSB = 0x2,
    AS5600_HYST_3LSB = 0x3
} AS5600_HYST_t;

/* CONF: Output Stage (OUTS). Irrelevant if you read purely over I2C. */
typedef enum {
    AS5600_OUTS_ANALOG_FULL = 0x0, /* 0%..100% of VDD */
    AS5600_OUTS_ANALOG_RED  = 0x1, /* 10%..90% of VDD */
    AS5600_OUTS_PWM         = 0x2  /* digital PWM      */
} AS5600_OUTS_t;

/* CONF: PWM Frequency (PWMF) — only relevant in PWM output mode */
typedef enum {
    AS5600_PWMF_115HZ = 0x0,
    AS5600_PWMF_230HZ = 0x1,
    AS5600_PWMF_460HZ = 0x2,
    AS5600_PWMF_920HZ = 0x3
} AS5600_PWMF_t;

/* CONF: Slow Filter (SF) — sets step-response delay vs. output noise (Fig. 32) */
typedef enum {
    AS5600_SF_16X = 0x0, /* delay 2.2ms,  noise 0.015 deg */
    AS5600_SF_8X  = 0x1, /* delay 1.1ms,  noise 0.021 deg */
    AS5600_SF_4X  = 0x2, /* delay 0.55ms, noise 0.030 deg */
    AS5600_SF_2X  = 0x3  /* delay 0.286ms,noise 0.043 deg */
} AS5600_SF_t;

/* CONF: Fast Filter Threshold (FTH) — 000 disables the fast filter (Fig. 33) */
typedef enum {
    AS5600_FTH_SLOW_ONLY = 0x0,
    AS5600_FTH_6LSB      = 0x1,
    AS5600_FTH_7LSB      = 0x2,
    AS5600_FTH_9LSB      = 0x3,
    AS5600_FTH_18LSB     = 0x4,
    AS5600_FTH_21LSB     = 0x5,
    AS5600_FTH_24LSB     = 0x6,
    AS5600_FTH_10LSB     = 0x7
} AS5600_FTH_t;

/* CONF: Watchdog (WD) */
typedef enum {
    AS5600_WD_OFF = 0x0,
    AS5600_WD_ON  = 0x1
} AS5600_WD_t;

/**
 * @brief Magnet health flags, decoded from the STATUS register.
 *
 * A trustworthy reading has magnet_detected == true and both weak/strong false.
 */
typedef struct {
    bool magnet_detected;   /* MD: magnet present                    */
    bool magnet_too_weak;   /* ML: AGC maxed out, airgap too large   */
    bool magnet_too_strong; /* MH: AGC bottomed out, airgap too small*/
} AS5600_Status_t;

/**
 * @brief Fault classification surfaced to the control loop.
 *
 * The bus vs. magnet distinction matters: an I2C timeout is a wiring/EMC
 * problem, a lost magnet is a mechanical problem, and the loop may want to
 * react differently to each (hold last-good vs. safe-stop).
 */
typedef enum {
    AS5600_FAULT_NONE = 0,
    AS5600_FAULT_I2C,          /* NACK / timeout / bus error            */
    AS5600_FAULT_MAGNET_LOST,  /* MD == 0                                */
    AS5600_FAULT_MAGNET_WEAK,  /* ML == 1 (warning-grade; still usable)  */
    AS5600_FAULT_MAGNET_STRONG /* MH == 1 (warning-grade; still usable)  */
} AS5600_Fault_t;

#endif /* AS5600_TYPES_H */
