/**
 * @file    tmc2209.h
 * @brief   Minimal TMC2209 UART driver (single-wire, blocking).
 *
 * Bare-minimum: init, register read/write with CRC, set current, set
 * microsteps, and spin the motor via VACTUAL (UART velocity mode -- no STEP
 * pulses needed, which makes it ideal for a first bring-up test).
 *
 * All values verified against TMC2209 datasheet Rev. 1.09 (2023-FEB-16):
 *   - CRC8-ATM, poly 0x07, init 0, applied LSB->MSB (sec. 4.2)
 *   - Write datagram: sync 0x05, addr, reg|0x80, data[4] MSB-first, CRC (4.1.1)
 *   - Read request:   sync 0x05, addr, reg,      CRC   (4 bytes)  (4.1.2)
 *   - Read reply:     sync 0x05, 0xFF, reg, data[4] MSB-first, CRC (8 bytes)
 *   - Bit 7 of the register address = 1 for write (e.g. 0x10 -> 0x90)
 *
 * Usage:
 *     TMC2209 drv;
 *     TMC2209_Init(&drv, &huart2, 0);      // node address 0 (MS1=MS2=low)
 *     TMC2209_SetCurrent(&drv, 16, 8);     // run 16/32, hold 8/32
 *     TMC2209_SetMicrosteps(&drv, 16);
 *     TMC2209_MoveVelocity(&drv, 10000);   // spin
 *     TMC2209_Stop(&drv);
 */
#ifndef TMC2209_H
#define TMC2209_H

#include "stm32f7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"	// included for use of mutex (02.09.2026)
#include "semphr.h"		// included for use of mutex (02.09.2026)

/* Registers used here (datasheet sec. 5) */
#define TMC_GCONF       0x00
#define TMC_GSTAT       0x01
#define TMC_IFCNT       0x02
#define TMC_IHOLD_IRUN  0x10
#define TMC_TPOWERDOWN  0x11
#define TMC_TSTEP       0x12   /* R: measured time between microsteps, units of 1/fCLK */
#define TMC_TPWMTHRS    0x13   /* W: velocity threshold StealthChop <-> SpreadCycle */
#define TMC_TCOOLTHRS   0x14   /* W: velocity threshold below which StallGuard/CoolStep are active */
#define TMC_VACTUAL     0x22
#define TMC_SGTHRS      0x40   /* W: StallGuard4 threshold (DIAG asserts when SG_RESULT < 2*SGTHRS) */
#define TMC_SG_RESULT   0x41   /* R: StallGuard4 load value, 0..1023, low = high load */
#define TMC_COOLCONF    0x42   /* W: CoolStep tuning (SEMIN/SEMAX/SEDN/SEUP/seimin) */
#define TMC_MSCNT       0x6A   /* R: actual microstep position in the sine table, 0..1023 */
#define TMC_MSCURACT    0x6B   /* R: actual coil currents CUR_A/CUR_B */
#define TMC_CHOPCONF    0x6C
#define TMC_DRV_STATUS  0x6F

typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t addr;
    SemaphoreHandle_t mutex;   // NEW: protects any UART-Transaction, included for use of mutex (02.09.2026)

    /* Schattenwerte fuer 1kHz-Telemetrie (main.c/EncoderTestTask): das, was
     * zuletzt tatsaechlich in VACTUAL/IHOLD_IRUN geschrieben wurde. Kein
     * Bus-Zugriff noetig, um sie zu lesen -- bei 460800 Baud kostet ein
     * einzelner Register-READ schon ~260us, fuer ein 1ms-Budget ist da kein
     * Platz fuer zusaetzliche Reads, die der Code sowieso schon kennt. */
    volatile int32_t vactual_shadow;   /* zuletzt per MoveVelocity/Stop geschriebener VACTUAL-Wert */
    volatile uint8_t irun_shadow;      /* zuletzt per SetCurrent geschriebener IRUN-Wert, 0..31 */
    volatile uint8_t ihold_shadow;     /* zuletzt per SetCurrent geschriebener IHOLD-Wert, 0..31 */
} TMC2209;

/**
 * Decoded DRV_STATUS (0x6F) -- driver diagnostics, no separate current
 * sensor needed for most of this. cs_actual is the *commanded* current
 * scale (reflects IRUN / any internal autoscale), NOT a measured current in
 * mA/A -- treat it as a proxy, not a calibrated torque signal.
 */
typedef struct {
    uint8_t  otpw;        /* overtemperature pre-warning (~120C) */
    uint8_t  ot;           /* overtemperature shutdown */
    uint8_t  s2ga, s2gb;   /* short to GND, coil A/B */
    uint8_t  s2vsa, s2vsb; /* short to supply, coil A/B */
    uint8_t  ola, olb;     /* open load, coil A/B (unreliable at low current/StealthChop) */
    uint8_t  stst;         /* 1 = standstill detected */
    uint8_t  stealth;      /* 1 = StealthChop active, 0 = SpreadCycle */
    uint8_t  cs_actual;    /* 0..31, actual current scale in use */
} TMC2209_Status;

/**
 * Bind to a UART and apply a minimal working config.
 * Sets pdn_disable + mstep_reg_select in GCONF (both required for UART
 * control), clears GSTAT, and enables the chopper (TOFF=3 in CHOPCONF).
 * @return true if the driver acknowledged the writes (IFCNT incremented).
 */
bool TMC2209_Init(TMC2209 *drv, UART_HandleTypeDef *huart, uint8_t addr);

/** Write a 32-bit register. */
void TMC2209_Write(TMC2209 *drv, uint8_t reg, uint32_t val);

/** Read a 32-bit register. Returns false on timeout / CRC error. */
bool TMC2209_Read(TMC2209 *drv, uint8_t reg, uint32_t *val);

/** Run/hold current, each 0..31 (0 = 1/32, 31 = 32/32 of full scale). */
void TMC2209_SetCurrent(TMC2209 *drv, uint8_t run, uint8_t hold);

/** Microsteps: 256,128,64,32,16,8,4,2,1. Sets MRES in CHOPCONF. */
void TMC2209_SetMicrosteps(TMC2209 *drv, uint16_t usteps);

/**
 * Spin the motor at a given velocity over UART -- no STEP pulses needed.
 * Sign sets direction. Range +-(2^23 - 1).
 * Speed: v[Hz] = VACTUAL * 0.715 Hz (datasheet sec. 14.1, at fCLK 12 MHz).
 */
void TMC2209_MoveVelocity(TMC2209 *drv, int32_t velocity);

/** Stop the motor (VACTUAL = 0; returns motion control to the STEP pin). */
void TMC2209_Stop(TMC2209 *drv);

/** True if UART comms are working (reads IFCNT successfully). */
bool TMC2209_IsConnected(TMC2209 *drv);

/* ---- StallGuard / CoolStep / diagnostics --------------------------------
 * Enable order matters: SGTHRS + TCOOLTHRS must both be set for SG_RESULT /
 * the DIAG stall flag to mean anything. Below TCOOLTHRS (i.e. TSTEP >=
 * TCOOLTHRS, since TSTEP is inversely proportional to speed) StallGuard is
 * active; above it, SG_RESULT reads stale/meaningless. There is a minimum
 * speed below which StallGuard4 is not reliable either (a few 10s of RPM
 * depending on motor/current) -- verify experimentally, do not trust
 * SG_RESULT near standstill. */

/** SGTHRS 0x40: StallGuard4 threshold, 0..255. Higher = trips (DIAG active,
 *  and the stallGuard flag in DRV_STATUS is implicit) at lower load. 0 disables. */
void TMC2209_SetStallguardThreshold(TMC2209 *drv, uint8_t sgthrs);

/** TCOOLTHRS 0x14: 20-bit velocity threshold (as a TSTEP value) below which
 *  CoolStep/StallGuard are active. Convert from a target rev/s using the
 *  same VACTUAL<->Hz relation as TMC2209_MoveVelocity. */
void TMC2209_SetCoolStepThreshold(TMC2209 *drv, uint32_t tcoolthrs);

/** SG_RESULT 0x41: 10-bit load value (0..1023), low = close to stall.
 *  Only meaningful while running inside the TCOOLTHRS window. */
bool TMC2209_ReadStallGuard(TMC2209 *drv, uint16_t *sg_result);

/** TSTEP 0x12: measured time between microsteps, units of 1/fCLK (~12 MHz
 *  internal osc). Independent cross-check of actual speed vs. commanded. */
bool TMC2209_ReadTStep(TMC2209 *drv, uint32_t *tstep);

/** DRV_STATUS 0x6F, decoded. Poll periodically for logging/fault detection
 *  (overcurrent, overtemp, stall/standstill, StealthChop/SpreadCycle state,
 *  actual current scale). Returns false on a communication error. */
bool TMC2209_ReadStatus(TMC2209 *drv, TMC2209_Status *st);

/* ---- Schattenwerte, kein Bus-Zugriff (siehe Kommentar am Struct) -------- */

/** Zuletzt per TMC2209_MoveVelocity()/TMC2209_Stop() geschriebener VACTUAL-
 *  Wert. In dieser Firmware laeuft die Bewegung normalerweise ueber
 *  STEP/DIR (motor_control.c), nicht ueber VACTUAL -- dann bleibt das 0,
 *  und das ist ein korrekter, kein fehlender Messwert. */
int32_t TMC2209_GetVActual(TMC2209 *drv);

/** Zuletzt per TMC2209_SetCurrent() geschriebener IRUN-Wert, 0..31. */
uint8_t TMC2209_GetIrun(TMC2209 *drv);

/** Zuletzt per TMC2209_SetCurrent() geschriebener IHOLD-Wert, 0..31. */
uint8_t TMC2209_GetIhold(TMC2209 *drv);

#endif /* TMC2209_H */
