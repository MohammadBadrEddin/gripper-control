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

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* Registers used here (datasheet sec. 5) */
#define TMC_GCONF       0x00
#define TMC_GSTAT       0x01
#define TMC_IFCNT       0x02
#define TMC_IHOLD_IRUN  0x10
#define TMC_TPOWERDOWN  0x11
#define TMC_VACTUAL     0x22
#define TMC_CHOPCONF    0x6C
#define TMC_DRV_STATUS  0x6F

typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t addr;        /* node address 0..3, set by the MS1/MS2 pins */
} TMC2209;

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

#endif /* TMC2209_H */
