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
#define TMC_TSTEP       0x12   /* gemessene Zeit zwischen zwei µSteps (read) */
#define TMC_TCOOLTHRS   0x14   /* untere Geschwindigkeitsschwelle fuer StallGuard/CoolStep */
#define TMC_VACTUAL     0x22
#define TMC_SGTHRS      0x40   /* StallGuard4-Schwelle */
#define TMC_SG_RESULT   0x41   /* StallGuard4-Lastwert 0..1023 (read) */
#define TMC_CHOPCONF    0x6C
#define TMC_DRV_STATUS  0x6F

typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t addr;
    SemaphoreHandle_t mutex;   // NEW: protects any UART-Transaction, included for use of mutex (02.09.2026)
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

/* Diagnose: Zustand des LETZTEN Register-Reads (fuer Kopfblock/Debugger).
 * g_tmc_rx_got = wieviele der 8 Antwort-Bytes ankamen (0 = Funkstille),
 * g_tmc_rx_isr = USART2->ISR danach (Bit1=FE, Bit3=ORE, Bit5=RXNE ...). */
extern volatile uint8_t  g_tmc_rx_got;
extern volatile uint32_t g_tmc_rx_isr;

/** StallGuard4-Schwelle SGTHRS (0..255). Hoeher = loest bei geringerer Last aus. */
void TMC2209_SetStallguardThreshold(TMC2209 *drv, uint8_t sgthrs);

/** StallGuard/CoolStep-Fenster: SG_RESULT ist nur gueltig, solange TSTEP >= TCOOLTHRS
 *  (d.h. oberhalb einer Mindestgeschwindigkeit). 20-bit-Wert. */
void TMC2209_SetCoolStepThreshold(TMC2209 *drv, uint32_t tcoolthrs);

#endif /* TMC2209_H */
