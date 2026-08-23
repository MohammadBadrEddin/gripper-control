/**
 * @file    tmc2209_driver.h
 * @brief   TMC2209 single-wire UART driver + init sequence.
 *
 * Layering (same as the AS5600 library):
 *   transport   : half-duplex UART datagrams, CRC8, TX->RX turnaround
 *   register    : read / write / verified write (via IFCNT)
 *   config      : TMC2209_Init() and the typed setters below
 *
 * The driver is peripheral agnostic: pass the UART handle in, no USART2 or
 * GPIO references live inside the driver. Board specific things (TMC_EN pin,
 * STEP timer) stay in the application.
 *
 * Build options:
 *   TMC2209_HAL_HEADER     - override the HAL header, defaults to STM32H7
 *   TMC2209_USE_FREERTOS   - use vTaskDelay + interrupt-mode transfers with
 *                            task notifications instead of blocking HAL calls
 */

#ifndef TMC2209_DRIVER_H
#define TMC2209_DRIVER_H

#ifndef TMC2209_HAL_HEADER
#define TMC2209_HAL_HEADER "stm32h7xx_hal.h"
#endif

#include TMC2209_HAL_HEADER

#include "tmc2209_regs.h"
#include "tmc2209_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------- */
/* Datagram / CRC primitives (pure functions - unit tested on host)          */
/* ------------------------------------------------------------------------- */

/** CRC8-ATM over @p len bytes, poly 0x07, init 0, LSB first (datasheet 4.2). */
uint8_t TMC2209_CRC8(const uint8_t *data, uint32_t len);

/** Build an 8-byte write datagram into @p out (must hold 8 bytes). */
void TMC2209_BuildWriteDatagram(uint8_t *out, uint8_t node_addr,
                                uint8_t reg, uint32_t value);

/** Build a 4-byte read request into @p out (must hold 4 bytes). */
void TMC2209_BuildReadRequest(uint8_t *out, uint8_t node_addr, uint8_t reg);

/**
 * @brief Find and decode an 8-byte read reply inside a raw RX buffer.
 *
 * Tolerates leading garbage, in particular the driver's own transmitted
 * request bytes echoed back on the single-wire line.
 *
 * @return TMC2209_OK, TMC2209_ERR_FRAME (no header found) or TMC2209_ERR_CRC.
 */
TMC2209_Status TMC2209_ParseReadReply(const uint8_t *buf, uint32_t len,
                                      uint8_t reg, uint32_t *value_out);

/* ------------------------------------------------------------------------- */
/* Register access                                                           */
/* ------------------------------------------------------------------------- */

TMC2209_Status TMC2209_WriteRegister(TMC2209_Handle *h, uint8_t reg, uint32_t value);
TMC2209_Status TMC2209_ReadRegister (TMC2209_Handle *h, uint8_t reg, uint32_t *value);

/** Write, then confirm IFCNT incremented by one (datasheet 4.1.1). */
TMC2209_Status TMC2209_WriteRegisterVerified(TMC2209_Handle *h, uint8_t reg,
                                             uint32_t value);

/* ------------------------------------------------------------------------- */
/* Configuration / init                                                      */
/* ------------------------------------------------------------------------- */

/** Fill @p cfg with the project defaults (see tmc2209_driver.c). */
void TMC2209_GetDefaultConfig(TMC2209_Config *cfg);

/**
 * @brief Probe the driver over UART and apply the full init sequence.
 *
 * Blocking - intended to run once at start-up, before the control loop.
 * The power stage is left disabled (CHOPCONF.TOFF is written last, and
 * cfg->set_enable(false) is called first if provided). Call TMC2209_Enable()
 * when the application is ready to move the motor.
 */
TMC2209_Status TMC2209_Init(TMC2209_Handle *h, UART_HandleTypeDef *huart,
                            const TMC2209_Config *cfg);

/** Re-apply the whole register set (e.g. after GSTAT.reset was seen). */
TMC2209_Status TMC2209_ApplyConfig(TMC2209_Handle *h);

/* ------------------------------------------------------------------------- */
/* Typed setters                                                             */
/* ------------------------------------------------------------------------- */

/** Convert a target RMS current to CS, picking vsense automatically. */
TMC2209_Status TMC2209_CurrentToCS(uint16_t irms_ma, uint16_t rsense_mohm,
                                   uint8_t *cs_out, bool *vsense_out);

/** RMS current in mA for a given CS / vsense / Rsense (inverse of the above). */
uint16_t TMC2209_CSToCurrent(uint8_t cs, uint16_t rsense_mohm, bool vsense);

TMC2209_Status TMC2209_SetCurrent(TMC2209_Handle *h, uint16_t irun_ma,
                                  uint8_t ihold_percent);
TMC2209_Status TMC2209_SetMicrosteps(TMC2209_Handle *h, TMC2209_Microsteps mres);
TMC2209_Status TMC2209_SetChopperMode(TMC2209_Handle *h, TMC2209_ChopperMode mode);
TMC2209_Status TMC2209_SetShaftDirection(TMC2209_Handle *h, bool inverted);
TMC2209_Status TMC2209_SetStallGuard(TMC2209_Handle *h, uint8_t sgthrs,
                                     uint32_t tcoolthrs);

/** Enable / disable the power stage over UART (CHOPCONF.TOFF). Also drives
 *  the TMC_EN pin if cfg.set_enable was provided. */
TMC2209_Status TMC2209_Enable(TMC2209_Handle *h, bool enable);

/* ------------------------------------------------------------------------- */
/* Motion / status                                                           */
/* ------------------------------------------------------------------------- */

/** UART velocity mode: microsteps/s = VACTUAL * (fCLK / 2^24), datasheet 14. */
TMC2209_Status TMC2209_SetVelocity(TMC2209_Handle *h, int32_t vactual);

TMC2209_Status TMC2209_GetStatus(TMC2209_Handle *h, TMC2209_DrvStatus *st);
TMC2209_Status TMC2209_ClearGSTAT(TMC2209_Handle *h);
TMC2209_Status TMC2209_GetStallGuardResult(TMC2209_Handle *h, uint16_t *sg);
TMC2209_Status TMC2209_GetMicrostepCounter(TMC2209_Handle *h, uint16_t *mscnt);
TMC2209_Status TMC2209_GetTStep(TMC2209_Handle *h, uint32_t *tstep);

#ifdef TMC2209_USE_FREERTOS
/* ---- interrupt-mode transfers for the control loop ----------------------- */
TMC2209_Status TMC2209_WriteRegisterIT(TMC2209_Handle *h, uint8_t reg, uint32_t value);
TMC2209_Status TMC2209_ReadRegisterIT (TMC2209_Handle *h, uint8_t reg, uint32_t *value);

/* Call these from the matching HAL callbacks in the application. */
void TMC2209_UART_TxCpltCallback(TMC2209_Handle *h);
void TMC2209_UART_RxEventCallback(TMC2209_Handle *h, uint16_t size);
void TMC2209_UART_ErrorCallback(TMC2209_Handle *h);
#endif

#ifdef __cplusplus
}
#endif

#endif /* TMC2209_DRIVER_H */
