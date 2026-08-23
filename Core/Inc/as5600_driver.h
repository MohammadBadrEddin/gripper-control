/**
 * @file    as5600_driver.h
 * @brief   Low-level AS5600 register-access driver for STM32 (HAL-based).
 *
 * Design rules (see the library plan, §3):
 *  - This driver is peripheral/pin-agnostic. It never references I2C1/I2C2/etc.
 *    The caller passes the I2C_HandleTypeDef*, so wiring can be decided later
 *    in CubeMX with zero changes to this file.
 *  - Blocking transactions (config, status, one-shot reads) use HAL_I2C_Mem_*.
 *  - A DMA path is provided for the per-tick angle read in the control loop.
 *
 * The STM32 HAL header is included indirectly. Define AS5600_HAL_HEADER before
 * including this file if your project uses a family other than H7, e.g.:
 *     #define AS5600_HAL_HEADER "stm32g4xx_hal.h"
 */
#ifndef AS5600_DRIVER_H
#define AS5600_DRIVER_H

#ifndef AS5600_HAL_HEADER
#define AS5600_HAL_HEADER "stm32h7xx_hal.h"   /* NUCLEO-H753ZI default */
#endif
#include AS5600_HAL_HEADER

#include "as5600_regs.h"
#include "as5600_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Default per-transaction timeout for blocking calls (ms). */
#ifndef AS5600_I2C_TIMEOUT_MS
#define AS5600_I2C_TIMEOUT_MS   (10U)
#endif

/* Number of extra attempts on a failed blocking transaction (retry-once). */
#ifndef AS5600_I2C_RETRIES
#define AS5600_I2C_RETRIES      (1U)
#endif

/* State of an in-flight interrupt-mode angle read. */
typedef enum {
    AS5600_XFER_IDLE = 0,
    AS5600_XFER_BUSY,
    AS5600_XFER_DONE,
    AS5600_XFER_ERROR
} AS5600_XferState_t;

/*
 * FreeRTOS integration (optional).
 *
 * Define AS5600_USE_FREERTOS in your build (it is defined automatically when
 * INCLUDE_vTaskSuspend from FreeRTOS is visible via as5600_driver.h's includes,
 * but keeping it explicit is safest). When enabled, the interrupt-mode read
 * blocks the calling task on a task notification instead of spinning, and the
 * ISR wakes it. When disabled (e.g. host unit tests), a poll-based API is used.
 *
 * To enable, add -DAS5600_USE_FREERTOS to your compiler flags, or define it in
 * a project config header included before this one.
 */
#ifdef AS5600_USE_FREERTOS
#include "FreeRTOS.h"
#include "task.h"
#endif

/**
 * @brief Driver handle. One per physical AS5600 on the bus.
 *
 * All fields are owned by the driver except @p hi2c, which the caller supplies.
 */
typedef struct {
    I2C_HandleTypeDef *hi2c;      /* caller-owned; identifies the bus + pins    */
    uint32_t           timeout_ms;/* blocking transaction timeout               */
    uint8_t            addr_hal;  /* HAL-format (8-bit) address                 */

    /* interrupt-mode angle-read machinery */
    volatile AS5600_XferState_t xfer_state;
    uint8_t            rx_buf[2]; /* raw big-endian angle bytes                 */

#ifdef AS5600_USE_FREERTOS
    TaskHandle_t       waiting_task; /* task to notify from the I2C ISR         */
#endif
} AS5600_Handle_t;

/* -------------------------------------------------------------------------- */
/* Initialisation                                                             */
/* -------------------------------------------------------------------------- */

/**
 * @brief Bind the handle to an I2C peripheral and verify the device responds.
 * @param h     driver handle (zero-initialised by this call)
 * @param hi2c  a fully-configured HAL I2C handle (peripheral + pins are the
 *              caller's choice; the driver does not care which)
 * @return HAL_OK if the device ACKs, HAL_ERROR/HAL_TIMEOUT otherwise.
 */
HAL_StatusTypeDef AS5600_Init(AS5600_Handle_t *h, I2C_HandleTypeDef *hi2c);

/**
 * @brief Probe the bus for the device (HAL_I2C_IsDeviceReady wrapper).
 */
HAL_StatusTypeDef AS5600_IsPresent(AS5600_Handle_t *h);

/* -------------------------------------------------------------------------- */
/* Blocking reads                                                             */
/* -------------------------------------------------------------------------- */

HAL_StatusTypeDef AS5600_ReadRawAngle(AS5600_Handle_t *h, uint16_t *raw12);
HAL_StatusTypeDef AS5600_ReadAngle(AS5600_Handle_t *h, uint16_t *angle12);
HAL_StatusTypeDef AS5600_ReadStatus(AS5600_Handle_t *h, AS5600_Status_t *status);
HAL_StatusTypeDef AS5600_ReadAGC(AS5600_Handle_t *h, uint8_t *agc);
HAL_StatusTypeDef AS5600_ReadMagnitude(AS5600_Handle_t *h, uint16_t *mag12);
HAL_StatusTypeDef AS5600_ReadZMCO(AS5600_Handle_t *h, uint8_t *zmco);

/* -------------------------------------------------------------------------- */
/* Non-blocking (interrupt-mode) angle read — for the control-loop task        */
/* -------------------------------------------------------------------------- */
/*
 * This path uses HAL_I2C_Mem_Read_IT (no DMA required, so no .ioc change). The
 * I2C peripheral raises its event/error IRQ on completion; wire the two
 * callback hooks below into the matching HAL callbacks, matching on hi2c.
 *
 * FreeRTOS build (AS5600_USE_FREERTOS defined): use AS5600_ReadRawAngle_Wait().
 *   The calling task blocks on a task notification while the transfer runs and
 *   is woken by the ISR — no busy-waiting, scheduler stays free.
 *
 * Bare-metal / host build: use the Start + Poll pair instead.
 */

#ifdef AS5600_USE_FREERTOS
/**
 * @brief Blocking-on-notification RAW ANGLE read for use inside a FreeRTOS task.
 *        Starts an interrupt-mode read, blocks the calling task until the ISR
 *        signals completion (or @p timeout_ticks elapses), then returns.
 * @param raw12         filled with the 12-bit result on success.
 * @param timeout_ticks max ticks to wait (e.g. pdMS_TO_TICKS(2)).
 * @return HAL_OK on success, HAL_TIMEOUT if the notification never arrived,
 *         HAL_ERROR on a bus/transfer error.
 * @note   Call only from task context, never from an ISR. One reader task per
 *         handle (the handle stores a single waiting-task reference).
 */
HAL_StatusTypeDef AS5600_ReadRawAngle_Wait(AS5600_Handle_t *h, uint16_t *raw12,
                                           uint32_t timeout_ticks);
#endif

/** @brief Start an interrupt-mode RAW ANGLE read; returns immediately. */
HAL_StatusTypeDef AS5600_ReadRawAngle_IT_Start(AS5600_Handle_t *h);

/**
 * @brief Poll for interrupt-mode completion (bare-metal / test use).
 * @param raw12  filled with the 12-bit result when the function returns true.
 * @return true once the transfer completed successfully (state consumed).
 */
bool AS5600_ReadRawAngle_IT_Poll(AS5600_Handle_t *h, uint16_t *raw12);

/** @brief True if the last interrupt-mode transfer errored (state consumed). */
bool AS5600_ReadRawAngle_IT_Failed(AS5600_Handle_t *h);

/*
 * ISR hooks. Call these from the HAL callbacks, matching on hi2c, e.g.:
 *
 *   void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
 *       AS5600_I2C_RxCpltCallback(&g_enc, hi2c);
 *   }
 *   void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
 *       AS5600_I2C_ErrorCallback(&g_enc, hi2c);
 *   }
 *
 * They are ISR-safe and, under FreeRTOS, perform the FromISR notify + yield.
 */
void AS5600_I2C_RxCpltCallback(AS5600_Handle_t *h, I2C_HandleTypeDef *hi2c);
void AS5600_I2C_ErrorCallback(AS5600_Handle_t *h, I2C_HandleTypeDef *hi2c);

/* -------------------------------------------------------------------------- */
/* Configuration (call once at startup, never inside the loop)                */
/* -------------------------------------------------------------------------- */

HAL_StatusTypeDef AS5600_ReadConf(AS5600_Handle_t *h, uint16_t *conf);
HAL_StatusTypeDef AS5600_WriteConf(AS5600_Handle_t *h, uint16_t conf);

HAL_StatusTypeDef AS5600_SetPowerMode(AS5600_Handle_t *h, AS5600_PM_t pm);
HAL_StatusTypeDef AS5600_SetHysteresis(AS5600_Handle_t *h, AS5600_HYST_t hyst);
HAL_StatusTypeDef AS5600_SetOutputStage(AS5600_Handle_t *h, AS5600_OUTS_t outs);
HAL_StatusTypeDef AS5600_SetPwmFrequency(AS5600_Handle_t *h, AS5600_PWMF_t pwmf);
HAL_StatusTypeDef AS5600_SetSlowFilter(AS5600_Handle_t *h, AS5600_SF_t sf);
HAL_StatusTypeDef AS5600_SetFastFilterThreshold(AS5600_Handle_t *h, AS5600_FTH_t fth);
HAL_StatusTypeDef AS5600_SetWatchdog(AS5600_Handle_t *h, AS5600_WD_t wd);

/**
 * @brief Convenience: sensible defaults for active closed-loop control.
 *        PM=NOM, HYST=1LSB, fast filter enabled (FTH=10LSB), SF=16x, WD=OFF,
 *        output stage left at analog-full (harmless when reading over I2C).
 */
HAL_StatusTypeDef AS5600_ApplyControlLoopDefaults(AS5600_Handle_t *h);

/* -------------------------------------------------------------------------- */
/* Angular-range programming (RAM only — burning is separate below)           */
/* -------------------------------------------------------------------------- */

HAL_StatusTypeDef AS5600_SetZPos(AS5600_Handle_t *h, uint16_t zpos12);
HAL_StatusTypeDef AS5600_SetMPos(AS5600_Handle_t *h, uint16_t mpos12);
HAL_StatusTypeDef AS5600_SetMaxAngle(AS5600_Handle_t *h, uint16_t mang12);

/* -------------------------------------------------------------------------- */
/* OTP burn commands — IRREVERSIBLE. For a one-time calibration tool only.     */
/* -------------------------------------------------------------------------- */

/**
 * @brief Permanently burn ZPOS + MPOS. Guarded: requires MD==1 and ZMCO<3.
 * @warning One-way operation. Never call this from normal firmware paths.
 * @param confirm  must be true; a deliberate speed bump against accidents.
 */
HAL_StatusTypeDef AS5600_BurnAngle(AS5600_Handle_t *h, bool confirm);

/**
 * @brief Permanently burn MANG + CONF. Guarded: requires MD==1; MANG is only
 *        writable while ZMCO==0. Can be performed only once in the device life.
 * @warning One-way operation. Never call this from normal firmware paths.
 * @param confirm  must be true.
 */
HAL_StatusTypeDef AS5600_BurnSetting(AS5600_Handle_t *h, bool confirm);

#ifdef __cplusplus
}
#endif
#endif /* AS5600_DRIVER_H */
