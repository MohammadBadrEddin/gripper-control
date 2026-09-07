/**
 * @file    debug_log.h
 * @brief   Non-blocking, timestamped CSV logging over a UART, for offline
 *          analysis (TeraTerm capture -> MATLAB).
 *
 * Design:
 *   - Timestamp source: DWT cycle counter (Cortex-M7 has it, no extra timer
 *     needed), converted to microseconds -- monotonic, wraps every ~53 min
 *     at 400 MHz, well outside a lab session, and the wrap is easy to
 *     detect offline if it ever matters (timestamp decreases).
 *   - Transport: interrupt-driven (HAL_UART_Transmit_IT + a ring buffer),
 *     so a call from the control task never blocks on the UART -- the byte
 *     budget of one CSV line (~80-120 B) at 460800 Bd is ~2 ms, which alone
 *     would blow a 250-500 Hz loop if done with a blocking HAL_UART_Transmit.
 *   - No DMA/DMAMUX config needed: works with just the UART's own global
 *     IRQ (already enabled in the .ioc for USART1 and USART3).
 *
 * Usage:
 *     DebugLog_Init(&huart3);                 // once, after MX_USART3_UART_Init()
 *     DebugLog_Printf("t_ms,theta_soll,theta_ist,e,v_cmd,sg,cs,mode\r\n");
 *     ...
 *     DebugLog_Printf("%lu,%.3f,%.3f,%.3f,%ld,%u,%u,%u\r\n",
 *                      DebugLog_TimestampMs(), soll, ist, e, v_cmd,
 *                      sg_result, cs_actual, mode);
 *
 * Wire HAL_UART_TxCpltCallback(huart) for the chosen UART instance to call
 * DebugLog_TxCpltFromISR(huart) -- see main.c USER CODE 4.
 *
 * NOT thread-safe against concurrent callers from multiple tasks -- call it
 * from a single logging/control task, or add a mutex around DebugLog_Printf
 * if that changes.
 */
#ifndef DEBUG_LOG_H
#define DEBUG_LOG_H

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>

/** Bind the logger to a UART (e.g. &huart3 for the ST-LINK VCP, &huart1 for
 *  the external-adapter alternative). Starts the DWT cycle counter. */
void DebugLog_Init(UART_HandleTypeDef *huart);

/** printf-style, formats into the ring buffer and returns immediately.
 *  Silently drops the line (no blocking, no partial writes) if the ring
 *  buffer is still full from a previous burst -- check DebugLog_Drops()
 *  if that turns out to matter for a given test run. */
void DebugLog_Printf(const char *fmt, ...);

/** Free-running microsecond timestamp since DebugLog_Init(), from DWT->CYCCNT. */
uint32_t DebugLog_TimestampUs(void);

/** Same, in milliseconds (float, sub-ms resolution preserved for CSV). */
float DebugLog_TimestampMs(void);

/** Call from HAL_UART_TxCpltCallback() when huart == the bound instance. */
void DebugLog_TxCpltFromISR(UART_HandleTypeDef *huart);

/** Cumulative count of dropped lines (ring buffer full). Log/inspect
 *  occasionally -- a nonzero, growing count means the chosen baud rate or
 *  logging rate is too high for the line volume being produced. */
uint32_t DebugLog_Drops(void);

#endif /* DEBUG_LOG_H */
