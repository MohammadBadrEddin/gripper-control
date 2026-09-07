#include "debug_log.h"
#include <string.h>
#include <stdio.h>

#define LOG_BUF_SIZE   2048u    /* ring buffer, bytes. Bump if lines/drops climb. */
#define LOG_LINE_MAX    200u    /* max formatted length of a single Printf() call */

static UART_HandleTypeDef *s_huart;
static uint8_t   s_buf[LOG_BUF_SIZE];
static volatile uint16_t s_head = 0;   /* next free slot to write into */
static volatile uint16_t s_tail = 0;   /* next byte to transmit */
static volatile bool     s_txBusy = false;
static volatile uint32_t s_drops = 0;

static uint32_t s_us_per_tick_shift; /* unused placeholder kept for clarity */

void DebugLog_Init(UART_HandleTypeDef *huart)
{
    s_huart = huart;
    s_head = s_tail = 0;
    s_txBusy = false;
    s_drops = 0;

    /* Enable the Cortex-M7 cycle counter (DWT) for a free-running,
     * high-resolution timestamp -- no extra timer peripheral needed. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t DebugLog_TimestampUs(void)
{
    /* SystemCoreClock is in Hz; division below is exact enough for logging
     * (e.g. 400 MHz -> 400 cycles/us). Wraps ~ every 2^32 cycles (~10.7 s at
     * 400 MHz on the raw CYCCNT; the returned *microsecond* value wraps at
     * 2^32 us =~ 71.6 min, whichever is hit first still just wraps -- treat
     * a decreasing timestamp in the log as "wrapped", not an error. */
    return DWT->CYCCNT / (SystemCoreClock / 1000000u);
}

float DebugLog_TimestampMs(void)
{
    return (float)DebugLog_TimestampUs() / 1000.0f;
}

uint32_t DebugLog_Drops(void)
{
    return s_drops;
}

/* Kick a transmission of the next contiguous run of bytes in the ring
 * buffer. Must be called with tx not already busy. */
static void start_tx(void)
{
    if (s_head == s_tail) {
        return;   /* nothing to send */
    }
    uint16_t len;
    if (s_head > s_tail) {
        len = s_head - s_tail;
    } else {
        len = LOG_BUF_SIZE - s_tail;   /* up to the wrap point only */
    }
    s_txBusy = true;
    HAL_UART_Transmit_IT(s_huart, &s_buf[s_tail], len);
}

void DebugLog_Printf(const char *fmt, ...)
{
    char line[LOG_LINE_MAX];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    if (n <= 0) {
        return;
    }
    if ((uint32_t)n >= sizeof(line)) {
        n = sizeof(line) - 1;   /* truncated; still send what fits */
    }

    __disable_irq();
    uint16_t free_space = (uint16_t)((s_tail - s_head - 1) & (LOG_BUF_SIZE - 1));
    /* NOTE: the mask trick above requires LOG_BUF_SIZE to be a power of two. */
    if (free_space < (uint16_t)n) {
        s_drops++;
        __enable_irq();
        return;
    }
    for (int i = 0; i < n; i++) {
        s_buf[s_head] = (uint8_t)line[i];
        s_head = (uint16_t)((s_head + 1) & (LOG_BUF_SIZE - 1));
    }
    bool wasIdle = !s_txBusy;
    __enable_irq();

    if (wasIdle) {
        start_tx();
    }
}

void DebugLog_TxCpltFromISR(UART_HandleTypeDef *huart)
{
    if (huart->Instance != s_huart->Instance) {
        return;
    }
    /* Advance tail by whatever the just-completed transfer covered. HAL
     * already knows huart->TxXferSize for the chunk just sent. */
    s_tail = (uint16_t)((s_tail + huart->TxXferSize) & (LOG_BUF_SIZE - 1));
    s_txBusy = false;
    start_tx();   /* re-arms only if more data is queued */
}
