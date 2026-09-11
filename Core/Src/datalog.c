/*
 * datalog.c -- siehe datalog.h fuer das Konzept.
 */
#include "datalog.h"
#include "main.h"            /* Error_Handler() */
#include "motor_control.h"   /* MOTOR_MICROSTEPS fuer den Kopfblock */
#include <stdio.h>
#include <string.h>
#include <math.h>

/* ---- Ringpuffer ----------------------------------------------------------
 * Global (nicht static), damit Adresse/Zaehler fuer den ST-Link-Memory-Dump
 * leicht auffindbar sind (Map-File / Watch-Expression). */
#define LOG_CAPACITY   5000u                 /* 5000 * 36 B = 180000 B (~176 KB) */

LogRecord         g_log[LOG_CAPACITY];       /* Rohpuffer                        */
volatile uint32_t g_log_count = 0;           /* gefuellte Eintraege (max CAPACITY)*/
volatile uint32_t g_log_head  = 0;           /* naechster Schreibindex           */
volatile uint32_t g_log_total = 0;           /* ALLE Abtastungen seit Reset (ungedeckelt) */

static volatile bool s_active   = false;     /* true: es wird aufgezeichnet      */
static UART_HandleTypeDef s_huart3;          /* USART3 = ST-Link VCP             */
static uint32_t s_start_us   = 0;            /* Timer-Startwert (us) seit Reset  */
static uint32_t s_start_tick = 0;            /* HAL-Tick (1 kHz) bei Reset -- Referenz */

/* ---- Zeitbasis: freilaufender TIM2 @ 1 MHz (32-bit) ----------------------
 * Robuster als der DWT-Zyklenzaehler (der bei Debug-Halts einfriert). TIM2 ist
 * 32-bit auf dem F7 -> 1 count = 1 us, Ueberlauf erst nach ~71 min. */
static void timebase_init(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();
    uint32_t timclk = 2u * HAL_RCC_GetPCLK1Freq();   /* APB1-Timer-Takt (Div!=1 -> x2) */
    TIM2->PSC = (timclk / 1000000u) - 1u;            /* -> 1 MHz                        */
    TIM2->ARR = 0xFFFFFFFFu;                         /* voller 32-bit-Umfang           */
    TIM2->EGR = TIM_EGR_UG;                          /* PSC uebernehmen                */
    TIM2->CR1 = TIM_CR1_CEN;                         /* starten                        */
}

uint32_t Datalog_TimestampUs(void)
{
    return TIM2->CNT - s_start_us;   /* us seit Reset; uint32-Wrap ist unkritisch */
}

/* ---- USART3 (PD8 TX / PD9 RX, AF7) ---------------------------------------
 * Bewusst selbst konfiguriert (GPIO + Clock + HAL_UART_Init), damit keine
 * generierte Datei (hal_msp.c/.ioc) angefasst werden muss und ein CubeMX-
 * Regenerieren das hier nicht ueberschreibt. */
static void usart3_init(void)
{
    __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Pin       = GPIO_PIN_8 | GPIO_PIN_9;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOD, &g);

    __HAL_RCC_USART3_CLK_ENABLE();

    s_huart3.Instance                    = USART3;
    s_huart3.Init.BaudRate               = 500000;      /* VCP-Baud (Dump-Transport) */
    s_huart3.Init.WordLength             = UART_WORDLENGTH_8B;
    s_huart3.Init.StopBits               = UART_STOPBITS_1;
    s_huart3.Init.Parity                 = UART_PARITY_NONE;
    s_huart3.Init.Mode                   = UART_MODE_TX_RX;
    s_huart3.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
    s_huart3.Init.OverSampling           = UART_OVERSAMPLING_16;
    s_huart3.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
    s_huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&s_huart3) != HAL_OK) {
        Error_Handler();
    }
}

/* ---- USER-Button PC13 ----------------------------------------------------- */
static void button_init(void)
{
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    g.Pin   = GPIO_PIN_13;
    g.Mode  = GPIO_MODE_INPUT;
    g.Pull  = GPIO_NOPULL;         /* Nucleo B1: extern beschaltet, gedrueckt = HIGH */
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &g);
}

bool Datalog_ButtonPressed(void)
{
    /* Steigende Flanke; Aufruf im ~20-ms-Takt wirkt als Entprellung. */
    static uint8_t prev = 0;
    uint8_t now = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET) ? 1u : 0u;
    bool edge = (now && !prev);
    prev = now;
    return edge;
}

/* ---- API ------------------------------------------------------------------ */
void Datalog_Init(void)
{
    timebase_init();
    usart3_init();
    button_init();
    Datalog_Reset();
}

void Datalog_Reset(void)
{
    g_log_count = 0;
    g_log_head  = 0;
    g_log_total = 0;
    s_start_us   = TIM2->CNT;
    s_start_tick = HAL_GetTick();   /* reale Startzeit, TIM8-Tick 1 kHz */
    s_active     = true;
}

bool Datalog_IsActive(void)
{
    return s_active;
}

void Datalog_Sample(const LogRecord *rec)
{
    if (!s_active) return;
    g_log_total++;                 /* ungedeckelt -> echte Abtastzahl */
    g_log[g_log_head] = *rec;
    g_log_head = (g_log_head + 1u) % LOG_CAPACITY;
    if (g_log_count < LOG_CAPACITY) {
        g_log_count++;                 /* Puffer fuellt sich                    */
    }
    /* sonst: Ring laeuft ueber, g_log_head zeigt jetzt auf den aeltesten Eintrag */
}

/* Float ohne printf-%f: fixe 3 Nachkommastellen, z.B. "-0.026". */
static void put_f(char *dst, float v)
{
    int32_t m = (int32_t)lrintf(v * 1000.0f);
    const char *sign = "";
    if (m < 0) { sign = "-"; m = -m; }
    snprintf(dst, 16, "%s%ld.%03ld", sign, (long)(m / 1000), (long)(m % 1000));
}

static void tx_str(const char *s)
{
    HAL_UART_Transmit(&s_huart3, (uint8_t *)s, (uint16_t)strlen(s), HAL_MAX_DELAY);
}

void Datalog_Dump(void)
{
    s_active = false;   /* zuerst stoppen: keine neuen Samples waehrend des Dumps */

    uint32_t n        = g_log_count;
    uint32_t start    = (g_log_count == LOG_CAPACITY) ? g_log_head : 0u;  /* aeltester */
    uint32_t total    = g_log_total;                                      /* ALLE Abtastungen */
    uint32_t real_ms  = HAL_GetTick() - s_start_tick;                     /* reale Laufzeit (1 kHz) */
    uint32_t per_real = (total > 0u) ? (real_ms * 1000u) / total : 0u;    /* ECHTE Abtastperiode */

    /* Was die DWT-Zeitbasis fuer denselben Puffer glaubt (zum Vergleich): */
    uint32_t t_old    = g_log[start].t_us;
    uint32_t t_new    = g_log[(start + (n ? n - 1u : 0u)) % LOG_CAPACITY].t_us;
    uint32_t per_dwt  = (n > 1u) ? (t_new - t_old) / (n - 1u) : 0u;

    char hdr[512];

    tx_str("#BEGIN\r\n");
    snprintf(hdr, sizeof hdr,
             "# fw=gripper-control mainv1, MCU=STM32F767ZI\r\n"
             "# IRUN=27, IHOLD=16, microsteps=%u, T_regler_s=0.002, fCLK_MHz=12\r\n"
             "# VREF_V=0.6, VM_V=12.0, R_SENSE_ohm=0.11  (R_SENSE=Annahme, am Modul verifizieren)\r\n"
             "# buffer=%lu, total_samples=%lu, real_elapsed_ms=%lu (HAL-Tick 1kHz)\r\n"
             "# ECHTE Periode = %lu us/Sample | Zeitbasis(TIM2) = %lu us/Sample\r\n"
             "# TMC-DIAG: driver_ready=%d, last_read_bytes=%u/8, USART2_ISR=0x%08lX\r\n",
             (unsigned)MOTOR_MICROSTEPS, (unsigned long)n, (unsigned long)total,
             (unsigned long)real_ms, (unsigned long)per_real, (unsigned long)per_dwt,
             (MotorControl_GetDriver() != 0) ? 1 : 0,
             (unsigned)g_tmc_rx_got, (unsigned long)g_tmc_rx_isr);
    tx_str(hdr);
    tx_str("t_us,x_soll_mm,x_ist_mm,e_mm,v_cmd_mms,vactual,enc_raw,step_cnt,sg_result,state,i_run_akt\r\n");

    for (uint32_t i = 0; i < n; i++) {
        const LogRecord *r = &g_log[(start + i) % LOG_CAPACITY];
        char b1[16], b2[16], b3[16], b4[16], line[192];
        put_f(b1, r->x_soll_mm);
        put_f(b2, r->x_ist_mm);
        put_f(b3, r->e_mm);
        put_f(b4, r->v_cmd_mms);
        int len = snprintf(line, sizeof line,
                           "%lu,%s,%s,%s,%s,%ld,%u,%ld,%u,%u,%u\r\n",
                           (unsigned long)r->t_us, b1, b2, b3, b4,
                           (long)r->vactual, (unsigned)r->enc_raw,
                           (long)r->step_cnt, (unsigned)r->sg_result,
                           (unsigned)r->state, (unsigned)r->i_run_akt);
        if (len > 0) {
            HAL_UART_Transmit(&s_huart3, (uint8_t *)line, (uint16_t)len, HAL_MAX_DELAY);
        }
    }

    tx_str("#END\r\n");
}
