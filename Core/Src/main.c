/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"

#include "tmc2209.h"
#include "as5600.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Set to 1 to re-enable the stepper and encoder test tasks. Keep at 0 while
 * debugging the TMC2209 UART link: the stepper task busy-waits at a higher
 * priority and will preempt the probe mid-datagram. */
#define ENABLE_MOTION_TASKS   0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* ---------------------------------------------------------------------------
 * Encoder telemetry. Declared volatile and at file scope so they can be watched
 * live in the debugger (Expressions view). There is no serial console here,
 * since USART2 is dedicated to the TMC2209 in half-duplex mode.
 *
 * Add to Expressions:  g_enc_present, g_enc_magnet, g_enc_raw,
 *                      g_enc_deg, g_enc_errors, g_enc_samples
 * ------------------------------------------------------------------------- */
volatile uint8_t  g_enc_present = 0;    /* 1 = AS5600 ACKed on the I2C bus    */
volatile uint8_t  g_enc_magnet  = 0;    /* 1 = magnet detected (MD bit)       */
volatile uint16_t g_enc_raw     = 0;    /* last raw count, 0..4095            */
volatile float    g_enc_deg     = 0.0f; /* last angle in degrees, 0..360      */
volatile uint32_t g_enc_errors  = 0;    /* cumulative failed reads            */
volatile uint32_t g_enc_samples = 0;    /* cumulative successful reads        */

/* ---------------------------------------------------------------------------
 * TMC2209 UART probe telemetry.
 *
 * The link is not answering, so this captures the RAW bytes on the wire rather
 * than relying on the library's parser. Watch these in the debugger:
 *
 *   g_uart_tx[a][0..3]   the 4-byte read request actually transmitted
 *   g_uart_rx[a][0..15]  everything received afterwards, unparsed
 *   g_uart_nrx[a]        how many bytes actually arrived (0..12)
 *   g_uart_echo_ok[a]    1 = the first 4 rx bytes match the request we sent
 *   g_uart_crc_ok[a]     1 = the 8-byte reply passed CRC
 *   g_uart_val[a]        decoded 32-bit register value when crc_ok
 *   g_uart_flags[a]      sticky error flags: b0=ORE b1=FE b2=NE
 *   g_uart_isr[a]        USART2->ISR snapshot after the exchange
 *   g_uart_pass          increments each full sweep of all 4 addresses
 *
 * Expected transmitted bytes (IOIN read, reg 0x06), precomputed CRC8-ATM:
 *   addr 0 -> 05 00 06 6F
 *   addr 1 -> 05 01 06 D9
 *   addr 2 -> 05 02 06 34
 *   addr 3 -> 05 03 06 82
 * If g_uart_tx does not match these, the CRC or framing is wrong.
 *
 * How to read the results:
 *   echo_ok = 0, nrx = 0     nothing on the wire: pin mapping, AF, or wiring
 *   echo_ok = 1, nrx = 4     TX works and echoes, driver never replies:
 *                            node address, PDN_UART routing, or motor supply
 *   echo_ok = 1, nrx = 12,
 *   crc_ok = 1               link is good; g_uart_val holds IOIN
 * ------------------------------------------------------------------------- */
volatile uint8_t  g_uart_tx[4][4];
volatile uint8_t  g_uart_rx[4][16];
volatile uint8_t  g_uart_txn[4];        /* 0 = all 4 bytes went out           */
volatile uint8_t  g_uart_rxn[4];        /* 0 = full 12 bytes received         */
volatile uint8_t  g_uart_nonzero[4];    /* non-zero byte count in g_uart_rx   */
volatile uint32_t g_uart_pass = 0;      /* sweeps completed                   */
volatile uint32_t g_uart_isr[4];        /* USART2->ISR after the exchange     */
volatile uint8_t  g_uart_flags[4];      /* sticky: b0=ORE b1=FE b2=NE         */
volatile uint8_t  g_uart_echo_ok[4];    /* 1 = first 4 rx bytes match request */
volatile uint8_t  g_uart_nrx[4];        /* how many bytes actually arrived    */
volatile uint8_t  g_uart_crc_ok[4];     /* 1 = 8-byte reply passed CRC        */
volatile uint32_t g_uart_val[4];        /* decoded 32-bit register value      */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);

/* USER CODE BEGIN PFP */
#if ENABLE_MOTION_TASKS
void StartStepperTestTask(void *argument);
void EncoderTestTask(void *argument);
#endif
void UartProbeTask(void *argument);
void HeartBeatTask(void *argument);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* Keep the TMC2209 output stage DISABLED until a task configures it.
   * EN is active-low, so HIGH = outputs off. MX_GPIO_Init() drives it LOW,
   * which would energise the motor before any software setup runs. */
  HAL_GPIO_WritePin(TMC_EN_GPIO_Port, TMC_EN_Pin, GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* USER CODE BEGIN RTOS_THREADS */
#if ENABLE_MOTION_TASKS
  xTaskCreate(StartStepperTestTask, "StepperTest", 512, NULL, 3, NULL);
  xTaskCreate(EncoderTestTask,      "Encoder",     512, NULL, 2, NULL);
#endif
  xTaskCreate(UartProbeTask,        "UartProbe",   512, NULL, 2, NULL);
  xTaskCreate(HeartBeatTask,        "vHB",         128, NULL, 1, NULL);
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  vTaskStartScheduler();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    __WFI();
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x20404768;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_HalfDuplex_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, TMC_STEP_Pin|TMC_DIR_Pin|TMC_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_GREEN_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GREEN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TMC_STEP_Pin */
  GPIO_InitStruct.Pin = TMC_STEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(TMC_STEP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TMC_DIR_Pin TMC_EN_Pin */
  GPIO_InitStruct.Pin = TMC_DIR_Pin|TMC_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : TMC_DIAG_Pin */
  GPIO_InitStruct.Pin = TMC_DIAG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TMC_DIAG_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

#if ENABLE_MOTION_TASKS

/**
  * @brief  Open-loop stepper test using STEP/DIR (standalone mode).
  *
  * UART/VACTUAL control is not used here: the TMC2209 UART link is still
  * unresolved, while STEP/DIR is confirmed working. In this mode microstepping
  * comes from the MS1/MS2 straps and motor current from the VREF pot -- there
  * is no software control of either.
  *
  * Behaviour: 400 steps one way, pause 0.5 s, reverse, repeat.
  *
  * NOTE: the busy-wait below runs at priority 3 for roughly 90 ms without
  * yielding. It will preempt UartProbeTask mid-datagram. Do not run this task
  * and the UART probe at the same time.
  */
void StartStepperTestTask(void *argument)
{
    (void)argument;

    HAL_GPIO_WritePin(TMC_DIR_GPIO_Port, TMC_DIR_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TMC_EN_GPIO_Port,  TMC_EN_Pin,  GPIO_PIN_RESET); /* enable */
    vTaskDelay(pdMS_TO_TICKS(10));

    for (;;) {
        /* Busy-wait pulse timing. The loop count is empirical -- raise it to
         * slow the motor down, lower it to speed up. */
        for (int i = 0; i < 400; i++) {
            HAL_GPIO_WritePin(TMC_STEP_GPIO_Port, TMC_STEP_Pin, GPIO_PIN_SET);
            for (volatile int d = 0; d < 5000; d++) { __NOP(); }
            HAL_GPIO_WritePin(TMC_STEP_GPIO_Port, TMC_STEP_Pin, GPIO_PIN_RESET);
            for (volatile int d = 0; d < 5000; d++) { __NOP(); }
        }

        vTaskDelay(pdMS_TO_TICKS(500));

        HAL_GPIO_TogglePin(TMC_DIR_GPIO_Port, TMC_DIR_Pin);   /* reverse */
    }
}

/**
  * @brief  AS5600 encoder read test.
  *
  * Publishes results into the g_enc_* globals for inspection in the debugger.
  * Runs at 10 Hz -- plenty for watching values change by hand, and keeps the
  * blocking I2C reads well clear of the stepper task's timing.
  *
  * What to expect once wired correctly:
  *   g_enc_present = 1        device ACKs at address 0x36
  *   g_enc_magnet  = 1        magnet is in range
  *   g_enc_raw     0..4095    changes as the magnet turns
  *   g_enc_deg     0..360     the same value in degrees
  *   g_enc_errors  stays 0    any climb means I2C reads are failing
  */
void EncoderTestTask(void *argument)
{
    (void)argument;
    static AS5600 enc;

    /* Let the AS5600 power up before the first transaction. */
    vTaskDelay(pdMS_TO_TICKS(100));

    /* Probe the bus. A failure here is almost always pull-ups, wiring, or the
     * module not being powered from 3.3 V. Retry rather than give up, so the
     * wiring can be fixed and seen to come alive without a reflash. */
    for (;;) {
        g_enc_present = AS5600_Init(&enc, &hi2c1) ? 1u : 0u;
        if (g_enc_present) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    for (;;) {
        g_enc_magnet = AS5600_MagnetOK(&enc) ? 1u : 0u;

        uint16_t raw = AS5600_ReadRaw(&enc);
        if (raw == 0xFFFF) {
            g_enc_errors++;
        } else {
            g_enc_raw = raw;
            g_enc_deg = raw * (360.0f / 4096.0f);
            g_enc_samples++;
        }

        vTaskDelay(pdMS_TO_TICKS(100));   /* 10 Hz */
    }
}

#endif /* ENABLE_MOTION_TASKS */

/**
  * @brief  CRC8-ATM used by the TMC2209 UART datagrams (datasheet sec. 4.2).
  *         Duplicated locally so the probe does not depend on the library.
  */
static uint8_t probe_crc(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        uint8_t byte = data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if ((crc >> 7) ^ (byte & 0x01)) {
                crc = (uint8_t)((crc << 1) ^ 0x07);
            } else {
                crc = (uint8_t)(crc << 1);
            }
            byte >>= 1;
        }
    }
    return crc;
}

#define PROBE_RX_MAX   12u   /* 4 echo + 8 reply */

/**
  * @brief  Collect one byte, spinning on RXNE.
  *         Records error flags rather than silently swallowing them.
  * @retval 1 on success, 0 on timeout.
  */
static uint8_t probe_getc(uint8_t *out, uint32_t timeout_ms, uint8_t *flags)
{
    TickType_t t0 = xTaskGetTickCount();

    for (;;) {
        uint32_t isr = USART2->ISR;

        if (isr & USART_ISR_ORE) { *flags |= 0x01; __HAL_UART_CLEAR_OREFLAG(&huart2); }
        if (isr & USART_ISR_FE)  { *flags |= 0x02; __HAL_UART_CLEAR_FEFLAG(&huart2);  }
        if (isr & USART_ISR_NE)  { *flags |= 0x04; __HAL_UART_CLEAR_NEFLAG(&huart2);  }

        if (isr & USART_ISR_RXNE) {
            *out = (uint8_t)(USART2->RDR & 0xFFu);
            return 1u;
        }

        if ((xTaskGetTickCount() - t0) > pdMS_TO_TICKS(timeout_ms)) {
            return 0u;
        }
    }
}

/**
  * @brief  Send one byte and immediately recover its echo.
  *
  * In half-duplex the echo lands about one byte-time later. Reading it here
  * keeps RDR empty and prevents the overrun that would otherwise destroy the
  * driver's reply.
  */
static uint8_t probe_putc(uint8_t b, uint8_t *echo, uint8_t *flags)
{
    TickType_t t0 = xTaskGetTickCount();

    while (!(USART2->ISR & USART_ISR_TXE)) {
        if ((xTaskGetTickCount() - t0) > pdMS_TO_TICKS(10)) return 0u;
    }
    USART2->TDR = b;

    return probe_getc(echo, 10u, flags);
}

/**
  * @brief  Raw TMC2209 UART probe -- bypasses the library parser entirely.
  *
  * Sweeps all four node addresses, sending an IOIN read request and capturing
  * whatever comes back verbatim into g_uart_rx. Nothing is interpreted, so a
  * wrong assumption in the driver's echo handling cannot hide a working link.
  *
  * Set a breakpoint on the g_uart_pass++ line and inspect the arrays.
  */
void UartProbeTask(void *argument)
{
    (void)argument;

    vTaskDelay(pdMS_TO_TICKS(200));

    /* Force the state half-duplex actually needs, in case MspInit or CubeMX
     * left something inconsistent. Both TE and RE stay set permanently. */
    USART2->CR1 &= ~USART_CR1_UE;
    USART2->CR2 &= ~(USART_CR2_LINEN | USART_CR2_CLKEN);
    USART2->CR3 |=  USART_CR3_HDSEL;
    USART2->CR1 |=  (USART_CR1_TE | USART_CR1_RE);
    USART2->CR1 |=  USART_CR1_UE;

    for (;;) {
        for (uint8_t a = 0; a < 4; a++) {

            uint8_t req[4];
            req[0] = 0x05;
            req[1] = a;
            req[2] = 0x06;              /* IOIN */
            req[3] = probe_crc(req, 3);

            for (uint8_t i = 0; i < 4; i++) g_uart_tx[a][i] = req[i];

            uint8_t rx[PROBE_RX_MAX] = {0};
            uint8_t flags = 0;
            uint8_t n = 0;

            /* Flush anything stale so a leftover byte can't masquerade
             * as part of this exchange. */
            __HAL_UART_CLEAR_OREFLAG(&huart2);
            __HAL_UART_SEND_REQ(&huart2, UART_RXDATA_FLUSH_REQUEST);

            /* --- request, capturing the echo byte by byte --- */
            uint8_t tx_ok = 1;
            for (uint8_t i = 0; i < 4 && tx_ok; i++) {
                tx_ok = probe_putc(req[i], &rx[n], &flags);
                if (tx_ok) n++;
            }
            g_uart_txn[a] = tx_ok ? 0u : 3u;

            /* --- reply: 8 bytes, driver answers within ~1 ms --- */
            while (n < PROBE_RX_MAX) {
                if (!probe_getc(&rx[n], 10u, &flags)) break;
                n++;
            }
            g_uart_rxn[a]   = (n >= PROBE_RX_MAX) ? 0u : 3u;
            g_uart_nrx[a]   = n;
            g_uart_isr[a]   = USART2->ISR;
            g_uart_flags[a] = flags;

            uint8_t nz = 0;
            for (uint8_t i = 0; i < 16; i++) {
                g_uart_rx[a][i] = (i < PROBE_RX_MAX) ? rx[i] : 0u;
                if (g_uart_rx[a][i] != 0x00) nz++;
            }
            g_uart_nonzero[a] = nz;

            /* Echo integrity: proves the pin drives and senses the line. */
            g_uart_echo_ok[a] = (n >= 4 &&
                                 rx[0] == req[0] && rx[1] == req[1] &&
                                 rx[2] == req[2] && rx[3] == req[3]) ? 1u : 0u;

            /* Reply frame: 05 FF <reg> <d3 d2 d1 d0> <crc> */
            g_uart_crc_ok[a] = 0;
            g_uart_val[a]    = 0;
            if (n >= PROBE_RX_MAX && rx[4] == 0x05 && rx[5] == 0xFF) {
                if (probe_crc(&rx[4], 7) == rx[11]) {
                    g_uart_crc_ok[a] = 1u;
                    g_uart_val[a] = ((uint32_t)rx[7]  << 24) |
                                    ((uint32_t)rx[8]  << 16) |
                                    ((uint32_t)rx[9]  <<  8) |
                                     (uint32_t)rx[10];
                }
            }

            vTaskDelay(pdMS_TO_TICKS(50));
        }

        g_uart_pass++;   /* <<< BREAKPOINT HERE, then inspect g_uart_* */

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/**
  * @brief  1 Hz LED blink -- proves the scheduler is running.
  */
void HeartBeatTask(void *argument)
{
    (void)argument;
    for (;;) {
        HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM8 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM8)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
