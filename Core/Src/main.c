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
#include "motor_control.h"

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;   /* debug/log, alternate: external USB-UART adapter, PA9/PA10 */
UART_HandleTypeDef huart2;   /* TMC2209, half-duplex single-wire, PD5 */
UART_HandleTypeDef huart3;   /* debug/log, ST-LINK Virtual COM Port, PD8/PD9 */
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

/* TMC_DIAG (PE15/EXTI15) -- der Pin war schon als Interrupt scharf (siehe
 * MX_GPIO_Init), aber es gab bisher KEINEN HAL_GPIO_EXTI_Callback, der
 * darauf reagiert -- der IRQ lief ins Leere. Zaehler statt Aktion: DIAG
 * signalisiert je nach GCONF/COOLCONF entweder StallGuard-Schwelle oder
 * einen Treiberfehler (u.a. Uebertemperatur-Vorwarnung otpw) -- welches von
 * beidem, steht in DRV_STATUS (per TMC2209_ReadStatus() bereits vorhanden).
 * Hier nur der Zeitpunkt/die Haeufigkeit; Einordnung passiert beim
 * Auswerten der sg_result/otpw-Spalten aus derselben Sekunde. */
volatile uint32_t g_diagEvents = 0;

volatile uint8_t control_UART_Tx_Flag = 0;

/* ---------------------------------------------------------------------------
 * Debug / telemetry logging -- see main.h for the public API and usage.
 * Timestamp: DWT cycle counter (Cortex-M7, no extra timer peripheral).
 * Transport: HAL_UART_Transmit_IT + a ring buffer, so Debug_Printf() never
 * blocks the caller on the UART -- one CSV line (~80-120 B) at 460800 Bd
 * takes ~2 ms, which alone would blow a 250-500 Hz control loop if done
 * with a blocking HAL_UART_Transmit. No DMA/DMAMUX config needed.
 * ------------------------------------------------------------------------- */
#define DEBUG_LOG_BUF_SIZE   2048u   /* must be a power of two */

static UART_HandleTypeDef *s_debugHuart = NULL;
static uint8_t   s_debugBuf[DEBUG_LOG_BUF_SIZE];
static volatile uint16_t s_debugHead = 0;   /* next free slot to write into */
static volatile uint16_t s_debugTail = 0;   /* next byte to transmit */
static volatile bool     s_debugTxBusy = false;
static volatile uint32_t s_debugDrops = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void CPU_CACHE_Enable(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);

/* USER CODE BEGIN PFP */
//void StartStepperTestTask(void *argument);
void EncoderTestTask(void *argument);
void HeartBeatTask(void *argument);
void MotorInitAndTestTask(void *argument);

static void Debug_StartTx(void);
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

  /* Enable I/D cache before anything else runs -- the STM32F767's Cortex-M7
   * has both, but CubeMX-generated F7 projects often leave them off by
   * default. Worth having on for the timing-sensitive step pulses/logging. */
  CPU_CACHE_Enable();

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
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  /* Keep the TMC2209 output stage DISABLED until a task configures it.
   * EN is active-low, so HIGH = outputs off. MX_GPIO_Init() drives it LOW,
   * which would energise the motor before any software setup runs.
   * 	motor activated: 	GPIO_PIN_RESET	=> EN enabled (LOW)!
   *	motor deactivated:	GPIO_PIN_SET	=> EN disabled (HIGH)! */
  HAL_GPIO_WritePin(TMC_EN_GPIO_Port, TMC_EN_Pin, GPIO_PIN_SET);

  /* Debug/telemetry logging over the ST-LINK Virtual COM Port -- open with
   * TeraTerm at 460800 8N1. See main.h for Debug_Printf() usage. */
  Debug_Init(&huart3);
  Debug_Printf("\r\n# gripper-control boot, STM32F767ZI, SYSCLK=%lu Hz\r\n",
               (unsigned long)SystemCoreClock);
  Debug_Printf("# t_ms,vactual,enc_raw,step_cnt,sg_result,i_run_akt\r\n");

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_THREADS */
  // xTaskCreate(StartStepperTestTask, "StepperTest", 512, NULL, 3, NULL);
  xTaskCreate(MotorInitAndTestTask, "MotorCtrl", 512, NULL, 3, NULL);
  xTaskCreate(EncoderTestTask,      "Encoder",     512, NULL, 2, NULL);
  xTaskCreate(HeartBeatTask,        "vHB",         128, NULL, 1, NULL);

  /* Start scheduler */
  vTaskStartScheduler();

  /* USER CODE END RTOS_THREADS */

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief  Enable the Cortex-M7 I-Cache and D-Cache.
  * @retval None
  */
static void CPU_CACHE_Enable(void)
{
  SCB_EnableICache();
  SCB_EnableDCache();
}

/**
  * @brief System Clock Configuration (original STM32F767ZI config, unchanged)
  *         System Clock source            = PLL (HSE)
  *         SYSCLK(Hz)                     = 216000000
  *         HCLK(Hz)                       = 216000000
  *         AHB Prescaler                  = 1
  *         APB1 Prescaler                 = 4 (APB1 Clock  54 MHz)
  *         APB2 Prescaler                 = 2 (APB2 Clock 108 MHz)
  *         HSE Frequency(Hz)               = 8000000
  *         PLL_M                           = 4
  *         PLL_N                           = 216
  *         PLL_P                           = 2
  *         PLL_Q                           = 2
  *         PLL_R                           = 2
  *         VDD(V)                          = 3.3
  *         Flash Latency(WS)               = 7
  *         Over-Drive                      = enabled (required for 216 MHz)
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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
  /* CubeMX-computed TIMINGR for I2C1 kernel clock = PCLK1 = 54 MHz
   * (APB1CLKDivider = /4 off the 216 MHz SYSCLK), Standard Mode 100 kHz. */
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  /* TIM3 is on APB1 (54 MHz, /4 off 216 MHz SYSCLK); timer kernel clock is
   * 2x APB when the APB divider != 1, so 108 MHz here. PSC=107 ->
   * 108 MHz/(107+1) = 1 MHz tick, matching motor_control.c's
   * TIM3_CLK_HZ=1000000UL assumption. */
  htim3.Init.Prescaler = 107;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function -- debug/log, alternate: external
  *        USB-UART adapter on PA9(TX)/PA10(RX), 3.3 V levels.
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 460800;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */
}

/**
  * @brief USART2 Initialization Function -- TMC2209, half-duplex single-wire.
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
  /* 460800 statt 115200: bei 115200 Baud dauert allein ein SG_RESULT-Read
   * (4-Byte-Request + 8-Byte-Reply) ~1,04ms auf dem Draht -- das sprengt ein
   * 1ms-Log-Intervall sofort, ohne jede Reserve fuer Scheduling-Jitter oder
   * den I2C-Encoder-Read im selben Zyklus. Bei 460800 Baud sinkt derselbe
   * Read auf ~260us, was fuer die 1kHz-Telemetrie (siehe EncoderTestTask)
   * noetig ist. Der TMC2209 erkennt die Baudrate am ersten Telegramm
   * automatisch neu, keine Aenderung am Treiber-IC noetig. */
  huart2.Init.BaudRate = 460800;
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
  * @brief USART3 Initialization Function -- debug/log, ST-LINK Virtual COM
  *        Port on PD8(TX)/PD9(RX). Open with TeraTerm over the existing
  *        ST-LINK USB cable, no extra hardware.
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{
  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 460800;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

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

/* -----------------------------------------------------------------------
 * Debug / telemetry logging -- implementation. See main.h for the API.
 * ------------------------------------------------------------------------- */

void Debug_Init(UART_HandleTypeDef *huart)
{
  s_debugHuart = huart;
  s_debugHead = s_debugTail = 0;
  s_debugTxBusy = false;
  s_debugDrops = 0;

  /* Cortex-M7 DWT cycle counter -- free-running, microsecond-resolution.
   * NICHT mehr fuer t_ms verwendet (siehe Debug_TimestampMs(), Grund dort),
   * aber jetzt gebraucht fuer dt_us (Laufzeit pro Zyklus, EncoderTestTask):
   * dafuer reicht die 1ms-Aufloesung des FreeRTOS-Ticks nicht, ein einzelner
   * Zyklus braucht nur ein paar 100us. DWT->LAR muss auf diesem Cortex-M7
   * VOR CYCCNTENA entsperrt werden, sonst zaehlt CYCCNT trotz TRCENA nicht
   * (das war der Grund, warum t_ms vorher immer 0 blieb). */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->LAR    = 0xC5ACCE55;   /* Cortex-M7 Lock Access Register unlock */
  DWT->CYCCNT = 0;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t Debug_TimestampUs(void)
{
  /* Wraps at 2^32 us (~71.6 min) -- treat a decreasing timestamp in the log
   * as "wrapped", not an error; well outside a lab session either way.
   * NICHT mehr die Quelle fuer Debug_TimestampMs() (siehe dort) -- bleibt
   * hier nur noch fuer den Fall stehen, dass mal wirklich us-Aufloesung
   * gebraucht wird; dafuer muesste vorher DWT->LAR = 0xC5ACCE55 entsperrt
   * werden (Cortex-M7 Lock Access Register), sonst zaehlt CYCCNT nicht. */
  return DWT->CYCCNT / (SystemCoreClock / 1000000u);
}

float Debug_TimestampMs(void)
{
  /* Zeitbasis = FreeRTOS-Tick, NICHT mehr DWT->CYCCNT: der DWT-Zykluszaehler
   * lief auf diesem Cortex-M7 nie (t_ms war deshalb immer 0) -- er braucht
   * zusaetzlich zu TRCENA/CYCCNTENA ein Unlock ueber DWT->LAR, sonst werden
   * Schreibzugriffe auf DWT->CTRL/CYCCNT stillschweigend ignoriert. Der
   * FreeRTOS-Tick laeuft dagegen garantiert (configTICK_RATE_HZ=1000 ->
   * exakt 1ms/Tick, siehe FreeRTOSConfig.h) und ist zugleich die gemeinsame
   * Zeitbasis fuer ALLE Telemetriefelder in EncoderTestTask -- genau das
   * war hier gefordert. */
  return (float)(xTaskGetTickCount() * (TickType_t)portTICK_PERIOD_MS);
}

uint32_t Debug_Drops(void)
{
  return s_debugDrops;
}

/* Kick a transmission of the next contiguous run of bytes in the ring
 * buffer. Only call this with s_debugTxBusy already false. */
static void Debug_StartTx(void)
{
  if (s_debugHead == s_debugTail) {
    return;   /* nothing queued */
  }
  uint16_t len;
  if (s_debugHead > s_debugTail) {
    len = s_debugHead - s_debugTail;
  } else {
    len = DEBUG_LOG_BUF_SIZE - s_debugTail;   /* up to the wrap point only */
  }
  s_debugTxBusy = true;
  HAL_UART_Transmit_IT(s_debugHuart, &s_debugBuf[s_debugTail], len);
}

void Debug_Printf(const char *fmt, ...)
{
  if (s_debugHuart == NULL) {
    return;   /* Debug_Init() not called yet */
  }

  char line[200];
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
  uint16_t free_space = (uint16_t)((s_debugTail - s_debugHead - 1) & (DEBUG_LOG_BUF_SIZE - 1));
  if (free_space < (uint16_t)n) {
    s_debugDrops++;
    __enable_irq();
    return;   /* ring buffer still full from a previous burst -- drop, don't block */
  }
  for (int i = 0; i < n; i++) {
    s_debugBuf[s_debugHead] = (uint8_t)line[i];
    s_debugHead = (uint16_t)((s_debugHead + 1) & (DEBUG_LOG_BUF_SIZE - 1));
  }
  bool wasIdle = !s_debugTxBusy;
  __enable_irq();

  if (wasIdle) {
    Debug_StartTx();
  }
}

/**
  * @brief  Open-loop stepper test using STEP/DIR (standalone mode).
  *
  * UART/VACTUAL control is not used here: the TMC2209 UART link is still
  * unresolved, while STEP/DIR is confirmed working. In this mode microstepping
  * comes from the MS1/MS2 straps and motor current from the VREF pot -- there
  * is no software control of either.
  *
  * Behaviour: 400 steps one way, pause 0.5 s, reverse, repeat.
  */
/*
void StartStepperTestTask(void *argument)
{
    (void)argument;

    HAL_GPIO_WritePin(TMC_DIR_GPIO_Port, TMC_DIR_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TMC_EN_GPIO_Port,  TMC_EN_Pin,  GPIO_PIN_RESET);
    vTaskDelay(pdMS_TO_TICKS(10));

    for (;;) {
        // Busy-wait pulse timing. The loop count is empirical -- raise it to slow the motor down, lower it to speed up
        for (int i = 0; i < 400; i++) {
            HAL_GPIO_WritePin(TMC_STEP_GPIO_Port, TMC_STEP_Pin, GPIO_PIN_SET);
            for (volatile int d = 0; d < 5000; d++) { __NOP(); }
            HAL_GPIO_WritePin(TMC_STEP_GPIO_Port, TMC_STEP_Pin, GPIO_PIN_RESET);
            for (volatile int d = 0; d < 5000; d++) { __NOP(); }
        }

        vTaskDelay(pdMS_TO_TICKS(500));

        HAL_GPIO_TogglePin(TMC_DIR_GPIO_Port, TMC_DIR_Pin);   // reverse
    }
} */


/**
  * @brief  TMC_DIAG (PE15/EXTI15) Interrupt-Callback.
  *
  * Der Pin war schon vor dieser Aenderung als Interrupt scharf (siehe
  * MX_GPIO_Init/NVIC), aber es gab keinen HAL_GPIO_EXTI_Callback -- der IRQ
  * lief in den HAL-Weak-Default (macht nichts). Zaehlt hier nur die
  * Ereignisse; DIAG allein sagt nicht, OB es StallGuard oder ein
  * Treiberfehler (u.a. Uebertemperatur-Vorwarnung otpw) war -- das steht in
  * DRV_STATUS, per TMC2209_ReadStatus() bereits vorhanden. Absicht: wer die
  * Dump-CSV auswertet, sieht an g_diagEvents (im Header mitgeloggt) OB und
  * WANN in der Aufzeichnung ueberhaupt etwas ausgeloest hat, und kann das
  * gegen die sg_result-Spalte zur selben Zeit gegenpruefen.
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == TMC_DIAG_Pin) {
        g_diagEvents++;
    }
}

/* ---------------------------------------------------------------------------
 * Ringpuffer-Telemetrie (ersetzt das fruehere Live-Printing).
 *
 * Grund fuer die Umstellung: Senden im Reglertakt kostet Zeit IM Task, den
 * man eigentlich zeitgenau vermessen will -- das verfaelscht die Zeitbasis
 * mit der Messung selbst. Stattdessen: waehrend des Versuchs in einen
 * RAM-Puffer schreiben (kein UART-Zugriff im Zeitkritischen Pfad ausser dem
 * einen SG_RESULT-Read, der fuer die Messung selbst gebraucht wird), danach
 * einmal am Stueck ueber USART3 raus.
 *
 * Trigger fuer "Versuch fertig, jetzt dumpen": hier bewusst einfach gehalten
 * -- der Puffer fasst TELEMETRY_CAPACITY Samples, ist bei 2ms/Sample nach
 * TELEMETRY_CAPACITY*2ms voll, dann wird automatisch einmalig gedumpt und
 * die Aufzeichnung stoppt. Kein Start/Stop-Kommando ueber UART o.ae. -- falls
 * ihr mehrere/gezielte Versuche aufnehmen wollt (z.B. per Taster oder
 * UART-Kommando), ist das der Punkt, wo eine echte Trigger-Logik hin muesste;
 * aktuell laeuft die Aufzeichnung einfach ab dem Boot los.
 * ------------------------------------------------------------------------- */
#define TELEMETRY_CAPACITY   5000u   /* bei 2ms/Sample = 10s Aufzeichnung */

typedef struct __attribute__((packed)) {
    uint32_t t_ms;
    int32_t  vactual;
    uint16_t enc_raw;
    int32_t  step_cnt;
    uint16_t sg_result;
    uint8_t  i_run_akt;
    uint32_t dt_us;       /* Laufzeit DIESES Zyklus (Encoder-Read bis Puffer-Write) */
} TelemetrySample;

static TelemetrySample s_ringBuf[TELEMETRY_CAPACITY];   /* ~5000*21B =~103KB, passt locker in die 512KB SRAM */
static uint32_t        s_ringCount = 0;

/* Formatiert+sendet EINE Zeile BLOCKIEREND direkt über USART3, unabhängig
 * vom Debug_Printf()-Ringpuffer/IT-Mechanismus. Fuer den Dump nach dem
 * Versuch ist Timing egal, dafuer ist Reihenfolge/Vollstaendigkeit wichtig
 * -- bei mehreren tausend Zeilen am Stueck wuerde Debug_Printf() aus dem
 * kleinen 2KB-Ringpuffer (DEBUG_LOG_BUF_SIZE) Zeilen verwerfen (s_debugDrops),
 * das wollen wir beim Dump nicht. */
static void DumpLineBlocking(const char *fmt, ...)
{
    char line[96];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    if (n <= 0) return;
    if ((uint32_t)n >= sizeof(line)) n = sizeof(line) - 1;
    HAL_UART_Transmit(&huart3, (uint8_t *)line, (uint16_t)n, 100);
}

/**
  * @brief  Einmaliger Selbsttest beim ersten Mal, dass der Treiber bereit ist:
  *         IFCNT (Reg 0x02, zaehlt akzeptierte Schreibzugriffe) vor/nach
  *         einem echten Write lesen. Steigt IFCNT, kommt die UART2-
  *         Kommunikation zum TMC2209 tatsaechlich an -- unabhaengig davon,
  *         ob die Werte im laufenden Log plausibel aussehen.
  */
static void TMC_IfcntSelfTest(TMC2209 *drv)
{
    uint32_t before = 0, after = 0;
    bool ok_before = TMC2209_Read(drv, TMC_IFCNT, &before);
    TMC2209_SetStallguardThreshold(drv, 10);   /* echter Write, Wert unveraendert */
    bool ok_after  = TMC2209_Read(drv, TMC_IFCNT, &after);

    DumpLineBlocking("# IFCNT-Selbsttest: vorher=%lu(ok=%d) nachher=%lu(ok=%d) delta=%ld -- %s\r\n",
                      (unsigned long)before, ok_before,
                      (unsigned long)after, ok_after,
                      (long)(after - before),
                      (ok_before && ok_after && after != before) ? "UART2-Kommunikation OK" : "PRUEFEN -- IFCNT stieg nicht");
}

/**
  * @brief  Telemetrie-Aufzeichnung + Dump. Ein Sample alle 2ms (500Hz, fest
  *         aus der Auslegung), in den Ringpuffer, kein Live-Senden.
  *
  * CSV-Spalten im Dump: t_ms,vactual,enc_raw,step_cnt,sg_result,i_run_akt,dt_us
  *
  *   t_ms       uint32_t   FreeRTOS-Tick * portTICK_PERIOD_MS -- gemeinsame
  *                         Zeitbasis fuer ALLE Felder dieser Zeile, exakt im
  *                         2ms-Raster (siehe Debug_TimestampMs()).
  *   vactual    int32_t    Schattenwert, VACTUAL bleibt bei STEP/DIR-Betrieb 0.
  *   enc_raw    uint16_t   AS5600 RAW ANGLE, 0..4095, unumgerechnet.
  *   step_cnt   int32_t    absolute Position seit Boot, in MIKROSCHRITTEN.
  *   sg_result  uint16_t   TMC2209 SG_RESULT (0x41), einziger echter Bus-Read
  *                         pro Zyklus (~1,04ms bei 115200 Baud auf UART2 --
  *                         passt in die 2ms, siehe MX_USART2_UART_Init()).
  *                         Aktualisiert intern nur je Vollschritt-Aequivalent
  *                         -- bei 2ms-Abtastung also mehrfach derselbe Wert
  *                         hintereinander, im Stillstand eingefroren. So
  *                         gewollt (Team-Absprache 08.09.2026).
  *   i_run_akt  uint8_t    Schattenwert, aktuell gesetztes IRUN.
  *   dt_us      uint32_t   Laufzeit dieses Zyklus in us (DWT-Zykluszaehler,
  *                         NICHT die t_ms-Zeitbasis) -- zum Pruefen, wie viel
  *                         Reserve im 2ms-Budget noch da ist.
  *
  * Was NOCH FEHLT und hier bewusst NICHT erraten wurde: x_soll_mm, x_ist_mm,
  * e_mm, v_cmd_mms, state, sowie KP/KV/v_max/deadband in der Kopfzeile -- es
  * gibt aktuell keinen Positionsregler im Code, der diese Werte liefert. Das
  * ist kein Feld, das man einfach dazuschreibt, sondern ein eigener
  * Regelkreis, der erst spezifiziert werden muss.
  */
void EncoderTestTask(void *argument)
{
    (void)argument;
    static AS5600 enc;
    static bool selfTestDone = false;

    /* Let the AS5600 power up before the first transaction. */
    vTaskDelay(pdMS_TO_TICKS(100));

    while (s_ringCount < TELEMETRY_CAPACITY) {
        uint32_t t0 = Debug_TimestampUs();

        if (!g_enc_present) {
            g_enc_present = AS5600_Init(&enc, &hi2c1) ? 1u : 0u;
        }

        if (g_enc_present) {
            g_enc_magnet = AS5600_MagnetOK(&enc) ? 1u : 0u;

            uint16_t raw = AS5600_ReadRaw(&enc);
            if (raw == 0xFFFF) {
                g_enc_errors++;
            } else {
                g_enc_raw = raw;
                g_enc_deg = raw * (360.0f / 4096.0f);
                g_enc_samples++;
            }
        }

        int32_t  vactual   = 0;
        uint16_t sg_result = 0;
        uint8_t  irun_akt  = 0;

        TMC2209 *drv = MotorControl_GetDriver();
        if (drv != NULL) {
            if (!selfTestDone) {
                TMC_IfcntSelfTest(drv);
                selfTestDone = true;
            }
            vactual  = TMC2209_GetVActual(drv);
            irun_akt = TMC2209_GetIrun(drv);
            (void)TMC2209_ReadStallGuard(drv, &sg_result);   /* bleibt 0 bei Fehl-Read */
        }

        int32_t step_cnt = MotorControl_GetPositionMicrosteps();

        TelemetrySample *s = &s_ringBuf[s_ringCount++];
        s->t_ms      = (uint32_t)Debug_TimestampMs();
        s->vactual   = vactual;
        s->enc_raw   = g_enc_raw;
        s->step_cnt  = step_cnt;
        s->sg_result = sg_result;
        s->i_run_akt = irun_akt;
        s->dt_us     = Debug_TimestampUs() - t0;   /* Laufzeit DIESES Zyklus */

        vTaskDelay(pdMS_TO_TICKS(2));   /* 500 Hz, fest aus der Auslegung */
    }

    /* Aufzeichnung voll (10s bei 2ms) -- jetzt am Stueck dumpen. */
    TMC2209 *drv = MotorControl_GetDriver();
    DumpLineBlocking("\r\n# --- Versuchsparameter ---\r\n");
    DumpLineBlocking("# IRUN=%u IHOLD=%u microsteps=1/%u diag_events=%lu\r\n",
                      drv ? TMC2209_GetIrun(drv) : 0u,
                      drv ? TMC2209_GetIhold(drv) : 0u,
                      (unsigned)MOTOR_MICROSTEPS,
                      (unsigned long)g_diagEvents);
    DumpLineBlocking("# KP,KV,v_max,deadband: noch nicht implementiert -- kein Positionsregler im Code\r\n");
    DumpLineBlocking("# -------------------------\r\n");
    DumpLineBlocking("t_ms,vactual,enc_raw,step_cnt,sg_result,i_run_akt,dt_us\r\n");

    for (uint32_t i = 0; i < s_ringCount; i++) {
        TelemetrySample *s = &s_ringBuf[i];
        DumpLineBlocking("%lu,%ld,%u,%ld,%u,%u,%lu\r\n",
                          (unsigned long)s->t_ms, (long)s->vactual, s->enc_raw,
                          (long)s->step_cnt, s->sg_result, s->i_run_akt,
                          (unsigned long)s->dt_us);
    }
    DumpLineBlocking("# --- Ende Dump (%lu Samples) ---\r\n", (unsigned long)s_ringCount);

    /* Aufzeichnung ist ein einmaliger Vorgang -- Task ist danach fertig,
     * blockiert absichtlich fuer immer statt in eine leere Schleife zu
     * laufen (spart CPU, FreeRTOS schedult einfach weiter um ihn herum). */
    vTaskSuspend(NULL);
}

void MotorInitAndTestTask(void *argument)
{
//    (void)argument;
    MotorControl_Init(&htim3, &huart2);

    for (;;) {
        /* Geschwindigkeit/Beschleunigung x16 gegenueber den alten Vollschritt-
         * Werten (400 sps, 3000 sps^2) -- MOTOR_MICROSTEPS ist jetzt 16, die
         * Move()-Argumente sind in Schritten BEI DIESER Aufloesung angegeben,
         * sonst waere die reale Winkelgeschwindigkeit 16x langsamer geworden.
         * Ergebnis bleibt wie vorher: 2 U/s Reise, 2 Umdrehungen. */
        MotorControl_Move(2 * (int32_t)USTEPS_PER_REV, 6400, 48000);   // 2 Umdrehungen vor, Reise 6400 sps (=400 Vollschritte/s = 2 U/s), accel 48000 sps^2
        MotorControl_WaitIdle();
        vTaskDelay(pdMS_TO_TICKS(500));

        MotorControl_Move(-2 * (int32_t)USTEPS_PER_REV, 6400, 48000);  // 2 Umdrehungen zurück, gleiche Geschwindigkeit
        MotorControl_WaitIdle();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/*
// initialization of: TMC2209_Init
void MissionMotorControlTask (void* argument)
{
	static TMC2209 drive;
	TMC2209_Init(&drive, &huart2, 0);
	TMC2209_SetMicrosteps(&drive, 16);   // value based on mechanical characterization
	TMC2209_SetCurrent(&drive, 16, 8);
	HAL_GPIO_WritePin(TMC_EN_GPIO_Port, TMC_EN_Pin, GPIO_PIN_RESET); // activate motor just now
} */

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



void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if (huart->Instance == USART2) {
		control_UART_Tx_Flag++;
		return;
	}
	if (s_debugHuart != NULL && huart->Instance == s_debugHuart->Instance) {
		/* Advance tail by whatever the just-completed transfer covered, then
		 * re-arm if more data was queued while we were sending. */
		s_debugTail = (uint16_t)((s_debugTail + huart->TxXferSize) & (DEBUG_LOG_BUF_SIZE - 1));
		s_debugTxBusy = false;
		Debug_StartTx();
	}
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

}

void vApplicationIdleHook( void ){
	__WFI();
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
  else if (htim->Instance == TIM3) {
	 /* HAL_GPIO_TogglePin(TMC_STEP_GPIO_Port, TMC_STEP_Pin);
	 // ggf. Schrittzähler inkrementieren, ARR für Rampe nachführen */

	 MotorControl_TimerISR();
   }
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
