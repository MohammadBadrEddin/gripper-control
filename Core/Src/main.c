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
#include "datalog.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ---- Positionsregler (Auftrag §5) ----------------------------------------
 * Strecke = Integrator -> reiner P-Regler. Aktor = VACTUAL ueber UART. */
#define CTRL_T_S          0.002f      /* Abtastzeit 2 ms                       */
#define CTRL_KP           125.7f      /* P-Verstaerkung [1/s]                  */
#define CTRL_VMAX_MMS     40.0f       /* Geschwindigkeits-Saettigung [mm/s]    */
#define CTRL_AMAX_MMS2    5000.0f     /* Rate-Limiter (ersetzt Rampengen.)     */
#define CTRL_DEADBAND_MM  0.0261f     /* Dead-Zone (2 Encoderstufen)           */
#define VACTUAL_PER_MMS   83.77f      /* VACTUAL = 83.77 * v[mm/s] (1/16)      */
#define MM_PER_REV        53.407f     /* Ritzel Ø17: pi*17 mm/Umdrehung        */
#define MM_PER_COUNT      (MM_PER_REV / 4096.0f)   /* AS5600: 4096 counts/U    */
#define VACTUAL_TO_USTEP  0.71526f    /* usteps/s = 0.71526 * VACTUAL (fCLK 12MHz) */

#define CTRL_XMIN_MM      (-10.0f)    /* Runaway-Grenzen -> FAULT              */
#define CTRL_XMAX_MM      60.0f       /* > 1 Umdrehung (53.4 mm) + Reserve     */

/* Treppen-Trajektorie: eine volle Umdrehung (53.4 mm) in 20 Schritten */
#define REV_STEPS         20u
#define REV_STEP_MM       (MM_PER_REV / (float)REV_STEPS)  /* ~2.670 mm/Schritt */
#define REV_DWELL_CYC     200u        /* Verweildauer je Schritt: 200*2ms = 0.4 s */
#define DIR_TEST_MMS      5.0f        /* Open-Loop Richtungs-/Kopplungstest    */
#define DIR_TEST_CYCLES   100u        /* ~0.2 s                                */
#define DIR_MIN_COUNTS    20          /* Mindestbewegung, sonst FAULT          */

/* Zustaende der Ablaufsteuerung (Logfeld 'state') */
enum { ST_ZERO = 1, ST_DIRDETECT = 2, ST_CONTROL = 3, ST_FAULT = 5 };

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

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

volatile uint8_t control_UART_Tx_Flag = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);

/* USER CODE BEGIN PFP */
//void StartStepperTestTask(void *argument);
void EncoderTestTask(void *argument);
void HeartBeatTask(void *argument);
void MotorInitAndTestTask(void *argument);
void LoggerTask(void *argument);
void DumpButtonTask(void *argument);
void ControlTask(void *argument);
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
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  /* Keep the TMC2209 output stage DISABLED until a task configures it.
   * EN is active-low, so HIGH = outputs off. MX_GPIO_Init() drives it LOW,
   * which would energise the motor before any software setup runs.
   * 	motor activated: 	GPIO_PIN_RESET	=> EN enabled (LOW)!
   *	motor deactivated:	GPIO_PIN_SET	=> EN disabled (HIGH)! */
  HAL_GPIO_WritePin(TMC_EN_GPIO_Port, TMC_EN_Pin, GPIO_PIN_SET);

  /* Messdaten-Logger: USART3-VCP (PD8/PD9) + DWT-Zeitbasis + USER-Button PC13.
   * Startet das Logging sofort (Ringpuffer). Dump spaeter per Button. */
  Datalog_Init();

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_THREADS */
  // ControlTask = 2ms Positionsregler (VACTUAL) + Logging in einem Kontext.
  // Ersetzt MotorInitAndTestTask (STEP/DIR-Demo) und LoggerTask.
  xTaskCreate(ControlTask,          "Control",   768, NULL, 4, NULL);   // 2ms, hoechste Prio
  xTaskCreate(DumpButtonTask,       "Dump",      512, NULL, 2, NULL);   // Button PC13 -> CSV-Dump ueber USART3
  xTaskCreate(HeartBeatTask,        "vHB",       128, NULL, 1, NULL);

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
  * @brief System Clock Configuration
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
  huart2.Init.BaudRate = 115200;   /* DIAGNOSE: temporaer zurueck von 500k, um TMC-Comms zu isolieren (Auftrag will 500k) */
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

/**
  * @brief  Positionsregler (Auftrag §5) + Logging, fester 2-ms-Takt.
  *
  * Aktor: VACTUAL ueber UART (kein STEP/DIR). Ablauf:
  *   ST_ZERO      Startposition = 0 mm (Boot-Nullpunkt).
  *   ST_DIRDETECT kurzer Open-Loop +VACTUAL -> enc_dir bestimmen; keine
  *                Bewegung erkannt -> FAULT (Motor/Encoder nicht gekoppelt).
  *   ST_CONTROL   P-Regler auf x_soll, Deadband, Saettigung, Rate-Limiter,
  *                Runaway-Schutz (x_ist verlaesst [XMIN,XMAX] -> FAULT).
  *   ST_FAULT     VACTUAL = 0, stehen bleiben.
  * Alle Logfelder werden pro Zyklus gefuellt.
  */
void ControlTask(void *argument)
{
    (void)argument;

    /* TMC hochziehen (setzt 1/16, Strom, StallGuard, EN=on). Prio 4 -> der
     * Init-Read wird von keinem hoeher-prioren Task zerhackt. */
    MotorControl_Init(&htim3, &huart2);
    TMC2209 *drv = MotorControl_GetDriver();

    /* Encoder anlernen (nicht endlos). */
    static AS5600 enc;
    bool encOk = false;
    for (int i = 0; i < 10 && !encOk; i++) {
        encOk = AS5600_Init(&enc, &hi2c1);
        if (!encOk) vTaskDelay(pdMS_TO_TICKS(100));
    }
    g_enc_present = encOk ? 1u : 0u;

    uint8_t  state       = ST_ZERO;
    uint16_t enc_prev    = 0;
    bool     have_prev   = false;
    int32_t  total_counts = 0;      /* unwrapped, relativ zum Boot-Nullpunkt   */
    int32_t  enc_dir     = 1;       /* +1/-1, aus DIRDETECT                     */
    uint32_t dir_cnt     = 0;
    float    v_prev      = 0.0f;
    float    step_us_acc = 0.0f;    /* integrierte Mikroschritte (VACTUAL-Pfad) */
    uint16_t sgLast      = 0xFFFF;
    uint32_t sgDiv       = 0;
    uint32_t stepIdx     = 1;       /* aktueller Treppenschritt (1..REV_STEPS)  */
    uint32_t dwell       = 0;       /* Zaehler fuer die Verweildauer je Schritt */

    if (drv) TMC2209_MoveVelocity(drv, 0);   /* sicher stehen */

    TickType_t next = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&next, pdMS_TO_TICKS(2));   /* fester 2-ms-Takt */

        /* --- Encoder lesen + Multiturn-Unwrap --- */
        uint16_t raw = encOk ? AS5600_ReadRaw(&enc) : 0xFFFF;
        if (raw != 0xFFFF) {
            g_enc_raw = raw;
            if (!have_prev) { enc_prev = raw; have_prev = true; }
            int32_t d = (int32_t)raw - (int32_t)enc_prev;
            if (d >  2048) d -= 4096;      /* Unterlauf-Wrap */
            if (d < -2048) d += 4096;      /* Ueberlauf-Wrap  */
            total_counts += d;
            enc_prev = raw;
        }
        float x_ist = (float)enc_dir * (float)total_counts * MM_PER_COUNT;

        /* --- SG_RESULT fuers Log (unterabgetastet) --- */
        if (++sgDiv >= 5u) {
            sgDiv = 0;
            uint32_t sg;
            if (drv && TMC2209_Read(drv, TMC_SG_RESULT, &sg)) sgLast = (uint16_t)(sg & 0x3FFu);
            else                                              sgLast = 0xFFFF;
        }

        float   x_soll = 0.0f, e = 0.0f, v = 0.0f;
        int32_t vactual = 0;

        switch (state) {
        case ST_ZERO:
            total_counts = 0; x_ist = 0.0f;   /* Boot-Position = 0 mm */
            v_prev = 0.0f;
            dir_cnt = 0;
            state = ST_DIRDETECT;
            break;

        case ST_DIRDETECT:
            /* kleines +VACTUAL, schauen wie der Encoder reagiert */
            v = DIR_TEST_MMS;
            vactual = (int32_t)lrintf(VACTUAL_PER_MMS * v);
            if (drv) TMC2209_MoveVelocity(drv, vactual);
            if (++dir_cnt >= DIR_TEST_CYCLES) {
                if (drv) TMC2209_MoveVelocity(drv, 0);
                if (total_counts >  DIR_MIN_COUNTS)      enc_dir =  1;
                else if (total_counts < -DIR_MIN_COUNTS) enc_dir = -1;
                else                                     state   = ST_FAULT; /* keine Bewegung */
                if (state != ST_FAULT) {
                    v_prev = 0.0f;
                    state  = ST_CONTROL;    /* Boot-Nullpunkt bleibt (kein Re-Zero) */
                }
            }
            break;

        case ST_CONTROL:
            x_soll = (float)stepIdx * REV_STEP_MM;       /* Treppe: Schritt stepIdx */
            e = x_soll - x_ist;                          /* mm */
            if (fabsf(e) < CTRL_DEADBAND_MM) e = 0.0f;   /* Dead-Zone */
            v = CTRL_KP * e;                             /* mm/s */
            if (v >  CTRL_VMAX_MMS) v =  CTRL_VMAX_MMS;  /* Saettigung */
            if (v < -CTRL_VMAX_MMS) v = -CTRL_VMAX_MMS;
            {
                float dv = CTRL_AMAX_MMS2 * CTRL_T_S;    /* Rate-Limiter */
                if (v > v_prev + dv) v = v_prev + dv;
                if (v < v_prev - dv) v = v_prev - dv;
            }
            v_prev = v;
            vactual = (int32_t)lrintf(VACTUAL_PER_MMS * v);
            if (drv) TMC2209_MoveVelocity(drv, vactual);

            if (x_ist < CTRL_XMIN_MM || x_ist > CTRL_XMAX_MM) {  /* Runaway */
                if (drv) TMC2209_MoveVelocity(drv, 0);
                state = ST_FAULT;
            } else if (++dwell >= REV_DWELL_CYC) {       /* naechster Treppenschritt */
                dwell = 0;
                if (stepIdx < REV_STEPS) stepIdx++;      /* bis 20 (=1 Umdrehung), dann halten */
            }
            break;

        case ST_FAULT:
        default:
            v = 0.0f; vactual = 0; v_prev = 0.0f;
            if (drv) TMC2209_MoveVelocity(drv, 0);
            break;
        }

        /* step_cnt: kommandierte Mikroschritte integrieren (kein STEP-Pin). */
        step_us_acc += VACTUAL_TO_USTEP * (float)vactual * CTRL_T_S;

        /* --- Log --- */
        LogRecord r;
        r.t_us      = Datalog_TimestampUs();
        r.x_soll_mm = x_soll;
        r.x_ist_mm  = x_ist;
        r.e_mm      = e;
        r.v_cmd_mms = v;
        r.vactual   = vactual;
        r.enc_raw   = raw;                     /* 0xFFFF bei Lesefehler */
        r.step_cnt  = (int32_t)lrintf(step_us_acc);
        r.sg_result = sgLast;
        r.state     = state;
        r.i_run_akt = 16;
        Datalog_Sample(&r);
    }
}

void MotorInitAndTestTask(void *argument)
{
//    (void)argument;
    MotorControl_Init(&htim3, &huart2);

    for (;;) {
        MotorControl_Move(2 * (int32_t)USTEPS_PER_REV, 400, 3000);   // 2 Umdrehungen vor (400 Vollschritte), Reise 400 sps = 2 U/s, accel 3000 sps^2
        MotorControl_WaitIdle();
        vTaskDelay(pdMS_TO_TICKS(500));

        MotorControl_Move(-2 * (int32_t)USTEPS_PER_REV, 400, 3000);  // 2 Umdrehungen zurück, gleiche Geschwindigkeit
        MotorControl_WaitIdle();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/**
  * @brief  Messdaten-Logger, fester 2-ms-Takt (vTaskDelayUntil), hoechste Prio.
  *
  * Testphase (noch kein Regler): erfasst Zeitstempel (DWT), Encoder-Rohwert,
  * SG_RESULT (unterabgetastet) und die Mikroschritt-Position in den RAM-Ring-
  * puffer. Regler-Felder bleiben 0. Auslesen spaeter per Button (USART3-CSV).
  * Uebernimmt zugleich die AS5600-Lesung (frueher EncoderTestTask).
  */
void LoggerTask(void *argument)
{
    (void)argument;
    static AS5600 enc;
    bool encOk = false;

    /* Warten, bis MotorControl_Init den TMC hochgezogen hat, BEVOR der 2-ms-Takt
     * losläuft. Sonst zerhackt die Preemption (Prio 4 > 3) den Half-Duplex-
     * Init-Read des TMC -> RX-Overrun (F7-USART ohne FIFO) -> Init scheitert.
     * Max ~5 s; kommt der TMC nicht, wird trotzdem geloggt (Logger-Test). */
    for (int i = 0; i < 100 && MotorControl_GetDriver() == NULL; i++) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    /* AS5600 kurz anlernen -- NICHT endlos blockieren: fehlt der Encoder,
     * loggen wir enc_raw=0xFFFF weiter, damit die Logger-Pipeline testbar ist. */
    for (int i = 0; i < 5 && !encOk; i++) {
        encOk = AS5600_Init(&enc, &hi2c1);
        if (!encOk) vTaskDelay(pdMS_TO_TICKS(100));
    }
    g_enc_present = encOk ? 1u : 0u;

    uint32_t sgDiv  = 0;         /* SG nur jeden 5. Zyklus (10 ms), Wert wird gehalten */
    uint16_t sgLast = 0xFFFF;

    TickType_t next = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&next, pdMS_TO_TICKS(2));   /* fester 2-ms-Takt */
        if (!Datalog_IsActive()) continue;          /* nach dem Dump nichts mehr abtasten */

        LogRecord r = {0};
        r.t_us = Datalog_TimestampUs();

        r.enc_raw = encOk ? AS5600_ReadRaw(&enc) : 0xFFFF;
        if (r.enc_raw != 0xFFFF) g_enc_raw = r.enc_raw;

        if (++sgDiv >= 5u) {
            sgDiv = 0;
            TMC2209 *drv = MotorControl_GetDriver();
            uint32_t sg;
            if (drv && TMC2209_Read(drv, TMC_SG_RESULT, &sg)) sgLast = (uint16_t)(sg & 0x3FFu);
            else                                              sgLast = 0xFFFF;
        }
        r.sg_result = sgLast;

        r.step_cnt  = MotorControl_GetPositionMicrosteps();
        r.i_run_akt = 16;      /* aktuell fest; spaeter aus der Ablaufsteuerung */
        r.state     = 0;       /* noch keine FSM */
        /* x_soll/x_ist/e/v_cmd/vactual bleiben 0 -> noch kein Regler */

        Datalog_Sample(&r);
    }
}

/**
  * @brief  Pollt den USER-Button PC13 (~20 ms) und dumpt bei Druck den
  *         Ringpuffer als CSV ueber USART3 (blockierend, ~einige Sekunden).
  */
void DumpButtonTask(void *argument)
{
    (void)argument;
    for (;;) {
        if (Datalog_ButtonPressed()) {
            Datalog_Dump();
        }
        vTaskDelay(pdMS_TO_TICKS(20));
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
	control_UART_Tx_Flag++;

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
