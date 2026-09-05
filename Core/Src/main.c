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
//void EncoderTestTask(void *argument);
void HeartBeatTask(void *argument);
void MotorInitAndTestTask(void *argument);
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



  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_THREADS */
  // xTaskCreate(StartStepperTestTask, "StepperTest", 512, NULL, 3, NULL);
  xTaskCreate(MotorInitAndTestTask, "MotorCtrl", 512, NULL, 3, NULL);
//  xTaskCreate(EncoderTestTask,      "Encoder",     512, NULL, 2, NULL);
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
//void EncoderTestTask(void *argument)
//{
//    (void)argument;
//    static AS5600 enc;
//
//    /* Let the AS5600 power up before the first transaction. */
//    vTaskDelay(pdMS_TO_TICKS(100));
//
//    /* Probe the bus. A failure here is almost always pull-ups, wiring, or the
//     * module not being powered from 3.3 V. Retry rather than give up, so the
//     * wiring can be fixed and seen to come alive without a reflash. */
//    for (;;) {
//        g_enc_present = AS5600_Init(&enc, &hi2c1) ? 1u : 0u;
//        if (g_enc_present) {
//            break;
//        }
//        vTaskDelay(pdMS_TO_TICKS(500));
//    }
//
//    for (;;) {
//        g_enc_magnet = AS5600_MagnetOK(&enc) ? 1u : 0u;
//
//        uint16_t raw = AS5600_ReadRaw(&enc);
//        if (raw == 0xFFFF) {
//            g_enc_errors++;
//        } else {
//            g_enc_raw = raw;
//            g_enc_deg = raw * (360.0f / 4096.0f);
//            g_enc_samples++;
//        }
//
//        vTaskDelay(pdMS_TO_TICKS(100));   /* 10 Hz */
//    }
//}

void MotorInitAndTestTask(void *argument)
{
//    (void)argument;
    MotorControl_Init(&htim3, &huart2);

    for (;;) {
        MotorControl_Move(3200, 2000, 8000);   // 3200 Mikroschritte vor, 2000 sps, 8000 sps^2
        MotorControl_WaitIdle();
        vTaskDelay(pdMS_TO_TICKS(500));

        MotorControl_Move(-3200, 2000, 8000);  // 3200 Mikroschritte zurück
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
