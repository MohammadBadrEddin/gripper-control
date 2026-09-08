/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* ---------------------------------------------------------------------------
 * Debug / telemetry logging -- USART3 (ST-LINK VCP, PD8/PD9) by default,
 * see Debug_Init() in main.c. Non-blocking (interrupt-driven ring buffer),
 * timestamped via the DWT cycle counter (microsecond resolution). Capture
 * with TeraTerm's "Terminal -> Log..." to get a CSV file straight into
 * MATLAB (readtable/readmatrix) -- see docs/logging-signale.md.
 *
 *     Debug_Init(&huart3);                  // once, after MX_USART3_UART_Init()
 *     Debug_Printf("t_ms,theta,e\r\n");
 *     Debug_Printf("%.3f,%.3f,%.3f\r\n", Debug_TimestampMs(), theta, e);
 *
 * Wired into HAL_UART_TxCpltCallback() in main.c already -- nothing else to
 * hook up. Not thread-safe against concurrent callers from multiple tasks;
 * call it from a single task, or add a mutex around Debug_Printf if needed.
 * ------------------------------------------------------------------------- */
void Debug_Init(UART_HandleTypeDef *huart);
void Debug_Printf(const char *fmt, ...);
uint32_t Debug_TimestampUs(void);
float Debug_TimestampMs(void);
uint32_t Debug_Drops(void);   /* cumulative dropped lines (ring buffer was full) */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_GREEN_Pin GPIO_PIN_0
#define LED_GREEN_GPIO_Port GPIOB
#define TMC_STEP_Pin GPIO_PIN_9
#define TMC_STEP_GPIO_Port GPIOE
#define TMC_DIR_Pin GPIO_PIN_11
#define TMC_DIR_GPIO_Port GPIOE
#define TMC_EN_Pin GPIO_PIN_13
#define TMC_EN_GPIO_Port GPIOE
#define TMC_DIAG_Pin GPIO_PIN_15
#define TMC_DIAG_GPIO_Port GPIOE
#define TMC_DIAG_EXTI_IRQn EXTI15_10_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
