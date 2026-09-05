/*
 * motor_control.h
 *
 *  Created on: Sep 3, 2026
 *      Author: larsh
 */

#ifndef MOTOR_CONTROL_H_
#define MOTOR_CONTROL_H_

#include "FreeRTOS.h"
#include "task.h"
#include "tmc2209.h"
#include <stdint.h>
#include <stdbool.h>

void MotorControl_Init(TIM_HandleTypeDef *htim, UART_HandleTypeDef *huart);
void MotorControlTask(void *argument);

// Startet eine Bewegung, blockiert NICHT. Gibt false zurück, falls bereits eine Bewegung läuft
bool MotorControl_Move(int32_t steps, uint32_t max_speed_sps, uint32_t accel_sps2);

// Blockiert den AUFRUFENDEN Task, bis aktuelle Bewegung fertig ist
void MotorControl_WaitIdle(void);

// Aus HAL_TIM_PeriodElapsedCallback() fuer TIM3 aufrufen!
void MotorControl_TimerISR(void);

#endif /* MOTOR_CONTROL_H_ */
