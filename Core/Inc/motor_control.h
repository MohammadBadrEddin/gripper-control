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

/* ---- Mechanik / Auflösung -------------------------------------------------
 * EINE Quelle der Wahrheit: die Mikroschritt-Auflösung, die MotorControl_Init
 * per TMC2209_SetMicrosteps() in den Treiber schreibt, und die daraus
 * abgeleiteten Weg-Konstanten. Move-Distanzen als Vielfache von USTEPS_PER_REV
 * angeben, damit Kommando und tatsächliche Auflösung nie auseinanderlaufen. */
#define MOTOR_FULLSTEPS_PER_REV   200u                                  /* 1,8°/Schritt */
#define MOTOR_MICROSTEPS          1u                                    /* Vollschritt -> TMC MRES=8 */
#define USTEPS_PER_REV            (MOTOR_FULLSTEPS_PER_REV * MOTOR_MICROSTEPS)  /* 200 */

void MotorControl_Init(TIM_HandleTypeDef *htim, UART_HandleTypeDef *huart);
void MotorControlTask(void *argument);

// Startet eine Bewegung, blockiert NICHT. Gibt false zurück, falls bereits eine Bewegung läuft
bool MotorControl_Move(int32_t steps, uint32_t max_speed_sps, uint32_t accel_sps2);

// Blockiert den AUFRUFENDEN Task, bis aktuelle Bewegung fertig ist
void MotorControl_WaitIdle(void);

// Aus HAL_TIM_PeriodElapsedCallback() fuer TIM3 aufrufen!
void MotorControl_TimerISR(void);

// TMC2209-Handle fuer Diagnose-Reads (SG_RESULT, DRV_STATUS ...) aus anderen Tasks.
// Liefert NULL, solange MotorControl_Init() noch nicht durchgelaufen ist. (aus Stand A)
TMC2209 *MotorControl_GetDriver(void);

// Absolute Position seit Boot in MIKROSCHRITTEN, aus der Schrittzaehlung der ISR. (aus Stand A)
int32_t MotorControl_GetPositionMicrosteps(void);

#endif /* MOTOR_CONTROL_H_ */
