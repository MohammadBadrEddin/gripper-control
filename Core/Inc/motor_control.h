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

/* Zugriff auf den intern gehaltenen TMC2209-Treiber-Handle, fuer periodische
 * Diagnose-Reads (StallGuard/DRV_STATUS/TSTEP) aus einem ANDEREN Task heraus
 * (z.B. EncoderTestTask), ohne den Treiberzustand hier zu duplizieren. Reads
 * ueber diesen Handle sind Thread-safe (drv->mutex in tmc2209.c serialisiert
 * sie gegen die Reads/Writes, die MotorControl_Init intern macht). Liefert
 * NULL, solange MotorControl_Init noch nicht erfolgreich durchgelaufen ist --
 * IMMER auf NULL pruefen, bevor der Pointer an TMC2209_Read*() übergeben wird. */
TMC2209 *MotorControl_GetDriver(void);

// Startet eine Bewegung, blockiert NICHT. Gibt false zurück, falls bereits eine Bewegung läuft
bool MotorControl_Move(int32_t steps, uint32_t max_speed_sps, uint32_t accel_sps2);

// Blockiert den AUFRUFENDEN Task, bis aktuelle Bewegung fertig ist
void MotorControl_WaitIdle(void);

// Aus HAL_TIM_PeriodElapsedCallback() fuer TIM3 aufrufen!
void MotorControl_TimerISR(void);

#endif /* MOTOR_CONTROL_H_ */
