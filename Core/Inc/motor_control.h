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
/* 1/16 Mikroschritt -> TMC MRES=4. War kurzzeitig auf Vollschritt (1) gestellt,
 * das war ein Denkfehler (siehe Team-Absprache 08.09.2026): bei Vollschritt
 * ist 1 Schritt = 0,267mm, der Lastwinkel bei 5N liegt aber nur bei 0,056mm
 * -- damit gar keine Auflösung, um die Soll-Ist-Differenz überhaupt
 * abzubilden. Mit 1/16 sind es ~0,0167mm/Mikroschritt, genug Reserve.
 * Wer will, kann hier auf 32/64 hochgehen fuer noch mehr Marge -- einfach
 * nur diese eine Konstante aendern, der Rest (USTEPS_PER_REV, Positions-
 * zaehler in motor_control.c, alle MotorControl_Move()-Distanzen) skaliert
 * automatisch mit. NUR die Geschwindigkeits-/Beschleunigungs-Argumente an
 * den MotorControl_Move()-Aufrufen sind in "Schritten bei DIESER Auflösung"
 * angegeben und muessen von Hand mitgezogen werden (in main.c bereits
 * getan: x16 gegenueber den alten Vollschritt-Werten). */
#define MOTOR_MICROSTEPS          16u
#define USTEPS_PER_REV            (MOTOR_FULLSTEPS_PER_REV * MOTOR_MICROSTEPS)  /* 3200 */

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

/* Absolute Position in MIKROSCHRITTEN (nicht Vollschritten), seit Boot, aus
 * der ISR fortgeschrieben -- kein Bus-Zugriff, kostet nichts, darum fuer
 * 1kHz-Telemetrie geeignet. Skaliert mit MOTOR_MICROSTEPS, das (per
 * Vorgabe) fest verdrahtet ist und nicht zur Laufzeit wechselt: ein
 * kompletter STEP-Puls entspricht immer MOTOR_MICROSTEPS Mikroschritten. */
int32_t MotorControl_GetPositionMicrosteps(void);

// Startet eine Bewegung, blockiert NICHT. Gibt false zurück, falls bereits eine Bewegung läuft
bool MotorControl_Move(int32_t steps, uint32_t max_speed_sps, uint32_t accel_sps2);

// Blockiert den AUFRUFENDEN Task, bis aktuelle Bewegung fertig ist
void MotorControl_WaitIdle(void);

// Aus HAL_TIM_PeriodElapsedCallback() fuer TIM3 aufrufen!
void MotorControl_TimerISR(void);

#endif /* MOTOR_CONTROL_H_ */
