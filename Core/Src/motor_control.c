/*
 * motor_control.c
 *
 *  Created on: Sep 3, 2026
 *      Author: larsh
 */

#include "motor_control.h"
#include "main.h"   		// für TMC_DIR_Pin, TMC_STEP_Pin, TMC_EN_Pin, GPIO-Ports
#include <math.h>

/* ---- Konfiguration ------------------------------------------------------ */
#define TIM3_CLK_HZ        1000000UL   // PSC=107 -> 1 MHz Zähltakt
#define MIN_ARR            9U          // Sicherheitsuntergrenze, ~100 kHz max
#define MAX_ARR            65000U      // Sicherheitsobergrenze für sehr langsame Starts

//volatile uint32_t checkCount_TIM3 = 0;


/* ---- Modulzustand --------------------------------------------------------
 * ISR-Variablen sind volatile, da sie zwischen Task und ISR geteilt werden. */
static TIM_HandleTypeDef *s_htim;
static TMC2209 s_drv;

static volatile int32_t  s_stepsRemaining;   	// verbleibende Vollschritte
static volatile uint32_t s_arrAccel;         	// aktueller ARR-Wert (Austin-Algorithmus)
static volatile uint32_t s_arrMin;           	// ARR bei Reisegeschwindigkeit (= obere Speedgrenze)
static volatile uint32_t s_rampStep;         	// n im Austin-Algorithmus (Beschleunigungsphase)
static volatile uint32_t s_decelStep;        	// Restschritt-Zaehler in der Bremsphase
static volatile uint32_t s_accelSteps;       	// Schritte bis Reisegeschwindigkeit (fuer symmetrisches Bremsen)
static volatile bool     s_cruising = false; 	// true: Reisegeschwindigkeit erreicht -> konstante Drehzahl
static volatile bool     s_decel    = false; 	// true: Bremsphase
static volatile bool     s_moveActive = false;
static volatile bool     s_pulseHigh  = false;	// Toggle-Zustand: hoch/runter
static volatile bool     s_driverReady = false;	// true erst NACH erfolgreichem TMC2209_Init() (aus Stand A)

static volatile int32_t  s_positionFullSteps = 0;	// absolute Position seit Boot, in VOLLSCHRITTEN (aus Stand A)
static volatile int32_t  s_moveDirSign = 1;			// +1/-1, pro Move() aus dem Vorzeichen von steps gesetzt

// Task, der gerade auf WaitIdle() wartet --> wird bei jedem Move-Aufruf neu gesetzt
static volatile TaskHandle_t s_waitingTask = NULL;

/* ---------------------------------------------------------------------------
 * Austin/Aryeh-Rampenalgorithmus (ganzzahlig, ISR-tauglich):
 *   c0   = f_timer * sqrt(2 / accel)          (ARR für den ersten Schritt)
 *   c_n  = c_{n-1} - (2*c_{n-1}) / (4*n + 1)   (jeder weitere Schritt, konvergiert gegen sqrt-Rampe ohne sqrt() in der ISR)
 * Quelle: D. Austin, "Generate stepper-motor speed profiles in real time",
 * Embedded Systems Programming, 2005 -- Standardverfahren für diese Anwendung.
 * ------------------------------------------------------------------------- */
static uint32_t austin_c0(uint32_t accel_sps2)
{
    /* c0 = f * sqrt(2/accel); Korrekturfaktor 0.676 kompensiert Diskretisierungsfehler des iterativen Verfahrens (Austin, Gl. 15/16). */
    float c0 = (float)TIM3_CLK_HZ * sqrtf(2.0f / (float)accel_sps2) * 0.676f;
    return (uint32_t)c0;
}

/* ---------------------------------------------------------------------------
 * MotorControl_Init -- einmalig aus Task heraus (NICHT vor Scheduler-Start!)
 * ------------------------------------------------------------------------- */
void MotorControl_Init(TIM_HandleTypeDef *htim, UART_HandleTypeDef *huart)
{
    s_htim = htim;

    bool ok = false;
	for (;;) {
		ok = TMC2209_Init(&s_drv, huart, 0);
		if (ok)
			break;
		vTaskDelay(pdMS_TO_TICKS(500));   /* Retry statt Halt -- per Debugger beobachtbar */
	}

    TMC2209_SetMicrosteps(&s_drv, MOTOR_MICROSTEPS);   // 1/16 -- muss zu USTEPS_PER_REV passen
    /* IRUN=27 -> ~0.86 A_RMS (Nennstrom 1.2A, R_SENSE=0.11, vsense=1), sicher < 1.2 A.
     * IHOLD = IRUN: Effort-Halt gegen Feder braucht volles Haltemoment, sonst
     * drueckt die Federlast den Motor im HOLD zurueck (IHOLD=16 => zu wenig). */
    TMC2209_SetCurrent(&s_drv, 27, 27);

    /* StallGuard/CoolStep-Fenster oeffnen, sonst ist SG_RESULT ausserhalb des
     * TCOOLTHRS-Fensters bedeutungslos. Konservative Startwerte -- experimentell
     * nachjustieren, sobald SG_RESULT-Logging reale Werte unter Last zeigt. (aus Stand A) */
    /* TCOOLTHRS hoch -> StallGuard schon ab ~0.7 mm/s gueltig. Mit 6000 war SG
     * erst > ~33 mm/s gueltig; der Effort-Regler faehrt aber <=20 mm/s. */
    TMC2209_SetCoolStepThreshold(&s_drv, 300000);
    TMC2209_SetStallguardThreshold(&s_drv, 10);

    HAL_GPIO_WritePin(TMC_EN_GPIO_Port, TMC_EN_Pin, GPIO_PIN_RESET);	// Motor jetzt erst aktivieren

    s_driverReady = true;   /* ab hier darf MotorControl_GetDriver() den Handle rausgeben */
}

/* TMC-Handle fuer Diagnose-Reads; NULL bis Init durch. (aus Stand A) */
TMC2209 *MotorControl_GetDriver(void)
{
    return s_driverReady ? &s_drv : NULL;
}

/* Absolute Position seit Boot in Mikroschritten. (aus Stand A) */
int32_t MotorControl_GetPositionMicrosteps(void)
{
    return s_positionFullSteps * (int32_t)MOTOR_MICROSTEPS;
}

/* ---------------------------------------------------------------------------
 * Öffentliche API: Bewegung anstoßen (non-blocking, geht in die Queue)
 * ------------------------------------------------------------------------- */
bool MotorControl_Move(int32_t steps, uint32_t max_speed_sps, uint32_t accel_sps2)
{
	if (s_moveActive || steps == 0) {
	        return false;
	    }

	    if (steps > 0) {
	        HAL_GPIO_WritePin(TMC_DIR_GPIO_Port, TMC_DIR_Pin, GPIO_PIN_SET);
	        s_moveDirSign = 1;
	    } else {
	        HAL_GPIO_WritePin(TMC_DIR_GPIO_Port, TMC_DIR_Pin, GPIO_PIN_RESET);
	        s_moveDirSign = -1;
	    }
	    vTaskDelay(1);   /* DIR->STEP Setup-Zeit */

	    int32_t totalSteps = (steps > 0) ? steps : -steps;

	    uint32_t arrMin = TIM3_CLK_HZ / max_speed_sps;
	    if (arrMin < MIN_ARR) arrMin = MIN_ARR;
	    if (arrMin > MAX_ARR) arrMin = MAX_ARR;

	    s_stepsRemaining = totalSteps;
	    s_rampStep       = 0;
	    s_decelStep      = 0;
	    s_accelSteps     = 0;
	    s_cruising       = false;
	    s_decel          = false;
	    s_arrAccel       = austin_c0(accel_sps2);
	    s_arrMin         = arrMin;
	    s_pulseHigh      = false;
	    s_waitingTask    = xTaskGetCurrentTaskHandle();   /* NEU: richtiger Task! */

	    __HAL_TIM_SET_COUNTER(s_htim, 0);
	    __HAL_TIM_SET_AUTORELOAD(s_htim, s_arrAccel);
	    s_moveActive = true;

	    HAL_TIM_Base_Start_IT(s_htim);
	    return true;
}

void MotorControl_WaitIdle(void)
{
    /* Wartet auf Notification, die der Task sich selbst nach Bewegungsende schickt --> siehe Task-Ende der aktuellen Bewegung. */
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
}


/* ---------------------------------------------------------------------------
 * MotorControl_TimerISR aus HAL_TIM_PeriodElapsedCallback() für TIM3 aufrufen --> Läuft im Interrupt-Kontext!
 * Nur am LETZTEN Schritt wird FreeRTOS-API genutzt (Notify) --> alle anderen Aufrufe sind reine Register-Zugriffe,
 * damit ISR im Normalfall minimal-jittrig bleibt.
 * ------------------------------------------------------------------------- */
void MotorControl_TimerISR(void)
{
//	checkCount_TIM3++;

	if (!s_moveActive) return;

	    HAL_GPIO_TogglePin(TMC_STEP_GPIO_Port, TMC_STEP_Pin);
	    s_pulseHigh = !s_pulseHigh;

	    if (!s_pulseHigh) {
	        s_stepsRemaining--;
	        s_positionFullSteps += s_moveDirSign;	// Positions-Tracking fuer Telemetrie (aus Stand A)

	        if (s_stepsRemaining <= 0) {
	            HAL_TIM_Base_Stop_IT(s_htim);
	            s_moveActive = false;

	            if (s_waitingTask != NULL) {
	                BaseType_t hpw = pdFALSE;
	                vTaskNotifyGiveFromISR(s_waitingTask, &hpw);
	                portYIELD_FROM_ISR(hpw);
	            }
	            return;
	        }

	        /* Trapezprofil: beschleunigen -> KONSTANTE Reisegeschwindigkeit -> bremsen.
	         * Die Bremsphase ist genau so lang wie die Beschleunigungsphase, damit die
	         * Drehzahl ueber die gesamte Fahrt (ausser den kurzen Rampen) konstant ist. */
	        if (!s_decel) {
	            /* Zum Bremsen noetige Schritte = zum Beschleunigen gebrauchte Schritte.
	             * Vor Erreichen der Reisegeschwindigkeit ist das der laufende Zaehler,
	             * sodass bei zu kurzer Strecke automatisch ein Dreieck entsteht. */
	            uint32_t decelNeeded = s_cruising ? s_accelSteps : s_rampStep;
	            if (decelNeeded == 0U) decelNeeded = 1U;

	            if ((uint32_t)s_stepsRemaining <= decelNeeded) {
	                s_decel     = true;          /* ab jetzt bremsen */
	                s_decelStep = decelNeeded;   /* symmetrisch zur Beschleunigung */
	            }
	        }

	        if (!s_decel) {
	            if (!s_cruising) {
	                /* Beschleunigen: Austin-Iteration, ARR verkleinern (schneller). */
	                s_rampStep++;
	                s_arrAccel = s_arrAccel - (2U * s_arrAccel) / (4U * s_rampStep + 1U);
	                if (s_arrAccel <= s_arrMin) {
	                    s_arrAccel   = s_arrMin;   /* Reisegeschwindigkeit erreicht */
	                    s_cruising   = true;
	                    s_accelSteps = s_rampStep;
	                }
	            }
	            /* Reisephase: ARR bleibt s_arrMin -> konstante Drehzahl (nichts zu tun). */
	        } else {
	            /* Bremsen: Austin-Iteration rueckwaerts, ARR vergroessern (langsamer). */
	            if (s_decelStep > 1U) {
	                s_decelStep--;
	                s_arrAccel = s_arrAccel + (2U * s_arrAccel) / (4U * s_decelStep + 1U);
	                if (s_arrAccel > MAX_ARR) s_arrAccel = MAX_ARR;
	            }
	        }
	        __HAL_TIM_SET_AUTORELOAD(s_htim, s_arrAccel);
	    }
}
