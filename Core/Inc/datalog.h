/*
 * datalog.h -- Messdaten-Logger fuer die Gripper-Firmware.
 *
 * Konzept (Auftrag §3.3): Waehrend des Versuchs schreibt der Reglertask jeden
 * Takt einen kompakten Binaer-Eintrag in einen statischen RAM-Ringpuffer. Es
 * wird NICHTS gesendet -> die Zeitbasis bleibt sauber. NACH dem Versuch wird
 * der Puffer auf Knopfdruck (USER-Button PC13) als CSV ueber USART3 (ST-Link
 * Virtual COM Port, PD8/PD9) ausgegeben; ein Terminal (TeraTerm "Log to file")
 * speichert das direkt als CSV.
 *
 * Zeitbasis: DWT-Zyklenzaehler (CYCCNT), Aufloesung 1 CPU-Takt, hier als
 * Mikrosekunden seit dem letzten Datalog_Reset(). Damit ist der Jitter des
 * 2-ms-Takts in den Timestamps sichtbar.
 *
 * Rueckfall (Auftrag §3.3): Der Puffer g_log[] ist ein statisches Symbol fester
 * Groesse -> per ST-Link Memory-Dump auslesbar, wenn USART3 nicht genutzt wird.
 * Byte-Layout eines Eintrags (little-endian, 36 Byte) fuer das Decode-Skript:
 *   off 0  uint32 t_us
 *   off 4  float  x_soll_mm
 *   off 8  float  x_ist_mm
 *   off 12 float  e_mm
 *   off 16 float  v_cmd_mms
 *   off 20 int32  vactual
 *   off 24 int32  step_cnt      (Mikroschritte)
 *   off 28 uint16 enc_raw       (0..4095, 0xFFFF = Lesefehler)
 *   off 30 uint16 sg_result     (0..1023, 0xFFFF = Lesefehler)
 *   off 32 uint8  state
 *   off 33 uint8  i_run_akt
 *   off 34 (2 Byte Padding)
 */
#ifndef DATALOG_H
#define DATALOG_H

#include "stm32f7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* Ein Logeintrag. Reihenfolge so gewaehlt, dass die natuerliche Ausrichtung
 * genau 36 Byte ergibt (kein packed -> schnelle Zugriffe im 2-ms-Takt). */
typedef struct {
    uint32_t t_us;        /* Zeit seit Reset, Mikrosekunden (DWT)             */
    float    x_soll_mm;   /* Sollposition                                     */
    float    x_ist_mm;    /* Istposition (aus enc_raw umgerechnet)            */
    float    e_mm;        /* Regelabweichung, wie tatsaechlich gerechnet      */
    float    v_cmd_mms;   /* Reglerausgang vor Umrechnung                     */
    int32_t  vactual;     /* was ins VACTUAL-Register geschrieben wurde       */
    int32_t  step_cnt;    /* absolute Position in Mikroschritten              */
    uint16_t enc_raw;     /* AS5600 RAW_ANGLE 0..4095 (0xFFFF = Fehler)       */
    uint16_t sg_result;   /* TMC2209 SG_RESULT 0..1023 (0xFFFF = Fehler)      */
    uint8_t  state;       /* Zustand der Ablaufsteuerung                      */
    uint8_t  i_run_akt;   /* aktuell wirksamer IRUN                           */
} LogRecord;

/* USART3-VCP + DWT-Zeitbasis + USER-Button PC13 einrichten. Einmal in main(),
 * nach den MX_*-Inits, vor dem Scheduler. Startet das Logging (aktiv). */
void Datalog_Init(void);

/* Puffer leeren und Zeit-Null neu setzen; Logging danach aktiv. */
void Datalog_Reset(void);

/* Mikrosekunden seit dem letzten Datalog_Reset(). */
uint32_t Datalog_TimestampUs(void);

/* true, solange aufgezeichnet wird (false waehrend/nach dem Dump). Der
 * Reglertask soll bei false nicht abtasten. */
bool Datalog_IsActive(void);

/* Einen Eintrag ablegen (nicht blockierend, single-producer). t_us wird hier
 * NICHT gesetzt -- der Aufrufer setzt rec->t_us aus Datalog_TimestampUs(). */
void Datalog_Sample(const LogRecord *rec);

/* Entprellte steigende Flanke des USER-Buttons PC13. In ~20-ms-Takt pollen. */
bool Datalog_ButtonPressed(void);

/* Logging stoppen und den GESAMTEN Puffer chronologisch als CSV ueber USART3
 * ausgeben (blockierend -- nur nach dem Versuch aufrufen). */
void Datalog_Dump(void);

#endif /* DATALOG_H */
