# Analyse: Firmware für die Messdatenerfassung

**Branch:** `mainv1` (Stand 2026-09-08). **Nur Analyse — kein Code geändert.**
Reihenfolge folgt dem Auftrag (A/B/C), danach Synthese nach §5.

## Annahmen (offengelegt)

- TMC-`fCLK` = interner 12-MHz-Oszillator → VACTUAL-Skalierung 83,77 (aus Auftrag §6 übernommen, nicht neu geprüft).
- Linker sieht **512 KB RAM** als einen Block ab `0x20000000` (`STM32F767ZITX_FLASH.ld`).
- I²C1-Timing `0x20404768` → tatsächliche Busfrequenz **nicht verifiziert** (100 vs. 400 kHz offen, siehe A-Frage 7).

## Zentraler Widerspruch (Code vs. Auftrag)

Der Auftrag steht auf **VACTUAL/UART, 16 Mikroschritte**. `mainv1` macht das **nicht**:

- Aktor ist **STEP/DIR über TIM3** (`motor_control.c`, `MotorControl_*`), nicht VACTUAL. `TMC2209_MoveVelocity` (VACTUAL) existiert im Treiber, wird aber **nirgends genutzt**.
- Auflösung ist auf **Vollschritt** gesetzt (`MOTOR_MICROSTEPS = 1`), Auftrag verlangt **16**.
- Es gibt **keinen 2-ms-Reglertask**, keinen Positions- und keinen Effort-Regler. `MotorInitAndTestTask` ist eine Open-Loop-Demo (2 Umdrehungen vor/zurück).

Folge: Die Messdaten-Firmware ist im Kern **noch nicht vorhanden**; große Teile sind Neubau, nicht Anpassung. Das ist der bestimmende Aufwandstreiber.

---

## Auftrag A — Logging

### A.1 / A.2 Ist-Zustand Logging & Felder

**Kein Logging-Mechanismus vorhanden.** Kein Ringpuffer, kein CSV, kein Kopfblock. Die einzigen „Telemetrie"-Globals sind Debugger-Watches:

- `g_enc_present/magnet/raw/deg/errors/samples` (main.c) — nur für Expressions, kein Buffer.
- AS5600-Rohwert `g_enc_raw` (0…4095) existiert bereits → deckt Feld `enc_raw` ab.

Abgleich der 11 Soll-Felder mit dem Code:

| Feld | verfügbar? | Quelle im Ist-Code |
|------|-----------|--------------------|
| `t_ms` | teilweise | `xTaskGetTickCount()` (1 kHz Tick) vorhanden, aber kein Zeitstempel geloggt |
| `x_soll_mm` | nein | keine Sollwertquelle im Code |
| `x_ist_mm` | nein | Umrechnung `enc_raw→mm` fehlt |
| `e_mm` | nein | kein Regler |
| `v_cmd_mms` | nein | kein Regler |
| `vactual` | nein | VACTUAL-Pfad ungenutzt |
| `enc_raw` | **ja** | `AS5600_ReadRaw` / `g_enc_raw` |
| `step_cnt` (µSchritt) | nein | nur `s_stepsRemaining` (pro Move, kein absoluter Zähler) |
| `sg_result` | nein | Register 0x41 nicht definiert, nicht gelesen |
| `state` | nein | keine Ablaufsteuerung |
| `i_run_akt` | nein | nicht als Variable geführt (SetCurrent schreibt nur) |

**step_cnt-Nuance:** Im VACTUAL-Betrieb erzeugt die Firmware keine Schrittpulse, kennt die Mikroschrittposition also nicht direkt. Ehrliche Quelle: **kommandierte Geschwindigkeit integrieren** — `step_cnt += vactual · (fCLK/2^24) · T`. Alternativ TMC-Register `MSCNT` (0x6A), aber das ist nur 0…1023 innerhalb einer elektrischen Periode (wraps, nicht absolut) → ungeeignet als Wegzähler. Empfehlung: Integration der Stellgröße.

### A.3 Übertragung — Ist

- **USART1 nicht konfiguriert** (nur Vektor + weak-Handler in Startup). Laut Auftrag ohnehin „defekt".
- **USART2 ist belegt** — Eindraht-Half-Duplex zum TMC2209 (PD5). Kann **nicht** gleichzeitig Konsole sein.
- **ST-Link-VCP** läge auf **USART3 (PD8/PD9)** — **nicht konfiguriert**.
- Kein CSV-Export, kein Dump-Kommando.

**Zwei realistische Auslesewege (Auftrag will beide bewertet):**

1. **UART-Dump über USART3/VCP** (neu konfigurieren). Vorteil: kabelloser Standard-COM über ST-Link, kein Debugger nötig, Skript liest CSV direkt. Aufwand: USART3 in CubeMX/Code anlegen + Dump-Routine. Risiko: gering, Pins frei.
2. **ST-Link-Memory-Dump** (SWD) des Ringpuffer-Arrays nach dem Versuch. Vorteil: **keine** UART nötig, null Laufzeit-Einfluss. Nachteil: manueller Schritt in CubeIDE/`st-info`/`pyocd`, Rohbytes → externes Skript muss CSV bauen. Guter Rückfall.

**Bewertung:** USART3-VCP ist der bessere Dauerweg (CSV direkt, wiederholbar). Memory-Dump als sofort verfügbarer Tag-1-Rückfall, weil er ohne funktionierende UART auskommt. Empfehlung: **beides** — Buffer als statisches, im Map-File auffindbares Symbol (Dump-fähig) **und** eine USART3-Dumproutine.

### A.4 Kopfblock — Ist

Nicht vorhanden. Firmware-bekannte Werte (IRUN, IHOLD, microsteps, T_regler, KP, KV, v_max, a_max, deadband, SG_SOLL) sind als `#define`/Init-Konstanten teils vorhanden (SetCurrent 16/8, TPOWERDOWN 20), aber nirgends ausgegeben. Von-Hand-Werte (VREF, R_SENSE, VM) existieren nirgends → müssen als Konstanten **oder** per UART-Kommando setzbar werden (derzeit kein UART-Kommandoparser vorhanden).

### A.5 Die sieben Fragen

1. **Logging vorhanden?** Nein (s. A.1).
2. **Reglertask mit fester Periode via `vTaskDelayUntil`?** Nein — es gibt keinen Reglertask. Zusätzlich ist **`INCLUDE_vTaskDelayUntil = 0`** (FreeRTOSConfig.h:93), d. h. die Funktion ist **nicht einkompiliert**. Muss auf 1 gesetzt werden. Tick = 1 kHz → 2-ms-Periode = 2 Ticks, sauber teilbar.
3. **Schrittfrequenz zur Laufzeit stellbar, ohne den Takt zu stören?** Im VACTUAL-Betrieb **ja** — ein einzelner Registerschreib (0x22) pro Zyklus ändert die Drehzahl stoßfrei, der interne Rampengenerator ist ohnehin aus. (Der aktuelle STEP/DIR-Pfad ändert dagegen ARR in der TIM3-ISR und ist dafür ungeeignet — spricht zusätzlich für den Wechsel auf VACTUAL.)
4. **Mikroschrittzähler geführt?** Nein. Herkunft: Integration der Stellgröße (s. A.2).
5. **`configENABLE_FPU` aktiv?** **Nein (=0)** — kritisch. Regler und Encoder-Task rechnen `float`; bei zwei FPU-nutzenden Tasks sichert der M7-Port ohne `configENABLE_FPU=1` die FPU-Register **nicht** über den Kontextwechsel → stille Zahlenfehler/Fault. **Umstellung: `configENABLE_FPU 1`.** Kosten: minimal — je FPU-Task ~+128 B Stack, FPU-Kontext im Port aktiv. Geringer Aufwand, hoher Nutzen.
6. **Freier RAM für Ringpuffer?** Reichlich. MCU-RAM **512 KB**, FreeRTOS-Heap nur 15 KB, newlib Heap/Stack 0x200/0x400. 180 KB als **statisches Array** (nicht FreeRTOS-Heap, nicht malloc) passen locker. Empfehlung: `static` Array in `.bss`, damit im Map-File adressierbar (Memory-Dump).
7. **Zeitbudget im 2-ms-Task?** Bei **500 kBd** grob: VACTUAL-Write (8 B) ~160 µs, SG-Read (4+8 B + Turnaround) ~300–400 µs, I²C RAW-Read ~100–200 µs (busfrequenzabhängig, **offen**), Rechnen (float) < 50 µs, Logging (memcpy ~36 B) vernachlässigbar. Summe ~0,7–1,0 ms → **passt in 2 ms**, aber: alle HAL-Aufrufe sind **blockierend** mit 20-ms-Timeout und 50-ms-Mutex-Timeout (`tmc2209.c`) — bei einem einzigen Timeout platzt der Takt. Jitter/Robustheit sind das eigentliche Risiko, nicht der Mittelwert.

---

## Auftrag B — Analoge Größen

### B.1 ADC-Hypothese

**Bestätigt.** `HAL_ADC_MODULE_ENABLED` ist auskommentiert (`stm32f7xx_hal_conf.h:41`), kein `ADC_HandleTypeDef`, keine `MX_ADCx_Init`, nur der weak `ADC_IRQHandler` im Startup. Der STM32 misst **keine** Spannung. Alle Messgrößen kommen digital: AS5600 über I²C, TMC2209 über UART. Wandlung passiert in den Sensorchips.

### B.2 VM-Überwachung per ADC — lohnt sich?

Technisch billig (ein ADC-Kanal + Spannungsteiler auf VM). Nutzen laut Auftrag real: VM-Einbruch verschiebt die SG-Kennlinie. **Bewertung:** Für den **MVP nicht nötig** — eine einmalige VM-Notiz im Kopfblock reicht, solange ein stabiles Labornetzteil versorgt. Sinnvoll erst, wenn SG-Kalibrierung quantitativ gegen VM abgesichert werden soll oder die Versorgung nicht steif ist. Einordnung: **optionaler späterer Zusatz**, kein Tag-1-Thema.

### B.3 Shunt-Momentmessung ausgeschlossen?

**Bestätigt.** Der Chopper prägt den eingestellten Phasenstrom lastunabhängig ein; ein Strommesswert sagt beim Schrittmotor nichts über das Moment. Lastinformation kommt aus **SG_RESULT** (Gegen-EMK-Auswertung im TMC), nicht aus einem Shunt. Kein Grund, Shunts einzuplanen.

### B.4 DIAG-Pin

**Verdrahtet, aber ungenutzt.** `TMC_DIAG = PE15`, als EXTI15_10 rising konfiguriert, `EXTI15_10_IRQHandler` ruft `HAL_GPIO_EXTI_IRQHandler(TMC_DIAG_Pin)` — aber **kein `HAL_GPIO_EXTI_Callback`** implementiert, der Interrupt verpufft. Für den MVP **entbehrlich** (bestätigt). Später nutzbar als StallGuard-Stall-Flag (DIAG geht bei SG-Unterschreitung high), dann Callback + Auswertung nötig.

---

## Auftrag C — Haltestrom-Befund

### Ist-Zustand

`TMC2209_Init` (`tmc2209.c`) schreibt fest: **IRUN=16, IHOLD=8, IHOLDDELAY=6, TPOWERDOWN=20**. Der Befund des Auftrags trifft den Code exakt.

1. **TPOWERDOWN aktuell = 20** → ≈ 0,44 s nach Stillstand fällt der Strom von IRUN(16) auf IHOLD(8). Damit tritt die im Auftrag beschriebene 8-%-Reserve real ein.
2. **IHOLD zur Laufzeit auf IRUN setzen?** **Ja, machbar** — `TMC2209_SetCurrent(&drv, 16, 16)` schreibt IHOLD_IRUN (0x10) live. Beim Wechsel in den Haltezustand `SetCurrent(&drv, IRUN, IRUN)` aufrufen, beim Losfahren zurück auf (IRUN, IHOLD). Setzt eine Ablaufsteuerung (`state`) voraus, die es noch nicht gibt.
3. **Thermische Kosten:** Halten mit IRUN statt IHOLD ≈ Verlustleistung ∝ I² → ~3,6× höhere Wicklungsverlustleistung im Stillstand (0,531² vs. 0,281²). Quantitativ nur mit R_SENSE/VREF/Motornennstrom bewertbar → **offen** (Kopfblock-Werte). Qualitativ: bei kurzen Greifzyklen unkritisch, bei Dauergriff Motor-/Treibererwärmung beobachten.
4. **Alternative „nicht herunterschalten":** TPOWERDOWN nur *verzögert*; „nie herunter" erreicht man am saubersten über **IHOLD = IRUN** (dann ist TPOWERDOWN egal). Nachteil: Dauerstrom auch in echten Ruhephasen. Deshalb ist die **laufzeitgesteuerte Umschaltung (Punkt 2)** die bessere Lösung — Haltestrom nur während des aktiven Griffs hoch.

**Zusatzrisiko (Effort-Regler):** SG_RESULT friert im Stillstand ein (Auftrag §6, bestätigt als Konzept). „Erfolg" darf daher nicht allein aus SG im Halt abgeleitet werden — relevant fürs `state`-Design, nicht für den Haltestrom selbst.

---

## §5 Synthese

### Vorgeschlagene Änderungen — nach Aufwand sortiert

| # | Änderung | Aufwand |
|---|----------|---------|
| 1 | `configENABLE_FPU = 1`, `INCLUDE_vTaskDelayUntil = 1` | XS (~0,5 h) |
| 2 | USART2-Baud 115200 → **500000** | XS (~0,25 h) |
| 3 | TMC-Treiber ergänzen: `SG_RESULT` (0x41) lesen, `MoveVelocity`/VACTUAL im Reglerpfad nutzen, `SetCurrent` für Hold-Umschaltung | S (~2–3 h) |
| 4 | **Auf VACTUAL/UART umstellen**, Mikroschritte 1 → 16; STEP/DIR-Pfad (TIM3) stilllegen/entfernen | M (~3–4 h) |
| 5 | Ringpuffer-Logging: statisches Array (~180 KB), 11-Feld-Struct, Kopfblock, CSV-Dump über **USART3-VCP** + Dump-fähiges Symbol für ST-Link | M (~4–6 h) |
| 6 | **2-ms-Reglertask** (`vTaskDelayUntil`) mit Positionsregler nach §7 inkl. Rate-Limiter, Deadband, Runaway-Schutz, step_cnt-Integration | M–L (~5–7 h) |
| 7 | Effort-Regler-Variante (SG_ist−SG_soll, untere Sat. 0, KV, SG_SOLL) + `state`-Ablaufsteuerung inkl. Hold-Strom-Umschaltung | M (~3–4 h) |
| 8 | Kopfblock-Werte per UART-Kommando setzbar (VREF/R_SENSE/VM) — Mini-Parser | S–M (~2–3 h) |
| 9 | *(optional, später)* VM-ADC-Überwachung, DIAG-Callback | S je |

Tag-1-Minimalpfad für verwertbare Messdaten: **1,2,3,4,5,6** (Positionsvariante + Logging). Effort (7) und Kopfblock-Kommando (8) danach.

### Risiken & Nebenwirkungen

- **FPU aktivieren** ändert das Kontext-Layout aller Tasks → Stacks der FPU-Tasks knapp prüfen (`configMINIMAL_STACK_SIZE` 256 Worte ist ok, aber Regler-Task großzügig dimensionieren).
- **VACTUAL-Umstellung** entfernt die getestete STEP/DIR-Bewegung (aktuell die einzige nachweislich laufende Funktion). Bis der VACTUAL-Pfad am Board bestätigt ist, gibt es keinen Fallback → beide Pfade evtl. übergangsweise per `#define` umschaltbar halten.
- **500 kBd Half-Duplex**: höhere Baudrate verschärft das Turnaround-Timing des Eindraht-Reads (der Echo-Fix von `mainv1` muss auch bei 500 k halten). Am Board mit Scope gegenprüfen.
- **Blockierende HAL-Aufrufe im 2-ms-Task**: ein UART-/I²C-Timeout sprengt den Takt und verfälscht die Zeitbasis. Timeouts klein halten oder auf IT/DMA gehen; Reglertask höchste Prio, aber TMC-Mutex-Konflikt mit anderen Tasks vermeiden.
- **Haltestrom hoch**: thermische Grenze Motor/Treiber; nur während aktivem Griff, nicht dauerhaft.
- **180 KB statisches Array**: reduziert freien RAM, aber unkritisch (512 KB). Nicht in DTCM-kritische Bereiche legen, wenn später DMA genutzt wird.

### Offene Fragen (ohne Hardware nicht entscheidbar)

- I²C1-Ist-Frequenz (100/400 kHz) → bestimmt A-Frage-7-Budget endgültig.
- `enc_dir`-Vorzeichen (Auftrag §6) → Runaway-Schutz zwingend vor erstem geschlossenen Lauf.
- R_SENSE/VREF/VM/Motornennstrom → nötig für thermische Bewertung des Haltestroms und für den Kopfblock.
- Ausleseweg final: USART3-VCP verfügbar/erwünscht, oder reicht Tag 1 der reine ST-Link-Dump?
- Reale TMC-UART-Verlässlichkeit bei 500 kBd (Scope).

### Aufwandsschätzung (Summe)

- **MVP Positionsvariante + Logging (1–6):** ~15–20 h.
- **+ Effort + Kopfblock-Kommando (7,8):** ~5–7 h.
- **Optional (9):** ~1–2 h.

**Kein Code in diesem Schritt.** Umsetzung nach Freigabe in der oben nummerierten Reihenfolge (bzw. abgestimmt).
