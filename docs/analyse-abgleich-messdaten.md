# Phase-1-Bericht: Abgleich Stand A/B + Messdatenerfassung

**Nur Analyse — kein Code geändert, nichts committet.** Aufbau nach §6 des Auftrags.
Ergänzt den früheren Bericht `docs/analyse-messdaten-firmware.md` (galt nur für Stand B).

- **Stand A** = `C:\Users\masou\Documents\SimReg\gripper-control` — Git auf `main @ 9532c74` **plus umfangreiche untracked-Dateien**.
- **Stand B** = dieses Repo, `SimReg_Porjekt\gripper-control` — Git auf `mainv1` (Tag `v1`), kohärent.

Beide sind Checkouts **desselben** GitHub-Repos.

## Kernbefund vorweg

**Stand A ist ein inkonsistenter, nicht baubarer Mid-Port.** Sein *committeter* Kern ist noch **H753-Zeit** (H7-`SystemClock_Config`, alte TMC-API mit `stm32h7xx_hal.h`), während **untracked** F767-Dateien und eine neuere `motor_control.c` danebengelegt wurden. Die untracked `motor_control.c` ruft TMC-Funktionen auf (`TMC2209_SetCoolStepThreshold`, `…SetStallguardThreshold`, neue `TMC2209`-API), die **in A's `tmc2209.h` gar nicht existieren** → **A kompiliert nicht**. Das Dokument `logging-signale.md` in A beschreibt zusätzlich Logging-/StallGuard-/VCP-Features, die **im Code nirgends stehen**.

**Stand B ist kohärent, baut, läuft** (Half-Duplex-TMC-Read-Fix verifiziert, Trapezprofil, Encoder-Task aktiv, getaggt `v1`).

**Empfehlung: B als Basis.** Aus A werden nur einzelne *Ideen/Codefragmente* übernommen (unten). A's alter TMC-Treiber, H7-`main.c` und die H7-Reste werden **verworfen**.

---

## 1. Ordnerabgleich

### 1.1 Generierte / auszunehmende Artefakte

Nicht bewertet (Build/IDE/vendored/CubeMX-generiert): `Debug/`, `.settings/`, `.metadata/`, `.cproject`, `.project`, `.mxproject`, `*.launch`, `Drivers/`, `Middlewares/`, `Core/Startup/*`, `Core/Src/system_stm32*.c`, `Core/Src/stm32*_it.c`, `Core/Src/stm32*_hal_msp.c`, `Core/Src/stm32*_hal_timebase_tim.c`. (Die MSP-/it-Dateien enthalten Pin-/IRQ-Config und werden dort erwähnt, wo relevant.)

### 1.2 Nur in einem Stand

| Datei | nur in | Bewertung |
|---|---|---|
| `stm32h7xx_*` (hal_conf, it.c/.h, hal_msp, timebase, system_h7), `startup_stm32h753zitx.s`, `STM32H753ZITX_*.ld`, `gripper-control.ioc` | **A** | **Tote H7-Reste** aus der Zeit vor dem F767-Port. Clutter, gehören gelöscht. Nicht übernehmen. |
| `SuR_Motor Debug.launch` | B | IDE-Artefakt, ignorieren. |
| `docs/logging-signale.md` | **A** | Zieldokument für Logging/Signale — **wertvoll als Design-Input**, aber nicht als Ist-Code (s. §3, §6). |
| `docs/analyse-messdaten-firmware.md` | B | Mein Vorbericht. |
| `.metadata/` | B | IDE-Workspace-Müll (bekannt). |

### 1.3 In beiden, inhaltlich verschieden — Bewertung

| Datei | Unterschied | Bessere Fassung |
|---|---|---|
| `Core/Src/tmc2209.c`, `Core/Inc/tmc2209.h` | **A:** alte API `TMC2209_HandleTypeDef`, `stm32h7xx_hal.h`, Read = `Transmit(4)+Receive(8)` **ohne** Half-Duplex-Umschalten, **ohne** CRC-Prüfung, `HAL_MAX_DELAY` (hängt ewig). **B:** neue `TMC2209`-API, F767, Half-Duplex mit **Read-Fix**, CRC-Validierung, Timeout+Mutex. | **B, eindeutig.** A's Read kann am Eindraht-Bus prinzipiell nicht funktionieren. |
| `Core/Src/main.c` | **A:** H7-Bring-up, leerer `tmc_task`, alte `TMC2209_MoveSteps`-Demo, USART1+2 @115200, kein Encoder/Regler/Logging. **B:** F767, `MotorInitAndTestTask` (Trapez) + `EncoderTestTask` aktiv. | **B, eindeutig.** |
| `Core/Src/motor_control.c`, `.h` | **A:** Trapezprofil **+ Extras**: `s_positionFullSteps`/`s_moveDirSign` → `MotorControl_GetPositionMicrosteps()`, `MotorControl_GetDriver()`, `s_driverReady`, `SetCoolStep/StallguardThreshold`-Aufrufe. **B:** nur Trapezprofil. | **A inhaltlich reicher** — aber referenziert TMC-Funktionen, die nur die **B**-Treiberfamilie hätte (und auch B noch nicht hat). → **A-Extras auf B portieren** (§2). |
| `Core/Inc/FreeRTOSConfig.h` | A: Kernel-Hdr V10.3.1, `IDLE_HOOK=0`, `MINIMAL_STACK=128`. B: V10.2.1-Hdr, `IDLE_HOOK=1` (+`__WFI`-Idle), `MINIMAL_STACK=256`. **Beide: `configENABLE_FPU=0`, `INCLUDE_vTaskDelayUntil=0`.** | Wash. B's Idle-Hook ist verdrahtet. **Beide** brauchen FPU=1 + delayUntil=1 (§4). |
| `Core/Src/as5600.c`, `.h`, `Core/Inc/main.h`, `Core/Src/freertos.c` | HAL-Include-Familie / Pin-Defs / Task-Hooks; A H7-nah, B F767-konsistent. | **B** (zum F767-Ziel passend). |
| `SuR_Motor_Remake.ioc` | A: enthält zusätzlich USART1 (PA9/PA10) und einen USART3-Handler-Stub. B: USART2 + I2C1 + TIM3. | Kein klarer Sieger — A hat UART-Ansätze fürs Logging, aber H7-verunreinigt. **Neu in CubeMX auf B** nachziehen (§2), nicht kopieren. |

### 1.4 Auseinanderlaufend ohne klaren Sieger (Entscheidung liegt bei dir)

- **FreeRTOSConfig-Detail** `MINIMAL_STACK` 128 (A) vs 256 (B) und `IDLE_HOOK` 0/1 — kosmetisch, deine Wahl.
- **.ioc-Peripherie-Set** (welche UARTs fürs Logging final): USART3-VCP (ST-Link, kein Zusatzkabel) vs USART1-Extern-Adapter. Beides in A's Doc vorgesehen; Auswahl ist eine HW-/Bequemlichkeitsfrage für dich.

---

## 2. Übernahmevorschlag (kein Patch, nur Text)

**Basis: Stand B (`mainv1`).** Begründung: kohärent, baut, TMC-Link verifiziert, getaggt. A liefert nur isolierte Bausteine.

| # | Quelle→Ziel | Was | Begründung | Risiko/Nebenwirkung | Aufwand |
|---|---|---|---|---|---|
| Ü1 | A→B | `s_positionFullSteps`/`s_moveDirSign` + `MotorControl_GetPositionMicrosteps()` | liefert `step_cnt` (Logfeld). Sauber portierbar, gleiche Trapez-Basis. | Zählt derzeit nur **Vollschritte** (STEP/DIR-Fullstep) → Auflösung 0,267 mm, zu grob fürs Lastwinkel-Kriterium (§3, §6). | XS (~0,5 h) |
| Ü2 | A→B | `MotorControl_GetDriver()` + `s_driverReady` | Handle-Zugriff für periodische Diagnose-Reads (SG/DRV_STATUS) aus anderer Task. | gering; Reentranz über bestehenden TMC-Mutex. | XS (~0,25 h) |
| Ü3 | A→B (Design) | `logging-signale.md` als Register-/Transport-Referenz (SG_RESULT 0x41, TSTEP 0x12, DRV_STATUS-Bits, VCP-Pins) | gute Vorarbeit; spart Datenblatt-Recherche. | **Achtung:** beschreibt **Live-Streaming**, Auftrag verlangt Post-Run-Ringpuffer → Methode **nicht** übernehmen, nur Registerwissen. | — (Lesehilfe) |
| Ü4 | A→B | UART-Transport fürs Auslesen (USART3-VCP) **in CubeMX neu** auf B anlegen | A's .ioc ist H7-verunreinigt → nicht kopieren, nachbauen. | Pin-Konflikte prüfen (PD8/9 frei). | S (~1 h) |
| — | verworfen | A: `tmc2209.c/.h` (alt/kaputt), `main.c` (H7), H7-Reste, beide .ld/ioc-Dubletten | schlechter/tot/nicht baubar | — | — |

Ziel: **ein** konsolidierter Stand auf B; A wird nach der Übernahme archiviert/gelöscht.

---

## 3. Ist-Zustand der Messdatenerfassung (konsolidiert über A+B)

**Kurz: die geforderte Messdatenerfassung ist in keinem der beiden Stände implementiert.** A hat ein *Design* + ein paar Haken (Positionszähler, Handle-Zugriff), aber kein Logging im Code.

### 3.1 Die 11 Logfelder

| Feld | Status | Quelle/Lücke |
|---|---|---|
| `t_ms` | fehlt (Baustein da) | Tick vorhanden; A-Doc plant DWT-µs-Timestamp (nicht im Code) |
| `x_soll_mm` | fehlt | kein Sollwertgeber/Regler |
| `x_ist_mm` | fehlt | Umrechnung enc→mm fehlt |
| `e_mm` | fehlt | kein Regler |
| `v_cmd_mms` | fehlt | kein Regler |
| `vactual` | fehlt | VACTUAL-Pfad ungenutzt (beide STEP/DIR) |
| `enc_raw` | **vorhanden** | `g_enc_raw`/`AS5600_ReadRaw` (beide) |
| `step_cnt` (µSchritt) | **teilw. in A** | A `GetPositionMicrosteps()` — aber nur **Vollschritt**-Granularität; B fehlt ganz |
| `sg_result` | fehlt | **keine** SG-Lesefunktion in A **oder** B (Doc behauptet sie, Code hat sie nicht) |
| `state` | fehlt | kein Zustandsautomat |
| `i_run_akt` | fehlt | IRUN nicht als Variable geführt |

### 3.2 Übertragung / Format / Kopfblock

- **Ringpuffer (Post-Run):** nicht vorhanden. A's Ansatz ist **Live-Streaming** per `Debug_Printf`/UART-IT — **widerspricht** §3.3 (verfälscht Zeitbasis). Nur als TX-Puffer gedacht, nicht als Messpuffer.
- **CSV:** nur als Idee in A's Doc, nicht im Code.
- **Auslesewege:** USART1 in A committet (PA9/PA10, defekt lt. Auftrag), USART3-VCP nur als Plan/Stub. B hat gar keinen Log-UART. ST-Link-Memory-Dump (Rückfall) in beiden nutzbar, sobald ein statischer Puffer existiert.
- **Kopfblock:** nirgends implementiert.

### 3.3 Bekannter Befund Haltestrom (§3.5) — **nicht adressiert**

Unverändert in beiden: **IRUN=16, IHOLD=8**. B setzt zusätzlich **TPOWERDOWN=20** (≈0,44 s → Herunterschalten auf IHOLD tritt real ein); A's alter Treiber setzt TPOWERDOWN gar nicht (Default). Die 8-%-Reserve besteht fort.
1. TPOWERDOWN aktuell = **20** (B) / ungesetzt (A-alt).
2. IHOLD→IRUN zur Laufzeit: **machbar** über `TMC2209_SetCurrent` (B-Treiber), braucht aber die fehlende `state`-Steuerung.
3. Thermik: quantitativ offen (R_SENSE/VREF/VM fehlen).
4. „Nie herunterschalten": am saubersten IHOLD=IRUN während des Griffs, nicht dauerhaft.

### 3.4 Die 10 Fragen (§3.6)

1. **Logging?** Nein (nur Design in A).
2. **Reglertask fixe Periode via `vTaskDelayUntil`?** Kein Reglertask; **`INCLUDE_vTaskDelayUntil=0` in BEIDEN** (nicht einkompiliert). Tick 1 kHz. Jitter später über DWT-Timestamp messbar (A-Doc-Idee).
3. **Schrittfrequenz laufzeitstellbar ohne Taktstörung?** Über VACTUAL wäre es 1 Registerschreib — VACTUAL aber ungenutzt. Aktueller STEP/DIR-Pfad ändert ARR in der ISR, dafür ungeeignet.
4. **Mikroschrittzähler?** A: `s_positionFullSteps`→`GetPositionMicrosteps()`, **aber Vollschritt-Auflösung**. B: keiner.
5. **`configENABLE_FPU`?** **0 in beiden** — Risiko (float in mehreren Tasks). Umstellung billig (§4).
6. **Freier RAM?** 512 KB gesamt, FreeRTOS-Heap 15 KB (beide) → 180-KB-Statik-Puffer passt locker.
7. **2-ms-Budget?** Bei 500 kBd machbar (~0,7–1 ms), Risiko sind blockierende HAL-Timeouts; Regler existiert noch nicht.
8. **UART 500 kBd?** **Nein** — USART2=115200 in beiden. A-Doc nennt 460800 für den VCP (nicht 500 k, und nicht der TMC-Bus).
9. **Runaway-Schutz (`enc_dir`)?** In keinem vorhanden.
10. **Zustandsautomat?** Nicht implementiert (A nennt `mode`/`state` nur im Doc).

### 3.5 Analoggrößen (§3.7)

- **ADC-Hypothese bestätigt:** in beiden ungenutzt (`HAL_ADC_MODULE_ENABLED` aus). Keine Spannungsmessung.
- **VM-Überwachung:** fürs MVP unnötig, Kopfblock-Notiz reicht.
- **Shunt-Moment:** ausgeschlossen — bestätigt (Chopper prägt Strom lastunabhängig).
- **DIAG-Pin:** in beiden als EXTI (PE15) verdrahtet, **ohne Callback** → ungenutzt, MVP-entbehrlich.

---

## 4. Vorgeschlagene Änderungen (nach Aufwand)

| # | Änderung | Aufwand |
|---|---|---|
| 1 | Konsolidieren auf **B**; A's H7-Reste/Dubletten löschen; Ü1/Ü2 aus A einpflegen | S (~1–2 h) |
| 2 | `configENABLE_FPU=1`, `INCLUDE_vTaskDelayUntil=1` | XS (~0,5 h) |
| 3 | TMC-Treiber erweitern: `SG_RESULT`(0x41), `TSTEP`(0x12), `DRV_STATUS`-Decode, `SGTHRS`/`TCOOLTHRS`-Setter — Voraussetzung für A's `motor_control`-Aufrufe **und** `sg_result`-Logging | S (~2–3 h) |
| 4 | Auf **VACTUAL/UART** umstellen + 16 Mikroschritte; STEP/DIR (TIM3) stilllegen; `step_cnt` per Stellgrößen-Integration (echte µSchritt-Auflösung) | M (~3–4 h) |
| 5 | USART2 auf **500 kBd**; Auslese-UART (USART3-VCP) neu in CubeMX (Ü4) | S (~1–1,5 h) |
| 6 | **RAM-Ringpuffer-Logging** (statisch ~180 KB, 11-Feld-Struct, Kopfblock, CSV-Dump nach Versuch über VCP + Dump-fähiges Symbol) — **Post-Run, kein Streaming** | M (~4–6 h) |
| 7 | **2-ms-Reglertask** (`vTaskDelayUntil`), Positionsregler nach §5, Rate-Limiter, Deadband, Runaway-Schutz | M–L (~5–7 h) |
| 8 | Effort-Regler + Zustandsautomat inkl. Haltestrom-Umschaltung IHOLD→IRUN | M (~3–4 h) |
| 9 | Kopfblock per UART-Kommando (VREF/R_SENSE/VM) | S–M (~2–3 h) |
| 10 | *(optional)* VM-ADC, DIAG-Callback | S je |

Tag-1-Minimalpfad: **1,2,3,4,5,6,7**.

---

## 5. Risiken & Nebenwirkungen

- **Ü1 step_cnt in Vollschritten** erfüllt das Lastwinkel-Kriterium (0,056 mm) **nicht**, solange STEP/DIR-Fullstep läuft — erst mit VACTUAL/16-µStep (Änderung 4) sinnvoll.
- **A's Logging-Methode (Streaming) übernehmen wäre ein Fehler** — verfälscht die Reglertakt-Zeitbasis (Auftrag §3.3). Nur Post-Run-Dump.
- **FPU aktivieren** ändert Task-Kontext → Stacks der FPU-Tasks prüfen.
- **VACTUAL-Umstellung** entfernt den einzigen nachweislich laufenden Bewegungspfad (STEP/DIR) → Übergangsweise per `#define` umschaltbar halten, bis VACTUAL am Board bestätigt.
- **500 kBd Half-Duplex** verschärft das Read-Turnaround; der Echo-Fix muss auch dort halten (Scope).
- **Blockierende HAL-Reads im 2-ms-Task** → ein Timeout sprengt den Takt. Timeouts klein / IT-DMA erwägen.
- **A weiterhin als Arbeitskopie** offen zu lassen ist riskant (baut nicht, driftet weiter). Nach Übernahme löschen/archivieren.

---

## 6. Widersprüche (Dokument ↔ Code)

1. **Auftrag §2 nennt „STM32H753ZI"** — der reale Ziel-/Baustand (B) ist **F767ZI**. Kurios: A's *committeter* Kern ist noch H753 (H7-Clock/Treiber), d. h. der Auftrag matcht A's **veralteten** Stand, nicht die Realität. **Code gilt: F767.**
2. **Auftrag §4: Aktor VACTUAL/UART** — beide Stände fahren **STEP/DIR** (A's Doc deklariert die Abweichung ausdrücklich).
3. **Auftrag §4: 500 kBd** — in keinem Stand gesetzt (115200); Prämisse (20 B/Zyklus) hängt am VACTUAL-Pfad.
4. **Auftrag §3.3: Post-Run-Ringpuffer, kein Streaming** — A's Logging-Design ist Live-Streaming.
5. **A-intern:** `logging-signale.md` behauptet SG/TSTEP/StallGuard in `tmc2209.h` und `Debug_*` in `main.c` — **existieren im Code nicht**. Zusätzlich ruft A's untracked `motor_control.c` TMC-Funktionen, die A's `tmc2209.h` nicht deklariert → **A baut nicht**.
6. **Auftrag §3.1: `state` 0…6 / `step_cnt` µSchritt** — kein Zustandsautomat; `step_cnt` (falls Ü1) nur Vollschritt.

---

## 7. Offene Fragen (ohne Hardware nicht entscheidbar)

- I²C1-Ist-Frequenz (A-Timing `0x00B03FDB` ≠ B `0x20404768`, unterschiedliche Clock-Domänen) → Budget/§3.4-Frage 7.
- `enc_dir`-Vorzeichen → Runaway-Schutz zwingend vor erstem geschlossenen Lauf.
- R_SENSE/VREF/VM/Motornennstrom → Haltestrom-Thermik + Kopfblock.
- Ausleseweg final: USART3-VCP oder USART1-Extern-Adapter? (A-Doc lässt beides offen; HW-Frage.)
- TMC-UART-Verlässlichkeit bei 500 kBd (Scope auf PD5).

---

## 8. Aufwandsschätzung (Summe)

- **Konsolidierung A→B + Grundschalter (Änd. 1–2):** ~2–3 h.
- **MVP Position + Logging (3–7):** ~15–22 h.
- **+ Effort/Zustandsautomat + Kopfblock (8,9):** ~5–7 h.
- **Optional (10):** ~1–2 h.

**Kein Code in Phase 1. Umsetzung erst nach deiner Freigabe, in abgestimmter Reihenfolge.**
