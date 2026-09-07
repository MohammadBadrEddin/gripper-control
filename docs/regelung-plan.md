# Regelungsplan — Gripper-Control (STM32H753ZI + TMC2209 + AS5600)

> Briefing zum Einfügen in Claude Code. Ziel: aus dem vorhandenen Open-Loop-Stand
> eine geschlossene Regelung planen und implementieren. Erst planen, dann bauen.
> Terse, technisch, deutsch. Annahmen offenlegen, nichts schönreden.

---

## 1. Ist-Zustand der Firmware (Fakten aus dem Code, nicht Wunsch)

- **Toolchain:** STM32CubeIDE + HAL, FreeRTOS V10.3.1, Tick 1 kHz (`configTICK_RATE_HZ=1000`).
- **MCU-Takt:** SYSCLK 480 MHz (PLLN=120, HSE), VOS0.
- **Aktor:** TMC2209 über **UART-`VACTUAL`** (interner Schrittgenerator), **kein Step/Dir**.
  - Ansteuerung: USART2 **Half-Duplex, single-wire** = PDN_UART, blockierend (`HAL_UART_Transmit/Receive`, 20 ms Timeout).
  - Modus: StealthChop (SPREAD=GND), `intpol` an, `I_scale_analog` an → **VREF skaliert weiterhin den Strom**.
  - Microsteps per Register auf **16** gesetzt → **3200 µsteps/Umdrehung** (nicht 1600).
  - Strom: IRUN=16/32, IHOLD=8/32 (bewusst niedrig für Bringup).
- **Sensor:** AS5600 über I²C1 (`hi2c1`), 12 bit, **single-turn absolut** (0–4095), blockierend, 10 ms Timeout.
  Treiber liefert Rohwert, Grad, Magnet-OK. **Kein Multiturn, keine Filterung, kein Sampling-Task.**
- **Aktueller Motor-Task:** `StartStepperTestTask` fährt open-loop `VACTUAL=±10000` mit `vTaskDelay`.
  Das ist eine **Demo und muss für die Regelung ersetzt/deaktiviert werden.**
- **Am Board verifiziert:** Motor lief mit genau diesem Code → `TMC2209_Init` liefert `true`,
  `PDN_UART` ist an USART2 verdrahtet, Half-Duplex + `VACTUAL` funktionieren. Aktor-Pfad steht.
  **Encoder-Pfad (AS5600) noch ungetestet** — wird von diesem Code nicht angefasst.
- **DIAG:** PE15 als EXTI rising, NVIC-Prio 5 (= `configMAX_SYSCALL_INTERRUPT_PRIORITY`), Handler leer.
- **Pinbelegung:** STEP=PE9, DIR=PE11, EN=PE13 (active-low), DIAG=PE15, LED=PB0.
  STEP/DIR liegen brach — bei UART-Modus egal.

---

## 2. Regelgröße — die zentrale Entscheidung (MUSS geklärt werden)

Der Aktor ist **geschwindigkeitskommandiert** (`VACTUAL` = Sollgeschwindigkeit). Der TMC integriert
diese Geschwindigkeit intern zu Schritten. Die Strecke von *Geschwindigkeitskommando → Position*
ist also ein **reiner Integrator**.

**Empfehlung:** **Positionsregelung** (Gripper = Position/Winkel), realisiert als
- **P- oder PI-Positionsregler**, dessen Stellgröße die **Geschwindigkeit `VACTUAL`** ist.
- Weil die Strecke ein Integrator ist, ist schon ein **reiner P-Regler stabil** (Verhalten erster Ordnung).
  I-Anteil nur gegen stationären Fehler unter Last, mit Anti-Windup und Geschwindigkeitsbegrenzung.

Offen (bitte im Plan beantworten):
- [ ] **Positions-** oder **Geschwindigkeitsregelung**? (Default: Position)
- [ ] Soll später ein **Rampen-/Trajektoriengenerator** die Sollposition vorgeben (Trapez in Geschwindigkeit)? Für „definiert um n Schritte drehen mit Rampe" ja.
- [ ] Führt der AS5600 die **Motorwelle direkt** (1:1) oder sitzt Getriebe/Übersetzung dazwischen? (Encoder-zu-Motor-Verhältnis)
- [ ] Benötigter **Verfahrbereich**: innerhalb einer Umdrehung (single-turn reicht) oder mehr → **Multiturn-Unwrap** nötig?

---

## 3. Konstanten & Einheitenumrechnung (vorab gerechnet, im Code als `#define`/`static const`)

**VACTUAL → Geschwindigkeit** (Datenblatt 14.1):
`v[µsteps/s] = VACTUAL · fCLK / 2^24`. Interner Oszillator **fCLK ≈ 12 MHz** ⇒
`v[µsteps/s] = VACTUAL · 0,7153`.

Mit 3200 µsteps/Umdrehung:
- `1 U/s (360°/s)  = 3200 µsteps/s  → VACTUAL ≈ 4476`
- `VACTUAL = 10000 → 7153 µsteps/s ≈ 2,235 U/s ≈ 134 U/min`
- Umrechnung Stellgröße: **`VACTUAL ≈ ω[°/s] · 12,42`**  bzw. `VACTUAL ≈ (U/s) · 4476`

**AS5600:** 4096 counts/Umdrehung ⇒ **0,08789 °/count**. Umgekehrt `counts = °/0,08789`.

> ⚠️ **fCLK-Toleranz:** Der interne 12-MHz-Oszillator hat mehrere % Toleranz → die absolute
> Geschwindigkeitsskala ist ungenau. Für **Positionsregelung mit Encoder-Rückführung egal**
> (der Regler kompensiert), für offene Geschwindigkeitsvorgabe nicht. Nicht auf exakte `VACTUAL`-Drehzahl verlassen.

**Vorzeichen/Richtung:** Positives `VACTUAL` ↔ steigender/fallender Encoder-count ist **unbekannt**.
Muss experimentell bestimmt und als Vorzeichenfaktor `enc_dir ∈ {+1,-1}` in die Fehlerbildung.

---

## 4. Timing-Budget (harte Randbedingung für die Abtastrate)

Alle Bus-Zugriffe sind **blockierend** in der HAL:
- AS5600-Read: I²C `Mem_Read` 2 Byte @ Bus-Timing `0x00B03FDB` → grob **0,2–0,5 ms**.
- `VACTUAL`-Write: UART 8 Byte @ 115200 Bd → **~0,7 ms** reine Sendezeit.

⇒ Eine 1-kHz-Schleife (1 ms Budget) ist mit blockierenden Calls **knapp bis unmöglich**.
**Empfohlene Reglerrate: 200–500 Hz** (2–5 ms), Task per `vTaskDelayUntil` mit festem Intervall
(nicht `vTaskDelay` — sonst driftet die Periode um die Ausführungszeit).

Optionaler Ausbau (später, nicht MVP): I²C und UART auf **DMA/IT** umstellen, dann sind höhere Raten möglich.
Bei DMA auf H7: **Cache-Kohärenz** und SRAM-Bereich beachten (D-Cache aktuell prüfen: `SCB_EnableDCache`?).

---

## 5. Vorgeschlagene Architektur

```
[Sollwert / Trajektorie]         (Task oder Kommando)
          │  θ_soll [°] oder counts
          ▼
   ┌──────────────────┐   e = θ_soll − θ_ist
   │  Positionsregler  │   Stellgröße: v_cmd [°/s]
   │  (P / PI + AW)    │────────────┐
   └──────────────────┘             │  clamp auf ±v_max
          ▲                          ▼
          │ θ_ist            VACTUAL = v_cmd · 12,42
   ┌──────────────┐          ┌───────────────┐
   │ Encoder-Read  │          │ TMC2209_Move  │
   │ + Unwrap      │          │ Velocity()    │
   └──────────────┘          └───────────────┘
          ▲                          │
          │ I²C                       │ UART VACTUAL
       [AS5600]  ◄──── Motorwelle ────► [TMC2209 → Motor]
```

**Ein Regel-Task** (`ControlTask`), fester Takt `vTaskDelayUntil`:
1. Encoder lesen (`AS5600_ReadRaw`), Fehler-/`0xFFFF`-Fall behandeln (letzten gültigen Wert halten, Zähler).
2. **Multiturn-Unwrap:** Δ zwischen aktuellem und letztem Rohwert, Wrap bei ±2048 korrigieren, in 32-bit-Positionsakku aufaddieren → `θ_ist` in counts.
3. Fehler `e = θ_soll − θ_ist`, `enc_dir` anwenden.
4. Regler rechnen (P bzw. PI), Stellgröße `v_cmd` in °/s.
5. Anti-Windup (nur PI): I-Anteil nur integrieren, wenn Stellgröße nicht in Sättigung (Clamping/Back-Calculation).
6. `v_cmd` auf `±v_max` begrenzen, in `VACTUAL` umrechnen, per `TMC2209_MoveVelocity` senden.
7. Bei Zielankunft (|e| < Toleranz und |v| klein) optional `VACTUAL=0` / Halten über IHOLD.

**Trajektorie (falls Punkt 2 = ja):** separater, langsamer Sollwertgeber, der `θ_soll` als
Trapezprofil (a_max, v_max) hochrampt. Hält den Positionsregler-Eingang stetig → keine Sprünge.

---

## 6. Aufgabenverteilung FreeRTOS

- `ControlTask`: Prio **höher** als HeartBeat, z. B. 3. Stack min. 512 Words (float-Rechnung; FPU ist in `FreeRTOSConfig` mit `configENABLE_FPU=0` **deaktiviert** → prüfen/aktivieren, sonst SW-Float und teurer Kontextwechsel).
- Kein Zugriff auf `hi2c1`/`huart2` aus zwei Tasks gleichzeitig — Bus-Handles bleiben **taskexklusiv** oder per Mutex schützen.
- DIAG-EXTI: falls StallGuard genutzt wird, nur **`...FromISR`-APIs** verwenden (Prio 5 ist ok bzgl. `configMAX_SYSCALL_INTERRUPT_PRIORITY`). Für MVP DIAG ignorieren.

---

## 7. Sicherheit / Grenzen (vor dem ersten geschlossenen Lauf)

- **Strom bleibt niedrig** (IRUN 16/32) bis die Regelung stabil läuft; VREF vorher gemessen/gesetzt.
- **`v_max` konservativ** wählen (z. B. ≤ 1 U/s), damit ein Vorzeichenfehler nicht sofort weglaufen lässt.
- **Runaway-Schutz:** wenn |e| über Zeit **wächst** statt fällt → Vorzeichen `enc_dir` falsch → sofort `VACTUAL=0`, Fehlerzustand.
- **Magnet-Check** beim Start (`AS5600_MagnetOK`); ohne Magnet keine Freigabe der Regelung.
- **Encoder-Read-Fehler** (`0xFFFF`) über N Zyklen → Stopp.
- Motor **nie unter Spannung abziehen** (zerstört den Treiber) — Reihenfolge Enable/Disable im Code sauber halten.

---

## 8. Milestones (jeweils einzeln verifizieren)

1. **Encoder-Telemetrie:** ControlTask liest AS5600 mit fester Rate, gibt Rohwert/Grad über USART1 (Debug) aus. Magnet-OK prüfen. Noch **keine** Motoransteuerung.
2. **Richtung/Skala kalibrieren:** kleines festes `VACTUAL` fahren, Encoder-Änderung beobachten → `enc_dir` und counts-pro-`VACTUAL`-Sekunde verifizieren (Rechnung aus §3 gegenprüfen).
3. **Multiturn-Unwrap** testen: über Nulldurchgang (4095→0) drehen, Positionsakku muss stetig bleiben.
4. **P-Positionsregler**: kleiner Sollsprung innerhalb einer Umdrehung, `v_max` klein. Einschwingen ohne Überschwingen/Runaway.
5. **PI + Anti-Windup**: stationären Fehler unter Last eliminieren, Windup-Test mit blockierter Welle.
6. **Trajektorie**: „um n Schritte / n Grad mit Rampe", Richtungswechsel, Reproduzierbarkeit prüfen.
7. (später) **StallGuard/DIAG**, Strom hochsetzen, Grenzdrehzahl bestimmen.

---

## 9. Offene Punkte, die der Plan beantworten soll

- [ ] Regelgröße Position vs. Geschwindigkeit (Default Position)
- [ ] Encoder-zu-Motor-Übersetzung, Verfahrbereich (Multiturn ja/nein)
- [ ] Reglertyp P vs. PI, Startwerte Kp/Ki, Abtastrate (Vorschlag: 250 Hz)
- [ ] `configENABLE_FPU` aktivieren?
- [ ] Debug-Ausgabe über USART1 (printf/ITM) — vorhanden nutzen?
- [ ] Bleibt es bei blockierender HAL oder DMA/IT-Ausbau?
- [ ] Sollwertquelle: fest im Code, UART-Kommando, oder Trajektoriengenerator?

---

### Referenzdateien
- `Core/Src/main.c` — Init, Tasks, `StartStepperTestTask` (zu ersetzen)
- `Core/Src/tmc2209.c` / `Core/Inc/tmc2209.h` — `MoveVelocity`, `SetCurrent`, `SetMicrosteps`, Reg-R/W
- `Core/Src/as5600.c` / `Core/Inc/as5600.h` — `ReadRaw`, `ReadDeg`, `MagnetOK`
- `Core/Inc/FreeRTOSConfig.h` — Tick 1 kHz, FPU aus, Prioritäten
