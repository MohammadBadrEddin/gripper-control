# Datenlogging & Signale — Gripper-Control (STM32F767ZI)

> Antwort auf: Timestamp+Signale für MATLAB-Offline-Auswertung, lokales Speichern via
> TeraTerm, benötigte Signale für Reglerentwurf, Fullstep, StallGuard/Treiber-Features,
> UART-Konfiguration (VCP + Alternative). Terse, technisch, Annahmen offengelegt.

---

## 1. Fullstep — Status: bereits umgesetzt

`motor_control.h` setzt `MOTOR_MICROSTEPS = 1u` -> `TMC2209_SetMicrosteps(&s_drv, 1)` -> MRES=8
im CHOPCONF-Register = Vollschritt, 200 Schritte/Umdrehung. Antrieb läuft über STEP/DIR
(TIM3-ISR, Austin-Rampenalgorithmus), **nicht** mehr über UART-`VACTUAL` — das ist ein
Unterschied zum älteren `regelung-plan.md`-Stand, der noch VACTUAL-only beschreibt. USART2
(single-wire, TMC2209) wird dadurch nur noch für Konfiguration/Telemetrie gebraucht (Init,
`SetMicrosteps`, `SetCurrent`, StallGuard-Register), nicht für die Bewegung selbst — das
schafft Bandbreite für die neuen Register-Reads unten, ohne den Bewegungspfad zu stören.

`CHOPCONF.intpol` (Bit 28) bleibt auf 1: der Treiber interpoliert den Chopper-Waveform intern
weiterhin auf 256 µSteps für einen ruhigeren Stromverlauf, **auch bei MRES=8** — die
Positionsauflösung bleibt trotzdem exakt 200 Schritte/Umdrehung. Vollschritt heißt hier also
nicht "grobe Stromform", nur "grobe Positionsauflösung".

---

## 2. Lokal speichern — TeraTerm reicht, kein extra Tool nötig

**Einfachste Methode: TeraTerm selbst.** `Terminal → Log...` schreibt alles, was am COM-Port
ankommt, 1:1 in eine Textdatei mit. Wenn die Firmware Zeilen im CSV-Format sendet
(`t_ms,theta_soll,...\r\n`), ist die geloggte `.log`-Datei direkt eine CSV-Datei — in MATLAB
per `readtable("log.txt")` oder `readmatrix` einlesbar, ohne weiteres PC-Tool. Kein Skript,
kein zweites Programm, kein manuelles Copy-Paste aus dem Terminal.

Alternative, falls später mehr Durchsatz nötig wird als UART hergibt (siehe Bandbreitenrechnung
unten): SD-Karte lokal auf dem Board (SDMMC1, FatFs) — mehr Aufwand (Dateisystem-Stack), aber
kein Baudraten-Limit. Für dieses Projekt (Regler-Tuning, keine Rohdaten-Vielkanal-Erfassung)
ist UART+TeraTerm die pragmatische Wahl und ausreichend.

**Bandbreitenbudget:** eine CSV-Zeile mit ~8 Feldern liegt bei ~60–90 Byte. Bei 460800 Bd
(≈46 kB/s nutzbar) sind damit **problemlos 200–400 Zeilen/s** möglich — deckt die im
`regelung-plan.md` vorgeschlagene Reglerrate von 200–500 Hz ab, ohne Downsampling. Falls der
Regler-Task selbst schneller läuft als geloggt werden soll, einfach nur jeden n-ten Zyklus
loggen (Logging-Rate von der Regelrate entkoppeln, z. B. Regler 500 Hz / Log 200 Hz).

---

## 3. Welche Signale — für Reglerentwurf und Bericht (Kapitel 5 "Erfassung des Motorfeedbacks")

### Pflicht (Minimum für P/PI-Reglerauslegung und -verifikation)

| Signal | Quelle | Zweck |
|---|---|---|
| `t` (Timestamp) | DWT-Zykluszähler, µs | Zeitachse, Abtastrate verifizieren (Jitter aus `vTaskDelayUntil` sichtbar machen) |
| `theta_soll` | Trajektorie/Sollwertgeber | Führungsgröße |
| `theta_ist` | AS5600, unwrapped, in ° oder counts | Regelgröße (Position) |
| `e = theta_soll - theta_ist` | berechnet | Regelfehler — direkt für Einschwingverhalten/Überschwingen |
| `v_cmd` (Stellgröße) | Reglerausgang, °/s bzw. steps/s | zeigt Sättigung/Anti-Windup-Aktivität |
| `mode` | Statusvariable (Position/Drehmoment/Fehler) | für Umschaltlogik-Kapitel, Zustandswechsel im Plot sichtbar |

### Für den Drehmomentregler — **Achtung, Lücke in der aktuellen Hardware**

Die Aufgabenstellung verlangt explizit "Auswertung geeigneter Sensorik (Strommessung)"
(Thema 6, Punkt 2a). Der TMC2209 hat **keinen kalibrierten Stromsensor-Ausgang**:

- `CS_ACTUAL` (DRV_STATUS Bit 16–20) ist die **eingestellte** Stromstufe (IRUN oder ggf.
  internes Autoscale), **kein gemessener Ist-Strom in mA** — reflektiert die Sollvorgabe, nicht
  die Last.
- `SG_RESULT` (StallGuard4) ist ein **relativer** Lastindikator (10 bit, hoch = wenig Last,
  niedrig = nahe Stillstand/Blockade), **keine kalibrierte Drehmoment-/Kraftgröße** — nur nach
  Kalibrierung gegen bekannte Lasten (z. B. Referenzgewichte am Greifer) als grobe Proxy-Größe
  für die Greifkraft nutzbar, und nur oberhalb einer Mindestgeschwindigkeit zuverlässig
  (siehe unten).

**Empfehlung:** für den Laborbericht früh klären, ob (a) `SG_RESULT`, kalibriert, als indirekte
Kraftgröße ausreicht (Aufgabenstellung erlaubt ausdrücklich "indirekte... Regelung der
Greifkraft über Motorstrom/-drehmoment"), oder (b) ein echter Stromsensor (z. B. ACS712/INA219
im Motorstrompfad) ergänzt wird für eine belastbare `Strommessung`-Sektion im Bericht. (a) ist
schneller umsetzbar, (b) ist näher an dem, was die Aufgabenstellung wörtlich fordert. Beides ist
mit dieser Firmware kombinierbar.

### TMC2209-Diagnostik — jetzt im Treiber verfügbar (`tmc2209.h`, neu hinzugefügt)

| Signal | Register | Bedeutung |
|---|---|---|
| `sg_result` | SG_RESULT (0x41) | StallGuard-Lastwert, 0..1023 |
| `tstep` | TSTEP (0x12) | gemessene Zeit zwischen µSteps — unabhängiger Ist-Geschwindigkeits-Check gegen die Rampe |
| `cs_actual` | DRV_STATUS[20:16] | aktuelle Stromstufe |
| `stealth` | DRV_STATUS[30] | StealthChop (1) vs. SpreadCycle (0) — relevant, da StallGuard4 in beiden Modi funktioniert |
| `stst` | DRV_STATUS[31] | Standstill-Erkennung |
| `otpw`, `ot` | DRV_STATUS[0:1] | Übertemperatur-Vorwarnung / -Abschaltung — Pflichtsignal für den geforderten Überlastschutz |
| `s2ga/s2gb/s2vsa/s2vsb` | DRV_STATUS[2:5] | Kurzschluss-Flags (GND/Versorgung je Spule) — Fehlerzustand, sofort `VACTUAL=0`/EN=high |
| `ola/olb` | DRV_STATUS[6:7] | Open-Load — bei StealthChop/niedrigem Strom unzuverlässig, nur informativ loggen |
| GSTAT | GSTAT (0x01) | Reset-/Fehler-Latch seit letztem Clear — beim Task-Start einmal prüfen |

**StallGuard aktivieren** (in `MotorControl_Init` oder danach, mit den neuen Funktionen in
`tmc2209.h`):
```c
TMC2209_SetStallguardThreshold(&s_drv, sgthrs);   // 0..255, empirisch bestimmen
TMC2209_SetCoolStepThreshold(&s_drv, tcoolthrs);  // TSTEP-Schwelle, unterhalb aktiv
```
Ohne `TCOOLTHRS` ist `SG_RESULT` außerhalb des Geschwindigkeitsfensters nicht aussagekräftig
(siehe Doku-Kommentar im Header). Für den MVP reicht: Werte periodisch loggen (10–20 Hz reicht,
muss nicht mit der Regelrate mitlaufen), SGTHRS/TCOOLTHRS experimentell einstellen, bevor DIAG
(PE15, bereits als EXTI verdrahtet) als harter Blockade-Trigger genutzt wird.

### Optional / später

- `mscnt`, `mscuract` (0x6A/0x6B): tatsächliche µStep-Position und Spulenströme aus der
  Sinustabelle — Cross-Check gegen den Sollwert, eher fürs Debugging als für den Bericht.
- `enc_raw`, `enc_magnet` (schon vorhanden als `g_enc_*`-Globals) — Rohdaten vor Skalierung,
  falls Filterungs-/Skalierungs-Kapitel (5.2) Vorher/Nachher zeigen soll.

---

## 4. UART-Konfiguration (im `.ioc` umgesetzt)

| Peripherie | Pins | Zweck | Baud |
|---|---|---|---|
| **USART3** | PD8 (TX) / PD9 (RX) | **ST-LINK Virtual COM Port** — läuft über das ohnehin angeschlossene ST-LINK/USB-Kabel, kein Zusatzteil. Einfachste Option. | 460800 8N1 |
| **USART1** | PA9 (TX) / PA10 (RX) | Alternative: externer USB-UART-Adapter (FTDI/CP2102/CH340, 3.3 V!), z. B. wenn der ST-LINK-Port für etwas anderes gebraucht wird oder auf einem Custom-Board ohne VCP-Verdrahtung. | 460800 8N1 |
| USART2 | PD5, Half-Duplex | unverändert — TMC2209-Konfiguration/Telemetrie | 115200 |

Annahme: NUCLEO-F767ZI-Board (Nucleo-144-Formfaktor) mit HSE 8 MHz und USART3-VCP-Routing wie
beim offiziellen Board — falls es sich um ein Custom-PCB ohne diese Verdrahtung handelt, bitte
Rückmeldung, dann ist USART1 (externer Adapter) die primäre statt die Alternativ-Option.

Beide UARTs sind im `.ioc` mit eigenem NVIC-IRQ aktiviert (Prio 5, gleiche Ebene wie USART2/
TIM3/EXTI — sicher für `...FromISR`-Aufrufe unterhalb `configMAX_SYSCALL_INTERRUPT_PRIORITY`).
Bewusst **kein DMA** konfiguriert: das Logging sendet interrupt-getrieben
(`HAL_UART_Transmit_IT` + Ringpuffer) — keine DMAMUX-Kanalzuordnung nötig, unblockierend genug
für 200–500 Hz Logging, siehe Bandbreitenrechnung oben.

### Debug-/Logging-Funktionen: direkt in `main.c` / `main.h`

Auf ausdrücklichen Wunsch **keine eigene Datei** — das komplette Logging (Timestamp per
DWT-Zykluszähler, µs-Auflösung, kein Zusatztimer, + Ringpuffer + `HAL_UART_Transmit_IT`)
lebt in `Core/Src/main.c` (USER CODE BEGIN/END 4) und wird über `Core/Inc/main.h`
(USER CODE BEGIN/END EFP) exportiert. API:

```c
void     Debug_Init(UART_HandleTypeDef *huart);   // einmal, nach MX_USART3_UART_Init()
void     Debug_Printf(const char *fmt, ...);       // nicht-blockierend, printf-artig
uint32_t Debug_TimestampUs(void);                  // DWT-Zykluszähler in µs
float    Debug_TimestampMs(void);
uint32_t Debug_Drops(void);                        // kumulierte verworfene Zeilen (Puffer voll)
```

Aufruf in `main()` direkt nach der UART-Initialisierung:

```c
Debug_Init(&huart3);
Debug_Printf("\r\n# gripper-control boot, STM32F767ZI, SYSCLK=%lu Hz\r\n",
             (unsigned long)SystemCoreClock);
```

`HAL_UART_TxCpltCallback()` in `main.c` ist bereits so umgebaut, dass sie nach Instanz
unterscheidet: USART2 (TMC2209-Telemetrie, unverändert `control_UART_Tx_Flag++`) und
USART3/USART1 (Debug-Transport, treibt den Ringpuffer weiter) laufen nebeneinander, ohne
sich zu stören.

Aktuell aufgerufen aus `EncoderTestTask()` (10 Hz), Beispielzeile:

```c
Debug_Printf("%lu,enc,%u,%u,%u,%.2f,%lu\r\n",
             (unsigned long)Debug_TimestampMs(),
             g_enc_present, g_enc_magnet, g_enc_raw, g_enc_deg,
             (unsigned long)g_enc_errors);
```

Für den Regler-Task nach demselben Muster erweitern (siehe Signalliste oben), z. B. als
CSV-Header einmalig beim Boot und danach eine Zeile pro Regelzyklus.

### TMC2209-UART-Diagnose (`tmc`-Zeilen), ebenfalls in `EncoderTestTask()`, 10 Hz

Fragt periodisch `SG_RESULT`, `TSTEP` und `DRV_STATUS` über USART2 (PD5) ab und loggt
Erfolg/Wert jedes einzelnen Reads:

```c
Debug_Printf("%lu,tmc,%d,%u,%d,%lu,%d,%u,%u,%u,%u\r\n",
             (unsigned long)Debug_TimestampMs(),
             ok_sg, sg_result, ok_ts, (unsigned long)tstep,
             ok_st, st.cs_actual, st.otpw, st.ot, st.stealth);
```

Spalten: `t_ms, ok_sg, sg_result, ok_tstep, tstep, ok_status, cs_actual, otpw, ot, stealth`.
`ok_*` ist `0`, sobald ein Read timeoutet oder die CRC nicht passt — direkt gegen eine
gleichzeitige Logic-Analyzer-Aufnahme auf PD5 gegenprüfbar: kommen dort Antwort-Bytes an,
aber `ok_*` bleibt `0`, ist es ein Firmware-Parsing-Problem (CRC/Timing); kommt elektrisch
gar nichts zurück, ist es Verdrahtung/Pull-up. Zugriff auf den TMC2209-Handle über
`MotorControl_GetDriver()` (liefert `NULL`, solange `MotorControl_Init()` noch nicht
durchgelaufen ist).

`SGTHRS`/`TCOOLTHRS` werden in `MotorControl_Init()` (`motor_control.c`) mit
konservativen Startwerten gesetzt (SGTHRS=10, TCOOLTHRS=6000) — experimentell
nachjustieren, sobald reale `sg_result`-Werte unter Last vorliegen.

---

## 5. Referenz

- `Core/Inc/tmc2209.h` / `Core/Src/tmc2209.c` — StallGuard/DRV_STATUS/TSTEP-Funktionen neu
- `Core/Src/main.c` / `Core/Inc/main.h` — Debug-/Logging-Funktionen (`Debug_*`), siehe oben
- `docs/regelung-plan.md` — Reglerarchitektur, noch offene Punkte (P vs. PI, Encoder-Übersetzung)
- TMC2209-Datenblatt Rev. 1.09, Abschnitt 5 (Register), 14 (StallGuard/CoolStep)
