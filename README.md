# gripper-control

FreeRTOS-based position and torque control firmware for a stepper-driven two-finger gripper.

## Hardware

- MCU: **STM32H753ZI** (migrated from STM32F767ZI — see note below)
- Motor: NEMA17 stepper (17HE12-1204S), driven **fullstep only** (MRES=8, 200 steps/rev)
- Motor driver: TMC2209 (STEP/DIR + UART for config/telemetry, single-wire half-duplex on USART2)
- Position feedback: AS5600 magnetic encoder (I2C1)
- Debug/telemetry UART: USART3 (PD8/PD9) over the on-board ST-LINK Virtual COM Port — no extra
  cable, open with TeraTerm et al. Alternative: USART1 (PA9/PA10) for an external USB-UART adapter.

## MCU migration note (F767ZI -> H753ZI)

The project has been fully migrated to STM32H753ZI — not just the `.ioc`, but every build
artifact that actually determines what runs on the chip:

- `Core/Startup/startup_stm32h753zitx.s` (replaces `startup_stm32f767zitx.s`)
- `STM32H753ZITX_FLASH.ld` / `STM32H753ZITX_RAM.ld` (replace the F767 linker scripts)
- `Drivers/CMSIS/Device/ST/STM32H7xx/` and `Drivers/STM32H7xx_HAL_Driver/` (replace the F7 trees)
- `Core/Src/system_stm32h7xx.c`, `Core/Src/stm32h7xx_it.c` / `Core/Inc/stm32h7xx_it.h`,
  `Core/Src/stm32h7xx_hal_msp.c` (replace their F7 counterparts)
- `.cproject` updated to reference the H753 device/linker/HAL names
- `Core/Inc/main.h` / `Core/Src/main.c`: `SystemClock_Config()` rewritten for H753's clock tree,
  TIM3 prescaler recomputed, USART1/USART3 init added

CMSIS/HAL sources were taken verbatim from ST's own `cmsis_device_h7` / `stm32h7xx_hal_driver`
GitHub repositories, so no CubeMX regeneration step is required to get a buildable tree — just
open the project in STM32CubeIDE and build. **This has been reviewed but not compiler-verified**
(no ARM GCC toolchain was available to test-build in the environment this migration was done in)
— do a clean build before flashing.

Clock tree: HSE 8 MHz -> PLL1 -> **SYSCLK 400 MHz**, VOS1 (no overdrive, so it doesn't depend on
confirming the silicon revision needed for 480 MHz/VOS0). HCLK 200 MHz, APB1-4 all 100 MHz.

### Two items that still need your input

1. **I2C1 timing (`hi2c1.Init.Timing` in `MX_I2C1_Init()`, `main.c`)** — still set to the old
   F767 value (`0x20404768`), computed for a 54 MHz I2C1 kernel clock. On H753, I2C1's kernel
   clock is 100 MHz (`RCC_I2C1235CLKSOURCE_D2PCLK1`), so this value is almost certainly wrong.
   Recompute it with CubeMX's I2C1 parameter view (Standard Mode, 100 kHz) and replace the
   line flagged with a loud comment in `MX_I2C1_Init()` (`main.c`) — deliberately not guessed
   here to avoid a wrong-but-plausible register value silently shipping.
2. **`HSEState = RCC_HSE_ON` (crystal) vs. `RCC_HSE_BYPASS` (external oscillator signal)** — kept
   as `RCC_HSE_ON` to match the original F767 config, since the `.ioc` has `board=custom` and the
   actual oscillator circuit on your board couldn't be confirmed. If your H753 board feeds HSE
   from an active oscillator/MCO rather than a crystal, switch to `RCC_HSE_BYPASS` in
   `SystemClock_Config()`.

## Debug / telemetry logging

Timestamped, non-blocking UART logging lives directly in `Core/Src/main.c` and
`Core/Inc/main.h` (`Debug_Init` / `Debug_Printf` / `Debug_TimestampMs` / `Debug_Drops`) —
deliberately not a separate file. Timestamp source is the DWT cycle counter (µs resolution, no
extra timer needed); transport is `HAL_UART_Transmit_IT` over a ring buffer, currently on
USART3 (ST-LINK VCP). Capture with TeraTerm's `Terminal -> Log...` to get a plain-text/CSV file
straight into MATLAB (`readtable`/`readmatrix`) — no extra PC tool needed. See
`docs/logging-signale.md` for the full signal list and integration details.

## Tools / firmware

- STM32CubeIDE
- STM32Cube FW_H7 firmware package
- FreeRTOS, CMSIS-RTOS **V1** interface

## Repo structure

```
Core/                 app code + drivers (TMC2209, AS5600), debug logging in main.c/main.h
Drivers/               HAL/CMSIS for STM32H7xx
Middlewares/          FreeRTOS
docs/                 planning notes (regelung-plan.md, logging-signale.md)
<project>.ioc          CubeMX config
```

Note: `.mxproject` still has some stale STM32F7xx bookkeeping entries left over from before the
migration. This is a CubeMX/CubeIDE-internal cache file, not something the GCC build reads (the
`.cproject` build config already points at the H7 sources), so STM32CubeIDE silently regenerates
it correctly the next time the project is opened/saved there — no action needed unless you want
it clean immediately.
