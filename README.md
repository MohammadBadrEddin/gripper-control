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

The `.ioc` now targets `STM32H753ZIT6`. **This is more than an `.ioc` edit**: switching MCU
family in CubeMX regenerates `Drivers/` (STM32H7xx HAL/CMSIS instead of STM32F7xx), the startup
file, and the linker script. Open the updated `.ioc` in STM32CubeIDE, accept the migration prompt,
and let it regenerate before building — a binary built against the old F7 `Drivers/` will not run
correctly on H753 silicon even if it happens to flash without error. The three hand-written driver
files (`tmc2209.h`, `as5600.h`) already have their `#include` switched to `stm32h7xx_hal.h`.

Clock tree after migration: HSE 8 MHz -> PLL1 -> **SYSCLK 400 MHz**, VOS1 (no overdrive, so it
doesn't depend on confirming the silicon revision needed for 480 MHz/VOS0). HCLK/APB1-4 all at
200/100 MHz. Re-open Clock Configuration in CubeMX once to let it validate/refill the peripheral
kernel-clock fields for anything beyond what this project uses (ADC/USB/SAI etc. are untouched).

## Tools / firmware

- STM32CubeIDE
- STM32Cube FW_H7 firmware package (regenerate via CubeMX after the migration)
- FreeRTOS, CMSIS-RTOS **V1** interface

## Repo structure

```
Core/                generated code + drivers (TMC2209, AS5600, debug_log)
Drivers/             HAL/CMSIS (CubeMX generated)
Middlewares/          FreeRTOS (CubeMX generated)
docs/                 planning notes (regelung-plan.md, logging-signale.md)
<project>.ioc          CubeMX config
```
