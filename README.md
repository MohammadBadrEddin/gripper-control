# gripper-control

FreeRTOS-based position and torque control firmware for a stepper-driven two-finger gripper.

## Hardware

- MCU: **STM32F767ZI** (NUCLEO-F767ZI)
- Motor: NEMA17 stepper (17HE12-1204S), driven **fullstep only** (MRES=8, 200 steps/rev)
- Motor driver: TMC2209 (STEP/DIR + UART for config/telemetry, single-wire half-duplex on USART2)
- Position feedback: AS5600 magnetic encoder (I2C1)
- Debug/telemetry UART: USART3 (PD8/PD9) over the on-board ST-LINK Virtual COM Port — no extra
  cable, open with TeraTerm et al. Alternative: USART1 (PA9/PA10) for an external USB-UART adapter.

## Debug / telemetry logging

Timestamped, non-blocking UART logging lives directly in `Core/Src/main.c` and
`Core/Inc/main.h` (`Debug_Init` / `Debug_Printf` / `Debug_TimestampMs` / `Debug_Drops`) —
deliberately not a separate file. Timestamp source is the DWT cycle counter (µs resolution, no
extra timer needed); transport is `HAL_UART_Transmit_IT` over a ring buffer, on USART3
(ST-LINK VCP, PD8/PD9), with USART1 (PA9/PA10) wired up as an alternate/external-adapter path.
Capture with TeraTerm's `Terminal -> Log...` to get a plain-text/CSV file straight into MATLAB
(`readtable`/`readmatrix`) — no extra PC tool needed. See `docs/logging-signale.md` for the
full signal list and integration details.

New peripherals added to the `.ioc` for this: USART1 (PA9/PA10, 460800 8N1) and USART3
(PD8/PD9, 460800 8N1), both full-duplex, own NVIC IRQ at priority 5 (same level as
USART2/TIM3/EXTI — safe for `...FromISR` calls below `configMAX_SYSCALL_INTERRUPT_PRIORITY`).
The existing clock tree (HSE 8 MHz -> PLL -> SYSCLK 216 MHz, Over-Drive enabled, VOS1,
FLASH_LATENCY_7) and TIM3 config (PSC=107 -> 1 MHz tick) are unchanged.

TMC2209 driver additions for capturing load/stall data alongside the logging: StallGuard4
threshold (`TMC2209_SetStallguardThreshold`), CoolStep threshold
(`TMC2209_SetCoolStepThreshold`), `SG_RESULT`/`TSTEP` readback, and `DRV_STATUS` decode
(`TMC2209_ReadStatus` — otpw/ot/short-to-ground/open-load/cs_actual/stealth/standstill bits).
See `Core/Inc/tmc2209.h`.

## Tools / firmware

- STM32CubeIDE
- STM32Cube FW_F7 firmware package
- FreeRTOS, CMSIS-RTOS **V1** interface

## Repo structure

```
Core/                 app code + drivers (TMC2209, AS5600), debug logging in main.c/main.h
Drivers/               HAL/CMSIS for STM32F7xx
Middlewares/          FreeRTOS
docs/                 planning notes (regelung-plan.md, logging-signale.md)
<project>.ioc          CubeMX config
```
