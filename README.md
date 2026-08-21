# gripper-control

FreeRTOS-based position and torque control firmware for a stepper-driven two-finger gripper.

## Hardware

- MCU: STM32H753ZI (NUCLEO-H753ZI)
- Motor: NEMA17 stepper (17HE12-1204S)
- Motor driver: TMC2209 
- Position feedback: AS5600 magnetic encoder (I2C)

## Tools / firmware

- STM32CubeIDE
- STM32Cube FW_H7 firmware package **V1.12.1**
- FreeRTOS, CMSIS-RTOS **V1** interface

## Repo structure

```
Core/                generated code + drivers (TMC2209, AS5600)
Drivers/             HAL/CMSIS (CubeMX generated)
Middlewares/          FreeRTOS (CubeMX generated)
<project>.ioc          CubeMX config
```
