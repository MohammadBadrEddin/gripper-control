#ifndef TMC2209_H
#define TMC2209_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

// UART node address, set via MS1/MS2 pin strapping (0x00 if both are low)
#define TMC2209_UART_ADDR 0x00

// Register addresses (from TMC2209 datasheet)
#define TMC2209_REG_GCONF      0x00
#define TMC2209_REG_GSTAT      0x01
#define TMC2209_REG_IHOLD_IRUN 0x10
#define TMC2209_REG_CHOPCONF   0x6C
#define TMC2209_REG_DRV_STATUS 0x6F

// Config values (from datasheet register tables)
#define TMC2209_GCONF_VAL      0x00000040UL // pdn_disable=1, required for UART use
#define TMC2209_IHOLD_IRUN_VAL 0x00061408UL // IHOLD=8, IRUN=20, IHOLDDELAY=6
#define TMC2209_CHOPCONF_VAL   0x08000003UL // TOFF=3 (enabled), MRES=8 -> full step

typedef enum {
    TMC2209_OK    = 0,
    TMC2209_ERROR = 1
} TMC2209_Status;

typedef enum {
    TMC2209_DIR_CW  = 0,
    TMC2209_DIR_CCW = 1
} TMC2209_Direction;

typedef struct {
    UART_HandleTypeDef *huart;
    GPIO_TypeDef *step_port; uint16_t step_pin;
    GPIO_TypeDef *dir_port;  uint16_t dir_pin;
    GPIO_TypeDef *en_port;   uint16_t en_pin;
} TMC2209_HandleTypeDef;

TMC2209_Status TMC2209_Init(TMC2209_HandleTypeDef *dev, UART_HandleTypeDef *huart,
                             GPIO_TypeDef *step_port, uint16_t step_pin,
                             GPIO_TypeDef *dir_port,  uint16_t dir_pin,
                             GPIO_TypeDef *en_port,   uint16_t en_pin);

TMC2209_Status TMC2209_WriteReg(TMC2209_HandleTypeDef *dev, uint8_t reg, uint32_t val);
TMC2209_Status TMC2209_ReadReg(TMC2209_HandleTypeDef *dev, uint8_t reg, uint32_t *val);

void TMC2209_Enable(TMC2209_HandleTypeDef *dev);
void TMC2209_Disable(TMC2209_HandleTypeDef *dev);
void TMC2209_SetDirection(TMC2209_HandleTypeDef *dev, TMC2209_Direction dir);
void TMC2209_Step(TMC2209_HandleTypeDef *dev);
void TMC2209_MoveSteps(TMC2209_HandleTypeDef *dev, uint32_t steps, uint32_t step_delay_ms);

#endif // TMC2209_H
