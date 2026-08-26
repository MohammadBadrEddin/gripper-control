#ifndef AS5600_H
#define AS5600_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

#define AS5600_ADDR (0x36 << 1) // 7-bit addr, HAL wants it 8-bit left-shifted

// Register addresses (from AS5600 datasheet)
#define AS5600_REG_STATUS    0x0B
#define AS5600_REG_RAW_ANGLE 0x0C
#define AS5600_REG_ANGLE     0x0E
#define AS5600_REG_AGC       0x1A
#define AS5600_REG_MAGNITUDE 0x1B

#define AS5600_STATUS_MD_BIT (1U << 5) // magnet detected

typedef enum {
    AS5600_OK    = 0,
    AS5600_ERROR = 1
} AS5600_Status;

typedef struct {
    uint16_t raw_angle; // 0..4095
    float    angle_deg; // 0..360
} AS5600_Data;

AS5600_Status AS5600_Init(I2C_HandleTypeDef *hi2c);
AS5600_Status AS5600_ReadData(I2C_HandleTypeDef *hi2c, AS5600_Data *data);

#endif // AS5600_H
