/**
 * @file    as5600.h
 * @brief   Minimal AS5600 magnetic encoder driver (blocking I2C).
 *
 * Bare-minimum version: init, magnet check, read angle. No filtering, no
 * multi-turn tracking, no interrupts, no RTOS dependency.
 *
 * Usage:
 *     AS5600 enc;
 *     AS5600_Init(&enc, &hi2c1);
 *     if (AS5600_MagnetOK(&enc)) {
 *         uint16_t raw = AS5600_ReadRaw(&enc);   // 0..4095
 *         float deg    = AS5600_ReadDeg(&enc);   // 0..360
 *     }
 */
#ifndef AS5600_H
#define AS5600_H

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* 7-bit address 0x36, shifted left for the HAL. */
#define AS5600_ADDR      (0x36 << 1)

/* Registers used here. */
#define AS5600_RAW_ANGLE 0x0C   /* 12-bit, high byte first */
#define AS5600_STATUS    0x0B
#define AS5600_MD_BIT    (1 << 5)   /* magnet detected */

typedef struct {
    I2C_HandleTypeDef *hi2c;
} AS5600;

/** Bind to an I2C bus. Returns true if the device responds. */
bool AS5600_Init(AS5600 *enc, I2C_HandleTypeDef *hi2c);

/** True if a magnet is detected. */
bool AS5600_MagnetOK(AS5600 *enc);

/** Raw angle 0..4095. Returns 0xFFFF on a read error. */
uint16_t AS5600_ReadRaw(AS5600 *enc);

/** Angle in degrees 0..360. Returns -1.0f on a read error. */
float AS5600_ReadDeg(AS5600 *enc);

#endif /* AS5600_H */
