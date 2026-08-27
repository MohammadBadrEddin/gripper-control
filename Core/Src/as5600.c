#include "as5600.h"

#define TIMEOUT 10  /* ms per I2C transaction */

bool AS5600_Init(AS5600 *enc, I2C_HandleTypeDef *hi2c)
{
    enc->hi2c = hi2c;
    return (HAL_I2C_IsDeviceReady(hi2c, AS5600_ADDR, 2, TIMEOUT) == HAL_OK);
}

bool AS5600_MagnetOK(AS5600 *enc)
{
    uint8_t status;
    if (HAL_I2C_Mem_Read(enc->hi2c, AS5600_ADDR, AS5600_STATUS,
                         I2C_MEMADD_SIZE_8BIT, &status, 1, TIMEOUT) != HAL_OK) {
        return false;
    }
    return (status & AS5600_MD_BIT) != 0;
}

uint16_t AS5600_ReadRaw(AS5600 *enc)
{
    uint8_t buf[2];
    if (HAL_I2C_Mem_Read(enc->hi2c, AS5600_ADDR, AS5600_RAW_ANGLE,
                         I2C_MEMADD_SIZE_8BIT, buf, 2, TIMEOUT) != HAL_OK) {
        return 0xFFFF;   /* error */
    }
    /* high byte first, 12-bit value */
    return (uint16_t)(((buf[0] << 8) | buf[1]) & 0x0FFF);
}

float AS5600_ReadDeg(AS5600 *enc)
{
    uint16_t raw = AS5600_ReadRaw(enc);
    if (raw == 0xFFFF) {
        return -1.0f;
    }
    return raw * (360.0f / 4096.0f);
}
