#include "as5600.h"

static AS5600_Status ReadRegs(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *buf, uint16_t len)
{
    if (HAL_I2C_Master_Transmit(hi2c, AS5600_ADDR, &reg, 1, HAL_MAX_DELAY) != HAL_OK)
        return AS5600_ERROR;
    if (HAL_I2C_Master_Receive(hi2c, AS5600_ADDR, buf, len, HAL_MAX_DELAY) != HAL_OK)
        return AS5600_ERROR;
    return AS5600_OK;
}

AS5600_Status AS5600_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t status = 0;
    if (ReadRegs(hi2c, AS5600_REG_STATUS, &status, 1) != AS5600_OK)
        return AS5600_ERROR;
    if (!(status & AS5600_STATUS_MD_BIT))
        return AS5600_ERROR; // no magnet detected
    return AS5600_OK;
}

AS5600_Status AS5600_ReadData(I2C_HandleTypeDef *hi2c, AS5600_Data *data)
{
    uint8_t raw[2];
    if (ReadRegs(hi2c, AS5600_REG_RAW_ANGLE, raw, 2) != AS5600_OK)
        return AS5600_ERROR;

    /* RAW_ANGLE: high byte bits[3:0] then low byte bits[7:0] */
    data->raw_angle = ((uint16_t)(raw[0] & 0x0F) << 8) | raw[1];
    data->angle_deg = (data->raw_angle * 360.0f) / 4096.0f;
    return AS5600_OK;
}
