#include "tmc2209.h"
#include "FreeRTOS.h"
#include "task.h"

/* CRC8 per TMC2209 datasheet UART datagram spec */
static uint8_t CRC8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        uint8_t b = data[i];
        for (uint8_t n = 0; n < 8; n++) {
            crc = ((crc >> 7) ^ (b & 0x01)) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
            b >>= 1;
        }
    }
    return crc;
}

TMC2209_Status TMC2209_WriteReg(TMC2209_HandleTypeDef *dev, uint8_t reg, uint32_t val)
{
    uint8_t frame[8];
    frame[0] = 0x05;
    frame[1] = TMC2209_UART_ADDR;
    frame[2] = reg | 0x80; // write access
    frame[3] = (uint8_t)(val >> 24);
    frame[4] = (uint8_t)(val >> 16);
    frame[5] = (uint8_t)(val >> 8);
    frame[6] = (uint8_t)(val);
    frame[7] = CRC8(frame, 7);

    if (HAL_UART_Transmit(dev->huart, frame, 8, HAL_MAX_DELAY) != HAL_OK)
        return TMC2209_ERROR;
    return TMC2209_OK;
}

TMC2209_Status TMC2209_ReadReg(TMC2209_HandleTypeDef *dev, uint8_t reg, uint32_t *val)
{
    uint8_t req[4], reply[8];
    req[0] = 0x05;
    req[1] = TMC2209_UART_ADDR;
    req[2] = reg & 0x7F; // read access
    req[3] = CRC8(req, 3);

    if (HAL_UART_Transmit(dev->huart, req, 4, HAL_MAX_DELAY) != HAL_OK)
        return TMC2209_ERROR;
    if (HAL_UART_Receive(dev->huart, reply, 8, HAL_MAX_DELAY) != HAL_OK)
        return TMC2209_ERROR;

    *val = ((uint32_t)reply[3] << 24) | ((uint32_t)reply[4] << 16)
         | ((uint32_t)reply[5] << 8)  | reply[6];
    return TMC2209_OK;
}

TMC2209_Status TMC2209_Init(TMC2209_HandleTypeDef *dev, UART_HandleTypeDef *huart,
                             GPIO_TypeDef *step_port, uint16_t step_pin,
                             GPIO_TypeDef *dir_port,  uint16_t dir_pin,
                             GPIO_TypeDef *en_port,   uint16_t en_pin)
{
    dev->huart      = huart;
    dev->step_port  = step_port;
    dev->step_pin   = step_pin;
    dev->dir_port   = dir_port;
    dev->dir_pin    = dir_pin;
    dev->en_port    = en_port;
    dev->en_pin     = en_pin;

    TMC2209_Disable(dev);
    HAL_GPIO_WritePin(dev->step_port, dev->step_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(dev->dir_port, dev->dir_pin, GPIO_PIN_RESET);

    if (TMC2209_WriteReg(dev, TMC2209_REG_GCONF, TMC2209_GCONF_VAL) != TMC2209_OK)
        return TMC2209_ERROR;
    if (TMC2209_WriteReg(dev, TMC2209_REG_CHOPCONF, TMC2209_CHOPCONF_VAL) != TMC2209_OK)
        return TMC2209_ERROR;
    if (TMC2209_WriteReg(dev, TMC2209_REG_IHOLD_IRUN, TMC2209_IHOLD_IRUN_VAL) != TMC2209_OK)
        return TMC2209_ERROR;

    return TMC2209_OK;
}

/* EN is active low */
void TMC2209_Enable(TMC2209_HandleTypeDef *dev)
{
    HAL_GPIO_WritePin(dev->en_port, dev->en_pin, GPIO_PIN_RESET);
}

void TMC2209_Disable(TMC2209_HandleTypeDef *dev)
{
    HAL_GPIO_WritePin(dev->en_port, dev->en_pin, GPIO_PIN_SET);
}

void TMC2209_SetDirection(TMC2209_HandleTypeDef *dev, TMC2209_Direction dir)
{
    HAL_GPIO_WritePin(dev->dir_port, dev->dir_pin, (GPIO_PinState)dir);
}

void TMC2209_Step(TMC2209_HandleTypeDef *dev)
{
    HAL_GPIO_WritePin(dev->step_port, dev->step_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(dev->step_port, dev->step_pin, GPIO_PIN_RESET);
}

void TMC2209_MoveSteps(TMC2209_HandleTypeDef *dev, uint32_t steps, uint32_t step_delay_ms)
{
    for (uint32_t i = 0; i < steps; i++) {
        TMC2209_Step(dev);
        vTaskDelay(pdMS_TO_TICKS(step_delay_ms));
    }
}
