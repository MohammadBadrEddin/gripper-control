/**
 * @file    as5600_driver.c
 * @brief   AS5600 low-level driver implementation (STM32 HAL).
 */
#include "as5600_driver.h"
#include <string.h>

/* ========================================================================== */
/* Internal helpers                                                           */
/* ========================================================================== */

/* Millisecond settle delay used only by the one-time OTP burn paths.
 * Under FreeRTOS this yields the CPU (vTaskDelay) instead of busy-waiting;
 * bare-metal falls back to HAL_Delay. Burn is never called from the control
 * loop, so this only matters for scheduler-friendliness during calibration. */
static void as5600_delay_ms(uint32_t ms)
{
#ifdef AS5600_USE_FREERTOS
    vTaskDelay(pdMS_TO_TICKS(ms == 0u ? 1u : ms));
#else
    HAL_Delay(ms);
#endif
}

/** Blocking register write of @p len bytes, with retry-once on failure. */
static HAL_StatusTypeDef reg_write(AS5600_Handle_t *h, uint8_t reg,
                                   const uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef st = HAL_ERROR;
    for (uint32_t attempt = 0; attempt <= AS5600_I2C_RETRIES; ++attempt) {
        st = HAL_I2C_Mem_Write(h->hi2c, h->addr_hal, reg,
                               I2C_MEMADD_SIZE_8BIT,
                               (uint8_t *)data, len, h->timeout_ms);
        if (st == HAL_OK) {
            break;
        }
    }
    return st;
}

/** Blocking register read of @p len bytes, with retry-once on failure. */
static HAL_StatusTypeDef reg_read(AS5600_Handle_t *h, uint8_t reg,
                                  uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef st = HAL_ERROR;
    for (uint32_t attempt = 0; attempt <= AS5600_I2C_RETRIES; ++attempt) {
        st = HAL_I2C_Mem_Read(h->hi2c, h->addr_hal, reg,
                              I2C_MEMADD_SIZE_8BIT,
                              data, len, h->timeout_ms);
        if (st == HAL_OK) {
            break;
        }
    }
    return st;
}

/** Read a 12-bit big-endian register pair starting at @p reg_high. */
static HAL_StatusTypeDef read_u12(AS5600_Handle_t *h, uint8_t reg_high,
                                  uint16_t *out12)
{
    uint8_t buf[2];
    HAL_StatusTypeDef st = reg_read(h, reg_high, buf, 2);
    if (st == HAL_OK) {
        *out12 = ((uint16_t)buf[0] << 8 | buf[1]) & AS5600_ANGLE_MASK;
    }
    return st;
}

/** Write a 12-bit value big-endian to a register pair starting at @p reg_high. */
static HAL_StatusTypeDef write_u12(AS5600_Handle_t *h, uint8_t reg_high,
                                   uint16_t val12)
{
    uint8_t buf[2];
    val12 &= AS5600_ANGLE_MASK;
    buf[0] = (uint8_t)(val12 >> 8);
    buf[1] = (uint8_t)(val12 & 0xFF);
    return reg_write(h, reg_high, buf, 2);
}

/** Read-modify-write a field inside the 16-bit CONF register. */
static HAL_StatusTypeDef conf_rmw(AS5600_Handle_t *h, uint16_t mask,
                                  uint16_t pos, uint16_t field)
{
    uint16_t conf;
    HAL_StatusTypeDef st = AS5600_ReadConf(h, &conf);
    if (st != HAL_OK) {
        return st;
    }
    conf = (uint16_t)((conf & ~mask) | ((field << pos) & mask));
    return AS5600_WriteConf(h, conf);
}

/* ========================================================================== */
/* Initialisation                                                             */
/* ========================================================================== */

HAL_StatusTypeDef AS5600_Init(AS5600_Handle_t *h, I2C_HandleTypeDef *hi2c)
{
    if (h == NULL || hi2c == NULL) {
        return HAL_ERROR;
    }
    memset(h, 0, sizeof(*h));
    h->hi2c       = hi2c;
    h->timeout_ms = AS5600_I2C_TIMEOUT_MS;
    h->addr_hal   = AS5600_I2C_ADDR_HAL;
    h->xfer_state = AS5600_XFER_IDLE;

    return AS5600_IsPresent(h);
}

HAL_StatusTypeDef AS5600_IsPresent(AS5600_Handle_t *h)
{
    return HAL_I2C_IsDeviceReady(h->hi2c, h->addr_hal, 2, h->timeout_ms);
}

/* ========================================================================== */
/* Blocking reads                                                             */
/* ========================================================================== */

HAL_StatusTypeDef AS5600_ReadRawAngle(AS5600_Handle_t *h, uint16_t *raw12)
{
    return read_u12(h, AS5600_REG_RAWANGLE_H, raw12);
}

HAL_StatusTypeDef AS5600_ReadAngle(AS5600_Handle_t *h, uint16_t *angle12)
{
    return read_u12(h, AS5600_REG_ANGLE_H, angle12);
}

HAL_StatusTypeDef AS5600_ReadStatus(AS5600_Handle_t *h, AS5600_Status_t *status)
{
    uint8_t s;
    HAL_StatusTypeDef st = reg_read(h, AS5600_REG_STATUS, &s, 1);
    if (st == HAL_OK) {
        status->magnet_detected   = (s & AS5600_STATUS_MD_Msk) != 0;
        status->magnet_too_weak   = (s & AS5600_STATUS_ML_Msk) != 0;
        status->magnet_too_strong = (s & AS5600_STATUS_MH_Msk) != 0;
    }
    return st;
}

HAL_StatusTypeDef AS5600_ReadAGC(AS5600_Handle_t *h, uint8_t *agc)
{
    return reg_read(h, AS5600_REG_AGC, agc, 1);
}

HAL_StatusTypeDef AS5600_ReadMagnitude(AS5600_Handle_t *h, uint16_t *mag12)
{
    return read_u12(h, AS5600_REG_MAGNITUDE_H, mag12);
}

HAL_StatusTypeDef AS5600_ReadZMCO(AS5600_Handle_t *h, uint8_t *zmco)
{
    uint8_t v;
    HAL_StatusTypeDef st = reg_read(h, AS5600_REG_ZMCO, &v, 1);
    if (st == HAL_OK) {
        *zmco = v & 0x03U;
    }
    return st;
}

/* ========================================================================== */
/* Interrupt-mode angle read                                                  */
/* ========================================================================== */

HAL_StatusTypeDef AS5600_ReadRawAngle_IT_Start(AS5600_Handle_t *h)
{
    if (h->xfer_state == AS5600_XFER_BUSY) {
        return HAL_BUSY;
    }
    h->xfer_state = AS5600_XFER_BUSY;
    HAL_StatusTypeDef st = HAL_I2C_Mem_Read_IT(h->hi2c, h->addr_hal,
                                               AS5600_REG_RAWANGLE_H,
                                               I2C_MEMADD_SIZE_8BIT,
                                               h->rx_buf, 2);
    if (st != HAL_OK) {
        h->xfer_state = AS5600_XFER_ERROR;
    }
    return st;
}

bool AS5600_ReadRawAngle_IT_Poll(AS5600_Handle_t *h, uint16_t *raw12)
{
    if (h->xfer_state == AS5600_XFER_DONE) {
        *raw12 = ((uint16_t)h->rx_buf[0] << 8 | h->rx_buf[1]) & AS5600_ANGLE_MASK;
        h->xfer_state = AS5600_XFER_IDLE;
        return true;
    }
    return false;
}

bool AS5600_ReadRawAngle_IT_Failed(AS5600_Handle_t *h)
{
    if (h->xfer_state == AS5600_XFER_ERROR) {
        h->xfer_state = AS5600_XFER_IDLE;
        return true;
    }
    return false;
}

#ifdef AS5600_USE_FREERTOS
HAL_StatusTypeDef AS5600_ReadRawAngle_Wait(AS5600_Handle_t *h, uint16_t *raw12,
                                           uint32_t timeout_ticks)
{
    /* Record which task to wake, then start the transfer. Clear any stale
     * notification first so a previous timeout can't satisfy this wait. */
    h->waiting_task = xTaskGetCurrentTaskHandle();
    ulTaskNotifyValueClear(NULL, 0xFFFFFFFFu);

    HAL_StatusTypeDef st = AS5600_ReadRawAngle_IT_Start(h);
    if (st != HAL_OK) {
        h->waiting_task = NULL;
        return st;
    }

    /* Block until the ISR notifies us (or we time out). */
    uint32_t notified = ulTaskNotifyTake(pdTRUE, timeout_ticks);
    h->waiting_task = NULL;

    if (notified == 0u) {
        /* No notification arrived: abort the in-flight transfer so the bus and
         * state machine don't stay stuck BUSY. */
        HAL_I2C_Master_Abort_IT(h->hi2c, h->addr_hal);
        h->xfer_state = AS5600_XFER_IDLE;
        return HAL_TIMEOUT;
    }
    if (h->xfer_state == AS5600_XFER_ERROR) {
        h->xfer_state = AS5600_XFER_IDLE;
        return HAL_ERROR;
    }

    *raw12 = ((uint16_t)h->rx_buf[0] << 8 | h->rx_buf[1]) & AS5600_ANGLE_MASK;
    h->xfer_state = AS5600_XFER_IDLE;
    return HAL_OK;
}
#endif /* AS5600_USE_FREERTOS */

void AS5600_I2C_RxCpltCallback(AS5600_Handle_t *h, I2C_HandleTypeDef *hi2c)
{
    if (h->hi2c != hi2c || h->xfer_state != AS5600_XFER_BUSY) {
        return;
    }
    h->xfer_state = AS5600_XFER_DONE;
#ifdef AS5600_USE_FREERTOS
    if (h->waiting_task != NULL) {
        BaseType_t hp_woken = pdFALSE;
        vTaskNotifyGiveFromISR(h->waiting_task, &hp_woken);
        portYIELD_FROM_ISR(hp_woken);
    }
#endif
}

void AS5600_I2C_ErrorCallback(AS5600_Handle_t *h, I2C_HandleTypeDef *hi2c)
{
    if (h->hi2c != hi2c || h->xfer_state != AS5600_XFER_BUSY) {
        return;
    }
    h->xfer_state = AS5600_XFER_ERROR;
#ifdef AS5600_USE_FREERTOS
    if (h->waiting_task != NULL) {
        BaseType_t hp_woken = pdFALSE;
        vTaskNotifyGiveFromISR(h->waiting_task, &hp_woken);
        portYIELD_FROM_ISR(hp_woken);
    }
#endif
}

/* ========================================================================== */
/* Configuration                                                              */
/* ========================================================================== */

HAL_StatusTypeDef AS5600_ReadConf(AS5600_Handle_t *h, uint16_t *conf)
{
    uint8_t buf[2];
    HAL_StatusTypeDef st = reg_read(h, AS5600_REG_CONF_H, buf, 2);
    if (st == HAL_OK) {
        *conf = (uint16_t)buf[0] << 8 | buf[1];
    }
    return st;
}

HAL_StatusTypeDef AS5600_WriteConf(AS5600_Handle_t *h, uint16_t conf)
{
    uint8_t buf[2];
    buf[0] = (uint8_t)(conf >> 8);
    buf[1] = (uint8_t)(conf & 0xFF);
    return reg_write(h, AS5600_REG_CONF_H, buf, 2);
}

HAL_StatusTypeDef AS5600_SetPowerMode(AS5600_Handle_t *h, AS5600_PM_t pm)
{
    return conf_rmw(h, AS5600_CONF_PM_Msk, AS5600_CONF_PM_Pos, (uint16_t)pm);
}

HAL_StatusTypeDef AS5600_SetHysteresis(AS5600_Handle_t *h, AS5600_HYST_t hyst)
{
    return conf_rmw(h, AS5600_CONF_HYST_Msk, AS5600_CONF_HYST_Pos, (uint16_t)hyst);
}

HAL_StatusTypeDef AS5600_SetOutputStage(AS5600_Handle_t *h, AS5600_OUTS_t outs)
{
    return conf_rmw(h, AS5600_CONF_OUTS_Msk, AS5600_CONF_OUTS_Pos, (uint16_t)outs);
}

HAL_StatusTypeDef AS5600_SetPwmFrequency(AS5600_Handle_t *h, AS5600_PWMF_t pwmf)
{
    return conf_rmw(h, AS5600_CONF_PWMF_Msk, AS5600_CONF_PWMF_Pos, (uint16_t)pwmf);
}

HAL_StatusTypeDef AS5600_SetSlowFilter(AS5600_Handle_t *h, AS5600_SF_t sf)
{
    return conf_rmw(h, AS5600_CONF_SF_Msk, AS5600_CONF_SF_Pos, (uint16_t)sf);
}

HAL_StatusTypeDef AS5600_SetFastFilterThreshold(AS5600_Handle_t *h, AS5600_FTH_t fth)
{
    return conf_rmw(h, AS5600_CONF_FTH_Msk, AS5600_CONF_FTH_Pos, (uint16_t)fth);
}

HAL_StatusTypeDef AS5600_SetWatchdog(AS5600_Handle_t *h, AS5600_WD_t wd)
{
    return conf_rmw(h, AS5600_CONF_WD_Msk, AS5600_CONF_WD_Pos, (uint16_t)wd);
}

HAL_StatusTypeDef AS5600_ApplyControlLoopDefaults(AS5600_Handle_t *h)
{
    uint16_t conf;
    HAL_StatusTypeDef st = AS5600_ReadConf(h, &conf);
    if (st != HAL_OK) {
        return st;
    }
    /* Clear every field we manage, then set our defaults in one write so we
     * only touch the bus once. */
    conf &= ~(AS5600_CONF_PM_Msk | AS5600_CONF_HYST_Msk |
              AS5600_CONF_SF_Msk | AS5600_CONF_FTH_Msk | AS5600_CONF_WD_Msk);
    conf |= ((uint16_t)AS5600_PM_NOM    << AS5600_CONF_PM_Pos)   & AS5600_CONF_PM_Msk;
    conf |= ((uint16_t)AS5600_HYST_1LSB << AS5600_CONF_HYST_Pos) & AS5600_CONF_HYST_Msk;
    conf |= ((uint16_t)AS5600_SF_16X    << AS5600_CONF_SF_Pos)   & AS5600_CONF_SF_Msk;
    conf |= ((uint16_t)AS5600_FTH_10LSB << AS5600_CONF_FTH_Pos)  & AS5600_CONF_FTH_Msk;
    conf |= ((uint16_t)AS5600_WD_OFF    << AS5600_CONF_WD_Pos)   & AS5600_CONF_WD_Msk;
    return AS5600_WriteConf(h, conf);
}

/* ========================================================================== */
/* Angular-range programming (RAM)                                            */
/* ========================================================================== */

HAL_StatusTypeDef AS5600_SetZPos(AS5600_Handle_t *h, uint16_t zpos12)
{
    return write_u12(h, AS5600_REG_ZPOS_H, zpos12);
}

HAL_StatusTypeDef AS5600_SetMPos(AS5600_Handle_t *h, uint16_t mpos12)
{
    return write_u12(h, AS5600_REG_MPOS_H, mpos12);
}

HAL_StatusTypeDef AS5600_SetMaxAngle(AS5600_Handle_t *h, uint16_t mang12)
{
    return write_u12(h, AS5600_REG_MANG_H, mang12);
}

/* ========================================================================== */
/* OTP burn commands (irreversible)                                           */
/* ========================================================================== */

HAL_StatusTypeDef AS5600_BurnAngle(AS5600_Handle_t *h, bool confirm)
{
    if (!confirm) {
        return HAL_ERROR;
    }

    /* Guard 1: magnet must be present (datasheet requirement). */
    AS5600_Status_t status;
    HAL_StatusTypeDef st = AS5600_ReadStatus(h, &status);
    if (st != HAL_OK) {
        return st;
    }
    if (!status.magnet_detected) {
        return HAL_ERROR;
    }

    /* Guard 2: at most 3 burns allowed. */
    uint8_t zmco;
    st = AS5600_ReadZMCO(h, &zmco);
    if (st != HAL_OK) {
        return st;
    }
    if (zmco >= 3U) {
        return HAL_ERROR;
    }

    uint8_t cmd = AS5600_CMD_BURN_ANGLE;
    st = reg_write(h, AS5600_REG_BURN, &cmd, 1);
    if (st == HAL_OK) {
        as5600_delay_ms(1); /* datasheet: allow >=1 ms for the write to take effect */
    }
    return st;
}

HAL_StatusTypeDef AS5600_BurnSetting(AS5600_Handle_t *h, bool confirm)
{
    if (!confirm) {
        return HAL_ERROR;
    }

    /* Magnet must be present. */
    AS5600_Status_t status;
    HAL_StatusTypeDef st = AS5600_ReadStatus(h, &status);
    if (st != HAL_OK) {
        return st;
    }
    if (!status.magnet_detected) {
        return HAL_ERROR;
    }

    /* MANG is only writable while ZPOS/MPOS have never been burned (ZMCO==0). */
    uint8_t zmco;
    st = AS5600_ReadZMCO(h, &zmco);
    if (st != HAL_OK) {
        return st;
    }
    if (zmco != 0U) {
        return HAL_ERROR;
    }

    uint8_t cmd = AS5600_CMD_BURN_SETTING;
    st = reg_write(h, AS5600_REG_BURN, &cmd, 1);
    if (st == HAL_OK) {
        as5600_delay_ms(1);
    }
    return st;
}
