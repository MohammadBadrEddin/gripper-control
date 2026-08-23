/**
 * @file    tmc2209_driver.c
 * @brief   TMC2209 single-wire UART driver + init sequence.
 * @see     TMC2209 datasheet Rev. 1.09, chapters 4, 5, 9, 16.
 */

#include "tmc2209_driver.h"

#include <string.h>

#ifdef TMC2209_USE_FREERTOS
#include "FreeRTOS.h"
#include "task.h"
#endif

/* ------------------------------------------------------------------------- */
/* Local helpers                                                             */
/* ------------------------------------------------------------------------- */

#define TMC_UART(h)   ((UART_HandleTypeDef *)((h)->huart))

/* Per-byte receive timeout once the first reply byte has arrived. At 115200
 * baud one byte is ~87 us, so 2 ms is generous but still bounded. */
#define TMC_BYTE_TIMEOUT_MS   2u

enum {
    TMC_XFER_IDLE = 0,
    TMC_XFER_TX,
    TMC_XFER_RX,
    TMC_XFER_DONE,
    TMC_XFER_ERROR
};

static void tmc_delay(const TMC2209_Handle *h, uint32_t ms)
{
    if (h != NULL && h->cfg.delay_ms != NULL) {
        h->cfg.delay_ms(ms);
        return;
    }
#ifdef TMC2209_USE_FREERTOS
    vTaskDelay(pdMS_TO_TICKS(ms));
#else
    HAL_Delay(ms);
#endif
}

/** Discard anything sitting in the RX path before starting a transaction.
 *  On a single wire line the previous datagram's echo is a common leftover. */
static void tmc_flush_rx(UART_HandleTypeDef *huart)
{
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);
}

/* ------------------------------------------------------------------------- */
/* Datagram primitives (datasheet 4.1 / 4.2)                                 */
/* ------------------------------------------------------------------------- */

uint8_t TMC2209_CRC8(const uint8_t *data, uint32_t len)
{
    uint8_t crc = 0;
    uint32_t i;

    if (data == NULL) {
        return 0;
    }

    for (i = 0; i < len; i++) {
        uint8_t current = data[i];
        int j;
        for (j = 0; j < 8; j++) {
            if (((crc >> 7) ^ (current & 0x01u)) != 0u) {
                crc = (uint8_t)((crc << 1) ^ TMC2209_CRC_POLY);
            } else {
                crc = (uint8_t)(crc << 1);
            }
            current = (uint8_t)(current >> 1);
        }
    }
    return crc;
}

void TMC2209_BuildWriteDatagram(uint8_t *out, uint8_t node_addr,
                                uint8_t reg, uint32_t value)
{
    if (out == NULL) {
        return;
    }
    out[0] = TMC2209_SYNC;
    out[1] = node_addr;
    out[2] = (uint8_t)((reg & TMC2209_REG_ADDR_MASK) | TMC2209_WRITE_FLAG);
    /* 32 bit data words are transmitted highest byte first (datasheet 4.1.1) */
    out[3] = (uint8_t)((value >> 24) & 0xFFu);
    out[4] = (uint8_t)((value >> 16) & 0xFFu);
    out[5] = (uint8_t)((value >> 8) & 0xFFu);
    out[6] = (uint8_t)(value & 0xFFu);
    out[7] = TMC2209_CRC8(out, TMC2209_WRITE_DATAGRAM_LEN - 1u);
}

void TMC2209_BuildReadRequest(uint8_t *out, uint8_t node_addr, uint8_t reg)
{
    if (out == NULL) {
        return;
    }
    out[0] = TMC2209_SYNC;
    out[1] = node_addr;
    out[2] = (uint8_t)(reg & TMC2209_REG_ADDR_MASK);
    out[3] = TMC2209_CRC8(out, TMC2209_READ_REQUEST_LEN - 1u);
}

TMC2209_Status TMC2209_ParseReadReply(const uint8_t *buf, uint32_t len,
                                      uint8_t reg, uint32_t *value_out)
{
    uint32_t i;
    bool header_seen = false;

    if ((buf == NULL) || (value_out == NULL)) {
        return TMC2209_ERR_PARAM;
    }
    if (len < TMC2209_READ_REPLY_LEN) {
        return TMC2209_ERR_FRAME;
    }

    /* Scan for the reply header. Anything before it is either line noise or
     * our own request echoed back by the single-wire transceiver. */
    for (i = 0; i + TMC2209_READ_REPLY_LEN <= len; i++) {
        if ((buf[i] != TMC2209_SYNC) ||
            (buf[i + 1] != TMC2209_MASTER_ADDR) ||
            (buf[i + 2] != (uint8_t)(reg & TMC2209_REG_ADDR_MASK))) {
            continue;
        }
        header_seen = true;

        if (TMC2209_CRC8(&buf[i], TMC2209_READ_REPLY_LEN - 1u) !=
            buf[i + TMC2209_READ_REPLY_LEN - 1u]) {
            continue;   /* keep scanning, a later frame may be intact */
        }

        *value_out = ((uint32_t)buf[i + 3] << 24) |
                     ((uint32_t)buf[i + 4] << 16) |
                     ((uint32_t)buf[i + 5] << 8)  |
                     ((uint32_t)buf[i + 6]);
        return TMC2209_OK;
    }

    return header_seen ? TMC2209_ERR_CRC : TMC2209_ERR_FRAME;
}

/* ------------------------------------------------------------------------- */
/* Blocking transport                                                        */
/* ------------------------------------------------------------------------- */

static TMC2209_Status tmc_transmit(TMC2209_Handle *h, const uint8_t *buf, uint16_t len)
{
    UART_HandleTypeDef *u = TMC_UART(h);

    tmc_flush_rx(u);
    HAL_HalfDuplex_EnableTransmitter(u);

    if (HAL_UART_Transmit(u, (uint8_t *)buf, len, h->cfg.timeout_ms) != HAL_OK) {
        return TMC2209_ERR_TX;
    }
    return TMC2209_OK;
}

/**
 * Receive the reply byte by byte so we never depend on an exact byte count:
 * depending on the board wiring the TX bytes may or may not be echoed back.
 * The framing scanner in TMC2209_ParseReadReply() sorts that out.
 */
static uint16_t tmc_receive_frame(TMC2209_Handle *h)
{
    UART_HandleTypeDef *u = TMC_UART(h);
    uint16_t n = 0;
    uint32_t timeout = h->cfg.timeout_ms;   /* generous for the first byte */

    HAL_HalfDuplex_EnableReceiver(u);

    while (n < (uint16_t)sizeof(h->rx_buf)) {
        if (HAL_UART_Receive(u, &h->rx_buf[n], 1u, timeout) != HAL_OK) {
            break;
        }
        n++;
        timeout = TMC_BYTE_TIMEOUT_MS;
    }
    return n;
}

TMC2209_Status TMC2209_WriteRegister(TMC2209_Handle *h, uint8_t reg, uint32_t value)
{
    TMC2209_Status st;

    if ((h == NULL) || (h->huart == NULL)) {
        return TMC2209_ERR_PARAM;
    }

    TMC2209_BuildWriteDatagram(h->tx_buf, h->cfg.node_address, reg, value);
    st = tmc_transmit(h, h->tx_buf, TMC2209_WRITE_DATAGRAM_LEN);

    /* Leave the line in receive (idle high) so the IC is not held by our TX. */
    HAL_HalfDuplex_EnableReceiver(TMC_UART(h));
    return st;
}

TMC2209_Status TMC2209_ReadRegister(TMC2209_Handle *h, uint8_t reg, uint32_t *value)
{
    TMC2209_Status st = TMC2209_ERR_TIMEOUT;
    uint8_t attempt;

    if ((h == NULL) || (h->huart == NULL) || (value == NULL)) {
        return TMC2209_ERR_PARAM;
    }

    for (attempt = 0; attempt <= h->cfg.retries; attempt++) {
        uint16_t n;

        TMC2209_BuildReadRequest(h->tx_buf, h->cfg.node_address, reg);
        st = tmc_transmit(h, h->tx_buf, TMC2209_READ_REQUEST_LEN);
        if (st != TMC2209_OK) {
            continue;
        }

        n = tmc_receive_frame(h);
        if (n < TMC2209_READ_REPLY_LEN) {
            h->timeouts++;
            st = TMC2209_ERR_TIMEOUT;
            continue;
        }

        st = TMC2209_ParseReadReply(h->rx_buf, n, reg, value);
        if (st == TMC2209_OK) {
            return TMC2209_OK;
        }
        if (st == TMC2209_ERR_CRC) {
            h->crc_errors++;
        }
        /* Datasheet 4.1: after an error the bus must be idle for at least
         * 12 bit times before retrying. 1 ms is far more than that. */
        tmc_delay(h, 1u);
    }
    return st;
}

TMC2209_Status TMC2209_WriteRegisterVerified(TMC2209_Handle *h, uint8_t reg,
                                             uint32_t value)
{
    TMC2209_Status st;
    uint32_t ifcnt_before = 0, ifcnt_after = 0;

    if (h == NULL) {
        return TMC2209_ERR_PARAM;
    }

    st = TMC2209_ReadRegister(h, TMC2209_REG_IFCNT, &ifcnt_before);
    if (st != TMC2209_OK) {
        return st;
    }

    st = TMC2209_WriteRegister(h, reg, value);
    if (st != TMC2209_OK) {
        return st;
    }

    st = TMC2209_ReadRegister(h, TMC2209_REG_IFCNT, &ifcnt_after);
    if (st != TMC2209_OK) {
        return st;
    }

    h->ifcnt = (uint8_t)ifcnt_after;

    /* IFCNT is 8 bit and wraps 255 -> 0 (datasheet 5.1). */
    if ((uint8_t)(ifcnt_after - ifcnt_before) != 1u) {
        h->lost_writes++;
        return TMC2209_ERR_WRITE_LOST;
    }
    return TMC2209_OK;
}

/* ------------------------------------------------------------------------- */
/* Current scaling (datasheet 9)                                             */
/* ------------------------------------------------------------------------- */

/*  I_RMS = (CS + 1)/32 * VFS / (Rsense + 20 mOhm) * 1/sqrt(2)
 *  =>  CS + 1 = 32 * sqrt(2) * I_RMS * (Rsense + 20 mOhm) / VFS
 *  Integer form with mA / mOhm / mV, sqrt(2) ~ 1414/1000.
 */
static uint32_t tmc_cs_from_current(uint16_t irms_ma, uint16_t rsense_mohm,
                                    uint16_t vfs_mv)
{
    uint64_t num = (uint64_t)32u * 1414u * (uint64_t)irms_ma *
                   ((uint64_t)rsense_mohm + TMC2209_RSENSE_INTERNAL_MOHM);
    uint64_t den = (uint64_t)1000u * 1000u * (uint64_t)vfs_mv;
    /* Truncate rather than round: the resulting current is then always at or
     * below the requested value, never above the motor's rating.            */
    uint64_t cs_plus_1 = num / den;

    return (cs_plus_1 == 0u) ? 0u : (uint32_t)(cs_plus_1 - 1u);
}

uint16_t TMC2209_CSToCurrent(uint8_t cs, uint16_t rsense_mohm, bool vsense)
{
    uint16_t vfs = vsense ? TMC2209_VFS_HIGH_SENS_MV : TMC2209_VFS_LOW_SENS_MV;
    uint64_t num = (uint64_t)((uint32_t)cs + 1u) * (uint64_t)vfs * 1000u * 1000u;
    uint64_t den = (uint64_t)32u * 1414u *
                   ((uint64_t)rsense_mohm + TMC2209_RSENSE_INTERNAL_MOHM);

    return (uint16_t)((num + (den / 2u)) / den);
}

TMC2209_Status TMC2209_CurrentToCS(uint16_t irms_ma, uint16_t rsense_mohm,
                                   uint8_t *cs_out, bool *vsense_out)
{
    uint32_t cs;
    bool vsense = false;

    if ((cs_out == NULL) || (vsense_out == NULL) || (rsense_mohm == 0u)) {
        return TMC2209_ERR_PARAM;
    }

    cs = tmc_cs_from_current(irms_ma, rsense_mohm, TMC2209_VFS_LOW_SENS_MV);

    /* Datasheet 9 hint: keep CS in 16..31 for good microstep quality. If the
     * low-sensitivity range puts us below that, switch to vsense = 1, which
     * lowers VFS to 180 mV and moves CS up by ~1.8x. */
    if (cs < 16u) {
        uint32_t cs_hi = tmc_cs_from_current(irms_ma, rsense_mohm,
                                             TMC2209_VFS_HIGH_SENS_MV);
        if (cs_hi <= TMC2209_CS_MAX) {
            cs = cs_hi;
            vsense = true;
        }
    }

    *vsense_out = vsense;

    if (cs > TMC2209_CS_MAX) {
        *cs_out = TMC2209_CS_MAX;
        /* Requested current cannot be reached with this sense resistor. */
        return TMC2209_ERR_PARAM;
    }
    *cs_out = (uint8_t)cs;
    return TMC2209_OK;
}

/* ------------------------------------------------------------------------- */
/* Defaults                                                                  */
/* ------------------------------------------------------------------------- */

void TMC2209_GetDefaultConfig(TMC2209_Config *cfg)
{
    if (cfg == NULL) {
        return;
    }
    memset(cfg, 0, sizeof(*cfg));

    /* Interface. Node address 0 = MS1/MS2 both low, the stepstick default. */
    cfg->node_address  = 0u;
    cfg->senddelay     = 2u;      /* 3 * 8 bit times of TX->RX turnaround   */
    cfg->timeout_ms    = 10u;
    cfg->retries       = 2u;

    /* Current: 17HE12-1204S is rated 1.2 A/phase; 0.11 Ohm sense resistors
     * are the common value on v1.3 stepstick modules (verify both!). */
    cfg->rsense_mohm   = 110u;
    cfg->irun_ma       = 1000u;   /* conservative start, below the 1.2 A rating */
    cfg->ihold_percent = 50u;
    cfg->iholddelay    = 4u;
    cfg->tpowerdown    = 20u;     /* reset default; >= 2 needed for StealthChop AT */
    cfg->use_vref      = false;   /* current fully defined by IRUN + Rsense  */
    cfg->internal_rsense = false;

    /* Resolution: 1/16 with MicroPlyer interpolation to 256 - 3200 steps/rev
     * on a 1.8 deg motor, an easy STEP rate for the timer. */
    cfg->microsteps    = TMC2209_MRES_16;
    cfg->interpolate   = true;
    cfg->invert_shaft  = false;
    cfg->double_edge   = false;

    /* Chopper: StealthChop, basic values from the quick config guide (16.1). */
    cfg->chopper       = TMC2209_CHOPPER_STEALTHCHOP;
    cfg->toff          = 5u;
    cfg->tbl           = 2u;
    cfg->hstrt         = 4u;
    cfg->hend          = 0u;
    cfg->tpwmthrs      = 0u;      /* StealthChop over the whole speed range */

    /* StealthChop PWM: automatic tuning (datasheet 6.1 / 6.2). */
    cfg->pwm_autoscale = true;
    cfg->pwm_autograd  = true;
    cfg->pwm_freq      = 1u;      /* 35.1 kHz at 12 MHz internal clock       */
    cfg->pwm_reg       = 4u;
    cfg->pwm_lim       = 12u;     /* reset default                           */
    cfg->pwm_ofs       = 36u;     /* reset default                           */
    cfg->pwm_grad      = 14u;     /* OTP default starting point              */
    cfg->freewheel     = TMC2209_FREEWHEEL_NORMAL;

    /* StallGuard4 off until SGTHRS has been tuned on the real gripper. */
    cfg->stallguard_enable = false;
    cfg->sgthrs        = 0u;
    cfg->tcoolthrs     = 0u;

    cfg->set_enable    = NULL;
    cfg->delay_ms      = NULL;
}

/* ------------------------------------------------------------------------- */
/* Register composition                                                      */
/* ------------------------------------------------------------------------- */

static uint32_t tmc_build_gconf(const TMC2209_Handle *h)
{
    uint32_t g = 0;

    if (h->cfg.use_vref)         { g |= TMC2209_GCONF_I_SCALE_ANALOG; }
    if (h->cfg.internal_rsense)  { g |= TMC2209_GCONF_INTERNAL_RSENSE; }
    if (h->cfg.chopper == TMC2209_CHOPPER_SPREADCYCLE) {
        g |= TMC2209_GCONF_EN_SPREADCYCLE;
    }
    if (h->cfg.invert_shaft)     { g |= TMC2209_GCONF_SHAFT; }

    /* Mandatory for UART operation: the PDN_UART pin function must be off,
     * and MRES must come from CHOPCONF rather than the MS1/MS2 pins
     * (datasheet 3.4 and 5.1). */
    g |= TMC2209_GCONF_PDN_DISABLE;
    g |= TMC2209_GCONF_MSTEP_REG_SELECT;
    g |= TMC2209_GCONF_MULTISTEP_FILT;   /* reset default = 1 */

    return g;
}

static uint32_t tmc_build_chopconf(const TMC2209_Handle *h, bool enable_power_stage)
{
    uint32_t c = 0;

    c = TMC2209_FIELD_SET(c, TMC2209_CHOPCONF_TOFF_MASK,
                          TMC2209_CHOPCONF_TOFF_SHIFT,
                          enable_power_stage ? h->cfg.toff : 0u);
    c = TMC2209_FIELD_SET(c, TMC2209_CHOPCONF_HSTRT_MASK,
                          TMC2209_CHOPCONF_HSTRT_SHIFT, h->cfg.hstrt);
    c = TMC2209_FIELD_SET(c, TMC2209_CHOPCONF_HEND_MASK,
                          TMC2209_CHOPCONF_HEND_SHIFT, h->cfg.hend);
    c = TMC2209_FIELD_SET(c, TMC2209_CHOPCONF_TBL_MASK,
                          TMC2209_CHOPCONF_TBL_SHIFT, h->cfg.tbl);
    c = TMC2209_FIELD_SET(c, TMC2209_CHOPCONF_MRES_MASK,
                          TMC2209_CHOPCONF_MRES_SHIFT, (uint32_t)h->cfg.microsteps);

    if (h->vsense)             { c |= TMC2209_CHOPCONF_VSENSE; }
    if (h->cfg.interpolate)    { c |= TMC2209_CHOPCONF_INTPOL; }
    /* dedge and multistep_filt are mutually exclusive (datasheet 5.5.1). */
    if (h->cfg.double_edge && !h->cfg.interpolate) {
        c |= TMC2209_CHOPCONF_DEDGE;
    }
    return c;
}

static uint32_t tmc_build_ihold_irun(const TMC2209_Handle *h)
{
    uint32_t r = 0;

    r = TMC2209_FIELD_SET(r, TMC2209_IHOLD_MASK, TMC2209_IHOLD_SHIFT, h->cs_hold);
    r = TMC2209_FIELD_SET(r, TMC2209_IRUN_MASK,  TMC2209_IRUN_SHIFT,  h->cs_run);
    r = TMC2209_FIELD_SET(r, TMC2209_IHOLDDELAY_MASK, TMC2209_IHOLDDELAY_SHIFT,
                          h->cfg.iholddelay & 0x0Fu);
    return r;
}

static uint32_t tmc_build_pwmconf(const TMC2209_Handle *h)
{
    uint32_t p = 0;

    p = TMC2209_FIELD_SET(p, TMC2209_PWMCONF_PWM_OFS_MASK,
                          TMC2209_PWMCONF_PWM_OFS_SHIFT, h->cfg.pwm_ofs);
    p = TMC2209_FIELD_SET(p, TMC2209_PWMCONF_PWM_GRAD_MASK,
                          TMC2209_PWMCONF_PWM_GRAD_SHIFT, h->cfg.pwm_grad);
    p = TMC2209_FIELD_SET(p, TMC2209_PWMCONF_PWM_FREQ_MASK,
                          TMC2209_PWMCONF_PWM_FREQ_SHIFT, h->cfg.pwm_freq & 0x03u);
    p = TMC2209_FIELD_SET(p, TMC2209_PWMCONF_FREEWHEEL_MASK,
                          TMC2209_PWMCONF_FREEWHEEL_SHIFT, (uint32_t)h->cfg.freewheel);
    p = TMC2209_FIELD_SET(p, TMC2209_PWMCONF_PWM_REG_MASK,
                          TMC2209_PWMCONF_PWM_REG_SHIFT, h->cfg.pwm_reg & 0x0Fu);
    p = TMC2209_FIELD_SET(p, TMC2209_PWMCONF_PWM_LIM_MASK,
                          TMC2209_PWMCONF_PWM_LIM_SHIFT, h->cfg.pwm_lim & 0x0Fu);

    if (h->cfg.pwm_autoscale) { p |= TMC2209_PWMCONF_AUTOSCALE; }
    if (h->cfg.pwm_autograd)  { p |= TMC2209_PWMCONF_AUTOGRAD; }
    return p;
}

/* ------------------------------------------------------------------------- */
/* Init                                                                      */
/* ------------------------------------------------------------------------- */

/* Write and verify, bailing out on the first failure. */
#define TMC_WRITE_CHECKED(h, reg, val)                        \
    do {                                                      \
        TMC2209_Status _st = TMC2209_WriteRegisterVerified((h), (reg), (val)); \
        if (_st != TMC2209_OK) { return _st; }                \
    } while (0)

TMC2209_Status TMC2209_ApplyConfig(TMC2209_Handle *h)
{
    TMC2209_Status st;
    uint32_t gstat = 0;

    if (h == NULL) {
        return TMC2209_ERR_PARAM;
    }

    /* Resolve the current setting first: IHOLD_IRUN must be correct before
     * the power stage is enabled, otherwise the motor sees the reset default
     * IRUN = 31 at full sense-resistor current (datasheet 8, attention box). */
    st = TMC2209_CurrentToCS(h->cfg.irun_ma, h->cfg.rsense_mohm,
                             &h->cs_run, &h->vsense);
    if (st != TMC2209_OK) {
        return st;
    }
    h->cs_hold = (uint8_t)(((uint32_t)h->cs_run * h->cfg.ihold_percent) / 100u);

    /* 1. UART reply turnaround (write-only register, cannot be verified). */
    st = TMC2209_WriteRegister(h, TMC2209_REG_NODECONF,
                               TMC2209_FIELD_SET(0u,
                                   TMC2209_NODECONF_SENDDELAY_MASK,
                                   TMC2209_NODECONF_SENDDELAY_SHIFT,
                                   h->cfg.senddelay & 0x0Fu));
    if (st != TMC2209_OK) {
        return st;
    }

    /* 2. Clear the power-on reset / error flags (write 1 to clear). */
    st = TMC2209_WriteRegister(h, TMC2209_REG_GSTAT, TMC2209_GSTAT_CLEAR_ALL);
    if (st != TMC2209_OK) {
        return st;
    }

    /* 3. Global config - pdn_disable and mstep_reg_select go in here. */
    h->shadow_gconf = tmc_build_gconf(h);
    TMC_WRITE_CHECKED(h, TMC2209_REG_GCONF, h->shadow_gconf);

    /* 4. Current control, before the driver is enabled. */
    h->shadow_ihold_irun = tmc_build_ihold_irun(h);
    TMC_WRITE_CHECKED(h, TMC2209_REG_IHOLD_IRUN, h->shadow_ihold_irun);
    TMC_WRITE_CHECKED(h, TMC2209_REG_TPOWERDOWN, h->cfg.tpowerdown);

    /* 5. StealthChop PWM configuration. */
    h->shadow_pwmconf = tmc_build_pwmconf(h);
    TMC_WRITE_CHECKED(h, TMC2209_REG_PWMCONF, h->shadow_pwmconf);
    TMC_WRITE_CHECKED(h, TMC2209_REG_TPWMTHRS,
                      h->cfg.tpwmthrs & TMC2209_20BIT_MASK);

    /* 6. Make sure the internal step generator is stopped before enabling. */
    h->shadow_vactual = 0;
    TMC_WRITE_CHECKED(h, TMC2209_REG_VACTUAL, 0u);

    /* 7. StallGuard4 / DIAG. TCOOLTHRS gates the stall output; SGTHRS sets
     *    the trip level (datasheet 11.2). Both are 0 when disabled. */
    TMC_WRITE_CHECKED(h, TMC2209_REG_TCOOLTHRS,
                      h->cfg.stallguard_enable ?
                          (h->cfg.tcoolthrs & TMC2209_20BIT_MASK) : 0u);
    TMC_WRITE_CHECKED(h, TMC2209_REG_SGTHRS,
                      h->cfg.stallguard_enable ? h->cfg.sgthrs : 0u);

    /* 8. Chopper last: writing TOFF != 0 is what actually enables the power
     *    stage, so everything else is already in place by now.             */
    h->shadow_chopconf = tmc_build_chopconf(h, true);
    TMC_WRITE_CHECKED(h, TMC2209_REG_CHOPCONF, h->shadow_chopconf);

    /* 9. Re-read status: a set GSTAT.reset here means the IC rebooted mid
     *    configuration and the register set is not what we think it is.    */
    st = TMC2209_ReadRegister(h, TMC2209_REG_GSTAT, &gstat);
    if (st != TMC2209_OK) {
        return st;
    }
    if ((gstat & TMC2209_GSTAT_RESET) != 0u) {
        return TMC2209_ERR_DRIVER_FAULT;
    }
    if ((gstat & (TMC2209_GSTAT_DRV_ERR | TMC2209_GSTAT_UV_CP)) != 0u) {
        return TMC2209_ERR_DRIVER_FAULT;
    }

    return TMC2209_OK;
}

TMC2209_Status TMC2209_Init(TMC2209_Handle *h, UART_HandleTypeDef *huart,
                            const TMC2209_Config *cfg)
{
    TMC2209_Status st;
    uint32_t ioin = 0;

    if ((h == NULL) || (huart == NULL)) {
        return TMC2209_ERR_PARAM;
    }

    memset(h, 0, sizeof(*h));
    h->huart = (void *)huart;

    if (cfg != NULL) {
        h->cfg = *cfg;
    } else {
        TMC2209_GetDefaultConfig(&h->cfg);
    }
    if (h->cfg.node_address > TMC2209_NODE_ADDR_MAX) {
        return TMC2209_ERR_PARAM;
    }
    if (h->cfg.timeout_ms == 0u) {
        h->cfg.timeout_ms = 10u;
    }

    /* Keep the power stage off while we configure. Note that MX_GPIO_Init()
     * drives TMC_EN low, i.e. the driver is ENABLED from reset with the
     * default IRUN = 31 - provide set_enable() and call it early. */
    if (h->cfg.set_enable != NULL) {
        h->cfg.set_enable(false);
    }

    /* Let VCC_IO / VS settle and the line idle high before the first sync. */
    tmc_delay(h, 10u);
    HAL_HalfDuplex_EnableReceiver(TMC_UART(h));

    /* Comms check: IOIN is read-only and carries the IC version, so a
     * correct reply proves framing, baud rate and CRC all line up. */
    st = TMC2209_ReadRegister(h, TMC2209_REG_IOIN, &ioin);
    if (st != TMC2209_OK) {
        return st;
    }
    h->version = (uint8_t)TMC2209_FIELD_GET(ioin, TMC2209_IOIN_VERSION_MASK,
                                            TMC2209_IOIN_VERSION_SHIFT);
    if (h->version != TMC2209_IOIN_VERSION_TMC2209) {
        return TMC2209_ERR_VERSION;
    }

    st = TMC2209_ApplyConfig(h);
    if (st != TMC2209_OK) {
        return st;
    }

    h->initialised = true;
    return TMC2209_OK;
}

/* ------------------------------------------------------------------------- */
/* Setters                                                                   */
/* ------------------------------------------------------------------------- */

TMC2209_Status TMC2209_SetCurrent(TMC2209_Handle *h, uint16_t irun_ma,
                                  uint8_t ihold_percent)
{
    TMC2209_Status st;
    bool vsense;
    uint8_t cs;

    if (h == NULL) {
        return TMC2209_ERR_PARAM;
    }
    if (ihold_percent > 100u) {
        return TMC2209_ERR_PARAM;
    }

    st = TMC2209_CurrentToCS(irun_ma, h->cfg.rsense_mohm, &cs, &vsense);
    if (st != TMC2209_OK) {
        return st;
    }

    h->cfg.irun_ma       = irun_ma;
    h->cfg.ihold_percent = ihold_percent;
    h->cs_run  = cs;
    h->cs_hold = (uint8_t)(((uint32_t)cs * ihold_percent) / 100u);

    /* vsense lives in CHOPCONF, so it may need rewriting too. */
    if (vsense != h->vsense) {
        h->vsense = vsense;
        h->shadow_chopconf = tmc_build_chopconf(h, true);
        st = TMC2209_WriteRegisterVerified(h, TMC2209_REG_CHOPCONF,
                                           h->shadow_chopconf);
        if (st != TMC2209_OK) {
            return st;
        }
    }

    h->shadow_ihold_irun = tmc_build_ihold_irun(h);
    return TMC2209_WriteRegisterVerified(h, TMC2209_REG_IHOLD_IRUN,
                                         h->shadow_ihold_irun);
}

TMC2209_Status TMC2209_SetMicrosteps(TMC2209_Handle *h, TMC2209_Microsteps mres)
{
    if ((h == NULL) || ((uint32_t)mres > (uint32_t)TMC2209_MRES_FULLSTEP)) {
        return TMC2209_ERR_PARAM;
    }
    h->cfg.microsteps = mres;
    h->shadow_chopconf = TMC2209_FIELD_SET(h->shadow_chopconf,
                                           TMC2209_CHOPCONF_MRES_MASK,
                                           TMC2209_CHOPCONF_MRES_SHIFT,
                                           (uint32_t)mres);
    return TMC2209_WriteRegisterVerified(h, TMC2209_REG_CHOPCONF,
                                         h->shadow_chopconf);
}

TMC2209_Status TMC2209_SetChopperMode(TMC2209_Handle *h, TMC2209_ChopperMode mode)
{
    if (h == NULL) {
        return TMC2209_ERR_PARAM;
    }
    h->cfg.chopper = mode;
    if (mode == TMC2209_CHOPPER_SPREADCYCLE) {
        h->shadow_gconf |= TMC2209_GCONF_EN_SPREADCYCLE;
    } else {
        h->shadow_gconf &= ~TMC2209_GCONF_EN_SPREADCYCLE;
    }
    return TMC2209_WriteRegisterVerified(h, TMC2209_REG_GCONF, h->shadow_gconf);
}

TMC2209_Status TMC2209_SetShaftDirection(TMC2209_Handle *h, bool inverted)
{
    if (h == NULL) {
        return TMC2209_ERR_PARAM;
    }
    h->cfg.invert_shaft = inverted;
    if (inverted) {
        h->shadow_gconf |= TMC2209_GCONF_SHAFT;
    } else {
        h->shadow_gconf &= ~TMC2209_GCONF_SHAFT;
    }
    return TMC2209_WriteRegisterVerified(h, TMC2209_REG_GCONF, h->shadow_gconf);
}

TMC2209_Status TMC2209_SetStallGuard(TMC2209_Handle *h, uint8_t sgthrs,
                                     uint32_t tcoolthrs)
{
    TMC2209_Status st;

    if (h == NULL) {
        return TMC2209_ERR_PARAM;
    }
    h->cfg.sgthrs    = sgthrs;
    h->cfg.tcoolthrs = tcoolthrs;
    h->cfg.stallguard_enable = (sgthrs != 0u);

    st = TMC2209_WriteRegisterVerified(h, TMC2209_REG_TCOOLTHRS,
                                       tcoolthrs & TMC2209_20BIT_MASK);
    if (st != TMC2209_OK) {
        return st;
    }
    return TMC2209_WriteRegisterVerified(h, TMC2209_REG_SGTHRS, sgthrs);
}

TMC2209_Status TMC2209_Enable(TMC2209_Handle *h, bool enable)
{
    TMC2209_Status st;

    if (h == NULL) {
        return TMC2209_ERR_PARAM;
    }

    if (!enable && (h->cfg.set_enable != NULL)) {
        h->cfg.set_enable(false);   /* hardware off first */
    }

    h->shadow_chopconf = tmc_build_chopconf(h, enable);
    st = TMC2209_WriteRegisterVerified(h, TMC2209_REG_CHOPCONF,
                                       h->shadow_chopconf);
    if (st != TMC2209_OK) {
        return st;
    }

    if (enable && (h->cfg.set_enable != NULL)) {
        h->cfg.set_enable(true);    /* hardware on last */
    }
    return TMC2209_OK;
}

/* ------------------------------------------------------------------------- */
/* Motion / status                                                           */
/* ------------------------------------------------------------------------- */

TMC2209_Status TMC2209_SetVelocity(TMC2209_Handle *h, int32_t vactual)
{
    if (h == NULL) {
        return TMC2209_ERR_PARAM;
    }
    if ((vactual > TMC2209_VACTUAL_MAX) || (vactual < TMC2209_VACTUAL_MIN)) {
        return TMC2209_ERR_PARAM;
    }
    h->shadow_vactual = vactual;
    /* 24 bit two's complement (datasheet 5.2). */
    return TMC2209_WriteRegister(h, TMC2209_REG_VACTUAL,
                                 (uint32_t)vactual & TMC2209_VACTUAL_MASK);
}

TMC2209_Status TMC2209_ClearGSTAT(TMC2209_Handle *h)
{
    if (h == NULL) {
        return TMC2209_ERR_PARAM;
    }
    return TMC2209_WriteRegister(h, TMC2209_REG_GSTAT, TMC2209_GSTAT_CLEAR_ALL);
}

TMC2209_Status TMC2209_GetStatus(TMC2209_Handle *h, TMC2209_DrvStatus *st_out)
{
    TMC2209_Status st;
    uint32_t drv = 0, gstat = 0;

    if ((h == NULL) || (st_out == NULL)) {
        return TMC2209_ERR_PARAM;
    }

    st = TMC2209_ReadRegister(h, TMC2209_REG_DRV_STATUS, &drv);
    if (st != TMC2209_OK) {
        return st;
    }
    st = TMC2209_ReadRegister(h, TMC2209_REG_GSTAT, &gstat);
    if (st != TMC2209_OK) {
        return st;
    }

    memset(st_out, 0, sizeof(*st_out));
    st_out->raw_drv_status = drv;
    st_out->raw_gstat      = (uint8_t)gstat;

    st_out->reset_flag  = (gstat & TMC2209_GSTAT_RESET)   != 0u;
    st_out->drv_err     = (gstat & TMC2209_GSTAT_DRV_ERR) != 0u;
    st_out->uv_cp       = (gstat & TMC2209_GSTAT_UV_CP)   != 0u;

    st_out->otpw        = (drv & TMC2209_DRV_STATUS_OTPW)  != 0u;
    st_out->ot          = (drv & TMC2209_DRV_STATUS_OT)    != 0u;
    st_out->short_gnd_a = (drv & TMC2209_DRV_STATUS_S2GA)  != 0u;
    st_out->short_gnd_b = (drv & TMC2209_DRV_STATUS_S2GB)  != 0u;
    st_out->short_vs_a  = (drv & TMC2209_DRV_STATUS_S2VSA) != 0u;
    st_out->short_vs_b  = (drv & TMC2209_DRV_STATUS_S2VSB) != 0u;
    st_out->open_load_a = (drv & TMC2209_DRV_STATUS_OLA)   != 0u;
    st_out->open_load_b = (drv & TMC2209_DRV_STATUS_OLB)   != 0u;
    st_out->t120        = (drv & TMC2209_DRV_STATUS_T120)  != 0u;
    st_out->t143        = (drv & TMC2209_DRV_STATUS_T143)  != 0u;
    st_out->t150        = (drv & TMC2209_DRV_STATUS_T150)  != 0u;
    st_out->t157        = (drv & TMC2209_DRV_STATUS_T157)  != 0u;
    st_out->stealth     = (drv & TMC2209_DRV_STATUS_STEALTH) != 0u;
    st_out->standstill  = (drv & TMC2209_DRV_STATUS_STST)  != 0u;
    st_out->cs_actual   = (uint8_t)TMC2209_FIELD_GET(drv,
                              TMC2209_DRV_STATUS_CSACTUAL_MASK,
                              TMC2209_DRV_STATUS_CSACTUAL_SHIFT);

    return TMC2209_OK;
}

TMC2209_Status TMC2209_GetStallGuardResult(TMC2209_Handle *h, uint16_t *sg)
{
    TMC2209_Status st;
    uint32_t v = 0;

    if ((h == NULL) || (sg == NULL)) {
        return TMC2209_ERR_PARAM;
    }
    st = TMC2209_ReadRegister(h, TMC2209_REG_SG_RESULT, &v);
    if (st == TMC2209_OK) {
        *sg = (uint16_t)(v & TMC2209_SG_RESULT_MASK);
    }
    return st;
}

TMC2209_Status TMC2209_GetMicrostepCounter(TMC2209_Handle *h, uint16_t *mscnt)
{
    TMC2209_Status st;
    uint32_t v = 0;

    if ((h == NULL) || (mscnt == NULL)) {
        return TMC2209_ERR_PARAM;
    }
    st = TMC2209_ReadRegister(h, TMC2209_REG_MSCNT, &v);
    if (st == TMC2209_OK) {
        *mscnt = (uint16_t)(v & TMC2209_MSCNT_MASK);
    }
    return st;
}

TMC2209_Status TMC2209_GetTStep(TMC2209_Handle *h, uint32_t *tstep)
{
    TMC2209_Status st;
    uint32_t v = 0;

    if ((h == NULL) || (tstep == NULL)) {
        return TMC2209_ERR_PARAM;
    }
    st = TMC2209_ReadRegister(h, TMC2209_REG_TSTEP, &v);
    if (st == TMC2209_OK) {
        *tstep = v & TMC2209_20BIT_MASK;
    }
    return st;
}

/* ------------------------------------------------------------------------- */
/* Interrupt-mode transfers for the control loop                             */
/* ------------------------------------------------------------------------- */
#ifdef TMC2209_USE_FREERTOS

#define TMC_NOTIFY_TIMEOUT(h) pdMS_TO_TICKS((h)->cfg.timeout_ms)

static TMC2209_Status tmc_wait_notify(TMC2209_Handle *h)
{
    if (ulTaskNotifyTake(pdTRUE, TMC_NOTIFY_TIMEOUT(h)) == 0u) {
        h->xfer_state = TMC_XFER_IDLE;
        h->timeouts++;
        return TMC2209_ERR_TIMEOUT;
    }
    return (h->xfer_state == TMC_XFER_ERROR) ? TMC2209_ERR_TX : TMC2209_OK;
}

TMC2209_Status TMC2209_WriteRegisterIT(TMC2209_Handle *h, uint8_t reg, uint32_t value)
{
    UART_HandleTypeDef *u;
    TMC2209_Status st;

    if ((h == NULL) || (h->huart == NULL)) {
        return TMC2209_ERR_PARAM;
    }
    if (h->xfer_state != TMC_XFER_IDLE) {
        return TMC2209_ERR_BUSY;
    }

    u = TMC_UART(h);
    h->task = (void *)xTaskGetCurrentTaskHandle();
    h->xfer_state = TMC_XFER_TX;

    TMC2209_BuildWriteDatagram(h->tx_buf, h->cfg.node_address, reg, value);

    tmc_flush_rx(u);
    HAL_HalfDuplex_EnableTransmitter(u);
    if (HAL_UART_Transmit_IT(u, h->tx_buf, TMC2209_WRITE_DATAGRAM_LEN) != HAL_OK) {
        h->xfer_state = TMC_XFER_IDLE;
        return TMC2209_ERR_TX;
    }

    st = tmc_wait_notify(h);
    h->xfer_state = TMC_XFER_IDLE;
    HAL_HalfDuplex_EnableReceiver(u);
    return st;
}

TMC2209_Status TMC2209_ReadRegisterIT(TMC2209_Handle *h, uint8_t reg, uint32_t *value)
{
    UART_HandleTypeDef *u;
    TMC2209_Status st;

    if ((h == NULL) || (h->huart == NULL) || (value == NULL)) {
        return TMC2209_ERR_PARAM;
    }
    if (h->xfer_state != TMC_XFER_IDLE) {
        return TMC2209_ERR_BUSY;
    }

    u = TMC_UART(h);
    h->task = (void *)xTaskGetCurrentTaskHandle();
    TMC2209_BuildReadRequest(h->tx_buf, h->cfg.node_address, reg);

    /* Phase 1: transmit the request. */
    h->xfer_state = TMC_XFER_TX;
    tmc_flush_rx(u);
    HAL_HalfDuplex_EnableTransmitter(u);
    if (HAL_UART_Transmit_IT(u, h->tx_buf, TMC2209_READ_REQUEST_LEN) != HAL_OK) {
        h->xfer_state = TMC_XFER_IDLE;
        return TMC2209_ERR_TX;
    }
    st = tmc_wait_notify(h);
    if (st != TMC2209_OK) {
        h->xfer_state = TMC_XFER_IDLE;
        HAL_HalfDuplex_EnableReceiver(u);
        return st;
    }

    /* Phase 2: receive-to-idle, so we do not need to know up front whether
     * our own bytes are echoed back on the shared wire. */
    h->xfer_state = TMC_XFER_RX;
    h->rx_len = 0;
    HAL_HalfDuplex_EnableReceiver(u);
    if (HAL_UARTEx_ReceiveToIdle_IT(u, h->rx_buf,
                                    (uint16_t)sizeof(h->rx_buf)) != HAL_OK) {
        h->xfer_state = TMC_XFER_IDLE;
        return TMC2209_ERR_TX;
    }

    st = tmc_wait_notify(h);
    h->xfer_state = TMC_XFER_IDLE;
    if (st != TMC2209_OK) {
        HAL_UART_AbortReceive_IT(u);
        return st;
    }

    st = TMC2209_ParseReadReply(h->rx_buf, h->rx_len, reg, value);
    if (st == TMC2209_ERR_CRC) {
        h->crc_errors++;
    }
    return st;
}

void TMC2209_UART_TxCpltCallback(TMC2209_Handle *h)
{
    BaseType_t woken = pdFALSE;

    if ((h == NULL) || (h->xfer_state != TMC_XFER_TX) || (h->task == NULL)) {
        return;
    }
    h->xfer_state = TMC_XFER_DONE;
    vTaskNotifyGiveFromISR((TaskHandle_t)h->task, &woken);
    portYIELD_FROM_ISR(woken);
}

void TMC2209_UART_RxEventCallback(TMC2209_Handle *h, uint16_t size)
{
    BaseType_t woken = pdFALSE;

    if ((h == NULL) || (h->xfer_state != TMC_XFER_RX) || (h->task == NULL)) {
        return;
    }
    h->rx_len = size;
    h->xfer_state = TMC_XFER_DONE;
    vTaskNotifyGiveFromISR((TaskHandle_t)h->task, &woken);
    portYIELD_FROM_ISR(woken);
}

void TMC2209_UART_ErrorCallback(TMC2209_Handle *h)
{
    BaseType_t woken = pdFALSE;

    if ((h == NULL) || (h->task == NULL)) {
        return;
    }
    if ((h->xfer_state != TMC_XFER_TX) && (h->xfer_state != TMC_XFER_RX)) {
        return;
    }
    h->xfer_state = TMC_XFER_ERROR;
    vTaskNotifyGiveFromISR((TaskHandle_t)h->task, &woken);
    portYIELD_FROM_ISR(woken);
}

#endif /* TMC2209_USE_FREERTOS */
