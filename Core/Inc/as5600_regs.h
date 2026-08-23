/**
 * @file    as5600_regs.h
 * @brief   AS5600 register map and bitfield definitions.
 *
 * Source: ams OSRAM AS5600 datasheet [v1-06] 2018-Jun-20, Fig. 21-23.
 * All multi-byte registers are big-endian (high byte at the lower address).
 * Angle/position values are 12-bit (0..4095).
 *
 * This header contains constants only. It pulls in no HAL headers and can be
 * included by host-side unit tests.
 */
#ifndef AS5600_REGS_H
#define AS5600_REGS_H

/* ---- I2C device address --------------------------------------------------
 * 7-bit slave address. The STM32 HAL expects the address left-shifted by 1.
 */
#define AS5600_I2C_ADDR_7BIT        (0x36U)
#define AS5600_I2C_ADDR_HAL         (AS5600_I2C_ADDR_7BIT << 1)   /* 0x6C */

/* ---- Configuration registers (R/W, some OTP-programmable) ---------------- */
#define AS5600_REG_ZMCO             (0x00U) /* burn count for ZPOS/MPOS (0..3) */
#define AS5600_REG_ZPOS_H           (0x01U) /* start position, bits 11:8       */
#define AS5600_REG_ZPOS_L           (0x02U) /* start position, bits 7:0        */
#define AS5600_REG_MPOS_H           (0x03U) /* stop position,  bits 11:8       */
#define AS5600_REG_MPOS_L           (0x04U) /* stop position,  bits 7:0        */
#define AS5600_REG_MANG_H           (0x05U) /* max angle,      bits 11:8       */
#define AS5600_REG_MANG_L           (0x06U) /* max angle,      bits 7:0        */
#define AS5600_REG_CONF_H           (0x07U) /* WD, FTH, SF                     */
#define AS5600_REG_CONF_L           (0x08U) /* PWMF, OUTS, HYST, PM            */

/* ---- Output registers (read-only) --------------------------------------- */
#define AS5600_REG_RAWANGLE_H       (0x0CU) /* unscaled angle, bits 11:8       */
#define AS5600_REG_RAWANGLE_L       (0x0DU) /* unscaled angle, bits 7:0        */
#define AS5600_REG_ANGLE_H          (0x0EU) /* scaled angle,   bits 11:8       */
#define AS5600_REG_ANGLE_L          (0x0FU) /* scaled angle,   bits 7:0        */

/* ---- Status registers (read-only) --------------------------------------- */
#define AS5600_REG_STATUS           (0x0BU)
#define AS5600_REG_AGC              (0x1AU) /* gain: 0..255 (5V) / 0..128 (3V3)*/
#define AS5600_REG_MAGNITUDE_H      (0x1BU) /* CORDIC magnitude, bits 11:8     */
#define AS5600_REG_MAGNITUDE_L      (0x1CU) /* CORDIC magnitude, bits 7:0      */

/* ---- Burn command register (OTP, irreversible) -------------------------- */
#define AS5600_REG_BURN             (0xFFU)
#define AS5600_CMD_BURN_ANGLE       (0x80U) /* burns ZPOS + MPOS               */
#define AS5600_CMD_BURN_SETTING     (0x40U) /* burns MANG + CONF               */

/* OTP reload sequence written to 0xFF to read back burned content (Fig. 24). */
#define AS5600_CMD_OTP_RELOAD_1     (0x01U)
#define AS5600_CMD_OTP_RELOAD_2     (0x11U)
#define AS5600_CMD_OTP_RELOAD_3     (0x10U)

/* ---- STATUS register bit masks (Fig. 23) -------------------------------- */
#define AS5600_STATUS_MH_Msk        (1U << 3) /* AGC min gain overflow: magnet too strong */
#define AS5600_STATUS_ML_Msk        (1U << 4) /* AGC max gain overflow: magnet too weak   */
#define AS5600_STATUS_MD_Msk        (1U << 5) /* magnet detected                          */

/* ---- CONF register field positions/masks (16-bit view: H<<8 | L) --------- */
#define AS5600_CONF_PM_Pos          (0U)
#define AS5600_CONF_PM_Msk          (0x3U << AS5600_CONF_PM_Pos)   /* power mode      */
#define AS5600_CONF_HYST_Pos        (2U)
#define AS5600_CONF_HYST_Msk        (0x3U << AS5600_CONF_HYST_Pos) /* hysteresis      */
#define AS5600_CONF_OUTS_Pos        (4U)
#define AS5600_CONF_OUTS_Msk        (0x3U << AS5600_CONF_OUTS_Pos) /* output stage    */
#define AS5600_CONF_PWMF_Pos        (6U)
#define AS5600_CONF_PWMF_Msk        (0x3U << AS5600_CONF_PWMF_Pos) /* PWM frequency   */
#define AS5600_CONF_SF_Pos          (8U)
#define AS5600_CONF_SF_Msk          (0x3U << AS5600_CONF_SF_Pos)   /* slow filter     */
#define AS5600_CONF_FTH_Pos         (10U)
#define AS5600_CONF_FTH_Msk         (0x7U << AS5600_CONF_FTH_Pos)  /* fast filter thr.*/
#define AS5600_CONF_WD_Pos          (13U)
#define AS5600_CONF_WD_Msk          (0x1U << AS5600_CONF_WD_Pos)   /* watchdog        */

/* ---- Fixed device characteristics --------------------------------------- */
#define AS5600_RESOLUTION_BITS      (12U)
#define AS5600_COUNTS_PER_REV       (4096U)   /* 2^12 */
#define AS5600_ANGLE_MASK           (0x0FFFU) /* 12-bit value mask */
#define AS5600_MIN_ANGULAR_RANGE_DEG (18U)    /* programmable range must exceed this */

#endif /* AS5600_REGS_H */
