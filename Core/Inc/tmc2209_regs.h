/**
 * @file    tmc2209_regs.h
 * @brief   TMC2209 register map, bitfields and UART datagram constants.
 *
 * Every address, bit position and reset default in this file was taken from:
 *   TMC2209 DATASHEET (Rev. 1.09 / 2023-FEB-16)
 *     ch. 4   UART Single Wire Interface   (datagram framing, CRC)
 *     ch. 5   Register Map                 (addresses, bitfields)
 *     ch. 9   Motor Current Control        (IRUN/IHOLD, VFS)
 *
 * Section references are given per block so they can be re-checked quickly.
 * HAL independent - safe to compile on a host for unit tests.
 */

#ifndef TMC2209_REGS_H
#define TMC2209_REGS_H

#include <stdint.h>

/* ------------------------------------------------------------------------- */
/* UART datagram framing (datasheet 4.1)                                     */
/* ------------------------------------------------------------------------- */

/* Sync nibble is %1010 sent LSB first in the first byte -> 0x05.
 * Bits 4..7 of that byte are "reserved / don't care" but ARE included in CRC. */
#define TMC2209_SYNC                 0x05u

/* Read replies are addressed to the master using %11111111 (datasheet 4.1.2). */
#define TMC2209_MASTER_ADDR          0xFFu

/* Bit 7 of the register address byte: 1 = write, 0 = read (datasheet 4.1). */
#define TMC2209_WRITE_FLAG           0x80u
#define TMC2209_REG_ADDR_MASK        0x7Fu

#define TMC2209_WRITE_DATAGRAM_LEN   8u   /* sync, addr, reg|0x80, 4x data, crc */
#define TMC2209_READ_REQUEST_LEN     4u   /* sync, addr, reg,      crc          */
#define TMC2209_READ_REPLY_LEN       8u   /* sync, 0xFF, reg, 4x data, crc      */

/* CRC8-ATM, polynomial x^8 + x^2 + x^1 + x^0, init 0, applied LSB..MSB
 * including sync and address byte (datasheet 4.2). */
#define TMC2209_CRC_POLY             0x07u

/* Node address is set by MS1 (bit0) / MS2 (bit1), range 0..3 (datasheet 4.1). */
#define TMC2209_NODE_ADDR_MAX        3u

/* ------------------------------------------------------------------------- */
/* Register addresses (datasheet 5.1 - 5.5)                                  */
/* ------------------------------------------------------------------------- */

/* General configuration registers 0x00..0x0F */
#define TMC2209_REG_GCONF            0x00u  /* RW,  10 bit */
#define TMC2209_REG_GSTAT            0x01u  /* R+WC, 3 bit */
#define TMC2209_REG_IFCNT            0x02u  /* R,    8 bit */
#define TMC2209_REG_NODECONF         0x03u  /* W,    4 bit (a.k.a. SLAVECONF) */
#define TMC2209_REG_OTP_PROG         0x04u  /* W,   16 bit */
#define TMC2209_REG_OTP_READ         0x05u  /* R,   24 bit */
#define TMC2209_REG_IOIN             0x06u  /* R,   10+8 bit */
#define TMC2209_REG_FACTORY_CONF     0x07u  /* RW,  5+2 bit */

/* Velocity dependent control 0x10..0x1F, 0x22 */
#define TMC2209_REG_IHOLD_IRUN       0x10u  /* W,  5+5+4 bit */
#define TMC2209_REG_TPOWERDOWN       0x11u  /* W,    8 bit */
#define TMC2209_REG_TSTEP            0x12u  /* R,   20 bit */
#define TMC2209_REG_TPWMTHRS         0x13u  /* W,   20 bit */
#define TMC2209_REG_VACTUAL          0x22u  /* W,   24 bit, signed */

/* CoolStep / StallGuard 0x14, 0x40..0x42 */
#define TMC2209_REG_TCOOLTHRS        0x14u  /* W,   20 bit */
#define TMC2209_REG_SGTHRS           0x40u  /* W,    8 bit */
#define TMC2209_REG_SG_RESULT        0x41u  /* R,   10 bit */
#define TMC2209_REG_COOLCONF         0x42u  /* W,   16 bit */

/* Sequencer (read only) 0x60..0x6B */
#define TMC2209_REG_MSCNT            0x6Au  /* R,   10 bit */
#define TMC2209_REG_MSCURACT         0x6Bu  /* R,   9+9 bit */

/* Chopper control 0x6C..0x7F */
#define TMC2209_REG_CHOPCONF         0x6Cu  /* RW,  32 bit */
#define TMC2209_REG_DRV_STATUS       0x6Fu  /* R,   32 bit */
#define TMC2209_REG_PWMCONF          0x70u  /* RW,  22 bit */
#define TMC2209_REG_PWM_SCALE        0x71u  /* R,   9+8 bit */
#define TMC2209_REG_PWM_AUTO         0x72u  /* R,   8+8 bit */

/* Reset defaults explicitly documented in the register map (5.5).
 * All other registers reset to 0 unless loaded from OTP. */
#define TMC2209_CHOPCONF_RESET       0x10000053UL
#define TMC2209_PWMCONF_RESET        0xC10D0024UL

/* ------------------------------------------------------------------------- */
/* Generic field helpers                                                     */
/* ------------------------------------------------------------------------- */

#define TMC2209_FIELD_GET(reg, mask, shift) \
    (((uint32_t)(reg) & (uint32_t)(mask)) >> (shift))

#define TMC2209_FIELD_SET(reg, mask, shift, value)                       \
    ((((uint32_t)(reg)) & ~(uint32_t)(mask)) |                           \
     ((((uint32_t)(value)) << (shift)) & (uint32_t)(mask)))

/* ------------------------------------------------------------------------- */
/* GCONF  0x00 (datasheet 5.1)                                               */
/* ------------------------------------------------------------------------- */

#define TMC2209_GCONF_I_SCALE_ANALOG   (1UL << 0)  /* reset default = 1      */
#define TMC2209_GCONF_INTERNAL_RSENSE  (1UL << 1)  /* reset default from OTP */
#define TMC2209_GCONF_EN_SPREADCYCLE   (1UL << 2)  /* 1 = SpreadCycle        */
#define TMC2209_GCONF_SHAFT            (1UL << 3)  /* 1 = invert direction   */
#define TMC2209_GCONF_INDEX_OTPW       (1UL << 4)
#define TMC2209_GCONF_INDEX_STEP       (1UL << 5)
#define TMC2209_GCONF_PDN_DISABLE      (1UL << 6)  /* MUST be 1 for UART use */
#define TMC2209_GCONF_MSTEP_REG_SELECT (1UL << 7)  /* 1 = MRES from CHOPCONF */
#define TMC2209_GCONF_MULTISTEP_FILT   (1UL << 8)  /* reset default = 1      */
#define TMC2209_GCONF_TEST_MODE        (1UL << 9)  /* keep 0                 */

/* ------------------------------------------------------------------------- */
/* GSTAT  0x01 (R + write-1-to-clear)                                        */
/* ------------------------------------------------------------------------- */

#define TMC2209_GSTAT_RESET            (1UL << 0)
#define TMC2209_GSTAT_DRV_ERR          (1UL << 1)
#define TMC2209_GSTAT_UV_CP            (1UL << 2)  /* not latched            */
#define TMC2209_GSTAT_CLEAR_ALL        0x00000007UL

/* ------------------------------------------------------------------------- */
/* NODECONF 0x03 - SENDDELAY, bits 11..8 (datasheet 5.1 / 4.1.2)             */
/* Encoding: 0,1 -> 8 bit times; 2,3 -> 3*8; 4,5 -> 5*8; ... 14,15 -> 15*8.  */
/* Use >= 2 in a multi-node system.                                          */
/* ------------------------------------------------------------------------- */

#define TMC2209_NODECONF_SENDDELAY_MASK   (0x0FUL << 8)
#define TMC2209_NODECONF_SENDDELAY_SHIFT  8

/* ------------------------------------------------------------------------- */
/* IOIN 0x06 (datasheet 5.1)                                                 */
/* ------------------------------------------------------------------------- */

#define TMC2209_IOIN_ENN               (1UL << 0)
#define TMC2209_IOIN_MS1               (1UL << 2)
#define TMC2209_IOIN_MS2               (1UL << 3)
#define TMC2209_IOIN_DIAG              (1UL << 4)
#define TMC2209_IOIN_PDN_UART          (1UL << 6)
#define TMC2209_IOIN_STEP              (1UL << 7)
#define TMC2209_IOIN_SPREAD_EN         (1UL << 8)
#define TMC2209_IOIN_DIR               (1UL << 9)
#define TMC2209_IOIN_VERSION_MASK      (0xFFUL << 24)
#define TMC2209_IOIN_VERSION_SHIFT     24
#define TMC2209_IOIN_VERSION_TMC2209   0x21u  /* "first version of the IC"   */

/* ------------------------------------------------------------------------- */
/* IHOLD_IRUN 0x10 (datasheet 5.2)                                           */
/* ------------------------------------------------------------------------- */

#define TMC2209_IHOLD_MASK             (0x1FUL << 0)
#define TMC2209_IHOLD_SHIFT            0
#define TMC2209_IRUN_MASK              (0x1FUL << 8)
#define TMC2209_IRUN_SHIFT             8
#define TMC2209_IHOLDDELAY_MASK        (0x0FUL << 16)
#define TMC2209_IHOLDDELAY_SHIFT       16
#define TMC2209_CS_MAX                 31u

/* VACTUAL 0x22 is a 24 bit two's complement value, +-(2^23)-1 (5.2). */
#define TMC2209_VACTUAL_MASK           0x00FFFFFFUL
#define TMC2209_VACTUAL_MAX            8388607L
#define TMC2209_VACTUAL_MIN            (-8388607L)

/* TSTEP / TPWMTHRS / TCOOLTHRS are 20 bit. */
#define TMC2209_20BIT_MASK             0x000FFFFFUL

/* ------------------------------------------------------------------------- */
/* CHOPCONF 0x6C (datasheet 5.5.1)                                           */
/* ------------------------------------------------------------------------- */

#define TMC2209_CHOPCONF_TOFF_MASK     (0x0FUL << 0)
#define TMC2209_CHOPCONF_TOFF_SHIFT    0
#define TMC2209_CHOPCONF_HSTRT_MASK    (0x07UL << 4)
#define TMC2209_CHOPCONF_HSTRT_SHIFT   4
#define TMC2209_CHOPCONF_HEND_MASK     (0x0FUL << 7)
#define TMC2209_CHOPCONF_HEND_SHIFT    7
#define TMC2209_CHOPCONF_TBL_MASK      (0x03UL << 15)
#define TMC2209_CHOPCONF_TBL_SHIFT     15
#define TMC2209_CHOPCONF_VSENSE        (1UL << 17)
#define TMC2209_CHOPCONF_MRES_MASK     (0x0FUL << 24)
#define TMC2209_CHOPCONF_MRES_SHIFT    24
#define TMC2209_CHOPCONF_INTPOL        (1UL << 28)
#define TMC2209_CHOPCONF_DEDGE         (1UL << 29)
#define TMC2209_CHOPCONF_DISS2G        (1UL << 30)
#define TMC2209_CHOPCONF_DISS2VS       (1UL << 31)

/* Full scale sense voltage, datasheet 20.2 (VSRTL / VSRTH). */
#define TMC2209_VFS_LOW_SENS_MV        325u  /* vsense = 0 */
#define TMC2209_VFS_HIGH_SENS_MV       180u  /* vsense = 1 */

/* Internal resistance from BRxy pin to the sense comparator, datasheet 20.2. */
#define TMC2209_RSENSE_INTERNAL_MOHM   20u

/* ------------------------------------------------------------------------- */
/* PWMCONF 0x70 (datasheet 5.5.2)                                            */
/* ------------------------------------------------------------------------- */

#define TMC2209_PWMCONF_PWM_OFS_MASK   (0xFFUL << 0)
#define TMC2209_PWMCONF_PWM_OFS_SHIFT  0
#define TMC2209_PWMCONF_PWM_GRAD_MASK  (0xFFUL << 8)
#define TMC2209_PWMCONF_PWM_GRAD_SHIFT 8
#define TMC2209_PWMCONF_PWM_FREQ_MASK  (0x03UL << 16)
#define TMC2209_PWMCONF_PWM_FREQ_SHIFT 16
#define TMC2209_PWMCONF_AUTOSCALE      (1UL << 18)
#define TMC2209_PWMCONF_AUTOGRAD       (1UL << 19)
#define TMC2209_PWMCONF_FREEWHEEL_MASK (0x03UL << 20)
#define TMC2209_PWMCONF_FREEWHEEL_SHIFT 20
#define TMC2209_PWMCONF_PWM_REG_MASK   (0x0FUL << 24)
#define TMC2209_PWMCONF_PWM_REG_SHIFT  24
#define TMC2209_PWMCONF_PWM_LIM_MASK   (0x0FUL << 28)
#define TMC2209_PWMCONF_PWM_LIM_SHIFT  28

/* ------------------------------------------------------------------------- */
/* COOLCONF 0x42 (datasheet 5.3.1)                                           */
/* ------------------------------------------------------------------------- */

#define TMC2209_COOLCONF_SEMIN_MASK    (0x0FUL << 0)
#define TMC2209_COOLCONF_SEMIN_SHIFT   0
#define TMC2209_COOLCONF_SEUP_MASK     (0x03UL << 5)
#define TMC2209_COOLCONF_SEUP_SHIFT    5
#define TMC2209_COOLCONF_SEMAX_MASK    (0x0FUL << 8)
#define TMC2209_COOLCONF_SEMAX_SHIFT   8
#define TMC2209_COOLCONF_SEDN_MASK     (0x03UL << 13)
#define TMC2209_COOLCONF_SEDN_SHIFT    13
#define TMC2209_COOLCONF_SEIMIN        (1UL << 15)

/* ------------------------------------------------------------------------- */
/* DRV_STATUS 0x6F (datasheet 5.5.3)                                         */
/* ------------------------------------------------------------------------- */

#define TMC2209_DRV_STATUS_OTPW        (1UL << 0)
#define TMC2209_DRV_STATUS_OT          (1UL << 1)
#define TMC2209_DRV_STATUS_S2GA        (1UL << 2)
#define TMC2209_DRV_STATUS_S2GB        (1UL << 3)
#define TMC2209_DRV_STATUS_S2VSA       (1UL << 4)
#define TMC2209_DRV_STATUS_S2VSB       (1UL << 5)
#define TMC2209_DRV_STATUS_OLA         (1UL << 6)
#define TMC2209_DRV_STATUS_OLB         (1UL << 7)
#define TMC2209_DRV_STATUS_T120        (1UL << 8)
#define TMC2209_DRV_STATUS_T143        (1UL << 9)
#define TMC2209_DRV_STATUS_T150        (1UL << 10)
#define TMC2209_DRV_STATUS_T157        (1UL << 11)
#define TMC2209_DRV_STATUS_CSACTUAL_MASK  (0x1FUL << 16)
#define TMC2209_DRV_STATUS_CSACTUAL_SHIFT 16
#define TMC2209_DRV_STATUS_STEALTH     (1UL << 30)
#define TMC2209_DRV_STATUS_STST        (1UL << 31)

/* SG_RESULT 0x41: 10 bit, 0..510 in steps of 2 (bits 9 and 0 always read 0). */
#define TMC2209_SG_RESULT_MASK         0x03FFUL

/* MSCNT 0x6A: 0..1023 */
#define TMC2209_MSCNT_MASK             0x03FFUL

#endif /* TMC2209_REGS_H */
