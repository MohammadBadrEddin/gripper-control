/**
 * @file    tmc2209_types.h
 * @brief   HAL-independent enums, configuration and status types for the
 *          TMC2209 driver.
 *
 * Mirrors the layering of as5600_types.h: nothing in here pulls in the STM32
 * HAL, so this header (and the CRC / datagram code that uses it) can be built
 * and unit tested on a host.
 */

#ifndef TMC2209_TYPES_H
#define TMC2209_TYPES_H

#include <stdint.h>
#include <stdbool.h>

/* ------------------------------------------------------------------------- */
/* Status / error codes                                                      */
/* ------------------------------------------------------------------------- */

typedef enum {
    TMC2209_OK = 0,
    TMC2209_ERR_PARAM,        /*!< NULL pointer or out-of-range argument      */
    TMC2209_ERR_TX,           /*!< HAL transmit failed                        */
    TMC2209_ERR_TIMEOUT,      /*!< No reply datagram within the timeout       */
    TMC2209_ERR_CRC,          /*!< Reply received but CRC8 mismatch           */
    TMC2209_ERR_FRAME,        /*!< No valid sync/master/reg header in reply   */
    TMC2209_ERR_VERSION,      /*!< IOIN.VERSION is not 0x21                   */
    TMC2209_ERR_WRITE_LOST,   /*!< IFCNT did not increment as expected        */
    TMC2209_ERR_BUSY,         /*!< Transfer already in progress               */
    TMC2209_ERR_DRIVER_FAULT  /*!< GSTAT.drv_err / DRV_STATUS error flag set  */
} TMC2209_Status;

/* ------------------------------------------------------------------------- */
/* Microstep resolution - CHOPCONF.MRES encoding (datasheet 5.5.1)           */
/* ------------------------------------------------------------------------- */

typedef enum {
    TMC2209_MRES_256      = 0,
    TMC2209_MRES_128      = 1,
    TMC2209_MRES_64       = 2,
    TMC2209_MRES_32       = 3,
    TMC2209_MRES_16       = 4,
    TMC2209_MRES_8        = 5,
    TMC2209_MRES_4        = 6,
    TMC2209_MRES_2        = 7,
    TMC2209_MRES_FULLSTEP = 8
} TMC2209_Microsteps;

/* ------------------------------------------------------------------------- */
/* Chopper mode - GCONF.en_SpreadCycle                                       */
/* ------------------------------------------------------------------------- */

typedef enum {
    TMC2209_CHOPPER_STEALTHCHOP = 0,  /*!< quiet, StallGuard4 usable         */
    TMC2209_CHOPPER_SPREADCYCLE = 1   /*!< dynamic, no StallGuard4           */
} TMC2209_ChopperMode;

/* PWMCONF.freewheel - standstill behaviour when IHOLD = 0 (datasheet 5.5.2) */
typedef enum {
    TMC2209_FREEWHEEL_NORMAL   = 0,
    TMC2209_FREEWHEEL_FREEWHEEL = 1,
    TMC2209_FREEWHEEL_SHORT_LS = 2,
    TMC2209_FREEWHEEL_SHORT_HS = 3
} TMC2209_Freewheel;

/* ------------------------------------------------------------------------- */
/* Configuration                                                             */
/* ------------------------------------------------------------------------- */

typedef struct {
    /* --- interface ------------------------------------------------------- */
    uint8_t  node_address;     /*!< 0..3, set by MS1/MS2 strapping           */
    uint8_t  senddelay;        /*!< NODECONF.SENDDELAY, 0..15 (use >=2 multi)*/
    uint32_t timeout_ms;       /*!< reply timeout for a read access          */
    uint8_t  retries;          /*!< retries per read before giving up        */

    /* --- current control ------------------------------------------------- */
    uint16_t rsense_mohm;      /*!< external sense resistor, e.g. 110 mOhm   */
    uint16_t irun_ma;          /*!< target RMS run current per coil, in mA   */
    uint8_t  ihold_percent;    /*!< hold current as % of irun_ma             */
    uint8_t  iholddelay;       /*!< 0..15, current decay steps (5.2)         */
    uint8_t  tpowerdown;       /*!< 0..255, >=2 required for StealthChop AT  */
    bool     use_vref;         /*!< GCONF.i_scale_analog: VREF scales current*/
    bool     internal_rsense;  /*!< GCONF.internal_Rsense (RDSon sensing)    */

    /* --- motion / resolution --------------------------------------------- */
    TMC2209_Microsteps microsteps;
    bool     interpolate;      /*!< CHOPCONF.intpol -> 256 uStep MicroPlyer  */
    bool     invert_shaft;     /*!< GCONF.shaft                              */
    bool     double_edge;      /*!< CHOPCONF.dedge (STEP on both edges)      */

    /* --- chopper --------------------------------------------------------- */
    TMC2209_ChopperMode chopper;
    uint8_t  toff;             /*!< 1..15, 0 disables the power stage        */
    uint8_t  tbl;              /*!< blank time 0..3, 1 or 2 recommended      */
    uint8_t  hstrt;            /*!< 0..7                                     */
    uint8_t  hend;             /*!< 0..15 (register value, -3..+12 effective)*/
    uint32_t tpwmthrs;         /*!< 20 bit, 0 = StealthChop only             */

    /* --- StealthChop PWM ------------------------------------------------- */
    bool     pwm_autoscale;
    bool     pwm_autograd;
    uint8_t  pwm_freq;         /*!< 0..3, see datasheet table 6.1            */
    uint8_t  pwm_reg;          /*!< 1..15 regulation gradient                */
    uint8_t  pwm_lim;          /*!< 0..15, default 12                        */
    uint8_t  pwm_ofs;          /*!< init value, reset default 36             */
    uint8_t  pwm_grad;         /*!< init value for automatic tuning          */
    TMC2209_Freewheel freewheel;

    /* --- StallGuard4 / DIAG ---------------------------------------------- */
    bool     stallguard_enable;
    uint8_t  sgthrs;           /*!< 0..255, stall if SG_RESULT <= 2*SGTHRS   */
    uint32_t tcoolthrs;        /*!< 20 bit, lower velocity limit for DIAG    */

    /* --- optional board hooks (keep the driver pin-agnostic) -------------- */
    void (*set_enable)(bool enable);   /*!< drive TMC_EN (active low) or NULL*/
    void (*delay_ms)(uint32_t ms);     /*!< NULL -> HAL_Delay / vTaskDelay   */
} TMC2209_Config;

/* ------------------------------------------------------------------------- */
/* Live driver status (decoded DRV_STATUS + GSTAT)                           */
/* ------------------------------------------------------------------------- */

typedef struct {
    uint32_t raw_drv_status;
    uint8_t  raw_gstat;

    /* GSTAT */
    bool reset_flag;      /*!< IC was reset since last GSTAT read            */
    bool drv_err;         /*!< shutdown due to overtemp / short             */
    bool uv_cp;           /*!< charge pump undervoltage                     */

    /* DRV_STATUS */
    bool otpw;            /*!< overtemperature prewarning                   */
    bool ot;              /*!< overtemperature shutdown                     */
    bool short_gnd_a;
    bool short_gnd_b;
    bool short_vs_a;
    bool short_vs_b;
    bool open_load_a;
    bool open_load_b;
    bool t120, t143, t150, t157;
    bool stealth;         /*!< 1 = running StealthChop                      */
    bool standstill;      /*!< stst                                         */
    uint8_t cs_actual;    /*!< actual current scale 0..31                   */
} TMC2209_DrvStatus;

/* ------------------------------------------------------------------------- */
/* Handle                                                                    */
/* ------------------------------------------------------------------------- */

/* Forward declared so this header stays HAL free; the driver header supplies
 * the concrete UART handle type. */
struct UART_HandleTypeDef;

typedef struct {
    void            *huart;        /*!< UART_HandleTypeDef* (half-duplex)    */
    TMC2209_Config   cfg;

    /* Shadow copies - several registers are write-only, so read-modify-write
     * is only possible against a master-side shadow (datasheet 1.2.1).      */
    uint32_t shadow_gconf;
    uint32_t shadow_chopconf;
    uint32_t shadow_ihold_irun;
    uint32_t shadow_pwmconf;
    uint32_t shadow_coolconf;
    int32_t  shadow_vactual;

    uint8_t  version;              /*!< IOIN.VERSION read at init, 0x21      */
    uint8_t  ifcnt;                /*!< last IFCNT value seen                */
    uint8_t  cs_run;               /*!< resolved IRUN current scale          */
    uint8_t  cs_hold;              /*!< resolved IHOLD current scale         */
    bool     vsense;               /*!< resolved CHOPCONF.vsense             */
    bool     initialised;

    /* Diagnostics counters, same idea as the AS5600 fault counters. */
    uint32_t crc_errors;
    uint32_t timeouts;
    uint32_t lost_writes;

    /* Runtime (interrupt mode) transfer state. */
    void    *task;                 /*!< TaskHandle_t waiting for the notify  */
    volatile uint8_t  xfer_state;
    uint8_t  tx_buf[8];
    uint8_t  rx_buf[24];           /*!< reply + room for echoed TX bytes     */
    volatile uint16_t rx_len;
} TMC2209_Handle;

#endif /* TMC2209_TYPES_H */
