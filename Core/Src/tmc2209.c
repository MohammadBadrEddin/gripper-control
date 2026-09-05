#include "tmc2209.h"
#include <string.h>

#define TMC_SYNC     0x05
#define TMC_TIMEOUT  20     /* ms per UART transaction */
#define TMC_MUTEX_TIMEOUT	pdMS_TO_TICKS(50)	// included for mutex (02.09.2026)

//volatile uint32_t uart_success = 0;

/* --------------------------------------------------------------------------
 * CRC8-ATM, polynomial 0x07, initial value 0, applied LSB->MSB.
 * Transcribed from the datasheet's own C example (sec. 4.2).
 * -------------------------------------------------------------------------- */
static uint8_t tmc_crc(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        uint8_t byte = data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if ((crc >> 7) ^ (byte & 0x01)) {
                crc = (uint8_t)((crc << 1) ^ 0x07);
            } else {
                crc = (uint8_t)(crc << 1);
            }
            byte >>= 1;
        }
    }
    return crc;
}

/* --------------------------------------------------------------------------
 * Write: 8-byte datagram. Register address gets bit 7 set to mark a write.
 * -------------------------------------------------------------------------- */
void TMC2209_Write(TMC2209 *drv, uint8_t reg, uint32_t val)
{
    uint8_t d[8];
    d[0] = TMC_SYNC;
    d[1] = drv->addr;
    d[2] = reg | 0x80;                  /* bit 7 = write */
    d[3] = (uint8_t)(val >> 24);        /* 32-bit data, highest byte first */
    d[4] = (uint8_t)(val >> 16);
    d[5] = (uint8_t)(val >> 8);
    d[6] = (uint8_t)(val);
    d[7] = tmc_crc(d, 7);

    if(xSemaphoreTake(drv->mutex, TMC_MUTEX_TIMEOUT) != pdTRUE) {		// included for mutex (02.09.2026)
    	return;		// Bus currently in use, no writing
    }

    HAL_HalfDuplex_EnableTransmitter(drv->huart);
    HAL_UART_Transmit(drv->huart, d, 8, TMC_TIMEOUT);

    xSemaphoreGive(drv->mutex);											// included for mutex (02.09.2026)
}

/* --------------------------------------------------------------------------
 * Read: send a 4-byte request, receive an 8-byte reply.
 *
 * On a single-wire half-duplex line the transmitted request is echoed back,
 * so we read back and discard those 4 bytes before parsing the reply.
 * -------------------------------------------------------------------------- */
bool TMC2209_Read(TMC2209 *drv, uint8_t reg, uint32_t *val)
{
    uint8_t req[4];
    req[0] = TMC_SYNC;
    req[1] = drv->addr;
    req[2] = reg & 0x7F;                /* bit 7 clear = read */
    req[3] = tmc_crc(req, 3);

    if(xSemaphoreTake(drv->mutex, TMC_MUTEX_TIMEOUT) != pdTRUE) {		// included for mutex (02.09.2026)
    	return false;	// Bus currently in use
    }

    bool ok = false;

    HAL_HalfDuplex_EnableTransmitter(drv->huart);
	if (HAL_UART_Transmit(drv->huart, req, 4, TMC_TIMEOUT) == HAL_OK) {

		// Verzögerung
		__HAL_UART_CLEAR_FLAG(drv->huart, UART_CLEAR_TCF);   // TC-Flag sauber löschen

		HAL_HalfDuplex_EnableReceiver(drv->huart);

		/* Discard our own echoed request bytes (single-wire artifact). */
		uint8_t echo[4];
		HAL_UART_Receive(drv->huart, echo, 4, TMC_TIMEOUT);

		uint8_t rx[8];

		/* commented for mutex (02.09.2026)
		if (HAL_UART_Receive(drv->huart, rx, 8, TMC_TIMEOUT) != HAL_OK) {
			return false;
		}
		// Validate: sync nibble, master address 0xFF, register, CRC.
		if (rx[0] != TMC_SYNC || rx[1] != 0xFF || rx[2] != (reg & 0x7F)) {
			return false;
		}
		if (tmc_crc(rx, 7) != rx[7]) {
			return false;
		}

		*val = ((uint32_t)rx[3] << 24) | ((uint32_t)rx[4] << 16) |
			   ((uint32_t)rx[5] << 8)  |  (uint32_t)rx[6];
		return true;

		*/

		if (HAL_UART_Receive(drv->huart, rx, 8, TMC_TIMEOUT) == HAL_OK) {
			if (rx[0] == TMC_SYNC && rx[1] == 0xFF && rx[2] == (reg & 0x7F)
				&& tmc_crc(rx, 7) == rx[7]) {
				*val = ((uint32_t)rx[3] << 24) | ((uint32_t)rx[4] << 16) |
					   ((uint32_t)rx[5] << 8)  |  (uint32_t)rx[6];
				ok = true;
			}
		}
    }
    xSemaphoreGive(drv->mutex);
    return ok;
}

/* -------------------------------------------------------------------------- */

bool TMC2209_IsConnected(TMC2209 *drv)
{
    uint32_t dummy;
    return TMC2209_Read(drv, TMC_IFCNT, &dummy);
}

bool TMC2209_Init(TMC2209 *drv, UART_HandleTypeDef *huart, uint8_t addr)
{
    drv->huart = huart;
    drv->addr  = addr;

    // included for mutex (02.09.2026)
    drv->mutex = xSemaphoreCreateMutex();
    if(drv->mutex == NULL) {
    	return false;			// heap depleted => check TOTAL_HEAP_SIZE!
    }

//    uint32_t before = 0;
//    bool have_ifcnt = TMC2209_Read(drv, TMC_IFCNT, &before);

    /* GCONF: pdn_disable (bit 6) is REQUIRED when using UART, otherwise the
     * PDN_UART pin function interferes. mstep_reg_select (bit 7) lets us set
     * microsteps by register instead of the MS1/MS2 pins.
     * I_scale_analog (bit 0) stays set so VREF still scales the current. */

    // TMC2209_Write(drv, TMC_GCONF, (1u << 6) | (1u << 7) | (1u << 0));
    TMC2209_Write(drv, TMC_GCONF, (1u << 6) | (1u << 7));   	// Bit 0=0 => Stromreferenz kommt aus interner Versorgung, IHOLD_IRUN bestimmt Strom direkt & vollständig per Register

    /* Clear any latched reset/error flags (write 1 to clear). */
    TMC2209_Write(drv, TMC_GSTAT, 0x07);

    /* TPOWERDOWN: datasheet notes a minimum of 2 for StealthChop autotuning. */
    TMC2209_Write(drv, TMC_TPOWERDOWN, 20);

    /* CHOPCONF: TOFF=3 enables the chopper (TOFF=0 means driver off).
     * Bit 17 = vsense, bit 28 = intpol (interpolate to 256 usteps).
     * MRES defaults to 0 here = 256 microsteps; SetMicrosteps overrides it. */
    TMC2209_Write(drv, TMC_CHOPCONF, (1u << 28) | (1u << 17) | 0x03);

    /* Conservative default current: run 16/32, hold 8/32. */
    TMC2209_SetCurrent(drv, 16, 8);

    /* Verify the writes actually landed: IFCNT increments per accepted write. */
    uint32_t gconf = 0;
    if (!TMC2209_Read(drv, TMC_GCONF, &gconf)) {
        return false;		// Chip antwortet nicht --> Hardware-Problem!
    }

//    uart_success = 1;

    return (gconf & ((1u << 6) | (1u << 7))) == ((1u << 6) | (1u << 7));
}

void TMC2209_SetCurrent(TMC2209 *drv, uint8_t run, uint8_t hold)
{
    if (run  > 31) run  = 31;
    if (hold > 31) hold = 31;
    /* IHOLD bits 4:0, IRUN bits 12:8, IHOLDDELAY bits 19:16 */
    uint32_t v = ((uint32_t)hold) |
                 ((uint32_t)run << 8) |
                 ((uint32_t)6   << 16);			// IHOLDDELAY fixed on 6
    TMC2209_Write(drv, TMC_IHOLD_IRUN, v);
}

void TMC2209_SetMicrosteps(TMC2209 *drv, uint16_t usteps)
{
    /* MRES is CHOPCONF bits 27:24; microsteps = 2^(8-MRES), MRES 0 = 256. */
    uint32_t mres;
    switch (usteps) {
        case 256: mres = 0; break;
        case 128: mres = 1; break;
        case  64: mres = 2; break;
        case  32: mres = 3; break;
        case  16: mres = 4; break;
        case   8: mres = 5; break;
        case   4: mres = 6; break;
        case   2: mres = 7; break;
        case   1: mres = 8; break;
        default:  mres = 4; break;   /* fall back to 16 */
    }

    uint32_t chop;
    if (!TMC2209_Read(drv, TMC_CHOPCONF, &chop)) {
        chop = (1u << 28) | (1u << 17) | 0x03;   /* same default as init */
    }
    chop &= ~(0xFu << 24);
    chop |= (mres << 24);
    TMC2209_Write(drv, TMC_CHOPCONF, chop);
}

void TMC2209_MoveVelocity(TMC2209 *drv, int32_t velocity)
{
    /* VACTUAL is 24-bit signed; mask to 24 bits (two's complement preserved). */
    TMC2209_Write(drv, TMC_VACTUAL, (uint32_t)velocity & 0x00FFFFFF);
}

void TMC2209_Stop(TMC2209 *drv)
{
    TMC2209_Write(drv, TMC_VACTUAL, 0);
}
