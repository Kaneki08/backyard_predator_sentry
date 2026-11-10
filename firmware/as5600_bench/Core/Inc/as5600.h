/**
 * @file    as5600.h
 * @brief   Instance-based, read-only driver for the ams-OSRAM AS5600 12-bit
 *          magnetic rotary position sensor (I2C, fixed 7-bit address 0x36).
 *
 * Design rules
 *  - No globals: every piece of state lives in an as5600_t instance, so a
 *    second sensor on a second I2C bus is simply a second instance.
 *  - READ-ONLY: this driver never writes to the device. In particular it never
 *    touches BURN (0xFF), which is one-time programmable and permanent, and it
 *    never writes ZPOS/MPOS/MANG/CONF. Zeroing is done in software (RAM only).
 *    The only I2C write ever issued is the 1-byte register-pointer write that
 *    is part of every HAL_I2C_Mem_Read() transaction.
 *  - Every transaction has a timeout. Errors are reported, never hang. The
 *    driver tracks consecutive failures, marks the sensor offline, and
 *    as5600_service() performs periodic bus recovery + re-probe so the sensor
 *    comes back automatically when replugged.
 *  - Angle math is integer-only (no float printf needed by callers).
 *
 * Register map: ams AS5600 datasheet (DS000365), "Register Map" table and
 * "Register Description" section.
 */
#ifndef AS5600_H
#define AS5600_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* ------------------------------------------------------------------------- */
/* Device constants                                                          */
/* ------------------------------------------------------------------------- */
#define AS5600_I2C_ADDR_7BIT      0x36u
#define AS5600_I2C_ADDR_HAL       (AS5600_I2C_ADDR_7BIT << 1)  /* HAL wants 8-bit form */

#define AS5600_COUNTS_PER_REV     4096
#define AS5600_COUNTS_MASK        0x0FFFu

/* Register addresses (datasheet register map). */
#define AS5600_REG_ZMCO           0x00u  /* [1:0] number of ZPOS/MPOS burns done   */
#define AS5600_REG_ZPOS_H         0x01u  /* [3:0] zero position, high nibble       */
#define AS5600_REG_ZPOS_L         0x02u
#define AS5600_REG_MPOS_H         0x03u  /* [3:0] max position                     */
#define AS5600_REG_MPOS_L         0x04u
#define AS5600_REG_MANG_H         0x05u  /* [3:0] max angle                        */
#define AS5600_REG_MANG_L         0x06u
#define AS5600_REG_CONF_H         0x07u  /* [5:0] WD, FTH, SF                      */
#define AS5600_REG_CONF_L         0x08u  /* [7:0] PWMF, OUTS, HYST, PM             */
#define AS5600_REG_STATUS         0x0Bu  /* MD, ML, MH                             */
#define AS5600_REG_RAW_ANGLE_H    0x0Cu  /* [3:0] unscaled angle, high nibble      */
#define AS5600_REG_RAW_ANGLE_L    0x0Du
#define AS5600_REG_ANGLE_H        0x0Eu  /* [3:0] scaled angle (ZPOS/MPOS/MANG)    */
#define AS5600_REG_ANGLE_L        0x0Fu
#define AS5600_REG_AGC            0x1Au  /* automatic gain control value           */
#define AS5600_REG_MAGNITUDE_H    0x1Bu  /* [3:0] CORDIC magnitude, high nibble    */
#define AS5600_REG_MAGNITUDE_L    0x1Cu
/* 0xFF is BURN. Deliberately NOT given a macro: this driver must never write it. */

/* STATUS (0x0B) bits. */
#define AS5600_STATUS_MH          (1u << 3)  /* AGC minimum gain overflow: magnet too strong */
#define AS5600_STATUS_ML          (1u << 4)  /* AGC maximum gain overflow: magnet too weak   */
#define AS5600_STATUS_MD          (1u << 5)  /* magnet detected                               */

/* CONF word (CONF_H << 8 | CONF_L) field positions. */
#define AS5600_CONF_PM_POS        0u   /* [1:0]   power mode               */
#define AS5600_CONF_HYST_POS      2u   /* [3:2]   hysteresis               */
#define AS5600_CONF_OUTS_POS      4u   /* [5:4]   output stage             */
#define AS5600_CONF_PWMF_POS      6u   /* [7:6]   PWM frequency            */
#define AS5600_CONF_SF_POS        8u   /* [9:8]   slow filter              */
#define AS5600_CONF_FTH_POS       10u  /* [12:10] fast filter threshold    */
#define AS5600_CONF_WD_POS        13u  /* [13]    watchdog                 */

/* ------------------------------------------------------------------------- */
/* Types                                                                     */
/* ------------------------------------------------------------------------- */
typedef enum {
    AS5600_OK = 0,
    AS5600_ERR_NACK,      /* device did not acknowledge (unplugged / wrong address) */
    AS5600_ERR_TIMEOUT,   /* transaction did not complete within timeout_ms        */
    AS5600_ERR_BUS,       /* bus error / arbitration lost / bus stuck busy          */
    AS5600_ERR_DATA,      /* transaction OK but data impossible (e.g. raw > 4095)   */
    AS5600_ERR_OFFLINE,   /* call skipped because the sensor is marked offline      */
    AS5600_ERR_PARAM      /* bad argument                                           */
} as5600_status_t;

typedef enum {
    AS5600_MAGNET_NOT_DETECTED = 0,  /* MD = 0                                   */
    AS5600_MAGNET_TOO_WEAK,          /* MD = 1, ML = 1 -> move magnet closer     */
    AS5600_MAGNET_TOO_STRONG,        /* MD = 1, MH = 1 -> move magnet farther    */
    AS5600_MAGNET_OK                 /* MD = 1, ML = 0, MH = 0                   */
} as5600_magnet_t;

typedef enum {
    AS5600_SVC_IDLE = 0,       /* nothing to do (online, or retry not due yet) */
    AS5600_SVC_RETRY_FAILED,   /* offline, a recovery attempt was made, still offline */
    AS5600_SVC_RECOVERED       /* offline -> online transition happened this call */
} as5600_svc_t;

/** Static configuration. Fill in, then pass to as5600_init(). */
typedef struct {
    I2C_HandleTypeDef *hi2c;     /* HAL handle, already initialised by MX_I2Cx_Init() */
    GPIO_TypeDef *scl_port;      /* SCL pin, used only for bus recovery bit-banging   */
    uint16_t      scl_pin;
    GPIO_TypeDef *sda_port;      /* SDA pin, used only for bus recovery               */
    uint16_t      sda_pin;
    uint32_t timeout_ms;         /* per-transaction timeout. HAL ticks are 1 ms, so use >= 2 */
    uint8_t  offline_after;      /* consecutive failures before marking offline (>= 1)       */
    uint32_t retry_period_ms;    /* how often as5600_service() retries while offline         */
    uint16_t agc_full_scale;     /* AGC range: 255 in 5 V mode, 128 in 3.3 V mode            */
    bool     use_fast_read;      /* see as5600_read_raw_angle_fast()                         */
} as5600_config_t;

/** One angle sample produced by as5600_update(). */
typedef struct {
    uint32_t t_us;        /* caller-supplied timestamp (free-running microseconds) */
    uint16_t raw;         /* RAW ANGLE register, 0..4095                           */
    uint16_t zeroed;      /* (raw - software zero) mod 4096, 0..4095               */
    int32_t  unwrapped;   /* multi-turn position in counts relative to the software
                             zero; 4096 counts per revolution, signed              */
} as5600_sample_t;

/** Decoded CONF register. */
typedef struct {
    uint8_t pm, hyst, outs, pwmf, sf, fth, wd;
} as5600_conf_t;

/** Full register snapshot for diagnostics (all reads, no writes). */
typedef struct {
    uint8_t  zmco;
    uint16_t zpos, mpos, mang;
    uint16_t conf_word;
    as5600_conf_t conf;
    uint8_t  status;
    uint16_t raw_angle;
    uint16_t angle;
    uint8_t  agc;
    uint16_t magnitude;
} as5600_diag_t;

/** Driver instance. Treat fields as read-only outside as5600.c. */
typedef struct {
    as5600_config_t cfg;

    /* --- position tracking (software only, RAM) --- */
    uint16_t zero_raw;       /* software zero, in raw counts                      */
    int32_t  accum;          /* unwrapped counts relative to zero_raw             */
    uint16_t last_raw;       /* raw value of the previous successful update       */
    bool     have_last;      /* false until the first successful update          */
    as5600_sample_t last;    /* last successful sample                            */

    /* --- link health --- */
    bool     online;
    bool     ptr_at_raw;     /* device register pointer known to sit at 0x0C      */
    uint8_t  consec_errors;
    uint32_t total_errors;
    uint32_t recoveries;     /* successful offline -> online transitions          */
    uint32_t retry_attempts; /* recovery attempts since going offline             */
    uint32_t offline_since_ms;
    uint32_t last_retry_ms;
    as5600_status_t last_error;
    uint32_t last_hal_error; /* hi2c->ErrorCode captured at the last failure      */
    bool     last_recovery_sda_stuck; /* SDA was low when recovery started        */
    bool     last_recovery_bus_free;  /* both lines high after recovery           */
    bool     discontinuity;  /* set on reconnect: rotation while offline unknown  */
} as5600_t;

/* ------------------------------------------------------------------------- */
/* Lifecycle / link management                                               */
/* ------------------------------------------------------------------------- */

/** Store config and reset state. No I2C traffic. Sensor starts "online" so the
 *  first read is attempted immediately. */
as5600_status_t as5600_init(as5600_t *dev, const as5600_config_t *cfg);

/** Address-only probe (HAL_I2C_IsDeviceReady). AS5600_OK means ACK. Does not
 *  change online/offline state. */
as5600_status_t as5600_probe(as5600_t *dev);

/** I2C bus recovery: HAL de-init, bit-bang up to 9 SCL pulses while SDA is held
 *  low by a slave, generate a STOP, then HAL re-init. Safe to call any time. */
as5600_status_t as5600_bus_recover(as5600_t *dev);

/** Call regularly from the main loop. While offline, every retry_period_ms
 *  (measured with HAL_GetTick) it runs bus recovery + probe + one raw read; on
 *  success the sensor goes back online, the unwrap continues by the shortest
 *  path from the last known angle and `discontinuity` is set (rotation that
 *  happened while unplugged cannot be known). */
as5600_svc_t as5600_service(as5600_t *dev);

static inline bool as5600_is_online(const as5600_t *dev) { return dev->online; }

/** Force the offline state (e.g. boot-time probe got no ACK) so that
 *  as5600_service() starts its periodic recovery on its next call. */
void as5600_set_offline(as5600_t *dev);

/* ------------------------------------------------------------------------- */
/* Register reads (each one is a single bounded I2C transaction)            */
/* ------------------------------------------------------------------------- */
as5600_status_t as5600_read_raw_angle(as5600_t *dev, uint16_t *raw);
as5600_status_t as5600_read_angle(as5600_t *dev, uint16_t *angle);
as5600_status_t as5600_read_status(as5600_t *dev, uint8_t *status);
as5600_status_t as5600_read_agc(as5600_t *dev, uint8_t *agc);
as5600_status_t as5600_read_magnitude(as5600_t *dev, uint16_t *magnitude);
as5600_status_t as5600_read_conf(as5600_t *dev, uint16_t *conf_word);
as5600_status_t as5600_read_diag(as5600_t *dev, as5600_diag_t *d);

/** "Fast" raw-angle read: a plain 2-byte receive with no register-pointer
 *  write. Valid only because the AS5600 does not advance its address pointer
 *  past RAW ANGLE once it points at RAW ANGLE high (datasheet: automatic
 *  increment of the address pointer for ANGLE, RAW ANGLE and MAGNITUDE).
 *  The driver tracks whether the pointer is known to be at 0x0C and falls back
 *  to a full register read otherwise (after any other register read, any error,
 *  or a reconnect). */
as5600_status_t as5600_read_raw_angle_fast(as5600_t *dev, uint16_t *raw);

/* ------------------------------------------------------------------------- */
/* Position tracking (software zero + multi-turn unwrap)                     */
/* ------------------------------------------------------------------------- */

/** Read RAW ANGLE once and advance the multi-turn unwrap. The unwrap takes the
 *  shortest path between consecutive samples, so the shaft must turn less than
 *  half a revolution (2048 counts) between calls. Updates link health. When
 *  offline it returns AS5600_ERR_OFFLINE without bus traffic. */
as5600_status_t as5600_update(as5600_t *dev, uint32_t t_us, as5600_sample_t *out);

/** Make the current position the software zero (RAM only, never written to
 *  the device). Reads RAW ANGLE fresh; unwrapped position becomes 0. */
as5600_status_t as5600_zero_here(as5600_t *dev, uint16_t *zero_raw_out);

/* ------------------------------------------------------------------------- */
/* Pure helpers (no I/O)                                                     */
/* ------------------------------------------------------------------------- */
as5600_magnet_t as5600_magnet_from_status(uint8_t status);
const char *as5600_magnet_short(as5600_magnet_t m);   /* "OK", "WEAK", ...      */
const char *as5600_magnet_advice(as5600_magnet_t m);  /* plain-English verdict */
const char *as5600_status_str(as5600_status_t s);
void as5600_decode_conf(uint16_t conf_word, as5600_conf_t *c);

/** Fold a count difference into [-2048, +2047] (shortest signed path). */
static inline int32_t as5600_fold(int32_t d)
{
    d &= (AS5600_COUNTS_PER_REV - 1);                /* 0..4095 */
    if (d >= AS5600_COUNTS_PER_REV / 2) d -= AS5600_COUNTS_PER_REV;
    return d;
}

/** counts -> hundredths of a degree, rounded half away from zero. 64-bit so
 *  multi-turn positions beyond +/-14.5 turns do not overflow. */
int64_t as5600_counts_to_cdeg(int32_t counts);

/** Velocity between two samples, in tenths of a degree per second. */
int32_t as5600_velocity_ddps(const as5600_sample_t *a, const as5600_sample_t *b);

#ifdef __cplusplus
}
#endif
#endif /* AS5600_H */
