/**
 * @file    as5600.c
 * @brief   Read-only AS5600 driver. See as5600.h for the design rules.
 *
 * SAFETY: there is intentionally no register-write function in this file.
 * The AS5600 BURN register (0xFF) and the ZPOS/MPOS/MANG/CONF registers are
 * never written. Every transaction is HAL_I2C_Mem_Read (pointer write + read),
 * HAL_I2C_Master_Receive (read only) or HAL_I2C_IsDeviceReady (address only).
 */
#include "as5600.h"
#include <string.h>

/* ------------------------------------------------------------------------- */
/* Internal helpers                                                          */
/* ------------------------------------------------------------------------- */

/* Map a HAL return code + hi2c->ErrorCode to a driver status. */
static as5600_status_t map_hal(HAL_StatusTypeDef hs, const I2C_HandleTypeDef *hi2c)
{
    if (hs == HAL_OK)      return AS5600_OK;
    if (hs == HAL_BUSY)    return AS5600_ERR_BUS;      /* peripheral or bus busy */
    if (hs == HAL_TIMEOUT) return AS5600_ERR_TIMEOUT;

    uint32_t ec = hi2c->ErrorCode;
    if (ec & HAL_I2C_ERROR_AF)                         return AS5600_ERR_NACK;
    if (ec & (HAL_I2C_ERROR_BERR | HAL_I2C_ERROR_ARLO)) return AS5600_ERR_BUS;
    if (ec & HAL_I2C_ERROR_TIMEOUT)                    return AS5600_ERR_TIMEOUT;
    return AS5600_ERR_BUS;
}

static void note_ok(as5600_t *dev)
{
    dev->consec_errors = 0;
}

static void note_error(as5600_t *dev, as5600_status_t st)
{
    dev->total_errors++;
    dev->last_error     = st;
    dev->last_hal_error = dev->cfg.hi2c->ErrorCode;
    dev->ptr_at_raw     = false;               /* pointer state now unknown */
    if (dev->consec_errors < 0xFFu) dev->consec_errors++;

    if (dev->online && dev->consec_errors >= dev->cfg.offline_after) {
        dev->online           = false;
        dev->offline_since_ms = HAL_GetTick();
        dev->retry_attempts   = 0;
        /* Make the first retry happen on the next as5600_service() call. */
        dev->last_retry_ms    = dev->offline_since_ms - dev->cfg.retry_period_ms;
    }
}

/* Read `len` bytes starting at register `reg`. One bounded transaction. */
static as5600_status_t rd(as5600_t *dev, uint8_t reg, uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef hs = HAL_I2C_Mem_Read(dev->cfg.hi2c, AS5600_I2C_ADDR_HAL, reg,
                                            I2C_MEMADD_SIZE_8BIT, buf, len,
                                            dev->cfg.timeout_ms);
    as5600_status_t st = map_hal(hs, dev->cfg.hi2c);
    if (st != AS5600_OK) {
        note_error(dev, st);
        return st;
    }
    note_ok(dev);
    /* The device pointer only stays parked at RAW ANGLE after a read that
     * started at RAW ANGLE high (see as5600_read_raw_angle_fast()). */
    dev->ptr_at_raw = (reg == AS5600_REG_RAW_ANGLE_H && len == 2u);
    return AS5600_OK;
}

/* Read a 12-bit big-endian value (high nibble register + low byte register). */
static as5600_status_t rd12(as5600_t *dev, uint8_t reg_h, uint16_t *out)
{
    uint8_t b[2];
    as5600_status_t st = rd(dev, reg_h, b, 2);
    if (st != AS5600_OK) return st;
    if (b[0] & 0xF0u) {                 /* bits 15:12 are always 0 on a real AS5600 */
        dev->ptr_at_raw = false;
        note_error(dev, AS5600_ERR_DATA);
        return AS5600_ERR_DATA;
    }
    *out = (uint16_t)(((uint16_t)b[0] << 8) | b[1]);
    return AS5600_OK;
}

/* Crude busy-wait used only for bit-banged bus recovery. Accuracy is not
 * important as long as it is not shorter than requested (keeps SCL <= 100 kHz). */
static void delay_us(uint32_t us)
{
    uint32_t loops = (SystemCoreClock / 1000000u) * us / 3u + 1u;  /* ~3+ cycles/iter */
    while (loops--) { __NOP(); }
}

static void gpio_od_output_high(GPIO_TypeDef *port, uint16_t pin)
{
    GPIO_InitTypeDef g = {0};
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);   /* preload ODR: no glitch low */
    g.Pin   = pin;
    g.Mode  = GPIO_MODE_OUTPUT_OD;
    g.Pull  = GPIO_NOPULL;                        /* external pull-ups only     */
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(port, &g);
}

static bool pin_high(GPIO_TypeDef *port, uint16_t pin)
{
    /* IDR reflects the real pin level even when configured as an output. */
    return HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET;
}

/* Pulse the RCC reset line of the I2C peripheral (belt and braces after DeInit). */
static void i2c_force_reset(I2C_TypeDef *inst)
{
#if defined(I2C1)
    if (inst == I2C1) { __HAL_RCC_I2C1_FORCE_RESET(); __HAL_RCC_I2C1_RELEASE_RESET(); }
#endif
#if defined(I2C2)
    if (inst == I2C2) { __HAL_RCC_I2C2_FORCE_RESET(); __HAL_RCC_I2C2_RELEASE_RESET(); }
#endif
#if defined(I2C3)
    if (inst == I2C3) { __HAL_RCC_I2C3_FORCE_RESET(); __HAL_RCC_I2C3_RELEASE_RESET(); }
#endif
#if defined(I2C4)
    if (inst == I2C4) { __HAL_RCC_I2C4_FORCE_RESET(); __HAL_RCC_I2C4_RELEASE_RESET(); }
#endif
}

/* ------------------------------------------------------------------------- */
/* Lifecycle / link management                                               */
/* ------------------------------------------------------------------------- */

as5600_status_t as5600_init(as5600_t *dev, const as5600_config_t *cfg)
{
    if (dev == NULL || cfg == NULL || cfg->hi2c == NULL ||
        cfg->scl_port == NULL || cfg->sda_port == NULL) {
        return AS5600_ERR_PARAM;
    }
    memset(dev, 0, sizeof(*dev));
    dev->cfg = *cfg;
    if (dev->cfg.timeout_ms < 2u)       dev->cfg.timeout_ms = 2u;  /* 1 ms tick granularity */
    if (dev->cfg.offline_after == 0u)   dev->cfg.offline_after = 1u;
    if (dev->cfg.retry_period_ms == 0u) dev->cfg.retry_period_ms = 500u;
    if (dev->cfg.agc_full_scale == 0u)  dev->cfg.agc_full_scale = 255u;
    dev->online     = true;
    dev->last_error = AS5600_OK;
    return AS5600_OK;
}

as5600_status_t as5600_probe(as5600_t *dev)
{
    /* NOTE (STM32L4 HAL): on NACK, HAL_I2C_IsDeviceReady returns HAL_ERROR with
     * ErrorCode = HAL_I2C_ERROR_TIMEOUT (it does not report AF), so any
     * HAL_ERROR here means "no ACK". HAL_BUSY means the bus is held. */
    HAL_StatusTypeDef hs = HAL_I2C_IsDeviceReady(dev->cfg.hi2c, AS5600_I2C_ADDR_HAL,
                                                 2u, dev->cfg.timeout_ms);
    dev->ptr_at_raw = false;
    if (hs == HAL_OK)   return AS5600_OK;
    if (hs == HAL_BUSY) return AS5600_ERR_BUS;
    return AS5600_ERR_NACK;
}

as5600_status_t as5600_bus_recover(as5600_t *dev)
{
    I2C_HandleTypeDef *h = dev->cfg.hi2c;
    GPIO_TypeDef *scl_p = dev->cfg.scl_port, *sda_p = dev->cfg.sda_port;
    uint16_t scl = dev->cfg.scl_pin, sda = dev->cfg.sda_pin;

    /* 1. Release the pins from the I2C peripheral (MspDeInit -> GPIO analog). */
    (void)HAL_I2C_DeInit(h);
    i2c_force_reset(h->Instance);

    /* 2. Take both lines as open-drain GPIO, released (high via pull-ups). */
    gpio_od_output_high(scl_p, scl);
    gpio_od_output_high(sda_p, sda);
    delay_us(10);

    /* 3. If a slave is holding SDA low (stopped mid-byte), clock SCL until it
     *    lets go: at most 9 clocks finish any byte + ACK slot. */
    dev->last_recovery_sda_stuck = !pin_high(sda_p, sda);
    for (int i = 0; i < 9 && !pin_high(sda_p, sda); i++) {
        HAL_GPIO_WritePin(scl_p, scl, GPIO_PIN_RESET);
        delay_us(5);
        HAL_GPIO_WritePin(scl_p, scl, GPIO_PIN_SET);
        /* Respect clock stretching, but bounded (~1 ms). */
        for (int w = 0; w < 200 && !pin_high(scl_p, scl); w++) delay_us(5);
        delay_us(5);
    }

    /* 4. Generate a STOP: SDA low while SCL high, then SDA high. */
    HAL_GPIO_WritePin(scl_p, scl, GPIO_PIN_RESET);
    delay_us(5);
    HAL_GPIO_WritePin(sda_p, sda, GPIO_PIN_RESET);
    delay_us(5);
    HAL_GPIO_WritePin(scl_p, scl, GPIO_PIN_SET);
    delay_us(5);
    HAL_GPIO_WritePin(sda_p, sda, GPIO_PIN_SET);
    delay_us(10);

    dev->last_recovery_bus_free = pin_high(scl_p, scl) && pin_high(sda_p, sda);

    /* 5. Hand the pins back: HAL_I2C_Init -> HAL_I2C_MspInit re-applies the
     *    alternate function, kernel clock and peripheral clock. */
    HAL_GPIO_DeInit(scl_p, scl);
    HAL_GPIO_DeInit(sda_p, sda);
    h->State = HAL_I2C_STATE_RESET;            /* force MspInit to run */
    HAL_StatusTypeDef hs = HAL_I2C_Init(h);
    dev->ptr_at_raw = false;

    if (hs != HAL_OK) return AS5600_ERR_BUS;
    return dev->last_recovery_bus_free ? AS5600_OK : AS5600_ERR_BUS;
}

void as5600_set_offline(as5600_t *dev)
{
    if (!dev->online) return;
    dev->online           = false;
    dev->ptr_at_raw       = false;
    dev->offline_since_ms = HAL_GetTick();
    dev->retry_attempts   = 0;
    dev->last_retry_ms    = dev->offline_since_ms - dev->cfg.retry_period_ms;
}

as5600_svc_t as5600_service(as5600_t *dev)
{
    if (dev->online) return AS5600_SVC_IDLE;

    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - dev->last_retry_ms) < dev->cfg.retry_period_ms) {
        return AS5600_SVC_IDLE;
    }
    dev->last_retry_ms = now;
    dev->retry_attempts++;

    (void)as5600_bus_recover(dev);
    if (as5600_probe(dev) != AS5600_OK) return AS5600_SVC_RETRY_FAILED;

    uint16_t raw;
    if (rd12(dev, AS5600_REG_RAW_ANGLE_H, &raw) != AS5600_OK) {
        return AS5600_SVC_RETRY_FAILED;
    }

    /* Back online. Continue the unwrap by the shortest path from the last
     * known angle; flag that turns made while unplugged are unknowable. */
    if (dev->have_last) {
        dev->accum += as5600_fold((int32_t)raw - (int32_t)dev->last_raw);
        dev->discontinuity = true;
    } else {
        dev->accum     = (int32_t)(((uint32_t)raw - dev->zero_raw) & AS5600_COUNTS_MASK);
        dev->have_last = true;
    }
    dev->last_raw      = raw;
    dev->online        = true;
    dev->consec_errors = 0;
    dev->recoveries++;
    return AS5600_SVC_RECOVERED;
}

/* ------------------------------------------------------------------------- */
/* Register reads                                                            */
/* ------------------------------------------------------------------------- */

as5600_status_t as5600_read_raw_angle(as5600_t *dev, uint16_t *raw)
{
    return rd12(dev, AS5600_REG_RAW_ANGLE_H, raw);
}

as5600_status_t as5600_read_raw_angle_fast(as5600_t *dev, uint16_t *raw)
{
    if (!dev->ptr_at_raw) {
        return rd12(dev, AS5600_REG_RAW_ANGLE_H, raw);   /* also parks the pointer */
    }
    uint8_t b[2];
    HAL_StatusTypeDef hs = HAL_I2C_Master_Receive(dev->cfg.hi2c, AS5600_I2C_ADDR_HAL,
                                                  b, 2, dev->cfg.timeout_ms);
    as5600_status_t st = map_hal(hs, dev->cfg.hi2c);
    if (st != AS5600_OK) { note_error(dev, st); return st; }
    if (b[0] & 0xF0u)    { note_error(dev, AS5600_ERR_DATA); return AS5600_ERR_DATA; }
    note_ok(dev);
    *raw = (uint16_t)(((uint16_t)b[0] << 8) | b[1]);
    return AS5600_OK;
}

as5600_status_t as5600_read_angle(as5600_t *dev, uint16_t *angle)
{
    return rd12(dev, AS5600_REG_ANGLE_H, angle);
}

as5600_status_t as5600_read_status(as5600_t *dev, uint8_t *status)
{
    return rd(dev, AS5600_REG_STATUS, status, 1);
}

as5600_status_t as5600_read_agc(as5600_t *dev, uint8_t *agc)
{
    return rd(dev, AS5600_REG_AGC, agc, 1);
}

as5600_status_t as5600_read_magnitude(as5600_t *dev, uint16_t *magnitude)
{
    return rd12(dev, AS5600_REG_MAGNITUDE_H, magnitude);
}

as5600_status_t as5600_read_conf(as5600_t *dev, uint16_t *conf_word)
{
    uint8_t b[2];
    as5600_status_t st = rd(dev, AS5600_REG_CONF_H, b, 2);
    if (st != AS5600_OK) return st;
    *conf_word = (uint16_t)((((uint16_t)b[0] & 0x3Fu) << 8) | b[1]);   /* 14-bit field */
    return AS5600_OK;
}

as5600_status_t as5600_read_diag(as5600_t *dev, as5600_diag_t *d)
{
    /* Each multi-byte register is read in its own transaction: because of the
     * AS5600's special pointer handling for RAW ANGLE / ANGLE / MAGNITUDE, one
     * long burst across 0x0B..0x0F would NOT return STATUS, RAW ANGLE, ANGLE. */
    as5600_status_t st;
    memset(d, 0, sizeof(*d));
    if ((st = rd(dev, AS5600_REG_ZMCO, &d->zmco, 1)) != AS5600_OK)          return st;
    d->zmco &= 0x03u;
    if ((st = rd12(dev, AS5600_REG_ZPOS_H, &d->zpos)) != AS5600_OK)         return st;
    if ((st = rd12(dev, AS5600_REG_MPOS_H, &d->mpos)) != AS5600_OK)         return st;
    if ((st = rd12(dev, AS5600_REG_MANG_H, &d->mang)) != AS5600_OK)         return st;
    if ((st = as5600_read_conf(dev, &d->conf_word)) != AS5600_OK)           return st;
    as5600_decode_conf(d->conf_word, &d->conf);
    if ((st = as5600_read_status(dev, &d->status)) != AS5600_OK)            return st;
    if ((st = as5600_read_raw_angle(dev, &d->raw_angle)) != AS5600_OK)      return st;
    if ((st = as5600_read_angle(dev, &d->angle)) != AS5600_OK)              return st;
    if ((st = as5600_read_agc(dev, &d->agc)) != AS5600_OK)                  return st;
    if ((st = as5600_read_magnitude(dev, &d->magnitude)) != AS5600_OK)      return st;
    return AS5600_OK;
}

/* ------------------------------------------------------------------------- */
/* Position tracking                                                         */
/* ------------------------------------------------------------------------- */

as5600_status_t as5600_update(as5600_t *dev, uint32_t t_us, as5600_sample_t *out)
{
    if (!dev->online) return AS5600_ERR_OFFLINE;

    uint16_t raw;
    as5600_status_t st = dev->cfg.use_fast_read ? as5600_read_raw_angle_fast(dev, &raw)
                                                : as5600_read_raw_angle(dev, &raw);
    if (st != AS5600_OK) return st;

    if (!dev->have_last) {
        /* First sample: start inside [0, 1 rev) relative to the zero. */
        dev->accum     = (int32_t)(((uint32_t)raw - dev->zero_raw) & AS5600_COUNTS_MASK);
        dev->have_last = true;
    } else {
        /* Shortest-path unwrap handles 4095 -> 0 (+1) and 0 -> 4095 (-1). */
        dev->accum += as5600_fold((int32_t)raw - (int32_t)dev->last_raw);
    }
    dev->last_raw = raw;

    dev->last.t_us      = t_us;
    dev->last.raw       = raw;
    dev->last.zeroed    = (uint16_t)(((uint32_t)raw - dev->zero_raw) & AS5600_COUNTS_MASK);
    dev->last.unwrapped = dev->accum;
    if (out) *out = dev->last;
    return AS5600_OK;
}

as5600_status_t as5600_zero_here(as5600_t *dev, uint16_t *zero_raw_out)
{
    uint16_t raw;
    as5600_status_t st = as5600_read_raw_angle(dev, &raw);
    if (st != AS5600_OK) return st;
    dev->zero_raw  = raw;
    dev->accum     = 0;
    dev->last_raw  = raw;
    dev->have_last = true;
    dev->last.raw       = raw;
    dev->last.zeroed    = 0;
    dev->last.unwrapped = 0;
    if (zero_raw_out) *zero_raw_out = raw;
    return AS5600_OK;
}

/* ------------------------------------------------------------------------- */
/* Pure helpers                                                              */
/* ------------------------------------------------------------------------- */

as5600_magnet_t as5600_magnet_from_status(uint8_t status)
{
    if (!(status & AS5600_STATUS_MD)) return AS5600_MAGNET_NOT_DETECTED;
    if (status & AS5600_STATUS_MH)    return AS5600_MAGNET_TOO_STRONG;
    if (status & AS5600_STATUS_ML)    return AS5600_MAGNET_TOO_WEAK;
    return AS5600_MAGNET_OK;
}

const char *as5600_magnet_short(as5600_magnet_t m)
{
    switch (m) {
    case AS5600_MAGNET_NOT_DETECTED: return "NO_MAGNET";
    case AS5600_MAGNET_TOO_WEAK:     return "WEAK";
    case AS5600_MAGNET_TOO_STRONG:   return "STRONG";
    case AS5600_MAGNET_OK:           return "OK";
    default:                         return "?";
    }
}

const char *as5600_magnet_advice(as5600_magnet_t m)
{
    switch (m) {
    case AS5600_MAGNET_NOT_DETECTED:
        return "Magnet NOT DETECTED - check that a diametric magnet is centred over the chip";
    case AS5600_MAGNET_TOO_WEAK:
        return "Magnet TOO WEAK - move the magnet CLOSER to the chip";
    case AS5600_MAGNET_TOO_STRONG:
        return "Magnet TOO STRONG - move the magnet FARTHER from the chip";
    case AS5600_MAGNET_OK:
        return "Magnet OK - field strength is within the AGC range";
    default:
        return "unknown";
    }
}

const char *as5600_status_str(as5600_status_t s)
{
    switch (s) {
    case AS5600_OK:          return "OK";
    case AS5600_ERR_NACK:    return "NACK (no response at 0x36 - unplugged or wiring?)";
    case AS5600_ERR_TIMEOUT: return "TIMEOUT";
    case AS5600_ERR_BUS:     return "BUS ERROR / BUS BUSY (line stuck or floating?)";
    case AS5600_ERR_DATA:    return "BAD DATA (impossible register value)";
    case AS5600_ERR_OFFLINE: return "OFFLINE";
    case AS5600_ERR_PARAM:   return "BAD PARAMETER";
    default:                 return "?";
    }
}

void as5600_decode_conf(uint16_t w, as5600_conf_t *c)
{
    c->pm   = (uint8_t)((w >> AS5600_CONF_PM_POS)   & 0x3u);
    c->hyst = (uint8_t)((w >> AS5600_CONF_HYST_POS) & 0x3u);
    c->outs = (uint8_t)((w >> AS5600_CONF_OUTS_POS) & 0x3u);
    c->pwmf = (uint8_t)((w >> AS5600_CONF_PWMF_POS) & 0x3u);
    c->sf   = (uint8_t)((w >> AS5600_CONF_SF_POS)   & 0x3u);
    c->fth  = (uint8_t)((w >> AS5600_CONF_FTH_POS)  & 0x7u);
    c->wd   = (uint8_t)((w >> AS5600_CONF_WD_POS)   & 0x1u);
}

int64_t as5600_counts_to_cdeg(int32_t counts)
{
    int64_t num = (int64_t)counts * 36000;
    return (num >= 0) ? (num + AS5600_COUNTS_PER_REV / 2) / AS5600_COUNTS_PER_REV
                      : (num - AS5600_COUNTS_PER_REV / 2) / AS5600_COUNTS_PER_REV;
}

int32_t as5600_velocity_ddps(const as5600_sample_t *a, const as5600_sample_t *b)
{
    uint32_t dt = b->t_us - a->t_us;          /* wrap-safe */
    if (dt == 0u) return 0;
    /* tenths of a degree per second = dcounts * 3600 * 1e6 / (4096 * dt_us) */
    int64_t num = (int64_t)(b->unwrapped - a->unwrapped) * 3600LL * 1000000LL;
    int64_t den = (int64_t)AS5600_COUNTS_PER_REV * (int64_t)dt;
    int64_t q = (num >= 0) ? (num + den / 2) / den : (num - den / 2) / den;
    return (int32_t)q;
}
