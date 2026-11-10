/**
 * @file    enc_cli.c
 * @brief   AS5600 bench-test application and UART command interface.
 *
 * Timing model
 *  - A free-running 32-bit timer (TIM2 at 1 MHz) gives microsecond timestamps.
 *  - A background tracker reads RAW ANGLE every ENC_TRACK_PERIOD_US so the
 *    multi-turn unwrap stays valid regardless of the stream rate (the unwrap
 *    needs < 180 deg of travel between reads; at 200 Hz that is 36,000 deg/s).
 *  - Stream lines are emitted at the user rate (default 10 Hz). Velocity on a
 *    line is the average over the interval since the previous line.
 *
 * All numbers are printed with integer fixed-point formatting, so the default
 * newlib-nano printf (no float support) is sufficient. Never pass a float or
 * double to a printf-style function in this file.
 *
 * Output conventions: data lines start with a digit; every other line the
 * firmware emits on its own (events, headers) starts with '#', so a CSV log
 * can be filtered with  grep -v '^#'.
 */
#include "enc_cli.h"
#include "as5600.h"
#include "serial_io.h"
#include "main.h"

#include <ctype.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ------------------------------------------------------------------------- */
/* Configuration                                                             */
/* ------------------------------------------------------------------------- */
#define ENC_FW_VERSION          "1.0.0"
#define ENC_DEFAULT_RATE_HZ     10u
#define ENC_MAX_RATE_HZ         100u     /* ~85 B/line * 100 Hz fits 115200 baud */
#define ENC_TRACK_PERIOD_US     5000u    /* background unwrap tracker: 200 Hz    */
#define ENC_I2C_TIMEOUT_MS      5u       /* per transaction                      */
#define ENC_OFFLINE_AFTER       3u       /* consecutive failures -> offline      */
#define ENC_RETRY_PERIOD_MS     500u     /* recovery attempt period              */
#define ENC_POWERUP_DELAY_MS    20u      /* AS5600 power-up time before first probe */
#define ENC_NOISE_DEFAULT_N     1000u
#define ENC_NOISE_MAX_N         100000u
#define ENC_SPEED_N             1000u
#define ENC_LINE_MAX            80u

/* AGC full scale. The AS5600 AGC range is 0..255 in 5 V mode and 0..128 in
 * 3.3 V mode (VDD5V tied to VDD3V3). Used only to print AGC as a percentage
 * and to give "near the edge" hints; the verdict itself comes from the
 * MD/ML/MH status bits, which the chip computes. */
#ifndef ENC_AGC_FULL_SCALE
#define ENC_AGC_FULL_SCALE      128u
#endif

/* Recovery pins. CubeMX generates these from the PB8/PB9 user labels. */
#ifndef AS5600_SCL_Pin
#define AS5600_SCL_Pin          GPIO_PIN_8
#define AS5600_SCL_GPIO_Port    GPIOB
#endif
#ifndef AS5600_SDA_Pin
#define AS5600_SDA_Pin          GPIO_PIN_9
#define AS5600_SDA_GPIO_Port    GPIOB
#endif

/* ------------------------------------------------------------------------- */
/* Application state (the driver itself holds no globals; this is the app)   */
/* ------------------------------------------------------------------------- */
static as5600_t           s_enc;
static I2C_HandleTypeDef *s_hi2c;
static TIM_HandleTypeDef *s_htim;

static bool     s_stream_on;
static bool     s_csv;
static uint32_t s_rate_hz = ENC_DEFAULT_RATE_HZ;
static uint32_t s_next_stream_us;
static uint32_t s_last_track_us;

static as5600_sample_t s_prev_line;     /* velocity reference */
static bool            s_have_prev_line;

static bool     s_was_online;
static uint32_t s_lines_dropped;
static uint32_t s_lines_dropped_reported;
static uint32_t s_last_drop_report_ms;

static char     s_line[ENC_LINE_MAX + 1];
static uint32_t s_line_len;
static bool     s_last_was_cr;

/* ------------------------------------------------------------------------- */
/* Small utilities                                                           */
/* ------------------------------------------------------------------------- */
static inline uint32_t micros(void)
{
    return s_htim->Instance->CNT;   /* TIM2: 32-bit, 1 MHz, wraps every ~71.6 min */
}

/* Format a signed fixed-point value: v / 10^dec, e.g. (-12345, 2) -> "-123.45". */
static const char *fx(char *buf, size_t n, int64_t v, unsigned dec)
{
    static const uint32_t pow10[] = {1u, 10u, 100u, 1000u, 10000u};
    if (dec > 4u) dec = 4u;
    bool neg = v < 0;
    uint64_t a = neg ? (uint64_t)(-v) : (uint64_t)v;
    unsigned long ip = (unsigned long)(a / pow10[dec]);
    unsigned long fp = (unsigned long)(a % pow10[dec]);
    switch (dec) {
    case 0:  snprintf(buf, n, "%s%lu", neg ? "-" : "", ip); break;
    case 1:  snprintf(buf, n, "%s%lu.%01lu", neg ? "-" : "", ip, fp); break;
    case 2:  snprintf(buf, n, "%s%lu.%02lu", neg ? "-" : "", ip, fp); break;
    case 3:  snprintf(buf, n, "%s%lu.%03lu", neg ? "-" : "", ip, fp); break;
    default: snprintf(buf, n, "%s%lu.%04lu", neg ? "-" : "", ip, fp); break;
    }
    return buf;
}

static const char *cdeg_str(char *buf, size_t n, int32_t counts)
{
    return fx(buf, n, as5600_counts_to_cdeg(counts), 2);
}

static unsigned agc_pct(uint8_t agc)
{
    unsigned p = (unsigned)agc * 100u / ENC_AGC_FULL_SCALE;
    return p > 100u ? 100u : p;
}

static void event(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void event(const char *fmt, ...)
{
    char buf[200];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    serial_printf("# %s\r\n", buf);
}

/* Print a transition to offline (once) and the first error of a burst. */
static void report_error(const char *what, as5600_status_t st)
{
    if (s_enc.consec_errors == 1u || (!s_enc.online && s_was_online)) {
        event("I2C error on %s: %s (HAL ErrorCode 0x%02lx)", what, as5600_status_str(st),
              (unsigned long)s_enc.last_hal_error);
    }
    if (s_was_online && !s_enc.online) {
        event("SENSOR OFFLINE after %u consecutive errors. Recovering every %lu ms "
              "(bus recovery + re-probe). Replug to resume.",
              (unsigned)s_enc.consec_errors, (unsigned long)ENC_RETRY_PERIOD_MS);
    }
    s_was_online = s_enc.online;
}

static bool require_online(void)
{
    if (s_enc.online) return true;
    serial_puts("Sensor is offline - command needs the sensor. Waiting for automatic reconnect.\r\n");
    return false;
}

/* ------------------------------------------------------------------------- */
/* Magnet check (boot, diag, reconnect)                                      */
/* ------------------------------------------------------------------------- */
static bool magnet_check(bool verbose)
{
    uint8_t status = 0, agc = 0;
    uint16_t mag = 0;
    as5600_status_t st;

    if ((st = as5600_read_status(&s_enc, &status)) != AS5600_OK) { report_error("STATUS", st); return false; }
    if ((st = as5600_read_agc(&s_enc, &agc)) != AS5600_OK)       { report_error("AGC", st);    return false; }
    if ((st = as5600_read_magnitude(&s_enc, &mag)) != AS5600_OK) { report_error("MAGNITUDE", st); return false; }

    as5600_magnet_t m = as5600_magnet_from_status(status);
    if (verbose) {
        serial_printf("STATUS 0x%02X  MD=%u ML=%u MH=%u | AGC %u/%u (%u%%) | MAGNITUDE %u\r\n",
                      status,
                      (status & AS5600_STATUS_MD) ? 1u : 0u,
                      (status & AS5600_STATUS_ML) ? 1u : 0u,
                      (status & AS5600_STATUS_MH) ? 1u : 0u,
                      agc, (unsigned)ENC_AGC_FULL_SCALE, agc_pct(agc), mag);
    }
    serial_printf(">>> %s\r\n", as5600_magnet_advice(m));
    if (m == AS5600_MAGNET_OK) {
        unsigned p = agc_pct(agc);
        if (p < 15u) {
            serial_puts("    (AGC is low: field is on the STRONG side. A slightly larger gap gives more margin.)\r\n");
        } else if (p > 85u) {
            serial_puts("    (AGC is high: field is on the WEAK side. Move slightly closer for more margin.)\r\n");
        } else {
            serial_puts("    (AGC is mid-range: good margin both ways.)\r\n");
        }
    }
    return m == AS5600_MAGNET_OK;
}

/* ------------------------------------------------------------------------- */
/* Streaming                                                                 */
/* ------------------------------------------------------------------------- */
static void stream_header(void)
{
    if (s_csv) {
        serial_puts("t_ms,raw,deg,zeroed_deg,unwrapped_deg,vel_dps,md,ml,mh,magnet,agc\r\n");
    } else {
        serial_puts("#     t_ms   raw     deg   zeroed     unwrapped    vel_dps  magnet     agc\r\n");
    }
}

static void stream_restart(void)
{
    s_have_prev_line = false;
    s_next_stream_us = micros();
}

static void stream_emit(void)
{
    as5600_sample_t s;
    uint32_t now = micros();
    s_last_track_us = now;                 /* this read also counts as a tracker read */
    as5600_status_t st = as5600_update(&s_enc, now, &s);
    if (st != AS5600_OK) {
        report_error("RAW_ANGLE", st);
        return;
    }

    uint8_t status = 0, agc = 0;
    bool aux_ok = (as5600_read_status(&s_enc, &status) == AS5600_OK) &&
                  (as5600_read_agc(&s_enc, &agc) == AS5600_OK);
    if (!aux_ok) report_error("STATUS/AGC", s_enc.last_error);

    int32_t vel = s_have_prev_line ? as5600_velocity_ddps(&s_prev_line, &s) : 0;
    s_prev_line = s;
    s_have_prev_line = true;

    char d1[16], d2[16], d3[24], d4[16];
    cdeg_str(d1, sizeof d1, s.raw);
    cdeg_str(d2, sizeof d2, s.zeroed);
    cdeg_str(d3, sizeof d3, s.unwrapped);
    fx(d4, sizeof d4, vel, 1);

    const char *mag = aux_ok ? as5600_magnet_short(as5600_magnet_from_status(status)) : "ERR";
    char line[160];
    int n;
    unsigned long t_ms = (unsigned long)HAL_GetTick();
    if (s_csv) {
        n = snprintf(line, sizeof line, "%lu,%u,%s,%s,%s,%s,%u,%u,%u,%s,%u\r\n",
                     t_ms, s.raw, d1, d2, d3, d4,
                     (status & AS5600_STATUS_MD) ? 1u : 0u,
                     (status & AS5600_STATUS_ML) ? 1u : 0u,
                     (status & AS5600_STATUS_MH) ? 1u : 0u,
                     mag, agc);
    } else {
        n = snprintf(line, sizeof line, "%10lu  %4u  %6s  %7s  %12s  %9s  %-9s  %3u\r\n",
                     t_ms, s.raw, d1, d2, d3, d4, mag, agc);
    }
    if (n <= 0) return;
    if (!serial_try_write(line, (size_t)n)) s_lines_dropped++;
}

static void track_once(void)
{
    as5600_status_t st = as5600_update(&s_enc, micros(), NULL);
    if (st != AS5600_OK) report_error("RAW_ANGLE (tracker)", st);
}

/* ------------------------------------------------------------------------- */
/* Commands                                                                  */
/* ------------------------------------------------------------------------- */
static void cmd_help(void)
{
    serial_puts(
        "Commands (case-insensitive):\r\n"
        "  help              this list\r\n"
        "  stream on|off     start/stop the sample stream (no arg: show state)\r\n"
        "  rate <hz>         stream rate, 1..100 Hz (no arg: show rate)\r\n"
        "  csv on|off        CSV output for logging (comment lines start with '#')\r\n"
        "  zero              make the current angle the software zero (RAM only)\r\n"
        "  diag              decoded register dump + magnet verdict + link stats\r\n"
        "  noise [n]         n back-to-back reads while STATIONARY (default 1000,\r\n"
        "                    max 100000): min/max/p-p/std dev. Any key aborts.\r\n"
        "  speed             time per read and max read rate\r\n"
        "  scan              re-probe the I2C bus (all addresses, then 0x36)\r\n"
        "Stream columns: t_ms raw(0..4095) deg zeroed_deg unwrapped_deg vel_dps magnet agc\r\n"
        "This firmware never writes to the AS5600 (no BURN, no ZPOS/MPOS/MANG/CONF).\r\n");
}

static void cmd_stream(const char *arg)
{
    if (arg == NULL) {
        serial_printf("stream is %s at %lu Hz (%s)\r\n", s_stream_on ? "ON" : "OFF",
                      (unsigned long)s_rate_hz, s_csv ? "CSV" : "table");
        return;
    }
    if (strcmp(arg, "on") == 0) {
        s_stream_on = true;
        stream_header();
        stream_restart();
    } else if (strcmp(arg, "off") == 0) {
        s_stream_on = false;
        serial_puts("stream OFF\r\n");
    } else {
        serial_puts("usage: stream on|off\r\n");
    }
}

static void cmd_rate(const char *arg)
{
    if (arg == NULL) {
        serial_printf("rate = %lu Hz\r\n", (unsigned long)s_rate_hz);
        return;
    }
    char *end;
    long hz = strtol(arg, &end, 10);
    if (*end != '\0' || hz < 1 || hz > (long)ENC_MAX_RATE_HZ) {
        serial_printf("usage: rate <1..%u>\r\n", (unsigned)ENC_MAX_RATE_HZ);
        return;
    }
    s_rate_hz = (uint32_t)hz;
    serial_printf("rate = %lu Hz\r\n", (unsigned long)s_rate_hz);
    stream_restart();
}

static void cmd_csv(const char *arg)
{
    if (arg == NULL) {
        serial_printf("csv is %s\r\n", s_csv ? "ON" : "OFF");
        return;
    }
    if (strcmp(arg, "on") == 0)       s_csv = true;
    else if (strcmp(arg, "off") == 0) s_csv = false;
    else { serial_puts("usage: csv on|off\r\n"); return; }
    serial_printf("csv %s\r\n", s_csv ? "ON" : "OFF");
    if (s_stream_on) stream_header();
}

static void cmd_zero(void)
{
    if (!require_online()) return;
    uint16_t z;
    as5600_status_t st = as5600_zero_here(&s_enc, &z);
    if (st != AS5600_OK) { report_error("RAW_ANGLE (zero)", st); return; }
    char d[16];
    serial_printf("Software zero set at raw %u (%s deg). Unwrapped position reset to 0. "
                  "RAM only - nothing written to the sensor.\r\n", z, cdeg_str(d, sizeof d, z));
    s_have_prev_line = false;     /* velocity reference jumped */
}

static const char *pm_str(uint8_t v)   { static const char *t[] = {"NOM", "LPM1", "LPM2", "LPM3"}; return t[v & 3u]; }
static const char *hyst_str(uint8_t v) { static const char *t[] = {"OFF", "1 LSB", "2 LSB", "3 LSB"}; return t[v & 3u]; }
static const char *outs_str(uint8_t v) { static const char *t[] = {"analog 0-100%", "analog 10-90%", "digital PWM", "(reserved)"}; return t[v & 3u]; }
static const char *pwmf_str(uint8_t v) { static const char *t[] = {"115 Hz", "230 Hz", "460 Hz", "920 Hz"}; return t[v & 3u]; }
static const char *sf_str(uint8_t v)   { static const char *t[] = {"16x", "8x", "4x", "2x"}; return t[v & 3u]; }
static const char *fth_str(uint8_t v)
{
    static const char *t[] = {"slow filter only", "6 LSB", "7 LSB", "9 LSB",
                              "18 LSB", "21 LSB", "24 LSB", "10 LSB"};
    return t[v & 7u];
}

static void cmd_diag(void)
{
    as5600_diag_t d;
    char a[16], b[16];
    serial_puts("--- AS5600 register dump (read-only) ---\r\n");
    as5600_status_t st = as5600_read_diag(&s_enc, &d);
    if (st != AS5600_OK) {
        report_error("diag read", st);
        serial_printf("diag aborted: %s\r\n", as5600_status_str(st));
    } else {
        serial_printf("ZMCO      0x00     %u  (burn count; 0 = ZPOS/MPOS never burned)\r\n", d.zmco);
        serial_printf("ZPOS      0x01-02  %4u\r\n", d.zpos);
        serial_printf("MPOS      0x03-04  %4u\r\n", d.mpos);
        serial_printf("MANG      0x05-06  %4u  %s\r\n", d.mang,
                      (d.zpos == 0 && d.mpos == 0 && d.mang == 0) ? "(all 0 = factory default, ANGLE = full 360 deg)" : "");
        serial_printf("CONF      0x07-08  0x%04X\r\n", d.conf_word);
        serial_printf("          PM=%s HYST=%s OUTS=%s PWMF=%s\r\n",
                      pm_str(d.conf.pm), hyst_str(d.conf.hyst), outs_str(d.conf.outs), pwmf_str(d.conf.pwmf));
        serial_printf("          SF=%s FTH=%s WD=%s\r\n",
                      sf_str(d.conf.sf), fth_str(d.conf.fth), d.conf.wd ? "ON" : "OFF");
        serial_printf("STATUS    0x0B     0x%02X  MD=%u ML=%u MH=%u\r\n", d.status,
                      (d.status & AS5600_STATUS_MD) ? 1u : 0u,
                      (d.status & AS5600_STATUS_ML) ? 1u : 0u,
                      (d.status & AS5600_STATUS_MH) ? 1u : 0u);
        serial_printf("RAW ANGLE 0x0C-0D  %4u  (%s deg)\r\n", d.raw_angle, cdeg_str(a, sizeof a, d.raw_angle));
        serial_printf("ANGLE     0x0E-0F  %4u  (%s deg)\r\n", d.angle, cdeg_str(b, sizeof b, d.angle));
        serial_printf("AGC       0x1A     %4u  (%u%% of %u)\r\n", d.agc, agc_pct(d.agc), (unsigned)ENC_AGC_FULL_SCALE);
        serial_printf("MAGNITUDE 0x1B-1C  %4u\r\n", d.magnitude);
        serial_printf(">>> %s\r\n", as5600_magnet_advice(as5600_magnet_from_status(d.status)));
    }
    char z[16], u[24];
    serial_printf("Link: %s | errors %lu | recoveries %lu | last error %s | I2C TIMINGR 0x%08lX\r\n",
                  s_enc.online ? "ONLINE" : "OFFLINE", (unsigned long)s_enc.total_errors,
                  (unsigned long)s_enc.recoveries, as5600_status_str(s_enc.last_error),
                  (unsigned long)s_hi2c->Init.Timing);
    serial_printf("Software: zero at raw %u (%s deg) | unwrapped %s deg | stream lines dropped %lu | UART rx errors %lu\r\n",
                  s_enc.zero_raw, cdeg_str(z, sizeof z, s_enc.zero_raw),
                  cdeg_str(u, sizeof u, s_enc.accum),
                  (unsigned long)s_lines_dropped, (unsigned long)serial_rx_overruns());
    s_was_online = s_enc.online;
}

/* Any key except CR/LF aborts (the LF of a CRLF terminal that ended the
 * command line itself is still in the buffer when the test starts). */
static bool abort_requested(void)
{
    int c;
    while ((c = serial_getc()) >= 0) {
        if (c != '\r' && c != '\n') return true;
    }
    return false;
}

static void cmd_noise(const char *arg)
{
    if (!require_online()) return;
    uint32_t n = ENC_NOISE_DEFAULT_N;
    if (arg != NULL) {
        char *end;
        long v = strtol(arg, &end, 10);
        if (*end != '\0' || v < 2 || v > (long)ENC_NOISE_MAX_N) {
            serial_printf("usage: noise <2..%lu>\r\n", (unsigned long)ENC_NOISE_MAX_N);
            return;
        }
        n = (uint32_t)v;
    }
    serial_printf("Noise test: %lu back-to-back reads. Keep the shaft STATIONARY. Any key aborts.\r\n",
                  (unsigned long)n);
    serial_flush(200);

    /* Deviations are measured relative to the first sample and folded to the
     * shortest path, so a magnet sitting right on the 4095/0 boundary does not
     * produce a fake 4095-count spread. Welford's algorithm for mean/variance. */
    as5600_sample_t s;
    uint16_t ref = 0;
    int32_t  dmin = 0, dmax = 0;
    double   mean = 0.0, m2 = 0.0;
    uint32_t got = 0, errors = 0;
    uint32_t t0 = micros();

    while (got < n) {
        if ((got & 0x3FFu) == 0u && abort_requested()) {
            serial_puts("aborted.\r\n");
            break;
        }
        as5600_status_t st = as5600_update(&s_enc, micros(), &s);
        if (st != AS5600_OK) {
            errors++;
            report_error("RAW_ANGLE (noise)", st);
            if (!s_enc.online) break;
            continue;
        }
        if (got == 0u) ref = s.raw;
        int32_t d = as5600_fold((int32_t)s.raw - (int32_t)ref);
        if (got == 0u || d < dmin) dmin = d;
        if (got == 0u || d > dmax) dmax = d;
        got++;
        double delta = (double)d - mean;
        mean += delta / (double)got;
        m2   += delta * ((double)d - mean);
    }
    uint32_t dt_us = micros() - t0;
    s_last_track_us = micros();
    s_have_prev_line = false;

    if (got < 2u) {
        serial_printf("Not enough samples (%lu good, %lu errors).\r\n", (unsigned long)got, (unsigned long)errors);
        return;
    }
    double sd = sqrt(m2 / (double)(got - 1u));
    int32_t pp = dmax - dmin;
    /* Fixed-point conversions for printing (no float printf). */
    int64_t sd_milli_counts = (int64_t)(sd * 1000.0 + 0.5);
    int64_t sd_deg_e4       = (int64_t)(sd * 360.0 / 4096.0 * 10000.0 + 0.5);   /* 1e-4 deg */
    int32_t mean_raw_x100   = (int32_t)((double)ref * 100.0 + mean * 100.0 + 0.5);
    char a[20], b[20], c[20], e[20], f[20], g[20];

    serial_printf("Samples %lu (errors %lu) in %lu ms -> %lu reads/s\r\n",
                  (unsigned long)got, (unsigned long)errors, (unsigned long)(dt_us / 1000u),
                  (unsigned long)(dt_us ? (uint64_t)got * 1000000u / dt_us : 0u));
    serial_printf("Mean raw  %s counts\r\n", fx(a, sizeof a, mean_raw_x100, 2));
    serial_printf("Min/Max   %ld / %ld counts relative to first sample (raw %u)\r\n",
                  (long)dmin, (long)dmax, ref);
    serial_printf("Peak-peak %ld counts = %s deg\r\n", (long)pp, cdeg_str(b, sizeof b, pp));
    serial_printf("Std dev   %s counts = %s deg\r\n",
                  fx(c, sizeof c, sd_milli_counts, 3), fx(e, sizeof e, sd_deg_e4, 4));
    serial_printf("(1 count = 360/4096 = %s deg; resolution limit is +/-0.5 count = %s deg)\r\n",
                  fx(f, sizeof f, 879, 4), fx(g, sizeof g, 440, 4));
    if (pp > 20) {
        serial_puts("WARNING: spread > 20 counts - was the shaft moving, or is the magnet marginal? Check 'diag'.\r\n");
    }
}

typedef as5600_status_t (*read_fn_t)(as5600_t *, uint16_t *);

static void time_reads(const char *label, read_fn_t fn, uint16_t *last_val)
{
    uint32_t tmin = UINT32_MAX, tmax = 0, errors = 0, ok = 0;
    uint64_t total = 0;
    for (uint32_t i = 0; i < ENC_SPEED_N; i++) {
        uint16_t v;
        uint32_t t0 = micros();
        as5600_status_t st = fn(&s_enc, &v);
        uint32_t dt = micros() - t0;
        if (st != AS5600_OK) { errors++; continue; }
        ok++;
        *last_val = v;
        total += dt;
        if (dt < tmin) tmin = dt;
        if (dt > tmax) tmax = dt;
    }
    if (ok == 0u) {
        serial_printf("  %-34s all %lu reads failed\r\n", label, (unsigned long)ENC_SPEED_N);
        return;
    }
    uint32_t avg_x10 = (uint32_t)((total * 10u + ok / 2u) / ok);
    char a[16];
    serial_printf("  %-34s avg %s us  min %lu  max %lu  -> max %lu reads/s  (errors %lu)\r\n",
                  label, fx(a, sizeof a, avg_x10, 1), (unsigned long)tmin, (unsigned long)tmax,
                  (unsigned long)(avg_x10 ? 10000000u / avg_x10 : 0u), (unsigned long)errors);
}

/* A "stream line" costs three transactions: RAW ANGLE + STATUS + AGC. */
static as5600_status_t read_line_cost(as5600_t *dev, uint16_t *out)
{
    uint8_t s, a;
    as5600_status_t st;
    if ((st = as5600_read_raw_angle(dev, out)) != AS5600_OK) return st;
    if ((st = as5600_read_status(dev, &s)) != AS5600_OK) return st;
    return as5600_read_agc(dev, &a);
}

static void cmd_speed(void)
{
    if (!require_online()) return;
    serial_printf("Timing %lu reads per method (I2C TIMINGR 0x%08lX, SYSCLK %lu MHz, TIM2 1 us resolution):\r\n",
                  (unsigned long)ENC_SPEED_N, (unsigned long)s_hi2c->Init.Timing,
                  (unsigned long)(SystemCoreClock / 1000000u));
    serial_flush(200);

    uint16_t v_full = 0, v_fast = 0, v_line = 0, v_check = 0;
    time_reads("RAW ANGLE, register read (Mem_Read)", as5600_read_raw_angle, &v_full);
    /* The register read above leaves the device pointer parked on RAW ANGLE,
     * so the fast path below really is a pure 2-byte receive. */
    time_reads("RAW ANGLE, fast read (no pointer)", as5600_read_raw_angle_fast, &v_fast);
    (void)as5600_read_raw_angle(&s_enc, &v_check);
    time_reads("stream line (RAW+STATUS+AGC)", read_line_cost, &v_line);

    int32_t diff = as5600_fold((int32_t)v_fast - (int32_t)v_check);
    serial_printf("Fast-read cross-check: last fast %u vs register read %u (diff %ld) -> %s\r\n",
                  v_fast, v_check, (long)diff, (diff >= -8 && diff <= 8) ? "consistent" : "MISMATCH");
    serial_puts("Notes: the AS5600 refreshes its output internally at a fixed rate, so reads\r\n"
                "faster than that return repeated values. This firmware uses the register\r\n"
                "read path (use_fast_read = false) by default.\r\n");
    s_last_track_us = micros();
    s_have_prev_line = false;
}

static void cmd_scan(void)
{
    serial_puts("Scanning I2C1 (7-bit 0x08..0x77)...\r\n");
    if (__HAL_I2C_GET_FLAG(s_hi2c, I2C_FLAG_BUSY) == SET) {
        serial_puts("  bus BUSY (a line is held low or floating) - running bus recovery first\r\n");
        as5600_status_t r = as5600_bus_recover(&s_enc);
        serial_printf("  recovery: SDA was %s, bus %s\r\n",
                      s_enc.last_recovery_sda_stuck ? "STUCK LOW" : "high",
                      (r == AS5600_OK) ? "free" : "STILL NOT FREE (check wiring / pull-ups / power)");
    }
    uint32_t found = 0;
    for (uint16_t addr = 0x08; addr <= 0x77; addr++) {
        HAL_StatusTypeDef hs = HAL_I2C_IsDeviceReady(s_hi2c, (uint16_t)(addr << 1), 1u, 2u);
        if (hs == HAL_OK) {
            serial_printf("  0x%02X ACK%s\r\n", addr, (addr == AS5600_I2C_ADDR_7BIT) ? "  <- AS5600" : "");
            found++;
        } else if (hs == HAL_BUSY) {
            serial_printf("  bus BUSY at 0x%02X - aborting scan\r\n", addr);
            break;
        }
    }
    serial_printf("%lu device(s) found.\r\n", (unsigned long)found);
    as5600_status_t p = as5600_probe(&s_enc);
    serial_printf("Probe 0x36: %s\r\n", (p == AS5600_OK) ? "ACK - AS5600 present" :
                  (p == AS5600_ERR_BUS) ? "BUS BUSY" : "NACK - not responding");
    if (p == AS5600_OK && !s_enc.online) {
        serial_puts("Sensor answers but is marked offline - the reconnect logic will bring it back now.\r\n");
    }
}

/* Lower-case, split into "cmd" and an optional single argument. */
static void dispatch(char *line)
{
    for (char *p = line; *p; p++) *p = (char)tolower((unsigned char)*p);
    char *cmd = strtok(line, " \t");
    if (cmd == NULL) return;
    char *arg = strtok(NULL, " \t");
    char *extra = strtok(NULL, " \t");
    if (extra != NULL) { serial_puts("too many arguments (try 'help')\r\n"); return; }

    if      (strcmp(cmd, "help") == 0 || strcmp(cmd, "?") == 0) cmd_help();
    else if (strcmp(cmd, "stream") == 0) cmd_stream(arg);
    else if (strcmp(cmd, "rate") == 0)   cmd_rate(arg);
    else if (strcmp(cmd, "csv") == 0)    cmd_csv(arg);
    else if (strcmp(cmd, "zero") == 0)   cmd_zero();
    else if (strcmp(cmd, "diag") == 0)   cmd_diag();
    else if (strcmp(cmd, "noise") == 0)  cmd_noise(arg);
    else if (strcmp(cmd, "speed") == 0)  cmd_speed();
    else if (strcmp(cmd, "scan") == 0)   cmd_scan();
    else serial_printf("unknown command '%s' (try 'help')\r\n", cmd);
}

/* Echoing line editor: printable chars, backspace/DEL, CR, LF or CRLF. */
static void cli_poll(void)
{
    int c;
    while ((c = serial_getc()) >= 0) {
        if (c == '\r' || c == '\n') {
            if (c == '\n' && s_last_was_cr) { s_last_was_cr = false; continue; }
            s_last_was_cr = (c == '\r');
            serial_puts("\r\n");
            s_line[s_line_len] = '\0';
            if (s_line_len > 0u) dispatch(s_line);
            s_line_len = 0;
            continue;
        }
        s_last_was_cr = false;
        if (c == 0x08 || c == 0x7F) {
            if (s_line_len > 0u) { s_line_len--; serial_puts("\b \b"); }
        } else if (c >= 0x20 && c < 0x7F) {
            if (s_line_len < ENC_LINE_MAX) {
                s_line[s_line_len++] = (char)c;
                char e = (char)c;
                serial_write(&e, 1);
            }
        }
    }
}

/* ------------------------------------------------------------------------- */
/* Public entry points                                                       */
/* ------------------------------------------------------------------------- */
void enc_app_init(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart, TIM_HandleTypeDef *htim_us)
{
    s_hi2c = hi2c;
    s_htim = htim_us;
    serial_init(huart);
    HAL_TIM_Base_Start(s_htim);

    as5600_config_t cfg = {
        .hi2c            = hi2c,
        .scl_port        = AS5600_SCL_GPIO_Port,
        .scl_pin         = AS5600_SCL_Pin,
        .sda_port        = AS5600_SDA_GPIO_Port,
        .sda_pin         = AS5600_SDA_Pin,
        .timeout_ms      = ENC_I2C_TIMEOUT_MS,
        .offline_after   = ENC_OFFLINE_AFTER,
        .retry_period_ms = ENC_RETRY_PERIOD_MS,
        .agc_full_scale  = ENC_AGC_FULL_SCALE,
        .use_fast_read   = false,
    };
    (void)as5600_init(&s_enc, &cfg);

    serial_puts("\r\n\r\n");
    serial_puts("==============================================================\r\n");
    serial_printf("AS5600 bench test v%s  (built " __DATE__ " " __TIME__ ")\r\n", ENC_FW_VERSION);
    serial_printf("NUCLEO-L476RG  SYSCLK %lu MHz  I2C1 PB8=SCL PB9=SDA  TIMINGR 0x%08lX\r\n",
                  (unsigned long)(SystemCoreClock / 1000000u), (unsigned long)hi2c->Init.Timing);
    serial_puts("Read-only driver: never writes BURN/ZPOS/MPOS/MANG/CONF.\r\n");
    serial_puts("==============================================================\r\n");

    HAL_Delay(ENC_POWERUP_DELAY_MS);
    as5600_status_t p = as5600_probe(&s_enc);
    serial_printf("Probe 0x36: %s\r\n", (p == AS5600_OK) ? "ACK" :
                  (p == AS5600_ERR_BUS) ? "BUS BUSY (SDA/SCL held low or floating?)" : "NACK");

    if (p == AS5600_OK) {
        (void)magnet_check(true);
        as5600_status_t st = as5600_update(&s_enc, micros(), NULL);   /* seed unwrap */
        if (st != AS5600_OK) report_error("RAW_ANGLE", st);
    } else {
        serial_puts("Sensor not found. Check wiring (README). Retrying automatically every 500 ms.\r\n");
        as5600_set_offline(&s_enc);
    }
    s_was_online = s_enc.online;

    serial_puts("Type 'help' for commands. Streaming at 10 Hz; 'stream off' to stop.\r\n");
    s_stream_on = true;
    stream_header();
    stream_restart();
    s_last_track_us = micros();
}

void enc_app_poll(void)
{
    cli_poll();

    uint32_t now = micros();
    if (s_enc.online) {
        uint32_t period = 1000000u / s_rate_hz;
        if (s_stream_on && (int32_t)(now - s_next_stream_us) >= 0) {
            s_next_stream_us += period;
            if ((int32_t)(now - s_next_stream_us) >= 0) {
                s_next_stream_us = now + period;     /* fell behind: resync, no burst */
            }
            stream_emit();
        } else if ((uint32_t)(now - s_last_track_us) >= ENC_TRACK_PERIOD_US) {
            s_last_track_us = now;
            track_once();
        }
    } else {
        as5600_svc_t r = as5600_service(&s_enc);
        if (r == AS5600_SVC_RECOVERED) {
            event("SENSOR BACK ONLINE after %lu ms (%lu recovery attempt%s).",
                  (unsigned long)(HAL_GetTick() - s_enc.offline_since_ms),
                  (unsigned long)s_enc.retry_attempts, s_enc.retry_attempts == 1u ? "" : "s");
            if (s_enc.discontinuity) {
                event("Multi-turn count continued by shortest path; any turns made while "
                      "unplugged are unknown. Re-'zero' if you need an absolute reference.");
                s_enc.discontinuity = false;
            }
            (void)magnet_check(false);
            s_was_online = s_enc.online;
            stream_restart();
            s_last_track_us = micros();
            if (s_stream_on) stream_header();
        } else if (r == AS5600_SVC_RETRY_FAILED) {
            uint32_t n = s_enc.retry_attempts;
            if (n == 1u || (n % 10u) == 0u) {
                event("still offline (attempt %lu): SDA %s, bus %s, probe got no ACK",
                      (unsigned long)n,
                      s_enc.last_recovery_sda_stuck ? "was stuck low" : "high",
                      s_enc.last_recovery_bus_free ? "free" : "NOT free");
            }
        }
    }

    /* Rate-limited report of dropped stream lines (UART could not keep up). */
    uint32_t ms = HAL_GetTick();
    if (s_lines_dropped != s_lines_dropped_reported && (ms - s_last_drop_report_ms) >= 2000u) {
        event("%lu stream line(s) dropped - UART is saturated, lower 'rate'",
              (unsigned long)(s_lines_dropped - s_lines_dropped_reported));
        s_lines_dropped_reported = s_lines_dropped;
        s_last_drop_report_ms = ms;
    }
}
