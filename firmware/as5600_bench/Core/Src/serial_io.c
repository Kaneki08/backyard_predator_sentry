/**
 * @file    serial_io.c
 * @brief   Interrupt-driven UART console. See serial_io.h.
 *
 * Concurrency model: the main loop is the only producer of TX data and the
 * only consumer of RX data. The UART ISR (via HAL callbacks) is the only TX
 * consumer and RX producer. Indices are free-running uint32_t; each side only
 * writes its own index. Starting a transfer (tx_kick) is done with interrupts
 * masked so the main loop and the TX-complete ISR cannot both start one.
 */
#include "serial_io.h"
#include <stdio.h>
#include <string.h>

static UART_HandleTypeDef *s_uart;

static uint8_t           s_tx_buf[SERIAL_TX_BUF_SIZE];
static volatile uint32_t s_tx_head;      /* written by main            */
static volatile uint32_t s_tx_tail;      /* written by ISR (and kick)  */
static volatile uint32_t s_tx_inflight;  /* bytes handed to HAL       */

static uint8_t           s_rx_buf[SERIAL_RX_BUF_SIZE];
static volatile uint32_t s_rx_head;      /* written by ISR             */
static volatile uint32_t s_rx_tail;      /* written by main            */
static volatile uint32_t s_rx_overruns;
static uint8_t           s_rx_byte;

/* Must be called with interrupts masked (or from the TX-complete ISR). */
static void tx_kick_locked(void)
{
    if (s_tx_inflight != 0u) return;
    uint32_t used = s_tx_head - s_tx_tail;
    if (used == 0u) return;

    uint32_t start = s_tx_tail & (SERIAL_TX_BUF_SIZE - 1u);
    uint32_t chunk = SERIAL_TX_BUF_SIZE - start;           /* contiguous run */
    if (chunk > used)   chunk = used;
    if (chunk > 0xFFFFu) chunk = 0xFFFFu;

    s_tx_inflight = chunk;
    if (HAL_UART_Transmit_IT(s_uart, &s_tx_buf[start], (uint16_t)chunk) != HAL_OK) {
        s_tx_inflight = 0u;                               /* retried on next kick */
    }
}

static void tx_kick(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    tx_kick_locked();
    __set_PRIMASK(primask);
}

static void rx_arm(void)
{
    (void)HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1u);
}

void serial_init(UART_HandleTypeDef *huart)
{
    s_uart = huart;
    s_tx_head = s_tx_tail = s_tx_inflight = 0u;
    s_rx_head = s_rx_tail = s_rx_overruns = 0u;
    rx_arm();
}

size_t serial_tx_free(void)
{
    return SERIAL_TX_BUF_SIZE - (s_tx_head - s_tx_tail);
}

static void tx_put(const char *s, size_t n)
{
    /* Caller guarantees n <= free space. */
    uint32_t head = s_tx_head;
    for (size_t i = 0; i < n; i++) {
        s_tx_buf[(head + i) & (SERIAL_TX_BUF_SIZE - 1u)] = (uint8_t)s[i];
    }
    __DMB();
    s_tx_head = head + (uint32_t)n;
}

void serial_write(const char *s, size_t n)
{
    while (n > 0u) {
        size_t space = serial_tx_free();
        if (space == 0u) {
            tx_kick();            /* make sure the drain is running, then wait */
            continue;
        }
        size_t k = (n < space) ? n : space;
        tx_put(s, k);
        s += k;
        n -= k;
        tx_kick();
    }
}

bool serial_try_write(const char *s, size_t n)
{
    if (n > serial_tx_free()) return false;
    tx_put(s, n);
    tx_kick();
    return true;
}

void serial_puts(const char *s)
{
    serial_write(s, strlen(s));
}

void serial_printf(const char *fmt, ...)
{
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    if (n < 0) return;
    if ((size_t)n >= sizeof buf) n = (int)sizeof buf - 1;   /* truncated */
    serial_write(buf, (size_t)n);
}

void serial_flush(uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    while ((s_tx_head != s_tx_tail) && (HAL_GetTick() - t0 < timeout_ms)) {
        tx_kick();
    }
}

int serial_getc(void)
{
    if (s_rx_head == s_rx_tail) return -1;
    uint8_t c = s_rx_buf[s_rx_tail & (SERIAL_RX_BUF_SIZE - 1u)];
    __DMB();
    s_rx_tail++;
    return c;
}

uint32_t serial_rx_overruns(void)
{
    return s_rx_overruns;
}

/* ------------------------------------------------------------------------- */
/* HAL callbacks (ISR context)                                               */
/* ------------------------------------------------------------------------- */

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != s_uart) return;
    s_tx_tail += s_tx_inflight;
    s_tx_inflight = 0u;
    tx_kick_locked();                 /* already in ISR: no other kicker can run */
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != s_uart) return;
    if ((s_rx_head - s_rx_tail) < SERIAL_RX_BUF_SIZE) {
        s_rx_buf[s_rx_head & (SERIAL_RX_BUF_SIZE - 1u)] = s_rx_byte;
        __DMB();
        s_rx_head++;
    } else {
        s_rx_overruns++;
    }
    rx_arm();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart != s_uart) return;
    /* Overrun / framing / noise: HAL has already cleared the flags and aborted
     * the blocking part of the transfer. Re-arm reception. If a transmit was
     * aborted, drop the in-flight chunk accounting so TX restarts cleanly. */
    s_rx_overruns++;
    if (huart->gState == HAL_UART_STATE_READY && s_tx_inflight != 0u) {
        s_tx_tail += s_tx_inflight;
        s_tx_inflight = 0u;
        tx_kick_locked();
    }
    rx_arm();
}
