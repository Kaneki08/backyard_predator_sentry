/**
 * @file    serial_io.h
 * @brief   Interrupt-driven UART console (ST-LINK VCP on USART2).
 *
 *  - TX: ring buffer drained by HAL_UART_Transmit_IT, so printing never stalls
 *    the sensor loop. serial_write() blocks only while the buffer is full;
 *    serial_try_write() is all-or-nothing and never blocks (used for stream
 *    lines, which are dropped and counted if the UART cannot keep up).
 *  - RX: 1-byte HAL_UART_Receive_IT, re-armed from the completion and error
 *    callbacks, feeding a small ring buffer.
 *
 * This module defines HAL_UART_TxCpltCallback, HAL_UART_RxCpltCallback and
 * HAL_UART_ErrorCallback. Do not define them again in main.c.
 */
#ifndef SERIAL_IO_H
#define SERIAL_IO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"
#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define SERIAL_TX_BUF_SIZE  4096u   /* power of two */
#define SERIAL_RX_BUF_SIZE  256u    /* power of two */

void   serial_init(UART_HandleTypeDef *huart);
void   serial_write(const char *s, size_t n);         /* blocks while full     */
bool   serial_try_write(const char *s, size_t n);     /* all-or-nothing        */
void   serial_puts(const char *s);                    /* blocking, no newline  */
void   serial_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
size_t serial_tx_free(void);
void   serial_flush(uint32_t timeout_ms);             /* wait until TX drained */
int    serial_getc(void);                             /* -1 if nothing waiting */
uint32_t serial_rx_overruns(void);

#ifdef __cplusplus
}
#endif
#endif /* SERIAL_IO_H */
