/**
 * @file    enc_cli.h
 * @brief   AS5600 bench-test application: boot checks, streaming and the
 *          line-based UART command interface.
 *
 * Integration (main.c USER CODE blocks):
 *   USER CODE BEGIN Includes : #include "enc_cli.h"
 *   USER CODE BEGIN 2        : enc_app_init(&hi2c1, &huart2, &htim2);
 *   USER CODE BEGIN 3        : enc_app_poll();
 */
#ifndef ENC_CLI_H
#define ENC_CLI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"

/** One-time start-up: console, microsecond timer, sensor probe and magnet
 *  check, then streaming at the default rate. Call after all MX_*_Init(). */
void enc_app_init(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart, TIM_HandleTypeDef *htim_us);

/** Call continuously from the main while(1) loop. Never blocks for longer than
 *  one bounded I2C transaction (except inside the noise/speed commands). */
void enc_app_poll(void);

#ifdef __cplusplus
}
#endif
#endif /* ENC_CLI_H */
