/**
 * @file    uart_debug.h
 * @brief   UART debug output header
 * @note    printf-style debug output via USART2 (ST-LINK VCP)
 */

#ifndef __UART_DEBUG_H
#define __UART_DEBUG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* === per-frame verbose debug (1=on, 0=off) ===
 * Each CAN/ISO-TP frame UART output is blocking (~4ms per line at 115200 baud), so
 * it is a response latency bottleneck. Set to 0 for production/latency measurement.
 * Init/error logs are separate.
 */
#ifndef DEBUG_VERBOSE
#define DEBUG_VERBOSE 1
#endif

/* === UART initialization function === */

/**
 * @brief  USART2 debug port initialization
 * @param  huart: UART handler pointer
 * @retval HAL status (HAL_OK = success)
 * @note   115200 baud, 8N1, TX-only (debug output only)
 */
HAL_StatusTypeDef UART_DebugInit(UART_HandleTypeDef *huart);

/**
 * @brief  Single character transmit (for printf retarget)
 * @param  ch: character to transmit
 * @retval transmitted character
 */
int __io_putchar(int ch);

/**
 * @brief  String debug output (USART2)
 * @param  fmt: printf format string
 * @retval None
 */
void Debug_Print(const char *fmt, ...);

/**
 * @brief  Log received CAN message
 * @param  id:   CAN ID
 * @param  data: data buffer
 * @param  len:  data length
 * @retval None
 */
void Debug_LogCAN_Rx(uint32_t id, const uint8_t *data, uint32_t len);

/**
 * @brief  Log transmitted CAN message
 * @param  id:   CAN ID
 * @param  data: data buffer
 * @param  len:  data length
 * @retval None
 */
void Debug_LogCAN_Tx(uint32_t id, const uint8_t *data, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* __UART_DEBUG_H */
