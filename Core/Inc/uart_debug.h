/**
 * @file    uart_debug.h
 * @brief   UART 디버그 출력 헤더
 * @note    USART2 (ST-LINK VCP)를 통한 printf 스타일 디버그 출력
 */

#ifndef __UART_DEBUG_H
#define __UART_DEBUG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* === UART 초기화 함수 === */

/**
 * @brief  USART2 디버그 포트 초기화
 * @param  huart: UART 핸들러 포인터
 * @retval HAL 상태 (HAL_OK = 성공)
 * @note   115200 baud, 8N1, TX-only (디버그 출력 전용)
 */
HAL_StatusTypeDef UART_DebugInit(UART_HandleTypeDef *huart);

/**
 * @brief  단일 문자 전송 (printf retarget용)
 * @param  ch: 전송할 문자
 * @retval 전송된 문자
 */
int __io_putchar(int ch);

/**
 * @brief  문자열 디버그 출력 (USART2)
 * @param  fmt: printf 형식 문자열
 * @retval None
 */
void Debug_Print(const char *fmt, ...);

/**
 * @brief  수신된 CAN 메시지 로그 출력
 * @param  id:   CAN ID
 * @param  data: 데이터 버퍼
 * @param  len:  데이터 길이
 * @retval None
 */
void Debug_LogCAN_Rx(uint32_t id, const uint8_t *data, uint32_t len);

/**
 * @brief  전송한 CAN 메시지 로그 출력
 * @param  id:   CAN ID
 * @param  data: 데이터 버퍼
 * @param  len:  데이터 길이
 * @retval None
 */
void Debug_LogCAN_Tx(uint32_t id, const uint8_t *data, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* __UART_DEBUG_H */
