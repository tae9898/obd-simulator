/**
 * @file    rs485.h
 * @brief   RS485 통신 헤더 (USART1 + MAX485 DE/RE 제어)
 * @note    STM32G431RB Nucleo 보드, PA9(TX)/PA10(RX)/PA8(DE/RE)
 */

#ifndef __RS485_H
#define __RS485_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* === RS485 핀 정의 === */
/** USART1 TX - MAX485 DI (PA9) */
#define RS485_TX_PIN          GPIO_PIN_9
#define RS485_TX_PORT         GPIOA
#define RS485_TX_AF           GPIO_AF7_USART1

/** USART1 RX - MAX485 RO (PA10) */
#define RS485_RX_PIN          GPIO_PIN_10
#define RS485_RX_PORT         GPIOA
#define RS485_RX_AF           GPIO_AF7_USART1

/** MAX485 DE/RE 방향 제어 (PA8) — HIGH=송신, LOW=수신 */
#define RS485_DE_PIN          GPIO_PIN_8
#define RS485_DE_PORT         GPIOA
#define RS485_DE_HIGH()       HAL_GPIO_WritePin(RS485_DE_PORT, RS485_DE_PIN, GPIO_PIN_SET)
#define RS485_DE_LOW()        HAL_GPIO_WritePin(RS485_DE_PORT, RS485_DE_PIN, GPIO_PIN_RESET)

/* === RS485 파라미터 === */
#define RS485_BAUDRATE        115200U
#define RS485_MAX_DATA_LEN    64U
#define RS485_RX_QUEUE_LEN    8U

/* === RS485 수신 메시지 구조체 === */
typedef struct {
    uint8_t  data[RS485_MAX_DATA_LEN];
    uint8_t  len;
} RS485_RxMessage_t;

/* === RS485 RX Queue (ISR → Task) === */
extern QueueHandle_t xRS485RxQueue;

/* === 함수 선언 === */

/**
 * @brief  RS485 초기화 (USART1 + GPIO DE/RE + 인터럽트 수신)
 * @retval HAL 상태
 */
HAL_StatusTypeDef RS485_Init(void);

/**
 * @brief  RS485 데이터 송신
 * @param  data: 송신할 데이터 버퍼
 * @param  len:  데이터 길이
 * @retval HAL 상태
 * @note   DE/RE 자동 제어: HIGH(TX) → 전송 → LOW(RX)
 */
HAL_StatusTypeDef RS485_SendData(const uint8_t *data, uint16_t len);

/**
 * @brief  CAN 메시지를 RS485로 포워딩 (CAN→RS485 라우팅)
 * @param  can_id: CAN ID (11-bit)
 * @param  data:   CAN 데이터
 * @param  dlc:    데이터 길이
 * @retval HAL 상태
 * @note   RS485 프레임 포맷: [ID_H][ID_L][DLC][DATA 0..N]
 */
HAL_StatusTypeDef RS485_ForwardCANMessage(uint32_t can_id, const uint8_t *data, uint8_t dlc);

/**
 * @brief  UART 에러 후 수신 재시작
 * @note   HAL_UART_ErrorCallback에서 s_rx_byte static 접근용
 */
void RS485_RestartReceive(void);

#ifdef __cplusplus
}
#endif

#endif /* __RS485_H */
