/**
 * @file    rs485.h
 * @brief   RS485 communication header (USART1 + MAX485 DE/RE control)
 * @note    STM32G431RB Nucleo board, PA9(TX)/PA10(RX)/PA8(DE/RE)
 */

#ifndef __RS485_H
#define __RS485_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* === RS485 pin definitions === */
/** USART1 TX - MAX485 DI (PA9) */
#define RS485_TX_PIN          GPIO_PIN_9
#define RS485_TX_PORT         GPIOA
#define RS485_TX_AF           GPIO_AF7_USART1

/** USART1 RX - MAX485 RO (PA10) */
#define RS485_RX_PIN          GPIO_PIN_10
#define RS485_RX_PORT         GPIOA
#define RS485_RX_AF           GPIO_AF7_USART1

/** MAX485 DE/RE direction control (PA8) -- HIGH=transmit, LOW=receive */
#define RS485_DE_PIN          GPIO_PIN_8
#define RS485_DE_PORT         GPIOA
#define RS485_DE_HIGH()       HAL_GPIO_WritePin(RS485_DE_PORT, RS485_DE_PIN, GPIO_PIN_SET)
#define RS485_DE_LOW()        HAL_GPIO_WritePin(RS485_DE_PORT, RS485_DE_PIN, GPIO_PIN_RESET)

/* === RS485 parameters === */
#define RS485_BAUDRATE        115200U
/* CAN-FD 64-byte payload + header 3 bytes (ID_H, ID_L, DLC) = 67 bytes */
#define RS485_MAX_DATA_LEN    67U
#define RS485_RX_QUEUE_LEN    8U

/* === RS485 receive message structure === */
typedef struct {
    uint8_t  data[RS485_MAX_DATA_LEN];
    uint8_t  len;
} RS485_RxMessage_t;

/* === RS485 RX Queue (ISR -> Task) === */
extern QueueHandle_t xRS485RxQueue;

/* === Function declarations === */

/**
 * @brief  RS485 initialization (USART1 + GPIO DE/RE + interrupt receive)
 * @retval HAL status
 */
HAL_StatusTypeDef RS485_Init(void);

/**
 * @brief  RS485 data transmit
 * @param  data: transmit data buffer
 * @param  len:  data length
 * @retval HAL status
 * @note   DE/RE automatic control: HIGH(TX) -> transmit -> LOW(RX)
 */
HAL_StatusTypeDef RS485_SendData(const uint8_t *data, uint16_t len);

/**
 * @brief  Forward CAN message to RS485 (CAN->RS485 routing)
 * @param  can_id: CAN ID (11-bit)
 * @param  data:   CAN data
 * @param  dlc:    data length
 * @retval HAL status
 * @note   RS485 frame format: [ID_H][ID_L][DLC][DATA 0..N]
 */
HAL_StatusTypeDef RS485_ForwardCANMessage(uint32_t can_id, const uint8_t *data, uint8_t dlc);

/**
 * @brief  Restart receive after UART error
 * @note   For s_rx_byte static access from HAL_UART_ErrorCallback
 */
void RS485_RestartReceive(void);

#ifdef __cplusplus
}
#endif

#endif /* __RS485_H */
