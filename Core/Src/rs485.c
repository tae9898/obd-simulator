/**
 * @file    rs485.c
 * @brief   RS485 communication implementation (USART1 + MAX485 DE/RE control)
 * @note    Half-duplex RS485: TX/RX direction switching via GPIO PA8
 *         Interrupt-based 1-byte receive -> frame assembly -> Queue delivery
 */

#include "rs485.h"
#include "uart_debug.h"
#include <string.h>

/* === USART1 handler (defined in main.c) === */
extern UART_HandleTypeDef huart1;

/* === RS485 RX Queue (created in main.c) === */
extern QueueHandle_t xRS485RxQueue;

/* === Interrupt receive buffer === */
static volatile uint8_t  s_rx_byte;
static volatile uint8_t  s_rx_buf[RS485_MAX_DATA_LEN];
static volatile uint8_t  s_rx_len = 0;

/**
 * @brief  RS485 initialization
 * @retval HAL_OK = success
 *
 * @note   Configuration:
 *         - USART1, PA9(TX) / PA10(RX)
 *         - 115200 baud, 8N1
 *         - PA8 GPIO output: MAX485 DE/RE (initial=LOW, receive mode)
 *         - USART1 interrupt priority 7 (lower than FDCAN=6)
 *         - Start 1-byte interrupt receive
 */
HAL_StatusTypeDef RS485_Init(void)
{
    HAL_StatusTypeDef status;

    /* --- USART1 configuration --- */
    huart1.Instance             = USART1;
    huart1.Init.BaudRate        = RS485_BAUDRATE;
    huart1.Init.WordLength      = UART_WORDLENGTH_8B;
    huart1.Init.StopBits        = UART_STOPBITS_1;
    huart1.Init.Parity          = UART_PARITY_NONE;
    huart1.Init.Mode            = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl       = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling    = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling  = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.Init.ClockPrescaler  = UART_PRESCALER_DIV1;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    status = HAL_UART_Init(&huart1);
    if (status != HAL_OK) {
        Debug_Print("[RS485] UART init failed: %d\r\n", status);
        return status;
    }

    /* --- USART1 interrupt priority (7: lower than FDCAN=6, FreeRTOS safe) --- */
    HAL_NVIC_SetPriority(USART1_IRQn, 7, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

    /* --- Start 1-byte receive interrupt --- */
    s_rx_len = 0;
    HAL_UART_Receive_IT(&huart1, (uint8_t *)&s_rx_byte, 1);

    RS485_DE_LOW();

    Debug_Print("[RS485] Init OK - USART1 %ubps, DE/RE on PA8\r\n", RS485_BAUDRATE);
    return HAL_OK;
}

/**
 * @brief  RS485 data transmit (Half-duplex DE/RE control)
 * @param  data: data to transmit
 * @param  len:  data length
 * @retval HAL status
 *
 * @note   Flow:
 *         1. DE=HIGH (switch to transmit mode)
 *         2. HAL_UART_Transmit (polling, wait until complete)
 *         3. DE=LOW (return to receive mode)
 */
HAL_StatusTypeDef RS485_SendData(const uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef status;

    RS485_DE_HIGH();
    status = HAL_UART_Transmit(&huart1, data, len, HAL_MAX_DELAY);
    RS485_DE_LOW();

    return status;
}

/**
 * @brief  Forward CAN message to RS485 (CAN->RS485 routing)
 * @param  can_id: CAN ID (11-bit)
 * @param  data:   CAN data
 * @param  dlc:    data length (CAN-FD max 64)
 * @retval HAL status
 *
 * @note   RS485 frame format:
 *         [ID_H] [ID_L] [DLC] [DATA 0..N]
 *         ID_H = (can_id >> 8) & 0xFF  (upper 3 bits, 11-bit ID)
 *         ID_L = can_id & 0xFF          (lower 8 bits)
 *         DLC  = data length (0~64, CAN-FD support)
 *         DATA = CAN payload (DLC bytes)
 *         Maximum frame = 2(ID) + 1(DLC) + 64(DATA) = 67 bytes
 */
HAL_StatusTypeDef RS485_ForwardCANMessage(uint32_t can_id, const uint8_t *data, uint8_t dlc)
{
    uint8_t frame[3U + 64U];  /* 2(ID) + 1(DLC) + 64(DATA) = 67 bytes */
    uint8_t pos = 0;

    frame[pos++] = (uint8_t)((can_id >> 8) & 0xFFU);  /* ID upper */
    frame[pos++] = (uint8_t)(can_id & 0xFFU);           /* ID lower */
    frame[pos++] = dlc;

    for (uint8_t i = 0U; i < dlc && i < 64U; i++) {
        frame[pos++] = data[i];
    }

    Debug_Print("[ROUTE] CAN->RS485 ID:0x%03lX DLC:%u\r\n", can_id, dlc);
    return RS485_SendData(frame, pos);
}

/**
 * @brief  Restart receive after UART error
 * @note   Called from HAL_UART_ErrorCallback.
 *         s_rx_byte is static, so must be handled within this module.
 */
void RS485_RestartReceive(void)
{
    HAL_UART_Receive_IT(&huart1, (uint8_t *)&s_rx_byte, 1);
}

/**
 * @brief  UART receive complete callback (HAL shared)
 *
 * @note   USART1 interrupt-based receive:
 *         - Called on each 1-byte receive
 *         - Store byte in buffer
 *         - Deliver to Queue when buffer is full or frame ends
 *         - Re-enable next 1-byte receive
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (huart->Instance == USART1) {
        /* Store byte in RX buffer */
        if (s_rx_len < RS485_MAX_DATA_LEN) {
            s_rx_buf[s_rx_len] = (uint8_t)s_rx_byte;
            s_rx_len++;
        }

        /* Deliver to Queue when buffer is full */
        if (s_rx_len >= RS485_MAX_DATA_LEN) {
            RS485_RxMessage_t rx_msg;
            rx_msg.len = s_rx_len;
            (void)memcpy(rx_msg.data, (const void *)s_rx_buf, s_rx_len);

            xQueueSendFromISR(xRS485RxQueue, &rx_msg, &xHigherPriorityTaskWoken);
            s_rx_len = 0;
        }

        /* Re-enable next byte receive */
        HAL_UART_Receive_IT(&huart1, (uint8_t *)&s_rx_byte, 1);

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
