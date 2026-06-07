/**
 * @file    rs485.c
 * @brief   RS485 통신 구현 (USART1 + MAX485 DE/RE 제어)
 * @note    Half-duplex RS485: GPIO PA8로 송수신 방향 전환
 *         인터럽트 기반 1바이트 수신 → 프레임 조립 → Queue 전달
 */

#include "rs485.h"
#include "uart_debug.h"
#include <string.h>

/* === USART1 핸들러 (main.c에서 정의) === */
extern UART_HandleTypeDef huart1;

/* === RS485 RX Queue (main.c에서 생성) === */
extern QueueHandle_t xRS485RxQueue;

/* === 인터럽트 수신 버퍼 === */
static volatile uint8_t  s_rx_byte;
static volatile uint8_t  s_rx_buf[RS485_MAX_DATA_LEN];
static volatile uint8_t  s_rx_len = 0;

/**
 * @brief  RS485 초기화
 * @retval HAL_OK = 성공
 *
 * @note   설정:
 *         - USART1, PA9(TX) / PA10(RX)
 *         - 115200 baud, 8N1
 *         - PA8 GPIO 출력: MAX485 DE/RE (초기=LOW, 수신 모드)
 *         - USART1 인터럽트 우선순위 7 (FDCAN=6보다 낮음)
 *         - 1바이트 인터럽트 수신 시작
 */
HAL_StatusTypeDef RS485_Init(void)
{
    HAL_StatusTypeDef status;

    /* --- USART1 설정 --- */
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

    /* --- USART1 인터럽트 우선순위 (7: FDCAN=6보다 낮음, FreeRTOS 안전) --- */
    HAL_NVIC_SetPriority(USART1_IRQn, 7, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

    /* --- 1바이트 수신 인터럽트 시작 --- */
    s_rx_len = 0;
    HAL_UART_Receive_IT(&huart1, (uint8_t *)&s_rx_byte, 1);

    RS485_DE_LOW();

    Debug_Print("[RS485] Init OK - USART1 %ubps, DE/RE on PA8\r\n", RS485_BAUDRATE);
    return HAL_OK;
}

/**
 * @brief  RS485 데이터 송신 (Half-duplex DE/RE 제어)
 * @param  data: 송신할 데이터
 * @param  len:  데이터 길이
 * @retval HAL 상태
 *
 * @note   흐름:
 *         1. DE=HIGH (송신 모드 전환)
 *         2. HAL_UART_Transmit (폴링, 완료까지 대기)
 *         3. DE=LOW (수신 모드 복귀)
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
 * @brief  CAN 메시지를 RS485로 포워딩 (CAN→RS485 라우팅)
 * @param  can_id: CAN ID (11-bit)
 * @param  data:   CAN 데이터
 * @param  dlc:    데이터 길이
 * @retval HAL 상태
 *
 * @note   RS485 프레임 포맷:
 *         [ID_H] [ID_L] [DLC] [DATA 0..N]
 *         ID_H = (can_id >> 8) & 0xFF  (상위 3비트, 11-bit ID)
 *         ID_L = can_id & 0xFF          (하위 8비트)
 *         DLC  = 데이터 길이 (0~8)
 *         DATA = CAN 페이로드
 */
HAL_StatusTypeDef RS485_ForwardCANMessage(uint32_t can_id, const uint8_t *data, uint8_t dlc)
{
    uint8_t frame[11];  /* 최대: 2(ID) + 1(DLC) + 8(DATA) = 11바이트 */
    uint8_t pos = 0;

    frame[pos++] = (uint8_t)((can_id >> 8) & 0xFF);  /* ID 상위 */
    frame[pos++] = (uint8_t)(can_id & 0xFF);           /* ID 하위 */
    frame[pos++] = dlc;

    for (uint8_t i = 0; i < dlc && i < 8; i++) {
        frame[pos++] = data[i];
    }

    Debug_Print("[ROUTE] CAN→RS485 ID:0x%03lX DLC:%u\r\n", can_id, dlc);
    return RS485_SendData(frame, pos);
}

/**
 * @brief  UART 에러 후 수신 재시작
 * @note   HAL_UART_ErrorCallback에서 호출.
 *         s_rx_byte가 static이므로 이 모듈 내부에서 처리해야 함.
 */
void RS485_RestartReceive(void)
{
    HAL_UART_Receive_IT(&huart1, (uint8_t *)&s_rx_byte, 1);
}

/**
 * @brief  UART 수신 완료 콜백 (HAL 공유)
 *
 * @note   USART1 인터럽트 기반 수신:
 *         - 1바이트 수신 시마다 호출
 *         - 버퍼에 바이트 저장
 *         - 버퍼가 가득 차거나 프레임 종료 시 Queue에 전달
 *         - 다음 1바이트 수신 재활성화
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (huart->Instance == USART1) {
        /* 바이트를 RX 버퍼에 저장 */
        if (s_rx_len < RS485_MAX_DATA_LEN) {
            s_rx_buf[s_rx_len] = (uint8_t)s_rx_byte;
            s_rx_len++;
        }

        /* 버퍼가 가득 차면 Queue에 전달 */
        if (s_rx_len >= RS485_MAX_DATA_LEN) {
            RS485_RxMessage_t rx_msg;
            rx_msg.len = s_rx_len;
            (void)memcpy(rx_msg.data, (const void *)s_rx_buf, s_rx_len);

            xQueueSendFromISR(xRS485RxQueue, &rx_msg, &xHigherPriorityTaskWoken);
            s_rx_len = 0;
        }

        /* 다음 바이트 수신 재활성화 */
        HAL_UART_Receive_IT(&huart1, (uint8_t *)&s_rx_byte, 1);

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
