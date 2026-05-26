/**
 * @file    uart_debug.c
 * @brief   UART 디버그 출력 구현
 * @note    USART2 (ST-LINK VCP)를 통한 printf 스타일 디버그 출력
 *         __io_putchar 오버라이드로 printf retarget 지원
 */

#include "uart_debug.h"
#include <stdarg.h>
#include <stdio.h>

/**
 * @brief  USART2 디버그 포트 초기화
 * @param  huart: UART 핸들러 포인터
 * @retval HAL_OK = 성공
 *
 * @note   설정:
 *         - USART2, PA2(TX) / PA3(RX)
 *         - 115200 baud, 8N1
 *         - TX-only 사용 (디버그 출력 전용)
 *         - 클럭 소스: PCLK1 (42.5MHz)
 *         - BRR = PCLK1 / baud = 42500000 / 115200 = 368.9 -> 369
 */
HAL_StatusTypeDef UART_DebugInit(UART_HandleTypeDef *huart)
{
    HAL_StatusTypeDef status;

    /* --- UART 핸들러 설정 --- */
    huart->Instance             = USART2;
    huart->Init.BaudRate        = 115200U;
    huart->Init.WordLength      = UART_WORDLENGTH_8B;
    huart->Init.StopBits        = UART_STOPBITS_1;
    huart->Init.Parity          = UART_PARITY_NONE;
    huart->Init.Mode            = UART_MODE_TX_RX;
    huart->Init.HwFlowCtl       = UART_HWCONTROL_NONE;
    huart->Init.OverSampling    = UART_OVERSAMPLING_16;
    huart->Init.OneBitSampling  = UART_ONE_BIT_SAMPLE_DISABLE;
    huart->Init.ClockPrescaler  = UART_PRESCALER_DIV1;
    huart->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    /* --- HAL UART 초기화 --- */
    status = HAL_UART_Init(huart);

    if (status == HAL_OK) {
        /* printf retarget: stdout 버퍼 비활성화 (즉시 출력) */
        setvbuf(stdout, NULL, _IONBF, 0);
    }

    return status;
}

/**
 * @brief  단일 문자 전송 (printf retarget용)
 * @param  ch: 전송할 문자
 * @retval 전송된 문자
 *
 * @note   ARM Compiler/Keil과 GNU Toolchain 모두에서 동작
 *         Newlib 스텁으로 printf("%d", val) 등이 동작함
 */
int __io_putchar(int ch)
{
    /* 개행 문자 -> CR+LF 변환 */
    if (ch == '\n') {
        HAL_UART_Transmit(&huart2, (uint8_t *)"\r", 1U, HAL_MAX_DELAY);
    }

    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1U, HAL_MAX_DELAY);
    return ch;
}

/**
 * @brief  printf 스타일 디버그 출력 (USART2)
 * @param  fmt: printf 형식 문자열
 * @retval None
 */
void Debug_Print(const char *fmt, ...)
{
    char buf[256];
    va_list args;

    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    /* 문자열 길이 계산 */
    uint16_t len = 0;
    while (buf[len] != '\0' && len < sizeof(buf)) {
        len++;
    }

    /* Mutex로 UART 보호 (스케줄러 시작 전에는 xUartMutex == NULL) */
    if (xUartMutex != NULL) {
        xSemaphoreTake(xUartMutex, portMAX_DELAY);
    }

    /* USART2로 전송 */
    HAL_UART_Transmit(&huart2, (uint8_t *)buf, len, HAL_MAX_DELAY);

    if (xUartMutex != NULL) {
        xSemaphoreGive(xUartMutex);
    }
}

/**
 * @brief  수신된 CAN 메시지 로그 출력
 * @param  id:   CAN ID
 * @param  data: 데이터 버퍼
 * @param  len:  데이터 길이
 */
void Debug_LogCAN_Rx(uint32_t id, const uint8_t *data, uint32_t len)
{
    char buf[256];
    int pos = 0;

    pos += snprintf(buf + pos, sizeof(buf) - pos,
                    "[CAN] RX ID:0x%03lX DLC:%lu Data:", id, len);
    for (uint32_t i = 0U; i < len && pos < (int)(sizeof(buf) - 4); i++) {
        pos += snprintf(buf + pos, sizeof(buf) - pos, " %02X", data[i]);
    }
    snprintf(buf + pos, sizeof(buf) - pos, "\r\n");

    Debug_Print("%s", buf);
}

/**
 * @brief  전송한 CAN 메시지 로그 출력
 * @param  id:   CAN ID
 * @param  data: 데이터 버퍼
 * @param  len:  데이터 길이
 */
void Debug_LogCAN_Tx(uint32_t id, const uint8_t *data, uint32_t len)
{
    char buf[256];
    int pos = 0;

    pos += snprintf(buf + pos, sizeof(buf) - pos,
                    "[CAN] TX ID:0x%03lX DLC:%lu Data:", id, len);
    for (uint32_t i = 0U; i < len && pos < (int)(sizeof(buf) - 4); i++) {
        pos += snprintf(buf + pos, sizeof(buf) - pos, " %02X", data[i]);
    }
    snprintf(buf + pos, sizeof(buf) - pos, "\r\n");

    Debug_Print("%s", buf);
}
