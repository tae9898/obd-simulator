/**
 * @file    uart_debug.c
 * @brief   UART 디버그 출력 구현
 * @note    USART2 (ST-LINK VCP)를 통한 printf 스타일 디버그 출력
 *         __io_putchar 오버라이드로 printf retarget 지원
 */

#include "uart_debug.h"
#include "stm32g4xx_hal_dma.h"
#include <stdarg.h>
#include <stdio.h>

/* === DMA non-blocking UART log === */
DMA_HandleTypeDef hdma_usart2_tx;

#define LOG_RING_SIZE 2048
static uint8_t    s_log_ring[LOG_RING_SIZE];
static volatile uint16_t s_log_head = 0;
static volatile uint16_t s_log_tail = 0;
static volatile uint8_t  s_dma_busy = 0;
static uint16_t s_dma_chunk_len = 0;

static void log_dma_try_start(void)
{
    if (s_dma_busy) return;
    if (s_log_head == s_log_tail) return;

    uint16_t chunk;
    if (s_log_head > s_log_tail) {
        chunk = s_log_head - s_log_tail;
    } else {
        chunk = LOG_RING_SIZE - s_log_tail;
    }
    s_dma_chunk_len = chunk;
    s_dma_busy = 1;
    if (HAL_UART_Transmit_DMA(&huart2, &s_log_ring[s_log_tail], chunk) != HAL_OK) {
        s_dma_busy = 0;  /* 실패 시 재시도 허용 (stuck 방지) */
    }
}

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
    if (ch == (int)'\n') {
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
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (len < 0) return;
    if (len >= (int)sizeof(buf)) len = (int)sizeof(buf) - 1;

    if (xUartMutex != NULL) {
        xSemaphoreTake(xUartMutex, portMAX_DELAY);
    }

    for (int i = 0; i < len; i++) {
        uint16_t next = (uint16_t)((s_log_head + 1U) % LOG_RING_SIZE);
        if (next == s_log_tail) break;
        s_log_ring[s_log_head] = (uint8_t)buf[i];
        s_log_head = next;
    }

    if (xUartMutex != NULL) {
        xSemaphoreGive(xUartMutex);
    }

    log_dma_try_start();
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

    pos += snprintf(buf + pos, sizeof(buf) - (size_t)pos,
                    "[CAN] RX ID:0x%03lX DLC:%lu Data:", id, len);
    for (uint32_t i = 0U; i < len && pos < (int)(sizeof(buf) - 4U); i++) {
        pos += snprintf(buf + pos, sizeof(buf) - (size_t)pos, " %02X", data[i]);
    }
    snprintf(buf + pos, sizeof(buf) - (size_t)pos, "\r\n");

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

    pos += snprintf(buf + pos, sizeof(buf) - (size_t)pos,
                    "[CAN] TX ID:0x%03lX DLC:%lu Data:", id, len);
    for (uint32_t i = 0U; i < len && pos < (int)(sizeof(buf) - 4U); i++) {
        pos += snprintf(buf + pos, sizeof(buf) - (size_t)pos, " %02X", data[i]);
    }
    snprintf(buf + pos, sizeof(buf) - (size_t)pos, "\r\n");

    Debug_Print("%s", buf);
}

/**
 * @brief  DMA TX 완료 콜백 (ISR 컨텍스트)
 * @note   ring buffer tail 업데이트 후 다음 청크 전송
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        s_log_tail = (uint16_t)((s_log_tail + s_dma_chunk_len) % LOG_RING_SIZE);
        s_dma_busy = 0;
        log_dma_try_start();
    }
}
