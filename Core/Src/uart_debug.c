/**
 * @file    uart_debug.c
 * @brief   UART debug output implementation
 * @note    printf-style debug output via USART2 (ST-LINK VCP)
 *         printf retarget support via __io_putchar override
 */

#include "uart_debug.h"
#include <stdarg.h>
#include <stdio.h>

/**
 * @brief  USART2 debug port initialization
 * @param  huart: UART handler pointer
 * @retval HAL_OK = success
 *
 * @note   Configuration:
 *         - USART2, PA2(TX) / PA3(RX)
 *         - 115200 baud, 8N1
 *         - TX-only usage (debug output only)
 *         - Clock source: PCLK1 (42.5MHz)
 *         - BRR = PCLK1 / baud = 42500000 / 115200 = 368.9 -> 369
 */
HAL_StatusTypeDef UART_DebugInit(UART_HandleTypeDef *huart)
{
    HAL_StatusTypeDef status;

    /* --- UART handler configuration --- */
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

    /* --- HAL UART initialization --- */
    status = HAL_UART_Init(huart);

    if (status == HAL_OK) {
        /* printf retarget: disable stdout buffering (immediate output) */
        setvbuf(stdout, NULL, _IONBF, 0);
    }

    return status;
}

/**
 * @brief  Single character transmit (for printf retarget)
 * @param  ch: character to transmit
 * @retval transmitted character
 *
 * @note   Works with both ARM Compiler/Keil and GNU Toolchain
 *         Stub enables printf("%d", val) etc. to work
 */
int __io_putchar(int ch)
{
    /* newline character -> CR+LF conversion */
    if (ch == (int)'\n') {
        HAL_UART_Transmit(&huart2, (uint8_t *)"\r", 1U, HAL_MAX_DELAY);
    }

    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1U, HAL_MAX_DELAY);
    return ch;
}

/**
 * @brief  printf-style debug output (USART2)
 * @param  fmt: printf format string
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

    /* Blocking transmit. (rev) DMA ring buffer path confirmed non-functional via
     * SWD+blocking test, so simplified. Debug_Print is only called from task/main
     * context (ISR-FDCAN RX callback only does queue send) -> blocking safe.
     * 256B@115200 ~ 22ms < 200ms timeout. */
    HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)len, 200U);

    if (xUartMutex != NULL) {
        xSemaphoreGive(xUartMutex);
    }
}

/**
 * @brief  Log received CAN message
 * @param  id:   CAN ID
 * @param  data: data buffer
 * @param  len:  data length
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
 * @brief  Log transmitted CAN message
 * @param  id:   CAN ID
 * @param  data: data buffer
 * @param  len:  data length
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
