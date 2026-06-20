/**
 * @file    stm32g4xx_hal_msp.c
 * @brief   HAL MSP (MCU Support Package) 초기화
 * @note    FDCAN1, USART2 주변기기 클럭 및 핀 설정
 */

#include "main.h"
#include "rs485.h"

/**
 * @brief  FDCAN1 MSP 초기화 (HAL_FDCAN_Init에서 자동 호출)
 * @param  hfdcan: FDCAN 핸들러 포인터
 * @retval None
 *
 * @note   설정:
 *         - FDCAN1 클럭 활성화
 *         - PA11 (FDCAN1_RX), PA12 (FDCAN1_TX) GPIO AF 설정
 *         - FDCAN1 인터럽트 (NVIC) 활성화
 */
void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *hfdcan)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* --- FDCAN1 클럭 활성화 --- */
    __HAL_RCC_FDCAN_CLK_ENABLE();

    /* --- FDCAN1 핀 GPIO 클럭 활성화 --- */
    __HAL_RCC_GPIOA_CLK_ENABLE();   /* PA12 (TX) */
    __HAL_RCC_GPIOB_CLK_ENABLE();   /* PB8 (RX) — PA11 손상으로 이관 */

    /* --- FDCAN1_TX (PA12) GPIO 설정 --- */
    GPIO_InitStruct.Pin       = FDCAN1_TX_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = FDCAN1_TX_AF;  /* AF9 */
    HAL_GPIO_Init(FDCAN1_TX_PORT, &GPIO_InitStruct);

    /* --- FDCAN1_RX (PB8) GPIO 설정 --- */
    GPIO_InitStruct.Pin       = FDCAN1_RX_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;   /* RX는 풀업 권장 */
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = FDCAN1_RX_AF;  /* AF9 */
    HAL_GPIO_Init(FDCAN1_RX_PORT, &GPIO_InitStruct);

    /* --- FDCAN1 인터럽트 (NVIC) 설정 --- */
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 0U, 0U);  /* 최우선 순위 */
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
}

/**
 * @brief  FDCAN1 MSP 해제
 * @param  hfdcan: FDCAN 핸들러 포인터
 * @retval None
 */
void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef *hfdcan)
{
    /* GPIO 해제 */
    HAL_GPIO_DeInit(FDCAN1_TX_PORT, FDCAN1_TX_PIN);
    HAL_GPIO_DeInit(FDCAN1_RX_PORT, FDCAN1_RX_PIN);

    /* FDCAN1 클럭 비활성화 */
    __HAL_RCC_FDCAN_CLK_DISABLE();

    /* NVIC 인터럽트 비활성화 */
    HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
}

/**
 * @brief  USART2 MSP 초기화 (HAL_UART_Init에서 자동 호출)
 * @param  huart: UART 핸들러 포인터
 * @retval None
 *
 * @note   설정:
 *         - USART2 클럭 활성화
 *         - PA2 (TX), PA3 (RX) GPIO AF 설정
 *         - UART 인터럽트는 미사용 (폴링 모드)
 */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (huart->Instance == USART2) {
        /* --- USART2 클럭 활성화 --- */
        __HAL_RCC_USART2_CLK_ENABLE();

        /* --- USART2 GPIO 클럭 활성화 --- */
        __HAL_RCC_GPIOA_CLK_ENABLE();

        /* --- USART2_TX (PA2) GPIO 설정 --- */
        GPIO_InitStruct.Pin       = USART2_TX_PIN;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = USART2_TX_AF;  /* AF7 */
        HAL_GPIO_Init(USART2_TX_PORT, &GPIO_InitStruct);

        /* --- USART2_RX (PA3) GPIO 설정 --- */
        GPIO_InitStruct.Pin       = USART2_RX_PIN;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_PULLUP;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = USART2_RX_AF;  /* AF7 */
        HAL_GPIO_Init(USART2_RX_PORT, &GPIO_InitStruct);
    }
    else if (huart->Instance == USART1) {
        /* --- USART1 클럭 활성화 --- */
        __HAL_RCC_USART1_CLK_ENABLE();

        /* --- USART1 GPIO 클럭 활성화 --- */
        __HAL_RCC_GPIOA_CLK_ENABLE();

        /* --- USART1_TX (PA9) GPIO 설정 --- */
        GPIO_InitStruct.Pin       = RS485_TX_PIN;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = RS485_TX_AF;  /* AF7 */
        HAL_GPIO_Init(RS485_TX_PORT, &GPIO_InitStruct);

        /* --- USART1_RX (PA10) GPIO 설정 --- */
        GPIO_InitStruct.Pin       = RS485_RX_PIN;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_PULLUP;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = RS485_RX_AF;  /* AF7 */
        HAL_GPIO_Init(RS485_RX_PORT, &GPIO_InitStruct);

        /* --- MAX485 DE/RE (PA8) GPIO 출력 설정 --- */
        GPIO_InitStruct.Pin       = RS485_DE_PIN;
        GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(RS485_DE_PORT, &GPIO_InitStruct);
        RS485_DE_LOW();  /* 초기: 수신 모드 */
    }
}

/**
 * @brief  USART2 MSP 해제
 * @param  huart: UART 핸들러 포인터
 * @retval None
 */
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        /* GPIO 해제 */
        HAL_GPIO_DeInit(USART2_TX_PORT, USART2_TX_PIN);
        HAL_GPIO_DeInit(USART2_RX_PORT, USART2_RX_PIN);

        /* USART2 클럭 비활성화 */
        __HAL_RCC_USART2_CLK_DISABLE();
    }
    else if (huart->Instance == USART1) {
        HAL_GPIO_DeInit(RS485_TX_PORT, RS485_TX_PIN);
        HAL_GPIO_DeInit(RS485_RX_PORT, RS485_RX_PIN);
        HAL_GPIO_DeInit(RS485_DE_PORT, RS485_DE_PIN);

        __HAL_RCC_USART1_CLK_DISABLE();
    }
}
