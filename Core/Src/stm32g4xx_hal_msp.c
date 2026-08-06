/**
 * @file    stm32g4xx_hal_msp.c
 * @brief   HAL MSP (MCU Support Package) initialization
 * @note    FDCAN1, USART2 peripheral clock and pin configuration
 */

#include "main.h"
#include "rs485.h"

/**
 * @brief  FDCAN1 MSP initialization (called automatically by HAL_FDCAN_Init)
 * @param  hfdcan: FDCAN handler pointer
 * @retval None
 *
 * @note   Configuration:
 *         - FDCAN1 clock enable
 *         - PA11 (FDCAN1_RX), PA12 (FDCAN1_TX) GPIO AF configuration
 *         - FDCAN1 interrupt (NVIC) enable
 */
void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *hfdcan)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* --- FDCAN1 clock enable --- */
    __HAL_RCC_FDCAN_CLK_ENABLE();

    /* --- FDCAN1 pin GPIO clock enable --- */
    __HAL_RCC_GPIOA_CLK_ENABLE();   /* PA11 (RX) */
    __HAL_RCC_GPIOB_CLK_ENABLE();   /* PB9 (TX) */

    /* --- FDCAN1_TX (PB9) GPIO configuration --- */
    GPIO_InitStruct.Pin       = FDCAN1_TX_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = FDCAN1_TX_AF;  /* AF9 */
    HAL_GPIO_Init(FDCAN1_TX_PORT, &GPIO_InitStruct);

    /* --- FDCAN1_RX (PA11) GPIO configuration --- */
    GPIO_InitStruct.Pin       = FDCAN1_RX_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;   /* RX pull-up recommended */
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = FDCAN1_RX_AF;  /* AF9 */
    HAL_GPIO_Init(FDCAN1_RX_PORT, &GPIO_InitStruct);

    /* --- FDCAN1 interrupt (NVIC) configuration --- */
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 0U, 0U);  /* Highest priority */
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
}

/**
 * @brief  FDCAN1 MSP de-initialization
 * @param  hfdcan: FDCAN handler pointer
 * @retval None
 */
void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef *hfdcan)
{
    /* GPIO de-init */
    HAL_GPIO_DeInit(FDCAN1_TX_PORT, FDCAN1_TX_PIN);
    HAL_GPIO_DeInit(FDCAN1_RX_PORT, FDCAN1_RX_PIN);

    /* FDCAN1 clock disable */
    __HAL_RCC_FDCAN_CLK_DISABLE();

    /* NVIC interrupt disable */
    HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
}

/**
 * @brief  USART2 MSP initialization (called automatically by HAL_UART_Init)
 * @param  huart: UART handler pointer
 * @retval None
 *
 * @note   Configuration:
 *         - USART2 clock enable
 *         - PA2 (TX), PA3 (RX) GPIO AF configuration
 *         - UART interrupts unused (polling mode)
 */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (huart->Instance == USART2) {
        /* --- USART2 clock enable --- */
        __HAL_RCC_USART2_CLK_ENABLE();

        /* --- USART2 GPIO clock enable --- */
        __HAL_RCC_GPIOA_CLK_ENABLE();

        /* --- USART2_TX (PA2) GPIO configuration --- */
        GPIO_InitStruct.Pin       = USART2_TX_PIN;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = USART2_TX_AF;  /* AF7 */
        HAL_GPIO_Init(USART2_TX_PORT, &GPIO_InitStruct);

        /* --- USART2_RX (PA3) GPIO configuration --- */
        GPIO_InitStruct.Pin       = USART2_RX_PIN;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_PULLUP;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = USART2_RX_AF;  /* AF7 */
        HAL_GPIO_Init(USART2_RX_PORT, &GPIO_InitStruct);
    }
    else if (huart->Instance == USART1) {
        /* --- USART1 clock enable --- */
        __HAL_RCC_USART1_CLK_ENABLE();

        /* --- USART1 GPIO clock enable --- */
        __HAL_RCC_GPIOA_CLK_ENABLE();

        /* --- USART1_TX (PA9) GPIO configuration --- */
        GPIO_InitStruct.Pin       = RS485_TX_PIN;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = RS485_TX_AF;  /* AF7 */
        HAL_GPIO_Init(RS485_TX_PORT, &GPIO_InitStruct);

        /* --- USART1_RX (PA10) GPIO configuration --- */
        GPIO_InitStruct.Pin       = RS485_RX_PIN;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_PULLUP;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = RS485_RX_AF;  /* AF7 */
        HAL_GPIO_Init(RS485_RX_PORT, &GPIO_InitStruct);

        /* --- MAX485 DE/RE (PA8) GPIO output configuration --- */
        GPIO_InitStruct.Pin       = RS485_DE_PIN;
        GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(RS485_DE_PORT, &GPIO_InitStruct);
        RS485_DE_LOW();  /* Initial: receive mode */
    }
}

/**
 * @brief  USART2 MSP de-initialization
 * @param  huart: UART handler pointer
 * @retval None
 */
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        /* GPIO de-init */
        HAL_GPIO_DeInit(USART2_TX_PORT, USART2_TX_PIN);
        HAL_GPIO_DeInit(USART2_RX_PORT, USART2_RX_PIN);

        /* USART2 clock disable */
        __HAL_RCC_USART2_CLK_DISABLE();
    }
    else if (huart->Instance == USART1) {
        HAL_GPIO_DeInit(RS485_TX_PORT, RS485_TX_PIN);
        HAL_GPIO_DeInit(RS485_RX_PORT, RS485_RX_PIN);
        HAL_GPIO_DeInit(RS485_DE_PORT, RS485_DE_PIN);

        __HAL_RCC_USART1_CLK_DISABLE();
    }
}
