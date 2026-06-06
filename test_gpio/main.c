/**
 * @file    main.c
 * @brief   GPIO 핀 검증 테스트 - PA11, PA12 토글
 * @note    LED + 저항(330~1kΩ)으로 PA11/PA12가 실제 나오는지 확인
 *
 * 사용법:
 *   1. LED(+저항)를 PA11에 연결 (PA11 → 저항 → LED → GND)
 *   2. LED(+저항)를 PA12에 연결 (PA12 → 저항 → LED → GND)
 *   3. 빌드 후 플래시
 *   4. screen /dev/ttyACM0 115200 으로 시리얼 확인
 *
 * 기대 동작:
 *   - PA11 LED: 2초 켜짐 → 2초 꺼짐 반복
 *   - PA12 LED: PA11과 반대로 깜빡임 (교차)
 *   - 보드 LED(LD4, PA5): PA11과 같이 깜빡임
 */

#include "stm32g4xx_hal.h"
#include <stdio.h>
#include <string.h>

/* === 함수 프로토타입 === */
void SystemClock_Config(void);

/* === 핸들러 === */
UART_HandleTypeDef huart2;

/* === 심플 UART 출력 === */
static void uart_print(const char *msg)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}

/* === UART2 초기화 (PA2 TX, ST-LINK VCP) === */
static void uart_init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();

    /* PA2 = USART2_TX */
    GPIO_InitTypeDef g = {0};
    g.Pin       = GPIO_PIN_2;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_LOW;
    g.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &g);

    huart2.Instance                    = USART2;
    huart2.Init.BaudRate               = 115200U;
    huart2.Init.WordLength             = UART_WORDLENGTH_8B;
    huart2.Init.StopBits               = UART_STOPBITS_1;
    huart2.Init.Parity                 = UART_PARITY_NONE;
    huart2.Init.Mode                   = UART_MODE_TX;
    huart2.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling           = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.Init.ClockPrescaler         = UART_PRESCALER_DIV1;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    HAL_UART_Init(&huart2);
}

/* === 테스트 핀 초기화: PA5(LED), PA11, PA12 → GPIO 출력 === */
static void gpio_test_init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;

    /* PA5  - 온보드 LED (LD4, active low) */
    g.Pin = GPIO_PIN_5;
    HAL_GPIO_Init(GPIOA, &g);

    /* PA11 - 테스트 핀 1 */
    g.Pin = GPIO_PIN_11;
    HAL_GPIO_Init(GPIOA, &g);

    /* PA12 - 테스트 핀 2 */
    g.Pin = GPIO_PIN_12;
    HAL_GPIO_Init(GPIOA, &g);

    /* 초기값: 전부 LOW */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,  GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    uart_init();
    gpio_test_init();

    uart_print("\r\n");
    uart_print("================================\r\n");
    uart_print("  GPIO PIN TEST - PA11 / PA12\r\n");
    uart_print("================================\r\n");
    uart_print("PA5  = Onboard LED (LD4)\r\n");
    uart_print("PA11 = Test pin (FDCAN1_RX)\r\n");
    uart_print("PA12 = Test pin (FDCAN1_TX)\r\n");
    uart_print("LED+R to PA11, LED+R to PA12\r\n");
    uart_print("================================\r\n");
    uart_print("Starting blink test...\r\n\r\n");

    uint32_t count = 0;

    while (1) {
        /* --- Phase A: PA11=HIGH, PA12=LOW --- */
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
        /* LED: PA11=HIGH면 LED 켜짐 (active high 회로) */
        /* 보드 LED: PA5 active low라 SET=OFF, RESET=ON */
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);  /* LD4 ON */
        uart_print("[A] PA11=HIGH PA12=LOW  LED=ON  #");
        /* print count manually */
        {
            char buf[12];
            int n = 0;
            uint32_t tmp = count;
            char digits[11];
            if (tmp == 0) { digits[0] = '0'; n = 1; }
            else { while (tmp) { digits[n++] = '0' + (tmp % 10); tmp /= 10; } }
            for (int i = n - 1; i >= 0; i--) buf[n - 1 - i] = digits[i];
            buf[n] = '\r'; buf[n+1] = '\n'; buf[n+2] = '\0';
            uart_print(buf);
        }
        HAL_Delay(2000);

        /* --- Phase B: PA11=LOW, PA12=HIGH --- */
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);    /* LD4 OFF */
        uart_print("[B] PA11=LOW  PA12=HIGH LED=OFF #");
        {
            char buf[12];
            int n = 0;
            uint32_t tmp = count;
            char digits[11];
            if (tmp == 0) { digits[0] = '0'; n = 1; }
            else { while (tmp) { digits[n++] = '0' + (tmp % 10); tmp /= 10; } }
            for (int i = n - 1; i >= 0; i--) buf[n - 1 - i] = digits[i];
            buf[n] = '\r'; buf[n+1] = '\n'; buf[n+2] = '\0';
            uart_print(buf);
        }
        HAL_Delay(2000);

        count++;
    }
}

/* === 시스템 클럭: HSI 16MHz (심플 설정) === */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef       osc = {0};
    RCC_ClkInitTypeDef       clk = {0};

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState       = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState   = RCC_PLL_NONE;
    HAL_RCC_OscConfig(&osc);

    clk.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                         RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_0);
}

/* === 인터럽트 핸들러 === */
void SysTick_Handler(void)
{
    HAL_IncTick();
}

void HardFault_Handler(void)
{
    while (1);
}
