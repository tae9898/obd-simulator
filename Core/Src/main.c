/**
 * @file    main.c
 * @brief   OBD-II ECU 시뮬레이터 메인 루프
 * @note    HAL 초기화, FDCAN/UART 설정, 시뮬레이션 값 주기적 업데이트
 *          STM32G431RB Nucleo 보드용
 */

/* === 표준 라이브러리 === */
#include <stdio.h>
#include <string.h>

/* === 프로젝트 헤더 === */
#include "main.h"
#include "obd2_simulator.h"
#include "fdcan_config.h"
#include "uart_debug.h"

/* === 핸들러 전역 변수 === */
FDCAN_HandleTypeDef hfdcan1;   /* FDCAN1 핸들러 */
UART_HandleTypeDef  huart2;    /* USART2 핸들러 (디버그) */

/* === 시뮬레이션 상태 전역 변수 === */
OBD2_SimState_t g_sim_state;

/* === LED 토글 카운터 === */
static uint32_t s_led_tick_counter = 0;

/* === 함수 프로토타입 === */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

/**
 * @brief  메인 진입점
 * @retval int (0 = 정상)
 */
int main(void)
{
    /* --- HAL 라이브러리 초기화 --- */
    HAL_Init();

    /* --- 시스템 클럭 설정: HSI 16MHz -> PLL -> 170MHz --- */
    SystemClock_Config();

    /* --- GPIO 초기화 (LED 등) --- */
    MX_GPIO_Init();

    /* --- USART2 디버그 포트 초기화 --- */
    if (UART_DebugInit(&huart2) != HAL_OK) {
        /* UART 초기화 실패 - LED로 에러 표시 */
        while (1) {
            LED_ON();
            HAL_Delay(100);
            LED_OFF();
            HAL_Delay(100);
        }
    }

    Debug_Print("[INIT] OBD-II ECU Simulator v1.0\r\n");
    Debug_Print("[INIT] STM32G431RB Nucleo Board\r\n");
    Debug_Print("[INIT] SYSCLK = %lu MHz\r\n", SYSCLK_FREQ / 1000000U);

    /* --- FDCAN1 초기화 (Classic CAN 500kbps) --- */
    if (FDCAN1_Init(&hfdcan1) != HAL_OK) {
        Debug_Print("[ERROR] FDCAN1_Init failed\r\n");
        /* FDCAN 초기화 실패 - LED 빠른 깜빡임 */
        while (1) {
            LED_ON();
            HAL_Delay(50);
            LED_OFF();
            HAL_Delay(50);
        }
    }

    /* --- FDCAN1 필터 설정 (CAN ID 0x7E0만 수신) --- */
    if (FDCAN1_ConfigureFilters(&hfdcan1) != HAL_OK) {
        Debug_Print("[ERROR] FDCAN1 filter config failed\r\n");
        while (1) {
            LED_ON();
            HAL_Delay(200);
            LED_OFF();
            HAL_Delay(200);
        }
    }

    /* --- FDCAN1 RX 인터럽트 활성화 --- */
    if (FDCAN1_StartNotification(&hfdcan1) != HAL_OK) {
        Debug_Print("[ERROR] FDCAN1 notification failed\r\n");
        while (1) {
            LED_ON();
            HAL_Delay(300);
            LED_OFF();
            HAL_Delay(300);
        }
    }

    Debug_Print("[INIT] FDCAN1 ready - Classic CAN 500kbps\r\n");
    Debug_Print("[INIT] Listening on CAN ID 0x%03X\r\n", OBD2_REQUEST_ID);
    Debug_Print("[INIT] Supported PIDs: 0x00, 0x05, 0x0C, 0x0D\r\n");

    /* --- 시뮬레이션 상태 초기값 설정 --- */
    g_sim_state.engine_rpm      = RPM_IDLE;
    g_sim_state.coolant_temp    = COOLANT_TEMP_MIN;
    g_sim_state.vehicle_speed   = 0U;
    g_sim_state.rpm_direction   = 0U;  /* 램프 업 시작 */
    g_sim_state.temp_direction  = 0U;  /* 증가 시작 */
    g_sim_state.speed_direction = 0U;  /* 증가 시작 */

    /* --- 정상 동작 표시: LED 느린 깜빡임 --- */
    LED_ON();

    /* ========================================
     * 메인 루프
     * ======================================== */
    while (1)
    {
        /* --- 시뮬레이션 값 업데이트 (10ms 주기) --- */
        OBD2_UpdateSimValues(&g_sim_state);

        /* --- LED 토글 (500ms 주기) --- */
        s_led_tick_counter++;
        if (s_led_tick_counter >= (LED_TOGGLE_PERIOD_MS / SIM_UPDATE_PERIOD_MS)) {
            s_led_tick_counter = 0;
            LED_TOGGLE();
        }

        /* --- 10ms 대기 --- */
        HAL_Delay(SIM_UPDATE_PERIOD_MS);
    }
}

/**
 * @brief  시스템 클럭 설정
 * @note   HSI 16MHz -> PLL -> SYSCLK 170MHz
 *         - PLLM = 4  (HSI/4 = 4MHz)
 *         - PLLN = 85 (4MHz * 85 = 340MHz VCO)
 *         - PLLP = 2  (340MHz / 2 = 170MHz SYSCLK)
 *         - PLLQ = 2  (340MHz / 2 = 170MHz, FDCAN에 사용 가능)
 *         - PLLR = 2  (340MHz / 2 = 170MHz, SYSCLK용)
 *         - AHB prescaler = 1  -> HCLK = 170MHz
 *         - APB1 prescaler = 4 -> PCLK1 = 42.5MHz
 *         - APB2 prescaler = 2 -> PCLK2 = 85MHz
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef        RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef        RCC_ClkInitStruct = {0};

    /** 1. 전원 설정: Scale 1 모드 (170MHz 동작에 필요) */
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** 2. RCC 발진기 설정: HSI를 PLL 소스로 사용 */
    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState            = RCC_HSE_ON;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM            = 4U;   /* HSI/4 = 4MHz */
    RCC_OscInitStruct.PLL.PLLN            = 85U;  /* 4MHz * 85 = 340MHz VCO */
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV2;  /* 340/2 = 170MHz */
    RCC_OscInitStruct.PLL.PLLQ            = RCC_PLLQ_DIV2;  /* 340/2 = 170MHz */
    RCC_OscInitStruct.PLL.PLLR            = RCC_PLLR_DIV2;  /* 340/2 = 170MHz */
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        /* 클럭 설정 실패 - 무한 루프 */
        while (1);
    }

    /** 3. CPU, AHB, APB 버스 클럭 설정 */
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK   | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1  | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;     /* HCLK = 170MHz */
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;       /* PCLK1 = 42.5MHz */
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;       /* PCLK2 = 85MHz */

    /**
     * Flash 대기 상태 설정:
     * 170MHz >= 150MHz 이므로 WS = 4 (2.7V~3.6V, Scale 1 기준)
     */
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
        while (1);
    }

    /** 4. FDCAN 클럭 소스 설정: HSE (8MHz 크리스탈)
     *  @note  Nucleo 보드에 HSE 8MHz 크리스탈이 장착되어 있음
     *         FDCAN_KERCK = HSE = 8MHz
     *         Classic CAN 500kbps: 8MHz / (1*(1+13+2)) = 500kbps
     */
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInit.FdcanClockSelection   = RCC_FDCANCLKSOURCE_HSE;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        while (1);
    }
}

/**
 * @brief  GPIO 초기화
 * @note   LD4 LED (PA5) 출력 설정, 활성 Low
 */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO 클럭 활성화 */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* LD4 LED (PA5) 설정: 출력, 푸시풀, 저속, 초기 상태 OFF */
    GPIO_InitStruct.Pin   = LED_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);

    /* LED 초기 상태: OFF */
    LED_OFF();
}
