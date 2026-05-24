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
#include "iso_tp.h"
#include "uds_service.h"
#include "diag_session.h"

/* === FreeRTOS 헤더 === */
#include "FreeRTOS.h"
#include "task.h"

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
static void vMainTask(void *pvParameters);

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

    Debug_Print("[INIT] OBD-II / UDS ECU Simulator v2.0\r\n");
    Debug_Print("[INIT] STM32G431RB Nucleo Board\r\n");
    Debug_Print("[INIT] SYSCLK = %lu MHz\r\n", SYSCLK_FREQ / 1000000U);

    /* --- UDS / 세션 / ISO-TP 초기화 --- */
    DiagSession_Init();
    UDS_Init();
    ISO_TP_Init();

    /* --- FDCAN1 초기화 (CAN-FD: 500kbps arb / 2Mbps data) --- */
    if (FDCAN1_InitFD(&hfdcan1) != HAL_OK) {
        Debug_Print("[ERROR] FDCAN1_InitFD failed\r\n");
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

    Debug_Print("[INIT] FDCAN1 ready - CAN-FD 500kbps/2Mbps\r\n");
    Debug_Print("[INIT] Listening on CAN ID 0x%03X\r\n", OBD2_REQUEST_ID);
    Debug_Print("[INIT] UDS Services: 0x10, 0x11, 0x22, 0x27, 0x31\r\n");
    Debug_Print("[INIT] OBD-II PIDs: 0x00, 0x05, 0x0C, 0x0D\r\n");

    /* --- FDCAN1 시작 --- */
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        Debug_Print("[ERROR] FDCAN1 start failed\r\n");
        while (1) {
            LED_ON();
            HAL_Delay(100);
            LED_OFF();
            HAL_Delay(100);
        }
    }

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
     * FreeRTOS 태스크 생성 + 스케줄러 시작
     * ======================================== */

    /*
     * vMainTask: 기존 메인 루프를 태스크로 이동
     * - 스택: 512 words = 2048 bytes (ISO-TP 버퍼 + UDS 응답 + printf 고려)
     * - 우선순위: 2 (기본 작업)
     */
    xTaskCreate(vMainTask, "Main", 512, NULL, 2, NULL);

    Debug_Print("[RTOS] Starting FreeRTOS scheduler\r\n");
    vTaskStartScheduler();

    /* 스케줄러 시작 실패 시 도달 (heap 부족 등) */
    Debug_Print("[ERROR] Scheduler start failed (heap too small?)\r\n");
    while (1);
}

/**
 * @brief  메인 태스크 - 기존 while(1) 루프와 동일한 동작
 *
 * @note   vTaskDelay 사용: HAL_Delay와 달리
 *         다른 태스크에게 CPU를 양보함 (논블로킹 대기)
 *         pdMS_TO_TICKS(10) = 10ms를 틱 단위로 변환
 */
static void vMainTask(void *pvParameters)
{
    (void)pvParameters;

    Debug_Print("[RTOS] Main task started\r\n");

    while (1)
    {
        /* --- 시뮬레이션 값 업데이트 (10ms 주기) --- */
        OBD2_UpdateSimValues(&g_sim_state);

        /* --- ISO-TP 타임아웃 처리 --- */
        ISO_TP_Tick(xTaskGetTickCount() * portTICK_PERIOD_MS);

        /* --- 세션 S3 타임아웃 처리 --- */
        DiagSession_Tick(xTaskGetTickCount() * portTICK_PERIOD_MS);

        /* --- UDS ECU Reset 처리 --- */
        if (g_soft_reset_requested) {
            g_soft_reset_requested = 0U;
            Debug_Print("[UDS] Soft reset -> NVIC_SystemReset\r\n");
            vTaskDelay(pdMS_TO_TICKS(50));
            NVIC_SystemReset();
        }

        /* --- LED 토글 (500ms 주기) --- */
        s_led_tick_counter++;
        if (s_led_tick_counter >= (LED_TOGGLE_PERIOD_MS / SIM_UPDATE_PERIOD_MS)) {
            s_led_tick_counter = 0;
            LED_TOGGLE();
        }

        /* --- 10ms 대기 (다른 태스크에 CPU 양보) --- */
        vTaskDelay(pdMS_TO_TICKS(SIM_UPDATE_PERIOD_MS));
    }
}

/* ====================================================
 * FreeRTOS 훅 함수 (FreeRTOSConfig.h에서 활성화)
 * ==================================================== */

/**
 * @brief  스택 오버플로우 감지 시 호출
 * @note   configCHECK_FOR_STACK_OVERFLOW = 2 로 활성화됨
 *         태스크 스택이 부족하면 여기서 걸림
 *         → 해당 태스크의 스택 크기를 늘려야 함
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    Debug_Print("[FATAL] Stack overflow in: %s\r\n", pcTaskName);
    taskDISABLE_INTERRUPTS();
    while (1);
}

/**
 * @brief  pvPortMalloc 실패 시 호출
 * @note   configTOTAL_HEAP_SIZE가 부족하면 발생
 *         → configTOTAL_HEAP_SIZE 증가 또는 태스크/큐 수 감소
 */
void vApplicationMallocFailedHook(void)
{
    Debug_Print("[FATAL] Malloc failed (heap exhausted)\r\n");
    taskDISABLE_INTERRUPTS();
    while (1);
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
