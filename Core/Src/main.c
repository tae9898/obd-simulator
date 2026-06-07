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
#include "rs485.h"

/* === FreeRTOS 헤더 === */
#include "FreeRTOS.h"
#include "task.h"

/* === 핸들러 전역 변수 === */
FDCAN_HandleTypeDef hfdcan1;   /* FDCAN1 핸들러 */
UART_HandleTypeDef  huart2;    /* USART2 핸들러 (디버그) */
UART_HandleTypeDef  huart1;    /* USART1 핸들러 (RS485) */

/* === 시뮬레이션 상태 전역 변수 === */
OBD2_SimState_t g_sim_state;

/* === CAN RX Queue (ISR → Task 전달) === */
QueueHandle_t xCanRxQueue = NULL;

/* === UART Mutex (Debug_Print 스레드 안전성) === */
SemaphoreHandle_t xUartMutex = NULL;

/* === RS485 RX Queue (ISR → Task 전달) === */
QueueHandle_t xRS485RxQueue = NULL;

/* === IWDG 핸들러 === */
IWDG_HandleTypeDef hiwdg;

/* === LED 토글 카운터 === */
static uint32_t s_led_tick_counter = 0;

/* === 함수 프로토타입 === */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void vMainTask(void *pvParameters);
static void vCanRxTask(void *pvParameters);
static void vRS485Task(void *pvParameters);

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

    /* --- RS485 초기화 (USART1 + MAX485 DE/RE) --- */
    if (RS485_Init() != HAL_OK) {
        Debug_Print("[ERROR] RS485 init failed\r\n");
        while (1) {
            LED_ON();
            HAL_Delay(400);
            LED_OFF();
            HAL_Delay(400);
        }
    }

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

    /* --- IWDG 초기화 (Independent Watchdog) ---
     * LSI ~32kHz / Prescaler 32 = 1kHz (1ms/tick)
     * Reload 2000 → 타임아웃 2초
     * vMainTask에서 500ms마다 리프레시 (여유 1.5초)
     * 모든 초기화 완료 후 시작 (초기화 중 타임아웃 방지)
     */
    hiwdg.Instance            = IWDG;
    hiwdg.Init.Prescaler      = IWDG_PRESCALER_32;
    hiwdg.Init.Reload         = 2000U;
    hiwdg.Init.Window         = IWDG_WINDOW_DISABLE;
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
        Debug_Print("[ERROR] IWDG init failed\r\n");
        while (1);
    }
    Debug_Print("[IWDG] Watchdog started - timeout 2000ms, refresh every 500ms\r\n");

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
     * FreeRTOS 객체 생성 + 태스크 시작
     * ======================================== */

    /* --- CAN RX Queue 생성 --- */
    xCanRxQueue = xQueueCreate(CAN_RX_QUEUE_LEN, sizeof(CAN_RxMessage_t));
    if (xCanRxQueue == NULL) {
        Debug_Print("[ERROR] CAN RX Queue create failed\r\n");
        while (1);
    }
    Debug_Print("[RTOS] CAN RX Queue created (depth=%u)\r\n", CAN_RX_QUEUE_LEN);

    /* --- UART Mutex 생성 (Debug_Print 스레드 안전성) --- */
    xUartMutex = xSemaphoreCreateMutex();
    if (xUartMutex == NULL) {
        Debug_Print("[ERROR] UART Mutex create failed\r\n");
        while (1);
    }
    Debug_Print("[RTOS] UART Mutex created\r\n");

    /* --- RS485 RX Queue 생성 --- */
    xRS485RxQueue = xQueueCreate(RS485_RX_QUEUE_LEN, sizeof(RS485_RxMessage_t));
    if (xRS485RxQueue == NULL) {
        Debug_Print("[ERROR] RS485 RX Queue create failed\r\n");
        while (1);
    }
    Debug_Print("[RTOS] RS485 RX Queue created (depth=%u)\r\n", RS485_RX_QUEUE_LEN);

    /*
     * vMainTask: 기존 메인 루프를 태스크로 이동
     * - 스택: 512 words = 2048 bytes
     * - 우선순위: 2 (기본 작업)
     */
    xTaskCreate(vMainTask, "Main", 512, NULL, 2, NULL);

    /*
     * vCanRxTask: CAN 수신 메시지를 Queue에서 꺼내서 ISO-TP/UDS 처리
     * - 스택: 768 words = 3072 bytes (ISO-TP 버퍼 64B + UDS 응답 64B + printf 256B)
     * - 우선순위: 3 (Main보다 높음 → CAN 메시지 처리 우선)
     */
    xTaskCreate(vCanRxTask, "CANRx", 768, NULL, 3, NULL);

    /*
     * vRS485Task: RS485 수신 메시지를 Queue에서 꺼내서 처리
     * - 스택: 384 words = 1536 bytes
     * - 우선순위: 2 (Main과 동일, CAN-Rx보다 낮음)
     */
    xTaskCreate(vRS485Task, "RS485", 384, NULL, 2, NULL);

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

            /* IWDG 리프레시 (500ms마다, 타임아웃 2000ms 대비 여유 1.5초) */
            HAL_IWDG_Refresh(&hiwdg);
        }

        /* --- 10ms 대기 (다른 태스크에 CPU 양보) --- */
        vTaskDelay(pdMS_TO_TICKS(SIM_UPDATE_PERIOD_MS));
    }
}

/**
 * @brief  CAN 수신 태스크
 *
 * Queue에서 CAN 메시지를 꺼내서 ISO-TP → UDS 처리를 수행.
 * 기존에 ISR 안에서 하던 작업을 이 태스크로 이동.
 *
 * xQueueReceive: Queue에 데이터가 없으면 대기 (CPU 소비 없음)
 *                ISR가 xQueueSendFromISR로 데이터를 넣으면 즉시 깨어남
 */
static void vCanRxTask(void *pvParameters)
{
    (void)pvParameters;
    CAN_RxMessage_t rx_msg;

    Debug_Print("[RTOS] CAN-Rx task started\r\n");

    while (1)
    {
        /* Queue에서 메시지 대기 (무한 대기, CPU 양보) */
        if (xQueueReceive(xCanRxQueue, &rx_msg, portMAX_DELAY) == pdTRUE) {
            /* ISO-TP → UDS 처리 */
            ISO_TP_ProcessFrame(rx_msg.can_id, rx_msg.data, rx_msg.dlc);

            /* CAN→RS485 포워딩 (RPi4로 전달) */
            RS485_ForwardCANMessage(rx_msg.can_id, rx_msg.data, rx_msg.dlc);
        }
    }
}

/**
 * @brief  RS485 수신 태스크
 *
 * Queue에서 RS485 메시지를 꺼내서 처리.
 * 향후 CAN↔RS485 메시지 라우팅 로직이 여기에 추가됨.
 */
static void vRS485Task(void *pvParameters)
{
    (void)pvParameters;
    RS485_RxMessage_t rx_msg;

    Debug_Print("[RTOS] RS485 task started\r\n");

    while (1)
    {
        if (xQueueReceive(xRS485RxQueue, &rx_msg, portMAX_DELAY) == pdTRUE) {
            /* 최소 프레임 길이 확인: ID(2) + DLC(1) = 3바이트 */
            if (rx_msg.len >= 3) {
                uint32_t can_id = ((uint32_t)rx_msg.data[0] << 8) | rx_msg.data[1];
                uint8_t  dlc    = rx_msg.data[2];

                if (dlc <= 8 && (3 + dlc) <= rx_msg.len) {
                    /* RS485→CAN 포워딩 */
                    FDCAN_TxHeaderTypeDef tx_header = {0};
                    tx_header.Identifier          = can_id;
                    tx_header.IdType              = FDCAN_STANDARD_ID;
                    tx_header.TxFrameType         = FDCAN_DATA_FRAME;
                    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
                    tx_header.BitRateSwitch       = FDCAN_BRS_OFF;
                    tx_header.FDFormat            = FDCAN_CLASSIC_CAN;
                    tx_header.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
                    tx_header.MessageMarker       = 0U;

                    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, &rx_msg.data[3]) == HAL_OK) {
                        Debug_Print("[ROUTE] RS485→CAN ID:0x%03lX DLC:%u\r\n", can_id, dlc);
                    }
                }
            }
        }
    }
}

/* ====================================================
 * 에러 핸들러 + HAL 콜백
 * ==================================================== */

/**
 * @brief  Safe State 진입 (치명적 에러 시)
 * @param  msg: 에러 메시지
 *
 * @note   안전 상태:
 *         1. FDCAN 정지 (CAN 버스에 잘못된 데이터 송신 방지)
 *         2. RS485 DE/RE LOW (수신 모드로 전환, 버스 충돌 방지)
 *         3. 에러 메시지 출력
 *         4. LED 빠른 깜빡임
 *         5. IWDG 리프레시 안 함 → 2초 후 시스템 리셋
 */
static void Error_Handler_EnterSafeState(const char *msg)
{
    /* CAN 컨트롤러 정지 */
    (void)HAL_FDCAN_Stop(&hfdcan1);

    /* RS485 수신 모드 전환 */
    RS485_DE_LOW();

    /* 에러 로깅 */
    Debug_Print("[SAFE-STATE] %s\r\n", msg);
    Debug_Print("[SAFE-STATE] Waiting for IWDG reset...\r\n");

    /* LED 빠른 깜빡임 + IWDG 리프레시 안 함 → 2초 후 리셋 */
    while (1) {
        LED_ON();
        HAL_Delay(50);
        LED_OFF();
        HAL_Delay(50);
    }
}

/**
 * @brief  FDCAN 에러 상태 콜백 (Warning / Passive)
 * @note   에러 카운터 변화 시 호출. 버스오프는 ErrorCallback에서 처리.
 */
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan,
                                    uint32_t ErrorStatusITs)
{
    uint32_t ecr = hfdcan->Instance->ECR;
    uint32_t tec = (ecr >> 16) & 0xFF;
    uint32_t rec = (ecr >> 8) & 0xFF;

    if ((ErrorStatusITs & FDCAN_IT_ERROR_WARNING) != 0U) {
        Debug_Print("[FDCAN-WARN] Error Warning: TEC=%lu REC=%lu\r\n", tec, rec);
    }

    if ((ErrorStatusITs & FDCAN_IT_ERROR_PASSIVE) != 0U) {
        Debug_Print("[FDCAN-ERR] Error Passive: TEC=%lu REC=%lu\r\n", tec, rec);
    }
}

/**
 * @brief  FDCAN 버스오프 콜백
 * @note   버스오프 = TEC > 255. CAN 통신 불가 상태.
 *         복구 시도: Stop → 1초 대기 → 재초기화 → 재시작
 *         실패 시 IWDG가 시스템 리셋.
 */
void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
    uint32_t ecr = hfdcan->Instance->ECR;
    uint32_t psr = hfdcan->Instance->PSR;

    Debug_Print("[FDCAN-FATAL] Bus-Off! ECR=0x%08lX PSR=0x%08lX\r\n", ecr, psr);
    Debug_Print("[FDCAN-FATAL] Attempting recovery...\r\n");

    /* 1. CAN 정지 */
    (void)HAL_FDCAN_Stop(hfdcan);

    /* 2. 페리페럴 리셋 (잔류 에러 카운터 초기화) */
    __HAL_RCC_FDCAN_FORCE_RESET();
    __HAL_RCC_FDCAN_RELEASE_RESET();

    /* 3. 재초기화 */
    if (FDCAN1_InitFD(hfdcan) != HAL_OK) {
        Error_Handler_EnterSafeState("FDCAN re-init failed after bus-off");
        return;
    }

    if (FDCAN1_ConfigureFilters(hfdcan) != HAL_OK) {
        Error_Handler_EnterSafeState("FDCAN filter re-config failed after bus-off");
        return;
    }

    if (FDCAN1_StartNotification(hfdcan) != HAL_OK) {
        Error_Handler_EnterSafeState("FDCAN notification re-activate failed after bus-off");
        return;
    }

    if (HAL_FDCAN_Start(hfdcan) != HAL_OK) {
        Error_Handler_EnterSafeState("FDCAN restart failed after bus-off");
        return;
    }

    Debug_Print("[FDCAN-FATAL] Bus-off recovery successful\r\n");
}

/**
 * @brief  UART 에러 콜백 (프레이밍/오버런/노이즈)
 * @note   USART1 (RS485): 에러 플래그 클리어 + RX 재시작
 *         USART2 (Debug): 에러 로깅만 (ST-LINK는 안정적)
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    uint32_t error = huart->ErrorCode;

    if (huart->Instance == USART1) {
        /* RS485 UART 에러 */
        if ((error & HAL_UART_ERROR_FE) != 0U) {
            Debug_Print("[RS485-ERR] Framing error (check baudrate/wiring)\r\n");
        }
        if ((error & HAL_UART_ERROR_ORE) != 0U) {
            Debug_Print("[RS485-ERR] Overrun error (CPU too slow?)\r\n");
        }
        if ((error & HAL_UART_ERROR_NE) != 0U) {
            Debug_Print("[RS485-ERR] Noise error\r\n");
        }

        /* 오버런 플래그 클리어 (필수, 안 하면 RX 멈춤) */
        __HAL_UART_CLEAR_OREFLAG(huart);

        /* 1바이트 수신 재시작 */
        RS485_RestartReceive();
    }
    else if (huart->Instance == USART2) {
        /* Debug UART: 로깅만 */
        Debug_Print("[UART2-ERR] Error code: 0x%08lX\r\n", error);
        __HAL_UART_CLEAR_OREFLAG(huart);
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
