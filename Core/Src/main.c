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

/* === FDCAN 루프백 진단 모드 (1=테스트 실행, 0=정상 앱) ===
 * MCU FDCAN 주변기기 자체 정상 여부를 검증하는 내부 루프백 테스트.
 * 1로 설정하면 정상 OBD/UDS 앱 대신 루프백 테스트만 실행한다(복귀 안 함).
 * !! 루프백 테스트 결과: MCU FDCAN 정상(PLLQ 클럭 불량이 원인).
 *    현재 앱은 PCLK1 클럭으로 수정됨 -> 0(정상 앱)으로 사용.
 */
#define RUN_FDCAN_LOOPBACK_TEST 0

/* === per-frame verbose 디버그 (uart_debug.h 의 DEBUG_VERBOSE 참조) ===
 * 매 CAN/ISO-TP 프레임마다 UART 디버그 출력(HAL_UART_Transmit 블로킹, 115200baud
 * 에서 라인당 ~4ms)이 응답 latency의 주된 병목. production/latency 측정 시 0.
 * 초기화·에러·타임아웃 로그는 DEBUG_VERBOSE 와 무관하게 항상 출력.
 */

#if RUN_FDCAN_LOOPBACK_TEST
#include "fdcan_loopback_test.h"
#endif

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

/* === 태스크 생존 플래그 (IWDG 감시용) === */
volatile uint8_t g_task_alive_flags = 0U;

/* === FDCAN 에러 이벤트 (ISR → Task) === */
volatile uint8_t g_fdcan_busoff_detected = 0U;
volatile uint8_t g_fdcan_error_flags = 0U;      /* bit0=warning, bit1=passive */
volatile uint16_t g_fdcan_last_tec = 0U;
volatile uint16_t g_fdcan_last_rec = 0U;

/* === LED 토글 카운터 === */
static uint32_t s_led_tick_counter = 0;

/* === 함수 프로토타입 === */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void vMainTask(void *pvParameters);
static void vCanRxTask(void *pvParameters);
static void vRS485Task(void *pvParameters);
static void Error_Handler_EnterSafeState(const char *msg);
static void ota_stream_sink(ISO_TP_StreamEvent_t event,
                            const uint8_t *data, uint32_t len, uint32_t total_size);

/**
 * @brief  OTA 스트림 싱크 (플레이스홀더)
 * @note   ISO-TP 가 4095바이트 초과 메시지를 수신할 때 CF 청크를 순차 전달.
 *         실제 OTA 에서는 각 청크를 플래시에 순차 기록(erase/program) 하도록
 *         이 본문을 교체. HAL Flash 드라이버(stm32g4xx_hal_flash) 는 빌드에
 *         이미 포함되어 있음. 현재는 BEGIN/END/ERROR 만 로깅 (CF 단위 로깅은
 *         UART 플러딩 방지용 생략).
 */
static void ota_stream_sink(ISO_TP_StreamEvent_t event,
                            const uint8_t *data, uint32_t len, uint32_t total_size)
{
    (void)data;
    (void)len;

    switch (event) {
        case ISO_TP_STREAM_BEGIN:
            Debug_Print("[OTA] stream begin total=%lu bytes\r\n", (unsigned long)total_size);
            break;
        case ISO_TP_STREAM_END:
            Debug_Print("[OTA] stream end (%lu bytes received)\r\n", (unsigned long)total_size);
            break;
        case ISO_TP_STREAM_ERROR:
            Debug_Print("[OTA] stream ERROR (timeout/seq), transfer aborted\r\n");
            break;
        case ISO_TP_STREAM_DATA:
        default:
            /* CF 단위 로깅 생략 (플러딩 방지) */
            break;
    }
}

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

#if RUN_FDCAN_LOOPBACK_TEST
    /* --- FDCAN 내부 루프백 진단 모드: 정상 앱 초기화 생략, 테스트만 실행 --- */
    FDCAN_LoopbackTest_Run();   /* 복귀하지 않음 (무한 루프) */
#endif

    /* --- UDS / 세션 / ISO-TP 초기화 --- */
    DiagSession_Init();
    UDS_Init();
    ISO_TP_Init();

    /* OTA 스트림 싱크 등록 (>4095바이트 수신 시 CF 청크를 싱크로 전달) */
    ISO_TP_RegisterStreamSink(ota_stream_sink);

    /* --- FDCAN1 초기화 (CAN-FD: 노멀 500kbps / 데이터 2Mbps, BRS) --- */
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

    /* --- FDCAN1 필터 설정 (글로벌: 모든 표준 프레임 0x000-0x7FF → RX FIFO0; OBD-II 요청 0x7E0) --- */
    if (FDCAN1_ConfigureFilters(&hfdcan1) != HAL_OK) {
        Debug_Print("[ERROR] FDCAN1 filter config failed\r\n");
        while (1) {
            LED_ON();
            HAL_Delay(200);
            LED_OFF();
            HAL_Delay(200);
        }
    }

    /* === 글로벌 필터: 모든 표준 프레임 RX FIFO0로 수락 === */
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
        FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_REJECT,
        FDCAN_FILTER_REMOTE, FDCAN_REJECT_REMOTE);
    Debug_Print("[FILTER] Global: all std frames -> RX FIFO0\r\n");

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

    Debug_Print("[INIT] FDCAN1 ready - CAN-FD 500kbps/2Mbps (BRS) @ HSE 24MHz\r\n");

    Debug_Print("[INIT] Accepting all std frames 0x000-0x7FF -> RX FIFO0 (OBD-II req 0x%03X, resp 0x7E8)\r\n", OBD2_REQUEST_ID);
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

    /* --- 진단 통신 준비 완료: SecurityAccess boot-delay 기준점 (M1 수정) --- */
    DiagSession_MarkBootReady();

    /* --- IWDG 초기화 (독립 워치독, multitask 감시) ---
     * 모든 태스크(MAIN/CAN_RX/RS485)가 alive 플래그를 세트할 때만 refresh.
     * 하나라도 멈추면 ~2초 내 리셋. HAL_IWDG_Init 전에 LSI(내부 32kHz) 활성화 필수. */
#if 1
    __HAL_RCC_LSI_ENABLE();
    while (__HAL_RCC_GET_FLAG(RCC_FLAG_LSIRDY) == RESET) { }
    hiwdg.Instance            = IWDG;
    hiwdg.Init.Prescaler      = IWDG_PRESCALER_32;
    hiwdg.Init.Reload         = 2000U;
    hiwdg.Init.Window         = IWDG_WINDOW_DISABLE;
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
        Debug_Print("[ERROR] IWDG init failed\r\n");
        while (1);
    }
    Debug_Print("[IWDG] Watchdog started - timeout ~2000ms (refresh on all-tasks-alive)\r\n");
#else
    Debug_Print("[IWDG] DISABLED for debug\r\n");
#endif

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

    /* --- 클럭 소스 + FDCAN 레지스터 덤프 --- */
    {
        uint32_t nbtp = hfdcan1.Instance->NBTP;
        uint32_t nbrp    = ((nbtp >> 16) & 0x1FF) + 1;
        uint32_t ntseg1  = ((nbtp >>  8) & 0xFF)  + 1;
        uint32_t ntseg2  = ((nbtp >>  0) & 0x7F)  + 1;
        uint32_t nsjw    = ((nbtp >> 25) & 0x7F)  + 1;
        uint32_t bitrate = FDCAN_CLK_FREQ / (nbrp * (1 + ntseg1 + ntseg2));
        Debug_Print("[CLOCK] NBTP=0x%08lX NBRP=%lu NTSEG1=%lu NTSEG2=%lu NSJW=%lu\r\n",
                    nbtp, nbrp, ntseg1, ntseg2, nsjw);
        Debug_Print("[CLOCK] Nominal bitrate = %lu bps (expect 500000)\r\n", bitrate);

        /* 실제 FDCAN 클럭 소스 확인 (CCIPR[25:24] FDCANSEL: 0=HSE 1=PLLQ 2=PCLK1) */
        uint32_t ccipr = RCC->CCIPR;
        uint32_t fdsel = (ccipr >> 24) & 0x3;
        const char *fdsrc = (fdsel == 0U) ? "HSE" : (fdsel == 1U) ? "PLLQ"
                            : (fdsel == 2U) ? "PCLK1" : "reserved";
        Debug_Print("[CLOCK] CCIPR=0x%08lX FDCANSEL=%s (10=PCLK1=확정)\r\n",
                    ccipr, fdsrc);

        /* PB8(FDCAN1_RX) 실제 GPIO 설정 확인 */
        uint32_t moder = (GPIOB->MODER >> 16) & 0x3;   /* PB8: 0=in 1=out 2=AF 3=analog */
        uint32_t afrh  = (GPIOB->AFR[1] >> 0) & 0xF;   /* PB8 AF: 9=FDCAN1 */
        uint32_t pupdr = (GPIOB->PUPDR >> 16) & 0x3;    /* PB8: 0=none 1=PU 2=PD */
        const char *m[] = {"INPUT","OUTPUT","AF","ANALOG"};
        Debug_Print("[PB8] MODER=%s AFRH=%lu PUPDR=%lu IDR=%lu (AF,AFR=9 이 정상)\r\n",
                    moder < 4 ? m[moder] : "?", afrh, pupdr,
                    (GPIOB->IDR >> 8) & 0x1);
    }

    /* TX 테스트 없이 수신만 */
    Debug_Print("[LISTEN] Waiting for CAN frames...\r\n");

    while (1)
    {
        /* --- 시뮬레이션 값 업데이트 (10ms 주기) --- */
        OBD2_UpdateSimValues(&g_sim_state);

        /* --- DTC 상태머신 갱신 (시뮬 값 기반 fault 감지) --- */
        OBD2_DtcUpdate(&g_sim_state);

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

            /* IWDG 리프레시: 모든 태스크 생존 확인 후에만 */
            g_task_alive_flags |= TASK_ALIVE_MAIN;
            if ((g_task_alive_flags & TASK_ALIVE_ALL) == TASK_ALIVE_ALL) {
                HAL_IWDG_Refresh(&hiwdg);
                g_task_alive_flags = 0U;
            }
            /* 미확인 태스크가 있으면 리프레시 안 함 → 2초 후 리셋 */
        }

        /* --- FDCAN error-passive 복구: REC>127 이 3초 지속 → Stop/Start 로 카운터 리셋 ---
         * 부팅 직후 짧은 팬텀이 REC=255 로 고정되어 송신이 막히는 현상 회피. */
        {
            static uint32_t rec_check_tick = 0;
            rec_check_tick++;
            if ((rec_check_tick % 100U) == 0U) {  /* 1초마다 (100 * 10ms) */
                static uint32_t ep_ticks = 0;
                uint32_t ecr = hfdcan1.Instance->ECR;
                uint32_t rec = (ecr >> 8) & 0xFFU;
                if (rec > 127U) {
                    ep_ticks++;
                    if (ep_ticks >= 3U) {  /* 3초 지속 */
                        Debug_Print("[FDCAN-RECOV] REC=%lu 3초 지속 → Stop/Start 리셋\r\n", (unsigned long)rec);
                        HAL_FDCAN_Stop(&hfdcan1);
                        HAL_FDCAN_Start(&hfdcan1);
                        ep_ticks = 0;
                    }
                } else {
                    ep_ticks = 0;
                }
            }
        }

        /* --- FDCAN 에러 로깅 (ISR → 플래그 → 여기서 출력) --- */
        if (g_fdcan_error_flags != 0U) {
            uint8_t flags = g_fdcan_error_flags;
            g_fdcan_error_flags = 0U;
            if (flags & 0x01U) {
                Debug_Print("[FDCAN-WARN] Error Warning: TEC=%u REC=%u\r\n",
                            g_fdcan_last_tec, g_fdcan_last_rec);
            }
            if (flags & 0x02U) {
                Debug_Print("[FDCAN-ERR] Error Passive: TEC=%u REC=%u\r\n",
                            g_fdcan_last_tec, g_fdcan_last_rec);
            }
        }

        /* --- FDCAN 버스오프 복구 (ISR에서 플래그만 설정, 여기서 처리) --- */
        if (g_fdcan_busoff_detected != 0U) {
            g_fdcan_busoff_detected = 0U;
            Debug_Print("[FDCAN-FATAL] Bus-off recovery (task context)...\r\n");

            HAL_FDCAN_Stop(&hfdcan1);
            __HAL_RCC_FDCAN_FORCE_RESET();
            __HAL_RCC_FDCAN_RELEASE_RESET();

            if (FDCAN1_InitFD(&hfdcan1) != HAL_OK) {
                Error_Handler_EnterSafeState("FDCAN re-init failed");
            }
            if (FDCAN1_ConfigureFilters(&hfdcan1) != HAL_OK) {
                Error_Handler_EnterSafeState("FDCAN filter re-config failed");
            }
            if (FDCAN1_StartNotification(&hfdcan1) != HAL_OK) {
                Error_Handler_EnterSafeState("FDCAN notification re-activate failed");
            }
            if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
                Error_Handler_EnterSafeState("FDCAN restart failed");
            }

            Debug_Print("[FDCAN-FATAL] Bus-off recovery successful\r\n");
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
        /* 태스크 생존 알림 (항상 설정) */
        g_task_alive_flags |= TASK_ALIVE_CAN_RX;

        /* Queue에서 메시지 대기 (100ms 타임아웃, IWDG 리프레시 위해) */
        if (xQueueReceive(xCanRxQueue, &rx_msg, pdMS_TO_TICKS(100)) == pdTRUE) {
            /* 수신된 CAN 프레임 즉시 출력 (최대 64바이트까지, 디버그) */
#if DEBUG_VERBOSE
            Debug_LogCAN_Rx(rx_msg.can_id, rx_msg.data, rx_msg.dlc);
#endif

            /* ISO-TP → UDS 처리 (응답 송신) */
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
        /* 태스크 생존 알림 (항상 설정) */
        g_task_alive_flags |= TASK_ALIVE_RS485;

        if (xQueueReceive(xRS485RxQueue, &rx_msg, pdMS_TO_TICKS(100)) == pdTRUE) {

            /* 최소 프레임 길이 확인: ID(2) + DLC(1) = 3바이트 */
            if (rx_msg.len >= 3U) {
                uint32_t can_id = ((uint32_t)rx_msg.data[0] << 8) | (uint32_t)rx_msg.data[1];
                uint8_t  dlc    = rx_msg.data[2];

                if (dlc <= 64U && (3U + dlc) <= rx_msg.len) {
                    /* RS485→CAN 포워딩 (Classic CAN — CANable 호환) */
                    FDCAN_TxHeaderTypeDef tx_header = {0};
                    tx_header.Identifier          = can_id;
                    tx_header.IdType              = FDCAN_STANDARD_ID;
                    tx_header.TxFrameType         = FDCAN_DATA_FRAME;
                    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
                    tx_header.BitRateSwitch       = FDCAN_BRS_ON;
                    tx_header.FDFormat            = FDCAN_FD_CAN;
                    tx_header.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
                    tx_header.MessageMarker       = 0U;
                    tx_header.DataLength          = FDCAN_BytesToDlc(dlc);

                    /* CAN-FD 프레임의 HAL 전송은 round-up된 DLC 만큼 읽으므로
                     * 64바이트 버퍼에 복사 + 패딩(0xCC) 후 전송. stale 버퍼
                     * 잔여 바이트가 버스로 새어나가는 것을 방지. */
                    uint8_t tx_data[64];
                    (void)memset(tx_data, 0xCCU, sizeof(tx_data));
                    for (uint8_t i = 0U; i < dlc; i++) {
                        tx_data[i] = rx_msg.data[3U + i];
                    }

                    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, tx_data) == HAL_OK) {
                        Debug_Print("[ROUTE] RS485→CAN ID:0x%03lX DLC:%u (FD)\r\n", can_id, dlc);
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
 * @note   ISR 컨텍스트에서 호출 → 플래그만 설정 (Debug_Print 금지!)
 *         실제 로깅은 vMainTask에서 처리
 */
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan,
                                    uint32_t ErrorStatusITs)
{
    uint32_t ecr = hfdcan->Instance->ECR;
    g_fdcan_last_tec = (uint16_t)((ecr >> 16) & 0xFFU);
    g_fdcan_last_rec = (uint16_t)((ecr >> 8) & 0xFFU);

    if ((ErrorStatusITs & FDCAN_IT_ERROR_WARNING) != 0U) {
        g_fdcan_error_flags |= 0x01U;
    }

    if ((ErrorStatusITs & FDCAN_IT_ERROR_PASSIVE) != 0U) {
        g_fdcan_error_flags |= 0x02U;
    }
}

/**
 * @brief  FDCAN 버스오프 콜백
 * @note   ISR 컨텍스트에서 호출됨 → 플래그만 설정.
 *         실제 복구는 vMainTask에서 처리 (무거운 작업은 태스크에서).
 */
void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
    (void)hfdcan;
    g_fdcan_busoff_detected = 1U;
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

    /** 4. FDCAN 클럭 소스 설정: HSE 24MHz (정밀 크리스탈)
     *  @note  !! Nucleo-G431RB(MB1367) HSE 크리스탈 = 24MHz (UM2505). 과거 8MHz 는 오기.
     *         PLLQ->FDCAN 불량, HSI 기반 PCLK1 은 톨러런스 한계 초과(Form Error).
     *         정밀 HSE 크리스탈(±50ppm) 사용. CCIPR[25:24] = 00 -> HSE.
     *         Classic CAN 500kbps: 24MHz / (3*(1+13+2)) = 500kbps (SP 87.5%)
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
