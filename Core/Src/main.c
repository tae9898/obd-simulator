/**
 * @file    main.c
 * @brief   OBD-II ECU simulator main loop
 * @note    HAL initialization, FDCAN/UART configuration, periodic simulation value updates
 *          For STM32G431RB Nucleo board
 */

/* === Standard libraries === */
#include <stdio.h>
#include <string.h>

/* === Project headers === */
#include "main.h"
#include "obd2_simulator.h"
#include "fdcan_config.h"
#include "uart_debug.h"
#include "iso_tp.h"
#include "uds_service.h"
#include "diag_session.h"
#include "rs485.h"

/* === FreeRTOS headers === */
#include "FreeRTOS.h"
#include "task.h"

/* === FDCAN loopback diagnostic mode (1=test run, 0=normal app) ===
 * Internal loopback test to verify MCU FDCAN peripheral is functioning.
 * Set to 1 to run only the loopback test instead of normal OBD/UDS app (no return).
 * !! Loopback test result: MCU FDCAN OK (PLLQ clock issue was the cause).
 *    Current app runs on HSE 24MHz + FD_BRS(500k/2M) (normal app = 0).
 *
 * [BRS implementation complete] Sections 7-1~7-4 verified: VCP/Debug_Print fixed (blocking TX),
 *    CLASSIC loopback PASS, FD_BRS core loopback PASS, physical 2Mbps BRS bidirectional
 *    communication OK (UDS 0x10->0x50 positive response). 1=loopback diagnostic mode, 0=normal app.
 */
#define RUN_FDCAN_LOOPBACK_TEST 0

/* === Per-frame verbose debug (see DEBUG_VERBOSE in uart_debug.h) ===
 * UART debug output on every CAN/ISO-TP frame (HAL_UART_Transmit blocking, ~4ms per line
 * at 115200baud) is the main bottleneck for response latency. Set to 0 for production/latency
 * measurement. Initialization/error/timeout logs are always output regardless of DEBUG_VERBOSE.
 */

#if RUN_FDCAN_LOOPBACK_TEST
#include "fdcan_loopback_test.h"
#endif

/* === Handler global variables === */
FDCAN_HandleTypeDef hfdcan1;   /* FDCAN1 handle */
UART_HandleTypeDef  huart2;    /* USART2 handle (debug) */
UART_HandleTypeDef  huart1;    /* USART1 handle (RS485) */

/* === Simulation state global variables === */
OBD2_SimState_t g_sim_state;

/* === CAN RX Queue (ISR -> Task delivery) === */
QueueHandle_t xCanRxQueue = NULL;

/* === UART Mutex (Debug_Print thread safety) === */
SemaphoreHandle_t xUartMutex = NULL;

/* === RS485 RX Queue (ISR -> Task delivery) === */
QueueHandle_t xRS485RxQueue = NULL;

/* === IWDG handle === */
IWDG_HandleTypeDef hiwdg;

/* === Task alive flags (for IWDG supervision) === */
volatile uint8_t g_task_alive_flags = 0U;

/* === FDCAN error events (ISR -> Task) === */
volatile uint8_t g_fdcan_busoff_detected = 0U;
volatile uint8_t g_fdcan_error_flags = 0U;      /* bit0=warning, bit1=passive */
volatile uint16_t g_fdcan_last_tec = 0U;
volatile uint16_t g_fdcan_last_rec = 0U;

/* === LED toggle counter === */
static uint32_t s_led_tick_counter = 0;

/* === Function prototypes === */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void vMainTask(void *pvParameters);
static void vCanRxTask(void *pvParameters);
static void vRS485Task(void *pvParameters);
static void Error_Handler_EnterSafeState(const char *msg);
static void ota_stream_sink(ISO_TP_StreamEvent_t event,
                            const uint8_t *data, uint32_t len, uint32_t total_size);

/**
 * @brief  OTA stream sink (placeholder)
 * @note   ISO-TP delivers CF chunks sequentially when receiving messages >4095 bytes.
 *         For real OTA, replace this body with sequential flash write (erase/program)
 *         per chunk. HAL Flash driver (stm32g4xx_hal_flash) is already included in
 *         the build. Currently only logs BEGIN/END/ERROR (per-CF logging omitted to
 *         prevent UART flooding).
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
            /* Per-CF logging omitted (prevent flooding) */
            break;
    }
}

/**
 * @brief  Main entry point
 * @retval int (0 = normal)
 */
int main(void)
{
    /* --- HAL library initialization --- */
    HAL_Init();

    /* --- System clock configuration: HSI 16MHz -> PLL -> 170MHz --- */
    SystemClock_Config();

    /* --- GPIO initialization (LED, etc.) --- */
    MX_GPIO_Init();

    /* --- USART2 debug port initialization --- */
    if (UART_DebugInit(&huart2) != HAL_OK) {
        /* UART initialization failed - indicate error via LED */
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
    /* --- FDCAN internal loopback diagnostic mode: skip normal app init, run test only --- */
    FDCAN_LoopbackTest_Run();   /* Does not return (infinite loop) */
#endif

    /* --- UDS / session / ISO-TP initialization --- */
    DiagSession_Init();
    UDS_Init();
    ISO_TP_Init();

    /* Register OTA stream sink (delivers CF chunks to sink when receiving >4095 bytes) */
    ISO_TP_RegisterStreamSink(ota_stream_sink);

    /* --- FDCAN1 initialization (CAN-FD: nominal 500kbps / data 2Mbps, BRS) --- */
    if (FDCAN1_InitFD(&hfdcan1) != HAL_OK) {
        Debug_Print("[ERROR] FDCAN1_InitFD failed\r\n");
        /* FDCAN initialization failed - LED fast blink */
        while (1) {
            LED_ON();
            HAL_Delay(50);
            LED_OFF();
            HAL_Delay(50);
        }
    }

    /* --- FDCAN1 filter configuration (global: all std frames 0x000-0x7FF -> RX FIFO0; OBD-II request 0x7E0) --- */
    if (FDCAN1_ConfigureFilters(&hfdcan1) != HAL_OK) {
        Debug_Print("[ERROR] FDCAN1 filter config failed\r\n");
        while (1) {
            LED_ON();
            HAL_Delay(200);
            LED_OFF();
            HAL_Delay(200);
        }
    }

    /* === Global filter: accept all standard frames into RX FIFO0 === */
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
        FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_REJECT,
        FDCAN_FILTER_REMOTE, FDCAN_REJECT_REMOTE);
    Debug_Print("[FILTER] Global: all std frames -> RX FIFO0\r\n");

    /* --- FDCAN1 RX interrupt enable --- */
    if (FDCAN1_StartNotification(&hfdcan1) != HAL_OK) {
        Debug_Print("[ERROR] FDCAN1 notification failed\r\n");
        while (1) {
            LED_ON();
            HAL_Delay(300);
            LED_OFF();
            HAL_Delay(300);
        }
    }

    Debug_Print("[INIT] FDCAN1 ready - FD BRS 500k/2M @ HSE 24MHz\r\n");

    Debug_Print("[INIT] Accepting all std frames 0x000-0x7FF -> RX FIFO0 (OBD-II req 0x%03X, resp 0x7E8)\r\n", OBD2_REQUEST_ID);
    Debug_Print("[INIT] UDS Services: 0x10, 0x11, 0x22, 0x27, 0x31\r\n");
    Debug_Print("[INIT] OBD-II PIDs: 0x00, 0x05, 0x0C, 0x0D\r\n");

    /* --- RS485 initialization (USART1 + MAX485 DE/RE) --- */
    if (RS485_Init() != HAL_OK) {
        Debug_Print("[ERROR] RS485 init failed\r\n");
        while (1) {
            LED_ON();
            HAL_Delay(400);
            LED_OFF();
            HAL_Delay(400);
        }
    }

    /* --- FDCAN1 start --- */
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        Debug_Print("[ERROR] FDCAN1 start failed\r\n");
        while (1) {
            LED_ON();
            HAL_Delay(100);
            LED_OFF();
            HAL_Delay(100);
        }
    }

    /* --- Diagnostic communication ready: SecurityAccess boot-delay baseline (M1 fix) --- */
    DiagSession_MarkBootReady();

    /* --- IWDG initialization (independent watchdog, multitask supervision) ---
     * Refresh only when all tasks (MAIN/CAN_RX/RS485) have set their alive flags.
     * If any task stalls, reset within ~2s. LSI (internal 32kHz) must be enabled
     * before HAL_IWDG_Init. */
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

    /* --- Simulation state initial values --- */
    g_sim_state.engine_rpm      = RPM_IDLE;
    g_sim_state.coolant_temp    = COOLANT_TEMP_MIN;
    g_sim_state.vehicle_speed   = 0U;
    g_sim_state.rpm_direction   = 0U;  /* Start ramping up */
    g_sim_state.temp_direction  = 0U;  /* Start increasing */
    g_sim_state.speed_direction = 0U;  /* Start increasing */

    /* --- Normal operation indicator: LED slow blink --- */
    LED_ON();

    /* ========================================
     * FreeRTOS object creation + task start
     * ======================================== */

    /* --- CAN RX Queue creation --- */
    xCanRxQueue = xQueueCreate(CAN_RX_QUEUE_LEN, sizeof(CAN_RxMessage_t));
    if (xCanRxQueue == NULL) {
        Debug_Print("[ERROR] CAN RX Queue create failed\r\n");
        while (1);
    }
    Debug_Print("[RTOS] CAN RX Queue created (depth=%u)\r\n", CAN_RX_QUEUE_LEN);

    /* --- UART Mutex creation (Debug_Print thread safety) --- */
    xUartMutex = xSemaphoreCreateMutex();
    if (xUartMutex == NULL) {
        Debug_Print("[ERROR] UART Mutex create failed\r\n");
        while (1);
    }
    Debug_Print("[RTOS] UART Mutex created\r\n");

    /* --- RS485 RX Queue creation --- */
    xRS485RxQueue = xQueueCreate(RS485_RX_QUEUE_LEN, sizeof(RS485_RxMessage_t));
    if (xRS485RxQueue == NULL) {
        Debug_Print("[ERROR] RS485 RX Queue create failed\r\n");
        while (1);
    }
    Debug_Print("[RTOS] RS485 RX Queue created (depth=%u)\r\n", RS485_RX_QUEUE_LEN);

    /*
     * vMainTask: moved existing main loop to a task
     * - Stack: 512 words = 2048 bytes
     * - Priority: 2 (default work)
     */
    xTaskCreate(vMainTask, "Main", 512, NULL, 2, NULL);

    /*
     * vCanRxTask: dequeue CAN messages and process ISO-TP/UDS
     * - Stack: 768 words = 3072 bytes (ISO-TP buffer 64B + UDS response 64B + printf 256B)
     * - Priority: 3 (higher than Main -> CAN message processing takes priority)
     */
    xTaskCreate(vCanRxTask, "CANRx", 768, NULL, 3, NULL);

    /*
     * vRS485Task: dequeue RS485 messages and process
     * - Stack: 384 words = 1536 bytes
     * - Priority: 2 (same as Main, lower than CAN-Rx)
     */
    xTaskCreate(vRS485Task, "RS485", 384, NULL, 2, NULL);

    Debug_Print("[RTOS] Starting FreeRTOS scheduler\r\n");

    vTaskStartScheduler();

    /* Reached if scheduler start failed (insufficient heap, etc.) */
    Debug_Print("[ERROR] Scheduler start failed (heap too small?)\r\n");
    while (1);
}

/**
 * @brief  Main task - same behavior as the original while(1) loop
 *
 * @note   Uses vTaskDelay: unlike HAL_Delay, yields CPU to other tasks
 *         (non-blocking wait). pdMS_TO_TICKS(10) converts 10ms to tick units.
 */
static void vMainTask(void *pvParameters)
{
    (void)pvParameters;

    Debug_Print("[RTOS] Main task started\r\n");

    /* --- Clock source + FDCAN register dump --- */
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

        /* Verify actual FDCAN clock source (CCIPR[25:24] FDCANSEL: 0=HSE 1=PLLQ 2=PCLK1) */
        uint32_t ccipr = RCC->CCIPR;
        uint32_t fdsel = (ccipr >> 24) & 0x3;
        const char *fdsrc = (fdsel == 0U) ? "HSE" : (fdsel == 1U) ? "PLLQ"
                            : (fdsel == 2U) ? "PCLK1" : "reserved";
        Debug_Print("[CLOCK] CCIPR=0x%08lX FDCANSEL=%s (00=HSE=current)\r\n",
                    ccipr, fdsrc);

        /* Verify PB8(FDCAN1_RX) actual GPIO configuration */
        uint32_t moder = (GPIOB->MODER >> 16) & 0x3;   /* PB8: 0=in 1=out 2=AF 3=analog */
        uint32_t afrh  = (GPIOB->AFR[1] >> 0) & 0xF;   /* PB8 AF: 9=FDCAN1 */
        uint32_t pupdr = (GPIOB->PUPDR >> 16) & 0x3;    /* PB8: 0=none 1=PU 2=PD */
        const char *m[] = {"INPUT","OUTPUT","AF","ANALOG"};
        Debug_Print("[PB8] MODER=%s AFRH=%lu PUPDR=%lu IDR=%lu (AF,AFR=9 is correct)\r\n",
                    moder < 4 ? m[moder] : "?", afrh, pupdr,
                    (GPIOB->IDR >> 8) & 0x1);
    }

    /* Receive only, no TX test */
    Debug_Print("[LISTEN] Waiting for CAN frames...\r\n");

    while (1)
    {
        /* --- Update simulation values (10ms period) --- */
        OBD2_UpdateSimValues(&g_sim_state);

        /* --- DTC state machine update (fault detection based on sim values) --- */
        OBD2_DtcUpdate(&g_sim_state);

        /* --- ISO-TP timeout processing --- */
        ISO_TP_Tick(xTaskGetTickCount() * portTICK_PERIOD_MS);

        /* --- Session S3 timeout processing ---
         * last_activity_tick is based on HAL_GetTick() (updated in SetSession/VerifyKey).
         * now must also use HAL_GetTick() so S3 operates correctly (previous
         * xTaskGetTickCount*period basis mismatch caused false timeout right after
         * 0x10, forcing session back to DEFAULT). */
        DiagSession_Tick(HAL_GetTick());

        /* --- UDS ECU Reset processing --- */
        if (g_soft_reset_requested) {
            g_soft_reset_requested = 0U;
            Debug_Print("[UDS] Soft reset -> NVIC_SystemReset\r\n");
            vTaskDelay(pdMS_TO_TICKS(50));
            NVIC_SystemReset();
        }

        /* --- LED toggle (500ms period) --- */
        s_led_tick_counter++;
        if (s_led_tick_counter >= (LED_TOGGLE_PERIOD_MS / SIM_UPDATE_PERIOD_MS)) {
            s_led_tick_counter = 0;
            LED_TOGGLE();

            /* IWDG refresh: only after confirming all tasks are alive */
            g_task_alive_flags |= TASK_ALIVE_MAIN;
            if ((g_task_alive_flags & TASK_ALIVE_ALL) == TASK_ALIVE_ALL) {
                HAL_IWDG_Refresh(&hiwdg);
                g_task_alive_flags = 0U;
            }
            /* If any task is unconfirmed, do not refresh -> reset after 2s */
        }

        /* --- FDCAN error-passive recovery: REC>127 sustained for 3s -> Stop/Start to reset counters ---
         * Workaround for post-boot phantom where REC=255 locks up transmit. */
        {
            static uint32_t rec_check_tick = 0;
            rec_check_tick++;
            if ((rec_check_tick % 100U) == 0U) {  /* Every 1s (100 * 10ms) */
                static uint32_t ep_ticks = 0;
                uint32_t ecr = hfdcan1.Instance->ECR;
                uint32_t rec = (ecr >> 8) & 0xFFU;
                if (rec > 127U) {
                    ep_ticks++;
                    if (ep_ticks >= 3U) {  /* Sustained for 3s */
                        Debug_Print("[FDCAN-RECOV] REC=%lu sustained 3s -> Stop/Start reset\r\n", (unsigned long)rec);
                        HAL_FDCAN_Stop(&hfdcan1);
                        HAL_FDCAN_Start(&hfdcan1);
                        ep_ticks = 0;
                    }
                } else {
                    ep_ticks = 0;
                }
            }
        }

        /* --- FDCAN error logging (ISR -> flags -> output here) --- */
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

        /* --- FDCAN bus-off recovery (ISR only sets flag, processed here) --- */
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

        /* --- Wait 10ms (yield CPU to other tasks) --- */
        vTaskDelay(pdMS_TO_TICKS(SIM_UPDATE_PERIOD_MS));
    }
}

/**
 * @brief  CAN receive task
 *
 * Dequeues CAN messages and performs ISO-TP -> UDS processing.
 * Moved from ISR to this task.
 *
 * xQueueReceive: blocks when queue is empty (no CPU consumption)
 *                Wakes immediately when ISR puts data via xQueueSendFromISR
 */
static void vCanRxTask(void *pvParameters)
{
    (void)pvParameters;
    CAN_RxMessage_t rx_msg;

    Debug_Print("[RTOS] CAN-Rx task started\r\n");

    while (1)
    {
        /* Task alive notification (always set) */
        g_task_alive_flags |= TASK_ALIVE_CAN_RX;

        /* Wait for message from queue (100ms timeout, for IWDG refresh) */
        if (xQueueReceive(xCanRxQueue, &rx_msg, pdMS_TO_TICKS(100)) == pdTRUE) {
            /* Output received CAN frame immediately (up to 64 bytes, debug) */
#if DEBUG_VERBOSE
            Debug_LogCAN_Rx(rx_msg.can_id, rx_msg.data, rx_msg.dlc);
#endif

            /* ISO-TP -> UDS processing (transmit response) */
            ISO_TP_ProcessFrame(rx_msg.can_id, rx_msg.data, rx_msg.dlc);

            /* CAN->RS485 forwarding (deliver to RPi4) */
            RS485_ForwardCANMessage(rx_msg.can_id, rx_msg.data, rx_msg.dlc);
        }
    }
}

/**
 * @brief  RS485 receive task
 *
 * Dequeues RS485 messages and processes them.
 * Future CAN<->RS485 message routing logic will be added here.
 */
static void vRS485Task(void *pvParameters)
{
    (void)pvParameters;
    RS485_RxMessage_t rx_msg;

    Debug_Print("[RTOS] RS485 task started\r\n");

    while (1)
    {
        /* Task alive notification (always set) */
        g_task_alive_flags |= TASK_ALIVE_RS485;

        if (xQueueReceive(xRS485RxQueue, &rx_msg, pdMS_TO_TICKS(100)) == pdTRUE) {

            /* Minimum frame length check: ID(2) + DLC(1) = 3 bytes */
            if (rx_msg.len >= 3U) {
                uint32_t can_id = ((uint32_t)rx_msg.data[0] << 8) | (uint32_t)rx_msg.data[1];
                uint8_t  dlc    = rx_msg.data[2];

                if (dlc <= 64U && (3U + dlc) <= rx_msg.len) {
                    /* RS485->CAN forwarding (Classic CAN - CANable compatible) */
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

                    /* HAL CAN-FD transmit reads the rounded-up DLC worth of bytes, so
                     * copy to 64-byte buffer + pad (0xCC) before transmit. Prevents stale
                     * buffer residual bytes from leaking onto the bus. */
                    uint8_t tx_data[64];
                    (void)memset(tx_data, 0xCCU, sizeof(tx_data));
                    for (uint8_t i = 0U; i < dlc; i++) {
                        tx_data[i] = rx_msg.data[3U + i];
                    }

                    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, tx_data) == HAL_OK) {
                        Debug_Print("[ROUTE] RS485->CAN ID:0x%03lX DLC:%u (FD)\r\n", can_id, dlc);
                    }
                }
            }
        }
    }
}

/* ====================================================
 * Error handlers + HAL callbacks
 * ==================================================== */

/**
 * @brief  Enter Safe State (on fatal error)
 * @param  msg: error message
 *
 * @note   Safe state procedure:
 *         1. Stop FDCAN (prevent transmitting invalid data on CAN bus)
 *         2. RS485 DE/RE LOW (switch to receive mode, prevent bus collision)
 *         3. Output error message
 *         4. LED fast blink
 *         5. No IWDG refresh -> system reset after 2s
 */
static void Error_Handler_EnterSafeState(const char *msg)
{
    /* Stop CAN controller */
    (void)HAL_FDCAN_Stop(&hfdcan1);

    /* Switch RS485 to receive mode */
    RS485_DE_LOW();

    /* Error logging */
    Debug_Print("[SAFE-STATE] %s\r\n", msg);
    Debug_Print("[SAFE-STATE] Waiting for IWDG reset...\r\n");

    /* LED fast blink + no IWDG refresh -> reset after 2s */
    while (1) {
        LED_ON();
        HAL_Delay(50);
        LED_OFF();
        HAL_Delay(50);
    }
}

/**
 * @brief  FDCAN error status callback (Warning / Passive)
 * @note   Called from ISR context -> only set flags (no Debug_Print!)
 *         Actual logging is handled in vMainTask
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
 * @brief  FDCAN bus-off callback
 * @note   Called from ISR context -> only set flag.
 *         Actual recovery is handled in vMainTask (heavy work belongs in a task).
 */
void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
    (void)hfdcan;
    g_fdcan_busoff_detected = 1U;
}

/**
 * @brief  UART error callback (framing/overrun/noise)
 * @note   USART1 (RS485): clear error flags + restart RX
 *         USART2 (Debug): logging only (ST-LINK is stable)
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    uint32_t error = huart->ErrorCode;

    if (huart->Instance == USART1) {
        /* RS485 UART error */
        if ((error & HAL_UART_ERROR_FE) != 0U) {
            Debug_Print("[RS485-ERR] Framing error (check baudrate/wiring)\r\n");
        }
        if ((error & HAL_UART_ERROR_ORE) != 0U) {
            Debug_Print("[RS485-ERR] Overrun error (CPU too slow?)\r\n");
        }
        if ((error & HAL_UART_ERROR_NE) != 0U) {
            Debug_Print("[RS485-ERR] Noise error\r\n");
        }

        /* Clear overrun flag (required, otherwise RX halts) */
        __HAL_UART_CLEAR_OREFLAG(huart);

        /* Restart 1-byte receive */
        RS485_RestartReceive();
    }
    else if (huart->Instance == USART2) {
        /* Debug UART: logging only */
        Debug_Print("[UART2-ERR] Error code: 0x%08lX\r\n", error);
        __HAL_UART_CLEAR_OREFLAG(huart);
    }
}

/* ====================================================
 * FreeRTOS hook functions (enabled in FreeRTOSConfig.h)
 * ==================================================== */

/**
 * @brief  Called on stack overflow detection
 * @note   Enabled with configCHECK_FOR_STACK_OVERFLOW = 2
 *         Triggers here when task stack is insufficient
 *         -> increase the affected task's stack size
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    Debug_Print("[FATAL] Stack overflow in: %s\r\n", pcTaskName);
    taskDISABLE_INTERRUPTS();
    while (1);
}

/**
 * @brief  Called on pvPortMalloc failure
 * @note   Occurs when configTOTAL_HEAP_SIZE is insufficient
 *         -> increase configTOTAL_HEAP_SIZE or reduce tasks/queues
 */
void vApplicationMallocFailedHook(void)
{
    Debug_Print("[FATAL] Malloc failed (heap exhausted)\r\n");
    taskDISABLE_INTERRUPTS();
    while (1);
}

/**
 * @brief  System clock configuration
 * @note   HSI 16MHz -> PLL -> SYSCLK 170MHz
 *         - PLLM = 4  (HSI/4 = 4MHz)
 *         - PLLN = 85 (4MHz * 85 = 340MHz VCO)
 *         - PLLP = 2  (340MHz / 2 = 170MHz SYSCLK)
 *         - PLLQ = 2  (340MHz / 2 = 170MHz, FDCAN unused - currently using HSE)
 *         - PLLR = 2  (340MHz / 2 = 170MHz, for SYSCLK)
 *         - AHB prescaler = 1  -> HCLK = 170MHz
 *         - APB1 prescaler = 4 -> PCLK1 = 42.5MHz
 *         - APB2 prescaler = 2 -> PCLK2 = 85MHz
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef        RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef        RCC_ClkInitStruct = {0};

    /** 1. Power configuration: Scale 1 mode (required for 170MHz operation) */
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** 2. RCC oscillator configuration: HSI (for SYSCLK/PLL 170MHz) + HSE (for FDCAN 2M BRS).
     *     HSE 24MHz crystal confirmed oscillating (SWD HSERDY verified, handoff section 1 correction). */
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
        /* Clock configuration failed - infinite loop */
        while (1);
    }

    /** 3. CPU, AHB, APB bus clock configuration */
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK   | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1  | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;     /* HCLK = 170MHz */
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;       /* PCLK1 = 42.5MHz */
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;       /* PCLK2 = 85MHz */

    /**
     * Flash wait state configuration:
     * 170MHz >= 150MHz so WS = 4 (2.7V~3.6V, Scale 1 reference)
     */
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
        while (1);
    }

    /** 4. FDCAN clock source configuration: HSE (24MHz) -- for CAN-FD 2Mbps BRS.
     *  @note  24MHz/2Mbps = 12 TQ (integer division, SP 83.3%). PCLK1 42.5MHz does not
     *         divide evenly to 2M (21.25), causing BRS issues. CCIPR[25:24]=00 -> HSE. */
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInit.FdcanClockSelection   = RCC_FDCANCLKSOURCE_HSE;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        while (1);
    }
}

/**
 * @brief  GPIO initialization
 * @note   LD4 LED (PA5) output configuration, active Low
 */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Enable GPIO clock */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* LD4 LED (PA5) configuration: output, push-pull, low speed, initial state OFF */
    GPIO_InitStruct.Pin   = LED_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);

    /* LED initial state: OFF */
    LED_OFF();
}
