/**
 * @file    bl_main.c
 * @brief   Bootloader entry: clock init + boot gate (meta CRC / handoff flag)
 * @note    Bare-metal superloop, no FreeRTOS (-DBL_BUILD). App slot at 0x08005000.
 *
 * Boot gate:
 *   noinit flag UPDATE -> consume, stay in BL (programming session server)
 *   meta ABSENT        -> vector sanity -> jump (dev/SWD flash path)
 *   meta VALID + CRC ok -> jump
 *   otherwise          -> stay in BL (brick safety)
 */

#include "main.h"
#include "bl_handoff.h"
#include "uart_debug.h"
#include "fdcan_config.h"
#include "iso_tp.h"
#include "diag_session.h"

/* Handles owned here (app owns them in main.c) */
FDCAN_HandleTypeDef hfdcan1;   /* BL_BUILD: defined here, zero-init = .Instance NULL */
UART_HandleTypeDef  huart2;    /* USART2 debug VCP */

static void SystemClock_Config(void);
static void MX_GPIO_Init(void);

/* bl_boot.c */
uint32_t bl_crc32(const uint8_t *data, uint32_t len);
const bl_meta_t *bl_meta(void);
int bl_meta_absent(void);
int bl_app_crc_ok(void);
uint32_t bl_meta_rollback_counter(void);
int bl_app_vectors_ok(void);
void bl_jump_to_app(void);

/* bl_uds.c */
extern volatile uint8_t bl_reset_pending;

/* diag_session.c links these (Default-session restore); no OBD machinery in BL */
void OBD2_DtcSettingControl(uint8_t on) { (void)on; }
void UDS_PeriodicStopAll(void) { }

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();

    if (UART_DebugInit(&huart2) != HAL_OK) {
        while (1) { /* no debug link */ }
    }
    Debug_Print("\r\n[FBL] Bootloader v0.2 (BL 0x08000000, app 0x08007000, meta 0x0801E000)\r\n");

    /* --- Boot gate --- */
    volatile bl_handoff_t *ho = (volatile bl_handoff_t *)BL_HANDOFF_ADDR;
    int enter_via_flag = 0;

    if (ho->magic == BL_HANDOFF_MAGIC && ho->reason == BL_REASON_UPDATE) {
        ho->magic  = 0U;   /* consume: one-shot */
        ho->reason = 0U;
        enter_via_flag = 1;
        Debug_Print("[FBL] Stay-in-BL flag consumed -> UDS flash server\r\n");
    } else if (bl_meta_absent()) {
        if (!bl_app_vectors_ok()) {
            Debug_Print("[FBL] No valid app image -> waiting for UDS\r\n");
        } else {
            Debug_Print("[FBL] Meta absent, app vectors plausible -> jump (dev path)\r\n");
            bl_jump_to_app();
        }
    } else if (bl_app_crc_ok()) {
        const bl_meta_t *m = bl_meta();
        Debug_Print("[FBL] App OK (v%lu, %luB) -> jump\r\n",
                    (unsigned long)m->version, (unsigned long)m->size);
        bl_jump_to_app();
    } else {
        const bl_meta_t *m = bl_meta();
        Debug_Print("[FBL] App CRC/state invalid (magic=%08lX state=%08lX) -> waiting for UDS\r\n",
                    (unsigned long)m->magic, (unsigned long)m->state);
    }

    /* --- UDS flash server bring-up (same FDCAN/ISO-TP stack as the app) --- */
    DiagSession_Init();
    ISO_TP_Init();

    if (FDCAN1_InitFD(&hfdcan1) != HAL_OK ||
        FDCAN1_ConfigureFilters(&hfdcan1) != HAL_OK ||
        FDCAN1_StartNotification(&hfdcan1) != HAL_OK) {
        Debug_Print("[FBL] FDCAN init failed\r\n");
        while (1) { LED_TOGGLE(); HAL_Delay(50); }
    }
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
        FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_REJECT,
        FDCAN_FILTER_REMOTE, FDCAN_REJECT_REMOTE);
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        while (1) { LED_TOGGLE(); HAL_Delay(50); }
    }
    DiagSession_MarkBootReady();

    if (enter_via_flag) {
        /* Tester asked for programming: start there. S3 (5s without 0x3E)
         * drops to Default -> the auto-boot rule below reboots the app. */
        (void)DiagSession_SetSession(DIAG_SESSION_PROGRAMMING);
    }
    Debug_Print("[FBL] Server ready (0x10 02/27/28/31/34/36/37/3E/85)\r\n");

    uint32_t reset_at = 0U;
    while (1) {
        /* RX: drain the ISR ring -> ISO-TP reassembly (dispatch runs inline) */
        CAN_RxMessage_t frame;
        while (FDCAN_BL_Pop(&frame) != 0) {
            ISO_TP_ProcessFrame(frame.can_id, frame.data, frame.dlc);
        }

        ISO_TP_Tick(HAL_GetTick());
        DiagSession_Tick(HAL_GetTick());

        /* 0x11: reset after the response drained (ISO-TP TX is paced) */
        if (bl_reset_pending != 0U) {
            uint32_t now = HAL_GetTick();
            if (reset_at == 0U) {
                reset_at = now + 200U;
            } else if ((int32_t)(now - reset_at) >= 0) {
                NVIC_SystemReset();
            }
        }

        /* Self-heal: tester gone (S3 -> Default) with a valid app -> boot it.
         * CRC (~20ms @170MHz) only runs while idle in Default session. */
        if (DiagSession_Get() == DIAG_SESSION_DEFAULT && bl_app_crc_ok()) {
            Debug_Print("[FBL] Default session + valid app -> jump\r\n");
            bl_jump_to_app();
        }

        LED_TOGGLE();
        HAL_Delay(10);
    }
}

/**
 * @brief  System clock configuration (verbatim copy of app main.c -- proven on
 *         this board: HSI->PLL 170MHz + HSE 24MHz for the FDCAN kernel clock)
 */
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef        RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef        RCC_ClkInitStruct = {0};

    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState            = RCC_HSE_ON;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM            = 4U;   /* HSI/4 = 4MHz */
    RCC_OscInitStruct.PLL.PLLN            = 85U;  /* 4MHz * 85 = 340MHz VCO */
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ            = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR            = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        while (1);
    }

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK   | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1  | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;     /* HCLK = 170MHz */
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;       /* PCLK1 = 42.5MHz */
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;       /* PCLK2 = 85MHz */

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
        while (1);
    }

    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInit.FdcanClockSelection   = RCC_FDCANCLKSOURCE_HSE;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        while (1);
    }
}

/** LD4 (PA5), active low -- same as app */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin   = LED_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);
    LED_OFF();
}
