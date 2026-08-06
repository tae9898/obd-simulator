/**
 * @file    fdcan_config.c
 * @brief   FDCAN1 configuration implementation
 * @note    CAN-FD 500kbps/2Mbps (BRS), TX/RX FIFO, ID filter, receive interrupt
 */

#include "fdcan_config.h"
#include "iso_tp.h"
#include "uart_debug.h"
#include <string.h>

/* === Receive buffer (CAN-FD max 64 bytes) === */
static uint8_t s_rx_data[64];
static FDCAN_RxHeaderTypeDef s_rx_header;

/* === DLC code (0~15) -> actual byte count conversion table ===
 * HAL FDCAN: HAL_FDCAN_GetRxMessage populates rx_header.DataLength with
 * raw DLC code (0~15) (HAL extracts via >>16 from R1[19:16]).
 * Classic (0~8): code == byte count, but FD (9~15): 12/16/20/24/32/48/64.
 * Same mapping as HAL internal DLCtoBytes[].
 */
static const uint8_t DLC_CODE_TO_BYTES[16] = {
    0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U,
    12U, 16U, 20U, 24U, 32U, 48U, 64U
};

/**
 * @brief  Byte count -> FDCAN DLC code (raw, 0~15) conversion
 * @note   HAL rule: pass raw code to TxHeader.DataLength
 *         (HAL internally shifts <<16 into R1[19:16]).
 *         Rounded up to valid CAN-FD DLC sizes (8/12/16/20/24/32/48/64).
 */
uint32_t FDCAN_BytesToDlc(uint8_t bytes)
{
    uint32_t dlc;

    if (bytes <= 8U) {
        dlc = (uint32_t)bytes;             /* code 0~8 == byte count */
    } else if (bytes <= 12U) {
        dlc = FDCAN_DLC_BYTES_12;          /* 0x9 */
    } else if (bytes <= 16U) {
        dlc = FDCAN_DLC_BYTES_16;          /* 0xA */
    } else if (bytes <= 20U) {
        dlc = FDCAN_DLC_BYTES_20;          /* 0xB */
    } else if (bytes <= 24U) {
        dlc = FDCAN_DLC_BYTES_24;          /* 0xC */
    } else if (bytes <= 32U) {
        dlc = FDCAN_DLC_BYTES_32;          /* 0xD */
    } else if (bytes <= 48U) {
        dlc = FDCAN_DLC_BYTES_48;          /* 0xE */
    } else {
        dlc = FDCAN_DLC_BYTES_64;          /* 0xF -> 64 */
    }

    return dlc;
}

/**
 * @brief  FDCAN1 CAN-FD mode initialization
 * @param  hfdcan: FDCAN handle pointer
 * @retval HAL_OK = success
 *
 * @note   CAN-FD BRS (HSE 24MHz): arbitration 500kbps + data 2Mbps.
 *         - HSE crystal oscillation confirmed (SWD HSERDY verified) -> "HSE non-oscillation" was a misdiagnosis.
 *         - FD frame (max 64 bytes) + BRS -> data phase 2Mbps transmission.
 *         - 24MHz/2MHz=12 TQ integer division, SP 83.3% (main.h FDCAN_* macros).
 */
HAL_StatusTypeDef FDCAN1_InitFD(FDCAN_HandleTypeDef *hfdcan)
{
    HAL_StatusTypeDef status;

    /* --- FDCAN instance configuration --- */
    hfdcan->Instance                 = FDCAN1;
    hfdcan->Init.FrameFormat         = FDCAN_FRAME_FD_BRS;  /* CAN-FD + BRS (data 2Mbps) */
    hfdcan->Init.Mode                = FDCAN_MODE_NORMAL;
    hfdcan->Init.AutoRetransmission  = ENABLE;
    hfdcan->Init.TransmitPause       = DISABLE;
    hfdcan->Init.ProtocolException   = DISABLE;

    /* --- Arbitration phase: 500kbps (HSE 24MHz, presc4/seg1_9/seg2_2) --- */
    hfdcan->Init.NominalPrescaler     = FDCAN_PRESCALER;
    hfdcan->Init.NominalSyncJumpWidth = FDCAN_SJW;
    hfdcan->Init.NominalTimeSeg1      = FDCAN_TIME_SEG1;
    hfdcan->Init.NominalTimeSeg2      = FDCAN_TIME_SEG2;

    /* --- Data phase: 2Mbps BRS (HSE 24MHz, presc1/seg1_9/seg2_2) --- */
    hfdcan->Init.DataPrescaler       = FDCAN_DATA_PRESCALER;
    hfdcan->Init.DataSyncJumpWidth   = FDCAN_DATA_SJW;
    hfdcan->Init.DataTimeSeg1        = FDCAN_DATA_TIME_SEG1;
    hfdcan->Init.DataTimeSeg2        = FDCAN_DATA_TIME_SEG2;

    /* --- Filter and FIFO configuration --- */
    hfdcan->Init.StdFiltersNbr       = 1U;
    hfdcan->Init.ExtFiltersNbr       = 0U;
    hfdcan->Init.TxFifoQueueMode     = FDCAN_TX_FIFO_OPERATION;

    /* --- HAL FDCAN initialization --- */
    status = HAL_FDCAN_Init(hfdcan);
    if (status != HAL_OK) {
        Debug_Print("[FDCAN] HAL_FDCAN_Init (FD) failed: %d\r\n", status);
        return status;
    }

    Debug_Print("[FDCAN] Init OK - FD BRS nominal 500kbps / data 2Mbps (HSE 24MHz)\r\n");
    return HAL_OK;
}

/**
 * @brief  FDCAN1 RX filter configuration (receive only CAN ID 0x7E0)
 * @param  hfdcan: FDCAN handle pointer
 * @retval HAL_OK = success
 *
 * @note   Filter 0 configuration:
 *         - Filter type: range mode
 *         - ID type: standard (11-bit)
 *         - Filter ID: 0x7E0
 *         - Filter mask: 0x7FF (full bit match)
 *         - FIFO assignment: FIFO0
 */
HAL_StatusTypeDef FDCAN1_ConfigureFilters(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_FilterTypeDef filter_config;

    /* Filter: RANGE 0x000~0x7FF (all standard frames) -> RX FIFO0
     * Configuration verified in loopback test. (previous MASK filter did not work for external reception, switched to RANGE) */
    filter_config.IdType       = FDCAN_STANDARD_ID;
    filter_config.FilterIndex  = 0U;
    filter_config.FilterType   = FDCAN_FILTER_RANGE;
    filter_config.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter_config.FilterID1    = 0x000U;
    filter_config.FilterID2    = 0x7FFU;

    /* Call HAL filter configuration API */
    if (HAL_FDCAN_ConfigFilter(hfdcan, &filter_config) != HAL_OK) {
        Debug_Print("[FDCAN] Filter config failed\r\n");
        return HAL_ERROR;
    }

    /* Receive path register dump (verify global filter - STM32G4 has no RXF0C, FIFO0 size is fixed) */
    {
        uint32_t rxgfc = hfdcan->Instance->RXGFC;
        Debug_Print("[FILT] RXGFC=0x%08lX (ANFS=%lu LSS=%lu RRFE=%lu)\r\n",
                    (unsigned long)rxgfc, (unsigned long)(rxgfc & 0x3UL),
                    (unsigned long)((rxgfc >> 16UL) & 0x7UL),
                    (unsigned long)((rxgfc >> 7UL) & 0x1UL));
    }

    Debug_Print("[FDCAN] Filter OK - RANGE 0x000-0x7FF -> FIFO0\r\n");
    return HAL_OK;
}

/**
 * @brief  FDCAN1 receive interrupt enable
 * @param  hfdcan: FDCAN handle pointer
 * @retval HAL_OK = success
 *
 * @note   Enabled interrupts:
 *         - RX FIFO0 new message received
 */
HAL_StatusTypeDef FDCAN1_StartNotification(FDCAN_HandleTypeDef *hfdcan)
{
    HAL_StatusTypeDef status;

    /*
     * Set FDCAN interrupt priority to 6
     *
     * Cortex-M4 NVIC: lower number = higher priority (0=highest, 15=lowest)
     * FreeRTOS rule: FromISR API must not be called at priority 5 or below (0~4)
     * Priority 6 allows FromISR calls -> Queue send OK
     */
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);

    /* --- Error interrupt line (IT1) configuration --- */
    HAL_NVIC_SetPriority(FDCAN1_IT1_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT1_IRQn);

    /* Enable RX FIFO0 new message + error interrupts */
    status = HAL_FDCAN_ActivateNotification(
        hfdcan,
        FDCAN_IT_RX_FIFO0_NEW_MESSAGE
      | FDCAN_IT_ERROR_WARNING      /* TEC/REC > 96 */
      | FDCAN_IT_ERROR_PASSIVE      /* TEC/REC > 127 */
      | FDCAN_IT_BUS_OFF,           /* TEC > 255 (fatal) */
        0U  /* FIFO0 watermark: 0 = interrupt for every message */
    );

    if (status != HAL_OK) {
        Debug_Print("[FDCAN] Notification activate failed\r\n");
        return status;
    }

    Debug_Print("[FDCAN] RX + Error interrupts enabled\r\n");
    return HAL_OK;
}

/**
 * @brief  FDCAN1 RX FIFO0 new message callback (HAL registered)
 *
 * @note   ISR -> Queue -> Task flow:
 *         1. Read message from RX FIFO0
 *         2. Copy to CAN_RxMessage_t
 *         3. Send to Queue via xQueueSendFromISR
 *         4. Return immediately (keep ISR as short as possible)
 *
 *         xQueueSendFromISR: ISR-only Queue send function
 *         - Regular xQueueSend must not be called from ISR (deadlock risk)
 *         - FromISR version does not wake the scheduler directly,
 *           pxHigherPriorityTaskWoken just flags "context switch needed"
 *         - portYIELD_FROM_ISR performs the actual switch
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                                uint32_t RxFifo0ITs)
{
    HAL_StatusTypeDef status;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0U) {

        status = HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0,
                                         &s_rx_header, s_rx_data);
        if (status == HAL_OK) {
            CAN_RxMessage_t rx_msg;
            rx_msg.can_id = s_rx_header.Identifier;

            /* DLC code (0~15) -> actual byte count. HAL gives raw code,
             * so use lookup table instead of the old ">> 16" decoding (bug: always 0). */
            uint8_t dlc_code = (uint8_t)(s_rx_header.DataLength & 0xFU);
            uint8_t byte_len = DLC_CODE_TO_BYTES[dlc_code];
            if (byte_len > 64U) {
                byte_len = 64U;   /* guard (should not happen normally) */
            }
            rx_msg.dlc = byte_len;

            /* Copy data array (CAN-FD max 64 bytes).
             * HAL has already filled s_rx_data with byte_len bytes. */
            (void)memset(rx_msg.data, 0, sizeof(rx_msg.data));
            for (uint8_t i = 0U; i < byte_len; i++) {
                rx_msg.data[i] = s_rx_data[i];
            }

            /* Send to Queue (ISR-only function) */
            xQueueSendFromISR(xCanRxQueue, &rx_msg, &xHigherPriorityTaskWoken);
        }

        /* Re-enable all interrupts (RX + error) */
        HAL_FDCAN_ActivateNotification(
            hfdcan,
            FDCAN_IT_RX_FIFO0_NEW_MESSAGE
          | FDCAN_IT_ERROR_WARNING
          | FDCAN_IT_ERROR_PASSIVE
          | FDCAN_IT_BUS_OFF,
            0U
        );

        /* Immediate context switch if Queue receive task has higher priority */
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
