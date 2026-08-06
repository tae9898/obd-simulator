/**
 * @file    fdcan_loopback_test.c
 * @brief   FDCAN1 internal loopback diagnostic test implementation (rev2)
 * @note    STM32G431RB -- MCU FDCAN peripheral self-verification.
 *
 *          <Important correction (rev2)>
 *          STM32G4 FDCAN message RAM is hardware-fixed (RX FIFO0=3, TX FIFO=3, etc.)
 *          and configuration registers like RXF0C/RXF1C/TXBC size fields do not exist.
 *          (RXF0S@0x090, TXBC@0x0C0, TXFQS@0x0C4, etc. use CMSIS struct members)
 *          Therefore "FIFO depth 0" diagnosis was a misjudgment; the actual symptom was
 *          "message queued in TX FIFO (depth 3) but not transmitted (LEC=7)".
 *
 *          This rev2 checks the FDCAN core clock (RCC FDCANSEL/PLLRDY/HSERDY) and
 *          TX request status (TXBAR/TXBRP/TXBTO) at correct offsets.
 */

#include "fdcan_loopback_test.h"
#include "uart_debug.h"
#include "stm32g4xx_hal.h"
#include <string.h>

/* === TX frame for testing === */
#define LB_TX_ID    0x123U
#define LB_TX_DLC   FDCAN_DLC_BYTES_8

/* === Timing === */
#define LB_RX_POLL_TIMEOUT_MS  100U
#define LB_LOOP_PERIOD_MS      1000U

/* ------------------------------------------------------------------ *
 *  Configuration/clock/TX status checks
 * ------------------------------------------------------------------ */

/**
 * @brief  FDCAN clock source + oscillator/PLL ready state + core register summary
 * @note   FDCANSEL location is CCIPR[25:24] (0=HSE,1=PLLQ,2=PCLK1).
 *         The FDCAN core (protocol engine) runs on the FDCANSEL clock.
 *         Registers are accessed via APB clock, so even if core clock is dead,
 *         register reads still work but TX will not occur (LEC=7).
 */
static void lb_dump_clock_and_core(const char *tag)
{
    uint32_t cr     = RCC->CR;
    uint32_t ccipr  = RCC->CCIPR;
    uint32_t pllcfg = RCC->PLLCFGR;
    uint32_t sel    = (ccipr >> 24U) & 0x3U;
    const char *src = (sel == 0U) ? "HSE"
                    : (sel == 1U) ? "PLLQ"
                    : (sel == 2U) ? "PCLK1"
                                  : "reserved";

    Debug_Print("\r\n---- [CLK %s] ----\r\n", tag);
    Debug_Print("  RCC.CR      = 0x%08lX  (HSIRDY=%lu HSERDY=%lu PLLRDY=%lu)\r\n",
                (unsigned long)cr,
                (unsigned long)((cr >> 10U) & 1U),
                (unsigned long)((cr >> 17U) & 1U),
                (unsigned long)((cr >> 25U) & 1U));
    Debug_Print("  FDCANSEL[%lu]=%s  CCIPR=0x%08lX\r\n",
                (unsigned long)sel, src, (unsigned long)ccipr);
    Debug_Print("  PLLCFGR=0x%08lX (PLLSRC=%lu PLLM=%lu PLLN=%lu PLLQ=%lu)\r\n",
                (unsigned long)pllcfg,
                (unsigned long)((pllcfg >> 0U) & 0x3U),
                (unsigned long)((pllcfg >> 4U) & 0x7U) + 1U,
                (unsigned long)((pllcfg >> 8U) & 0x7FU),
                (unsigned long)((pllcfg >> 21U) & 0x7U));
    {
        uint32_t cccr  = hfdcan1.Instance->CCCR;
        uint32_t test  = hfdcan1.Instance->TEST;
        uint32_t nbtp  = hfdcan1.Instance->NBTP;
        uint32_t dbtp  = hfdcan1.Instance->DBTP;
        Debug_Print("  CCCR =0x%08lX (INIT=%lu CCE=%lu TEST=%lu MON=%lu PXHD=%lu)\r\n",
                    (unsigned long)cccr,
                    (unsigned long)(cccr & 0x1U),
                    (unsigned long)((cccr >> 1U) & 1U),
                    (unsigned long)((cccr >> 7U) & 1U),
                    (unsigned long)((cccr >> 5U) & 1U),
                    (unsigned long)((cccr >> 12U) & 1U));
        Debug_Print("  TEST =0x%08lX (LBCK=%lu)  NBTP=0x%08lX  DBTP=0x%08lX\r\n",
                    (unsigned long)test, (unsigned long)((test >> 4U) & 1U),
                    (unsigned long)nbtp, (unsigned long)dbtp);
    }
}

/* ------------------------------------------------------------------ *
 *  Main entry
 * ------------------------------------------------------------------ */

void FDCAN_LoopbackTest_Run(void)
{
    HAL_StatusTypeDef status;

    Debug_Print("\r\n");
    Debug_Print("######################################################\r\n");
    Debug_Print("# FDCAN1 INTERNAL LOOPBACK DIAGNOSTIC (rev2)          #\r\n");
    Debug_Print("# Tests MCU FDCAN core ONLY (no transceiver/wiring)   #\r\n");
    Debug_Print("######################################################\r\n");

    /* --- FDCAN1 initialization: INTERNAL LOOPBACK --- */
    hfdcan1.Instance                 = FDCAN1;
    hfdcan1.Init.FrameFormat         = FDCAN_FRAME_FD_BRS;  /* CAN-FD + bitrate switching */
    hfdcan1.Init.Mode                = FDCAN_MODE_INTERNAL_LOOPBACK;
    hfdcan1.Init.AutoRetransmission  = ENABLE;
    hfdcan1.Init.TransmitPause       = DISABLE;
    hfdcan1.Init.ProtocolException   = DISABLE;
    hfdcan1.Init.NominalPrescaler     = FDCAN_PRESCALER;
    hfdcan1.Init.NominalSyncJumpWidth = FDCAN_SJW;
    hfdcan1.Init.NominalTimeSeg1      = FDCAN_TIME_SEG1;
    hfdcan1.Init.NominalTimeSeg2      = FDCAN_TIME_SEG2;
    hfdcan1.Init.DataPrescaler       = FDCAN_DATA_PRESCALER;
    hfdcan1.Init.DataSyncJumpWidth   = FDCAN_DATA_SJW;
    hfdcan1.Init.DataTimeSeg1        = FDCAN_DATA_TIME_SEG1;
    hfdcan1.Init.DataTimeSeg2        = FDCAN_DATA_TIME_SEG2;
    hfdcan1.Init.StdFiltersNbr       = 1U;
    hfdcan1.Init.ExtFiltersNbr       = 0U;
    hfdcan1.Init.TxFifoQueueMode     = FDCAN_TX_FIFO_OPERATION;

    /* === FDCAN clock source note (uses value set by SystemClock_Config as-is) ===
     * !! Verification result: PLLQ->FDCAN clock does not work on this board.
     *    - PLLQ (CCIPR FDCANSEL=01): no TX, LEC=7, loopback FAIL
     *    - PCLK1 (=10)          : loopback PASS
     *    - HSE  (=00)           : FD_BRS 2Mbps loopback PASS   <-- current app default
     * SystemClock_Config sets HSE, so we do not force-change it here.
     * (past A/B experiment: MODIFY_REG(RCC->CCIPR, RCC_CCIPR_FDCANSEL, RCC_FDCANCLKSOURCE_HSE/PCLK1);)
     */

    status = HAL_FDCAN_Init(&hfdcan1);
    Debug_Print("[LB] HAL_FDCAN_Init(loopback) = %d (0=OK)\r\n", status);

    if (status != HAL_OK) {
        Debug_Print("[LB][FATAL] Init failed\r\n");
        while (1) { LED_TOGGLE(); HAL_Delay(100); }
    }

    lb_dump_clock_and_core("after-INIT");

    /* --- Global filter: all standard frames -> RX FIFO0 --- */
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                 FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_REJECT,
                                 FDCAN_FILTER_REMOTE, FDCAN_REJECT_REMOTE);

    /* --- Standard filter 0: range 0x000~0x7FF all -> RX FIFO0 --- */
    {
        FDCAN_FilterTypeDef f = {0};
        f.IdType       = FDCAN_STANDARD_ID;
        f.FilterIndex  = 0U;
        f.FilterType   = FDCAN_FILTER_RANGE;
        f.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
        f.FilterID1    = 0x000U;
        f.FilterID2    = 0x7FFU;
        HAL_FDCAN_ConfigFilter(&hfdcan1, &f);
    }

    status = HAL_FDCAN_Start(&hfdcan1);
    Debug_Print("[LB] HAL_FDCAN_Start = %d\r\n", status);
    if (status != HAL_OK) {
        Debug_Print("[LB][FATAL] Start failed\r\n");
        while (1) { LED_TOGGLE(); HAL_Delay(100); }
    }

    /* --- TX frame header --- */
    FDCAN_TxHeaderTypeDef tx_hdr = {0};
    tx_hdr.Identifier          = LB_TX_ID;
    tx_hdr.IdType              = FDCAN_STANDARD_ID;
    tx_hdr.TxFrameType         = FDCAN_DATA_FRAME;
    tx_hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_hdr.BitRateSwitch       = FDCAN_BRS_ON;
    tx_hdr.FDFormat            = FDCAN_FD_CAN;
    tx_hdr.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    tx_hdr.MessageMarker       = 0U;
    tx_hdr.DataLength          = LB_TX_DLC;

    Debug_Print("[LB] Loop: TX ID=0x%03lX / %lums. PASS=core OK, FAIL=core/clock issue.\r\n\r\n",
                (unsigned long)LB_TX_ID, (unsigned long)LB_LOOP_PERIOD_MS);

    uint32_t round = 0U, pass_cnt = 0U, fail_cnt = 0U;

    while (1)
    {
        round++;
        uint8_t tx_data[8] = { 0x11U, 0x22U, 0x33U, 0x44U,
                               0x55U, 0x66U, 0x77U, (uint8_t)(round & 0xFFU) };

        HAL_StatusTypeDef tx_st = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_hdr, tx_data);

        /* TX status snapshot (immediately after TX) */
        uint32_t txfqs_pre = hfdcan1.Instance->TXFQS;
        uint32_t txbar_pre = hfdcan1.Instance->TXBAR;

        /* RX polling */
        uint8_t got = 0U;
        uint32_t t0 = HAL_GetTick();
        uint32_t elapsed = 0U;
        if (tx_st == HAL_OK) {
            while ((elapsed = (HAL_GetTick() - t0)) < LB_RX_POLL_TIMEOUT_MS) {
                if ((hfdcan1.Instance->RXF0S & FDCAN_RXF0S_F0FL) != 0U) { got = 1U; break; }
            }
        }

        uint32_t txbrp = hfdcan1.Instance->TXBRP;
        uint32_t txbto = hfdcan1.Instance->TXBTO;

        Debug_Print("[LB] round #%lu (P=%lu/F=%lu) tx=%d TXFQS=0x%lX TXBAR=0x%lX TXBRP=0x%lX TXBTO=0x%lX\r\n",
                    (unsigned long)round, (unsigned long)pass_cnt, (unsigned long)fail_cnt,
                    tx_st,
                    (unsigned long)txfqs_pre, (unsigned long)txbar_pre,
                    (unsigned long)txbrp, (unsigned long)txbto);

        if (got) {
            FDCAN_RxHeaderTypeDef rx_hdr = {0};
            uint8_t rx_data[8] = {0};
            if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rx_hdr, rx_data) == HAL_OK) {
                int id_ok   = (rx_hdr.Identifier == LB_TX_ID);
                int data_ok = (memcmp(rx_data, tx_data, 8) == 0);
                Debug_Print("[LB]   RX ID=0x%03lX DLC=%lu data=%02X%02X%02X%02X%02X%02X%02X%02X (+%lums)\r\n",
                            (unsigned long)rx_hdr.Identifier,
                            (unsigned long)(rx_hdr.DataLength >> 16U),
                            rx_data[0], rx_data[1], rx_data[2], rx_data[3],
                            rx_data[4], rx_data[5], rx_data[6], rx_data[7],
                            (unsigned long)elapsed);
                if (id_ok && data_ok) {
                    pass_cnt++;
                    Debug_Print("[LB]   >>>> PASS: loopback OK. MCU FDCAN core normal. <<<<\r\n");
                } else {
                    fail_cnt++;
                    Debug_Print("[LB]   !!!! FAIL: received but mismatch (id_ok=%d data_ok=%d) !!!!\r\n", id_ok, data_ok);
                }
            } else {
                fail_cnt++;
                Debug_Print("[LB]   !!!! FAIL: F0FL>0 but GetRxMessage failed !!!!\r\n");
            }
        } else {
            fail_cnt++;
            if (tx_st != HAL_OK) {
                Debug_Print("[LB]   !!!! FAIL: AddTx=%d (TX FIFO full=depth3 full, not transmitted) !!!!\r\n", tx_st);
            } else {
                Debug_Print("[LB]   !!!! FAIL: TX queued but no RX within 100ms (not transmitted, check LEC) !!!!\r\n");
            }
            /* TXBAR set but TXBTO not asserted -> core did not transmit = clock/core issue */
            Debug_Print("[LB]        TXBAR=0x%lX TXBRP=0x%lX TXBTO=0x%lX (TXBAR bit set & TXBTO 0 = core not transmitting)\r\n",
                        (unsigned long)txbar_pre, (unsigned long)txbrp, (unsigned long)txbto);
        }

        /* Status summary */
        {
            uint32_t ecr = hfdcan1.Instance->ECR;
            uint32_t psr = hfdcan1.Instance->PSR;
            Debug_Print("[LB]   TEC=%lu REC=%lu LEC=%lu RXF0S=0x%08lX\r\n",
                        (unsigned long)((ecr >> 16U) & 0xFFU),
                        (unsigned long)((ecr >> 8U) & 0xFFU),
                        (unsigned long)(psr & 0x7U),
                        (unsigned long)hfdcan1.Instance->RXF0S);
        }

        if ((round % 8U) == 0U) {
            lb_dump_clock_and_core("live");
        }

        HAL_Delay(LB_LOOP_PERIOD_MS);
    }
}
