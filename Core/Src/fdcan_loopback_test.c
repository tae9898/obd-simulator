/**
 * @file    fdcan_loopback_test.c
 * @brief   FDCAN1 내부 루프백 진단 테스트 구현 (rev2)
 * @note    STM32G431RB — MCU FDCAN 주변기기 자체 검증.
 *
 *          <중요 정정 (rev2)>
 *          STM32G4 FDCAN의 메시지 RAM 은 하드웨어 고정(RX FIFO0=3, TX FIFO=3 등)
 *          이며 RXF0C/RXF1C/TXBC 크기 필드 같은 설정 레지스터는 존재하지 않는다.
 *          (RXF0S@0x090, TXBC@0x0C0, TXFQS@0x0C4 등은 CMSIS 구조체 멤버 사용)
 *          따라서 "FIFO 깊이 0" 진단은 오판이었고, 실제 현상은
 *          "메시지가 TX FIFO(깊이 3)에 큐잉되지만 전송되지 않음(LEC=7)" 이다.
 *
 *          본 rev2 에서는 FDCAN 코어 클럭(RCC FDCANSEL/PLLRDY/HSERDY) 과
 *          TX 요청 상태(TXBAR/TXBRP/TXBTO) 를 정확한 오프셋으로 점검한다.
 */

#include "fdcan_loopback_test.h"
#include "uart_debug.h"
#include "stm32g4xx_hal.h"
#include <string.h>

/* === 테스트용 TX 프레임 === */
#define LB_TX_ID    0x123U
#define LB_TX_DLC   FDCAN_DLC_BYTES_8

/* === 타이밍 === */
#define LB_RX_POLL_TIMEOUT_MS  100U
#define LB_LOOP_PERIOD_MS      1000U

/* ------------------------------------------------------------------ *
 *  설정/클럭/TX 상태 점검
 * ------------------------------------------------------------------ */

/**
 * @brief  FDCAN 클럭 소스 + 발진기/PLL 준비 상태 + 코어 레지스터 요약
 * @note   FDCANSEL 위치는 CCIPR[25:24] (0=HSE,1=PLLQ,2=PCLK1).
 *         FDCAN 코어(프로토콜 엔진)는 FDCANSEL 클럭으로 동작한다.
 *         레지스터는 APB 클럭으로 접근되므로, 코어 클럭이 죽어도
 *         레지스터 읽기는 되지만 TX 는 일어나지 않는다(LEC=7).
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
 *  메인 진입
 * ------------------------------------------------------------------ */

void FDCAN_LoopbackTest_Run(void)
{
    HAL_StatusTypeDef status;

    Debug_Print("\r\n");
    Debug_Print("######################################################\r\n");
    Debug_Print("# FDCAN1 INTERNAL LOOPBACK DIAGNOSTIC (rev2)          #\r\n");
    Debug_Print("# Tests MCU FDCAN core ONLY (no transceiver/wiring)   #\r\n");
    Debug_Print("######################################################\r\n");

    /* --- FDCAN1 초기화: INTERNAL LOOPBACK --- */
    hfdcan1.Instance                 = FDCAN1;
    hfdcan1.Init.FrameFormat         = FDCAN_FRAME_CLASSIC;
    hfdcan1.Init.Mode                = FDCAN_MODE_INTERNAL_LOOPBACK;
    hfdcan1.Init.AutoRetransmission  = ENABLE;
    hfdcan1.Init.TransmitPause       = DISABLE;
    hfdcan1.Init.ProtocolException   = DISABLE;
    hfdcan1.Init.NominalPrescaler     = FDCAN_PRESCALER;
    hfdcan1.Init.NominalSyncJumpWidth = FDCAN_SJW;
    hfdcan1.Init.NominalTimeSeg1      = FDCAN_TIME_SEG1;
    hfdcan1.Init.NominalTimeSeg2      = FDCAN_TIME_SEG2;
    hfdcan1.Init.DataPrescaler       = 1U;
    hfdcan1.Init.DataSyncJumpWidth   = 1U;
    hfdcan1.Init.DataTimeSeg1        = 1U;
    hfdcan1.Init.DataTimeSeg2        = 1U;
    hfdcan1.Init.StdFiltersNbr       = 1U;
    hfdcan1.Init.ExtFiltersNbr       = 0U;
    hfdcan1.Init.TxFifoQueueMode     = FDCAN_TX_FIFO_OPERATION;

    /* === FDCAN 클럭 소스 안내 (SystemClock_Config 가 설정한 값을 그대로 사용) ===
     * !! 검증 결과: 본 보드에서 PLLQ->FDCAN 클럭이 동작하지 않는다.
     *    - PLLQ(CCIPR FDCANSEL=01): TX 미발생, LEC=7, 루프백 FAIL
     *    - HSE (=00)            : 루프백 PASS
     *    - PCLK1 (=10)          : 루프백 PASS   <-- 앱 기본값(SystemClock_Config)
     * SystemClock_Config 가 PCLK1 을 설정하므로 여기서 강제 변경하지 않는다.
     * (과거 A/B 실험용: MODIFY_REG(RCC->CCIPR, RCC_CCIPR_FDCANSEL, RCC_FDCANCLKSOURCE_HSE/PCLK1);)
     */

    status = HAL_FDCAN_Init(&hfdcan1);
    Debug_Print("[LB] HAL_FDCAN_Init(loopback) = %d (0=OK)\r\n", status);

    if (status != HAL_OK) {
        Debug_Print("[LB][FATAL] Init failed\r\n");
        while (1) { LED_TOGGLE(); HAL_Delay(100); }
    }

    lb_dump_clock_and_core("after-INIT");

    /* --- 글로벌 필터: 모든 표준 프레임 -> RX FIFO0 --- */
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                 FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_REJECT,
                                 FDCAN_FILTER_REMOTE, FDCAN_REJECT_REMOTE);

    /* --- 표준 필터 0: range 0x000~0x7FF 전체 -> RX FIFO0 --- */
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

    /* --- TX 프레임 헤더 --- */
    FDCAN_TxHeaderTypeDef tx_hdr = {0};
    tx_hdr.Identifier          = LB_TX_ID;
    tx_hdr.IdType              = FDCAN_STANDARD_ID;
    tx_hdr.TxFrameType         = FDCAN_DATA_FRAME;
    tx_hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_hdr.BitRateSwitch       = FDCAN_BRS_OFF;
    tx_hdr.FDFormat            = FDCAN_CLASSIC_CAN;
    tx_hdr.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    tx_hdr.MessageMarker       = 0U;
    tx_hdr.DataLength          = LB_TX_DLC;

    Debug_Print("[LB] Loop: TX ID=0x%03lX / %lums. PASS=core OK, FAIL=core/clock 문제.\r\n\r\n",
                (unsigned long)LB_TX_ID, (unsigned long)LB_LOOP_PERIOD_MS);

    uint32_t round = 0U, pass_cnt = 0U, fail_cnt = 0U;

    while (1)
    {
        round++;
        uint8_t tx_data[8] = { 0x11U, 0x22U, 0x33U, 0x44U,
                               0x55U, 0x66U, 0x77U, (uint8_t)(round & 0xFFU) };

        HAL_StatusTypeDef tx_st = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_hdr, tx_data);

        /* TX 상태 스냅샷 (TX 직후) */
        uint32_t txfqs_pre = hfdcan1.Instance->TXFQS;
        uint32_t txbar_pre = hfdcan1.Instance->TXBAR;

        /* RX 폴링 */
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
                    Debug_Print("[LB]   >>>> PASS: loopback OK. MCU FDCAN 코어 정상. <<<<\r\n");
                } else {
                    fail_cnt++;
                    Debug_Print("[LB]   !!!! FAIL: 수신했지만 불일치(id_ok=%d data_ok=%d) !!!!\r\n", id_ok, data_ok);
                }
            } else {
                fail_cnt++;
                Debug_Print("[LB]   !!!! FAIL: F0FL>0 인데 GetRxMessage 실패 !!!!\r\n");
            }
        } else {
            fail_cnt++;
            if (tx_st != HAL_OK) {
                Debug_Print("[LB]   !!!! FAIL: AddTx=%d (TX FIFO 가득=깊이3 꽉참, 전송 안 됨) !!!!\r\n", tx_st);
            } else {
                Debug_Print("[LB]   !!!! FAIL: TX 큐잉됐으나 100ms 내 RX 없음(전송 안 됨, LEC 확인) !!!!\r\n");
            }
            /* TXBAR set 되었는데 TXBTO 안 뜨면 -> 코어가 전송 안 함 = 클럭/코어 문제 */
            Debug_Print("[LB]        TXBAR=0x%lX TXBRP=0x%lX TXBTO=0x%lX (TXBAR 비트 켜짐&TXBTO 0 = 코어 미전송)\r\n",
                        (unsigned long)txbar_pre, (unsigned long)txbrp, (unsigned long)txbto);
        }

        /* 요약 상태 */
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
