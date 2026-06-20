/**
 * @file    fdcan_loopback_test.h
 * @brief   FDCAN1 내부 루프백 진단 테스트 헤더
 * @note    MCU FDCAN 주변기기 자체 정상 여부 판별용.
 *
 *          외부 트랜시버(MCP)/배선/CAN 핀을 완전히 우회하고 FDCAN 코어
 *          내부에서 TX -> RX 루프백을 돌려, "주변기기 + HAL + 필터 +
 *          RX FIFO0 경로"가 정상인지만 검증한다.
 *
 *          <의도>
 *          - PASS : MCU 주변기기 정상. RX 불량 원인은 외부(트랜시버 RXD,
 *                   PA11 손상 이관 배선, 레벨 등). 보드 교체 불필요.
 *          - FAIL : MCU FDCAN 주변기기/설정 문제. 덤프로 원인 특정.
 *
 *          <활성화>
 *          main.c 상단 #define RUN_FDCAN_LOOPBACK_TEST 1 설정 후 빌드/플래시.
 */

#ifndef __FDCAN_LOOPBACK_TEST_H
#define __FDCAN_LOOPBACK_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/**
 * @brief  FDCAN1 내부 루프백 진단 실행 (복귀하지 않음 - 무한 루프)
 * @note   UART 디버그 출력으로 매 라운드 결과를 보고.
 *         HAL_Init/SystemClock/UART 초기화 이후에 호출할 것.
 */
void FDCAN_LoopbackTest_Run(void);

#ifdef __cplusplus
}
#endif

#endif /* __FDCAN_LOOPBACK_TEST_H */
