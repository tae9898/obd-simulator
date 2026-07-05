/**
 * @file    uds_service.h
 * @brief   UDS (ISO 14229) 서비스 디스패처 헤더
 * @note    5개 UDS 서비스 + OBD-II Mode 01 레거시 브리지
 *
 * UDS 서비스 ID (SID):
 *   첫 바이트가 SID. 응답은 SID + 0x40.
 *   에러 시 0x7F + SID + NRC.
 *
 *   예: 요청 [0x22 0xF1 0x90] (VIN 읽기)
 *       응답 [0x62 0xF1 0x90 W V W ...]  (0x22 + 0x40 = 0x62)
 *       에러  [0x7F 0x22 0x31]            (NRC 0x31 = 범위 초과)
 */

#ifndef __UDS_SERVICE_H
#define __UDS_SERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "can_addressing.h"   /* AddrType_t */

/* === UDS 서비스 ID === */
#define UDS_SID_DIAG_SESSION_CTRL    0x10U  /**< DiagnosticSessionControl */
#define UDS_SID_ECU_RESET           0x11U  /**< ECU Reset */
#define UDS_SID_READ_DATA_BY_ID     0x22U  /**< ReadDataByIdentifier */
#define UDS_SID_SECURITY_ACCESS     0x27U  /**< SecurityAccess */
#define UDS_SID_ROUTINE_CONTROL     0x31U  /**< RoutineControl */
#define UDS_SID_OBD2_CURRENT_DATA   0x01U  /**< OBD-II Mode 01 (레거시) */

/* === 응답 SID 오프셋 === */
#define UDS_RESPONSE_SID_OFFSET     0x40U  /**< 응답 SID = 요청 SID + 0x40 */

/* === NRC (Negative Response Codes) === */
/**
 * NRC는 UDS 에러 코드.
 * 부정 응답 형식: [0x7F, SID, NRC]
 */
#define NRC_SERVICE_NOT_SUPPORTED   0x11U  /**< 지원하지 않는 서비스 */
#define NRC_SUB_FUNC_NOT_SUPPORTED  0x12U  /**< 지원하지 않는 서브기능 */
#define NRC_INCORRECT_MSG_LEN       0x13U  /**< 잘못된 메시지 길이 */
#define NRC_CONDITIONS_NOT_CORRECT  0x22U  /**< 조건 불충족 */
#define NRC_REQUEST_OUT_OF_RANGE    0x31U  /**< 범위 초과 */
#define NRC_SECURITY_ACCESS_DENIED  0x33U  /**< 시큐리티 접근 거부 */
#define NRC_INVALID_KEY             0x35U  /**< 잘못된 키 */
#define NRC_EXCEEDED_ATTEMPTS       0x36U  /**< 시도 횟수 초과 */

/* === ECU Reset 서브기능 === */
#define UDS_RESET_HARD              0x01U  /**< 하드 리셋 (전체 재시작) */
#define UDS_RESET_SOFT              0x03U  /**< 소프트 리셋 (초기화) */

/* === DID (Data Identifier) === */
#define UDS_DID_VIN                 0xF190U  /**< 차량 식별 번호 */
#define UDS_DID_HW_VERSION          0xF193U  /**< 하드웨어 버전 */
#define UDS_DID_SW_VERSION          0xF195U  /**< 소프트웨어 버전 */
#define UDS_DID_ECU_NAME            0xF198U  /**< ECU 이름 */

/* === Routine Control === */
#define UDS_ROUTINE_START           0x01U
#define UDS_ROUTINE_STOP            0x02U
#define UDS_ROUTINE_REQUEST_RESULT  0x03U
#define UDS_ROUTINE_ID_DTC_CLEAR    0x0201U  /**< DTC 클리어 */
#define UDS_ROUTINE_ID_SELF_TEST    0x0202U  /**< 자가 테스트 */

/* === 최대 응답 크기 === */
#define UDS_MAX_RESPONSE_SIZE       64U

/* === API === */

void UDS_Init(void);

/**
 * @brief  UDS 요청 디스패치
 * @param  request:      요청 데이터 (SID + 파라미터)
 * @param  request_len:  요청 길이
 * @param  addr:         수신 어드레싱 타입 (Physical/Functional)
 * @param  response:     응답 버퍼
 * @param  response_len: 응답 길이 (출력, 0=응답 없음)
 * @note   ISO-TP에서 조립 완료 시 호출됨.
 *         기능적 요청(addr==ADDR_FUNCTIONAL)이고 해당 SID가 기능적 응답을
 *         지원하지 않으면 응답을 억제(*response_len=0)한다 (ISO 14229-1).
 */
void UDS_DispatchRequest(const uint8_t *request, uint16_t request_len,
                         AddrType_t addr,
                         uint8_t *response, uint16_t *response_len);

/**
 * @brief  SID 가 기능적 어드레싱(0x7DF)에서 응답 가능한 서비스인지 판단
 * @note   OBD-II 서비스(0x01~0x0A, ISO 15031-5)만 기능적 응답 허용.
 *         UDS 진단 서비스는 physical 전용(안전 정책: 브로드캐스트 세션/리셋 방지).
 *         이 테이블 한 곳에서 정책 조정 가능.
 */
uint8_t UDS_IsFunctionallyAddressable(uint8_t sid);

/* === 외부 변수 === */
extern volatile uint8_t g_soft_reset_requested;

#ifdef __cplusplus
}
#endif

#endif /* __UDS_SERVICE_H */
