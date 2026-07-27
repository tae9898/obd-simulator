/**
 * @file    uds_service.c
 * @brief   UDS (ISO 14229) 서비스 디스패처 구현
 * @note    6개 핸들러: 0x01(OBD-II), 0x10, 0x11, 0x22, 0x27, 0x31
 *
 * 디스패치 흐름:
 *   ISO-TP 조립 완료
 *     → UDS_DispatchRequest(request, len, response, &resp_len)
 *       → switch(request[0])  // SID
 *         → 각 서비스 핸들러
 *           → 긍정 응답: response에 데이터 채움
 *           → 부정 응답: build_negative_response()
 *     → ISO-TP가 response를 CAN 프레임으로 전송
 */

#include "uds_service.h"
#include "diag_session.h"
#include "obd2_simulator.h"
#include "uart_debug.h"
#include "vehicle_config.h"
#include <string.h>

/* === ECU 정체 정보 (값은 vehicle_config.h 의 단일 설정) === */
static const char s_ecu_name[]   = ECU_NAME;
static const char s_hw_version[] = VEHICLE_HW_VERSION;
static const char s_sw_version[] = VEHICLE_SW_VERSION;
static char s_vin[18]            = VEHICLE_VIN;  /* 0x2E 쓰기 가능(RAM). 17자리+NUL */

/* === 소프트 리셋 플래그 (main.c에서 확인) === */
volatile uint8_t g_soft_reset_requested = 0U;

/* === 내부 함수 === */
static void handle_session_control(const uint8_t *req, uint16_t req_len,
                                   uint8_t *resp, uint16_t *resp_len);
static void handle_ecu_reset(const uint8_t *req, uint16_t req_len,
                             uint8_t *resp, uint16_t *resp_len);
static void handle_read_data_by_id(const uint8_t *req, uint16_t req_len,
                                   uint8_t *resp, uint16_t *resp_len);
static void handle_security_access(const uint8_t *req, uint16_t req_len,
                                   uint8_t *resp, uint16_t *resp_len);
static void handle_routine_control(const uint8_t *req, uint16_t req_len,
                                   uint8_t *resp, uint16_t *resp_len);
static void handle_tester_present(const uint8_t *req, uint16_t req_len,
                                  uint8_t *resp, uint16_t *resp_len);
static void handle_write_data_by_id(const uint8_t *req, uint16_t req_len,
                                    uint8_t *resp, uint16_t *resp_len);
static void handle_communication_control(const uint8_t *req, uint16_t req_len,
                                         uint8_t *resp, uint16_t *resp_len);
static void handle_read_dtc_information(const uint8_t *req, uint16_t req_len,
                                        uint8_t *resp, uint16_t *resp_len);
static void handle_input_output_control(const uint8_t *req, uint16_t req_len,
                                        uint8_t *resp, uint16_t *resp_len);
static void handle_obd2_service01(const uint8_t *req, uint16_t req_len,
                                  uint8_t *resp, uint16_t *resp_len);
static void handle_obd2_service03(const uint8_t *req, uint16_t req_len,
                                  uint8_t *resp, uint16_t *resp_len);
static void handle_obd2_service04(const uint8_t *req, uint16_t req_len,
                                  uint8_t *resp, uint16_t *resp_len);
static void handle_obd2_service07(const uint8_t *req, uint16_t req_len,
                                  uint8_t *resp, uint16_t *resp_len);
static void handle_obd2_service09(const uint8_t *req, uint16_t req_len,
                                  uint8_t *resp, uint16_t *resp_len);
static void build_negative_response(uint8_t sid, uint8_t nrc,
                                    uint8_t *resp, uint16_t *resp_len);

/* ====================================================
 * 공개 API
 * ==================================================== */

void UDS_Init(void)
{
    g_soft_reset_requested = 0U;
    Debug_Print("[UDS] Dispatcher init OK\r\n");
}

uint8_t UDS_IsFunctionallyAddressable(uint8_t sid)
{
    /* 구현된 OBD-II 핵심 서비스(0x01/0x03/0x04/0x07/0x09)만 0x7DF 기능적 응답 허용.
     * UDS 진단 서비스(0x10/0x11/0x22/0x27/0x31)는 physical 전용 —
     * 브로드캐스트로의 세션 변경/리셋/보안접근을 막기 위한 안전 정책.
     * 미구현 OBD-II 모드(0x02/0x05/0x06/0x08/0x0A)도 제외 — 기능적 요청에는
     * 부정 응답을 보내면 안 되므로 여기서 걸러 응답을 억제한다. */
    switch (sid) {
        case UDS_SID_OBD2_CURRENT_DATA:   /* 0x01 */
        case UDS_SID_OBD2_STORED_DTC:     /* 0x03 */
        case UDS_SID_OBD2_CLEAR_DTC:      /* 0x04 */
        case UDS_SID_OBD2_PENDING_DTC:    /* 0x07 */
        case UDS_SID_OBD2_VEHICLE_INFO:   /* 0x09 */
            return 1U;
        case UDS_SID_TESTER_PRESENT:      /* 0x3E keep-alive, functional 도 허용 */
            return 1U;
        case UDS_SID_COMMUNICATION_CONTROL: /* 0x28 functional broadcast 허용 */
            return 1U;
        default:
            return 0U;
    }
}

void UDS_DispatchRequest(const uint8_t *request, uint16_t request_len,
                         AddrType_t addr,
                         uint8_t *response, uint16_t *response_len)
{
    if (request == NULL || request_len == 0U ||
        response == NULL || response_len == NULL) {
        return;
    }

    uint8_t sid = request[0];

    /* 기능적 요청(0x7DF) + 기능적 응답 미지원 서비스 → 응답 억제.
     * ISO 14229-1: 기능적 요청에 대해 응답 불가 서비스는 긍정/부정 응답 모두 송신 안 함. */
    if (addr == ADDR_FUNCTIONAL && UDS_IsFunctionallyAddressable(sid) == 0U) {
        *response_len = 0U;
        Debug_Print("[UDS] Functional req, SID 0x%02X suppressed\r\n", sid);
        return;
    }

    /* S3 타임아웃 리셋 (활동 있음) */
    DiagSession_ResetS3Timeout();

    /* SID별 라우팅 */
    switch (sid) {
        case UDS_SID_OBD2_CURRENT_DATA:
            handle_obd2_service01(request, request_len, response, response_len);
            break;

        case UDS_SID_OBD2_STORED_DTC:
            handle_obd2_service03(request, request_len, response, response_len);
            break;

        case UDS_SID_OBD2_CLEAR_DTC:
            handle_obd2_service04(request, request_len, response, response_len);
            break;

        case UDS_SID_OBD2_PENDING_DTC:
            handle_obd2_service07(request, request_len, response, response_len);
            break;

        case UDS_SID_OBD2_VEHICLE_INFO:
            handle_obd2_service09(request, request_len, response, response_len);
            break;

        case UDS_SID_DIAG_SESSION_CTRL:
            handle_session_control(request, request_len, response, response_len);
            break;

        case UDS_SID_ECU_RESET:
            handle_ecu_reset(request, request_len, response, response_len);
            break;

        case UDS_SID_READ_DATA_BY_ID:
            handle_read_data_by_id(request, request_len, response, response_len);
            break;

        case UDS_SID_READ_DTC_INFORMATION:
            handle_read_dtc_information(request, request_len, response, response_len);
            break;

        case UDS_SID_SECURITY_ACCESS:
            handle_security_access(request, request_len, response, response_len);
            break;

        case UDS_SID_ROUTINE_CONTROL:
            if (DiagSession_CheckAccess(sid) != 0) {
                build_negative_response(sid, NRC_SECURITY_ACCESS_DENIED,
                                       response, response_len);
            } else {
                handle_routine_control(request, request_len, response, response_len);
            }
            break;

        case UDS_SID_TESTER_PRESENT:
            handle_tester_present(request, request_len, response, response_len);
            break;

        case UDS_SID_COMMUNICATION_CONTROL:
            if (DiagSession_CheckAccess(sid) != 0) {
                build_negative_response(sid, NRC_SECURITY_ACCESS_DENIED,
                                       response, response_len);
            } else {
                handle_communication_control(request, request_len, response, response_len);
            }
            break;

        case UDS_SID_WRITE_DATA_BY_ID:
            if (DiagSession_CheckAccess(sid) != 0) {
                build_negative_response(sid, NRC_SECURITY_ACCESS_DENIED,
                                       response, response_len);
            } else {
                handle_write_data_by_id(request, request_len, response, response_len);
            }
            break;

        case UDS_SID_IO_CONTROL_BY_ID:
            if (DiagSession_CheckAccess(sid) != 0) {
                build_negative_response(sid, NRC_SECURITY_ACCESS_DENIED,
                                       response, response_len);
            } else {
                handle_input_output_control(request, request_len, response, response_len);
            }
            break;

        default:
            Debug_Print("[UDS] Unsupported SID: 0x%02X\r\n", sid);
            build_negative_response(sid, NRC_SERVICE_NOT_SUPPORTED,
                                   response, response_len);
            break;
    }

    /* SuppressPosRspMsgIndicationBit (ISO 14229-1 §7.1) — subfunc 기반 서비스에서
     * 진단기가 subfunc bit7=1 로 "긍정 응답 보내지 마" 요청하면 응답 송신 안 함.
     * NRC(0x7F)는 항상 송신. (0x3E 는 handle 내부에서 이미 처리; 여기선 긍정 응답만) */
    if ((*response_len > 0U) && (response[0] != 0x7FU) &&
        (request_len >= 2U) && ((request[1] & 0x80U) != 0U)) {
        switch (sid) {
            case UDS_SID_DIAG_SESSION_CTRL:
            case UDS_SID_ECU_RESET:
            case UDS_SID_SECURITY_ACCESS:
            case UDS_SID_ROUTINE_CONTROL:
            case UDS_SID_TESTER_PRESENT:
            case UDS_SID_READ_DTC_INFORMATION:
                *response_len = 0U;
                break;
            default:
                break;
        }
    }
}

/* ====================================================
 * 서비스 핸들러
 * ==================================================== */

/**
 * @brief  SID 0x10: DiagnosticSessionControl
 * @note   서브기능: 0x01=Default, 0x02=Programming, 0x03=Extended
 */
static void handle_session_control(const uint8_t *req, uint16_t req_len,
                                   uint8_t *resp, uint16_t *resp_len)
{
    if (req_len < 2U) {
        build_negative_response(UDS_SID_DIAG_SESSION_CTRL, NRC_INCORRECT_MSG_LEN,
                               resp, resp_len);
        return;
    }

    uint8_t sub = (uint8_t)(req[1] & 0x7FU);   /* bit7 = suppressPosRsp (dispatch 처리) */

    if (sub != 0x01U && sub != 0x02U && sub != 0x03U) {
        build_negative_response(UDS_SID_DIAG_SESSION_CTRL, NRC_SUB_FUNC_NOT_SUPPORTED,
                               resp, resp_len);
        return;
    }

    if (DiagSession_SetSession(sub) != 0) {
        build_negative_response(UDS_SID_DIAG_SESSION_CTRL, NRC_CONDITIONS_NOT_CORRECT,
                               resp, resp_len);
        return;
    }

    /* ISO 14229-1 DiagnosticSessionControl 긍정 응답 (6바이트):
     *   [0x50, sub, P2_H, P2_L, P2*_H, P2*_L]
     *   P2  = 서버 응답 최대 지연 (1ms 해상도)
     *   P2* = ResponsePending(0x78) 후 연장 지연 (10ms 해상도)
     * 이전엔 P2* 하나만 1ms 해상도로(0x1388) 잘못 넣었음 — 표준 위반. */
    uint16_t p2  = UDS_P2_SERVER_MAX_MS;             /* 50ms  → 0x0032 */
    uint16_t p2s = UDS_P2_STAR_SERVER_MAX_MS / 10U;  /* 5000/10 → 0x01F4 */
    resp[0] = UDS_SID_DIAG_SESSION_CTRL + UDS_RESPONSE_SID_OFFSET;
    resp[1] = sub;
    resp[2] = (uint8_t)(p2  >> 8U);
    resp[3] = (uint8_t)(p2  & 0xFFU);
    resp[4] = (uint8_t)(p2s >> 8U);
    resp[5] = (uint8_t)(p2s & 0xFFU);
    *resp_len = 6U;

    Debug_Print("[UDS] Session -> 0x%02X (P2=%ums P2*=%ums)\r\n",
                sub, UDS_P2_SERVER_MAX_MS, UDS_P2_STAR_SERVER_MAX_MS);
}

/**
 * @brief  SID 0x11: ECU Reset
 * @note   0x01=Hard (NVIC_SystemReset), 0x03=Soft (플래그만 설정)
 */
static void handle_ecu_reset(const uint8_t *req, uint16_t req_len,
                             uint8_t *resp, uint16_t *resp_len)
{
    if (req_len < 2U) {
        build_negative_response(UDS_SID_ECU_RESET, NRC_INCORRECT_MSG_LEN,
                               resp, resp_len);
        return;
    }

    uint8_t reset_type = (uint8_t)(req[1] & 0x7FU);   /* bit7 = suppressPosRsp */

    switch (reset_type) {
        case UDS_RESET_HARD:
            resp[0] = UDS_SID_ECU_RESET + UDS_RESPONSE_SID_OFFSET;
            resp[1] = UDS_RESET_HARD;
            *resp_len = 2U;
            g_soft_reset_requested = 2U;  /* hard reset 마커 */
            Debug_Print("[UDS] Hard reset requested\r\n");
            break;

        case UDS_RESET_SOFT:
            resp[0] = UDS_SID_ECU_RESET + UDS_RESPONSE_SID_OFFSET;
            resp[1] = UDS_RESET_SOFT;
            *resp_len = 2U;
            g_soft_reset_requested = 1U;
            Debug_Print("[UDS] Soft reset requested\r\n");
            break;

        default:
            build_negative_response(UDS_SID_ECU_RESET, NRC_SUB_FUNC_NOT_SUPPORTED,
                                   resp, resp_len);
            break;
    }
}

/**
 * @brief  SID 0x22: ReadDataByIdentifier
 * @note   DID: 0xF190(VIN), 0xF193(HW), 0xF195(SW), 0xF198(ECU 이름)
 */
static void handle_read_data_by_id(const uint8_t *req, uint16_t req_len,
                                   uint8_t *resp, uint16_t *resp_len)
{
    if (req_len < 3U) {
        build_negative_response(UDS_SID_READ_DATA_BY_ID, NRC_INCORRECT_MSG_LEN,
                               resp, resp_len);
        return;
    }

    uint16_t did = (uint16_t)((uint16_t)req[1] << 8U) | (uint16_t)req[2];
    const char *data_ptr = NULL;
    uint16_t data_len = 0U;

    switch (did) {
        case UDS_DID_VIN:
            data_ptr = s_vin;
            data_len = (uint16_t)(strlen(s_vin));
            break;
        case UDS_DID_HW_VERSION:
            data_ptr = s_hw_version;
            data_len = (uint16_t)(strlen(s_hw_version));
            break;
        case UDS_DID_SW_VERSION:
            data_ptr = s_sw_version;
            data_len = (uint16_t)(strlen(s_sw_version));
            break;
        case UDS_DID_ECU_NAME:
            data_ptr = s_ecu_name;
            data_len = (uint16_t)(strlen(s_ecu_name));
            break;
        default:
            build_negative_response(UDS_SID_READ_DATA_BY_ID, NRC_REQUEST_OUT_OF_RANGE,
                                   resp, resp_len);
            return;
    }

    /* 응답: SID+0x40, DID_MSB, DID_LSB, data... */
    resp[0] = UDS_SID_READ_DATA_BY_ID + UDS_RESPONSE_SID_OFFSET;
    resp[1] = (uint8_t)(did >> 8U);
    resp[2] = (uint8_t)(did & 0xFFU);
    if (data_len > 0U && data_ptr != NULL) {
        (void)memcpy(&resp[3], data_ptr, data_len);
    }
    *resp_len = (uint16_t)(3U + data_len);

    Debug_Print("[UDS] ReadDID 0x%04X -> %u bytes\r\n", did, data_len);
}

/**
 * @brief  SID 0x27: SecurityAccess
 * @note   홀수 레벨=requestSeed, 짝수 레벨=sendKey
 */
static void handle_security_access(const uint8_t *req, uint16_t req_len,
                                   uint8_t *resp, uint16_t *resp_len)
{
    if (req_len < 2U) {
        build_negative_response(UDS_SID_SECURITY_ACCESS, NRC_INCORRECT_MSG_LEN,
                               resp, resp_len);
        return;
    }

    uint8_t level = (uint8_t)(req[1] & 0x7FU);   /* bit7 = suppressPosRsp */

    if ((level & 0x01U) != 0U) {
        /* === Request Seed === */
        uint16_t seed = DiagSession_GenerateSeed();

        resp[0] = UDS_SID_SECURITY_ACCESS + UDS_RESPONSE_SID_OFFSET;
        resp[1] = level;
        resp[2] = (uint8_t)(seed >> 8U);
        resp[3] = (uint8_t)(seed & 0xFFU);
        *resp_len = 4U;

        Debug_Print("[UDS] Seed req, level=%u\r\n", level);
    } else {
        /* === Send Key === */
        if (req_len < 4U) {
            build_negative_response(UDS_SID_SECURITY_ACCESS, NRC_INCORRECT_MSG_LEN,
                                   resp, resp_len);
            return;
        }

        uint16_t key = (uint16_t)((uint16_t)req[2] << 8U) | (uint16_t)req[3];

        /* 검증 사유에 따라 ISO 14229-1 표준 NRC 선택:
         *   OK           → 긍정 응답 (0x67)
         *   INVALID      → 0x35 InvalidKey
         *   EXCEEDED     → 0x36 ExceededNumberOfAttempts (잠금 중)
         *   DELAY        → 0x37 RequiredTimeDelayNotExpired (부팅/딜레이) */
        switch (DiagSession_VerifyKey(key)) {
            case DIAG_KEY_OK:
                resp[0] = UDS_SID_SECURITY_ACCESS + UDS_RESPONSE_SID_OFFSET;
                resp[1] = level;
                *resp_len = 2U;
                Debug_Print("[UDS] Unlocked\r\n");
                break;
            case DIAG_KEY_EXCEEDED_ATTEMPTS:
                build_negative_response(UDS_SID_SECURITY_ACCESS, NRC_EXCEEDED_ATTEMPTS,
                                       resp, resp_len);
                break;
            case DIAG_KEY_DELAY_NOT_EXPIRED:
                build_negative_response(UDS_SID_SECURITY_ACCESS, NRC_REQUIRED_TIME_DELAY,
                                       resp, resp_len);
                break;
            case DIAG_KEY_INVALID:
            default:
                build_negative_response(UDS_SID_SECURITY_ACCESS, NRC_INVALID_KEY,
                                       resp, resp_len);
                break;
        }
    }
}

/**
 * @brief  SID 0x31: RoutineControl
 * @note   서브기능: 0x01=start, 0x02=stop, 0x03=requestResults
 *         루틴 ID: 0x0201=DTC Clear, 0x0202=Self Test
 */
static void handle_routine_control(const uint8_t *req, uint16_t req_len,
                                   uint8_t *resp, uint16_t *resp_len)
{
    if (req_len < 4U) {
        build_negative_response(UDS_SID_ROUTINE_CONTROL, NRC_INCORRECT_MSG_LEN,
                               resp, resp_len);
        return;
    }

    uint8_t  sub = req[1];   /* bit7(suppressPosRsp) 는 dispatch 에서 긍정 응답 억제로 처리 */
    uint16_t routine_id = (uint16_t)((uint16_t)req[2] << 8U) | (uint16_t)req[3];

    if (sub < 0x01U || sub > 0x03U) {
        build_negative_response(UDS_SID_ROUTINE_CONTROL, NRC_SUB_FUNC_NOT_SUPPORTED,
                               resp, resp_len);
        return;
    }

    if (routine_id != 0x0201U && routine_id != 0x0202U) {
        build_negative_response(UDS_SID_ROUTINE_CONTROL, NRC_REQUEST_OUT_OF_RANGE,
                               resp, resp_len);
        return;
    }

    /* RoutineControl 0x0201 (DTC Clear) start: FaultManager 리셋 */
    if (routine_id == 0x0201U && sub == UDS_ROUTINE_START) {
        OBD2_DtcClear();
    }

    resp[0] = UDS_SID_ROUTINE_CONTROL + UDS_RESPONSE_SID_OFFSET;
    resp[1] = sub;
    resp[2] = (uint8_t)(routine_id >> 8U);
    resp[3] = (uint8_t)(routine_id & 0xFFU);
    *resp_len = 4U;

    Debug_Print("[UDS] Routine 0x%04X sub=%u\r\n", routine_id, sub);
}

/**
 * @brief  SID 0x3E: TesterPresent
 * @note   진단기의 keep-alive 신호. 수신 = S3 타임아웃 리셋.
 *         (S3 리셋 자체는 dispatch 진입 시점에 이미 수행됨 — 모든 요청 공통)
 *         subfunc 0x00 = 긍정 응답 [0x7E, 0x00]
 *         subfunc 0x80 = 응답 억제 (suppressPosRspMsgIndicationBit, bit7)
 *         그 외 subfunc = NRC 0x12 (subFunctionNotSupported)
 */
static void handle_tester_present(const uint8_t *req, uint16_t req_len,
                                  uint8_t *resp, uint16_t *resp_len)
{
    if (req_len < 2U) {
        build_negative_response(UDS_SID_TESTER_PRESENT, NRC_INCORRECT_MSG_LEN,
                               resp, resp_len);
        return;
    }

    uint8_t sub = req[1];
    uint8_t sub_no_suppress = (uint8_t)(sub & 0x7FU);

    /* 유효 subfunc: 0x00 또는 0x80 (bit7 = 응답 억제) */
    if (sub_no_suppress != 0x00U) {
        build_negative_response(UDS_SID_TESTER_PRESENT, NRC_SUB_FUNC_NOT_SUPPORTED,
                               resp, resp_len);
        return;
    }

    if ((sub & 0x80U) != 0U) {
        /* suppressPosRspMsgIndicationBit → 응답 없음 (keep-alive 만 목적) */
        *resp_len = 0U;
    } else {
        resp[0] = UDS_SID_TESTER_PRESENT + UDS_RESPONSE_SID_OFFSET;  /* 0x7E */
        resp[1] = sub;                                                /* 0x00 */
        *resp_len = 2U;
    }
}

/**
 * @brief  SID 0x2E: WriteDataByIdentifier
 * @note   Extended 세션 + SecurityAccess 언락 필요 (DiagSession_CheckAccess).
 *         VIN(0xF190) 만 쓰기 가능(17바이트). 다른 DID 는 읽기 전용 → NRC 0x31.
 *         RAM 버퍼라 재부팅 시 초기화(비휘발성 아님 — 시뮬레이터 한계).
 */
static void handle_write_data_by_id(const uint8_t *req, uint16_t req_len,
                                    uint8_t *resp, uint16_t *resp_len)
{
    if (req_len < 4U) {
        build_negative_response(UDS_SID_WRITE_DATA_BY_ID, NRC_INCORRECT_MSG_LEN,
                               resp, resp_len);
        return;
    }

    uint16_t did = (uint16_t)(((uint16_t)req[1] << 8U) | (uint16_t)req[2]);
    uint16_t data_len = (uint16_t)(req_len - 3U);

    switch (did) {
        case UDS_DID_VIN:
            if (data_len != 17U) {  /* VIN = ISO 3779 17자리 */
                build_negative_response(UDS_SID_WRITE_DATA_BY_ID, NRC_INCORRECT_MSG_LEN,
                                       resp, resp_len);
                return;
            }
            (void)memcpy(s_vin, &req[3], 17U);
            s_vin[17] = '\0';
            Debug_Print("[UDS] WriteDID 0xF190 (VIN, %u bytes)\r\n", data_len);
            break;
        default:
            /* 읽기 전용 DID(HW/SW/ECU 이름) 또는 미지원 → 쓰기 거부 */
            build_negative_response(UDS_SID_WRITE_DATA_BY_ID, NRC_REQUEST_OUT_OF_RANGE,
                                   resp, resp_len);
            return;
    }

    resp[0] = UDS_SID_WRITE_DATA_BY_ID + UDS_RESPONSE_SID_OFFSET;  /* 0x6E */
    resp[1] = (uint8_t)(did >> 8U);
    resp[2] = (uint8_t)(did & 0xFFU);
    *resp_len = 3U;
}

/* === 0x28 통신 제어 상태 (시뮬레이터: 플래그만, 실제 CAN 송수신엔 영향 X) === */
static uint8_t s_comm_rx_enabled = 1U;
static uint8_t s_comm_tx_enabled = 1U;

/**
 * @brief  SID 0x28: CommunicationControl
 * @note   controlType: 0=enableRxTx 1=enableRx/disableTx 2=disableRx/enableTx
 *         3=disableRxTx. 시뮬레이터는 passive(정기 송신 없음)라 플래그만 저장하고
 *         진단 응답은 계속 송신(진단 자체는 제어 대상 아님).
 *         Extended 세션 필요(security 필수 아님). functional(0x7DF) 허용.
 */
static void handle_communication_control(const uint8_t *req, uint16_t req_len,
                                         uint8_t *resp, uint16_t *resp_len)
{
    if (req_len < 2U) {
        build_negative_response(UDS_SID_COMMUNICATION_CONTROL, NRC_INCORRECT_MSG_LEN,
                               resp, resp_len);
        return;
    }
    uint8_t control = (uint8_t)(req[1] & 0x7FU);  /* bit7 = suppressPosRsp */

    switch (control) {
        case 0x00U: s_comm_rx_enabled = 1U; s_comm_tx_enabled = 1U; break;
        case 0x01U: s_comm_rx_enabled = 1U; s_comm_tx_enabled = 0U; break;
        case 0x02U: s_comm_rx_enabled = 0U; s_comm_tx_enabled = 1U; break;
        case 0x03U: s_comm_rx_enabled = 0U; s_comm_tx_enabled = 0U; break;
        default:
            build_negative_response(UDS_SID_COMMUNICATION_CONTROL, NRC_SUB_FUNC_NOT_SUPPORTED,
                                   resp, resp_len);
            return;
    }
    /* communicationType (req[2]) — 시뮬레이터에선 무시 (애플리케이션 통신만 의미) */

    resp[0] = UDS_SID_COMMUNICATION_CONTROL + UDS_RESPONSE_SID_OFFSET;  /* 0x68 */
    resp[1] = control;
    *resp_len = 2U;
    Debug_Print("[UDS] CommCtrl: rx=%u tx=%u\r\n", s_comm_rx_enabled, s_comm_tx_enabled);
}

/**
 * @brief  SID 0x19: ReadDTCInformation (sub 0x01/0x02 만)
 * @note   0x01 = 활성 DTC 개수, 0x02 = 활성 DTC 목록(code+status).
 *         OBD-II Mode 03/07 의 UDS 표준 경로.
 *         statusMask(요청) 은 무시 — confirmed+pending 합으로 응답 (시뮬레이터 단순화).
 */
static void handle_read_dtc_information(const uint8_t *req, uint16_t req_len,
                                        uint8_t *resp, uint16_t *resp_len)
{
    if (req_len < 2U) {
        build_negative_response(UDS_SID_READ_DTC_INFORMATION, NRC_INCORRECT_MSG_LEN,
                               resp, resp_len);
        return;
    }
    uint8_t sub = (uint8_t)(req[1] & 0x7FU);  /* bit7 = suppressPosRsp */

    switch (sub) {
        case 0x01U: {  /* reportNumberOfDTCByStatusMask */
            uint8_t count = OBD2_DtcCountActive();
            resp[0] = UDS_SID_READ_DTC_INFORMATION + UDS_RESPONSE_SID_OFFSET;  /* 0x59 */
            resp[1] = sub;
            resp[2] = 0x0CU;  /* DTCStatusAvailabilityMask: bit2(pending)+bit3(confirmed) */
            resp[3] = 0x00U;  /* formatIdentifier: ISO 14229-1 (2바이트 DTC) */
            resp[4] = 0x00U;  /* DTC count high */
            resp[5] = count;  /* DTC count low */
            *resp_len = 6U;
            break;
        }
        case 0x02U: {  /* reportDTCByStatusMask */
            uint8_t dtc_buf[OBD2_DTC_COUNT * 3U];
            uint8_t n = OBD2_DtcGetActiveUds(dtc_buf, OBD2_DTC_COUNT);
            resp[0] = UDS_SID_READ_DTC_INFORMATION + UDS_RESPONSE_SID_OFFSET;
            resp[1] = sub;
            resp[2] = 0x0CU;  /* availabilityMask */
            for (uint8_t i = 0U; i < n; i++) {
                resp[(uint16_t)(3U + i * 3U)]      = dtc_buf[i * 3U];
                resp[(uint16_t)(4U + i * 3U)]      = dtc_buf[(uint8_t)(i * 3U + 1U)];
                resp[(uint16_t)(5U + i * 3U)]      = dtc_buf[(uint8_t)(i * 3U + 2U)];
            }
            *resp_len = (uint16_t)(3U + (uint16_t)n * 3U);
            break;
        }
        default:
            build_negative_response(UDS_SID_READ_DTC_INFORMATION, NRC_SUB_FUNC_NOT_SUPPORTED,
                                   resp, resp_len);
            return;
    }
}

/* === 0x2F 가상 IO 제어 상태 (시뮬레이터: 실제 액추에이터 대신 플래그) === */
static uint8_t s_io_port = 0U;

/**
 * @brief  SID 0x2F: InputOutputControlByIdentifier
 * @note   DID 0x0200(가상 IO 포트) 만 지원. Extended 세션 + SecurityAccess 필요.
 *         controlOption: 0x00=returnControlToECU(no-op), 0x03=shortTermAdjustment(값 지정).
 *         resetToDefault(0x01)/freezeCurrentState(0x02) 는 미지원 → NRC 0x12.
 */
static void handle_input_output_control(const uint8_t *req, uint16_t req_len,
                                        uint8_t *resp, uint16_t *resp_len)
{
    if (req_len < 4U) {  /* SID + DID(2) + controlOption */
        build_negative_response(UDS_SID_IO_CONTROL_BY_ID, NRC_INCORRECT_MSG_LEN,
                               resp, resp_len);
        return;
    }

    uint16_t did = (uint16_t)(((uint16_t)req[1] << 8U) | (uint16_t)req[2]);
    uint8_t  control = req[3];

    if (did != UDS_DID_IO_CONTROL) {
        build_negative_response(UDS_SID_IO_CONTROL_BY_ID, NRC_REQUEST_OUT_OF_RANGE,
                               resp, resp_len);
        return;
    }

    switch (control) {
        case 0x00U:  /* returnControlToECU — 시뮬레이터엔 자율 액추에이터 없음(no-op) */
            break;
        case 0x03U:  /* shortTermAdjustment — 테스터가 값 직접 지정 */
            if (req_len < 5U) {
                build_negative_response(UDS_SID_IO_CONTROL_BY_ID, NRC_INCORRECT_MSG_LEN,
                                       resp, resp_len);
                return;
            }
            s_io_port = req[4];
            break;
        default:  /* 0x01 resetToDefault / 0x02 freezeCurrentState 미지원 */
            build_negative_response(UDS_SID_IO_CONTROL_BY_ID, NRC_SUB_FUNC_NOT_SUPPORTED,
                                   resp, resp_len);
            return;
    }

    resp[0] = UDS_SID_IO_CONTROL_BY_ID + UDS_RESPONSE_SID_OFFSET;  /* 0x6F */
    resp[1] = (uint8_t)(did >> 8U);
    resp[2] = (uint8_t)(did & 0xFFU);
    resp[3] = control;
    resp[4] = s_io_port;   /* 현재 IO 상태 */
    *resp_len = 5U;
    Debug_Print("[UDS] IOControl 0x%04X ctrl=%u val=%u\r\n", did, control, s_io_port);
}

/**
 * @brief  SID 0x01: OBD-II Mode 01 브리지
 * @note   순수 로직 핸들러 OBD2_HandleService01()를 UDS 파이프라인에서 호출.
 *         CAN I/O는 ISO-TP가 담당하므로 여기서는 로직만 재사용.
 */
static void handle_obd2_service01(const uint8_t *req, uint16_t req_len,
                                  uint8_t *resp, uint16_t *resp_len)
{
    if (req_len < 2U) {
        build_negative_response(UDS_SID_OBD2_CURRENT_DATA, NRC_INCORRECT_MSG_LEN,
                               resp, resp_len);
        return;
    }

    uint8_t pid = req[1];

    /* 기존 OBD-II 핸들러 호출 (순수 로직, CAN I/O 없음) */
    uint8_t obd_resp[8];
    uint8_t obd_len = OBD2_HandleService01(pid, obd_resp);

    if (obd_len == 0U) {
        *resp_len = 0U;  /* 지원하지 않는 PID → 응답 없음 */
        return;
    }

    /*
     * OBD-II 응답: [len, 0x41, PID, data...]
     * UDS에서는 ISO-TP가 길이를 관리하므로 len 바이트 제외
     */
    (void)memcpy(resp, &obd_resp[1], (uint16_t)obd_len);
    *resp_len = (uint16_t)obd_len;
}

/**
 * @brief  Mode 0x03: 저장된 DTC 읽기
 * @note   ISO 15031-5 응답: [0x43, numDTC, (DTC 2바이트씩)...]
 *         시뮬레이터는 DTC 가 없으므로 빈 리스트(numDTC=0)로 응답.
 */
static void handle_obd2_service03(const uint8_t *req, uint16_t req_len,
                                  uint8_t *resp, uint16_t *resp_len)
{
    (void)req;
    (void)req_len;
    /* ISO 15031-5: [0x43, numDTC, (DTC_hi, DTC_lo)...]
     * CONFIRMED DTC 목록 반환 (FaultManager). */
    resp[0] = UDS_SID_OBD2_STORED_DTC + UDS_RESPONSE_SID_OFFSET;  /* 0x43 */
    uint8_t dtc_buf[OBD2_DTC_COUNT * 2U];
    uint8_t n = OBD2_DtcGetConfirmed(dtc_buf, OBD2_DTC_COUNT);
    resp[1] = n;  /* numDTC */
    for (uint8_t i = 0U; i < n; i++) {
        resp[(uint16_t)(2U + i * 2U)]      = dtc_buf[i * 2U];
        resp[(uint16_t)(3U + i * 2U)]      = dtc_buf[(uint8_t)(i * 2U + 1U)];
    }
    *resp_len = (uint16_t)(2U + (uint16_t)n * 2U);
}

/**
 * @brief  Mode 0x04: DTC 클리어
 * @note   ISO 15031-5 응답: [0x44]. (시뮬레이터엔 클리어할 DTC 가 없음)
 */
static void handle_obd2_service04(const uint8_t *req, uint16_t req_len,
                                  uint8_t *resp, uint16_t *resp_len)
{
    (void)req;
    (void)req_len;
    OBD2_DtcClear();  /* 모든 DTC 리셋 (stored + pending) */
    resp[0] = UDS_SID_OBD2_CLEAR_DTC + UDS_RESPONSE_SID_OFFSET;  /* 0x44 */
    *resp_len = 1U;
}

/**
 * @brief  Mode 0x07: Pending DTC 읽기
 * @note   ISO 15031-5 응답: [0x47, (DTC 2바이트씩)...] — Mode 03 과 달리
 *         numDTC 바이트가 없다. 빈이면 [0x47] 만.
 */
static void handle_obd2_service07(const uint8_t *req, uint16_t req_len,
                                  uint8_t *resp, uint16_t *resp_len)
{
    (void)req;
    (void)req_len;
    /* ISO 15031-5: [0x47, (DTC_hi, DTC_lo)...] — numDTC 바이트 없음.
     * PENDING DTC 목록 반환 (FaultManager). */
    resp[0] = UDS_SID_OBD2_PENDING_DTC + UDS_RESPONSE_SID_OFFSET;  /* 0x47 */
    uint8_t dtc_buf[OBD2_DTC_COUNT * 2U];
    uint8_t n = OBD2_DtcGetPending(dtc_buf, OBD2_DTC_COUNT);
    for (uint8_t i = 0U; i < n; i++) {
        resp[(uint16_t)(1U + i * 2U)]      = dtc_buf[i * 2U];
        resp[(uint16_t)(2U + i * 2U)]      = dtc_buf[(uint8_t)(i * 2U + 1U)];
    }
    *resp_len = (uint16_t)(1U + (uint16_t)n * 2U);
}

/**
 * @brief  Mode 0x09: 차량 정보
 * @note   INF type 0x00 = 지원 INF 목록, 0x02 = VIN (ISO 15031-5).
 *         미지원 INF 에는 OBD-II 표준대로 응답 없음(0x7F 부정 응답 사용 안 함).
 *         VIN 응답(20바이트)은 obd_resp[8] 에 안 들어가므로 UDS response 버퍼에 직접 기록.
 */
static void handle_obd2_service09(const uint8_t *req, uint16_t req_len,
                                  uint8_t *resp, uint16_t *resp_len)
{
    if (req_len < 2U) {
        *resp_len = 0U;
        return;
    }

    uint8_t inf_type = req[1];

    switch (inf_type) {
        case 0x00U:
            /* 지원 INF 비트맵: INF 0x02(VIN) 지원 → bit1 = 0x02 */
            resp[0] = UDS_SID_OBD2_VEHICLE_INFO + UDS_RESPONSE_SID_OFFSET;  /* 0x49 */
            resp[1] = 0x00U;
            resp[2] = 0x02U;  /* INF 0x02(VIN) 지원 */
            resp[3] = 0x00U;
            resp[4] = 0x00U;
            resp[5] = 0x00U;
            *resp_len = 6U;
            break;

        case 0x02U: {
            /* VIN: [0x49, 0x02, num=1, VIN(17)] = 20바이트 (CAN-FD SF) */
            size_t vin_len = strlen(s_vin);  /* VIN = ISO 3779 기준 17자리 */
            resp[0] = UDS_SID_OBD2_VEHICLE_INFO + UDS_RESPONSE_SID_OFFSET;  /* 0x49 */
            resp[1] = 0x02U;
            resp[2] = 0x01U;  /* 제공하는 VIN 개수 = 1 */
            (void)memcpy(&resp[3], s_vin, vin_len);
            *resp_len = (uint16_t)(3U + vin_len);
            break;
        }

        default:
            /* 미지원 INF → 응답 없음 */
            *resp_len = 0U;
            break;
    }
}

/**
 * @brief  부정 응답 생성: [0x7F, SID, NRC]
 */
static void build_negative_response(uint8_t sid, uint8_t nrc,
                                    uint8_t *resp, uint16_t *resp_len)
{
    resp[0] = 0x7FU;
    resp[1] = sid;
    resp[2] = nrc;
    *resp_len = 3U;

    Debug_Print("[UDS] NRC: SID=0x%02X NRC=0x%02X\r\n", sid, nrc);
}
