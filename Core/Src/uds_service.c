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
#include <string.h>

/* === ECU 정체 정보 === */
static const char s_ecu_name[]   = "OBD-SIM-G431";
static const char s_hw_version[] = "HW Rev1.0";
static const char s_sw_version[] = "SW Phase1.0";
static const char s_vin[]        = "WVWZZZ3CZWE000001";

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
static void handle_obd2_service01(const uint8_t *req, uint16_t req_len,
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
    /* OBD-II 서비스(ISO 15031-5, Mode 0x01~0x0A)만 0x7DF 기능적 응답 허용.
     * UDS 진단 서비스(0x10/0x11/0x22/0x27/0x31)는 physical 전용 —
     * 브로드캐스트로의 세션 변경/리셋/보안접근을 막기 위한 안전 정책. */
    switch (sid) {
        case UDS_SID_OBD2_CURRENT_DATA:   /* 0x01 */
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

        case UDS_SID_DIAG_SESSION_CTRL:
            handle_session_control(request, request_len, response, response_len);
            break;

        case UDS_SID_ECU_RESET:
            handle_ecu_reset(request, request_len, response, response_len);
            break;

        case UDS_SID_READ_DATA_BY_ID:
            handle_read_data_by_id(request, request_len, response, response_len);
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

        default:
            Debug_Print("[UDS] Unsupported SID: 0x%02X\r\n", sid);
            build_negative_response(sid, NRC_SERVICE_NOT_SUPPORTED,
                                   response, response_len);
            break;
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

    uint8_t sub = req[1];

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

    /* 응답: SID+0x40, sub, P2* 타임아웃 (5000ms = 0x1388) */
    resp[0] = UDS_SID_DIAG_SESSION_CTRL + UDS_RESPONSE_SID_OFFSET;
    resp[1] = sub;
    resp[2] = 0x13U;
    resp[3] = 0x88U;
    *resp_len = 4U;

    Debug_Print("[UDS] Session -> 0x%02X\r\n", sub);
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

    uint8_t reset_type = req[1];

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

    uint8_t level = req[1];

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

        if (DiagSession_VerifyKey(key) == 0) {
            resp[0] = UDS_SID_SECURITY_ACCESS + UDS_RESPONSE_SID_OFFSET;
            resp[1] = level;
            *resp_len = 2U;
            Debug_Print("[UDS] Unlocked\r\n");
        } else {
            build_negative_response(UDS_SID_SECURITY_ACCESS, NRC_INVALID_KEY,
                                   resp, resp_len);
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

    uint8_t  sub = req[1];
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

    resp[0] = UDS_SID_ROUTINE_CONTROL + UDS_RESPONSE_SID_OFFSET;
    resp[1] = sub;
    resp[2] = (uint8_t)(routine_id >> 8U);
    resp[3] = (uint8_t)(routine_id & 0xFFU);
    *resp_len = 4U;

    Debug_Print("[UDS] Routine 0x%04X sub=%u\r\n", routine_id, sub);
}

/**
 * @brief  SID 0x01: OBD-II Mode 01 브리지
 * @note   기존 OBD2_ProcessRequest()를 UDS 파이프라인에서 호출.
 *         Phase 0 코드를 재사용하면서 CAN 직접 처리는 제거.
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
