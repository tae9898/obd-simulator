/**
 * @file    uds_service.c
 * @brief   UDS (ISO 14229) service dispatcher implementation
 * @note    6 handlers: 0x01(OBD-II), 0x10, 0x11, 0x22, 0x27, 0x31
 *
 * Dispatch flow:
 *   ISO-TP reassembly complete
 *     -> UDS_DispatchRequest(request, len, response, &resp_len)
 *       -> switch(request[0])  // SID
 *         -> each service handler
 *           -> positive response: fill response with data
 *           -> negative response: build_negative_response()
 *     -> ISO-TP transmits response as CAN frame
 */

#include "uds_service.h"
#include "diag_session.h"
#include "obd2_simulator.h"
#include "uart_debug.h"
#include "vehicle_config.h"
#include "ota_flash.h"   /* OTA flash erase/write (0x34/0x36/0x37) */
#include <string.h>

/* === ECU identity information (values sourced from vehicle_config.h single config) === */
static const char s_ecu_name[]   = ECU_NAME;
static const char s_hw_version[] = VEHICLE_HW_VERSION;
static const char s_sw_version[] = VEHICLE_SW_VERSION;
static char s_vin[18]            = VEHICLE_VIN;  /* 0x2E writable (RAM). 17 chars + NUL */

/* DID read registry (0x22, 3.2) -- add one line per new DID.
 * 0x2E (write) is handled separately via VIN special case in handle_write_data_by_id. */
typedef struct {
    uint16_t   did;
    const char *data;
} did_read_t;

static const did_read_t k_did_read[] = {
    { UDS_DID_VIN,        s_vin        },
    { UDS_DID_HW_VERSION, s_hw_version },
    { UDS_DID_SW_VERSION, s_sw_version },
    { UDS_DID_ECU_NAME,   s_ecu_name   },
};

/* === Soft reset flag (checked in main.c) === */
volatile uint8_t g_soft_reset_requested = 0U;

/* === Internal functions === */
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
static void handle_request_download(const uint8_t *req, uint16_t req_len,
                                    uint8_t *resp, uint16_t *resp_len);
static void handle_transfer_data(const uint8_t *req, uint16_t req_len,
                                 uint8_t *resp, uint16_t *resp_len);
static void handle_request_transfer_exit(const uint8_t *req, uint16_t req_len,
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
 * Public API
 * ==================================================== */

void UDS_Init(void)
{
    g_soft_reset_requested = 0U;
    Debug_Print("[UDS] Dispatcher init OK\r\n");
}

uint8_t UDS_IsFunctionallyAddressable(uint8_t sid)
{
    /* Only implemented OBD-II core services (0x01/0x03/0x04/0x07/0x09) allow
     * 0x7DF functional response. UDS diagnostic services (0x10/0x11/0x22/0x27/0x31)
     * are physical-only -- safety policy to prevent broadcast session change/reset/security access.
     * Unimplemented OBD-II modes (0x02/0x05/0x06/0x08/0x0A) also excluded --
     * functional requests must not receive negative responses, so filter here
     * to suppress response. */
    switch (sid) {
        case UDS_SID_OBD2_CURRENT_DATA:   /* 0x01 */
        case UDS_SID_OBD2_STORED_DTC:     /* 0x03 */
        case UDS_SID_OBD2_CLEAR_DTC:      /* 0x04 */
        case UDS_SID_OBD2_PENDING_DTC:    /* 0x07 */
        case UDS_SID_OBD2_VEHICLE_INFO:   /* 0x09 */
            return 1U;
        case UDS_SID_TESTER_PRESENT:      /* 0x3E keep-alive, functional also allowed */
            return 1U;
        case UDS_SID_COMMUNICATION_CONTROL: /* 0x28 functional broadcast allowed */
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

    /* Functional request (0x7DF) + service not functional-response-capable -> suppress response.
     * ISO 14229-1: services that cannot respond to functional requests transmit
     * neither positive nor negative response. */
    if (addr == ADDR_FUNCTIONAL && UDS_IsFunctionallyAddressable(sid) == 0U) {
        *response_len = 0U;
        Debug_Print("[UDS] Functional req, SID 0x%02X suppressed\r\n", sid);
        return;
    }

    /* S3 timeout reset (activity detected) */
    DiagSession_ResetS3Timeout();

    /* Per-SID routing */
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

        case UDS_SID_REQUEST_DOWNLOAD:
            if (DiagSession_CheckAccess(sid) != 0) {
                build_negative_response(sid, NRC_SECURITY_ACCESS_DENIED,
                                       response, response_len);
            } else {
                handle_request_download(request, request_len, response, response_len);
            }
            break;

        case UDS_SID_TRANSFER_DATA:
            if (DiagSession_CheckAccess(sid) != 0) {
                build_negative_response(sid, NRC_SECURITY_ACCESS_DENIED,
                                       response, response_len);
            } else {
                handle_transfer_data(request, request_len, response, response_len);
            }
            break;

        case UDS_SID_REQUEST_TRANSFER_EXIT:
            if (DiagSession_CheckAccess(sid) != 0) {
                build_negative_response(sid, NRC_SECURITY_ACCESS_DENIED,
                                       response, response_len);
            } else {
                handle_request_transfer_exit(request, request_len, response, response_len);
            }
            break;

        default:
            Debug_Print("[UDS] Unsupported SID: 0x%02X\r\n", sid);
            build_negative_response(sid, NRC_SERVICE_NOT_SUPPORTED,
                                   response, response_len);
            break;
    }

    /* SuppressPosRspMsgIndicationBit (ISO 14229-1 7.1) -- subfunc-based services:
     * when tester sets subfunc bit7=1 meaning "do not send positive response",
     * suppress response transmission. NRC (0x7F) is always transmitted.
     * (0x3E already handled inside handler; only positive responses here) */
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
 * Service handlers
 * ==================================================== */

/**
 * @brief  SID 0x10: DiagnosticSessionControl
 * @note   Subfunction: 0x01=Default, 0x02=Programming, 0x03=Extended
 */
static void handle_session_control(const uint8_t *req, uint16_t req_len,
                                   uint8_t *resp, uint16_t *resp_len)
{
    if (req_len < 2U) {
        build_negative_response(UDS_SID_DIAG_SESSION_CTRL, NRC_INCORRECT_MSG_LEN,
                               resp, resp_len);
        return;
    }

    uint8_t sub = (uint8_t)(req[1] & 0x7FU);   /* bit7 = suppressPosRsp (handled by dispatch) */

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

    /* ISO 14229-1 DiagnosticSessionControl positive response (6 bytes):
     *   [0x50, sub, P2_H, P2_L, P2*_H, P2*_L]
     *   P2  = server max response delay (1ms resolution)
     *   P2* = extended delay after ResponsePending(0x78) (10ms resolution)
     * Previously P2* was incorrectly set with 1ms resolution (0x1388) -- standard violation. */
    uint16_t p2  = UDS_P2_SERVER_MAX_MS;             /* 50ms  -> 0x0032 */
    uint16_t p2s = UDS_P2_STAR_SERVER_MAX_MS / 10U;  /* 5000/10 -> 0x01F4 */
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
 * @note   0x01=Hard (NVIC_SystemReset), 0x03=Soft (flag only)
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
            g_soft_reset_requested = 2U;  /* hard reset marker */
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
 * @note   DID: 0xF190(VIN), 0xF193(HW), 0xF195(SW), 0xF198(ECU name)
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

    /* DID registry lookup (3.2) */
    for (uint8_t i = 0U;
         i < (uint8_t)(sizeof(k_did_read) / sizeof(k_did_read[0]));
         i++) {
        if (k_did_read[i].did == did) {
            data_ptr = k_did_read[i].data;
            break;
        }
    }
    if (data_ptr == NULL) {
        build_negative_response(UDS_SID_READ_DATA_BY_ID, NRC_REQUEST_OUT_OF_RANGE,
                               resp, resp_len);
        return;
    }
    uint16_t data_len = (uint16_t)(strlen(data_ptr));

    /* Response: SID+0x40, DID_MSB, DID_LSB, data... */
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
 * @note   Odd level=requestSeed, even level=sendKey
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
        /* Also reject requestSeed during boot delay/lockout (M2): prevents bypass where
         * attacker obtains seed early, computes key offline, then sends on lockout expiry. */
        switch (DiagSession_CheckSecurityGate()) {
            case DIAG_SEC_GATE_DELAY:
                build_negative_response(UDS_SID_SECURITY_ACCESS, NRC_REQUIRED_TIME_DELAY,
                                       resp, resp_len);
                return;
            case DIAG_SEC_GATE_LOCKED:
                build_negative_response(UDS_SID_SECURITY_ACCESS, NRC_EXCEEDED_ATTEMPTS,
                                       resp, resp_len);
                return;
            default:
                break;
        }

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

        /* Select ISO 14229-1 standard NRC based on verification result:
         *   OK           -> positive response (0x67)
         *   INVALID      -> 0x35 InvalidKey
         *   EXCEEDED     -> 0x36 ExceededNumberOfAttempts (locked)
         *   DELAY        -> 0x37 RequiredTimeDelayNotExpired (boot/delay) */
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
 * @note   Subfunction: 0x01=start, 0x02=stop, 0x03=requestResults
 *         Routine ID: 0x0201=DTC Clear, 0x0202=Self Test
 */
static void handle_routine_control(const uint8_t *req, uint16_t req_len,
                                   uint8_t *resp, uint16_t *resp_len)
{
    if (req_len < 4U) {
        build_negative_response(UDS_SID_ROUTINE_CONTROL, NRC_INCORRECT_MSG_LEN,
                               resp, resp_len);
        return;
    }

    uint8_t  sub = req[1];   /* bit7 (suppressPosRsp) handled by dispatch as positive response suppression */
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

    /* RoutineControl 0x0201 (DTC Clear) start: reset FaultManager */
    if (routine_id == 0x0201U && sub == UDS_ROUTINE_START) {
        OBD2_DtcClear();
    }
    /* RoutineControl 0x0202 (Self Test) start: inject demo CONFIRMED DTC (test hook,
     * see OBD2_DtcInjectDemo) so 0x19 data paths are verifiable on hardware. */
    if (routine_id == 0x0202U && sub == UDS_ROUTINE_START) {
        OBD2_DtcInjectDemo();
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
 * @note   Tester keep-alive signal. Reception = S3 timeout reset.
 *         (S3 reset itself is already done at dispatch entry -- common to all requests)
 *         subfunc 0x00 = positive response [0x7E, 0x00]
 *         subfunc 0x80 = suppress response (suppressPosRspMsgIndicationBit, bit7)
 *         any other subfunc = NRC 0x12 (subFunctionNotSupported)
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

    /* Valid subfunc: 0x00 or 0x80 (bit7 = suppress response) */
    if (sub_no_suppress != 0x00U) {
        build_negative_response(UDS_SID_TESTER_PRESENT, NRC_SUB_FUNC_NOT_SUPPORTED,
                               resp, resp_len);
        return;
    }

    if ((sub & 0x80U) != 0U) {
        /* suppressPosRspMsgIndicationBit -> no response (keep-alive only) */
        *resp_len = 0U;
    } else {
        resp[0] = UDS_SID_TESTER_PRESENT + UDS_RESPONSE_SID_OFFSET;  /* 0x7E */
        resp[1] = sub;                                                /* 0x00 */
        *resp_len = 2U;
    }
}

/**
 * @brief  SID 0x2E: WriteDataByIdentifier
 * @note   Requires Extended session + SecurityAccess unlock (DiagSession_CheckAccess).
 *         Only VIN (0xF190) is writable (17 bytes). Other DIDs are read-only -> NRC 0x31.
 *         RAM buffer, so reverts on reboot (non-volatile -- simulator limitation).
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
            if (data_len != 17U) {  /* VIN = ISO 3779 17 characters */
                build_negative_response(UDS_SID_WRITE_DATA_BY_ID, NRC_INCORRECT_MSG_LEN,
                                       resp, resp_len);
                return;
            }
            (void)memcpy(s_vin, &req[3], 17U);
            s_vin[17] = '\0';
            Debug_Print("[UDS] WriteDID 0xF190 (VIN, %u bytes)\r\n", data_len);
            break;
        default:
            /* Read-only DID (HW/SW/ECU name) or unsupported -> reject write */
            build_negative_response(UDS_SID_WRITE_DATA_BY_ID, NRC_REQUEST_OUT_OF_RANGE,
                                   resp, resp_len);
            return;
    }

    resp[0] = UDS_SID_WRITE_DATA_BY_ID + UDS_RESPONSE_SID_OFFSET;  /* 0x6E */
    resp[1] = (uint8_t)(did >> 8U);
    resp[2] = (uint8_t)(did & 0xFFU);
    *resp_len = 3U;
}

/* === 0x28 Communication control state (simulator: flags only, no actual CAN TX/RX effect) === */
static uint8_t s_comm_rx_enabled = 1U;
static uint8_t s_comm_tx_enabled = 1U;

/**
 * @brief  SID 0x28: CommunicationControl
 * @note   controlType: 0=enableRxTx 1=enableRx/disableTx 2=disableRx/enableTx
 *         3=disableRxTx. Simulator is passive (no periodic transmit), so flags are stored
 *         but diagnostic responses continue to be transmitted (diagnostics are not controlled).
 *         Extended session required (security not required). Functional (0x7DF) allowed.
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
    /* communicationType (req[2]) -- ignored by simulator (application communication only) */

    resp[0] = UDS_SID_COMMUNICATION_CONTROL + UDS_RESPONSE_SID_OFFSET;  /* 0x68 */
    resp[1] = control;
    *resp_len = 2U;
    Debug_Print("[UDS] CommCtrl: rx=%u tx=%u\r\n", s_comm_rx_enabled, s_comm_tx_enabled);
}

/* === 0x19 ReadDTCInformation helpers (ISO 14229-1:2013 11.3) === */

/* DTCSeverityAvailabilityMask: bits 5-7 severity + bit1 DTC class 1 (Annex D.3) */
#define UDS_DTC_SEV_AVAIL_MASK    0xE2U
/* DTCFormatIdentifier: 0x01 = ISO 14229-1 DTCAndStatusRecord (2-byte code << 8),
 * 0x04 = WWH-OBD (0x42/0x55 only -- same 3-byte encoding, failure type 0x00) */
#define UDS_DTC_FMT_ISO14229      0x01U
#define UDS_DTC_FMT_WWH_OBD       0x04U
/* FunctionalGroupIdentifier we serve (all monitored DTCs are emissions P0 codes) */
#define UDS_FGID_EMISSIONS        0x33U
#define UDS_FGID_ALL              0xFFU
/* Single user-defined memory selection byte we support (sub 0x17/0x18/0x19) */
#define UDS_MEMORY_SELECTION_1    0x01U
/* Snapshot record number we store (one freeze frame per DTC) */
#define UDS_DTC_SNAPSHOT_RECORD   0x01U

/** Append 3-byte DTC (2-byte J2010 code << 8, failure type 0x00). Returns bytes written. */
static uint8_t dtc_put3(uint8_t *p, uint16_t code)
{
    p[0] = (uint8_t)(code >> 8U);
    p[1] = (uint8_t)(code & 0xFFU);
    p[2] = 0x00U;
    return 3U;
}

/** Append snapshot payload [DID(2) rpm(2) speed(1) coolant(1)] for a DTC with snapshot. */
static uint8_t dtc_put_snapshot(uint8_t *p, const DtcEntry_t *d)
{
    p[0] = (uint8_t)(DTC_SNAPSHOT_DID >> 8U);
    p[1] = (uint8_t)(DTC_SNAPSHOT_DID & 0xFFU);
    p[2] = (uint8_t)(d->snapshot.rpm >> 8U);
    p[3] = (uint8_t)(d->snapshot.rpm & 0xFFU);
    p[4] = d->snapshot.speed;
    p[5] = d->snapshot.coolant;
    return 6U;
}

/** statusOfDTC mask match (unsupported mask bits ignored per 11.3.1) */
static uint8_t dtc_mask_match(const DtcEntry_t *d, uint8_t mask)
{
    return ((OBD2_DtcStatusByte(d) & mask & OBD2_DTC_STATUS_AVAILABILITY_MASK) != 0U)
               ? 1U : 0U;
}

/**
 * @brief  SID 0x19: ReadDTCInformation -- all sub-functions (ISO 14229-1:2013 11.3)
 * @note   Request [0x19, subFunction, ...], positive response [0x59, subFunction, ...].
 *         DTC records are 3-byte (2-byte J2010 code << 8).
 *         Zero matches -> positive response up to echo/availability mask only.
 *         NRC 0x12 unknown sub / 0x13 bad length / 0x31 unknown DTC, record number,
 *         memorySelection, or functionalGroupIdentifier.
 *         Data source: g_dtc_table state machine (OBD2_DtcUpdate) + g_dtc_mirror
 *         (archived at clear). Reading entry fields unlocked is safe here: all fields
 *         are <=16-bit aligned scalars and cross-field tearing is cosmetic in a simulator.
 */
static void handle_read_dtc_information(const uint8_t *req, uint16_t req_len,
                                        uint8_t *resp, uint16_t *resp_len)
{
    if (req_len < 2U) {
        build_negative_response(UDS_SID_READ_DTC_INFORMATION, NRC_INCORRECT_MSG_LEN,
                               resp, resp_len);
        return;
    }
    uint8_t sub = (uint8_t)(req[1] & 0x7FU);  /* bit7 = suppressPosRsp (dispatch) */
    uint16_t pos = 0U;
    resp[pos++] = UDS_SID_READ_DTC_INFORMATION + UDS_RESPONSE_SID_OFFSET;  /* 0x59 */
    resp[pos++] = sub;

    switch (sub) {
        case 0x01U:    /* reportNumberOfDTCByStatusMask */
        case 0x11U:    /* reportNumberOfMirrorMemoryDTCByStatusMask */
        case 0x12U: {  /* reportNumberOfEmissionsOBDDTCByStatusMask (all DTCs are emissions P0) */
            if (req_len < 3U) {
                build_negative_response(UDS_SID_READ_DTC_INFORMATION, NRC_INCORRECT_MSG_LEN,
                                       resp, resp_len);
                return;
            }
            uint8_t count = (sub == 0x11U) ? OBD2_MirrorCountByMask(req[2])
                                           : OBD2_DtcCountByMask(req[2]);
            resp[pos++] = OBD2_DTC_STATUS_AVAILABILITY_MASK;
            resp[pos++] = UDS_DTC_FMT_ISO14229;
            resp[pos++] = 0x00U;
            resp[pos++] = count;
            *resp_len = pos;
            break;
        }

        case 0x02U:    /* reportDTCByStatusMask */
        case 0x0FU:    /* reportMirrorMemoryDTCByStatusMask */
        case 0x13U:    /* reportEmissionsOBDDTCByStatusMask */
        case 0x17U: {  /* reportUserDefMemoryDTCByStatusMask: [statusMask, memorySelection] */
            uint8_t min_len = (sub == 0x17U) ? 4U : 3U;
            if (req_len < min_len) {
                build_negative_response(UDS_SID_READ_DTC_INFORMATION, NRC_INCORRECT_MSG_LEN,
                                       resp, resp_len);
                return;
            }
            uint8_t mask = req[2];
            if (sub == 0x17U) {
                if (req[3] != UDS_MEMORY_SELECTION_1) {
                    build_negative_response(UDS_SID_READ_DTC_INFORMATION, NRC_REQUEST_OUT_OF_RANGE,
                                           resp, resp_len);
                    return;
                }
                /* memorySelection echo precedes availability mask (Table 280) */
                resp[pos++] = req[3];
            }
            resp[pos++] = OBD2_DTC_STATUS_AVAILABILITY_MASK;
            uint8_t rec[OBD2_DTC_COUNT * 4U];
            uint8_t n = (sub == 0x0FU) ? OBD2_MirrorGetByMask(rec, OBD2_DTC_COUNT, mask)
                                       : OBD2_DtcGetByMask(rec, OBD2_DTC_COUNT, mask);
            for (uint8_t i = 0U; i < n; i++) {
                resp[pos++] = rec[i * 4U];
                resp[pos++] = rec[(uint8_t)(i * 4U + 1U)];
                resp[pos++] = rec[(uint8_t)(i * 4U + 2U)];
                resp[pos++] = rec[(uint8_t)(i * 4U + 3U)];
            }
            *resp_len = pos;
            break;
        }

        case 0x03U: {  /* reportDTCSnapshotIdentification: {DTC(3) recordNumber}* */
            for (uint8_t i = 0U; i < OBD2_DTC_COUNT; i++) {
                if (g_dtc_table[i].snapshot.valid != 0U) {
                    pos += dtc_put3(&resp[pos], g_dtc_table[i].code);
                    resp[pos++] = UDS_DTC_SNAPSHOT_RECORD;
                }
            }
            *resp_len = pos;
            break;
        }

        case 0x04U:    /* reportDTCSnapshotRecordByDTCNumber */
        case 0x18U: {  /* reportUserDefMemoryDTCSnapshotRecordByDTCNumber */
            uint8_t min_len = (sub == 0x18U) ? 7U : 6U;
            if (req_len < min_len) {
                build_negative_response(UDS_SID_READ_DTC_INFORMATION, NRC_INCORRECT_MSG_LEN,
                                       resp, resp_len);
                return;
            }
            if ((sub == 0x18U) && (req[6] != UDS_MEMORY_SELECTION_1)) {
                build_negative_response(UDS_SID_READ_DTC_INFORMATION, NRC_REQUEST_OUT_OF_RANGE,
                                       resp, resp_len);
                return;
            }
            /* [19 04|18 DTC(3) recordNumber (memorySelection)] -- recordNumber is req[5],
             * memorySelection (0x18 only) is req[6] */
            uint8_t rec_num = req[5];
            const DtcEntry_t *d = ((req[4] == 0x00U)
                                       ? OBD2_DtcFindByCode((uint16_t)((uint16_t)req[2] << 8U) | req[3])
                                       : NULL);
            if ((d == NULL) || (rec_num == 0x00U) ||
                ((rec_num != UDS_DTC_SNAPSHOT_RECORD) && (rec_num != 0xFFU))) {
                build_negative_response(UDS_SID_READ_DTC_INFORMATION, NRC_REQUEST_OUT_OF_RANGE,
                                       resp, resp_len);
                return;
            }
            if (sub == 0x18U) {
                resp[pos++] = req[6];  /* memorySelection echo */
            }
            pos += dtc_put3(&resp[pos], d->code);
            resp[pos++] = OBD2_DtcStatusByte(d);
            if (d->snapshot.valid != 0U) {
                resp[pos++] = UDS_DTC_SNAPSHOT_RECORD;   /* snapshot record number */
                resp[pos++] = 1U;                        /* numberOfIdentifiers */
                pos += dtc_put_snapshot(&resp[pos], d);
            }
            *resp_len = pos;
            break;
        }

        case 0x05U: {  /* reportDTCStoredDataByRecordNumber: {recNum DTC(3) status numIdent {DID data}}* */
            if (req_len < 3U) {
                build_negative_response(UDS_SID_READ_DTC_INFORMATION, NRC_INCORRECT_MSG_LEN,
                                       resp, resp_len);
                return;
            }
            if ((req[2] == 0x00U) ||
                ((req[2] != UDS_DTC_SNAPSHOT_RECORD) && (req[2] != 0xFFU))) {
                build_negative_response(UDS_SID_READ_DTC_INFORMATION, NRC_REQUEST_OUT_OF_RANGE,
                                       resp, resp_len);
                return;
            }
            for (uint8_t i = 0U; i < OBD2_DTC_COUNT; i++) {
                if (g_dtc_table[i].snapshot.valid != 0U) {
                    resp[pos++] = UDS_DTC_SNAPSHOT_RECORD;  /* DTCStoredDataRecordNumber */
                    pos += dtc_put3(&resp[pos], g_dtc_table[i].code);
                    resp[pos++] = OBD2_DtcStatusByte(&g_dtc_table[i]);
                    resp[pos++] = 1U;                       /* numberOfIdentifiers */
                    pos += dtc_put_snapshot(&resp[pos], &g_dtc_table[i]);
                }
            }
            *resp_len = pos;
            break;
        }

        case 0x06U:    /* reportDTCExtDataRecordByDTCNumber */
        case 0x10U:    /* reportMirrorMemoryDTCExtDataRecordByDTCNumber */
        case 0x19U: {  /* reportUserDefMemoryDTCExtDataRecordByDTCNumber */
            uint8_t min_len = (sub == 0x19U) ? 7U : 6U;
            if (req_len < min_len) {
                build_negative_response(UDS_SID_READ_DTC_INFORMATION, NRC_INCORRECT_MSG_LEN,
                                       resp, resp_len);
                return;
            }
            /* [19 06|10|19 DTC(3) extRecordNumber (memorySelection)] -- ext is req[5],
             * memorySelection (0x19 only) is req[6] */
            uint8_t ext_num = req[5];
            uint8_t mem_sel = (sub == 0x19U) ? req[6] : 0U;
            if ((sub == 0x19U) && (mem_sel != UDS_MEMORY_SELECTION_1)) {
                build_negative_response(UDS_SID_READ_DTC_INFORMATION, NRC_REQUEST_OUT_OF_RANGE,
                                       resp, resp_len);
                return;
            }
            uint16_t code = (uint16_t)((uint16_t)req[2] << 8U) | req[3];
            uint8_t occurrence = 0U;
            uint8_t status = 0U;
            if (sub == 0x10U) {
                const DtcMirrorEntry_t *m = ((req[4] == 0x00U) ? OBD2_MirrorFindByCode(code)
                                                               : NULL);
                if (m == NULL) {
                    build_negative_response(UDS_SID_READ_DTC_INFORMATION, NRC_REQUEST_OUT_OF_RANGE,
                                           resp, resp_len);
                    return;
                }
                status = m->status;
                occurrence = m->occurrence;
            } else {
                const DtcEntry_t *d = ((req[4] == 0x00U) ? OBD2_DtcFindByCode(code) : NULL);
                if (d == NULL) {
                    build_negative_response(UDS_SID_READ_DTC_INFORMATION, NRC_REQUEST_OUT_OF_RANGE,
                                           resp, resp_len);
                    return;
                }
                status = OBD2_DtcStatusByte(d);
                occurrence = d->occurrence;
            }
            if ((ext_num == 0x00U) || ((ext_num != DTC_EXT_RECORD_OCCURRENCE) &&
                                       (ext_num != 0xFFU))) {
                build_negative_response(UDS_SID_READ_DTC_INFORMATION, NRC_REQUEST_OUT_OF_RANGE,
                                       resp, resp_len);
                return;
            }
            if (sub == 0x19U) {
                resp[pos++] = mem_sel;  /* memorySelection echo */
            }
            pos += dtc_put3(&resp[pos], code);
            resp[pos++] = status;
            if (occurrence != 0U) {  /* ext record present only after first failure */
                resp[pos++] = DTC_EXT_RECORD_OCCURRENCE;
                resp[pos++] = occurrence;
            }
            *resp_len = pos;
            break;
        }

        case 0x07U: {  /* reportNumberOfDTCBySeverityMaskRecord: [severityMask, statusMask] */
            if (req_len < 4U) {
                build_negative_response(UDS_SID_READ_DTC_INFORMATION, NRC_INCORRECT_MSG_LEN,
                                       resp, resp_len);
                return;
            }
            uint8_t count = 0U;
            for (uint8_t i = 0U; i < OBD2_DTC_COUNT; i++) {
                if (((g_dtc_table[i].severity & req[2] & UDS_DTC_SEV_AVAIL_MASK) != 0U) &&
                    (dtc_mask_match(&g_dtc_table[i], req[3]) != 0U)) {
                    count++;
                }
            }
            resp[pos++] = OBD2_DTC_STATUS_AVAILABILITY_MASK;
            resp[pos++] = UDS_DTC_FMT_ISO14229;
            resp[pos++] = 0x00U;
            resp[pos++] = count;
            *resp_len = pos;
            break;
        }

        case 0x08U: {  /* reportDTCBySeverityMaskRecord: {severity funcUnit DTC(3) status}* */
            if (req_len < 4U) {
                build_negative_response(UDS_SID_READ_DTC_INFORMATION, NRC_INCORRECT_MSG_LEN,
                                       resp, resp_len);
                return;
            }
            resp[pos++] = OBD2_DTC_STATUS_AVAILABILITY_MASK;
            for (uint8_t i = 0U; i < OBD2_DTC_COUNT; i++) {
                if (((g_dtc_table[i].severity & req[2] & UDS_DTC_SEV_AVAIL_MASK) != 0U) &&
                    (dtc_mask_match(&g_dtc_table[i], req[3]) != 0U)) {
                    resp[pos++] = g_dtc_table[i].severity;
                    resp[pos++] = DTC_FUNCTIONAL_UNIT;
                    pos += dtc_put3(&resp[pos], g_dtc_table[i].code);
                    resp[pos++] = OBD2_DtcStatusByte(&g_dtc_table[i]);
                }
            }
            *resp_len = pos;
            break;
        }

        case 0x09U: {  /* reportSeverityInformationOfDTC: single DTC, fixed 9B */
            if (req_len < 5U) {
                build_negative_response(UDS_SID_READ_DTC_INFORMATION, NRC_INCORRECT_MSG_LEN,
                                       resp, resp_len);
                return;
            }
            const DtcEntry_t *d = ((req[4] == 0x00U)
                                       ? OBD2_DtcFindByCode((uint16_t)((uint16_t)req[2] << 8U) | req[3])
                                       : NULL);
            if (d == NULL) {
                build_negative_response(UDS_SID_READ_DTC_INFORMATION, NRC_REQUEST_OUT_OF_RANGE,
                                       resp, resp_len);
                return;
            }
            resp[pos++] = OBD2_DTC_STATUS_AVAILABILITY_MASK;
            resp[pos++] = d->severity;
            resp[pos++] = DTC_FUNCTIONAL_UNIT;
            pos += dtc_put3(&resp[pos], d->code);
            resp[pos++] = OBD2_DtcStatusByte(d);
            *resp_len = pos;
            break;
        }

        case 0x0AU:    /* reportSupportedDTC: all supported DTCs regardless of status */
        case 0x15U: {  /* reportDTCWithPermanentStatus */
            resp[pos++] = OBD2_DTC_STATUS_AVAILABILITY_MASK;
            for (uint8_t i = 0U; i < OBD2_DTC_COUNT; i++) {
                uint8_t include = (sub == 0x0AU) ? 1U : g_dtc_table[i].permanent;
                if (include != 0U) {
                    pos += dtc_put3(&resp[pos], g_dtc_table[i].code);
                    resp[pos++] = OBD2_DtcStatusByte(&g_dtc_table[i]);
                }
            }
            *resp_len = pos;
            break;
        }

        case 0x0BU:    /* reportFirstTestFailedDTC */
        case 0x0CU:    /* reportFirstConfirmedDTC */
        case 0x0DU:    /* reportMostRecentTestFailedDTC */
        case 0x0EU: {  /* reportMostRecentConfirmedDTC */
            uint8_t which = (sub == 0x0BU) ? OBD2_DTC_PICK_FIRST_FAILED
                          : (sub == 0x0CU) ? OBD2_DTC_PICK_FIRST_CONFIRMED
                          : (sub == 0x0DU) ? OBD2_DTC_PICK_RECENT_FAILED
                                           : OBD2_DTC_PICK_RECENT_CONFIRMED;
            resp[pos++] = OBD2_DTC_STATUS_AVAILABILITY_MASK;
            const DtcEntry_t *d = OBD2_DtcPick(which);
            if (d != NULL) {
                pos += dtc_put3(&resp[pos], d->code);
                resp[pos++] = OBD2_DtcStatusByte(d);
            }
            *resp_len = pos;
            break;
        }

        case 0x14U: {  /* reportDTCFaultDetectionCounter: {DTC(3) FDC}* prefailed only */
            for (uint8_t i = 0U; i < OBD2_DTC_COUNT; i++) {
                int8_t fdc = OBD2_DtcFdc(&g_dtc_table[i]);
                if ((fdc > 0) && (fdc < 127)) {
                    pos += dtc_put3(&resp[pos], g_dtc_table[i].code);
                    resp[pos++] = (uint8_t)fdc;
                }
            }
            *resp_len = pos;
            break;
        }

        case 0x16U: {  /* reportDTCExtDataRecordByRecordNumber: {DTC(3) status extData}* */
            if (req_len < 3U) {
                build_negative_response(UDS_SID_READ_DTC_INFORMATION, NRC_INCORRECT_MSG_LEN,
                                       resp, resp_len);
                return;
            }
            if (req[2] != DTC_EXT_RECORD_OCCURRENCE) {
                build_negative_response(UDS_SID_READ_DTC_INFORMATION, NRC_REQUEST_OUT_OF_RANGE,
                                       resp, resp_len);
                return;
            }
            resp[pos++] = req[2];  /* DTCExtDataRecordNumber echo */
            for (uint8_t i = 0U; i < OBD2_DTC_COUNT; i++) {
                if (g_dtc_table[i].occurrence != 0U) {
                    pos += dtc_put3(&resp[pos], g_dtc_table[i].code);
                    resp[pos++] = OBD2_DtcStatusByte(&g_dtc_table[i]);
                    resp[pos++] = g_dtc_table[i].occurrence;
                }
            }
            *resp_len = pos;
            break;
        }

        case 0x42U: {  /* reportWWHOBDDTCByMaskRecord: [FGID, statusMask, severityMask] */
            if (req_len < 5U) {
                build_negative_response(UDS_SID_READ_DTC_INFORMATION, NRC_INCORRECT_MSG_LEN,
                                       resp, resp_len);
                return;
            }
            if ((req[2] != UDS_FGID_EMISSIONS) && (req[2] != UDS_FGID_ALL)) {
                build_negative_response(UDS_SID_READ_DTC_INFORMATION, NRC_REQUEST_OUT_OF_RANGE,
                                       resp, resp_len);
                return;
            }
            resp[pos++] = req[2];  /* FGID echo */
            resp[pos++] = OBD2_DTC_STATUS_AVAILABILITY_MASK;
            resp[pos++] = UDS_DTC_SEV_AVAIL_MASK;
            resp[pos++] = UDS_DTC_FMT_WWH_OBD;
            for (uint8_t i = 0U; i < OBD2_DTC_COUNT; i++) {
                if (((g_dtc_table[i].severity & req[4] & UDS_DTC_SEV_AVAIL_MASK) != 0U) &&
                    (dtc_mask_match(&g_dtc_table[i], req[3]) != 0U)) {
                    resp[pos++] = g_dtc_table[i].severity;
                    pos += dtc_put3(&resp[pos], g_dtc_table[i].code);
                    resp[pos++] = OBD2_DtcStatusByte(&g_dtc_table[i]);
                }
            }
            *resp_len = pos;
            break;
        }

        case 0x55U: {  /* reportWWHOBDDTCWithPermanentStatus: [FGID] */
            if (req_len < 3U) {
                build_negative_response(UDS_SID_READ_DTC_INFORMATION, NRC_INCORRECT_MSG_LEN,
                                       resp, resp_len);
                return;
            }
            if ((req[2] != UDS_FGID_EMISSIONS) && (req[2] != UDS_FGID_ALL)) {
                build_negative_response(UDS_SID_READ_DTC_INFORMATION, NRC_REQUEST_OUT_OF_RANGE,
                                       resp, resp_len);
                return;
            }
            resp[pos++] = req[2];  /* FGID echo */
            resp[pos++] = OBD2_DTC_STATUS_AVAILABILITY_MASK;
            resp[pos++] = UDS_DTC_FMT_WWH_OBD;
            for (uint8_t i = 0U; i < OBD2_DTC_COUNT; i++) {
                if (g_dtc_table[i].permanent != 0U) {
                    pos += dtc_put3(&resp[pos], g_dtc_table[i].code);
                    resp[pos++] = OBD2_DtcStatusByte(&g_dtc_table[i]);
                }
            }
            *resp_len = pos;
            break;
        }

        default:
            build_negative_response(UDS_SID_READ_DTC_INFORMATION, NRC_SUB_FUNC_NOT_SUPPORTED,
                                   resp, resp_len);
            return;
    }
}

/* === 0x2F Virtual IO control state (simulator: flags instead of real actuators) === */
static uint8_t s_io_port = 0U;

/**
 * @brief  SID 0x2F: InputOutputControlByIdentifier
 * @note   Only DID 0x0200 (virtual IO port) supported. Extended session + SecurityAccess required.
 *         controlOption: 0x00=returnControlToECU(no-op), 0x03=shortTermAdjustment(set value).
 *         resetToDefault(0x01)/freezeCurrentState(0x02) unsupported -> NRC 0x12.
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
        case 0x00U:  /* returnControlToECU -- no autonomous actuator in simulator (no-op) */
            break;
        case 0x03U:  /* shortTermAdjustment -- tester sets value directly */
            if (req_len < 5U) {
                build_negative_response(UDS_SID_IO_CONTROL_BY_ID, NRC_INCORRECT_MSG_LEN,
                                       resp, resp_len);
                return;
            }
            s_io_port = req[4];
            break;
        default:  /* 0x01 resetToDefault / 0x02 freezeCurrentState unsupported */
            build_negative_response(UDS_SID_IO_CONTROL_BY_ID, NRC_SUB_FUNC_NOT_SUPPORTED,
                                   resp, resp_len);
            return;
    }

    resp[0] = UDS_SID_IO_CONTROL_BY_ID + UDS_RESPONSE_SID_OFFSET;  /* 0x6F */
    resp[1] = (uint8_t)(did >> 8U);
    resp[2] = (uint8_t)(did & 0xFFU);
    resp[3] = control;
    resp[4] = s_io_port;   /* current IO state */
    *resp_len = 5U;
    Debug_Print("[UDS] IOControl 0x%04X ctrl=%u val=%u\r\n", did, control, s_io_port);
}

/* === OTA transfer state (0x34/0x36/0x37) === */
static uint8_t  s_xfer_active    = 0U;   /* 1=download in progress */
static uint32_t s_xfer_addr      = 0U;   /* write start address */
static uint32_t s_xfer_size      = 0U;   /* total size */
static uint32_t s_xfer_written   = 0U;   /* bytes written */
static uint8_t  s_xfer_block_seq = 0U;   /* next expected blockSequenceCounter (1~) */

/**
 * @brief  SID 0x34: RequestDownload -- parse addr/size (ALFID), validate OTA region, erase, start transfer
 * @note   Writes to OTA data region (0x0801E000~) without bootloader. App replacement/jump
 *         is the bootloader stage.
 *         Extended session + SecurityAccess required.
 */
static void handle_request_download(const uint8_t *req, uint16_t req_len,
                                    uint8_t *resp, uint16_t *resp_len)
{
    if (req_len < 3U) {  /* SID + ALFID */
        build_negative_response(UDS_SID_REQUEST_DOWNLOAD, NRC_INCORRECT_MSG_LEN,
                               resp, resp_len);
        return;
    }

    uint8_t alfid = req[1];
    uint8_t alen = (uint8_t)(alfid & 0x0FU);
    uint8_t slen = (uint8_t)((alfid >> 4U) & 0x0FU);
    if (alen == 0U || slen == 0U || alen > 4U || slen > 4U) {
        build_negative_response(UDS_SID_REQUEST_DOWNLOAD, NRC_REQUEST_OUT_OF_RANGE,
                               resp, resp_len);
        return;
    }
    if (req_len < (uint16_t)(2U + (uint16_t)alen + (uint16_t)slen)) {
        build_negative_response(UDS_SID_REQUEST_DOWNLOAD, NRC_INCORRECT_MSG_LEN,
                               resp, resp_len);
        return;
    }

    uint32_t addr = 0U;
    uint32_t size = 0U;
    for (uint8_t i = 0U; i < alen; i++) {
        addr = (addr << 8U) | req[2U + i];
    }
    for (uint8_t i = 0U; i < slen; i++) {
        size = (size << 8U) | req[2U + alen + i];
    }

    /* Region validation: OTA data region (0x0801E000~0x0801FFFF) only */
    if ((addr < OTA_FLASH_BASE) || ((addr + size) > OTA_FLASH_END)) {
        build_negative_response(UDS_SID_REQUEST_DOWNLOAD, NRC_REQUEST_OUT_OF_RANGE,
                               resp, resp_len);
        return;
    }
    /* page-level erase */
    if (ota_flash_erase(addr, size) != 0) {
        build_negative_response(UDS_SID_REQUEST_DOWNLOAD, NRC_GENERAL_PROGRAMMING_FAILURE,
                               resp, resp_len);
        return;
    }

    s_xfer_active = 1U;
    s_xfer_addr = addr;
    s_xfer_size = size;
    s_xfer_written = 0U;
    s_xfer_block_seq = 1U;

    /* Response: [0x74, lengthFormatIdentifier, maxNumberOfBlockLength(2B big-endian)]
     * lengthFormatIdentifier low nibble = maxBlockLength length (2 bytes). maxBlockLength=248. */
    resp[0] = UDS_SID_REQUEST_DOWNLOAD + UDS_RESPONSE_SID_OFFSET;  /* 0x74 */
    resp[1] = 0x22U;
    resp[2] = 0x00U;
    resp[3] = 0xF8U;  /* maxNumberOfBlockLength = 248 */
    *resp_len = 4U;
    Debug_Print("[UDS] OTA download: addr=0x%08lX size=%lu\r\n",
                (unsigned long)addr, (unsigned long)size);
}

/**
 * @brief  SID 0x36: TransferData -- blockSequenceCounter verification, flash write
 */
static void handle_transfer_data(const uint8_t *req, uint16_t req_len,
                                 uint8_t *resp, uint16_t *resp_len)
{
    if ((s_xfer_active == 0U) || (req_len < 3U)) {  /* SID + blockSeq + data */
        build_negative_response(UDS_SID_TRANSFER_DATA, NRC_CONDITIONS_NOT_CORRECT,
                               resp, resp_len);
        return;
    }

    uint8_t seq = req[1];
    if (seq != s_xfer_block_seq) {
        build_negative_response(UDS_SID_TRANSFER_DATA, NRC_WRONG_BLOCK_SEQUENCE,
                               resp, resp_len);
        return;
    }

    uint32_t dlen = (uint32_t)(req_len - 2U);
    if ((s_xfer_written + dlen) > s_xfer_size) {
        build_negative_response(UDS_SID_TRANSFER_DATA, NRC_TRANSFER_DATA_SUSPENDED,
                               resp, resp_len);
        return;
    }
    if (ota_flash_write(s_xfer_addr + s_xfer_written, &req[2], dlen) != 0) {
        build_negative_response(UDS_SID_TRANSFER_DATA, NRC_GENERAL_PROGRAMMING_FAILURE,
                               resp, resp_len);
        return;
    }

    s_xfer_written += dlen;
    s_xfer_block_seq++;

    resp[0] = UDS_SID_TRANSFER_DATA + UDS_RESPONSE_SID_OFFSET;  /* 0x76 */
    resp[1] = seq;
    *resp_len = 2U;
}

/**
 * @brief  SID 0x37: RequestTransferExit -- transfer complete
 * @note   CRC/signature verification omitted (simulator). Production requires CRC32/signature.
 */
static void handle_request_transfer_exit(const uint8_t *req, uint16_t req_len,
                                         uint8_t *resp, uint16_t *resp_len)
{
    (void)req;
    (void)req_len;
    if (s_xfer_active == 0U) {
        build_negative_response(UDS_SID_REQUEST_TRANSFER_EXIT, NRC_CONDITIONS_NOT_CORRECT,
                               resp, resp_len);
        return;
    }
    s_xfer_active = 0U;
    resp[0] = UDS_SID_REQUEST_TRANSFER_EXIT + UDS_RESPONSE_SID_OFFSET;  /* 0x77 */
    *resp_len = 1U;
    Debug_Print("[UDS] OTA exit: written=%lu/%lu\r\n",
                (unsigned long)s_xfer_written, (unsigned long)s_xfer_size);
}

/**
 * @brief  SID 0x01: OBD-II Mode 01 bridge
 * @note   Calls pure-logic handler OBD2_HandleService01() from UDS pipeline.
 *         CAN I/O is handled by ISO-TP, so only logic is reused here.
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

    /* Call existing OBD-II handler (pure logic, no CAN I/O) */
    uint8_t obd_resp[8];
    uint8_t obd_len = OBD2_HandleService01(pid, obd_resp);

    if (obd_len == 0U) {
        *resp_len = 0U;  /* unsupported PID -> no response */
        return;
    }

    /*
     * OBD-II response: [len, 0x41, PID, data...]
     * UDS manages length via ISO-TP, so strip len byte
     */
    (void)memcpy(resp, &obd_resp[1], (uint16_t)obd_len);
    *resp_len = (uint16_t)obd_len;
}

/**
 * @brief  Mode 0x03: Read stored DTCs
 * @note   ISO 15031-5 response: [0x43, numDTC, (DTC 2 bytes each)...]
 *         Simulator has no DTCs, so responds with empty list (numDTC=0).
 */
static void handle_obd2_service03(const uint8_t *req, uint16_t req_len,
                                  uint8_t *resp, uint16_t *resp_len)
{
    (void)req;
    (void)req_len;
    /* ISO 15031-5: [0x43, numDTC, (DTC_hi, DTC_lo)...]
     * Return CONFIRMED DTC list (FaultManager). */
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
 * @brief  Mode 0x04: Clear DTCs
 * @note   ISO 15031-5 response: [0x44]. (No DTCs to clear in simulator)
 */
static void handle_obd2_service04(const uint8_t *req, uint16_t req_len,
                                  uint8_t *resp, uint16_t *resp_len)
{
    (void)req;
    (void)req_len;
    OBD2_DtcClear();  /* Reset all DTCs (stored + pending) */
    resp[0] = UDS_SID_OBD2_CLEAR_DTC + UDS_RESPONSE_SID_OFFSET;  /* 0x44 */
    *resp_len = 1U;
}

/**
 * @brief  Mode 0x07: Read pending DTCs
 * @note   ISO 15031-5 response: [0x47, (DTC 2 bytes each)...] -- unlike Mode 03,
 *         there is no numDTC byte. Empty -> [0x47] only.
 */
static void handle_obd2_service07(const uint8_t *req, uint16_t req_len,
                                  uint8_t *resp, uint16_t *resp_len)
{
    (void)req;
    (void)req_len;
    /* ISO 15031-5: [0x47, (DTC_hi, DTC_lo)...] -- no numDTC byte.
     * Return PENDING DTC list (FaultManager). */
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
 * @brief  Mode 0x09: Vehicle information
 * @note   INF type 0x00 = supported INF list, 0x02 = VIN (ISO 15031-5).
 *         Unsupported INF types get no response per OBD-II standard (no 0x7F negative response).
 *         VIN response (20 bytes) does not fit in obd_resp[8], so written directly to UDS response buffer.
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
            /* Supported INF bitmap: INF 0x02 (VIN) supported -> bit1 = 0x02 */
            resp[0] = UDS_SID_OBD2_VEHICLE_INFO + UDS_RESPONSE_SID_OFFSET;  /* 0x49 */
            resp[1] = 0x00U;
            resp[2] = 0x02U;  /* INF 0x02 (VIN) supported */
            resp[3] = 0x00U;
            resp[4] = 0x00U;
            resp[5] = 0x00U;
            *resp_len = 6U;
            break;

        case 0x02U: {
            /* VIN: [0x49, 0x02, num=1, VIN(17)] = 20 bytes (CAN-FD SF) */
            size_t vin_len = strlen(s_vin);  /* VIN = ISO 3779 standard 17 characters */
            resp[0] = UDS_SID_OBD2_VEHICLE_INFO + UDS_RESPONSE_SID_OFFSET;  /* 0x49 */
            resp[1] = 0x02U;
            resp[2] = 0x01U;  /* number of VINs provided = 1 */
            (void)memcpy(&resp[3], s_vin, vin_len);
            *resp_len = (uint16_t)(3U + vin_len);
            break;
        }

        default:
            /* Unsupported INF -> no response */
            *resp_len = 0U;
            break;
    }
}

/**
 * @brief  Build negative response: [0x7F, SID, NRC]
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
