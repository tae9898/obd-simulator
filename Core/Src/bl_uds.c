/**
 * @file    bl_uds.c
 * @brief   Bootloader UDS flash server (programming-session dispatcher)
 * @note    Bare-metal, BL_BUILD only. Provides UDS_DispatchRequest with the
 *          same signature the app's uds_service.c uses (iso_tp.c:668 calls it),
 *          so iso_tp.c links unchanged.
 *
 *          Supported: 0x10/0x11/0x27/0x28/0x31/0x34/0x36/0x37/0x3E/0x85
 *          Gating: 0x34/0x36/0x37/0x31 require PROGRAMMING session + security
 *          unlocked (inline check -- the shared k_access_rules table in
 *          diag_session.c is Extended-session based, which is inverted for a
 *          bootloader; NRC split as in the app: 0x7F wrong session / 0x33 locked).
 */

#include "uds_service.h"
#include "diag_session.h"
#include "bl_handoff.h"
#include "ota_flash.h"
#include "uart_debug.h"
#include "iso_tp.h"       /* ISO_TP_SendResponse: NRC 0x78 before the 0x34 erase */
#include <string.h>

/* bl_boot.c */
uint32_t bl_crc32(const uint8_t *data, uint32_t len);
uint32_t bl_meta_rollback_counter(void);
int bl_meta_write(const bl_meta_t *m);

/* === Transfer state (mirrors the app's s_xfer_*) === */
static uint8_t  s_xfer_active    = 0U;
static uint8_t  s_xfer_closed    = 0U;   /* 0x37 received */
static uint32_t s_xfer_size      = 0U;
static uint32_t s_xfer_written   = 0U;
static uint8_t  s_xfer_block_seq = 0U;
static uint32_t s_saved_rollback = 0U;   /* meta counter read at 0x34 (page erased there) */

/* 0x11 reset handshake: response must fully drain before NVIC_SystemReset */
volatile uint8_t bl_reset_pending = 0U;

/* Drop an in-flight transfer (interrupted update). Called when the session
 * returns to Default (S3 timeout or 0x10 01/03): without this, a stale
 * s_xfer_* survives in RAM and later 0x36s answer 0x73 instead of 0x22. */
void BL_UDS_ResetTransfer(void)
{
    if (s_xfer_active != 0U) {
        Debug_Print("[BL-UDS] Transfer state dropped (session -> Default)\r\n");
    }
    s_xfer_active    = 0U;
    s_xfer_closed    = 0U;
    s_xfer_size      = 0U;
    s_xfer_written   = 0U;
    s_xfer_block_seq = 0U;
}

/* === Helpers === */

static void nrc(uint8_t sid, uint8_t code, uint8_t *resp, uint16_t *resp_len)
{
    resp[0] = 0x7FU;
    resp[1] = sid;
    resp[2] = code;
    *resp_len = 3U;
}

/** PROGRAMMING session + security unlocked, else the ISO 14229-1 NRC split:
 *  wrong session -> 0x7F, session ok but locked -> 0x33 */
static int gate_flash_services(uint8_t sid, uint8_t *resp, uint16_t *resp_len)
{
    if (DiagSession_Get() != DIAG_SESSION_PROGRAMMING) {
        nrc(sid, NRC_SERVICE_NOT_SUPPORTED_IN_ACTIVE_SESSION, resp, resp_len);
        return -1;
    }
    if (DiagSession_SecurityUnlocked() == 0) {
        nrc(sid, NRC_SECURITY_ACCESS_DENIED, resp, resp_len);
        return -1;
    }
    return 0;
}

/* === Handlers === */

static void handle_session_control(const uint8_t *req, uint16_t req_len,
                                   uint8_t *resp, uint16_t *resp_len)
{
    if (req_len < 2U) {
        nrc(UDS_SID_DIAG_SESSION_CTRL, NRC_INCORRECT_MSG_LEN, resp, resp_len);
        return;
    }

    uint8_t sub = (uint8_t)(req[1] & 0x7FU);
    if (sub < 0x01U || sub > 0x03U) {
        nrc(UDS_SID_DIAG_SESSION_CTRL, NRC_SUB_FUNC_NOT_SUPPORTED, resp, resp_len);
        return;
    }
    if (DiagSession_SetSession(sub) != 0) {
        nrc(UDS_SID_DIAG_SESSION_CTRL, NRC_CONDITIONS_NOT_CORRECT, resp, resp_len);
        return;
    }

    uint16_t p2  = UDS_P2_SERVER_MAX_MS;
    uint16_t p2s = UDS_P2_STAR_SERVER_MAX_MS / 10U;
    resp[0] = UDS_SID_DIAG_SESSION_CTRL + UDS_RESPONSE_SID_OFFSET;
    resp[1] = sub;
    resp[2] = (uint8_t)(p2  >> 8U);
    resp[3] = (uint8_t)(p2  & 0xFFU);
    resp[4] = (uint8_t)(p2s >> 8U);
    resp[5] = (uint8_t)(p2s & 0xFFU);
    *resp_len = 6U;
    Debug_Print("[BL-UDS] Session -> 0x%02X\r\n", sub);
}

static void handle_ecu_reset(const uint8_t *req, uint16_t req_len,
                             uint8_t *resp, uint16_t *resp_len)
{
    if (req_len < 2U) {
        nrc(UDS_SID_ECU_RESET, NRC_INCORRECT_MSG_LEN, resp, resp_len);
        return;
    }

    uint8_t sub = (uint8_t)(req[1] & 0x7FU);
    if (sub != UDS_RESET_HARD && sub != UDS_RESET_SOFT) {
        nrc(UDS_SID_ECU_RESET, NRC_SUB_FUNC_NOT_SUPPORTED, resp, resp_len);
        return;
    }

    resp[0] = UDS_SID_ECU_RESET + UDS_RESPONSE_SID_OFFSET;
    resp[1] = sub;
    *resp_len = 2U;
    bl_reset_pending = 1U;   /* bl_main waits ~200ms (TX drain) then resets */
    Debug_Print("[BL-UDS] Reset requested\r\n");
}

static void handle_security_access(const uint8_t *req, uint16_t req_len,
                                   uint8_t *resp, uint16_t *resp_len)
{
    if (req_len < 2U) {
        nrc(UDS_SID_SECURITY_ACCESS, NRC_INCORRECT_MSG_LEN, resp, resp_len);
        return;
    }

    uint8_t level = (uint8_t)(req[1] & 0x7FU);

    if ((level & 0x01U) != 0U) {
        /* Request seed */
        switch (DiagSession_CheckSecurityGate()) {
            case DIAG_SEC_GATE_DELAY:
                nrc(UDS_SID_SECURITY_ACCESS, NRC_REQUIRED_TIME_DELAY, resp, resp_len);
                return;
            case DIAG_SEC_GATE_LOCKED:
                nrc(UDS_SID_SECURITY_ACCESS, NRC_EXCEEDED_ATTEMPTS, resp, resp_len);
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
    } else {
        /* Send key */
        if (req_len < 4U) {
            nrc(UDS_SID_SECURITY_ACCESS, NRC_INCORRECT_MSG_LEN, resp, resp_len);
            return;
        }
        uint16_t key = (uint16_t)((uint16_t)req[2] << 8U) | (uint16_t)req[3];
        switch (DiagSession_VerifyKey(key)) {
            case DIAG_KEY_OK:
                resp[0] = UDS_SID_SECURITY_ACCESS + UDS_RESPONSE_SID_OFFSET;
                resp[1] = level;
                *resp_len = 2U;
                Debug_Print("[BL-UDS] Unlocked\r\n");
                break;
            case DIAG_KEY_EXCEEDED_ATTEMPTS:
                nrc(UDS_SID_SECURITY_ACCESS, NRC_EXCEEDED_ATTEMPTS, resp, resp_len);
                break;
            case DIAG_KEY_DELAY_NOT_EXPIRED:
                nrc(UDS_SID_SECURITY_ACCESS, NRC_REQUIRED_TIME_DELAY, resp, resp_len);
                break;
            default:
                nrc(UDS_SID_SECURITY_ACCESS, NRC_INVALID_KEY, resp, resp_len);
                break;
        }
    }
}

static void handle_tester_present(const uint8_t *req, uint16_t req_len,
                                  uint8_t *resp, uint16_t *resp_len)
{
    if (req_len < 2U) {
        nrc(UDS_SID_TESTER_PRESENT, NRC_INCORRECT_MSG_LEN, resp, resp_len);
        return;
    }
    uint8_t sub = (uint8_t)(req[1] & 0x7FU);
    if (sub != 0x00U) {
        nrc(UDS_SID_TESTER_PRESENT, NRC_SUB_FUNC_NOT_SUPPORTED, resp, resp_len);
        return;
    }
    resp[0] = UDS_SID_TESTER_PRESENT + UDS_RESPONSE_SID_OFFSET;
    resp[1] = sub;
    *resp_len = 2U;
}

/* 0x28 CommunicationControl / 0x85 ControlDTCSetting: no-op in the BL.
 * NOTE: BL has no other communication or DTC machinery to disable;
 * accept and flag only -- if a BL-side comm gate is ever needed, hook here. */
static void handle_noop_control(uint8_t sid, const uint8_t *req, uint16_t req_len,
                                uint8_t *resp, uint16_t *resp_len)
{
    if (req_len < 2U) {
        nrc(sid, NRC_INCORRECT_MSG_LEN, resp, resp_len);
        return;
    }
    uint8_t sub = (uint8_t)(req[1] & 0x7FU);
    resp[0] = (uint8_t)(sid + UDS_RESPONSE_SID_OFFSET);
    resp[1] = sub;
    *resp_len = 2U;
}

/**
 * @brief SID 0x34 RequestDownload: mark meta UPDATING, erase app slot
 * @note   Meta is invalidated FIRST: any power loss from here until the 0x31
 *         VALID write leaves the boot gate in "stay in BL" state.
 */
static void handle_request_download(const uint8_t *req, uint16_t req_len,
                                    uint8_t *resp, uint16_t *resp_len)
{
    if (req_len < 3U) {
        nrc(UDS_SID_REQUEST_DOWNLOAD, NRC_INCORRECT_MSG_LEN, resp, resp_len);
        return;
    }

    uint8_t alfid = req[1];
    uint8_t alen = (uint8_t)(alfid & 0x0FU);
    uint8_t slen = (uint8_t)((alfid >> 4U) & 0x0FU);
    if (alen == 0U || slen == 0U || alen > 4U || slen > 4U) {
        nrc(UDS_SID_REQUEST_DOWNLOAD, NRC_REQUEST_OUT_OF_RANGE, resp, resp_len);
        return;
    }
    if (req_len < (uint16_t)(2U + (uint16_t)alen + (uint16_t)slen)) {
        nrc(UDS_SID_REQUEST_DOWNLOAD, NRC_INCORRECT_MSG_LEN, resp, resp_len);
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

    /* Only the app slot, whole-image downloads (same format the app had) */
    if ((addr != APP_BASE) || (size == 0U) || (size > APP_SLOT_SIZE)) {
        nrc(UDS_SID_REQUEST_DOWNLOAD, NRC_REQUEST_OUT_OF_RANGE, resp, resp_len);
        return;
    }

    /* The meta write + erase below run up to ~1s -- far beyond P2 (50ms).
     * Send NRC 0x78 first so the tester switches to its P2* (5s) budget
     * before the bus goes quiet; the final 0x74 (or an NRC) follows after. */
    {
        const uint8_t pending[3] = {0x7FU, UDS_SID_REQUEST_DOWNLOAD, NRC_RESPONSE_PENDING};
        ISO_TP_SendResponse(CAN_ID_PHYSICAL_RESP, pending, 3U);
    }

    /* 1. Save the rollback counter, then invalidate meta (UPDATING marker) */
    s_saved_rollback = bl_meta_rollback_counter();
    {
        bl_meta_t upd;
        (void)memset(&upd, 0xFF, sizeof(upd));
        upd.magic   = BL_META_MAGIC;
        upd.state   = BL_META_STATE_UPDATING;
        upd.version = s_saved_rollback;   /* carry counter across the erase */
        upd.size    = 0U;
        upd.crc32   = 0U;
        if (bl_meta_write(&upd) != 0) {
            nrc(UDS_SID_REQUEST_DOWNLOAD, NRC_GENERAL_PROGRAMMING_FAILURE, resp, resp_len);
            return;
        }
    }

    /* 2. Erase the app slot (up to 46 pages, ~1s with IRQs off) */
    if (ota_flash_erase(APP_BASE, size) != 0) {
        nrc(UDS_SID_REQUEST_DOWNLOAD, NRC_GENERAL_PROGRAMMING_FAILURE, resp, resp_len);
        return;
    }

    s_xfer_active    = 1U;
    s_xfer_closed    = 0U;
    s_xfer_size      = size;
    s_xfer_written   = 0U;
    s_xfer_block_seq = 1U;

    resp[0] = UDS_SID_REQUEST_DOWNLOAD + UDS_RESPONSE_SID_OFFSET;  /* 0x74 */
    resp[1] = 0x22U;   /* maxNumberOfBlockLength = 2 bytes */
    resp[2] = 0x00U;
    resp[3] = 0xF8U;   /* 248 */
    *resp_len = 4U;
    Debug_Print("[BL-UDS] Download: addr=0x%08lX size=%lu (rollback floor v%lu)\r\n",
                (unsigned long)addr, (unsigned long)size,
                (unsigned long)s_saved_rollback);
}

static void handle_transfer_data(const uint8_t *req, uint16_t req_len,
                                 uint8_t *resp, uint16_t *resp_len)
{
    if ((s_xfer_active == 0U) || (s_xfer_closed != 0U) || (req_len < 3U)) {
        nrc(UDS_SID_TRANSFER_DATA, NRC_CONDITIONS_NOT_CORRECT, resp, resp_len);
        return;
    }

    uint8_t seq = req[1];
    if (seq != s_xfer_block_seq) {
        nrc(UDS_SID_TRANSFER_DATA, NRC_WRONG_BLOCK_SEQUENCE, resp, resp_len);
        return;
    }

    uint32_t dlen = (uint32_t)(req_len - 2U);
    if ((s_xfer_written + dlen) > s_xfer_size) {
        nrc(UDS_SID_TRANSFER_DATA, NRC_TRANSFER_DATA_SUSPENDED, resp, resp_len);
        return;
    }
    if (ota_flash_write(APP_BASE + s_xfer_written, &req[2], dlen) != 0) {
        nrc(UDS_SID_TRANSFER_DATA, NRC_GENERAL_PROGRAMMING_FAILURE, resp, resp_len);
        return;
    }

    s_xfer_written += dlen;
    s_xfer_block_seq++;

    resp[0] = UDS_SID_TRANSFER_DATA + UDS_RESPONSE_SID_OFFSET;  /* 0x76 */
    resp[1] = seq;
    *resp_len = 2U;
}

static void handle_request_transfer_exit(const uint8_t *req, uint16_t req_len,
                                         uint8_t *resp, uint16_t *resp_len)
{
    (void)req;
    if ((s_xfer_active == 0U) || (s_xfer_closed != 0U) || (req_len < 1U)) {
        nrc(UDS_SID_REQUEST_TRANSFER_EXIT, NRC_CONDITIONS_NOT_CORRECT, resp, resp_len);
        return;
    }
    s_xfer_closed = 1U;
    resp[0] = UDS_SID_REQUEST_TRANSFER_EXIT + UDS_RESPONSE_SID_OFFSET;  /* 0x77 */
    *resp_len = 1U;
    Debug_Print("[BL-UDS] Transfer exit: written=%lu/%lu\r\n",
                (unsigned long)s_xfer_written, (unsigned long)s_xfer_size);
}

/**
 * @brief SID 0x31 RoutineControl 0x0203 CheckMemory:
 *        CRC32 over the written image, anti-rollback gate, meta VALID write.
 * @note   Request  [0x31 01 02 03 version(4B BE) expected_crc32(4B BE)]
 *          - version doubles as the anti-rollback counter
 *          - expected_crc32 = binascii.crc32(app_image_bytes)
 *         Response [0x71 01 02 03 crc32(4B BE)] (computed CRC)
 *          - CRC mismatch  -> NRC 0x72 (meta stays UPDATING: brick-safe)
 *          - version < saved counter -> NRC 0x31
 */
static void handle_routine_control(const uint8_t *req, uint16_t req_len,
                                   uint8_t *resp, uint16_t *resp_len)
{
    if (req_len < 4U) {
        nrc(UDS_SID_ROUTINE_CONTROL, NRC_INCORRECT_MSG_LEN, resp, resp_len);
        return;
    }

    uint8_t sub = (uint8_t)(req[1] & 0x7FU);
    uint16_t routine = (uint16_t)((req[2] << 8U) | req[3]);

    if (routine != 0x0203U) {
        nrc(UDS_SID_ROUTINE_CONTROL, NRC_REQUEST_OUT_OF_RANGE, resp, resp_len);
        return;
    }
    if (sub != 0x01U) {
        nrc(UDS_SID_ROUTINE_CONTROL, NRC_SUB_FUNC_NOT_SUPPORTED, resp, resp_len);
        return;
    }
    if ((s_xfer_active == 0U) || (s_xfer_closed == 0U) ||
        (req_len != 12U) || (s_xfer_written == 0U)) {
        nrc(UDS_SID_ROUTINE_CONTROL, NRC_CONDITIONS_NOT_CORRECT, resp, resp_len);
        return;
    }

    uint32_t version = ((uint32_t)req[4] << 24) | ((uint32_t)req[5] << 16) |
                       ((uint32_t)req[6] << 8)  |  (uint32_t)req[7];
    uint32_t expected = ((uint32_t)req[8] << 24) | ((uint32_t)req[9] << 16) |
                        ((uint32_t)req[10] << 8) |  (uint32_t)req[11];

    /* Anti-rollback: version must not go backwards */
    if (version < s_saved_rollback) {
        nrc(UDS_SID_ROUTINE_CONTROL, NRC_REQUEST_OUT_OF_RANGE, resp, resp_len);
        Debug_Print("[BL-UDS] Anti-rollback: v%lu < floor v%lu\r\n",
                    (unsigned long)version, (unsigned long)s_saved_rollback);
        return;
    }

    uint32_t crc = bl_crc32((const uint8_t *)APP_BASE, s_xfer_written);
    if (crc != expected) {
        nrc(UDS_SID_ROUTINE_CONTROL, NRC_GENERAL_PROGRAMMING_FAILURE, resp, resp_len);
        Debug_Print("[BL-UDS] CRC mismatch: got=%08lX exp=%08lX\r\n",
                    (unsigned long)crc, (unsigned long)expected);
        return;
    }

    bl_meta_t valid;
    (void)memset(&valid, 0xFF, sizeof(valid));
    valid.magic   = BL_META_MAGIC;
    valid.state   = BL_META_STATE_VALID;
    valid.version = version;
    valid.size    = s_xfer_written;
    valid.crc32   = crc;
    if (bl_meta_write(&valid) != 0) {
        nrc(UDS_SID_ROUTINE_CONTROL, NRC_GENERAL_PROGRAMMING_FAILURE, resp, resp_len);
        return;
    }

    s_xfer_active = 0U;   /* consume: next cycle needs a fresh 0x34 */

    resp[0] = UDS_SID_ROUTINE_CONTROL + UDS_RESPONSE_SID_OFFSET;  /* 0x71 */
    resp[1] = sub;
    resp[2] = 0x02U;
    resp[3] = 0x03U;
    resp[4] = (uint8_t)(crc >> 24);
    resp[5] = (uint8_t)(crc >> 16);
    resp[6] = (uint8_t)(crc >> 8);
    resp[7] = (uint8_t)(crc);
    *resp_len = 8U;
    Debug_Print("[BL-UDS] CheckMemory OK: v%lu %luB crc=%08lX (meta VALID)\r\n",
                (unsigned long)version, (unsigned long)s_xfer_written,
                (unsigned long)crc);
}

/* === Dispatcher (signature shared with app uds_service.c) === */

void UDS_DispatchRequest(const uint8_t *request, uint16_t request_len,
                         AddrType_t addr,
                         uint8_t *response, uint16_t *response_len)
{
    if (request == NULL || request_len == 0U ||
        response == NULL || response_len == NULL) {
        return;
    }

    uint8_t sid = request[0];

    /* Functional request (0x7DF): respond only to the BL's service subset
     * (0x3E keep-alive + 0x10/0x11 are the realistic broadcast cases). */
    if (addr == ADDR_FUNCTIONAL) {
        switch (sid) {
            case UDS_SID_TESTER_PRESENT:
            case UDS_SID_DIAG_SESSION_CTRL:
            case UDS_SID_ECU_RESET:
                break;
            default:
                *response_len = 0U;
                return;
        }
    }

    DiagSession_ResetS3Timeout();

    switch (sid) {
        case UDS_SID_DIAG_SESSION_CTRL:
            handle_session_control(request, request_len, response, response_len);
            break;

        case UDS_SID_ECU_RESET:
            handle_ecu_reset(request, request_len, response, response_len);
            break;

        case UDS_SID_SECURITY_ACCESS:
            handle_security_access(request, request_len, response, response_len);
            break;

        case UDS_SID_TESTER_PRESENT:
            handle_tester_present(request, request_len, response, response_len);
            break;

        case UDS_SID_COMMUNICATION_CONTROL:   /* no-op in BL (see handler) */
        case UDS_SID_CONTROL_DTC_SETTING:
            handle_noop_control(sid, request, request_len, response, response_len);
            break;

        case UDS_SID_REQUEST_DOWNLOAD:
            if (gate_flash_services(sid, response, response_len) == 0) {
                handle_request_download(request, request_len, response, response_len);
            }
            break;

        case UDS_SID_TRANSFER_DATA:
            if (gate_flash_services(sid, response, response_len) == 0) {
                handle_transfer_data(request, request_len, response, response_len);
            }
            break;

        case UDS_SID_REQUEST_TRANSFER_EXIT:
            if (gate_flash_services(sid, response, response_len) == 0) {
                handle_request_transfer_exit(request, request_len, response, response_len);
            }
            break;

        case UDS_SID_ROUTINE_CONTROL:
            if (gate_flash_services(sid, response, response_len) == 0) {
                handle_routine_control(request, request_len, response, response_len);
            }
            break;

        default:
            nrc(sid, NRC_SERVICE_NOT_SUPPORTED, response, response_len);
            break;
    }

    /* SuppressPosRspMsgIndicationBit -- same rule as the app dispatcher */
    if ((*response_len > 0U) && (response[0] != 0x7FU) &&
        (request_len >= 2U) && ((request[1] & 0x80U) != 0U)) {
        switch (sid) {
            case UDS_SID_DIAG_SESSION_CTRL:
            case UDS_SID_ECU_RESET:
            case UDS_SID_SECURITY_ACCESS:
            case UDS_SID_ROUTINE_CONTROL:
            case UDS_SID_TESTER_PRESENT:
            case UDS_SID_CONTROL_DTC_SETTING:
                *response_len = 0U;
                break;
            default:
                break;
        }
    }
}
