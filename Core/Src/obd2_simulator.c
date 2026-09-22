/**
 * @file    obd2_simulator.c
 * @brief   OBD-II ECU simulator implementation
 * @note    CAN request parsing, PID-specific response generation, periodic simulation value update
 */

#include "obd2_simulator.h"
#include "task.h"            /* taskENTER/EXIT_CRITICAL (DTC table cross-task protection) */
#include <string.h>

/* === Simulation state global variables (defined in main.c) === */

/**
 * @brief  OBD-II Mode 01 pure logic handler
 * @param  pid:     requested PID
 * @param  pTxData: response buffer (minimum 8 bytes)
 * @retval response length (0 = unsupported PID)
 * @note   No CAN I/O. Called from UDS dispatcher.
 */
uint8_t OBD2_HandleService01(uint8_t pid, uint8_t *pTxData)
{
    if (pTxData == NULL) {
        return 0U;
    }

    (void)memset(pTxData, 0, 8);

    switch (pid) {
        case OBD2_PID_SUPPORTED_PIDS:
            return OBD2_GetSupportedPIDs(pTxData);
        case OBD2_PID_COOLANT_TEMP:
            return OBD2_GetCoolantTemp(pTxData, g_sim_state.coolant_temp);
        case OBD2_PID_ENGINE_RPM:
            return OBD2_GetEngineRPM(pTxData, g_sim_state.engine_rpm);
        case OBD2_PID_VEHICLE_SPEED:
            return OBD2_GetVehicleSpeed(pTxData, g_sim_state.vehicle_speed);
        default:
            return 0U;
    }
}

/**
 * @brief  Periodically update simulation state values
 * @param  pState: simulation state structure
 *
 * @note   Called every 10ms:
 *         - RPM: 800~4000 RPM ramp up/down cycle (10 RPM/10ms = 1000 RPM/s)
 *         - Coolant temperature: 80~105C gradual change (0.1C/10ms = 10C/s)
 *         - Vehicle speed: 0~120 km/h gradual change (1 km/h/10ms = 100 km/h/s)
 */
void OBD2_UpdateSimValues(OBD2_SimState_t *pState)
{
    /* --- Engine RPM ramp simulation --- */
    if (pState->rpm_direction == 0U) {
        /* Ramp up: increase RPM */
        pState->engine_rpm += RPM_RAMP_STEP;
        if (pState->engine_rpm >= RPM_MAX) {
            pState->engine_rpm = RPM_MAX;
            pState->rpm_direction = 1U;  /* Direction switch: ramp down */
        }
    } else {
        /* Ramp down: decrease RPM */
        if (pState->engine_rpm > RPM_IDLE) {
            if (pState->engine_rpm < (RPM_IDLE + RPM_RAMP_STEP)) {
                pState->engine_rpm = RPM_IDLE;
            } else {
                pState->engine_rpm -= RPM_RAMP_STEP;
            }
        }
        if (pState->engine_rpm <= RPM_IDLE) {
            pState->engine_rpm = RPM_IDLE;
            pState->rpm_direction = 0U;  /* Direction switch: ramp up */
        }
    }

    /* --- Coolant temperature gradual change --- */
    if (pState->temp_direction == 0U) {
        /* Temperature increase */
        pState->coolant_temp += COOLANT_TEMP_STEP;
        if (pState->coolant_temp >= COOLANT_TEMP_MAX) {
            pState->coolant_temp = COOLANT_TEMP_MAX;
            pState->temp_direction = 1U;
        }
    } else {
        /* Temperature decrease */
        if (pState->coolant_temp > COOLANT_TEMP_MIN) {
            if (pState->coolant_temp < (COOLANT_TEMP_MIN + COOLANT_TEMP_STEP)) {
                pState->coolant_temp = COOLANT_TEMP_MIN;
            } else {
                pState->coolant_temp -= COOLANT_TEMP_STEP;
            }
        }
        if (pState->coolant_temp <= COOLANT_TEMP_MIN) {
            pState->coolant_temp = COOLANT_TEMP_MIN;
            pState->temp_direction = 0U;
        }
    }

    /* --- Vehicle speed gradual change --- */
    if (pState->speed_direction == 0U) {
        /* Speed increase */
        pState->vehicle_speed += VEHICLE_SPEED_STEP;
        if (pState->vehicle_speed >= VEHICLE_SPEED_MAX) {
            pState->vehicle_speed = VEHICLE_SPEED_MAX;
            pState->speed_direction = 1U;
        }
    } else {
        /* Speed decrease */
        if (pState->vehicle_speed > VEHICLE_SPEED_MIN) {
            pState->vehicle_speed -= VEHICLE_SPEED_STEP;
        }
        if (pState->vehicle_speed <= VEHICLE_SPEED_MIN) {
            pState->vehicle_speed = VEHICLE_SPEED_MIN;
            pState->speed_direction = 0U;
        }
    }
}

/**
 * @brief  PID 0x00: Generate supported PID bitmap response
 *
 * @note   Bitmap format: each bit indicates support for the corresponding PID
 *         Byte 4 (PID 0x00~0x07): Bit0=PID01, Bit1=PID02, ...
 *         Byte 5 (PID 0x08~0x0F): Bit0=PID09, Bit1=PID0A, ...
 *         Byte 6 (PID 0x10~0x17): Bit0=PID11, ...
 *         Byte 7 (PID 0x18~0x1F): Bit0=PID19, ...
 *
 *         Supported PIDs: 0x05, 0x0C, 0x0D
 *         Byte 4: Bit4(0x05) -> 0x10
 *         Byte 5: Bit4(0x0C), Bit5(0x0D) -> 0x18
 *         Byte 6: 0x00
 *         Byte 7: 0x00
 */
uint8_t OBD2_GetSupportedPIDs(uint8_t *pTxData)
{
    /* Response: [len, 0x41, 0x00, bitmap4, bitmap5, 0x00, 0x00, 0x00] */
    pTxData[0] = 6U;  /* ISO-TP Single Frame: lower nibble = payload length */
    pTxData[1] = OBD2_MODE_RESPONSE_PREFIX + OBD2_MODE_CURRENT_DATA;  /* 0x41 */
    pTxData[2] = OBD2_PID_SUPPORTED_PIDS;                              /* 0x00 */

    /* PID 0x01~0x07 bitmap: PID 0x05(bit4) supported */
    pTxData[3] = (1U << 4);  /* 0x10 */

    /* PID 0x09~0x0F bitmap: PID 0x0C(bit3), PID 0x0D(bit4) supported */
    pTxData[4] = (1U << 3) | (1U << 4);  /* 0x18 */

    /* PID 0x11~0x17, 0x19~0x1F bitmap: none supported */
    pTxData[5] = 0x00;
    pTxData[6] = 0x00;
    pTxData[7] = 0x00;

    return 6U;  /* Actual transmit DLC */
}

/**
 * @brief  PID 0x05: Generate coolant temperature response
 * @param  pTxData: transmit data buffer
 * @param  temp:    coolant temperature (Celsius, e.g. 90)
 *
 * @note   OBD-II encoding: A = temp + 40 (e.g. 90C -> A = 130 = 0x82)
 *         Response: [03, 0x41, 0x05, A, 00, 00, 00, 00]
 */
uint8_t OBD2_GetCoolantTemp(uint8_t *pTxData, uint8_t temp)
{
    uint8_t encoded = (uint8_t)(temp + 40U);

    pTxData[0] = 3U;  /* ISO-TP Single Frame: 3-byte payload */
    pTxData[1] = OBD2_MODE_RESPONSE_PREFIX + OBD2_MODE_CURRENT_DATA;  /* 0x41 */
    pTxData[2] = OBD2_PID_COOLANT_TEMP;                                /* 0x05 */
    pTxData[3] = encoded;
    pTxData[4] = 0x00;
    pTxData[5] = 0x00;
    pTxData[6] = 0x00;
    pTxData[7] = 0x00;

    return 3U;
}

/**
 * @brief  PID 0x0C: Generate engine RPM response
 * @param  pTxData: transmit data buffer
 * @param  rpm:     engine RPM (e.g. 2500)
 *
 * @note   OBD-II encoding: ((A*256) + B) / 4 = RPM
 *         Therefore: raw_value = RPM * 4 (e.g. 2500 * 4 = 10000 = 0x2710)
 *         A = raw_value >> 8, B = raw_value & 0xFF
 *         Response: [04, 0x41, 0x0C, A, B, 00, 00, 00]
 */
uint8_t OBD2_GetEngineRPM(uint8_t *pTxData, uint16_t rpm)
{
    uint32_t raw_value = (uint32_t)rpm * 4U;

    pTxData[0] = 4U;  /* ISO-TP Single Frame: 4-byte payload */
    pTxData[1] = OBD2_MODE_RESPONSE_PREFIX + OBD2_MODE_CURRENT_DATA;  /* 0x41 */
    pTxData[2] = OBD2_PID_ENGINE_RPM;                                  /* 0x0C */
    pTxData[3] = (uint8_t)(raw_value >> 8U);  /* High byte */
    pTxData[4] = (uint8_t)(raw_value & 0xFFU); /* Low byte */
    pTxData[5] = 0x00;
    pTxData[6] = 0x00;
    pTxData[7] = 0x00;

    return 4U;
}

/**
 * @brief  PID 0x0D: Generate vehicle speed response
 * @param  pTxData: transmit data buffer
 * @param  speed:   vehicle speed (km/h, e.g. 60)
 *
 * @note   OBD-II encoding: A = speed (1 km/h units)
 *         Response: [03, 0x41, 0x0D, A, 00, 00, 00, 00]
 */
uint8_t OBD2_GetVehicleSpeed(uint8_t *pTxData, uint8_t speed)
{
    pTxData[0] = 3U;  /* ISO-TP Single Frame: 3-byte payload */
    pTxData[1] = OBD2_MODE_RESPONSE_PREFIX + OBD2_MODE_CURRENT_DATA;  /* 0x41 */
    pTxData[2] = OBD2_PID_VEHICLE_SPEED;                               /* 0x0D */
    pTxData[3] = speed;
    pTxData[4] = 0x00;
    pTxData[5] = 0x00;
    pTxData[6] = 0x00;
    pTxData[7] = 0x00;

    return 3U;
}

/* ====================================================
 * DTC (Diagnostic Trouble Code) Fault Manager
 * ==================================================== */

DtcEntry_t g_dtc_table[OBD2_DTC_COUNT] = {
    { DTC_ENGINE_OVERTEMP,    DTC_STATE_INACTIVE, 0U, 0U,
      DTC_SEVERITY_OVERTEMP,   0U, 0U, 0U, { 0U, 0U, 0U, 0U } },
    { DTC_VSS_MALFUNCTION,    DTC_STATE_INACTIVE, 0U, 0U,
      DTC_SEVERITY_VSS,        0U, 0U, 0U, { 0U, 0U, 0U, 0U } },
    { DTC_COOLANT_THERMOSTAT, DTC_STATE_INACTIVE, 0U, 0U,
      DTC_SEVERITY_THERMOSTAT, 0U, 0U, 0U, { 0U, 0U, 0U, 0U } },
};

/* Mirror memory: CONFIRMED DTCs archived at last clear (UDS 0x19 sub 0x0F/0x10/0x11) */
DtcMirrorEntry_t g_dtc_mirror[OBD2_DTC_COUNT] = { 0U };

/* Test-failed detection sequence counter (increments per PENDING promotion) */
static uint16_t s_fail_seq_counter = 0U;

/**
 * @brief  Update DTC state machine from simulation values (10ms period)
 * @note   Continuous fault condition detection (debounce) -> PENDING -> (sustained) -> CONFIRMED.
 *         On condition clear: PENDING reverts to INACTIVE; CONFIRMED persists until cleared.
 */
void OBD2_DtcUpdate(const OBD2_SimState_t *st)
{
    if (st == NULL) {
        return;
    }

    /* DTC table is shared with vCanRxTask (Get/Clear) -> protect with critical section (H1).
     * Prevents update from overwriting entries while a Mode 04 (clear) is in progress. */
    taskENTER_CRITICAL();
    for (uint8_t i = 0U; i < OBD2_DTC_COUNT; i++) {
        DtcEntry_t *d = &g_dtc_table[i];
        uint8_t cond = 0U;

        switch (d->code) {
            case DTC_ENGINE_OVERTEMP:
                /* Coolant overtemp: coolant reaches MAX (105) */
                cond = (st->coolant_temp >= COOLANT_TEMP_MAX) ? 1U : 0U;
                break;
            case DTC_VSS_MALFUNCTION:
                /* Vehicle speed sensor mismatch: stopped (0 km/h) but high RPM (>2500) */
                cond = ((st->vehicle_speed == VEHICLE_SPEED_MIN) &&
                        (st->engine_rpm > 2500U)) ? 1U : 0U;
                break;
            case DTC_COOLANT_THERMOSTAT:
                /* Overcool / warmup incomplete: coolant at or below MIN (80) */
                cond = (st->coolant_temp <= COOLANT_TEMP_MIN) ? 1U : 0U;
                break;
            default:
                break;
        }

        if (cond != 0U) {
            if (d->debounce < 0xFFU) {
                d->debounce++;
            }
            if (d->state == DTC_STATE_INACTIVE &&
                d->debounce >= OBD2_DTC_DEBOUNCE_THRESH) {
                d->state = DTC_STATE_PENDING;
                d->hold = 0U;
                /* Test failed matured (ISO 14229-1 D.2): record order, occurrence,
                 * freeze frame. fail_seq != 0 also drives status bits 1/5. */
                if (s_fail_seq_counter < 0xFFFFU) {
                    s_fail_seq_counter++;
                }
                d->fail_seq = s_fail_seq_counter;
                if (d->occurrence < 0xFFU) {
                    d->occurrence++;
                }
                d->snapshot.valid = 1U;
                d->snapshot.rpm = st->engine_rpm;
                d->snapshot.speed = st->vehicle_speed;
                d->snapshot.coolant = st->coolant_temp;
            }
            if (d->state == DTC_STATE_PENDING) {
                if (d->hold < 0xFFU) {
                    d->hold++;
                }
                if (d->hold >= OBD2_DTC_CONFIRM_HOLD) {
                    d->state = DTC_STATE_CONFIRMED;
                    d->permanent = 1U;  /* permanentDTC latch (sim: clear on Mode 04) */
                }
            }
        } else {
            d->debounce = 0U;
            /* Condition cleared: PENDING reverts to INACTIVE, CONFIRMED requires clear */
            if (d->state == DTC_STATE_PENDING) {
                d->state = DTC_STATE_INACTIVE;
                d->hold = 0U;
            }
        }
    }
    taskEXIT_CRITICAL();
}

uint8_t OBD2_DtcGetConfirmed(uint8_t *out, uint8_t max_pairs)
{
    uint8_t n = 0U;
    if (out == NULL) {
        return 0U;
    }
    taskENTER_CRITICAL();
    for (uint8_t i = 0U; (i < OBD2_DTC_COUNT) && (n < max_pairs); i++) {
        if (g_dtc_table[i].state == DTC_STATE_CONFIRMED) {
            out[(uint8_t)(n * 2U)]      = (uint8_t)(g_dtc_table[i].code >> 8U);
            out[(uint8_t)(n * 2U + 1U)] = (uint8_t)(g_dtc_table[i].code & 0xFFU);
            n++;
        }
    }
    taskEXIT_CRITICAL();
    return n;
}

uint8_t OBD2_DtcGetPending(uint8_t *out, uint8_t max_pairs)
{
    uint8_t n = 0U;
    if (out == NULL) {
        return 0U;
    }
    taskENTER_CRITICAL();
    for (uint8_t i = 0U; (i < OBD2_DTC_COUNT) && (n < max_pairs); i++) {
        if (g_dtc_table[i].state == DTC_STATE_PENDING) {
            out[(uint8_t)(n * 2U)]      = (uint8_t)(g_dtc_table[i].code >> 8U);
            out[(uint8_t)(n * 2U + 1U)] = (uint8_t)(g_dtc_table[i].code & 0xFFU);
            n++;
        }
    }
    taskEXIT_CRITICAL();
    return n;
}

void OBD2_DtcInjectDemo(void)
{
    taskENTER_CRITICAL();
    DtcEntry_t *d = &g_dtc_table[0];  /* P0217 engine overtemp */
    if (s_fail_seq_counter < 0xFFFFU) {
        s_fail_seq_counter++;
    }
    d->fail_seq = s_fail_seq_counter;
    d->occurrence = 1U;
    d->state = DTC_STATE_CONFIRMED;
    d->permanent = 1U;
    d->debounce = 1U;  /* testFailed present at injection */
    d->hold = 0U;
    d->snapshot.valid = 1U;
    d->snapshot.rpm = g_sim_state.engine_rpm;
    d->snapshot.speed = g_sim_state.vehicle_speed;
    d->snapshot.coolant = g_sim_state.coolant_temp;
    taskEXIT_CRITICAL();
}

void OBD2_DtcClear(void)
{
    taskENTER_CRITICAL();
    /* Archive CONFIRMED DTCs to mirror memory (UDS 0x19 sub 0x0F/0x10/0x11):
     * mirror = fault state snapshot at last clear. */
    for (uint8_t i = 0U; i < OBD2_DTC_COUNT; i++) {
        if (g_dtc_table[i].state == DTC_STATE_CONFIRMED) {
            g_dtc_mirror[i].code = g_dtc_table[i].code;
            g_dtc_mirror[i].status = OBD2_DtcStatusByte(&g_dtc_table[i]);
            g_dtc_mirror[i].occurrence = g_dtc_table[i].occurrence;
        } else {
            g_dtc_mirror[i].code = 0U;
        }
    }
    for (uint8_t i = 0U; i < OBD2_DTC_COUNT; i++) {
        g_dtc_table[i].state = DTC_STATE_INACTIVE;
        g_dtc_table[i].debounce = 0U;
        g_dtc_table[i].hold = 0U;
        g_dtc_table[i].fail_seq = 0U;
        g_dtc_table[i].occurrence = 0U;
        g_dtc_table[i].permanent = 0U;
        g_dtc_table[i].snapshot.valid = 0U;
    }
    s_fail_seq_counter = 0U;
    taskEXIT_CRITICAL();
}

/* ====================================================
 * UDS 0x19 ReadDTCInformation support (ISO 14229-1:2013 D.2)
 * ==================================================== */

uint8_t OBD2_DtcStatusByte(const DtcEntry_t *d)
{
    if (d == NULL) {
        return 0U;
    }
    uint8_t sb = 0U;
    if (d->debounce > 0U) {
        sb |= 0x01U;                      /* bit0 testFailed (fault condition present) */
    }
    if (d->fail_seq != 0U) {
        sb |= 0x26U;                      /* bit1 failedThisOperationCycle + bit2 pendingDTC
                                           * (latched until cycle end/clear, ISO D.2) +
                                           * bit5 failedSinceLastClear */
    }
    if (d->state == DTC_STATE_CONFIRMED) {
        sb |= 0x08U;                      /* bit3 confirmedDTC */
        sb |= 0x80U;                      /* bit7 warningIndicatorRequested (MIL) */
    }
    /* bit4/bit6 (testNotCompleted*) stay 0: monitor completes every 10ms tick */
    return sb;
}

int8_t OBD2_DtcFdc(const DtcEntry_t *d)
{
    if (d == NULL) {
        return 0;
    }
    if ((d->state == DTC_STATE_PENDING) || (d->state == DTC_STATE_CONFIRMED)) {
        return 127;    /* test failed matured */
    }
    /* Prefailed: debounce 1..4 of 5 -> 25/50/75/100. Pass/idle -> 0. */
    return (int8_t)((d->debounce < OBD2_DTC_DEBOUNCE_THRESH)
                        ? (int8_t)(d->debounce * 25U) : 0);
}

uint8_t OBD2_DtcCountByMask(uint8_t mask)
{
    uint8_t n = 0U;
    taskENTER_CRITICAL();
    for (uint8_t i = 0U; i < OBD2_DTC_COUNT; i++) {
        uint8_t sb = OBD2_DtcStatusByte(&g_dtc_table[i]);
        if ((sb & mask & OBD2_DTC_STATUS_AVAILABILITY_MASK) != 0U) {
            n++;
        }
    }
    taskEXIT_CRITICAL();
    return n;
}

uint8_t OBD2_DtcGetByMask(uint8_t *out, uint8_t max_records, uint8_t mask)
{
    uint8_t n = 0U;
    if (out == NULL) {
        return 0U;
    }
    taskENTER_CRITICAL();
    for (uint8_t i = 0U; (i < OBD2_DTC_COUNT) && (n < max_records); i++) {
        uint8_t sb = OBD2_DtcStatusByte(&g_dtc_table[i]);
        if ((sb & mask & OBD2_DTC_STATUS_AVAILABILITY_MASK) != 0U) {
            /* 3-byte DTC record: 2-byte J2010 code << 8 (failure type 0x00) */
            out[(uint8_t)(n * 4U)]      = (uint8_t)(g_dtc_table[i].code >> 8U);
            out[(uint8_t)(n * 4U + 1U)] = (uint8_t)(g_dtc_table[i].code & 0xFFU);
            out[(uint8_t)(n * 4U + 2U)] = 0x00U;
            out[(uint8_t)(n * 4U + 3U)] = sb;
            n++;
        }
    }
    taskEXIT_CRITICAL();
    return n;
}

const DtcEntry_t *OBD2_DtcFindByCode(uint16_t code)
{
    const DtcEntry_t *found = NULL;
    taskENTER_CRITICAL();
    for (uint8_t i = 0U; i < OBD2_DTC_COUNT; i++) {
        if (g_dtc_table[i].code == code) {
            found = &g_dtc_table[i];
            break;
        }
    }
    taskEXIT_CRITICAL();
    return found;
}

const DtcEntry_t *OBD2_DtcPick(uint8_t which)
{
    const DtcEntry_t *best = NULL;
    taskENTER_CRITICAL();
    for (uint8_t i = 0U; i < OBD2_DTC_COUNT; i++) {
        const DtcEntry_t *d = &g_dtc_table[i];
        uint8_t candidate;
        switch (which) {
            case OBD2_DTC_PICK_FIRST_FAILED:
            case OBD2_DTC_PICK_RECENT_FAILED:
                candidate = (d->fail_seq != 0U) ? 1U : 0U;
                break;
            case OBD2_DTC_PICK_FIRST_CONFIRMED:
            case OBD2_DTC_PICK_RECENT_CONFIRMED:
                /* First/most-recent confirmed: fail order == confirm order in this
                 * state machine (confirm follows detection). */
                candidate = (d->state == DTC_STATE_CONFIRMED &&
                             d->fail_seq != 0U) ? 1U : 0U;
                break;
            default:
                candidate = 0U;
                break;
        }
        if (candidate == 0U) {
            continue;
        }
        if (best == NULL) {
            best = d;
        } else if (which == OBD2_DTC_PICK_FIRST_FAILED ||
                   which == OBD2_DTC_PICK_FIRST_CONFIRMED) {
            if (d->fail_seq < best->fail_seq) {
                best = d;
            }
        } else {
            if (d->fail_seq > best->fail_seq) {
                best = d;
            }
        }
    }
    taskEXIT_CRITICAL();
    return best;
}

uint8_t OBD2_MirrorCountByMask(uint8_t mask)
{
    uint8_t n = 0U;
    taskENTER_CRITICAL();
    for (uint8_t i = 0U; i < OBD2_DTC_COUNT; i++) {
        if ((g_dtc_mirror[i].code != 0U) &&
            ((g_dtc_mirror[i].status & mask &
              OBD2_DTC_STATUS_AVAILABILITY_MASK) != 0U)) {
            n++;
        }
    }
    taskEXIT_CRITICAL();
    return n;
}

uint8_t OBD2_MirrorGetByMask(uint8_t *out, uint8_t max_records, uint8_t mask)
{
    uint8_t n = 0U;
    if (out == NULL) {
        return 0U;
    }
    taskENTER_CRITICAL();
    for (uint8_t i = 0U; (i < OBD2_DTC_COUNT) && (n < max_records); i++) {
        if ((g_dtc_mirror[i].code != 0U) &&
            ((g_dtc_mirror[i].status & mask &
              OBD2_DTC_STATUS_AVAILABILITY_MASK) != 0U)) {
            out[(uint8_t)(n * 4U)]      = (uint8_t)(g_dtc_mirror[i].code >> 8U);
            out[(uint8_t)(n * 4U + 1U)] = (uint8_t)(g_dtc_mirror[i].code & 0xFFU);
            out[(uint8_t)(n * 4U + 2U)] = 0x00U;
            out[(uint8_t)(n * 4U + 3U)] = g_dtc_mirror[i].status;
            n++;
        }
    }
    taskEXIT_CRITICAL();
    return n;
}

const DtcMirrorEntry_t *OBD2_MirrorFindByCode(uint16_t code)
{
    const DtcMirrorEntry_t *found = NULL;
    taskENTER_CRITICAL();
    for (uint8_t i = 0U; i < OBD2_DTC_COUNT; i++) {
        if (g_dtc_mirror[i].code == code) {
            found = &g_dtc_mirror[i];
            break;
        }
    }
    taskEXIT_CRITICAL();
    return found;
}
