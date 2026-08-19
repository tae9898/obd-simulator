/**
 * @file    host_dtc_test.c
 * @brief   Host-side unit test for the DTC model behind UDS 0x19 ReadDTCInformation
 * @note    Builds obd2_simulator.c with stub HAL/FreeRTOS headers and drives the
 *          fault state machine the same way main.c does (10ms ticks).
 *          Run: gcc -Wall -I tests/host -I Core/Inc Core/Src/obd2_simulator.c \
 *                   tests/host/host_dtc_test.c -o /tmp/host_dtc_test && /tmp/host_dtc_test
 */
#include <stdio.h>
#include <assert.h>
#include "obd2_simulator.h"

/* main.c owns this in the firmware; host test defines it */
OBD2_SimState_t g_sim_state = { 800U, 90U, 0U, 0U, 0U, 0U };

static void tick(int n)
{
    for (int i = 0; i < n; i++) {
        OBD2_DtcUpdate(&g_sim_state);
    }
}

int main(void)
{
    const DtcEntry_t *d = &g_dtc_table[0];   /* P0217 */
    uint8_t rec[OBD2_DTC_COUNT * 4U];
    uint8_t n;

    /* --- idle: no fault, status 0 --- */
    tick(3);
    assert(OBD2_DtcStatusByte(d) == 0x00U);
    assert(OBD2_DtcCountByMask(0xFFU) == 0U);
    assert(OBD2_DtcFdc(d) == 0);
    assert(OBD2_DtcPick(OBD2_DTC_PICK_FIRST_FAILED) == NULL);
    n = OBD2_DtcGetByMask(rec, OBD2_DTC_COUNT, 0xFFU);
    assert(n == 0U);

    /* --- prefailed: debounce 1..4 -> FDC 25..100, only testFailed bit --- */
    g_sim_state.coolant_temp = 105U;   /* overtemp condition */
    tick(1);
    assert(OBD2_DtcFdc(d) == 25);
    tick(3);
    assert(OBD2_DtcFdc(d) == 100);
    assert(OBD2_DtcStatusByte(d) == 0x01U);
    assert(OBD2_DtcCountByMask(0x08U) == 0U);

    /* --- matured: PENDING at debounce 5, FDC 127 (excluded from 0x14 list) --- */
    tick(1);
    assert(d->state == DTC_STATE_PENDING);
    assert(OBD2_DtcFdc(d) == 127);
    /* testFailed(0x01)+failedThisCycle(0x02)+pendingLatched(0x04)+sinceLastClear(0x20) */
    assert(OBD2_DtcStatusByte(d) == 0x27U);
    assert(OBD2_DtcCountByMask(0x04U) == 1U);
    assert(d->fail_seq == 1U);
    assert(d->occurrence == 1U);
    assert(d->snapshot.valid == 1U);
    assert(d->snapshot.coolant == 105U);
    n = OBD2_DtcGetByMask(rec, OBD2_DTC_COUNT, 0x04U);
    assert(n == 1U && rec[0] == 0x02U && rec[1] == 0x17U && rec[2] == 0x00U && rec[3] == 0x27U);

    /* --- CONFIRMED after hold: +bit3 confirmed +bit7 MIL, permanent latched --- */
    tick(OBD2_DTC_CONFIRM_HOLD);
    assert(d->state == DTC_STATE_CONFIRMED);
    assert(d->permanent == 1U);
    assert(OBD2_DtcStatusByte(d) == 0xAFU);          /* 0x27 | 0x08 | 0x80 */
    assert(OBD2_DtcPick(OBD2_DTC_PICK_FIRST_CONFIRMED) == d);
    assert(OBD2_DtcPick(OBD2_DTC_PICK_RECENT_FAILED) == d);

    /* --- condition clears: testFailed drops, latched bits persist.
     * FDC stays 127: pass-sample decay is not modeled (simulator limitation;
     * the DTC is matured until ClearDiagnosticInformation). --- */
    g_sim_state.coolant_temp = 90U;
    tick(2);
    assert(OBD2_DtcStatusByte(d) == 0xAEU);          /* 0xAF & ~0x01 */
    assert(OBD2_DtcFdc(d) == 127);
    /* re-fail while CONFIRMED does not re-latch (no INACTIVE->PENDING transition) */
    g_sim_state.coolant_temp = 105U;
    tick(OBD2_DTC_DEBOUNCE_THRESH);
    assert(d->occurrence == 1U && d->fail_seq == 1U);

    /* --- clear: mirror archives CONFIRMED entry, everything else resets --- */
    OBD2_DtcClear();
    assert(d->state == DTC_STATE_INACTIVE);
    assert(d->fail_seq == 0U && d->occurrence == 0U && d->permanent == 0U);
    assert(d->snapshot.valid == 0U);
    assert(OBD2_DtcStatusByte(d) == 0x00U);
    assert(OBD2_DtcFdc(d) == 0);
    assert(OBD2_MirrorCountByMask(0xFFU) == 1U);
    assert(OBD2_MirrorFindByCode(DTC_ENGINE_OVERTEMP) != NULL);
    assert(OBD2_MirrorFindByCode(DTC_VSS_MALFUNCTION) == NULL);
    n = OBD2_MirrorGetByMask(rec, OBD2_DTC_COUNT, 0x08U);
    assert(n == 1U && rec[0] == 0x02U && rec[3] == 0xAFU);  /* status at archive time */
    assert(OBD2_DtcPick(OBD2_DTC_PICK_RECENT_FAILED) == NULL);

    /* --- demo injection (RoutineControl 0x0202 test hook) --- */
    OBD2_DtcInjectDemo();
    assert(d->state == DTC_STATE_CONFIRMED);
    assert(d->permanent == 1U);
    assert(d->occurrence == 1U);
    assert(d->snapshot.valid == 1U);
    assert(OBD2_DtcCountByMask(0x08U) == 1U);
    assert(OBD2_DtcFindByCode(0x0606U) == NULL);

    /* --- clear again resets injection --- */
    OBD2_DtcClear();
    assert(OBD2_DtcCountByMask(0xFFU) == 0U);

    /* --- multi-DTC ordering: overtemp + VSS mismatch mature simultaneously --- */
    g_sim_state.coolant_temp = 105U;
    g_sim_state.engine_rpm = 3000U;
    g_sim_state.vehicle_speed = 0U;
    tick(OBD2_DTC_DEBOUNCE_THRESH);
    assert(OBD2_DtcCountByMask(0x04U) == 2U);
    assert(OBD2_DtcPick(OBD2_DTC_PICK_FIRST_FAILED)->code == DTC_ENGINE_OVERTEMP);
    assert(OBD2_DtcPick(OBD2_DTC_PICK_RECENT_FAILED)->code == DTC_VSS_MALFUNCTION);
    n = OBD2_DtcGetByMask(rec, OBD2_DTC_COUNT, 0xFFU);
    assert(n == 2U);

    printf("host_dtc_test: all assertions passed\n");
    return 0;
}
