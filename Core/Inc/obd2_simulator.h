/**
 * @file    obd2_simulator.h
 * @brief   OBD-II ECU simulator header
 * @note    Supported PID definitions, simulation state structure, request/response processing functions
 */

#ifndef __OBD2_SIMULATOR_H
#define __OBD2_SIMULATOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* === OBD-II service mode definitions === */
#define OBD2_MODE_CURRENT_DATA       0x01U  /* Mode 01: current data request */
#define OBD2_MODE_RESPONSE_PREFIX    0x40U  /* Response mode = request mode + 0x40 */

/* === Supported PID definitions === */
#define OBD2_PID_SUPPORTED_PIDS      0x00U  /* Supported PID list */
#define OBD2_PID_COOLANT_TEMP        0x05U  /* Coolant temperature */
#define OBD2_PID_ENGINE_RPM          0x0CU  /* Engine RPM */
#define OBD2_PID_VEHICLE_SPEED       0x0DU  /* Vehicle speed */

/* === Simulation state structure === */
typedef struct {
    /** Current engine RPM (actual value, e.g. 800.0 ~ 4000.0) */
    uint16_t engine_rpm;

    /** Current coolant temperature (actual value Celsius, e.g. 80 ~ 105) */
    uint8_t  coolant_temp;

    /** Current vehicle speed (km/h, e.g. 0 ~ 120) */
    uint8_t  vehicle_speed;

    /** RPM ramp direction: 0 = ramp up (increase), 1 = ramp down (decrease) */
    uint8_t  rpm_direction;

    /** Temperature change direction: 0 = increase, 1 = decrease */
    uint8_t  temp_direction;

    /** Vehicle speed change direction: 0 = increase, 1 = decrease */
    uint8_t  speed_direction;
} OBD2_SimState_t;

/* === DTC (Diagnostic Trouble Code) system ===
 * Resolves the limitation where Mode 03 (stored) / 07 (pending) always returned
 * empty responses (numDTC=0).
 * OBD2_DtcUpdate() detects faults from simulation values and updates the state machine:
 *   INACTIVE -> (debounce) -> PENDING -> (sustained) -> CONFIRMED
 *   - Mode 03 : expose CONFIRMED DTCs
 *   - Mode 07 : expose PENDING DTCs
 *   - Mode 04 / RoutineControl 0x0201 : reset via OBD2_DtcClear()
 * On condition clear: PENDING reverts to INACTIVE, CONFIRMED persists until cleared.
 */
typedef enum {
    DTC_STATE_INACTIVE = 0,
    DTC_STATE_PENDING,    /* Exposed in Mode 07 (pending) */
    DTC_STATE_CONFIRMED   /* Exposed in Mode 03 (stored) */
} DtcState_t;

typedef struct {
    uint16_t   code;            /* SAE J2010 DTC (P0217 -> 0x0217) */
    DtcState_t state;
    uint8_t    debounce;        /* Consecutive condition detection counter */
    uint8_t    hold;            /* PENDING hold counter (-> CONFIRMED) */
} DtcEntry_t;

#define OBD2_DTC_COUNT            3U
#define OBD2_DTC_DEBOUNCE_THRESH  5U    /* Promote to PENDING after 5 consecutive detections (=50ms) */
#define OBD2_DTC_CONFIRM_HOLD     50U   /* Promote to CONFIRMED after 50 PENDING holds (=500ms) */

/* Monitored DTC definitions (SAE J2010 2-byte encoding) */
#define DTC_ENGINE_OVERTEMP       0x0217U  /* P0217: coolant overtemp (coolant >= MAX) */
#define DTC_VSS_MALFUNCTION       0x0500U  /* P0500: vehicle speed=0 but high RPM */
#define DTC_COOLANT_THERMOSTAT    0x0128U  /* P0128: coolant overcool (warmup incomplete) */

extern DtcEntry_t g_dtc_table[OBD2_DTC_COUNT];

/* === Pure logic API (no CAN I/O) === */

/**
 * @brief  OBD-II Mode 01 service handler (pure logic)
 * @param  pid:     requested PID number
 * @param  pTxData: response data buffer (minimum 8 bytes)
 * @retval response data length (0 = unsupported PID)
 * @note   Called when UDS dispatcher receives SID 0x01
 *         Response format: [len, 0x41, PID, data..., 0x00, 0x00, 0x00]
 *         No CAN transmit/receive - caller handles transmission
 */
uint8_t OBD2_HandleService01(uint8_t pid, uint8_t *pTxData);

/**
 * @brief  Periodically update simulation state values (called every 10ms)
 * @param  pState: simulation state structure pointer
 * @retval None
 * @note   Called from main loop or timer interrupt
 */
void OBD2_UpdateSimValues(OBD2_SimState_t *pState);

/**
 * @brief  PID 0x00 response generation: supported PID bitmap
 * @param  pTxData: transmit data buffer (8 bytes)
 * @retval response data length (DLC)
 */
uint8_t OBD2_GetSupportedPIDs(uint8_t *pTxData);

/**
 * @brief  PID 0x05 response generation: coolant temperature
 * @param  pTxData: transmit data buffer
 * @param  temp: coolant temperature (Celsius)
 * @retval response data length (DLC)
 */
uint8_t OBD2_GetCoolantTemp(uint8_t *pTxData, uint8_t temp);

/**
 * @brief  PID 0x0C response generation: engine RPM
 * @param  pTxData: transmit data buffer
 * @param  rpm: engine RPM
 * @retval response data length (DLC)
 */
uint8_t OBD2_GetEngineRPM(uint8_t *pTxData, uint16_t rpm);

/**
 * @brief  PID 0x0D response generation: vehicle speed
 * @param  pTxData: transmit data buffer
 * @param  speed: vehicle speed (km/h)
 * @retval response data length (DLC)
 */
uint8_t OBD2_GetVehicleSpeed(uint8_t *pTxData, uint8_t speed);

/* === DTC (Fault) management API === */

/**
 * @brief  Update DTC state machine from simulation values (called every 10ms)
 * @note   Call from main loop immediately after OBD2_UpdateSimValues().
 *         Continuous fault condition detection (debounce) -> PENDING -> (sustained) -> CONFIRMED.
 */
void OBD2_DtcUpdate(const OBD2_SimState_t *st);

/**
 * @brief  Write DTC codes sequentially to buffer (2 bytes/DTC, big-endian)
 * @param  out:       output buffer (minimum max_pairs*2 bytes)
 * @param  max_pairs: maximum number of DTCs to write
 * @retval actual number of DTCs written
 */
uint8_t OBD2_DtcGetConfirmed(uint8_t *out, uint8_t max_pairs);
uint8_t OBD2_DtcGetPending(uint8_t *out, uint8_t max_pairs);

/** Reset all DTCs to INACTIVE (Mode 04 / RoutineControl 0x0201) */
void OBD2_DtcClear(void);

/** Active (confirmed+pending) DTC count -- UDS 0x19 sub 0x01 */
uint8_t OBD2_DtcCountActive(void);

/**
 * @brief  Write active DTCs as [code_H, code_L, status] -- UDS 0x19 sub 0x02
 * @param  out:          output buffer (max_triples*3 bytes)
 * @param  max_triples:  maximum number of DTCs
 * @retval number of DTCs written (status: 0x08=confirmed, 0x04=pending)
 */
uint8_t OBD2_DtcGetActiveUds(uint8_t *out, uint8_t max_triples);

/* === Global simulation state === */
extern OBD2_SimState_t g_sim_state;

#ifdef __cplusplus
}
#endif

#endif /* __OBD2_SIMULATOR_H */
