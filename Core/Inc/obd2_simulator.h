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

/* Freeze frame captured at PENDING promotion (UDS 0x19 sub 0x03/0x04/0x05) */
typedef struct {
    uint8_t  valid;      /* 1 = snapshot captured (not cleared) */
    uint16_t rpm;        /* engine RPM at capture */
    uint8_t  speed;      /* vehicle speed at capture */
    uint8_t  coolant;    /* coolant temperature at capture */
} DtcSnapshot_t;

typedef struct {
    uint16_t   code;            /* SAE J2010 DTC (P0217 -> 0x0217) */
    DtcState_t state;
    uint8_t    debounce;        /* Consecutive condition detection counter */
    uint8_t    hold;            /* PENDING hold counter (-> CONFIRMED) */
    /* === ISO 14229-1 (0x19 ReadDTCInformation) additions === */
    uint8_t    severity;        /* DTCSeverity byte (ISO 14229-1 D.3), const per DTC */
    uint16_t   fail_seq;        /* Test-failed detection order (0 = never since clear) */
    uint8_t    occurrence;      /* Failed test count -> ext data record 0x01 */
    uint8_t    permanent;       /* Latched at CONFIRMED (UDS 0x19 sub 0x15/0x55).
                                 * Simulator limitation: cleared by ClearDiagnosticInformation
                                 * (real: only cleared when the monitor passes 3 cycles). */
    DtcSnapshot_t snapshot;     /* Freeze frame at first PENDING promotion */
} DtcEntry_t;

/* Mirror memory: CONFIRMED DTCs archived at ClearDiagnosticInformation (UDS 0x19 sub 0x0F/0x10/0x11) */
typedef struct {
    uint16_t code;
    uint8_t  status;      /* statusOfDTC at archive time */
    uint8_t  occurrence;  /* ext data record 0x01 at archive time */
} DtcMirrorEntry_t;

#define OBD2_DTC_COUNT            3U
#define OBD2_DTC_DEBOUNCE_THRESH  5U    /* Promote to PENDING after 5 consecutive detections (=50ms) */
#define OBD2_DTC_CONFIRM_HOLD     50U   /* Promote to CONFIRMED after 50 PENDING holds (=500ms) */

/* Monitored DTC definitions (SAE J2010 2-byte encoding) */
#define DTC_ENGINE_OVERTEMP       0x0217U  /* P0217: coolant overtemp (coolant >= MAX) */
#define DTC_VSS_MALFUNCTION       0x0500U  /* P0500: vehicle speed=0 but high RPM */
#define DTC_COOLANT_THERMOSTAT    0x0128U  /* P0128: coolant overcool (warmup incomplete) */

/* DTCSeverity (ISO 14229-1 D.3): bit7 checkImmediately / bit6 checkAtNextHalt /
 * bit5 maintenanceOnly / bit0-4 GTR DTC class (bit1 = Class A). */
#define DTC_SEVERITY_OVERTEMP     0x82U  /* P0217: checkImmediately + Class A */
#define DTC_SEVERITY_VSS          0x42U  /* P0500: checkAtNextHalt + Class A */
#define DTC_SEVERITY_THERMOSTAT   0x22U  /* P0128: maintenanceOnly + Class A */

/* DTCFunctionalUnit: no standard value -- 0x01 = powertrain (simulator definition) */
#define DTC_FUNCTIONAL_UNIT       0x01U

/* Snapshot/stored-data record content: one DID, 4 bytes [rpm(2) speed(1) coolant(1)] */
#define DTC_SNAPSHOT_DID          0xF500U
#define DTC_SNAPSHOT_DATA_LEN     4U
#define DTC_EXT_RECORD_OCCURRENCE 0x01U  /* ext data record 0x01 = occurrence counter */

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

/** Reset all DTCs to INACTIVE (Mode 04 / RoutineControl 0x0201).
 *  CONFIRMED entries are archived to mirror memory before reset. */
void OBD2_DtcClear(void);

/**
 * @brief  Inject a demo CONFIRMED DTC (P0217) with snapshot -- test hook
 * @note   RoutineControl 0x0202 (Self Test) start trigger. The natural simulation
 *         ramp only dwells 1 tick at temperature extremes, so monitored DTCs never
 *         mature on their own; this makes 0x19 data paths testable on hardware.
 */
void OBD2_DtcInjectDemo(void);

/* === UDS 0x19 ReadDTCInformation support API (ISO 14229-1:2013) === */

/** DTCStatusAvailabilityMask -- server supported statusOfDTC bits (all 8) */
#define OBD2_DTC_STATUS_AVAILABILITY_MASK  0xFFU

/**
 * @brief  Compute full ISO 14229-1 D.2 statusOfDTC byte for an entry
 * @note   bit0 testFailed = debounce>0 / bit1+bit2+bit5 = fail_seq!=0
 *         (failed+pending latched for the operation cycle) / bit3 confirmed /
 *         bit7 warningIndicator(MIL on confirmed).
 *         bit4+bit6 always 0: monitor runs to completion every 10ms tick.
 */
uint8_t OBD2_DtcStatusByte(const DtcEntry_t *d);

/** DTCFaultDetectionCounter (sub 0x14): signed scaled value, +127 = failed */
int8_t OBD2_DtcFdc(const DtcEntry_t *d);

/** Count DTCs whose (status & mask & availability) != 0 -- sub 0x01/0x07/0x11/0x12 */
uint8_t OBD2_DtcCountByMask(uint8_t mask);

/**
 * @brief  Write mask-matching DTCs as [DTC(3B: code<<8) + statusOfDTC] records
 * @retval number of records written (sub 0x02/0x0A/0x13/0x15/0x17)
 */
uint8_t OBD2_DtcGetByMask(uint8_t *out, uint8_t max_records, uint8_t mask);

/** Find a supported DTC by 2-byte code (NULL = not supported) -- sub 0x04/0x06/0x09/0x18/0x19 */
const DtcEntry_t *OBD2_DtcFindByCode(uint16_t code);

/* Single-DTC selectors (sub 0x0B~0x0E): NULL when none detected */
#define OBD2_DTC_PICK_FIRST_FAILED     0U  /* min fail_seq != 0 */
#define OBD2_DTC_PICK_RECENT_FAILED    1U  /* max fail_seq */
#define OBD2_DTC_PICK_FIRST_CONFIRMED  2U
#define OBD2_DTC_PICK_RECENT_CONFIRMED 3U
const DtcEntry_t *OBD2_DtcPick(uint8_t which);

/* Mirror memory (sub 0x0F/0x10/0x11): archived at last clear */
extern DtcMirrorEntry_t g_dtc_mirror[OBD2_DTC_COUNT];
uint8_t OBD2_MirrorCountByMask(uint8_t mask);
uint8_t OBD2_MirrorGetByMask(uint8_t *out, uint8_t max_records, uint8_t mask);
const DtcMirrorEntry_t *OBD2_MirrorFindByCode(uint16_t code);

/* === Global simulation state === */
extern OBD2_SimState_t g_sim_state;

#ifdef __cplusplus
}
#endif

#endif /* __OBD2_SIMULATOR_H */
