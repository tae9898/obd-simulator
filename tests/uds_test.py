#!/usr/bin/env python3
"""
UDS (Unified Diagnostic Services) ECU Simulator Test

Description:
    Tests UDS services (SID 0x10, 0x11, 0x22, 0x27, 0x31).
    Operates over ISO-TP single frame protocol.

Usage:
    sudo python3 uds_test.py [CAN interface]
    Example: sudo python3 uds_test.py vcan0 --setup-vcan

UDS CAN IDs:
    0x7E0 - Request (tester -> ECU)
    0x7E8 - Response (ECU -> tester)

UDS Services:
    0x10 - DiagnosticSessionControl
    0x11 - ECU Reset
    0x22 - ReadDataByIdentifier
    0x27 - SecurityAccess
    0x31 - RoutineControl

ISO-TP Single Frame (SF) structure:
    Request:  [length, SID, subfunction, parameter...]
    Positive response: [length, SID+0x40, subfunction, data...]
    Negative response: [length, 0x7F, SID, NRC]
"""

import sys
import time
import argparse
import subprocess
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional, List, Tuple

try:
    import can
except ImportError:
    print("python-can install required: pip install python-can")
    sys.exit(1)


#=======================================
# Common utilities
#=======================================
class Color:
    RED = '\033[0;31m'
    GREEN = '\033[0;32m'
    YELLOW = '\033[1;33m'
    CYAN = '\033[0;36m'
    NC = '\033[0m'

    @staticmethod
    def pass_msg(msg): return f"{Color.GREEN}[PASS]{Color.NC} {msg}"
    @staticmethod
    def fail_msg(msg): return f"{Color.RED}[FAIL]{Color.NC} {msg}"
    @staticmethod
    def info_msg(msg): return f"{Color.CYAN}[INFO]{Color.NC} {msg}"
    @staticmethod
    def header(msg): return f"\n{Color.CYAN}=== {msg} ==={Color.NC}"


class TestStatus(Enum):
    PASS = "PASS"
    FAIL = "FAIL"
    SKIP = "SKIP"


@dataclass
class TestResult:
    name: str
    status: TestStatus = TestStatus.FAIL  # unset result must not silently pass
    detail: str = ""
    raw_tx: str = ""
    raw_rx: str = ""


@dataclass
class TestReport:
    results: list = field(default_factory=list)

    def add(self, result: TestResult):
        self.results.append(result)

    def print_summary(self):
        print("\n" + "=" * 78)
        print(f"  {'Test Item':<35} {'Status':<10} {'Detail':<30}")
        print("=" * 78)
        for r in self.results:
            if r.status == TestStatus.PASS:
                s = Color.pass_msg(r.status.value)
            elif r.status == TestStatus.FAIL:
                s = Color.fail_msg(r.status.value)
            else:
                s = f"{Color.YELLOW}[SKIP]{Color.NC}"
            detail = r.detail[:28] + ".." if len(r.detail) > 30 else r.detail
            print(f"  {r.name:<35} {s:<20} {detail}")
            if r.raw_tx:
                print(f"  {'':>35}   TX: {r.raw_tx}")
            if r.raw_rx:
                print(f"  {'':>35}   RX: {r.raw_rx}")
        print("=" * 78)

        p = sum(1 for r in self.results if r.status == TestStatus.PASS)
        f = sum(1 for r in self.results if r.status == TestStatus.FAIL)
        total = len(self.results)
        print(f"  Total {total} | {Color.GREEN}Passed {p}{Color.NC} | {Color.RED}Failed {f}{Color.NC}")
        print("=" * 78)
        return f == 0


#=======================================
# UDS NRC (Negative Response Code)
#=======================================
NRC_NAMES = {
    0x10: "generalReject",
    0x11: "serviceNotSupported",
    0x12: "subFunctionNotSupported",
    0x13: "incorrectMessageLength",
    0x22: "conditionsNotCorrect",
    0x24: "requestSequenceError",
    0x31: "requestOutOfRange",
    0x33: "securityAccessDenied",
    0x35: "invalidKey",
    0x36: "exceededNumberOfAttempts",
    0x37: "requiredTimeDelayNotExpired",
    0x72: "generalProgrammingFailure",
    0x78: "requestCorrectlyReceived-ResponsePending",
}


def nrc_name(code: int) -> str:
    return NRC_NAMES.get(code, f"unknown(0x{code:02X})")


#=======================================
# UDS tester
#=======================================
class UDSTester:
    """UDS service test class"""

    REQUEST_ID = 0x7E0
    RESPONSE_ID = 0x7E8

    # UDS SIDs
    SID_DIAG_SESSION   = 0x10
    SID_ECU_RESET      = 0x11
    SID_READ_DID       = 0x22
    SID_SECURITY       = 0x27
    SID_ROUTINE        = 0x31

    # Session types
    SESSION_DEFAULT     = 0x01
    SESSION_PROGRAMMING = 0x02
    SESSION_EXTENDED    = 0x03

    # DIDs
    DID_VIN       = 0xF190
    DID_HW_VER    = 0xF193
    DID_SW_VER    = 0xF195

    # Reset types
    RESET_HARD = 0x01
    RESET_SOFT = 0x03

    # Security
    SEC_REQUEST_SEED = 0x01
    SEC_SEND_KEY     = 0x02

    # Seed-Key algorithm (same as ECU)
    SEED_XOR_MASK = 0x5A3C

    def __init__(self, interface: str, bitrate: int = 500000, data_bitrate: int = 2000000):
        self.interface = interface
        self.bitrate = bitrate
        self.data_bitrate = data_bitrate
        self.bus: Optional[can.Bus] = None

    def connect(self):
        # fd=True: the ECU transmits responses as CAN-FD BRS frames; a classic
        # socket silently drops them (every test times out otherwise).
        self.bus = can.Bus(
            interface='socketcan',
            channel=self.interface,
            bitrate=self.bitrate,
            fd=True,
            data_bitrate=self.data_bitrate,
        )
        print(Color.info_msg(f"CAN-FD connected: {self.interface} "
                             f"({self.bitrate}/{self.data_bitrate} bps)"))

    def disconnect(self):
        if self.bus:
            self.bus.shutdown()
            self.bus = None

    def compute_key(self, seed: int) -> int:
        """Seed-key algorithm identical to ECU"""
        xored = seed ^ self.SEED_XOR_MASK
        rotated = ((xored << 3) | (xored >> 13)) & 0xFFFF
        return rotated

    def send_uds(self, data: list, timeout: float = 2.0) -> Optional[bytes]:
        """Send UDS request and receive response"""
        if not self.bus:
            raise RuntimeError("CAN not connected")

        msg = can.Message(
            arbitration_id=self.REQUEST_ID,
            data=data + [0x00] * (8 - len(data)),
            is_extended_id=False,
        )
        self.bus.send(msg)

        resp = self.bus.recv(timeout=timeout)
        if resp is None:
            return None
        if resp.arbitration_id != self.RESPONSE_ID:
            return None
        return bytes(resp.data)

    def parse_positive(self, data: bytes, expected_sid: int) -> Optional[dict]:
        """Parse positive response: verify SID+0x40 (CAN-FD escape SF aware)"""
        if not data or len(data) < 2:
            return None
        if (data[0] & 0x0F) == 0x00 and len(data) >= 3:  # CAN-FD escape SF
            sid_resp = data[2]
            payload = data[3:2 + data[1]]
        else:
            sid_resp = data[1]
            payload = data[2:1 + (data[0] & 0x0F)]
        if sid_resp != expected_sid + 0x40:
            return None
        return {'sid_resp': sid_resp, 'payload': payload}

    def parse_negative(self, data: bytes) -> Optional[dict]:
        """Parse negative response: 0x7F + SID + NRC"""
        if not data or len(data) < 4:
            return None
        if data[1] != 0x7F:
            return None
        return {'sid': data[2], 'nrc': data[3], 'nrc_name': nrc_name(data[3])}


#=======================================
# Test cases
#=======================================
def test_diag_session(tester: UDSTester, report: TestReport):
    """SID 0x10: DiagnosticSessionControl test"""
    print(Color.header("SID 0x10: DiagnosticSessionControl"))

    # 1) Switch to Default session (must always succeed)
    resp = tester.send_uds([0x02, 0x10, 0x01])
    result = TestResult("Default session switch (0x01)")
    if resp:
        pos = tester.parse_positive(resp, 0x10)
        if pos and pos['payload'][0] == 0x01:
            result.status = TestStatus.PASS
            result.detail = "Normal response"
        else:
            neg = tester.parse_negative(resp)
            if neg:
                result.status = TestStatus.FAIL
                result.detail = f"NRC {neg['nrc_name']}"
            else:
                result.status = TestStatus.FAIL
                result.detail = "Parse failed"
        result.raw_rx = resp.hex().upper()
    else:
        result.status = TestStatus.FAIL
        result.detail = "Timeout"
    result.raw_tx = "02 10 01"
    print(f"  {result.status.value}: {result.detail}")
    report.add(result)

    # 2) Switch to Extended session
    resp = tester.send_uds([0x02, 0x10, 0x03])
    result = TestResult("Extended session switch (0x03)")
    if resp:
        pos = tester.parse_positive(resp, 0x10)
        if pos and pos['payload'][0] == 0x03:
            result.status = TestStatus.PASS
            result.detail = "Normal response"
        else:
            neg = tester.parse_negative(resp)
            if neg:
                result.status = TestStatus.FAIL
                result.detail = f"NRC {neg['nrc_name']}"
            else:
                result.status = TestStatus.FAIL
                result.detail = "Parse failed"
        result.raw_rx = resp.hex().upper()
    else:
        result.status = TestStatus.FAIL
        result.detail = "Timeout"
    result.raw_tx = "02 10 03"
    print(f"  {result.status.value}: {result.detail}")
    report.add(result)

    # 3) Invalid session type (0xFF) -> NRC expected
    resp = tester.send_uds([0x02, 0x10, 0xFF])
    result = TestResult("Invalid session type (0xFF)")
    if resp:
        neg = tester.parse_negative(resp)
        if neg:
            result.status = TestStatus.PASS
            result.detail = f"NRC {neg['nrc_name']} (expected)"
        else:
            result.status = TestStatus.FAIL
            result.detail = "Expected NRC but got positive response"
        result.raw_rx = resp.hex().upper()
    else:
        result.status = TestStatus.FAIL
        result.detail = "Timeout"
    result.raw_tx = "02 10 FF"
    print(f"  {result.status.value}: {result.detail}")
    report.add(result)

    # Return to Default
    tester.send_uds([0x02, 0x10, 0x01])


def test_read_did(tester: UDSTester, report: TestReport):
    """SID 0x22: ReadDataByIdentifier test"""
    print(Color.header("SID 0x22: ReadDataByIdentifier"))

    # 1) Read HW version (0xF193)
    resp = tester.send_uds([0x03, 0x22, 0xF1, 0x93])
    result = TestResult("Read DID 0xF193 (HW version)")
    if resp:
        pos = tester.parse_positive(resp, 0x22)
        if pos and len(pos['payload']) >= 3:
            did = (pos['payload'][0] << 8) | pos['payload'][1]
            value = pos['payload'][2:]
            result.status = TestStatus.PASS
            result.detail = f"DID=0x{did:04X} value={value}"
        else:
            neg = tester.parse_negative(resp)
            if neg:
                result.status = TestStatus.FAIL
                result.detail = f"NRC {neg['nrc_name']}"
            else:
                result.status = TestStatus.FAIL
                result.detail = "Parse failed"
        result.raw_rx = resp.hex().upper()
    else:
        result.status = TestStatus.FAIL
        result.detail = "Timeout"
    result.raw_tx = "03 22 F1 93"
    print(f"  {result.status.value}: {result.detail}")
    report.add(result)

    # 2) Read SW version (0xF195)
    resp = tester.send_uds([0x03, 0x22, 0xF1, 0x95])
    result = TestResult("Read DID 0xF195 (SW version)")
    if resp:
        pos = tester.parse_positive(resp, 0x22)
        if pos and len(pos['payload']) >= 3:
            did = (pos['payload'][0] << 8) | pos['payload'][1]
            value = pos['payload'][2:]
            result.status = TestStatus.PASS
            result.detail = f"DID=0x{did:04X} value={value}"
        else:
            neg = tester.parse_negative(resp)
            if neg:
                result.status = TestStatus.FAIL
                result.detail = f"NRC {neg['nrc_name']}"
            else:
                result.status = TestStatus.FAIL
                result.detail = "Parse failed"
        result.raw_rx = resp.hex().upper()
    else:
        result.status = TestStatus.FAIL
        result.detail = "Timeout"
    result.raw_tx = "03 22 F1 95"
    print(f"  {result.status.value}: {result.detail}")
    report.add(result)

    # 3) Non-existent DID -> NRC expected
    resp = tester.send_uds([0x03, 0x22, 0xFF, 0xFF])
    result = TestResult("Non-existent DID (0xFFFF)")
    if resp:
        neg = tester.parse_negative(resp)
        if neg:
            result.status = TestStatus.PASS
            result.detail = f"NRC {neg['nrc_name']} (expected)"
        else:
            result.status = TestStatus.FAIL
            result.detail = "Expected NRC"
        result.raw_rx = resp.hex().upper()
    else:
        result.status = TestStatus.FAIL
        result.detail = "Timeout"
    result.raw_tx = "03 22 FF FF"
    print(f"  {result.status.value}: {result.detail}")
    report.add(result)


#=======================================
# SID 0x19: ReadDTCInformation (all sub-functions)
#=======================================
DTC_P0217 = 0x0217  # demo injected DTC (RoutineControl 0x0202 Self Test)


def _rdtci_send(tester: UDSTester, sub: int, params: tuple) -> Optional[bytes]:
    """Send [0x19, sub, params...] SF; return response body WITHOUT the 0x59
    response SID (body starts at the sub-function echo). CAN-FD escape SF aware."""
    payload = [0x19, sub] + list(params)
    resp = tester.send_uds([len(payload) & 0x0F] + payload)
    if not resp:
        return None
    data = bytes(resp)
    if (data[0] & 0x0F) == 0x00 and len(data) >= 3:  # CAN-FD escape SF
        body = data[2:2 + data[1]]
    else:
        body = data[1:1 + (data[0] & 0x0F)]
    # strip the response SID echo (0x59) for sub-function-first checks,
    # keep 0x7F negative responses intact
    return body[1:] if (body and body[0] == 0x59) else body


def _rdtci_case(tester: UDSTester, report: TestReport, name: str,
                sub: int, params: tuple,
                neg_nrc: Optional[int] = None,
                check=None):
    """One 0x19 check. neg_nrc: expect NRC. check(body)->(ok, detail)."""
    body = _rdtci_send(tester, sub, params)
    result = TestResult(name)
    if body is None:
        result.status = TestStatus.FAIL
        result.detail = "Timeout"
    elif neg_nrc is not None:
        if len(body) >= 3 and body[0] == 0x7F and body[2] == neg_nrc:
            result.status = TestStatus.PASS
            result.detail = f"NRC 0x{neg_nrc:02X} (expected)"
        else:
            result.status = TestStatus.FAIL
            result.detail = f"Expected NRC 0x{neg_nrc:02X}, got {body.hex().upper()}"
    else:
        try:
            ok, detail = check(body)
        except (IndexError, ValueError) as e:
            ok, detail = False, f"check error: {e}"
        result.status = TestStatus.PASS if ok else TestStatus.FAIL
        result.detail = detail
    result.raw_tx = ' '.join(f'{b:02X}' for b in ([0x19, sub] + list(params)))
    if body is not None:
        result.raw_rx = body.hex().upper()
    print(f"  {result.status.value}: {name} -- {result.detail}")
    report.add(result)


def _body_ok(expected_head: bytes, name_extra: str = ""):
    """Check body == expected head + arbitrary tail (e.g. records)."""
    def check(body: bytes):
        if bytes(body[:len(expected_head)]) == expected_head:
            return True, f"head {expected_head.hex().upper()} OK{name_extra}"
        return False, f"expected head {expected_head.hex().upper()}, got {body.hex().upper()}"
    return check


def _n_records(n: int, rec_size: int, head_len: int):
    """Check body has head (head_len bytes) followed by exactly n records of rec_size."""
    def check(body: bytes):
        if len(body) == head_len + n * rec_size:
            return True, f"{n} record(s)"
        return False, f"expected len {head_len + n * rec_size}, got {len(body)}: {body.hex().upper()}"
    return check


def test_read_dtc_information(tester: UDSTester, report: TestReport):
    """SID 0x19: ReadDTCInformation -- all sub-functions (ISO 14229-1:2013)"""
    print(Color.header("SID 0x19: ReadDTCInformation"))

    # --- Phase 1: no DTC (monitored DTCs never mature in the natural ramp) ---
    def supported_dtc_idle(body: bytes):
        # 0x0A always lists every supported DTC (3), all status 0x00 when idle
        if (len(body) == 14 and body[0] == 0x0A and body[1] == 0xFF and
                body[2:5] == bytes([0x02, 0x17, 0x00]) and body[5] == 0x00 and
                body[6:9] == bytes([0x05, 0x00, 0x00]) and body[9] == 0x00 and
                body[10:13] == bytes([0x01, 0x28, 0x00]) and body[13] == 0x00):
            return True, "3 supported DTCs, all status 0x00"
        return False, f"got {body.hex().upper()}"

    _rdtci_case(tester, report, "0x0A reportSupportedDTC (3 idle DTCs)",
                0x0A, (), check=supported_dtc_idle)
    _rdtci_case(tester, report, "0x01 count by mask 0xFF = 0",
                0x01, (0xFF,), check=_body_ok(bytes([0x01, 0xFF, 0x01, 0x00, 0x00])))
    _rdtci_case(tester, report, "0x01 count by mask 0x08 = 0",
                0x01, (0x08,), check=_body_ok(bytes([0x01, 0xFF, 0x01, 0x00, 0x00])))
    _rdtci_case(tester, report, "0x02 list by mask empty (SAM only)",
                0x02, (0xFF,), check=_body_ok(bytes([0x02, 0xFF])))
    _rdtci_case(tester, report, "0x03 snapshot identification empty",
                0x03, (), check=_body_ok(bytes([0x03])))
    _rdtci_case(tester, report, "0x14 FDC list empty (no prefailed DTC)",
                0x14, (), check=_body_ok(bytes([0x14])))
    _rdtci_case(tester, report, "0x15 permanent empty (SAM only)",
                0x15, (), check=_body_ok(bytes([0x15, 0xFF])))
    _rdtci_case(tester, report, "0x16 ext data by record empty",
                0x16, (0x01,), check=_body_ok(bytes([0x16, 0x01])))
    _rdtci_case(tester, report, "0x17 userDefMemory list (memSel 0x01)",
                0x17, (0xFF, 0x01), check=_body_ok(bytes([0x17, 0x01, 0xFF])))
    _rdtci_case(tester, report, "0x42 WWH-OBD by mask empty",
                0x42, (0xFF, 0xFF, 0xFF),
                check=_body_ok(bytes([0x42, 0xFF, 0xFF, 0xE2, 0x04])))
    _rdtci_case(tester, report, "0x55 WWH-OBD permanent empty",
                0x55, (0x33,), check=_body_ok(bytes([0x55, 0x33, 0xFF, 0x04])))
    _rdtci_case(tester, report, "0x0B firstTestFailed none (SAM only)",
                0x0B, (), check=_body_ok(bytes([0x0B, 0xFF])))

    # --- Phase 2: NRC behaviour ---
    _rdtci_case(tester, report, "0x1A unsupported sub-function -> NRC 0x12",
                0x1A, (), neg_nrc=0x12)
    _rdtci_case(tester, report, "0x01 missing statusMask -> NRC 0x13",
                0x01, (), neg_nrc=0x13)
    _rdtci_case(tester, report, "0x04 unknown DTC -> NRC 0x31",
                0x04, (0x06, 0x06, 0x00, 0x01), neg_nrc=0x31)
    _rdtci_case(tester, report, "0x09 unknown DTC -> NRC 0x31",
                0x09, (0x06, 0x06, 0x00), neg_nrc=0x31)
    _rdtci_case(tester, report, "0x17 bad memorySelection -> NRC 0x31",
                0x17, (0xFF, 0x02), neg_nrc=0x31)
    _rdtci_case(tester, report, "0x42 bad FGID -> NRC 0x31",
                0x42, (0x00, 0xFF, 0xFF), neg_nrc=0x31)
    _rdtci_case(tester, report, "0x05 bad record number -> NRC 0x31",
                0x05, (0x02,), neg_nrc=0x31)
    _rdtci_case(tester, report, "0x04 bad snapshot record number -> NRC 0x31",
                0x04, (0x02, 0x17, 0x00, 0x02), neg_nrc=0x31)

    # --- Phase 3: inject demo DTC (P0217) via RoutineControl 0x0202 Self Test ---
    tester.send_uds([0x02, 0x10, 0x03])
    time.sleep(0.05)
    resp = tester.send_uds([0x02, 0x27, 0x01])
    injected = False
    if resp:
        pos = tester.parse_positive(resp, 0x27)
        if pos and len(pos['payload']) >= 3:
            seed = (pos['payload'][1] << 8) | pos['payload'][2]
            key = tester.compute_key(seed)
            resp2 = tester.send_uds([0x04, 0x27, 0x02, (key >> 8) & 0xFF, key & 0xFF])
            if resp2 and tester.parse_positive(resp2, 0x27):
                resp3 = tester.send_uds([0x04, 0x31, 0x01, 0x02, 0x02])
                injected = bool(resp3 and tester.parse_positive(resp3, 0x31))
    result = TestResult("Inject demo DTC P0217 (RoutineControl 0x0202)")
    result.status = TestStatus.PASS if injected else TestStatus.FAIL
    result.detail = "CONFIRMED P0217 injected" if injected else "Injection failed"
    result.raw_tx = "04 31 01 02 02"
    print(f"  {result.status.value}: {result.detail}")
    report.add(result)

    if injected:
        p0217 = (DTC_P0217 >> 8, DTC_P0217 & 0xFF, 0x00)

        _rdtci_case(tester, report, "0x0A supported list: P0217 confirmed + 2 idle",
                    0x0A, (), check=lambda b: (
                        (len(b) == 14 and b[0] == 0x0A and b[1] == 0xFF and
                         b[2:5] == bytes(p0217) and (b[5] & 0x08) != 0 and
                         b[9] == 0x00 and b[13] == 0x00,
                         f"P0217 status 0x{b[5]:02X}, others idle" if len(b) == 14 else "bad len")))
        _rdtci_case(tester, report, "0x01 count by mask 0x08 = 1",
                    0x01, (0x08,), check=_body_ok(bytes([0x01, 0xFF, 0x01, 0x00, 0x01])))
        _rdtci_case(tester, report, "0x02 mask 0x08 one record",
                    0x02, (0x08,), check=_n_records(1, 4, 2))
        _rdtci_case(tester, report, "0x02 mask 0x04 (pending latched) one record",
                    0x02, (0x04,), check=_n_records(1, 4, 2))
        _rdtci_case(tester, report, "0x03 snapshot identification one entry",
                    0x03, (), check=_n_records(1, 4, 1))
        _rdtci_case(tester, report, "0x04 snapshot record (DID 0xF500 + 4B data)",
                    0x04, p0217 + (0x01,), check=lambda b: (
                        (b[0] == 0x04 and b[1:4] == bytes(p0217) and (b[4] & 0x08) != 0 and
                         b[5] == 0x01 and b[6] == 0x01 and b[7:9] == b'\xF5\x00' and
                         len(b) == 13,
                         "snapshot payload OK" if len(b) == 13 else f"len {len(b)}")))
        _rdtci_case(tester, report, "0x05 stored data record 0x01",
                    0x05, (0x01,), check=_n_records(1, 12, 1))
        _rdtci_case(tester, report, "0x06 ext data record (occurrence=1)",
                    0x06, p0217 + (0x01,), check=lambda b: (
                        (b[0] == 0x06 and b[1:4] == bytes(p0217) and
                         b[5] == 0x01 and b[6] == 0x01 and len(b) == 7,
                         "occurrence 1" if len(b) == 7 else f"len {len(b)}")))
        _rdtci_case(tester, report, "0x07 count by severity+status = 1",
                    0x07, (0x80, 0x08,), check=_body_ok(bytes([0x07, 0xFF, 0x01, 0x00, 0x01])))
        _rdtci_case(tester, report, "0x08 by severity mask one record (sev 0x82)",
                    0x08, (0x80, 0x08,), check=lambda b: (
                        (b[0] == 0x08 and b[1] == 0xFF and b[2] == 0x82 and b[3] == 0x01 and
                         b[4:7] == bytes(p0217) and (b[7] & 0x08) != 0 and len(b) == 8,
                         "severity record OK" if len(b) == 8 else f"len {len(b)}")))
        _rdtci_case(tester, report, "0x09 severity info fixed 9B",
                    0x09, p0217, check=lambda b: (
                        (b[0] == 0x09 and b[1] == 0xFF and b[2] == 0x82 and b[3] == 0x01 and
                         b[4:7] == bytes(p0217) and (b[7] & 0x08) != 0 and len(b) == 8,
                         "9B payload OK" if len(b) == 8 else f"len {len(b)}")))
        for sub_pick, pick_name in ((0x0B, "firstTestFailed"), (0x0C, "firstConfirmed"),
                                    (0x0D, "mostRecentTestFailed"), (0x0E, "mostRecentConfirmed")):
            _rdtci_case(tester, report, f"0x{sub_pick:02X} {pick_name} -> P0217",
                        sub_pick, (), check=lambda b, s=sub_pick: (
                            (b[0] == s and b[1] == 0xFF and b[2:5] == bytes(p0217) and
                             len(b) == 6,
                             "single record OK" if len(b) == 6 else f"len {len(b)}")))
        _rdtci_case(tester, report, "0x14 FDC list empty (confirmed=127 excluded)",
                    0x14, (), check=_body_ok(bytes([0x14])))
        _rdtci_case(tester, report, "0x15 permanent one record",
                    0x15, (), check=_n_records(1, 4, 2))
        _rdtci_case(tester, report, "0x16 ext data by record one entry",
                    0x16, (0x01,), check=_n_records(1, 5, 2))
        _rdtci_case(tester, report, "0x17 userDefMemory list one record",
                    0x17, (0x08, 0x01), check=lambda b: (
                        (b[0] == 0x17 and b[1] == 0x01 and b[2] == 0xFF and
                         b[3:6] == bytes(p0217) and len(b) == 7,
                         "memSel echo + 1 record" if len(b) == 7 else f"len {len(b)}")))
        _rdtci_case(tester, report, "0x18 userDefMemory snapshot (memSel echo)",
                    0x18, p0217 + (0x01, 0x01), check=lambda b: (
                        (b[0] == 0x18 and b[1] == 0x01 and b[2:5] == bytes(p0217) and
                         b[8:10] == b'\xF5\x00' and len(b) == 14,
                         "snapshot OK" if len(b) == 14 else f"len {len(b)}")))
        _rdtci_case(tester, report, "0x19 userDefMemory ext data (memSel echo)",
                    0x19, p0217 + (0x01, 0x01), check=lambda b: (
                        (b[0] == 0x19 and b[1] == 0x01 and b[2:5] == bytes(p0217) and
                         b[6] == 0x01 and b[7] == 0x01 and len(b) == 8,
                         "occurrence 1" if len(b) == 8 else f"len {len(b)}")))
        _rdtci_case(tester, report, "0x42 WWH-OBD by mask one record",
                    0x42, (0x33, 0x08, 0x80,), check=lambda b: (
                        (b[0] == 0x42 and b[1] == 0x33 and b[2] == 0xFF and b[3] == 0xE2 and
                         b[4] == 0x04 and b[5] == 0x82 and b[6:9] == bytes(p0217) and
                         (b[9] & 0x08) != 0 and len(b) == 10,
                         "WWH record OK" if len(b) == 10 else f"len {len(b)}")))
        _rdtci_case(tester, report, "0x55 WWH-OBD permanent one record",
                    0x55, (0x33,), check=_n_records(1, 4, 4))

        # --- Phase 4: clear -> mirror memory retains archive ---
        tester.send_uds([0x04, 0x31, 0x01, 0x02, 0x01])  # RoutineControl 0x0201 DTC clear
        _rdtci_case(tester, report, "0x0A all idle again after clear",
                    0x0A, (), check=supported_dtc_idle)
        _rdtci_case(tester, report, "0x11 mirror count by mask 0x08 = 1",
                    0x11, (0x08,), check=_body_ok(bytes([0x11, 0xFF, 0x01, 0x00, 0x01])))
        _rdtci_case(tester, report, "0x0F mirror list one record",
                    0x0F, (0x08,), check=_n_records(1, 4, 2))
        _rdtci_case(tester, report, "0x10 mirror ext data (occurrence=1)",
                    0x10, p0217 + (0x01,), check=lambda b: (
                        (b[0] == 0x10 and b[1:4] == bytes(p0217) and
                         b[5] == 0x01 and b[6] == 0x01 and len(b) == 7,
                         "mirror occurrence 1" if len(b) == 7 else f"len {len(b)}")))

    # Return to Default session
    tester.send_uds([0x02, 0x10, 0x01])


def test_security_access(tester: UDSTester, report: TestReport):
    """SID 0x27: SecurityAccess Seed-Key test"""
    print(Color.header("SID 0x27: SecurityAccess"))

    # Switch to Extended session (required)
    tester.send_uds([0x02, 0x10, 0x03])
    time.sleep(0.05)

    # 1) Request seed (subFunction = 0x01)
    resp = tester.send_uds([0x02, 0x27, 0x01])
    result = TestResult("Seed request (0x27 0x01)")
    seed = None
    if resp:
        pos = tester.parse_positive(resp, 0x27)
        if pos and len(pos['payload']) >= 3:
            seed = (pos['payload'][1] << 8) | pos['payload'][2]
            result.status = TestStatus.PASS
            result.detail = f"Seed=0x{seed:04X}"
        else:
            neg = tester.parse_negative(resp)
            if neg:
                result.status = TestStatus.FAIL
                result.detail = f"NRC {neg['nrc_name']}"
            else:
                result.status = TestStatus.FAIL
                result.detail = "Parse failed"
        result.raw_rx = resp.hex().upper()
    else:
        result.status = TestStatus.FAIL
        result.detail = "Timeout"
    result.raw_tx = "02 27 01"
    print(f"  {result.status.value}: {result.detail}")
    report.add(result)

    if seed is None:
        # Cannot test key if seed not received
        report.add(TestResult("Key send (0x27 0x02)", TestStatus.SKIP, "Seed acquisition failed"))
        tester.send_uds([0x02, 0x10, 0x01])
        return

    # 2) Send correct key
    key = tester.compute_key(seed)
    key_hi = (key >> 8) & 0xFF
    key_lo = key & 0xFF
    resp = tester.send_uds([0x04, 0x27, 0x02, key_hi, key_lo])
    result = TestResult("Key send (0x27 0x02) - correct key")
    if resp:
        pos = tester.parse_positive(resp, 0x27)
        if pos:
            result.status = TestStatus.PASS
            result.detail = "Security unlock success"
        else:
            neg = tester.parse_negative(resp)
            if neg:
                result.status = TestStatus.FAIL
                result.detail = f"NRC {neg['nrc_name']}"
            else:
                result.status = TestStatus.FAIL
                result.detail = "Parse failed"
        result.raw_rx = resp.hex().upper()
    else:
        result.status = TestStatus.FAIL
        result.detail = "Timeout"
    result.raw_tx = f"04 27 02 {key_hi:02X} {key_lo:02X}"
    print(f"  {result.status.value}: {result.detail}")
    report.add(result)

    # 3) Send incorrect key (after re-requesting seed)
    resp = tester.send_uds([0x02, 0x27, 0x01])
    if resp:
        pos = tester.parse_positive(resp, 0x27)
        if pos and len(pos['payload']) >= 3:
            seed2 = (pos['payload'][1] << 8) | pos['payload'][2]
            # Intentionally wrong key
            wrong_key = seed2 ^ 0xFFFF
            wk_hi = (wrong_key >> 8) & 0xFF
            wk_lo = wrong_key & 0xFF
            resp2 = tester.send_uds([0x04, 0x27, 0x02, wk_hi, wk_lo])
            result = TestResult("Key send - incorrect key")
            if resp2:
                neg = tester.parse_negative(resp2)
                if neg and neg['nrc'] in (0x35, 0x36):
                    result.status = TestStatus.PASS
                    result.detail = f"NRC {neg['nrc_name']} (expected)"
                else:
                    result.status = TestStatus.FAIL
                    result.detail = "Expected NRC 0x35/0x36"
                result.raw_rx = resp2.hex().upper()
            else:
                result.status = TestStatus.FAIL
                result.detail = "Timeout"
            result.raw_tx = f"04 27 02 {wk_hi:02X} {wk_lo:02X}"
            print(f"  {result.status.value}: {result.detail}")
            report.add(result)

    # Return to Default
    tester.send_uds([0x02, 0x10, 0x01])


def test_routine_control(tester: UDSTester, report: TestReport):
    """SID 0x31: RoutineControl test"""
    print(Color.header("SID 0x31: RoutineControl"))

    # 1) Call from Default session -> NRC 0x33 (securityAccessDenied) expected
    tester.send_uds([0x02, 0x10, 0x01])
    time.sleep(0.05)
    resp = tester.send_uds([0x04, 0x31, 0x01, 0x02, 0x01])
    result = TestResult("RoutineControl from Default session")
    if resp:
        neg = tester.parse_negative(resp)
        if neg and neg['nrc'] == 0x33:
            result.status = TestStatus.PASS
            result.detail = f"NRC securityAccessDenied (expected)"
        else:
            result.status = TestStatus.FAIL
            result.detail = f"Expected NRC 0x33, got {neg['nrc_name'] if neg else 'positive'}"
        result.raw_rx = resp.hex().upper()
    else:
        result.status = TestStatus.FAIL
        result.detail = "Timeout"
    result.raw_tx = "04 31 01 02 01"
    print(f"  {result.status.value}: {result.detail}")
    report.add(result)

    # 2) Call after Extended + Security Unlock
    tester.send_uds([0x02, 0x10, 0x03])
    time.sleep(0.05)

    # Request seed
    resp = tester.send_uds([0x02, 0x27, 0x01])
    if resp:
        pos = tester.parse_positive(resp, 0x27)
        if pos and len(pos['payload']) >= 3:
            seed = (pos['payload'][1] << 8) | pos['payload'][2]
            key = tester.compute_key(seed)
            # Send key
            resp2 = tester.send_uds([0x04, 0x27, 0x02, (key >> 8) & 0xFF, key & 0xFF])
            if resp2:
                pos2 = tester.parse_positive(resp2, 0x27)
                if pos2:
                    # Now RoutineControl can be called
                    resp3 = tester.send_uds([0x04, 0x31, 0x01, 0x02, 0x01])
                    result = TestResult("RoutineControl after Extended+Unlock")
                    if resp3:
                        pos3 = tester.parse_positive(resp3, 0x31)
                        if pos3:
                            result.status = TestStatus.PASS
                            result.detail = "Normal response"
                        else:
                            neg3 = tester.parse_negative(resp3)
                            result.status = TestStatus.FAIL
                            result.detail = f"NRC {neg3['nrc_name']}" if neg3 else "Parse failed"
                        result.raw_rx = resp3.hex().upper()
                    else:
                        result.status = TestStatus.FAIL
                        result.detail = "Timeout"
                    result.raw_tx = "04 31 01 02 01"
                    print(f"  {result.status.value}: {result.detail}")
                    report.add(result)

    # Return to Default
    tester.send_uds([0x02, 0x10, 0x01])


def test_obd2_via_uds(tester: UDSTester, report: TestReport):
    """SID 0x01: Verify OBD-II PIDs also work via UDS path"""
    print(Color.header("SID 0x01: OBD-II via UDS"))

    pids = [
        (0x00, "Supported PID list"),
        (0x05, "Coolant temp"),
        (0x0C, "Engine RPM"),
        (0x0D, "Vehicle speed"),
    ]

    for pid, name in pids:
        resp = tester.send_uds([0x02, 0x01, pid])
        result = TestResult(f"PID 0x{pid:02X} ({name})")
        if resp:
            pos = tester.parse_positive(resp, 0x01)
            if pos and pos['payload'][0] == pid:
                result.status = TestStatus.PASS
                result.detail = "Normal response"
            else:
                neg = tester.parse_negative(resp)
                if neg:
                    result.status = TestStatus.FAIL
                    result.detail = f"NRC {neg['nrc_name']}"
                else:
                    result.status = TestStatus.FAIL
                    result.detail = "Parse failed"
            result.raw_rx = resp.hex().upper()
        else:
            result.status = TestStatus.FAIL
            result.detail = "Timeout"
        result.raw_tx = f"02 01 {pid:02X}"
        print(f"  {result.status.value}: PID 0x{pid:02X} ({name})")
        report.add(result)


#=======================================
# vcan setup helper
#=======================================
def setup_vcan(interface: str) -> bool:
    try:
        subprocess.run(['modprobe', 'vcan'], check=True, capture_output=True)
        subprocess.run(['ip', 'link', 'del', interface], capture_output=True, timeout=5)
        subprocess.run(['ip', 'link', 'add', 'dev', interface, 'type', 'vcan'],
                       check=True, capture_output=True, timeout=5)
        subprocess.run(['ip', 'link', 'set', interface, 'up'],
                       check=True, capture_output=True, timeout=5)
        return True
    except (subprocess.CalledProcessError, FileNotFoundError, subprocess.TimeoutExpired) as e:
        print(Color.fail_msg(f"vcan setup failed: {e}"))
        return False


def cleanup_vcan(interface: str):
    try:
        subprocess.run(['ip', 'link', 'set', interface, 'down'], capture_output=True, timeout=5)
        subprocess.run(['ip', 'link', 'del', interface], capture_output=True, timeout=5)
    except Exception:
        pass


#=======================================
# Main
#=======================================
def main():
    parser = argparse.ArgumentParser(description='UDS ECU Simulator Test')
    parser.add_argument('interface', nargs='?', default='vcan0', help='CAN interface')
    parser.add_argument('--bitrate', type=int, default=500000)
    parser.add_argument('--timeout', type=float, default=2.0)
    parser.add_argument('--setup-vcan', action='store_true', help='Auto setup vcan')
    args = parser.parse_args()

    print("=" * 50)
    print("  UDS ECU Simulator Test")
    print(f"  Interface: {args.interface}")
    print(f"  Date: {time.strftime('%Y-%m-%d %H:%M:%S')}")
    print("=" * 50)

    if args.setup_vcan:
        if not setup_vcan(args.interface):
            sys.exit(1)
        print(Color.pass_msg(f"vcan setup complete: {args.interface}"))

    report = TestReport()

    try:
        tester = UDSTester(args.interface, args.bitrate)
        tester.connect()
        time.sleep(0.3)

        # Run tests
        test_obd2_via_uds(tester, report)
        test_diag_session(tester, report)
        test_read_did(tester, report)
        test_security_access(tester, report)
        test_routine_control(tester, report)
        test_read_dtc_information(tester, report)

        tester.disconnect()

    except RuntimeError as e:
        print(Color.fail_msg(str(e)))
        sys.exit(1)
    except KeyboardInterrupt:
        print(f"\nUser interrupt")
        sys.exit(130)
    finally:
        if args.setup_vcan:
            cleanup_vcan(args.interface)

    all_passed = report.print_summary()
    sys.exit(0 if all_passed else 1)


if __name__ == '__main__':
    main()
