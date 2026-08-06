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
    status: TestStatus
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

    def __init__(self, interface: str, bitrate: int = 500000):
        self.interface = interface
        self.bitrate = bitrate
        self.bus: Optional[can.Bus] = None

    def connect(self):
        self.bus = can.Bus(
            interface='socketcan',
            channel=self.interface,
            bitrate=self.bitrate,
        )
        print(Color.info_msg(f"CAN connected: {self.interface}"))

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
        """Parse positive response: verify SID+0x40"""
        if not data or len(data) < 2:
            return None
        if data[1] != expected_sid + 0x40:
            return None
        return {'sid_resp': data[1], 'payload': data[2:]}

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
