#!/usr/bin/env python3
"""
OBD-II ECU Simulator Test (python-can)

Description:
    Tests the OBD-II ECU simulator using the Linux SocketCAN interface.
    Uses the python-can library; also works with virtual CAN (vcan).

Usage:
    sudo python3 obd2_test.py [CAN interface]
    Example: sudo python3 obd2_test.py can0       # Real CAN adapter
             sudo python3 obd2_test.py vcan0      # virtual CAN (for testing)

Prerequisites:
    1. python-can installed: pip install python-can
    2. For vcan: sudo modprobe vcan && sudo ip link add dev vcan0 type vcan && sudo ip link set vcan0 up
    3. root permission required

OBD-II CAN ID reference:
    0x7E0 - OBD-II request (ECU1, tester -> ECU)
    0x7E8 - OBD-II response (ECU1, ECU -> tester)
    0x7DF - Broadcast request (all ECUs)

ISO-TP Single Frame (SF) structure:
    Request:  [PCI type=0x01][length][service ID=0x01][PID][padding...]
    Response: [PCI type=0x01][length][service ID=0x41][PID][data][padding...]
"""

import sys
import time
import argparse
import subprocess
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

# python-can import
try:
    import can
except ImportError:
    print("Error: python-can library is not installed.")
    print("Install: pip install python-can")
    sys.exit(1)

#=======================================
# Color output class
#=======================================
class Color:
    """Utility class for terminal color output"""
    RED = '\033[0;31m'
    GREEN = '\033[0;32m'
    YELLOW = '\033[1;33m'
    CYAN = '\033[0;36m'
    BOLD = '\033[1m'
    NC = '\033[0m'  # Color reset

    @staticmethod
    def pass_msg(msg: str) -> str:
        return f"{Color.GREEN}[PASS]{Color.NC} {msg}"

    @staticmethod
    def fail_msg(msg: str) -> str:
        return f"{Color.RED}[FAIL]{Color.NC} {msg}"

    @staticmethod
    def skip_msg(msg: str) -> str:
        return f"{Color.YELLOW}[SKIP]{Color.NC} {msg}"

    @staticmethod
    def info_msg(msg: str) -> str:
        return f"{Color.CYAN}[INFO]{Color.NC} {msg}"

    @staticmethod
    def header(msg: str) -> str:
        return f"\n{Color.CYAN}=== {msg} ==={Color.NC}"


#=======================================
# Test result data classes
#=======================================
class TestStatus(Enum):
    PASS = "PASS"
    FAIL = "FAIL"
    SKIP = "SKIP"


@dataclass
class TestResult:
    """Individual test result"""
    name: str
    status: TestStatus
    detail: str = ""
    raw_data: str = ""
    decoded_value: str = ""


@dataclass
class TestReport:
    """Overall test report"""
    results: list = field(default_factory=list)

    def add(self, result: TestResult):
        self.results.append(result)

    def print_table(self):
        """Print test results in table format"""
        print("\n" + "=" * 78)
        print(f"  {'Test Item':<20} {'Status':<8} {'Decoded Value':<16} {'Detail':<30}")
        print("=" * 78)

        for r in self.results:
            # Select color based on status
            if r.status == TestStatus.PASS:
                status_str = Color.pass_msg(r.status.value)
            elif r.status == TestStatus.FAIL:
                status_str = Color.fail_msg(r.status.value)
            else:
                status_str = Color.skip_msg(r.status.value)

            # Truncate long detail
            detail = r.detail[:28] + ".." if len(r.detail) > 30 else r.detail

            print(f"  {r.name:<20} {status_str:<28} {r.decoded_value:<16} {detail}")
            if r.raw_data:
                print(f"  {'':>20}   Received frame: {r.raw_data}")

        print("=" * 78)

        # Summary statistics
        pass_count = sum(1 for r in self.results if r.status == TestStatus.PASS)
        fail_count = sum(1 for r in self.results if r.status == TestStatus.FAIL)
        skip_count = sum(1 for r in self.results if r.status == TestStatus.SKIP)
        total = len(self.results)

        print(f"  Total tests: {total}  |  ", end="")
        print(f"{Color.GREEN}Passed: {pass_count}{Color.NC}  |  ", end="")
        print(f"{Color.RED}Failed: {fail_count}{Color.NC}  |  ", end="")
        print(f"{Color.YELLOW}Skipped: {skip_count}{Color.NC}")
        print("=" * 78)

        if fail_count > 0:
            print(f"  Result: {Color.RED}Some tests failed{Color.NC}")
        else:
            print(f"  Result: {Color.GREEN}All tests passed{Color.NC}")

        return fail_count == 0


#=======================================
# OBD-II PID definitions
#=======================================
@dataclass
class OBD2PID:
    """OBD-II PID definition"""
    pid: int
    name: str
    description: str
    decode_func: Optional[callable] = None


def decode_coolant_temp(data: bytes) -> str:
    """
    PID 0x05: Coolant temperature decode
    Formula: Temperature(C) = A - 40
    Range: -40C ~ 215C
    """
    raw = data[0]
    temp_c = raw - 40
    return f"{temp_c} C (raw: 0x{raw:02X})"


def decode_rpm(data: bytes) -> str:
    """
    PID 0x0C: Engine RPM decode
    Formula: RPM = ((A * 256) + B) / 4
    Range: 0 ~ 16383.75 RPM
    """
    raw = (data[0] << 8) | data[1]
    rpm = raw / 4.0
    return f"{rpm:.0f} RPM (raw: 0x{data[0]:02X}{data[1]:02X})"


def decode_vehicle_speed(data: bytes) -> str:
    """
    PID 0x0D: Vehicle speed decode
    Formula: Speed(km/h) = A
    Range: 0 ~ 255 km/h
    """
    speed = data[0]
    return f"{speed} km/h (raw: 0x{data[0]:02X})"


def decode_supported_pids(data: bytes) -> str:
    """
    PID 0x00: Supported PID bitmap decode
    4-byte bitmap representing support status of PID 0x01 ~ 0x20
    """
    bitmap = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]
    supported = []
    pid_names = {
        0x01: "Monitor Status",
        0x03: "Fuel System Status",
        0x04: "Engine Load",
        0x05: "Coolant Temp",
        0x06: "Short Term Fuel Trim",
        0x07: "Long Term Fuel Trim",
        0x0B: "MAP",
        0x0C: "RPM",
        0x0D: "Speed",
        0x0E: "Timing Advance",
        0x0F: "Intake Temp",
        0x10: "MAF Rate",
        0x11: "Throttle Position",
        0x1F: "Run Time",
        0x20: "Distance with MIL",
    }
    for pid_bit, pid_name in pid_names.items():
        if bitmap & (1 << (32 - pid_bit)):
            supported.append(f"0x{pid_bit:02X}({pid_name})")
    return f"Supported PIDs: {', '.join(supported[:5])}{'...' if len(supported) > 5 else ''}"


# PID list to test
TEST_PIDS = [
    OBD2PID(0x00, "Supported PID list", "PID bitmap query", decode_supported_pids),
    OBD2PID(0x05, "Coolant temp", "PID 0x05 decode", decode_coolant_temp),
    OBD2PID(0x0C, "Engine RPM", "PID 0x0C decode", decode_rpm),
    OBD2PID(0x0D, "Vehicle speed", "PID 0x0D decode", decode_vehicle_speed),
]


#=======================================
# OBD-II communication class
#=======================================
class OBD2Tester:
    """
    OBD-II ECU simulator test class

    Sends OBD-II requests on the CAN bus using python-can and receives responses.
    Uses ISO-TP Single Frame (SF) protocol.
    """

    # OBD-II CAN IDs
    REQUEST_ID = 0x7E0    # tester -> ECU1
    RESPONSE_ID = 0x7E8   # ECU1 -> tester
    BROADCAST_ID = 0x7DF  # Broadcast request

    # OBD-II service IDs
    SERVICE_SHOW_CURRENT_DATA = 0x01
    SERVICE_RESPONSE_OFFSET = 0x40  # Response service = request service + 0x40

    # ISO-TP PCI type (upper nibble, ISO 15765-2): SF=0x00, FF=0x10, CF=0x20, FC=0x30
    ISO_TP_PCI_SF = 0x00

    def __init__(self, interface: str, bitrate: int = 500000):
        """
        Initialize OBD-II tester

        Args:
            interface: CAN interface name (e.g. "can0", "vcan0")
            bitrate: CAN communication speed (default: 500kbps)
        """
        self.interface = interface
        self.bitrate = bitrate
        self.bus: Optional[can.Bus] = None

    def connect(self):
        """
        Connect to CAN bus.

        No additional setup needed for virtual CAN (vcan).
        Bitrate configuration may be needed for real CAN interfaces.
        """
        try:
            self.bus = can.Bus(
                interface='socketcan',
                channel=self.interface,
                bitrate=self.bitrate,
                fd=True,   # Receive CAN-FD(BRS) responses -- firmware always transmits FD
            )
            print(Color.info_msg(f"CAN bus connected: {self.interface}"))
            print(Color.info_msg(f"  Bus state: {self.bus.state}"))
        except can.CanError as e:
            raise RuntimeError(f"CAN bus connection failed ({self.interface}): {e}") from e

    def disconnect(self):
        """Disconnect from CAN bus."""
        if self.bus:
            self.bus.shutdown()
            self.bus = None
            print(Color.info_msg(f"CAN bus disconnected: {self.interface}"))

    def send_service_request(self, service_id: int, payload=(),
                             can_id: Optional[int] = None,
                             timeout: float = 2.0) -> Optional[can.Message]:
        """
        Send an OBD-II/UDS service request as ISO-TP Single Frame (SF) and receive response.

        Standard ISO 15765-2 SF: byte0 = 0x0L (lower nibble = payload length),
        byte1 = service ID, byte2.. = payload (PID etc.). Padded to 8 bytes with 0x00.

        Args:
            service_id: Service ID (e.g. 0x01=Mode01, 0x03, 0x09, 0x10)
            payload:    Payload bytes after service byte (PID, INF type, etc.)
            can_id:     Request CAN ID (default 0x7E0 physical, 0x7DF=functional broadcast)
            timeout:    Response wait time (seconds)

        Returns:
            Received CAN message or None (on timeout)
        """
        if not self.bus:
            raise RuntimeError("Not connected to CAN bus")
        if can_id is None:
            can_id = self.REQUEST_ID

        payload = bytes(payload)
        length = 1 + len(payload)  # service byte + payload
        if length > 7:
            raise ValueError(f"SF payload exceeded ({length}>7) -- multi-frame needed (unsupported)")

        # Standard SF: [length, service, payload, 0x00 padding...]
        request_data = bytearray([length & 0x0F, service_id])
        request_data.extend(payload)
        request_data.extend(b'\x00' * (8 - len(request_data)))

        request_msg = can.Message(
            arbitration_id=can_id,
            data=bytes(request_data[:8]),
            is_extended_id=False,
        )

        try:
            self.bus.send(request_msg)
        except can.CanError as e:
            print(Color.fail_msg(f"Request transmit failed (SID 0x{service_id:02X}): {e}"))
            return None

        try:
            return self.bus.recv(timeout=timeout)
        except can.CanError as e:
            print(Color.fail_msg(f"Response receive error (SID 0x{service_id:02X}): {e}"))
            return None

    def send_request(self, pid: int, timeout: float = 2.0) -> Optional[can.Message]:
        """Mode 01 PID request -- thin wrapper around send_service_request(0x01, [pid])."""
        return self.send_service_request(
            self.SERVICE_SHOW_CURRENT_DATA, (pid,), self.REQUEST_ID, timeout
        )

    def _drain_rx(self):
        """Drain leftover frames from the socket buffer (e.g. before response suppression test)."""
        if not self.bus:
            return
        while True:
            if self.bus.recv(timeout=0.0) is None:
                break

    @staticmethod
    def parse_sf_response(msg: Optional[can.Message],
                          expected_service: Optional[int] = None) -> Optional[dict]:
        """
        Parse ISO-TP Single Frame (SF) response into standard layout.

        Standard SF: byte0 = 0x0L (lower nibble=length), byte1 = service ID, byte2.. = payload.
        Also handles CAN-FD escape SF (byte0=0x00, byte1=length). Recognizes negative response (0x7F).

        Returns:
            {service_id, payload: bytes, raw: str, neg: bool, rejected_sid, nrc}
            If expected_service is given, returns None when positive response (sid|0x40) mismatches.
        """
        if not msg or not msg.data:
            return None

        data = bytes(msg.data)
        if len(data) < 2:
            return None

        # SF length extraction (classic / escape)
        if (data[0] & 0x0F) == 0x00 and len(data) >= 3:
            length = data[1]
            body = data[2:2 + length]
        else:
            length = data[0] & 0x0F
            body = data[1:1 + length]

        if len(body) < 1:
            return None

        service_id = body[0]
        payload = bytes(body[1:])
        raw = ''.join(f'{b:02X}' for b in data)

        # Negative response: [0x7F, rejected_sid, nrc]
        if service_id == 0x7F:
            return {
                'service_id': 0x7F,
                'payload': payload,
                'raw': raw,
                'neg': True,
                'rejected_sid': body[1] if len(body) > 1 else None,
                'nrc': body[2] if len(body) > 2 else None,
            }

        # Positive response verification
        if expected_service is not None and service_id != (expected_service | 0x40):
            return None

        return {
            'service_id': service_id,
            'payload': payload,
            'raw': raw,
            'neg': False,
            'rejected_sid': None,
            'nrc': None,
        }

    def parse_response(self, msg: Optional[can.Message]) -> Optional[dict]:
        """Mode 01 response parse -- wrapper around parse_sf_response(expected_service=0x01).
        Returns {service_id, pid, data, raw} for compatibility with existing test_pid/ramp/stress."""
        sf = self.parse_sf_response(msg, expected_service=self.SERVICE_SHOW_CURRENT_DATA)
        if sf is None or sf['neg']:
            return None
        payload = sf['payload']
        if len(payload) < 1:
            return None
        return {
            'service_id': sf['service_id'],
            'pid': payload[0],
            'data': bytes(payload[1:]),
            'raw': sf['raw'],
        }

    def test_pid(self, obd2_pid: OBD2PID, timeout: float = 2.0) -> TestResult:
        """
        Test a single PID.

        Args:
            obd2_pid: OBD2PID object to test
            timeout: Response wait time (seconds)

        Returns:
            TestResult object
        """
        # Send OBD-II request
        response_msg = self.send_request(obd2_pid.pid, timeout)

        if response_msg is None:
            return TestResult(
                name=f"PID 0x{obd2_pid.pid:02X} ({obd2_pid.name})",
                status=TestStatus.FAIL,
                detail=f"No response (timeout {timeout}s)",
            )

        # Parse response
        parsed = self.parse_response(response_msg)

        if parsed is None:
            return TestResult(
                name=f"PID 0x{obd2_pid.pid:02X} ({obd2_pid.name})",
                status=TestStatus.FAIL,
                detail=f"Response parse failed",
                raw_data=response_msg.data.hex().upper(),
            )

        # Verify PID match
        if parsed['pid'] != obd2_pid.pid:
            return TestResult(
                name=f"PID 0x{obd2_pid.pid:02X} ({obd2_pid.name})",
                status=TestStatus.FAIL,
                detail=f"PID mismatch: request 0x{obd2_pid.pid:02X}, response 0x{parsed['pid']:02X}",
                raw_data=parsed['raw'],
            )

        # Decode data
        decoded = ""
        if obd2_pid.decode_func and parsed['data']:
            try:
                decoded = obd2_pid.decode_func(parsed['data'])
            except (IndexError, ValueError) as e:
                return TestResult(
                    name=f"PID 0x{obd2_pid.pid:02X} ({obd2_pid.name})",
                    status=TestStatus.FAIL,
                    detail=f"Decode error: {e}",
                    raw_data=parsed['raw'],
                )

        return TestResult(
            name=f"PID 0x{obd2_pid.pid:02X} ({obd2_pid.name})",
            status=TestStatus.PASS,
            detail=obd2_pid.description,
            raw_data=parsed['raw'],
            decoded_value=decoded,
        )

    # =============================================
    # OBD-II Mode 03/04/07/09 + Functional (0x7DF)
    # =============================================
    def _run_sf_test(self, name: str, service_id: int, request_payload=(),
                     can_id: Optional[int] = None, expected_service: Optional[int] = None,
                     expect_no_response: bool = False, expected_nrc: Optional[int] = None,
                     timeout: float = 2.0, extra_check=None) -> TestResult:
        """Common verification logic for SF single-frame service requests."""
        if can_id is None:
            can_id = self.REQUEST_ID

        # Response suppression case: drain leftover frames and expect timeout (no response)
        if expect_no_response:
            self._drain_rx()
            msg = self.send_service_request(service_id, request_payload, can_id, timeout)
            if msg is None:
                return TestResult(name=name, status=TestStatus.PASS,
                                  detail="No response (suppression verified)")
            raw = msg.data.hex().upper() if msg.data else ""
            return TestResult(name=name, status=TestStatus.FAIL,
                              detail=f"Should be suppressed but received 0x{msg.arbitration_id:03X}",
                              raw_data=raw)

        msg = self.send_service_request(service_id, request_payload, can_id, timeout)
        if msg is None:
            return TestResult(name=name, status=TestStatus.FAIL,
                              detail=f"No response (timeout {timeout}s)")

        sf = self.parse_sf_response(msg, expected_service=expected_service)
        if sf is None:
            return TestResult(name=name, status=TestStatus.FAIL,
                              detail="Response parse failed",
                              raw_data=msg.data.hex().upper())

        # NRC expected case
        if expected_nrc is not None:
            if not sf['neg'] or sf['nrc'] != expected_nrc:
                got = f"0x{sf['nrc']:02X}" if sf['neg'] else "positive"
                return TestResult(name=name, status=TestStatus.FAIL,
                                  detail=f"Expected NRC 0x{expected_nrc:02X}, received {got}",
                                  raw_data=sf['raw'])
            return TestResult(name=name, status=TestStatus.PASS,
                              detail=f"NRC 0x{expected_nrc:02X} (sid 0x{sf['rejected_sid']:02X})",
                              raw_data=sf['raw'])

        # Positive response -- negative response is failure
        if sf['neg']:
            return TestResult(name=name, status=TestStatus.FAIL,
                              detail=f"Negative response NRC 0x{sf['nrc']:02X}",
                              raw_data=sf['raw'])

        if extra_check is not None:
            ok, detail = extra_check(msg, sf)
            return TestResult(name=name,
                              status=TestStatus.PASS if ok else TestStatus.FAIL,
                              detail=detail, raw_data=sf['raw'])

        return TestResult(name=name, status=TestStatus.PASS,
                          detail=f"service 0x{sf['service_id']:02X}",
                          raw_data=sf['raw'])

    def test_mode03_stored_dtc(self, timeout: float = 2.0) -> TestResult:
        """Mode 0x03 Stored DTC: [0x01,0x03] -> [0x02,0x43,0x00] (0 DTCs)."""
        return self._run_sf_test(
            "Mode 03 Stored DTC", 0x03, expected_service=0x03, timeout=timeout,
            extra_check=lambda m, s: (
                s['payload'] == b'\x00',
                f"numDTC=0 ({s['payload'].hex().upper() or 'empty'})"))

    def test_mode04_clear_dtc(self, timeout: float = 2.0) -> TestResult:
        """Mode 0x04 Clear DTC: [0x01,0x04] -> [0x01,0x44]."""
        return self._run_sf_test(
            "Mode 04 Clear DTC", 0x04, expected_service=0x04, timeout=timeout,
            extra_check=lambda m, s: (
                len(s['payload']) == 0,
                f"length {len(s['payload'])} (empty response)"))

    def test_mode07_pending_dtc(self, timeout: float = 2.0) -> TestResult:
        """Mode 0x07 Pending DTC: [0x01,0x07] -> [0x01,0x47] (no numDTC byte)."""
        return self._run_sf_test(
            "Mode 07 Pending DTC", 0x07, expected_service=0x07, timeout=timeout,
            extra_check=lambda m, s: (
                len(s['payload']) == 0,
                f"length {len(s['payload'])} (no numDTC byte)"))

    def test_mode09_inf_support(self, timeout: float = 2.0) -> TestResult:
        """Mode 0x09 INF 0x00: [0x02,0x09,0x00] -> [0x06,0x49,0x00,0x02,...] (INF0x02=VIN supported)."""
        return self._run_sf_test(
            "Mode 09 INF Support List", 0x09, request_payload=b'\x00',
            expected_service=0x09, timeout=timeout,
            extra_check=lambda m, s: (
                len(s['payload']) >= 2 and (s['payload'][1] & 0x02) != 0,
                f"INF bitmap: {s['payload'].hex().upper()}"))

    def test_functional_response(self, timeout: float = 2.0) -> TestResult:
        """0x7DF functional Mode 01 PID 0x0C -> response ID 0x7E8 (not 0x7E7) mapping verification."""
        def check(m, s):
            if m.arbitration_id != self.RESPONSE_ID:
                return (False, f"Response ID 0x{m.arbitration_id:03X} (expected 0x{self.RESPONSE_ID:03X})")
            if not s['payload'] or s['payload'][0] != 0x0C:
                return (False, "PID mismatch")
            return (True, "0x7DF->0x7E8 mapping OK, PID 0x0C")
        return self._run_sf_test(
            "Functional 0x7DF Response Mapping", 0x01, request_payload=b'\x0C',
            can_id=self.BROADCAST_ID, expected_service=0x01,
            timeout=timeout, extra_check=check)

    def test_functional_suppressed(self, timeout: float = 0.6) -> TestResult:
        """0x7DF functional SID 0x10(session control) -> response suppressed (UDS does not support functional)."""
        return self._run_sf_test(
            "Functional 0x7DF SID 0x10 Suppressed", 0x10, request_payload=b'\x01',
            can_id=self.BROADCAST_ID, expect_no_response=True, timeout=timeout)

    def test_unsupported_mode_nrc(self, timeout: float = 2.0) -> TestResult:
        """Physical Mode 0x02(unimplemented) -> NRC 0x7F/0x02/0x11 (serviceNotSupported)."""
        return self._run_sf_test(
            "Unsupported Mode 0x02 NRC", 0x02, expected_nrc=0x11, timeout=timeout)

    def test_ramp_up_down(self,
                           pid: int = 0x0C,
                           samples: int = 20,
                           interval: float = 0.5,
                           timeout: float = 2.0) -> TestResult:
        """
        Ramp up/down simulation verification

        Checks whether the ECU simulator follows a ramp up/down pattern.
        Samples the specified PID multiple times to verify gradual value changes.

        Args:
            pid: PID to monitor (default: 0x0C = RPM)
            samples: Number of samples (default: 20)
            interval: Sampling interval (seconds, default: 0.5)
            timeout: Response wait time (seconds)

        Returns:
            TestResult object
        """
        values = []
        timestamps = []

        print(Color.info_msg(
            f"Ramp test start: PID 0x{pid:02X}, "
            f"{samples} samples, interval {interval}s"
        ))

        for i in range(samples):
            start = time.time()

            response_msg = self.send_request(pid, timeout)
            parsed = self.parse_response(response_msg) if response_msg else None

            elapsed = time.time() - start

            if parsed and parsed['data']:
                # RPM decode: (A*256 + B) / 4
                if pid == 0x0C and len(parsed['data']) >= 2:
                    raw = (parsed['data'][0] << 8) | parsed['data'][1]
                    rpm = raw / 4.0
                    values.append(rpm)
                else:
                    values.append(parsed['data'][0])
                timestamps.append(elapsed)
                print(f"    [{i+1:3d}/{samples}] RPM = {values[-1]:8.0f}  ({elapsed:.3f}s)")
            else:
                print(f"    [{i+1:3d}/{samples}] No response  ({elapsed:.3f}s)")

            # Wait for sampling interval (accounting for response time)
            remaining = interval - elapsed
            if remaining > 0:
                time.sleep(remaining)

        # Result analysis
        if len(values) < 3:
            return TestResult(
                name=f"Ramp test (PID 0x{pid:02X})",
                status=TestStatus.FAIL,
                detail=f"Insufficient valid samples: {len(values)}/{samples}",
            )

        min_val = min(values)
        max_val = max(values)
        avg_val = sum(values) / len(values)

        # Detect direction changes
        direction_changes = 0
        prev_direction = 0  # 1=rising, -1=falling, 0=same
        for i in range(1, len(values)):
            if values[i] > values[i-1]:
                curr = 1
            elif values[i] < values[i-1]:
                curr = -1
            else:
                curr = 0

            if curr != 0 and prev_direction != 0 and curr != prev_direction:
                direction_changes += 1

            if curr != 0:
                prev_direction = curr

        # Check value variation
        if min_val == max_val:
            status = TestStatus.FAIL
            detail = f"No value change (all samples at {min_val:.0f})"
        else:
            status = TestStatus.PASS
            detail = (
                f"Range: {min_val:.0f} ~ {max_val:.0f}, "
                f"Average: {avg_val:.0f}, "
                f"Direction changes: {direction_changes}"
            )

        # Print sample values
        values_str = ", ".join(f"{v:.0f}" for v in values)
        print(Color.info_msg(f"Sampled values ({len(values)} samples): [{values_str}]"))
        print(Color.info_msg(
            f"Statistics: min={min_val:.0f}, max={max_val:.0f}, "
            f"avg={avg_val:.0f}, direction changes={direction_changes}"
        ))

        return TestResult(
            name=f"Ramp test (PID 0x{pid:02X})",
            status=status,
            detail=detail,
            decoded_value=f"{len(values)} samples",
        )

    def test_stress(self,
                    pid: int = 0x0D,
                    count: int = 10,
                    timeout: float = 2.0) -> list:
        """
        Repeated stress test

        Sends multiple requests for the specified PID to verify response stability.

        Args:
            pid: PID to test (default: 0x0D = vehicle speed)
            count: Number of iterations (default: 10)
            timeout: Response wait time (seconds)

        Returns:
            TestResult list
        """
        results = []
        print(Color.info_msg(f"Stress test start: PID 0x{pid:02X}, {count} iterations"))

        for i in range(count):
            start = time.time()
            response_msg = self.send_request(pid, timeout)
            elapsed = time.time() - start

            if response_msg is None:
                results.append(TestResult(
                    name=f"Stress [{i+1}/{count}]",
                    status=TestStatus.FAIL,
                    detail=f"No response ({elapsed:.3f}s)",
                ))
                print(f"    [{i+1:3d}/{count}] {Color.RED}FAIL{Color.NC} No response ({elapsed:.3f}s)")
                continue

            parsed = self.parse_response(response_msg)
            if parsed and parsed['pid'] == pid:
                results.append(TestResult(
                    name=f"Stress [{i+1}/{count}]",
                    status=TestStatus.PASS,
                    detail=f"Normal response ({elapsed:.3f}s)",
                    raw_data=parsed['raw'],
                ))
                print(f"    [{i+1:3d}/{count}] {Color.GREEN}PASS{Color.NC} ({elapsed:.3f}s) {parsed['raw']}")
            else:
                results.append(TestResult(
                    name=f"Stress [{i+1}/{count}]",
                    status=TestStatus.FAIL,
                    detail=f"Response error ({elapsed:.3f}s)",
                    raw_data=response_msg.data.hex().upper() if response_msg else "",
                ))
                print(f"    [{i+1:3d}/{count}] {Color.RED}FAIL{Color.NC} ({elapsed:.3f}s)")

        # Stress test summary
        pass_count = sum(1 for r in results if r.status == TestStatus.PASS)
        fail_count = sum(1 for r in results if r.status == TestStatus.FAIL)
        print(Color.info_msg(
            f"Stress test results: {Color.GREEN}{pass_count} passed{Color.NC}, "
            f"{Color.RED}{fail_count} failed{Color.NC} (total: {count})"
        ))

        return results


#=======================================
# CAN interface setup helpers
#=======================================
def setup_vcan(interface: str) -> bool:
    """
    Set up virtual CAN interface.

    Args:
        interface: vcan interface name (e.g. "vcan0")

    Returns:
        Whether setup succeeded
    """
    try:
        # Load vcan kernel module
        subprocess.run(['modprobe', 'vcan'], check=True, capture_output=True)

        # Delete existing interface if present
        subprocess.run(
            ['ip', 'link', 'del', interface],
            capture_output=True, timeout=5
        )

        # Create vcan interface
        subprocess.run(
            ['ip', 'link', 'add', 'dev', interface, 'type', 'vcan'],
            check=True, capture_output=True, timeout=5
        )

        # Activate interface
        subprocess.run(
            ['ip', 'link', 'set', interface, 'up'],
            check=True, capture_output=True, timeout=5
        )

        return True
    except (subprocess.CalledProcessError, FileNotFoundError, subprocess.TimeoutExpired) as e:
        print(Color.fail_msg(f"vcan setup failed: {e}"))
        return False


def cleanup_vcan(interface: str):
    """
    Clean up virtual CAN interface.

    Args:
        interface: vcan interface name
    """
    try:
        subprocess.run(
            ['ip', 'link', 'set', interface, 'down'],
            capture_output=True, timeout=5
        )
        subprocess.run(
            ['ip', 'link', 'del', interface],
            capture_output=True, timeout=5
        )
    except (subprocess.CalledProcessError, FileNotFoundError, subprocess.TimeoutExpired):
        pass


#=======================================
# Main function
#=======================================
def main():
    """Main test execution"""
    parser = argparse.ArgumentParser(
        description='OBD-II ECU Simulator Test (python-can)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Usage examples:
  sudo python3 obd2_test.py can0           # Real CAN adapter
  sudo python3 obd2_test.py vcan0          # virtual CAN
  sudo python3 obd2_test.py vcan0 --stress 50   # Stress test 50 iterations
  sudo python3 obd2_test.py vcan0 --ramp-samples 30 --ramp-interval 0.3
  sudo python3 obd2_test.py vcan0 --no-modes     # Omit Mode 03/04/07/09+functional
        """,
    )
    parser.add_argument(
        'interface',
        nargs='?',
        default='can0',
        help='CAN interface name (default: can0)',
    )
    parser.add_argument(
        '--bitrate',
        type=int,
        default=500000,
        help='CAN bitrate (default: 500000)',
    )
    parser.add_argument(
        '--timeout',
        type=float,
        default=2.0,
        help='Response timeout (seconds, default: 2.0)',
    )
    parser.add_argument(
        '--stress',
        type=int,
        default=10,
        help='Stress test iteration count (default: 10)',
    )
    parser.add_argument(
        '--ramp-samples',
        type=int,
        default=20,
        help='Ramp test sample count (default: 20)',
    )
    parser.add_argument(
        '--ramp-interval',
        type=float,
        default=0.5,
        help='Ramp test sampling interval (seconds, default: 0.5)',
    )
    parser.add_argument(
        '--no-ramp',
        action='store_true',
        help='Skip ramp up/down test',
    )
    parser.add_argument(
        '--no-stress',
        action='store_true',
        help='Skip stress test',
    )
    parser.add_argument(
        '--no-modes',
        action='store_true',
        help='Skip Mode 03/04/07/09 + functional(0x7DF) tests',
    )
    parser.add_argument(
        '--setup-vcan',
        action='store_true',
        help='Auto setup and cleanup vcan interface',
    )

    args = parser.parse_args()

    # Print header
    print("=" * 50)
    print("  OBD-II ECU Simulator Test (python-can)")
    print(f"  Interface: {args.interface}")
    print(f"  Bitrate: {args.bitrate} bps")
    print(f"  Timeout: {args.timeout}s")
    print(f"  Date: {time.strftime('%Y-%m-%d %H:%M:%S')}")
    print("=" * 50)

    # vcan auto setup
    if args.setup_vcan:
        print(Color.header("Virtual CAN Setup"))
        if setup_vcan(args.interface):
            print(Color.pass_msg(f"vcan interface setup complete: {args.interface}"))
        else:
            print(Color.fail_msg("vcan interface setup failed"))
            sys.exit(1)

    # Create test report
    report = TestReport()

    try:
        # Connect to CAN bus
        tester = OBD2Tester(args.interface, args.bitrate)
        tester.connect()

        # Wait for ECU simulator response
        print(Color.info_msg("Waiting for ECU simulator response..."))
        time.sleep(0.5)

        # =============================================
        # 1. Individual PID tests
        # =============================================
        print(Color.header("Individual PID Tests"))

        for obd2_pid in TEST_PIDS:
            result = tester.test_pid(obd2_pid, args.timeout)
            report.add(result)

            if result.status == TestStatus.PASS:
                print(f"  {Color.pass_msg(result.name)}")
                if result.decoded_value:
                    print(f"    {result.decoded_value}")
            else:
                print(f"  {Color.fail_msg(result.name)} - {result.detail}")
                if result.raw_data:
                    print(f"    Received frame: {result.raw_data}")

        # =============================================
        # 2. OBD-II Mode 03/04/07/09 + Functional (0x7DF)
        # =============================================
        if not args.no_modes:
            print(Color.header("OBD-II Mode 03/04/07/09 + Functional (0x7DF)"))
            mode_tests = [
                tester.test_mode03_stored_dtc(args.timeout),
                tester.test_mode04_clear_dtc(args.timeout),
                tester.test_mode07_pending_dtc(args.timeout),
                tester.test_mode09_inf_support(args.timeout),
                tester.test_functional_response(args.timeout),
                tester.test_functional_suppressed(),
                tester.test_unsupported_mode_nrc(args.timeout),
            ]
            for result in mode_tests:
                report.add(result)
                if result.status == TestStatus.PASS:
                    print(f"  {Color.pass_msg(result.name)} - {result.detail}")
                else:
                    print(f"  {Color.fail_msg(result.name)} - {result.detail}")
                    if result.raw_data:
                        print(f"    Received frame: {result.raw_data}")

        # =============================================
        # 3. Ramp up/down simulation verification
        # =============================================
        if not args.no_ramp:
            print(Color.header("Ramp Up/Down Simulation Verification"))
            ramp_result = tester.test_ramp_up_down(
                pid=0x0C,  # RPM
                samples=args.ramp_samples,
                interval=args.ramp_interval,
                timeout=args.timeout,
            )
            report.add(ramp_result)

            if ramp_result.status == TestStatus.PASS:
                print(f"  {Color.pass_msg(ramp_result.name)} - {ramp_result.detail}")
            else:
                print(f"  {Color.fail_msg(ramp_result.name)} - {ramp_result.detail}")

        # =============================================
        # 4. Stress test
        # =============================================
        if not args.no_stress:
            print(Color.header("Stress Test"))
            stress_results = tester.test_stress(
                pid=0x0D,  # Vehicle speed
                count=args.stress,
                timeout=args.timeout,
            )
            report.results.extend(stress_results)

        # Disconnect from CAN bus
        tester.disconnect()

    except RuntimeError as e:
        print(Color.fail_msg(str(e)))
        sys.exit(1)
    except KeyboardInterrupt:
        print(f"\n{Color.YELLOW}User interrupt{Color.NC}")
        sys.exit(130)
    finally:
        # vcan cleanup
        if args.setup_vcan:
            cleanup_vcan(args.interface)
            print(Color.info_msg(f"vcan interface cleaned up: {args.interface}"))

    # Print results
    all_passed = report.print_table()
    sys.exit(0 if all_passed else 1)


if __name__ == '__main__':
    main()
