#!/usr/bin/env python3
"""
UDS (Unified Diagnostic Services) ECU 시뮬레이터 테스트

설명:
    UDS 서비스 (SID 0x10, 0x11, 0x22, 0x27, 0x31)를 테스트합니다.
    ISO-TP 단일 프레임 프로토콜 위에서 동작합니다.

사용법:
    sudo python3 uds_test.py [CAN 인터페이스]
    예: sudo python3 uds_test.py vcan0 --setup-vcan

UDS CAN ID:
    0x7E0 - 요청 (테스터 -> ECU)
    0x7E8 - 응답 (ECU -> 테스터)

UDS 서비스:
    0x10 - DiagnosticSessionControl
    0x11 - ECU Reset
    0x22 - ReadDataByIdentifier
    0x27 - SecurityAccess
    0x31 - RoutineControl

ISO-TP 단일 프레임 (SF) 구조:
    요청: [길이, SID, subfunction, parameter...]
    긍정 응답: [길이, SID+0x40, subfunction, data...]
    부정 응답: [길이, 0x7F, SID, NRC]
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
    print("python-can 설치 필요: pip install python-can")
    sys.exit(1)


#=======================================
# 공통 유틸리티
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
        print(f"  {'테스트 항목':<35} {'상태':<10} {'상세':<30}")
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
        print(f"  총 {total}개 | {Color.GREEN}성공 {p}{Color.NC} | {Color.RED}실패 {f}{Color.NC}")
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
# UDS 테스터
#=======================================
class UDSTester:
    """UDS 서비스 테스트 클래스"""

    REQUEST_ID = 0x7E0
    RESPONSE_ID = 0x7E8

    # UDS SID
    SID_DIAG_SESSION   = 0x10
    SID_ECU_RESET      = 0x11
    SID_READ_DID       = 0x22
    SID_SECURITY       = 0x27
    SID_ROUTINE        = 0x31

    # 세션 타입
    SESSION_DEFAULT     = 0x01
    SESSION_PROGRAMMING = 0x02
    SESSION_EXTENDED    = 0x03

    # DID
    DID_VIN       = 0xF190
    DID_HW_VER    = 0xF193
    DID_SW_VER    = 0xF195

    # 리셋 타입
    RESET_HARD = 0x01
    RESET_SOFT = 0x03

    # 시큐리티
    SEC_REQUEST_SEED = 0x01
    SEC_SEND_KEY     = 0x02

    # Seed-Key 알고리즘 (ECU와 동일)
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
        print(Color.info_msg(f"CAN 연결: {self.interface}"))

    def disconnect(self):
        if self.bus:
            self.bus.shutdown()
            self.bus = None

    def compute_key(self, seed: int) -> int:
        """ECU와 동일한 seed-key 알고리즘"""
        xored = seed ^ self.SEED_XOR_MASK
        rotated = ((xored << 3) | (xored >> 13)) & 0xFFFF
        return rotated

    def send_uds(self, data: list, timeout: float = 2.0) -> Optional[bytes]:
        """UDS 요청 전송 후 응답 수신"""
        if not self.bus:
            raise RuntimeError("CAN 미연결")

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
        """긍정 응답 파싱: SID+0x40 확인"""
        if not data or len(data) < 2:
            return None
        if data[1] != expected_sid + 0x40:
            return None
        return {'sid_resp': data[1], 'payload': data[2:]}

    def parse_negative(self, data: bytes) -> Optional[dict]:
        """부정 응답 파싱: 0x7F + SID + NRC"""
        if not data or len(data) < 4:
            return None
        if data[1] != 0x7F:
            return None
        return {'sid': data[2], 'nrc': data[3], 'nrc_name': nrc_name(data[3])}


#=======================================
# 테스트 케이스
#=======================================
def test_diag_session(tester: UDSTester, report: TestReport):
    """SID 0x10: DiagnosticSessionControl 테스트"""
    print(Color.header("SID 0x10: DiagnosticSessionControl"))

    # 1) Default 세션 전환 (항상 성공해야 함)
    resp = tester.send_uds([0x02, 0x10, 0x01])
    result = TestResult("Default 세션 전환 (0x01)")
    if resp:
        pos = tester.parse_positive(resp, 0x10)
        if pos and pos['payload'][0] == 0x01:
            result.status = TestStatus.PASS
            result.detail = "정상 응답"
        else:
            neg = tester.parse_negative(resp)
            if neg:
                result.status = TestStatus.FAIL
                result.detail = f"NRC {neg['nrc_name']}"
            else:
                result.status = TestStatus.FAIL
                result.detail = "파싱 실패"
        result.raw_rx = resp.hex().upper()
    else:
        result.status = TestStatus.FAIL
        result.detail = "타임아웃"
    result.raw_tx = "02 10 01"
    print(f"  {result.status.value}: {result.detail}")
    report.add(result)

    # 2) Extended 세션 전환
    resp = tester.send_uds([0x02, 0x10, 0x03])
    result = TestResult("Extended 세션 전환 (0x03)")
    if resp:
        pos = tester.parse_positive(resp, 0x10)
        if pos and pos['payload'][0] == 0x03:
            result.status = TestStatus.PASS
            result.detail = "정상 응답"
        else:
            neg = tester.parse_negative(resp)
            if neg:
                result.status = TestStatus.FAIL
                result.detail = f"NRC {neg['nrc_name']}"
            else:
                result.status = TestStatus.FAIL
                result.detail = "파싱 실패"
        result.raw_rx = resp.hex().upper()
    else:
        result.status = TestStatus.FAIL
        result.detail = "타임아웃"
    result.raw_tx = "02 10 03"
    print(f"  {result.status.value}: {result.detail}")
    report.add(result)

    # 3) 잘못된 세션 타입 (0xFF) → NRC 예상
    resp = tester.send_uds([0x02, 0x10, 0xFF])
    result = TestResult("잘못된 세션 타입 (0xFF)")
    if resp:
        neg = tester.parse_negative(resp)
        if neg:
            result.status = TestStatus.PASS
            result.detail = f"NRC {neg['nrc_name']} (예상됨)"
        else:
            result.status = TestStatus.FAIL
            result.detail = "NRC가 와야 하는데 긍정 응답"
        result.raw_rx = resp.hex().upper()
    else:
        result.status = TestStatus.FAIL
        result.detail = "타임아웃"
    result.raw_tx = "02 10 FF"
    print(f"  {result.status.value}: {result.detail}")
    report.add(result)

    # Default로 복귀
    tester.send_uds([0x02, 0x10, 0x01])


def test_read_did(tester: UDSTester, report: TestReport):
    """SID 0x22: ReadDataByIdentifier 테스트"""
    print(Color.header("SID 0x22: ReadDataByIdentifier"))

    # 1) HW 버전 읽기 (0xF193)
    resp = tester.send_uds([0x03, 0x22, 0xF1, 0x93])
    result = TestResult("Read DID 0xF193 (HW 버전)")
    if resp:
        pos = tester.parse_positive(resp, 0x22)
        if pos and len(pos['payload']) >= 3:
            did = (pos['payload'][0] << 8) | pos['payload'][1]
            value = pos['payload'][2:]
            result.status = TestStatus.PASS
            result.detail = f"DID=0x{did:04X} 값={value}"
        else:
            neg = tester.parse_negative(resp)
            if neg:
                result.status = TestStatus.FAIL
                result.detail = f"NRC {neg['nrc_name']}"
            else:
                result.status = TestStatus.FAIL
                result.detail = "파싱 실패"
        result.raw_rx = resp.hex().upper()
    else:
        result.status = TestStatus.FAIL
        result.detail = "타임아웃"
    result.raw_tx = "03 22 F1 93"
    print(f"  {result.status.value}: {result.detail}")
    report.add(result)

    # 2) SW 버전 읽기 (0xF195)
    resp = tester.send_uds([0x03, 0x22, 0xF1, 0x95])
    result = TestResult("Read DID 0xF195 (SW 버전)")
    if resp:
        pos = tester.parse_positive(resp, 0x22)
        if pos and len(pos['payload']) >= 3:
            did = (pos['payload'][0] << 8) | pos['payload'][1]
            value = pos['payload'][2:]
            result.status = TestStatus.PASS
            result.detail = f"DID=0x{did:04X} 값={value}"
        else:
            neg = tester.parse_negative(resp)
            if neg:
                result.status = TestStatus.FAIL
                result.detail = f"NRC {neg['nrc_name']}"
            else:
                result.status = TestStatus.FAIL
                result.detail = "파싱 실패"
        result.raw_rx = resp.hex().upper()
    else:
        result.status = TestStatus.FAIL
        result.detail = "타임아웃"
    result.raw_tx = "03 22 F1 95"
    print(f"  {result.status.value}: {result.detail}")
    report.add(result)

    # 3) 존재하지 않는 DID → NRC 예상
    resp = tester.send_uds([0x03, 0x22, 0xFF, 0xFF])
    result = TestResult("존재하지 않는 DID (0xFFFF)")
    if resp:
        neg = tester.parse_negative(resp)
        if neg:
            result.status = TestStatus.PASS
            result.detail = f"NRC {neg['nrc_name']} (예상됨)"
        else:
            result.status = TestStatus.FAIL
            result.detail = "NRC가 와야 함"
        result.raw_rx = resp.hex().upper()
    else:
        result.status = TestStatus.FAIL
        result.detail = "타임아웃"
    result.raw_tx = "03 22 FF FF"
    print(f"  {result.status.value}: {result.detail}")
    report.add(result)


def test_security_access(tester: UDSTester, report: TestReport):
    """SID 0x27: SecurityAccess Seed-Key 테스트"""
    print(Color.header("SID 0x27: SecurityAccess"))

    # Extended 세션으로 전환 (필수)
    tester.send_uds([0x02, 0x10, 0x03])
    time.sleep(0.05)

    # 1) Seed 요청 (subFunction = 0x01)
    resp = tester.send_uds([0x02, 0x27, 0x01])
    result = TestResult("Seed 요청 (0x27 0x01)")
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
                result.detail = "파싱 실패"
        result.raw_rx = resp.hex().upper()
    else:
        result.status = TestStatus.FAIL
        result.detail = "타임아웃"
    result.raw_tx = "02 27 01"
    print(f"  {result.status.value}: {result.detail}")
    report.add(result)

    if seed is None:
        # Seed 못 받으면 Key 테스트 불가
        report.add(TestResult("Key 전송 (0x27 0x02)", TestStatus.SKIP, "Seed 획득 실패"))
        tester.send_uds([0x02, 0x10, 0x01])
        return

    # 2) 올바른 Key 전송
    key = tester.compute_key(seed)
    key_hi = (key >> 8) & 0xFF
    key_lo = key & 0xFF
    resp = tester.send_uds([0x04, 0x27, 0x02, key_hi, key_lo])
    result = TestResult("Key 전송 (0x27 0x02) - 올바른 키")
    if resp:
        pos = tester.parse_positive(resp, 0x27)
        if pos:
            result.status = TestStatus.PASS
            result.detail = "시큐리티 언락 성공"
        else:
            neg = tester.parse_negative(resp)
            if neg:
                result.status = TestStatus.FAIL
                result.detail = f"NRC {neg['nrc_name']}"
            else:
                result.status = TestStatus.FAIL
                result.detail = "파싱 실패"
        result.raw_rx = resp.hex().upper()
    else:
        result.status = TestStatus.FAIL
        result.detail = "타임아웃"
    result.raw_tx = f"04 27 02 {key_hi:02X} {key_lo:02X}"
    print(f"  {result.status.value}: {result.detail}")
    report.add(result)

    # 3) 잘못된 Key 전송 (다시 seed 요청 후)
    resp = tester.send_uds([0x02, 0x27, 0x01])
    if resp:
        pos = tester.parse_positive(resp, 0x27)
        if pos and len(pos['payload']) >= 3:
            seed2 = (pos['payload'][1] << 8) | pos['payload'][2]
            # 일부러 틀린 key
            wrong_key = seed2 ^ 0xFFFF
            wk_hi = (wrong_key >> 8) & 0xFF
            wk_lo = wrong_key & 0xFF
            resp2 = tester.send_uds([0x04, 0x27, 0x02, wk_hi, wk_lo])
            result = TestResult("Key 전송 - 잘못된 키")
            if resp2:
                neg = tester.parse_negative(resp2)
                if neg and neg['nrc'] in (0x35, 0x36):
                    result.status = TestStatus.PASS
                    result.detail = f"NRC {neg['nrc_name']} (예상됨)"
                else:
                    result.status = TestStatus.FAIL
                    result.detail = "NRC 0x35/0x36이 와야 함"
                result.raw_rx = resp2.hex().upper()
            else:
                result.status = TestStatus.FAIL
                result.detail = "타임아웃"
            result.raw_tx = f"04 27 02 {wk_hi:02X} {wk_lo:02X}"
            print(f"  {result.status.value}: {result.detail}")
            report.add(result)

    # Default로 복귀
    tester.send_uds([0x02, 0x10, 0x01])


def test_routine_control(tester: UDSTester, report: TestReport):
    """SID 0x31: RoutineControl 테스트"""
    print(Color.header("SID 0x31: RoutineControl"))

    # 1) Default 세션에서 호출 → NRC 0x33 (securityAccessDenied) 예상
    tester.send_uds([0x02, 0x10, 0x01])
    time.sleep(0.05)
    resp = tester.send_uds([0x04, 0x31, 0x01, 0x02, 0x01])
    result = TestResult("Default 세션에서 RoutineControl")
    if resp:
        neg = tester.parse_negative(resp)
        if neg and neg['nrc'] == 0x33:
            result.status = TestStatus.PASS
            result.detail = f"NRC securityAccessDenied (예상됨)"
        else:
            result.status = TestStatus.FAIL
            result.detail = f"NRC 0x33이 와야 함, got {neg['nrc_name'] if neg else 'positive'}"
        result.raw_rx = resp.hex().upper()
    else:
        result.status = TestStatus.FAIL
        result.detail = "타임아웃"
    result.raw_tx = "04 31 01 02 01"
    print(f"  {result.status.value}: {result.detail}")
    report.add(result)

    # 2) Extended + Security Unlock 후 호출
    tester.send_uds([0x02, 0x10, 0x03])
    time.sleep(0.05)

    # Seed 요청
    resp = tester.send_uds([0x02, 0x27, 0x01])
    if resp:
        pos = tester.parse_positive(resp, 0x27)
        if pos and len(pos['payload']) >= 3:
            seed = (pos['payload'][1] << 8) | pos['payload'][2]
            key = tester.compute_key(seed)
            # Key 전송
            resp2 = tester.send_uds([0x04, 0x27, 0x02, (key >> 8) & 0xFF, key & 0xFF])
            if resp2:
                pos2 = tester.parse_positive(resp2, 0x27)
                if pos2:
                    # 이제 RoutineControl 호출 가능
                    resp3 = tester.send_uds([0x04, 0x31, 0x01, 0x02, 0x01])
                    result = TestResult("Extended+Unlock 후 RoutineControl")
                    if resp3:
                        pos3 = tester.parse_positive(resp3, 0x31)
                        if pos3:
                            result.status = TestStatus.PASS
                            result.detail = "정상 응답"
                        else:
                            neg3 = tester.parse_negative(resp3)
                            result.status = TestStatus.FAIL
                            result.detail = f"NRC {neg3['nrc_name']}" if neg3 else "파싱 실패"
                        result.raw_rx = resp3.hex().upper()
                    else:
                        result.status = TestStatus.FAIL
                        result.detail = "타임아웃"
                    result.raw_tx = "04 31 01 02 01"
                    print(f"  {result.status.value}: {result.detail}")
                    report.add(result)

    # Default로 복귀
    tester.send_uds([0x02, 0x10, 0x01])


def test_obd2_via_uds(tester: UDSTester, report: TestReport):
    """SID 0x01: OBD-II PID가 UDS 경로로도 동작하는지 확인"""
    print(Color.header("SID 0x01: OBD-II via UDS"))

    pids = [
        (0x00, "지원 PID 목록"),
        (0x05, "냉각수 온도"),
        (0x0C, "엔진 RPM"),
        (0x0D, "차속"),
    ]

    for pid, name in pids:
        resp = tester.send_uds([0x02, 0x01, pid])
        result = TestResult(f"PID 0x{pid:02X} ({name})")
        if resp:
            pos = tester.parse_positive(resp, 0x01)
            if pos and pos['payload'][0] == pid:
                result.status = TestStatus.PASS
                result.detail = "정상 응답"
            else:
                neg = tester.parse_negative(resp)
                if neg:
                    result.status = TestStatus.FAIL
                    result.detail = f"NRC {neg['nrc_name']}"
                else:
                    result.status = TestStatus.FAIL
                    result.detail = "파싱 실패"
            result.raw_rx = resp.hex().upper()
        else:
            result.status = TestStatus.FAIL
            result.detail = "타임아웃"
        result.raw_tx = f"02 01 {pid:02X}"
        print(f"  {result.status.value}: PID 0x{pid:02X} ({name})")
        report.add(result)


#=======================================
# vcan 설정 헬퍼
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
        print(Color.fail_msg(f"vcan 설정 실패: {e}"))
        return False


def cleanup_vcan(interface: str):
    try:
        subprocess.run(['ip', 'link', 'set', interface, 'down'], capture_output=True, timeout=5)
        subprocess.run(['ip', 'link', 'del', interface], capture_output=True, timeout=5)
    except Exception:
        pass


#=======================================
# 메인
#=======================================
def main():
    parser = argparse.ArgumentParser(description='UDS ECU 시뮬레이터 테스트')
    parser.add_argument('interface', nargs='?', default='vcan0', help='CAN 인터페이스')
    parser.add_argument('--bitrate', type=int, default=500000)
    parser.add_argument('--timeout', type=float, default=2.0)
    parser.add_argument('--setup-vcan', action='store_true', help='vcan 자동 설정')
    args = parser.parse_args()

    print("=" * 50)
    print("  UDS ECU 시뮬레이터 테스트")
    print(f"  인터페이스: {args.interface}")
    print(f"  날짜: {time.strftime('%Y-%m-%d %H:%M:%S')}")
    print("=" * 50)

    if args.setup_vcan:
        if not setup_vcan(args.interface):
            sys.exit(1)
        print(Color.pass_msg(f"vcan 설정 완료: {args.interface}"))

    report = TestReport()

    try:
        tester = UDSTester(args.interface, args.bitrate)
        tester.connect()
        time.sleep(0.3)

        # 테스트 실행
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
        print(f"\n사용자 중단")
        sys.exit(130)
    finally:
        if args.setup_vcan:
            cleanup_vcan(args.interface)

    all_passed = report.print_summary()
    sys.exit(0 if all_passed else 1)


if __name__ == '__main__':
    main()
