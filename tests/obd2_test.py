#!/usr/bin/env python3
"""
OBD-II ECU 시뮬레이터 테스트 (python-can)

설명:
    Linux SocketCAN 인터페이스를 사용하여 OBD-II ECU 시뮬레이터를 테스트합니다.
    python-can 라이브러리를 사용하며, virtual CAN (vcan)으로도 테스트 가능합니다.

사용법:
    sudo python3 obd2_test.py [CAN 인터페이스]
    예: sudo python3 obd2_test.py can0       # 실제 CAN 어댑터
        sudo python3 obd2_test.py vcan0      # virtual CAN (테스트용)

전제 조건:
    1. python-can 설치: pip install python-can
    2. vcan 사용 시:   sudo modprobe vcan && sudo ip link add dev vcan0 type vcan && sudo ip link set vcan0 up
    3. root 권한 필요

OBD-II CAN ID 참고:
    0x7E0 - OBD-II 요청 (ECU1, 테스터 -> ECU)
    0x7E8 - OBD-II 응답 (ECU1, ECU -> 테스터)
    0x7DF - 브로드캐스트 요청 (모든 ECU)

ISO-TP 단일 프레임 (SF) 구조:
    요청: [PCI 타입=0x01][길이][서비스ID=0x01][PID][패딩...]
    응답: [PCI 타입=0x01][길이][서비스ID=0x41][PID][데이터][패딩...]
"""

import sys
import time
import argparse
import subprocess
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

# python-can 임포트
try:
    import can
except ImportError:
    print("오류: python-can 라이브러리가 설치되어 있지 않습니다.")
    print("설치: pip install python-can")
    sys.exit(1)

#=======================================
# 색상 출력 클래스
#=======================================
class Color:
    """터미널 컬러 출력을 위한 유틸리티 클래스"""
    RED = '\033[0;31m'
    GREEN = '\033[0;32m'
    YELLOW = '\033[1;33m'
    CYAN = '\033[0;36m'
    BOLD = '\033[1m'
    NC = '\033[0m'  # 색상 리셋

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
# 테스트 결과 데이터 클래스
#=======================================
class TestStatus(Enum):
    PASS = "PASS"
    FAIL = "FAIL"
    SKIP = "SKIP"


@dataclass
class TestResult:
    """개별 테스트 결과"""
    name: str
    status: TestStatus
    detail: str = ""
    raw_data: str = ""
    decoded_value: str = ""


@dataclass
class TestReport:
    """전체 테스트 리포트"""
    results: list = field(default_factory=list)

    def add(self, result: TestResult):
        self.results.append(result)

    def print_table(self):
        """테스트 결과를 테이블 형태로 출력"""
        print("\n" + "=" * 78)
        print(f"  {'테스트 항목':<20} {'상태':<8} {'디코딩 값':<16} {'상세':<30}")
        print("=" * 78)

        for r in self.results:
            # 상태에 따른 색상 선택
            if r.status == TestStatus.PASS:
                status_str = Color.pass_msg(r.status.value)
            elif r.status == TestStatus.FAIL:
                status_str = Color.fail_msg(r.status.value)
            else:
                status_str = Color.skip_msg(r.status.value)

            # 상세 정보가 길면 자르기
            detail = r.detail[:28] + ".." if len(r.detail) > 30 else r.detail

            print(f"  {r.name:<20} {status_str:<28} {r.decoded_value:<16} {detail}")
            if r.raw_data:
                print(f"  {'':>20}   수신 프레임: {r.raw_data}")

        print("=" * 78)

        # 요약 통계
        pass_count = sum(1 for r in self.results if r.status == TestStatus.PASS)
        fail_count = sum(1 for r in self.results if r.status == TestStatus.FAIL)
        skip_count = sum(1 for r in self.results if r.status == TestStatus.SKIP)
        total = len(self.results)

        print(f"  총 테스트: {total}  |  ", end="")
        print(f"{Color.GREEN}성공: {pass_count}{Color.NC}  |  ", end="")
        print(f"{Color.RED}실패: {fail_count}{Color.NC}  |  ", end="")
        print(f"{Color.YELLOW}건너뜀: {skip_count}{Color.NC}")
        print("=" * 78)

        if fail_count > 0:
            print(f"  결과: {Color.RED}일부 테스트 실패{Color.NC}")
        else:
            print(f"  결과: {Color.GREEN}모든 테스트 통과{Color.NC}")

        return fail_count == 0


#=======================================
# OBD-II PID 정의
#=======================================
@dataclass
class OBD2PID:
    """OBD-II PID 정의"""
    pid: int
    name: str
    description: str
    decode_func: Optional[callable] = None


def decode_coolant_temp(data: bytes) -> str:
    """
    PID 0x05: 냉각수 온도 디코딩
    공식: 온도(°C) = A - 40
    범위: -40°C ~ 215°C
    """
    raw = data[0]
    temp_c = raw - 40
    return f"{temp_c} °C (원시값: 0x{raw:02X})"


def decode_rpm(data: bytes) -> str:
    """
    PID 0x0C: 엔진 RPM 디코딩
    공식: RPM = ((A * 256) + B) / 4
    범위: 0 ~ 16383.75 RPM
    """
    raw = (data[0] << 8) | data[1]
    rpm = raw / 4.0
    return f"{rpm:.0f} RPM (원시값: 0x{data[0]:02X}{data[1]:02X})"


def decode_vehicle_speed(data: bytes) -> str:
    """
    PID 0x0D: 차량 속도 디코딩
    공식: 속도(km/h) = A
    범위: 0 ~ 255 km/h
    """
    speed = data[0]
    return f"{speed} km/h (원시값: 0x{data[0]:02X})"


def decode_supported_pids(data: bytes) -> str:
    """
    PID 0x00: 지원되는 PID 비트맵 디코딩
    4바이트 비트맵으로 PID 0x01 ~ 0x20의 지원 여부 표현
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
    return f"지원 PID: {', '.join(supported[:5])}{'...' if len(supported) > 5 else ''}"


# 테스트할 PID 목록
TEST_PIDS = [
    OBD2PID(0x00, "지원 PID 목록", "PID 비트맵 조회", decode_supported_pids),
    OBD2PID(0x05, "냉각수 온도", "PID 0x05 디코딩", decode_coolant_temp),
    OBD2PID(0x0C, "엔진 RPM", "PID 0x0C 디코딩", decode_rpm),
    OBD2PID(0x0D, "차량 속도", "PID 0x0D 디코딩", decode_vehicle_speed),
]


#=======================================
# OBD-II 통신 클래스
#=======================================
class OBD2Tester:
    """
    OBD-II ECU 시뮬레이터 테스트 클래스

    python-can을 사용하여 CAN 버스에 OBD-II 요청을 전송하고 응답을 수신합니다.
    ISO-TP 단일 프레임 (Single Frame) 프로토콜을 사용합니다.
    """

    # OBD-II CAN ID
    REQUEST_ID = 0x7E0    # 테스터 -> ECU1
    RESPONSE_ID = 0x7E8   # ECU1 -> 테스터
    BROADCAST_ID = 0x7DF  # 브로드캐스트 요청

    # OBD-II 서비스 ID
    SERVICE_SHOW_CURRENT_DATA = 0x01
    SERVICE_RESPONSE_OFFSET = 0x40  # 응답 서비스 = 요청 서비스 + 0x40

    # ISO-TP PCI 타입
    ISO_TP_SINGLE_FRAME = 0x01

    def __init__(self, interface: str, bitrate: int = 500000):
        """
        OBD-II 테스터 초기화

        Args:
            interface: CAN 인터페이스 이름 (예: "can0", "vcan0")
            bitrate: CAN 통신 속도 (기본: 500kbps)
        """
        self.interface = interface
        self.bitrate = bitrate
        self.bus: Optional[can.Bus] = None

    def connect(self):
        """
        CAN 버스에 연결합니다.

        virtual CAN (vcan)인 경우 별도 설정이 필요하지 않습니다.
        실제 CAN 인터페이스인 경우 bitrate 설정이 필요할 수 있습니다.
        """
        try:
            self.bus = can.Bus(
                interface='socketcan',
                channel=self.interface,
                bitrate=self.bitrate,
            )
            print(Color.info_msg(f"CAN 버스 연결 성공: {self.interface}"))
            print(Color.info_msg(f"  버스 상태: {self.bus.state}"))
        except can.CanError as e:
            raise RuntimeError(f"CAN 버스 연결 실패 ({self.interface}): {e}") from e

    def disconnect(self):
        """CAN 버스 연결을 종료합니다."""
        if self.bus:
            self.bus.shutdown()
            self.bus = None
            print(Color.info_msg(f"CAN 버스 연결 종료: {self.interface}"))

    def send_request(self, pid: int, timeout: float = 2.0) -> Optional[can.Message]:
        """
        OBD-II PID 요청을 전송하고 응답을 수신합니다.

        ISO-TP 단일 프레임 (SF) 형식으로 요청을 생성합니다:
        - 바이트0: PCI 타입 = 0x01 (SF)
        - 바이트1: 데이터 길이 = 0x02
        - 바이트2: 서비스 ID = 0x01 (Show Current Data)
        - 바이트3: PID
        - 바이트4~7: 패딩 (0x00)

        Args:
            pid: 요청할 OBD-II PID (예: 0x0D = 차속)
            timeout: 응답 대기 시간 (초, 기본: 2초)

        Returns:
            수신된 CAN 메시지 또는 None (타임아웃 시)
        """
        if not self.bus:
            raise RuntimeError("CAN 버스에 연결되어 있지 않습니다")

        # ISO-TP 단일 프레임 요청 생성
        # PCI=0x01(SF), 길이=0x02, 서비스=0x01, PID, 패딩
        request_data = bytes([
            self.ISO_TP_SINGLE_FRAME,  # PCI: 단일 프레임
            0x02,                       # 데이터 길이
            self.SERVICE_SHOW_CURRENT_DATA,  # 서비스 ID: Show Current Data
            pid,                        # 요청 PID
            0x00, 0x00, 0x00, 0x00      # 패딩
        ])

        # CAN 메시지 생성 및 전송
        request_msg = can.Message(
            arbitration_id=self.REQUEST_ID,
            data=request_data,
            is_extended_id=False,
        )

        try:
            self.bus.send(request_msg)
        except can.CanError as e:
            print(Color.fail_msg(f"요청 전송 실패 (PID 0x{pid:02X}): {e}"))
            return None

        # 응답 수신 대기
        try:
            response_msg = self.bus.recv(timeout=timeout)
            if response_msg is None:
                return None

            # 응답 CAN ID 확인
            if response_msg.arbitration_id != self.RESPONSE_ID:
                print(Color.info_msg(
                    f"예상 외 CAN ID 수신: 0x{response_msg.arbitration_id:03X} "
                    f"(예상: 0x{self.RESPONSE_ID:03X})"
                ))
                # 브로드캐스트 응답 ID도 허용

            return response_msg

        except can.CanError as e:
            print(Color.fail_msg(f"응답 수신 오류 (PID 0x{pid:02X}): {e}"))
            return None

    @staticmethod
    def parse_response(msg: can.Message) -> Optional[dict]:
        """
        OBD-II 응답 메시지를 파싱합니다.

        ISO-TP 단일 프레임 응답 형식:
        - 바이트0: PCI 타입 (0x01 = SF) 또는 길이
        - 바이트1: 데이터 길이 (PCI=0x01인 경우)
        - 바이트2: 서비스 ID (요청 서비스 + 0x40)
        - 바이트3: 응답 PID
        - 바이트4+: 응답 데이터

        Args:
            msg: 수신된 CAN 메시지

        Returns:
            파싱된 응답 딕셔너리 또는 None (파싱 실패 시)
            {
                'service_id': int,
                'pid': int,
                'data': bytes,
                'raw': str (16진수 문자열)
            }
        """
        if not msg or not msg.data:
            return None

        data = msg.data

        # 최소 길이 확인 (PCI + 길이 + 서비스 + PID = 4바이트)
        if len(data) < 4:
            return None

        # 서비스 ID 확인: 0x41 (Show Current Data 응답)
        service_id = data[2]
        if service_id != 0x41:
            return None

        pid = data[3]
        response_data = data[4:]

        return {
            'service_id': service_id,
            'pid': pid,
            'data': bytes(response_data),
            'raw': ''.join(f'{b:02X}' for b in data),
        }

    def test_pid(self, obd2_pid: OBD2PID, timeout: float = 2.0) -> TestResult:
        """
        단일 PID를 테스트합니다.

        Args:
            obd2_pid: 테스트할 OBD2PID 객체
            timeout: 응답 대기 시간 (초)

        Returns:
            TestResult 객체
        """
        # OBD-II 요청 전송
        response_msg = self.send_request(obd2_pid.pid, timeout)

        if response_msg is None:
            return TestResult(
                name=f"PID 0x{obd2_pid.pid:02X} ({obd2_pid.name})",
                status=TestStatus.FAIL,
                detail=f"응답 없음 (타임아웃 {timeout}초)",
            )

        # 응답 파싱
        parsed = self.parse_response(response_msg)

        if parsed is None:
            return TestResult(
                name=f"PID 0x{obd2_pid.pid:02X} ({obd2_pid.name})",
                status=TestStatus.FAIL,
                detail=f"응답 파싱 실패",
                raw_data=response_msg.data.hex().upper(),
            )

        # PID 일치 확인
        if parsed['pid'] != obd2_pid.pid:
            return TestResult(
                name=f"PID 0x{obd2_pid.pid:02X} ({obd2_pid.name})",
                status=TestStatus.FAIL,
                detail=f"PID 불일치: 요청 0x{obd2_pid.pid:02X}, 응답 0x{parsed['pid']:02X}",
                raw_data=parsed['raw'],
            )

        # 데이터 디코딩
        decoded = ""
        if obd2_pid.decode_func and parsed['data']:
            try:
                decoded = obd2_pid.decode_func(parsed['data'])
            except (IndexError, ValueError) as e:
                return TestResult(
                    name=f"PID 0x{obd2_pid.pid:02X} ({obd2_pid.name})",
                    status=TestStatus.FAIL,
                    detail=f"디코딩 오류: {e}",
                    raw_data=parsed['raw'],
                )

        return TestResult(
            name=f"PID 0x{obd2_pid.pid:02X} ({obd2_pid.name})",
            status=TestStatus.PASS,
            detail=obd2_pid.description,
            raw_data=parsed['raw'],
            decoded_value=decoded,
        )

    def test_ramp_up_down(self,
                           pid: int = 0x0C,
                           samples: int = 20,
                           interval: float = 0.5,
                           timeout: float = 2.0) -> TestResult:
        """
        램프 업/다운 시뮬레이션 검증

        ECU 시뮬레이터가 램프 업/다운 패턴을 따르는지 확인합니다.
        지정된 PID를 여러 번 샘플링하여 값이 점진적으로 변하는지 검증합니다.

        Args:
            pid: 모니터링할 PID (기본: 0x0C = RPM)
            samples: 샘플링 횟수 (기본: 20)
            interval: 샘플링 간격 (초, 기본: 0.5)
            timeout: 응답 대기 시간 (초)

        Returns:
            TestResult 객체
        """
        values = []
        timestamps = []

        print(Color.info_msg(
            f"램프 테스트 시작: PID 0x{pid:02X}, "
            f"{samples}회 샘플링, 간격 {interval}s"
        ))

        for i in range(samples):
            start = time.time()

            response_msg = self.send_request(pid, timeout)
            parsed = self.parse_response(response_msg) if response_msg else None

            elapsed = time.time() - start

            if parsed and parsed['data']:
                # RPM 디코딩: (A*256 + B) / 4
                if pid == 0x0C and len(parsed['data']) >= 2:
                    raw = (parsed['data'][0] << 8) | parsed['data'][1]
                    rpm = raw / 4.0
                    values.append(rpm)
                else:
                    values.append(parsed['data'][0])
                timestamps.append(elapsed)
                print(f"    [{i+1:3d}/{samples}] RPM = {values[-1]:8.0f}  ({elapsed:.3f}s)")
            else:
                print(f"    [{i+1:3d}/{samples}] 응답 없음  ({elapsed:.3f}s)")

            # 샘플링 간격 대기 (응답 시간 고려)
            remaining = interval - elapsed
            if remaining > 0:
                time.sleep(remaining)

        # 결과 분석
        if len(values) < 3:
            return TestResult(
                name=f"램프 테스트 (PID 0x{pid:02X})",
                status=TestStatus.FAIL,
                detail=f"유효 샘플 부족: {len(values)}/{samples}",
            )

        min_val = min(values)
        max_val = max(values)
        avg_val = sum(values) / len(values)

        # 방향 전환 감지
        direction_changes = 0
        prev_direction = 0  # 1=상승, -1=하강, 0=동일
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

        # 값 변화 확인
        if min_val == max_val:
            status = TestStatus.FAIL
            detail = f"값 변화 없음 (모든 샘플이 {min_val:.0f})"
        else:
            status = TestStatus.PASS
            detail = (
                f"범위: {min_val:.0f} ~ {max_val:.0f}, "
                f"평균: {avg_val:.0f}, "
                f"방향전환: {direction_changes}회"
            )

        # 샘플 값 출력
        values_str = ", ".join(f"{v:.0f}" for v in values)
        print(Color.info_msg(f"샘플링된 값 ({len(values)}개): [{values_str}]"))
        print(Color.info_msg(
            f"통계: 최소={min_val:.0f}, 최대={max_val:.0f}, "
            f"평균={avg_val:.0f}, 방향전환={direction_changes}회"
        ))

        return TestResult(
            name=f"램프 테스트 (PID 0x{pid:02X})",
            status=status,
            detail=detail,
            decoded_value=f"샘플 {len(values)}개",
        )

    def test_stress(self,
                    pid: int = 0x0D,
                    count: int = 10,
                    timeout: float = 2.0) -> list:
        """
        반복 스트레스 테스트

        지정된 PID를 여러 번 요청하여 응답 안정성을 검증합니다.

        Args:
            pid: 테스트할 PID (기본: 0x0D = 차속)
            count: 반복 횟수 (기본: 10)
            timeout: 응답 대기 시간 (초)

        Returns:
            TestResult 리스트
        """
        results = []
        print(Color.info_msg(f"스트레스 테스트 시작: PID 0x{pid:02X}, {count}회 반복"))

        for i in range(count):
            start = time.time()
            response_msg = self.send_request(pid, timeout)
            elapsed = time.time() - start

            if response_msg is None:
                results.append(TestResult(
                    name=f"스트레스 [{i+1}/{count}]",
                    status=TestStatus.FAIL,
                    detail=f"응답 없음 ({elapsed:.3f}s)",
                ))
                print(f"    [{i+1:3d}/{count}] {Color.RED}FAIL{Color.NC} 응답 없음 ({elapsed:.3f}s)")
                continue

            parsed = self.parse_response(response_msg)
            if parsed and parsed['pid'] == pid:
                results.append(TestResult(
                    name=f"스트레스 [{i+1}/{count}]",
                    status=TestStatus.PASS,
                    detail=f"정상 응답 ({elapsed:.3f}s)",
                    raw_data=parsed['raw'],
                ))
                print(f"    [{i+1:3d}/{count}] {Color.GREEN}PASS{Color.NC} ({elapsed:.3f}s) {parsed['raw']}")
            else:
                results.append(TestResult(
                    name=f"스트레스 [{i+1}/{count}]",
                    status=TestStatus.FAIL,
                    detail=f"응답 오류 ({elapsed:.3f}s)",
                    raw_data=response_msg.data.hex().upper() if response_msg else "",
                ))
                print(f"    [{i+1:3d}/{count}] {Color.RED}FAIL{Color.NC} ({elapsed:.3f}s)")

        # 스트레스 테스트 요약
        pass_count = sum(1 for r in results if r.status == TestStatus.PASS)
        fail_count = sum(1 for r in results if r.status == TestStatus.FAIL)
        print(Color.info_msg(
            f"스트레스 테스트 결과: {Color.GREEN}{pass_count} 성공{Color.NC}, "
            f"{Color.RED}{fail_count} 실패{Color.NC} (합계: {count})"
        ))

        return results


#=======================================
# CAN 인터페이스 설정 헬퍼
#=======================================
def setup_vcan(interface: str) -> bool:
    """
    virtual CAN 인터페이스를 설정합니다.

    Args:
        interface: vcan 인터페이스 이름 (예: "vcan0")

    Returns:
        설정 성공 여부
    """
    try:
        # vcan 커널 모듈 로드
        subprocess.run(['modprobe', 'vcan'], check=True, capture_output=True)

        # 기존 인터페이스가 있으면 삭제
        subprocess.run(
            ['ip', 'link', 'del', interface],
            capture_output=True, timeout=5
        )

        # vcan 인터페이스 생성
        subprocess.run(
            ['ip', 'link', 'add', 'dev', interface, 'type', 'vcan'],
            check=True, capture_output=True, timeout=5
        )

        # 인터페이스 활성화
        subprocess.run(
            ['ip', 'link', 'set', interface, 'up'],
            check=True, capture_output=True, timeout=5
        )

        return True
    except (subprocess.CalledProcessError, FileNotFoundError, subprocess.TimeoutExpired) as e:
        print(Color.fail_msg(f"vcan 설정 실패: {e}"))
        return False


def cleanup_vcan(interface: str):
    """
    virtual CAN 인터페이스를 정리합니다.

    Args:
        interface: vcan 인터페이스 이름
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
# 메인 함수
#=======================================
def main():
    """메인 테스트 실행"""
    parser = argparse.ArgumentParser(
        description='OBD-II ECU 시뮬레이터 테스트 (python-can)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
사용 예:
  sudo python3 obd2_test.py can0           # 실제 CAN 어댑터
  sudo python3 obd2_test.py vcan0          # virtual CAN
  sudo python3 obd2_test.py vcan0 --stress 50   # 스트레스 테스트 50회
  sudo python3 obd2_test.py vcan0 --ramp-samples 30 --ramp-interval 0.3
        """,
    )
    parser.add_argument(
        'interface',
        nargs='?',
        default='can0',
        help='CAN 인터페이스 이름 (기본: can0)',
    )
    parser.add_argument(
        '--bitrate',
        type=int,
        default=500000,
        help='CAN 통신 속도 (기본: 500000)',
    )
    parser.add_argument(
        '--timeout',
        type=float,
        default=2.0,
        help='응답 타임아웃 (초, 기본: 2.0)',
    )
    parser.add_argument(
        '--stress',
        type=int,
        default=10,
        help='스트레스 테스트 반복 횟수 (기본: 10)',
    )
    parser.add_argument(
        '--ramp-samples',
        type=int,
        default=20,
        help='램프 테스트 샘플링 횟수 (기본: 20)',
    )
    parser.add_argument(
        '--ramp-interval',
        type=float,
        default=0.5,
        help='램프 테스트 샘플링 간격 (초, 기본: 0.5)',
    )
    parser.add_argument(
        '--no-ramp',
        action='store_true',
        help='램프 업/다운 테스트 생략',
    )
    parser.add_argument(
        '--no-stress',
        action='store_true',
        help='스트레스 테스트 생략',
    )
    parser.add_argument(
        '--setup-vcan',
        action='store_true',
        help='vcan 인터페이스 자동 설정 및 정리',
    )

    args = parser.parse_args()

    # 헤더 출력
    print("=" * 50)
    print("  OBD-II ECU 시뮬레이터 테스트 (python-can)")
    print(f"  인터페이스: {args.interface}")
    print(f"  속도: {args.bitrate} bps")
    print(f"  타임아웃: {args.timeout}s")
    print(f"  날짜: {time.strftime('%Y-%m-%d %H:%M:%S')}")
    print("=" * 50)

    # vcan 자동 설정
    if args.setup_vcan:
        print(Color.header("Virtual CAN 설정"))
        if setup_vcan(args.interface):
            print(Color.pass_msg(f"vcan 인터페이스 설정 완료: {args.interface}"))
        else:
            print(Color.fail_msg("vcan 인터페이스 설정 실패"))
            sys.exit(1)

    # 테스트 리포트 생성
    report = TestReport()

    try:
        # CAN 버스 연결
        tester = OBD2Tester(args.interface, args.bitrate)
        tester.connect()

        # ECU 시뮬레이터 응답 대기
        print(Color.info_msg("ECU 시뮬레이터 응답 대기 중..."))
        time.sleep(0.5)

        # =============================================
        # 1. 개별 PID 테스트
        # =============================================
        print(Color.header("개별 PID 테스트"))

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
                    print(f"    수신 프레임: {result.raw_data}")

        # =============================================
        # 2. 램프 업/다운 시뮬레이션 검증
        # =============================================
        if not args.no_ramp:
            print(Color.header("램프 업/다운 시뮬레이션 검증"))
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
        # 3. 스트레스 테스트
        # =============================================
        if not args.no_stress:
            print(Color.header("스트레스 테스트"))
            stress_results = tester.test_stress(
                pid=0x0D,  # 차속
                count=args.stress,
                timeout=args.timeout,
            )
            report.results.extend(stress_results)

        # CAN 버스 연결 종료
        tester.disconnect()

    except RuntimeError as e:
        print(Color.fail_msg(str(e)))
        sys.exit(1)
    except KeyboardInterrupt:
        print(f"\n{Color.YELLOW}사용자 중단${Color.NC}")
        sys.exit(130)
    finally:
        # vcan 정리
        if args.setup_vcan:
            cleanup_vcan(args.interface)
            print(Color.info_msg(f"vcan 인터페이스 정리: {args.interface}"))

    # 결과 출력
    all_passed = report.print_table()
    sys.exit(0 if all_passed else 1)


if __name__ == '__main__':
    main()
