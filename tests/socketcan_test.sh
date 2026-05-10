#!/bin/bash
#===============================================================================
# OBD-II ECU 시뮬레이터 SocketCAN 테스트 스크립트
#
# 설명:
#   Linux SocketCAN 인터페이스를 사용하여 OBD-II ECU 시뮬레이터를 테스트합니다.
#   can-utils 패키지(cansend, candump, cangen)가 필요합니다.
#   실제 CAN 어댑터가 없으면 virtual CAN (vcan)으로도 테스트 가능합니다.
#
# 사용법:
#   sudo ./socketcan_test.sh [CAN 인터페이스]
#   예: sudo ./socketcan_test.sh can0       # 실제 CAN 어댑터
#       sudo ./socketcan_test.sh vcan0      # virtual CAN (테스트용)
#
# 전제 조건:
#   1. can-utils 설치: sudo apt install can-utils
#   2. vcan 사용 시:   sudo modprobe vcan
#   3. root 권한 필요 (CAN 인터페이스 설정)
#
# OBD-II CAN ID 참고:
#   0x7E0 - OBD-II 요청 (ECU1, 테스터 -> ECU)
#   0x7E8 - OBD-II 응답 (ECU1, ECU -> 테스터)
#   0x7DF - 브로드캐스트 요청 (모든 ECU)
#   0x7E1 - OBD-II 요청 (ECU2)
#   0x7E9 - OBD-II 응답 (ECU2)
#
# ISO-TP 단일 프레임 (SF) 구조:
#   요청: [PCI 타입=01][길이][서비스ID=01][PID][패딩...]
#   예:   7E0#02010D0000000000  -> SF, 2바이트 데이터, 서비스01(PIDs), PID=0x0D(차속)
#   응답: [PCI 타입=01][길이][서비스ID=41][PID][데이터][패딩...]
#   예:   7E8#030410D000000000  -> SF, 4바이트 데이터, 서비스41(응답), PID=0x0D, 데이터=0x10
#===============================================================================

set -euo pipefail

#=======================================
# 설정 (기본값)
#=======================================
CAN_IF="${1:-can0}"              # CAN 인터페이스 이름 (기본: can0)
BITRATE="500000"                 # CAN 통신 속도 (500kbps, 자동차 표준)
REQUEST_ID="7E0"                 # OBD-II 요청 CAN ID (ECU1)
RESPONSE_ID="7E8"                # OBD-II 응답 CAN ID (ECU1)
TIMEOUT_SEC=2                    # 응답 대기 시간 (초)
STRESS_COUNT=10                  # 스트레스 테스트 반복 횟수
RAMP_SAMPLES=20                  # 램프 업/다운 샘플링 횟수
RAMP_INTERVAL=0.5                # 램프 테스트 샘플링 간격 (초)

# 카운터
PASS_COUNT=0
FAIL_COUNT=0
SKIP_COUNT=0

#=======================================
# 색상 출력 헬퍼
#=======================================
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m' # 색상 리셋

print_pass() { echo -e "  ${GREEN}[PASS]${NC} $1"; ((PASS_COUNT++)); }
print_fail() { echo -e "  ${RED}[FAIL]${NC} $1"; ((FAIL_COUNT++)); }
print_skip() { echo -e "  ${YELLOW}[SKIP]${NC} $1"; ((SKIP_COUNT++)); }
print_info() { echo -e "  ${CYAN}[INFO]${NC} $1"; }
print_header() { echo -e "\n${CYAN}=== $1 ===${NC}"; }

#=======================================
# 사전 확인
#=======================================
check_prerequisites() {
    print_header "사전 확인"

    # can-utils 설치 확인
    local missing_tools=()
    for tool in cansend candump cangen; do
        if ! command -v "$tool" &>/dev/null; then
            missing_tools+=("$tool")
        fi
    done

    if [[ ${#missing_tools[@]} -gt 0 ]]; then
        echo -e "  ${RED}[ERROR]${NC} 다음 도구가 설치되어 있지 않습니다: ${missing_tools[*]}"
        echo "  설치: sudo apt install can-utils"
        exit 1
    fi
    print_pass "can-utils 설치 확인"

    # ip 명령어 확인
    if ! command -v ip &>/dev/null; then
        echo -e "  ${RED}[ERROR]${NC} ip 명령어를 찾을 수 없습니다"
        exit 1
    fi
    print_pass "ip 명령어 확인"

    # root 권한 확인
    if [[ $EUID -ne 0 ]]; then
        echo -e "  ${RED}[ERROR]${NC} root 권한이 필요합니다. sudo로 실행하세요."
        echo "  사용법: sudo $0 $CAN_IF"
        exit 1
    fi
    print_pass "root 권한 확인"
}

#=======================================
# CAN 인터페이스 설정
#=======================================
setup_can_interface() {
    print_header "CAN 인터페이스 설정 (${CAN_IF})"

    # 인터페이스가 이미 올라와 있는지 확인
    if ip link show "$CAN_IF" &>/dev/null; then
        print_info "${CAN_IF} 인터페이스가 존재합니다. 재설정합니다."
        ip link set "$CAN_IF" down 2>/dev/null || true
    fi

    # vcan 인터페이스인 경우 모듈 로드 및 생성
    if [[ "$CAN_IF" == vcan* ]]; then
        # vcan 커널 모듈 로드
        if ! lsmod | grep -q vcan; then
            print_info "vcan 커널 모듈 로드 중..."
            modprobe vcan 2>/dev/null || true
        fi

        # vcan 인터페이스 생성
        if ! ip link show "$CAN_IF" &>/dev/null; then
            print_info "vcan 인터페이스 생성 중..."
            ip link add dev "$CAN_IF" type vcan
        fi
        ip link set "$CAN_IF" up
        print_pass "vcan 인터페이스 설정 완료 (${CAN_IF})"
    else
        # 실제 CAN 인터페이스 설정 (Classic CAN 500kbps)
        print_info "CAN 인터페이스 설정: bitrate=${BITRATE}"
        ip link set "$CAN_IF" type can bitrate "$BITRATE" 2>/dev/null
        ip link set "$CAN_IF" up
        print_pass "CAN 인터페이스 설정 완료 (${CAN_IF}, ${BITRATE}bps)"
    fi

    # 인터페이스 상태 확인
    if ip link show "$CAN_IF" | grep -q "UP"; then
        print_pass "인터페이스 상태: UP"
    else
        print_fail "인터페이스 상태 확인 실패"
        exit 1
    fi
}

#=======================================
# CAN 인터페이스 정리
#=======================================
cleanup_can_interface() {
    print_header "정리"

    # candump 백그라운드 프로세스 정리
    if [[ -n "${CANDUMP_PID:-}" ]] && kill -0 "$CANDUMP_PID" 2>/dev/null; then
        kill "$CANDUMP_PID" 2>/dev/null || true
        wait "$CANDUMP_PID" 2>/dev/null || true
        print_info "candump 프로세스 정리 완료 (PID: ${CANDUMP_PID})"
    fi

    # 캡처 파일 정리
    if [[ -f "${TMP_CAPTURE_FILE:-}" ]]; then
        rm -f "$TMP_CAPTURE_FILE"
    fi

    # 인터페이스 종료
    ip link set "$CAN_IF" down 2>/dev/null || true
    print_info "${CAN_IF} 인터페이스 종료"

    # vcan 인터페이스 삭제
    if [[ "$CAN_IF" == vcan* ]]; then
        ip link del dev "$CAN_IF" 2>/dev/null || true
        print_info "vcan 인터페이스 삭제"
    fi
}

#=======================================
# 응답 수신 함수
#=======================================
# 지정된 PID로 요청을 보내고 응답을 기다립니다.
# 인자:
#   $1 - PID (16진수, 예: "0D")
#   $2 - 응답 CAN ID (기본: 7E8)
# 반환:
#   표준 출력으로 수신된 CAN 프레임 (또는 빈 문자열)
send_obd2_request() {
    local pid="$1"
    local resp_id="${2:-$RESPONSE_ID}"
    local frame

    # ISO-TP 단일 프레임 요청 생성
    # PCI 타입 0x01 (SF), 길이 0x02, 서비스 0x01 (Show Current Data), PID
    frame="${REQUEST_ID}#02010${pid}00000000"

    # candump를 백그라운드에서 시작하여 응답 캡처
    # 지정된 응답 ID만 필터링
    TMP_CAPTURE_FILE=$(mktemp /tmp/obd2_capture_XXXXXX)
    timeout "${TIMEOUT_SEC}" candump "$CAN_IF,${resp_id}:7FF" -n 1 -T "${TIMEOUT_SEC}000" > "$TMP_CAPTURE_FILE" 2>/dev/null &
    local dump_pid=$!

    # 짧은 대기 후 요청 전송
    sleep 0.05
    cansend "$CAN_IF" "$frame" 2>/dev/null || true

    # 응답 대기
    wait "$dump_pid" 2>/dev/null || true

    # 캡처된 응답 반환
    if [[ -s "$TMP_CAPTURE_FILE" ]]; then
        cat "$TMP_CAPTURE_FILE"
    fi
}

#=======================================
# 응답 파싱 함수
#=======================================
# candump 출력에서 CAN 데이터를 추출합니다.
# candump 출력 형식: "<timestamp> <interface> <ID>#<DATA>"
# 인자:
#   $1 - candump 출력 라인
# 반환:
#   CAN 프레임 데이터 (16진수 문자열, 예: "030410D000000000")
parse_response() {
    local line="$1"
    # candump 형식에서 데이터 부분 추출: ... 7E8#030410D000000000
    echo "$line" | grep -oP '#\K[0-9A-Fa-f]+' | head -1
}

#=======================================
# 응답 검증 함수
#=======================================
# OBD-II 응답이 올바른 형식인지 검증합니다.
# 응답 형식 (단일 프레임):
#   바이트0: PCI 타입 (0x01 = SF) 또는 길이
#   바이트1: 데이터 길이 (PCI=0x01인 경우)
#   바이트2: 서비스 ID + 0x40 (0x41 = Show Current Data 응답)
#   바이트3: 요청한 PID
#   바이트4+: 응답 데이터
# 인자:
#   $1 - CAN 프레임 데이터 (16진수 문자열)
#   $2 - 예상 PID
# 반환:
#   0 = 검증 성공, 1 = 검증 실패
validate_response() {
    local data="$1"
    local expected_pid="$2"

    # 데이터 길이 확인 (최소 6바이트: PCI + 길이 + 서비스 + PID + 데이터 2바이트)
    local data_len=${#data}
    if [[ $data_len -lt 12 ]]; then  # 16진수 12자 = 6바이트
        return 1
    fi

    # 서비스 ID 확인: 바이트2(인덱스 4-5)가 "41"이어야 함 (0x01 + 0x40)
    local service_id="${data:4:2}"
    if [[ "${service_id^^}" != "41" ]]; then
        return 1
    fi

    # PID 확인: 바이트3(인덱스 6-7)가 요청한 PID와 일치해야 함
    local response_pid="${data:6:2}"
    if [[ "${response_pid^^}" != "${expected_pid^^}" ]]; then
        return 1
    fi

    return 0
}

#=======================================
# 개별 PID 테스트
#=======================================
test_single_pid() {
    local pid="$1"
    local pid_name="$2"
    local description="$3"
    local validate_fn="${4:-}"  # 선택적 추가 검증 함수

    echo -n "  테스트 PID 0x${pid} (${pid_name}): "

    # OBD-II 요청 전송 및 응답 수신
    local response
    response=$(send_obd2_request "$pid")

    if [[ -z "$response" ]]; then
        print_fail "${pid_name} - 응답 없음 (타임아웃 ${TIMEOUT_SEC}초)"
        return 1
    fi

    # 응답 파싱
    local frame_data
    frame_data=$(parse_response "$response")

    if [[ -z "$frame_data" ]]; then
        print_fail "${pid_name} - 응답 파싱 실패: ${response}"
        return 1
    fi

    print_info "수신 프레임: ${frame_data}"

    # 기본 응답 형식 검증
    if ! validate_response "$frame_data" "$pid"; then
        print_fail "${pid_name} - 응답 형식 오류: ${frame_data}"
        return 1
    fi

    # 추가 검증 (디코딩 값 확인)
    if [[ -n "$validate_fn" ]]; then
        if ! $validate_fn "$frame_data"; then
            print_fail "${pid_name} - ${description} 검증 실패: ${frame_data}"
            return 1
        fi
    fi

    print_pass "${pid_name} - ${description} (프레임: ${frame_data})"
    return 0
}

#=======================================
# PID 값 검증 함수들
#=======================================

# PID 0x00: 지원되는 PID 목록
# 응답 데이터 바이트4-5에 비트맵 (지원 PID 0x01-0x20)
validate_pid_00() {
    local data="$1"
    local supported_bitmap="${data:8:4}"

    # 최소한 PID 0x05, 0x0C, 0x0D는 지원되어야 함
    # 비트맵에서 해당 비트 확인
    # PID 0x05 -> 비트 5 -> 바이트0 비트4
    # PID 0x0C -> 비트 12 -> 바이트1 비트3
    # PID 0x0D -> 비트 13 -> 바이트1 비트4
    local byte0=$((16#${supported_bitmap:0:2}))
    local byte1=$((16#${supported_bitmap:2:2}))

    # PID 0x05 (bit 4 of byte0)
    if (( (byte0 & 0x10) == 0 )); then
        echo -e "    ${YELLOW}경고:${NC} PID 0x05(냉각수 온도) 미지원"
    fi
    # PID 0x0C (bit 3 of byte1)
    if (( (byte1 & 0x08) == 0 )); then
        echo -e "    ${YELLOW}경고:${NC} PID 0x0C(RPM) 미지원"
    fi
    # PID 0x0D (bit 4 of byte1)
    if (( (byte1 & 0x10) == 0 )); then
        echo -e "    ${YELLOW}경고:${NC} PID 0x0D(차속) 미지원"
    fi

    return 0  # 비트맵 검증은 경고만, 실패로 처리하지 않음
}

# PID 0x05: 냉각수 온도
# 디코딩: 온도(°C) = value - 40
# 유효 범위: -40°C ~ 215°C (1바이트)
validate_pid_05() {
    local data="$1"
    # 데이터 바이트4 (인덱스 8-9)
    local raw_value=$((16#${data:8:2}))
    local temp_c=$((raw_value - 40))

    # 합리적 범위 확인 (-40 ~ 150°C)
    if (( temp_c < -40 || temp_c > 150 )); then
        echo -e "    ${YELLOW}범위 경고:${NC} 냉각수 온도 ${temp_c}°C (원시값: 0x${data:8:2})"
    fi

    echo -e "    냉각수 온도: ${temp_c}°C (원시값: 0x${data:8:2})"
    return 0
}

# PID 0x0C: 엔진 RPM
# 디코딩: RPM = (A * 256 + B) / 4
# 유효 범위: 0 ~ 16383.75 RPM (2바이트)
validate_pid_0C() {
    local data="$1"
    # 데이터 바이트4-5 (인덱스 8-11)
    local byte_a=$((16#${data:8:2}))
    local byte_b=$((16#${data:10:2}))
    local rpm=$(( (byte_a * 256 + byte_b) / 4 ))

    # 합리적 범위 확인 (0 ~ 8000 RPM)
    if (( rpm > 8000 )); then
        echo -e "    ${YELLOW}범위 경고:${NC} RPM ${rpm} (원시값: 0x${data:8:2}${data:10:2})"
    fi

    echo -e "    엔진 RPM: ${rpm} (원시값: 0x${data:8:2}${data:10:2})"
    return 0
}

# PID 0x0D: 차량 속도
# 디코딩: 속도(km/h) = A (1바이트)
# 유효 범위: 0 ~ 255 km/h
validate_pid_0D() {
    local data="$1"
    # 데이터 바이트4 (인덱스 8-9)
    local speed=$((16#${data:8:2}))

    # 합리적 범위 확인 (0 ~ 255 km/h)
    if (( speed > 255 )); then
        echo -e "    ${YELLOW}범위 경고:${NC} 속도 ${speed} km/h"
    fi

    echo -e "    차량 속도: ${speed} km/h (원시값: 0x${data:8:2})"
    return 0
}

#=======================================
# 전체 PID 순차 테스트
#=======================================
run_all_pid_tests() {
    print_header "개별 PID 테스트"

    # PID 0x00: 지원되는 PID 목록
    test_single_pid "00" "지원 PID 목록" "PID 비트맵" "validate_pid_00"

    # PID 0x05: 냉각수 온도
    test_single_pid "05" "냉각수 온도" "온도 디코딩" "validate_pid_05"

    # PID 0x0C: 엔진 RPM
    test_single_pid "0C" "엔진 RPM" "RPM 디코딩" "validate_pid_0C"

    # PID 0x0D: 차량 속도
    test_single_pid "0D" "차량 속도" "속도 디코딩" "validate_pid_0D"
}

#=======================================
# 반복 스트레스 테스트
#=======================================
run_stress_test() {
    print_header "스트레스 테스트 (${STRESS_COUNT}회 반복)"

    local local_pass=0
    local local_fail=0
    local target_pid="0D"  # 차속 PID로 테스트

    for i in $(seq 1 "$STRESS_COUNT"); do
        echo -n "  [${i}/${STRESS_COUNT}] "

        local response
        response=$(send_obd2_request "$target_pid")

        if [[ -z "$response" ]]; then
            echo -e "${RED}응답 없음${NC}"
            ((local_fail++))
            ((FAIL_COUNT++))
            continue
        fi

        local frame_data
        frame_data=$(parse_response "$response")

        if validate_response "$frame_data" "$target_pid"; then
            echo -e "${GREEN}OK${NC} (프레임: ${frame_data})"
            ((local_pass++))
            ((PASS_COUNT++))
        else
            echo -e "${RED}형식 오류${NC} (프레임: ${frame_data})"
            ((local_fail++))
            ((FAIL_COUNT++))
        fi
    done

    echo ""
    print_info "스트레스 테스트 결과: ${GREEN}${local_pass} 성공${NC}, ${RED}${local_fail} 실패${NC} (합계: ${STRESS_COUNT})"
}

#=======================================
# 램프 업/다운 시뮬레이션 검증
#=======================================
# ECU 시뮬레이터가 램프 업/다운 패턴을 따르는지 확인합니다.
# RPM이 점진적으로 변하는지 여러 번 샘플링하여 검증합니다.
run_ramp_test() {
    print_header "램프 업/다운 시뮬레이션 검증"

    local pid="0C"  # RPM PID
    local samples=()
    local prev_rpm=-1
    local ramp_direction=""  # "up" 또는 "down"
    local direction_changes=0
    local valid_samples=0

    print_info "RPM을 ${RAMP_SAMPLES}회 샘플링합니다 (간격: ${RAMP_INTERVAL}s)"

    for i in $(seq 1 "$RAMP_SAMPLES"); do
        local response
        response=$(send_obd2_request "$pid")

        if [[ -z "$response" ]]; then
            echo -n "."
            continue
        fi

        local frame_data
        frame_data=$(parse_response "$response")

        if ! validate_response "$frame_data" "$pid"; then
            echo -n "x"
            continue
        fi

        # RPM 디코딩
        local byte_a=$((16#${frame_data:8:2}))
        local byte_b=$((16#${frame_data:10:2}))
        local rpm=$(( (byte_a * 256 + byte_b) / 4 ))

        samples+=("$rpm")
        ((valid_samples++))

        # 방향 변화 감지
        if [[ $prev_rpm -ge 0 ]]; then
            if (( rpm > prev_rpm )); then
                if [[ "$ramp_direction" == "down" ]]; then
                    ((direction_changes++))
                fi
                ramp_direction="up"
            elif (( rpm < prev_rpm )); then
                if [[ "$ramp_direction" == "up" ]]; then
                    ((direction_changes++))
                fi
                ramp_direction="down"
            fi
        else
            ramp_direction="unknown"
        fi

        prev_rpm=$rpm
        echo -n "."
        sleep "$RAMP_INTERVAL"
    done

    echo ""

    # 결과 분석
    if (( valid_samples < 3 )); then
        print_fail "유효 샘플 부족 (${valid_samples}/${RAMP_SAMPLES})"
        return
    fi

    # 샘플 값 출력
    echo -e "  샘플링된 RPM 값 (${valid_samples}개):"
    echo -n "  "
    for s in "${samples[@]}"; do
        printf "%6d" "$s"
    done
    echo ""

    # 통계 계산
    local min_rpm=${samples[0]}
    local max_rpm=${samples[0]}
    local sum=0
    for s in "${samples[@]}"; do
        ((sum += s))
        (( s < min_rpm )) && min_rpm=$s
        (( s > max_rpm )) && max_rpm=$s
    done
    local avg_rpm=$((sum / valid_samples))

    print_info "RPM 통계: 최소=${min_rpm}, 최대=${max_rpm}, 평균=${avg_rpm}"
    print_info "방향 전환 횟수: ${direction_changes}"

    # 램프 패턴 검증: 값이 변화하는지 확인
    if (( max_rpm == min_rpm )); then
        print_fail "RPM 값이 변화하지 않습니다 (모든 샘플이 ${min_rpm} RPM)"
    else
        print_pass "RPM 값이 변화합니다 (범위: ${min_rpm} ~ ${max_rpm} RPM)"
    fi
}

#=======================================
# 결과 요약
#=======================================
print_summary() {
    local total=$((PASS_COUNT + FAIL_COUNT + SKIP_COUNT))

    echo ""
    echo "========================================"
    echo "  테스트 결과 요약"
    echo "========================================"
    echo -e "  총 테스트:    ${total}"
    echo -e "  성공:         ${GREEN}${PASS_COUNT}${NC}"
    echo -e "  실패:         ${RED}${FAIL_COUNT}${NC}"
    echo -e "  건너뜀:       ${YELLOW}${SKIP_COUNT}${NC}"
    echo "========================================"

    if (( FAIL_COUNT > 0 )); then
        echo -e "  결과: ${RED}일부 테스트 실패${NC}"
        return 1
    else
        echo -e "  결과: ${GREEN}모든 테스트 통과${NC}"
        return 0
    fi
}

#=======================================
# 메인 실행
#=======================================
main() {
    echo "========================================"
    echo "  OBD-II ECU 시뮬레이터 SocketCAN 테스트"
    echo "  인터페이스: ${CAN_IF}"
    echo "  속도: ${BITRATE} bps"
    echo "  날짜: $(date '+%Y-%m-%d %H:%M:%S')"
    echo "========================================"

    # 정리 훅 등록 (스크립트 종료 시 실행)
    trap cleanup_can_interface EXIT INT TERM

    # 사전 확인
    check_prerequisites

    # CAN 인터페이스 설정
    setup_can_interface

    # ECU 시뮬레이터가 응답할 때까지 짧은 대기
    print_info "ECU 시뮬레이터 응답 대기 중..."
    sleep 0.5

    # 테스트 실행
    run_all_pid_tests
    run_ramp_test
    run_stress_test

    # 결과 요약
    print_summary
}

# 스크립트 직접 실행 시에만 main 호출
# source로 임포트된 경우 함수만 정의
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi
