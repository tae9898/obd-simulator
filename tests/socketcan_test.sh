#!/bin/bash
#===============================================================================
# OBD-II ECU Simulator SocketCAN Test Script
#
# Description:
#   Tests the OBD-II ECU simulator using the Linux SocketCAN interface.
#   Requires can-utils package (cansend, candump, cangen).
#   If no real CAN adapter is available, virtual CAN (vcan) can also be used.
#
# Usage:
#   sudo ./socketcan_test.sh [CAN interface]
#   Example: sudo ./socketcan_test.sh can0       # Real CAN adapter
#            sudo ./socketcan_test.sh vcan0      # virtual CAN (for testing)
#
# Prerequisites:
#   1. can-utils installed: sudo apt install can-utils
#   2. For vcan:          sudo modprobe vcan
#   3. Root permission required (CAN interface setup)
#
# OBD-II CAN ID reference:
#   0x7E0 - OBD-II request (ECU1, tester -> ECU)
#   0x7E8 - OBD-II response (ECU1, ECU -> tester)
#   0x7DF - Broadcast request (all ECUs)
#   0x7E1 - OBD-II request (ECU2)
#   0x7E9 - OBD-II response (ECU2)
#
# ISO-TP Single Frame (SF) structure:
#   Request:  [PCI type=01][length][service ID=01][PID][padding...]
#   Example:  7E0#02010D0000000000  -> SF, 2-byte data, service 01(PIDs), PID=0x0D(vehicle speed)
#   Response: [PCI type=01][length][service ID=41][PID][data][padding...]
#   Example:  7E8#030410D000000000  -> SF, 4-byte data, service 41(response), PID=0x0D, data=0x10
#===============================================================================

set -euo pipefail

#=======================================
# Configuration (defaults)
#=======================================
CAN_IF="${1:-can0}"              # CAN interface name (default: can0)
BITRATE="500000"                 # CAN bitrate (500kbps, automotive standard)
REQUEST_ID="7E0"                 # OBD-II request CAN ID (ECU1)
RESPONSE_ID="7E8"                # OBD-II response CAN ID (ECU1)
TIMEOUT_SEC=2                    # Response wait time (seconds)
STRESS_COUNT=10                  # Stress test iteration count
RAMP_SAMPLES=20                  # Ramp up/down sample count
RAMP_INTERVAL=0.5                # Ramp test sampling interval (seconds)

# Counters
PASS_COUNT=0
FAIL_COUNT=0
SKIP_COUNT=0

#=======================================
# Color output helpers
#=======================================
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m' # Color reset

print_pass() { echo -e "  ${GREEN}[PASS]${NC} $1"; ((++PASS_COUNT)); }
print_fail() { echo -e "  ${RED}[FAIL]${NC} $1"; ((++FAIL_COUNT)); }
print_skip() { echo -e "  ${YELLOW}[SKIP]${NC} $1"; ((++SKIP_COUNT)); }
# Note: pre-increment ((++VAR)) is used -- post-increment ((VAR++)) returns 0 when VAR==0,
# which becomes exit status 1 and terminates the script under set -euo pipefail.
print_info() { echo -e "  ${CYAN}[INFO]${NC} $1"; }
print_header() { echo -e "\n${CYAN}=== $1 ===${NC}"; }

#=======================================
# Pre-flight checks
#=======================================
check_prerequisites() {
    print_header "Pre-flight checks"

    # Check can-utils installation
    local missing_tools=()
    for tool in cansend candump cangen; do
        if ! command -v "$tool" &>/dev/null; then
            missing_tools+=("$tool")
        fi
    done

    if [[ ${#missing_tools[@]} -gt 0 ]]; then
        echo -e "  ${RED}[ERROR]${NC} The following tools are not installed: ${missing_tools[*]}"
        echo "  Install: sudo apt install can-utils"
        exit 1
    fi
    print_pass "can-utils installation verified"

    # Check ip command
    if ! command -v ip &>/dev/null; then
        echo -e "  ${RED}[ERROR]${NC} ip command not found"
        exit 1
    fi
    print_pass "ip command verified"

    # Check root permission
    if [[ $EUID -ne 0 ]]; then
        echo -e "  ${RED}[ERROR]${NC} Root permission required. Run with sudo."
        echo "  Usage: sudo $0 $CAN_IF"
        exit 1
    fi
    print_pass "Root permission verified"
}

#=======================================
# CAN interface setup
#=======================================
setup_can_interface() {
    print_header "CAN interface setup (${CAN_IF})"

    # Check if interface is already up
    if ip link show "$CAN_IF" &>/dev/null; then
        print_info "${CAN_IF} interface exists. Reconfiguring."
        ip link set "$CAN_IF" down 2>/dev/null || true
    fi

    # vcan interface: load module and create
    if [[ "$CAN_IF" == vcan* ]]; then
        # Load vcan kernel module
        if ! lsmod | grep -q vcan; then
            print_info "Loading vcan kernel module..."
            modprobe vcan 2>/dev/null || true
        fi

        # Create vcan interface
        if ! ip link show "$CAN_IF" &>/dev/null; then
            print_info "Creating vcan interface..."
            ip link add dev "$CAN_IF" type vcan
        fi
        ip link set "$CAN_IF" up
        print_pass "vcan interface setup complete (${CAN_IF})"
    else
        # Real CAN interface setup (Classic CAN 500kbps)
        print_info "CAN interface setup: bitrate=${BITRATE}"
        ip link set "$CAN_IF" type can bitrate "$BITRATE" 2>/dev/null
        ip link set "$CAN_IF" up
        print_pass "CAN interface setup complete (${CAN_IF}, ${BITRATE}bps)"
    fi

    # Verify interface state
    if ip link show "$CAN_IF" | grep -q "UP"; then
        print_pass "Interface state: UP"
    else
        print_fail "Interface state check failed"
        exit 1
    fi
}

#=======================================
# CAN interface cleanup
#=======================================
cleanup_can_interface() {
    print_header "Cleanup"

    # Cleanup candump background process
    if [[ -n "${CANDUMP_PID:-}" ]] && kill -0 "$CANDUMP_PID" 2>/dev/null; then
        kill "$CANDUMP_PID" 2>/dev/null || true
        wait "$CANDUMP_PID" 2>/dev/null || true
        print_info "candump process cleaned up (PID: ${CANDUMP_PID})"
    fi

    # Cleanup capture file
    if [[ -f "${TMP_CAPTURE_FILE:-}" ]]; then
        rm -f "$TMP_CAPTURE_FILE"
    fi

    # Bring down interface
    ip link set "$CAN_IF" down 2>/dev/null || true
    print_info "${CAN_IF} interface brought down"

    # Delete vcan interface
    if [[ "$CAN_IF" == vcan* ]]; then
        ip link del dev "$CAN_IF" 2>/dev/null || true
        print_info "vcan interface deleted"
    fi
}

#=======================================
# Response receive function
#=======================================
# Sends a request for the specified PID and waits for a response.
# Arguments:
#   $1 - PID (hex, e.g. "0D")
#   $2 - Response CAN ID (default: 7E8)
# Returns:
#   Received CAN frame on stdout (or empty string)
send_obd2_request() {
    local pid="$1"
    local resp_id="${2:-$RESPONSE_ID}"
    local frame

    # Build ISO-TP single frame request
    # PCI type 0x01 (SF), length 0x02, service 0x01 (Show Current Data), PID
    frame="${REQUEST_ID}#02010${pid}00000000"

    # Start candump in background to capture response
    # Filter for specified response ID only
    TMP_CAPTURE_FILE=$(mktemp /tmp/obd2_capture_XXXXXX)
    timeout "${TIMEOUT_SEC}" candump "$CAN_IF,${resp_id}:7FF" -n 1 -T "${TIMEOUT_SEC}000" > "$TMP_CAPTURE_FILE" 2>/dev/null &
    local dump_pid=$!

    # Brief wait then send request
    sleep 0.05
    cansend "$CAN_IF" "$frame" 2>/dev/null || true

    # Wait for response
    wait "$dump_pid" 2>/dev/null || true

    # Return captured response
    if [[ -s "$TMP_CAPTURE_FILE" ]]; then
        cat "$TMP_CAPTURE_FILE"
    fi
}

#=======================================
# Response parse function
#=======================================
# Extracts CAN data from candump output.
# candump output format: "<timestamp> <interface> <ID>#<DATA>"
# Arguments:
#   $1 - candump output line
# Returns:
#   CAN frame data (hex string, e.g. "030410D000000000")
parse_response() {
    local line="$1"
    # Extract data portion from candump format: ... 7E8#030410D000000000
    echo "$line" | grep -oP '#\K[0-9A-Fa-f]+' | head -1
}

#=======================================
# Response validation function
#=======================================
# Validates whether the OBD-II response has correct format.
# Response format (single frame):
#   Byte 0: PCI type (0x01 = SF) or length
#   Byte 1: Data length (when PCI=0x01)
#   Byte 2: Service ID + 0x40 (0x41 = Show Current Data response)
#   Byte 3: Requested PID
#   Byte 4+: Response data
# Arguments:
#   $1 - CAN frame data (hex string)
#   $2 - Expected PID
# Returns:
#   0 = validation pass, 1 = validation fail
validate_response() {
    local data="$1"
    local expected_pid="$2"

    # Check data length (minimum 6 bytes: PCI + length + service + PID + 2 data bytes)
    local data_len=${#data}
    if [[ $data_len -lt 12 ]]; then  # 12 hex chars = 6 bytes
        return 1
    fi

    # Check service ID: byte 2 (index 4-5) must be "41" (0x01 + 0x40)
    local service_id="${data:4:2}"
    if [[ "${service_id^^}" != "41" ]]; then
        return 1
    fi

    # Check PID: byte 3 (index 6-7) must match requested PID
    local response_pid="${data:6:2}"
    if [[ "${response_pid^^}" != "${expected_pid^^}" ]]; then
        return 1
    fi

    return 0
}

#=======================================
# Individual PID test
#=======================================
test_single_pid() {
    local pid="$1"
    local pid_name="$2"
    local description="$3"
    local validate_fn="${4:-}"  # Optional additional validation function

    echo -n "  Test PID 0x${pid} (${pid_name}): "

    # Send OBD-II request and receive response
    local response
    response=$(send_obd2_request "$pid")

    if [[ -z "$response" ]]; then
        print_fail "${pid_name} - No response (timeout ${TIMEOUT_SEC}s)"
        return 1
    fi

    # Parse response
    local frame_data
    frame_data=$(parse_response "$response")

    if [[ -z "$frame_data" ]]; then
        print_fail "${pid_name} - Response parse failed: ${response}"
        return 1
    fi

    print_info "Received frame: ${frame_data}"

    # Basic response format validation
    if ! validate_response "$frame_data" "$pid"; then
        print_fail "${pid_name} - Response format error: ${frame_data}"
        return 1
    fi

    # Additional validation (check decoded values)
    if [[ -n "$validate_fn" ]]; then
        if ! $validate_fn "$frame_data"; then
            print_fail "${pid_name} - ${description} validation failed: ${frame_data}"
            return 1
        fi
    fi

    print_pass "${pid_name} - ${description} (frame: ${frame_data})"
    return 0
}

#=======================================
# PID value validation functions
#=======================================

# PID 0x00: Supported PID list
# Response data bytes 4-5 contain bitmap (support for PIDs 0x01-0x20)
validate_pid_00() {
    local data="$1"
    local supported_bitmap="${data:8:4}"

    # At minimum, PID 0x05, 0x0C, 0x0D should be supported
    # Check corresponding bits in bitmap
    # PID 0x05 -> bit 5 -> byte 0 bit 4
    # PID 0x0C -> bit 12 -> byte 1 bit 3
    # PID 0x0D -> bit 13 -> byte 1 bit 4
    local byte0=$((16#${supported_bitmap:0:2}))
    local byte1=$((16#${supported_bitmap:2:2}))

    # PID 0x05 (bit 4 of byte0)
    if (( (byte0 & 0x10) == 0 )); then
        echo -e "    ${YELLOW}Warning:${NC} PID 0x05(coolant temp) not supported"
    fi
    # PID 0x0C (bit 3 of byte1)
    if (( (byte1 & 0x08) == 0 )); then
        echo -e "    ${YELLOW}Warning:${NC} PID 0x0C(RPM) not supported"
    fi
    # PID 0x0D (bit 4 of byte1)
    if (( (byte1 & 0x10) == 0 )); then
        echo -e "    ${YELLOW}Warning:${NC} PID 0x0D(vehicle speed) not supported"
    fi

    return 0  # Bitmap validation is warning-only, not treated as failure
}

# PID 0x05: Coolant temperature
# Decode: Temperature(C) = value - 40
# Valid range: -40C ~ 215C (1 byte)
validate_pid_05() {
    local data="$1"
    # Data byte 4 (index 8-9)
    local raw_value=$((16#${data:8:2}))
    local temp_c=$((raw_value - 40))

    # Check reasonable range (-40 ~ 150C)
    if (( temp_c < -40 || temp_c > 150 )); then
        echo -e "    ${YELLOW}Range warning:${NC} Coolant temp ${temp_c}C (raw: 0x${data:8:2})"
    fi

    echo -e "    Coolant temp: ${temp_c}C (raw: 0x${data:8:2})"
    return 0
}

# PID 0x0C: Engine RPM
# Decode: RPM = (A * 256 + B) / 4
# Valid range: 0 ~ 16383.75 RPM (2 bytes)
validate_pid_0C() {
    local data="$1"
    # Data bytes 4-5 (index 8-11)
    local byte_a=$((16#${data:8:2}))
    local byte_b=$((16#${data:10:2}))
    local rpm=$(( (byte_a * 256 + byte_b) / 4 ))

    # Check reasonable range (0 ~ 8000 RPM)
    if (( rpm > 8000 )); then
        echo -e "    ${YELLOW}Range warning:${NC} RPM ${rpm} (raw: 0x${data:8:2}${data:10:2})"
    fi

    echo -e "    Engine RPM: ${rpm} (raw: 0x${data:8:2}${data:10:2})"
    return 0
}

# PID 0x0D: Vehicle speed
# Decode: Speed(km/h) = A (1 byte)
# Valid range: 0 ~ 255 km/h
validate_pid_0D() {
    local data="$1"
    # Data byte 4 (index 8-9)
    local speed=$((16#${data:8:2}))

    # Check reasonable range (0 ~ 255 km/h)
    if (( speed > 255 )); then
        echo -e "    ${YELLOW}Range warning:${NC} Speed ${speed} km/h"
    fi

    echo -e "    Vehicle speed: ${speed} km/h (raw: 0x${data:8:2})"
    return 0
}

#=======================================
# All PID sequential tests
#=======================================
run_all_pid_tests() {
    print_header "Individual PID Tests"

    # PID 0x00: Supported PID list
    test_single_pid "00" "Supported PID list" "PID bitmap" "validate_pid_00"

    # PID 0x05: Coolant temperature
    test_single_pid "05" "Coolant temp" "Temperature decode" "validate_pid_05"

    # PID 0x0C: Engine RPM
    test_single_pid "0C" "Engine RPM" "RPM decode" "validate_pid_0C"

    # PID 0x0D: Vehicle speed
    test_single_pid "0D" "Vehicle speed" "Speed decode" "validate_pid_0D"
}

#=======================================
# Repeated stress test
#=======================================
run_stress_test() {
    print_header "Stress Test (${STRESS_COUNT} iterations)"

    local local_pass=0
    local local_fail=0
    local target_pid="0D"  # Test with vehicle speed PID

    for i in $(seq 1 "$STRESS_COUNT"); do
        echo -n "  [${i}/${STRESS_COUNT}] "

        local response
        response=$(send_obd2_request "$target_pid")

        if [[ -z "$response" ]]; then
            echo -e "${RED}No response${NC}"
            ((++local_fail))
            ((++FAIL_COUNT))
            continue
        fi

        local frame_data
        frame_data=$(parse_response "$response")

        if validate_response "$frame_data" "$target_pid"; then
            echo -e "${GREEN}OK${NC} (frame: ${frame_data})"
            ((++local_pass))
            ((++PASS_COUNT))
        else
            echo -e "${RED}Format error${NC} (frame: ${frame_data})"
            ((++local_fail))
            ((++FAIL_COUNT))
        fi
    done

    echo ""
    print_info "Stress test results: ${GREEN}${local_pass} passed${NC}, ${RED}${local_fail} failed${NC} (total: ${STRESS_COUNT})"
}

#=======================================
# Ramp up/down simulation verification
#=======================================
# Checks whether the ECU simulator follows a ramp up/down pattern.
# Verifies gradual RPM changes by sampling multiple times.
run_ramp_test() {
    print_header "Ramp Up/Down Simulation Verification"

    local pid="0C"  # RPM PID
    local samples=()
    local prev_rpm=-1
    local ramp_direction=""  # "up" or "down"
    local direction_changes=0
    local valid_samples=0

    print_info "Sampling RPM ${RAMP_SAMPLES} times (interval: ${RAMP_INTERVAL}s)"

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

        # RPM decode
        local byte_a=$((16#${frame_data:8:2}))
        local byte_b=$((16#${frame_data:10:2}))
        local rpm=$(( (byte_a * 256 + byte_b) / 4 ))

        samples+=("$rpm")
        ((++valid_samples))

        # Detect direction change
        if [[ $prev_rpm -ge 0 ]]; then
            if (( rpm > prev_rpm )); then
                if [[ "$ramp_direction" == "down" ]]; then
                    ((++direction_changes))
                fi
                ramp_direction="up"
            elif (( rpm < prev_rpm )); then
                if [[ "$ramp_direction" == "up" ]]; then
                    ((++direction_changes))
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

    # Analyze results
    if (( valid_samples < 3 )); then
        print_fail "Insufficient valid samples (${valid_samples}/${RAMP_SAMPLES})"
        return
    fi

    # Print sampled values
    echo -e "  Sampled RPM values (${valid_samples} samples):"
    echo -n "  "
    for s in "${samples[@]}"; do
        printf "%6d" "$s"
    done
    echo ""

    # Calculate statistics
    local min_rpm=${samples[0]}
    local max_rpm=${samples[0]}
    local sum=0
    for s in "${samples[@]}"; do
        sum=$((sum + s))
        (( s < min_rpm )) && min_rpm=$s
        (( s > max_rpm )) && max_rpm=$s
    done
    local avg_rpm=$((sum / valid_samples))

    print_info "RPM statistics: min=${min_rpm}, max=${max_rpm}, avg=${avg_rpm}"
    print_info "Direction changes: ${direction_changes}"

    # Ramp pattern verification: check if values change
    if (( max_rpm == min_rpm )); then
        print_fail "RPM values not changing (all samples at ${min_rpm} RPM)"
    else
        print_pass "RPM values changing (range: ${min_rpm} ~ ${max_rpm} RPM)"
    fi
}

#=======================================
# Result summary
#=======================================
print_summary() {
    local total=$((PASS_COUNT + FAIL_COUNT + SKIP_COUNT))

    echo ""
    echo "========================================"
    echo "  Test Results Summary"
    echo "========================================"
    echo -e "  Total tests:    ${total}"
    echo -e "  Passed:         ${GREEN}${PASS_COUNT}${NC}"
    echo -e "  Failed:         ${RED}${FAIL_COUNT}${NC}"
    echo -e "  Skipped:        ${YELLOW}${SKIP_COUNT}${NC}"
    echo "========================================"

    if (( FAIL_COUNT > 0 )); then
        echo -e "  Result: ${RED}Some tests failed${NC}"
        return 1
    else
        echo -e "  Result: ${GREEN}All tests passed${NC}"
        return 0
    fi
}

#=======================================
# Main execution
#=======================================
main() {
    echo "========================================"
    echo "  OBD-II ECU Simulator SocketCAN Test"
    echo "  Interface: ${CAN_IF}"
    echo "  Bitrate: ${BITRATE} bps"
    echo "  Date: $(date '+%Y-%m-%d %H:%M:%S')"
    echo "========================================"

    # Register cleanup hook (runs on script exit)
    trap cleanup_can_interface EXIT INT TERM

    # Pre-flight checks
    check_prerequisites

    # CAN interface setup
    setup_can_interface

    # Brief wait for ECU simulator response
    print_info "Waiting for ECU simulator response..."
    sleep 0.5

    # Run tests
    run_all_pid_tests
    run_ramp_test
    run_stress_test

    # Print summary
    print_summary
}

# Only call main when script is executed directly
# When sourced via source, only define functions
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi
