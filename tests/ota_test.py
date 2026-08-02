#!/usr/bin/env python3
"""
OTA 시퀀스 테스트 (UDS 0x34/0x36/0x37) — raw socketcan FD 직접.
python-can 4.6.1 gs_usb FD 버그 우회용.

사용: sudo python3 tests/ota_test.py can0
전제: ECU 부팅됨(LED 깜빡임, HSE 발진), can0 up, boot delay(1s) 경과.

검증: 0x76/0x77 응답으로 UDS 시퀀스 성공 여부.
flash 실제 데이터는 SWD(gdb)로 0x0801E000 읽어 확인 (스크립트 끝에 안내).
"""
import socket
import struct
import sys
import time

CAN_RAW = 1
SOL_CAN_RAW = 101
CAN_RAW_FILTER = 1
CAN_RAW_FD_FRAMES = 5
CANFD_FRAME_FMT = '=IBB2x64s'   # can_id(4) + len(1) + flags(1) + pad(2) + data(64) = 72

REQ_ID = 0x7E0
RESP_ID = 0x7E8


def rol3(x):
    """SecurityAccess key = ROL3(seed ^ 0x5A3C), 16비트."""
    return ((x << 3) | (x >> 13)) & 0xFFFF


def make_frame(can_id, data):
    dlc = len(data)
    return struct.pack(CANFD_FRAME_FMT, can_id, dlc, 0, bytes(data))


def send_uds(s, payload):
    """ISO-TP SF 인코딩(<=7 classic SF, 8~62 FD escape SF) 후 전송."""
    if len(payload) <= 7:
        iso = bytes([len(payload)]) + payload
    else:
        iso = bytes([0x00, len(payload)]) + payload
    s.send(make_frame(REQ_ID, iso))


def recv_uds(s, timeout=2.0):
    """응답(SF) 파싱. 멀티프레임 응답은 미지원(응답 다 짧아 SF)."""
    s.settimeout(timeout)
    try:
        frame = s.recv(72)
    except socket.timeout:
        return None
    _can_id, dlc, _flags = struct.unpack('=IBB', frame[:6])
    data = frame[8:8 + dlc]
    if len(data) >= 2 and data[0] == 0x00:        # FD escape SF
        tlen = data[1]
        return data[2:2 + tlen]
    if len(data) >= 1:                             # classic SF
        tlen = data[0] & 0x0F
        return data[1:1 + tlen]
    return None


def expect(s, label, want_sid):
    r = recv_uds(s)
    if r is None:
        print(f"  [{label}] TIMEOUT (응답 없음) ❌")
        return False
    if r[0] == 0x7F:
        print(f"  [{label}] NRC 0x{r[2]:02X} (실패) ❌  resp={r.hex()}")
        return False
    ok = (r[0] == want_sid)
    print(f"  [{label}] resp={r.hex()} {'✓' if ok else '❌(예상 0x%02X)' % want_sid}")
    return ok


def main():
    if len(sys.argv) < 2:
        print("사용: sudo python3 tests/ota_test.py can0")
        sys.exit(1)
    dev = sys.argv[1]

    s = socket.socket(socket.PF_CAN, socket.SOCK_RAW, CAN_RAW)
    s.bind((dev,))
    s.setsockopt(SOL_CAN_RAW, CAN_RAW_FD_FRAMES, 1)
    s.setsockopt(SOL_CAN_RAW, CAN_RAW_FILTER, struct.pack('=II', RESP_ID, 0x7FF))

    print("=== OTA 시퀀스 테스트 (UDS 0x34/0x36/0x37) ===")
    print("(전제: ECU 부팅(LED 깜빡임), can0 up, boot delay 경과)\n")
    time.sleep(0.2)

    # 1. Extended 세션
    send_uds(s, bytes([0x10, 0x03]))
    if not expect(s, "0x10 Extended", 0x50):
        return

    # 2. seed 요청
    send_uds(s, bytes([0x27, 0x01]))
    r = recv_uds(s)
    if r is None or r[0] != 0x67:
        print(f"  [seed] 실패 resp={r} ❌")
        return
    seed = (r[2] << 8) | r[3]
    key = rol3(seed ^ 0x5A3C)
    print(f"  seed=0x{seed:04X} → key=0x{key:04X}")

    # 3. sendKey (unlock)
    send_uds(s, bytes([0x27, 0x02, (key >> 8) & 0xFF, key & 0xFF]))
    if not expect(s, "0x27 unlock", 0x67):
        return

    # 4. 0x34 RequestDownload: addr=0x0801E000, size=8 (ALFID 0x44 = addr4/size4)
    dl = bytes([0x34, 0x44,
                0x08, 0x01, 0xE0, 0x00,   # addr
                0x00, 0x00, 0x00, 0x08])  # size=8
    send_uds(s, dl)
    if not expect(s, "0x34 download(erase)", 0x74):
        return

    # 5. 0x36 TransferData 블록1 (가짜 데이터 0xAA * 8)
    blk = bytes([0x36, 0x01]) + bytes([0xAA] * 8)
    send_uds(s, blk)
    if not expect(s, "0x36 block1(flash write)", 0x76):
        return

    # 6. 0x37 RequestTransferExit
    send_uds(s, bytes([0x37]))
    if not expect(s, "0x37 exit", 0x77):
        return

    print("\n=== UDS 시퀀스 성공 (응답 정상) ===")
    print("flash 실제 쓰기 확인 (SWD/gdb):")
    print("  st-util -p 4242 &")
    print("  arm-none-eabi-gdb -batch \\")
    print("    -ex 'target remote :4242' -ex 'monitor halt' \\")
    print("    -ex 'x/8bx 0x0801E000'    # 0xAA 8번 나오면 쓰기 성공")
    s.close()


if __name__ == '__main__':
    main()
