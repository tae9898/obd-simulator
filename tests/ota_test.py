#!/usr/bin/env python3
"""
OTA Sequence Test (UDS 0x34/0x36/0x37) -- raw socketcan FD direct.
Workaround for python-can 4.6.1 gs_usb FD bug.

Usage: sudo python3 tests/ota_test.py can0
Prerequisites: ECU booted (LED blinking, HSE oscillating), can0 up, boot delay(1s) elapsed.

Verification: UDS sequence success via 0x76/0x77 responses.
Verify actual flash data via SWD(gdb) at 0x0801E000 (instructions at end of script).
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
    """SecurityAccess key = ROL3(seed ^ 0x5A3C), 16-bit."""
    return ((x << 3) | (x >> 13)) & 0xFFFF


def make_frame(can_id, data):
    dlc = len(data)
    return struct.pack(CANFD_FRAME_FMT, can_id, dlc, 0, bytes(data))


def send_uds(s, payload):
    """ISO-TP SF encoding (<=7 classic SF, 8~62 FD escape SF) then transmit."""
    if len(payload) <= 7:
        iso = bytes([len(payload)]) + payload
    else:
        iso = bytes([0x00, len(payload)]) + payload
    s.send(make_frame(REQ_ID, iso))


def recv_uds(s, timeout=2.0):
    """Parse response (SF). Multi-frame responses not supported (responses are all short SF)."""
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
        print(f"  [{label}] TIMEOUT (no response)")
        return False
    if r[0] == 0x7F:
        print(f"  [{label}] NRC 0x{r[2]:02X} (failed)  resp={r.hex()}")
        return False
    ok = (r[0] == want_sid)
    print(f"  [{label}] resp={r.hex()} {'OK' if ok else 'FAIL (expected 0x%02X)' % want_sid}")
    return ok


def main():
    if len(sys.argv) < 2:
        print("Usage: sudo python3 tests/ota_test.py can0")
        sys.exit(1)
    dev = sys.argv[1]

    s = socket.socket(socket.PF_CAN, socket.SOCK_RAW, CAN_RAW)
    s.bind((dev,))
    s.setsockopt(SOL_CAN_RAW, CAN_RAW_FD_FRAMES, 1)
    s.setsockopt(SOL_CAN_RAW, CAN_RAW_FILTER, struct.pack('=II', RESP_ID, 0x7FF))

    print("=== OTA Sequence Test (UDS 0x34/0x36/0x37) ===")
    print("(Prerequisites: ECU booted (LED blinking), can0 up, boot delay elapsed)\n")
    time.sleep(0.2)

    # 1. Extended session
    send_uds(s, bytes([0x10, 0x03]))
    if not expect(s, "0x10 Extended", 0x50):
        return

    # 2. Request seed
    send_uds(s, bytes([0x27, 0x01]))
    r = recv_uds(s)
    if r is None or r[0] != 0x67:
        print(f"  [seed] failed resp={r}")
        return
    seed = (r[2] << 8) | r[3]
    key = rol3(seed ^ 0x5A3C)
    print(f"  seed=0x{seed:04X} -> key=0x{key:04X}")

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

    # 5. 0x36 TransferData block1 (dummy data 0xAA * 8)
    blk = bytes([0x36, 0x01]) + bytes([0xAA] * 8)
    send_uds(s, blk)
    if not expect(s, "0x36 block1(flash write)", 0x76):
        return

    # 6. 0x37 RequestTransferExit
    send_uds(s, bytes([0x37]))
    if not expect(s, "0x37 exit", 0x77):
        return

    print("\n=== UDS sequence success (responses OK) ===")
    print("Verify actual flash write (SWD/gdb):")
    print("  st-util -p 4242 &")
    print("  arm-none-eabi-gdb -batch \\")
    print("    -ex 'target remote :4242' -ex 'monitor halt' \\")
    print("    -ex 'x/8bx 0x0801E000'    # 0xAA 8 times means write success")
    s.close()


if __name__ == '__main__':
    main()
