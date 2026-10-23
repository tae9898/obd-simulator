#!/usr/bin/env python3
"""
FBL (Bootloader) UDS Flash Test

Tests the bootloader flash-server path end-to-end over CAN-FD:
    enter BL (0x10 02 handoff) -> 0x27 unlock -> 0x34/0x36/0x37 ->
    0x31 0x0203 CheckMemory (CRC32 + anti-rollback) -> 0x11 -> new app boots.

Reuses UDSTester/report infra from uds_test.py. Multi-frame (FF/CF) sender
added: 0x36 blocks exceed single-frame size.

Usage:
    sudo python3 tests/fbl_test.py can0
    Image flashed: build/phase0-obd-simulator.bin (rebuilt by `make all`)

Note:
    - 0x34 erases up to 46 flash pages with IRQs off (~1.3s): timeout 5s.
    - Anti-rollback floor persists in the meta page across runs: positive
      flashes use version = unix time (monotonic); the negative test uses
      version 1, which is always below an already-established floor.
"""

import os
import sys
import time
import binascii

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from uds_test import (UDSTester, Color, TestStatus, TestResult, TestReport,
                      nrc_name)

APP_BIN = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       '..', 'build', 'phase0-obd-simulator.bin')
APP_BASE = 0x08007000
MAX_BLOCK = 248          # BL 0x34 maxNumberOfBlockLength
PASS_VERSION = int(time.time()) & 0xFFFFFFFF   # monotonic across runs
LOW_VERSION = 1


#=======================================
# ISO-TP multi-frame sender + helpers
#=======================================
def send_uds(tester: UDSTester, payload: bytes, timeout: float = 2.0):
    """Send one UDS request (SF or FF/CF per size). Send-only half of
    send_recv_uds, split out so pending-response waits can reuse it.

    SF  (<=7B):  classic 8B frame, zero-padded (ECU accepts classic requests)
    SF  (<=62B): CAN-FD escape SF [0x00][len][payload] -- a bare [len] prefix
                 would collide with the FF PCI (e.g. len 22 = 0x16 = "FF, 1590B")
    FF/CF (>62B): first frame 0x10|hi lo + 62B, wait FC (BS=0 from ECU),
                  consecutive frames 0x20|seq + 63B, spaced by FC STmin
    """
    bus = tester.bus
    n = len(payload)

    if n <= 62:
        if n <= 7:
            # classic SF: [PCI=len][payload][pad to 8]
            msg = can_Message(tester, [n] + list(payload) + [0] * (7 - n), is_fd=False)
        else:
            # CAN-FD escape SF: [0x00][len][payload][pad to 64]
            msg = can_Message(tester, [0x00, n] + list(payload) + [0] * (62 - n), is_fd=True)
        bus.send(msg)
    else:
        # First Frame (12-bit length)
        ff = [0x10 | (n >> 8), n & 0xFF] + list(payload[:62])
        ff += [0] * (64 - len(ff))
        bus.send(can_Message(tester, ff, is_fd=True))

        # Flow control from ECU
        deadline = time.time() + timeout
        fc = None
        while time.time() < deadline:
            m = bus.recv(timeout=0.5)
            if m is None:
                continue
            if m.arbitration_id == tester.RESPONSE_ID and len(m.data) >= 3 \
               and (m.data[0] & 0xF0) == 0x30:
                fc = m.data
                break
        if fc is None:
            return
        stmin_ms = fc[2] & 0x7F if fc[2] < 0xF9 else 0
        stmin_ms = max(stmin_ms, 0)

        # Consecutive frames
        seq = 1
        off = 62
        while off < n:
            chunk = payload[off:off + 63]
            cf = [0x20 | (seq & 0x0F)] + list(chunk)
            cf += [0] * (64 - len(cf))
            bus.send(can_Message(tester, cf, is_fd=True))
            seq = (seq + 1) & 0x0F
            off += 63
            if stmin_ms:
                time.sleep(stmin_ms / 1000.0)
            elif off < n:
                time.sleep(0.002)   # be gentle even at STmin=0


def recv_uds(tester: UDSTester, timeout: float):
    """Next valid ECU response frame (all FBL responses fit a single frame).
    A flaky bus yields error frames / ghost IDs / late stale responses around
    the real one -- skip anything not from the ECU instead of giving up."""
    deadline = time.time() + timeout
    while True:
        remain = deadline - time.time()
        if remain <= 0:
            return None
        resp = bus.recv(timeout=min(remain, 0.5))
        if resp is None:
            continue
        if getattr(resp, 'is_error_frame', False):
            continue
        if resp.arbitration_id != tester.RESPONSE_ID:
            continue
        return bytes(resp.data)


def send_recv_uds(tester: UDSTester, payload: bytes, timeout: float = 2.0):
    send_uds(tester, payload, timeout)
    return recv_uds(tester, timeout)


def can_Message(tester, data, is_fd):
    import can
    return can.Message(arbitration_id=tester.REQUEST_ID,
                       data=data, is_extended_id=False, is_fd=is_fd)


def parse_uds(tester, resp, sid):
    """-> ('pos', payload) | ('nrc', code) | (None, None)"""
    if resp is None:
        return None, None
    pos = tester.parse_positive(resp, sid)
    if pos:
        return 'pos', pos['payload']
    neg = tester.parse_negative(resp)
    if neg:
        return 'nrc', neg['nrc']
    return None, None


def check(report, name, kind, code, expect_kind, expect_code=None,
          raw_tx="", raw_rx=""):
    ok = (kind == expect_kind) and \
         (expect_code is None or code == expect_code)
    r = TestResult(name)
    r.status = TestStatus.PASS if ok else TestStatus.FAIL
    if ok:
        r.detail = "positive" if kind == 'pos' else f"NRC 0x{code:02X} (expected)"
    elif kind is None:
        r.detail = "timeout/parse"
    elif isinstance(code, (bytes, bytearray)):
        r.detail = f"got pos {code.hex()}"
    elif isinstance(code, int):
        r.detail = f"got {kind} 0x{code:02X}"
    else:
        r.detail = f"got {kind} {code}"
    r.raw_tx, r.raw_rx = raw_tx, raw_rx
    print(f"  {r.status.value}: {name} -- {r.detail}")
    report.add(r)
    return ok


def in_app(tester):
    """App alive? (0x22 F190 positive -> app; NRC 0x11 -> BL)"""
    kind, code = parse_uds(tester, send_recv_uds(tester, bytes([0x22, 0xF1, 0x90])), 0x22)
    return kind == 'pos'


def enter_bl(tester, report, wait_unlock_delay=True):
    """Get into the BL in programming session.
    App alive  : 0x10 02 -> handoff reset -> BL boots with flag consumed.
    BL resident: (e.g. after a failed update) 0x10 02 switches its session."""
    if in_app(tester):
        # positive 0x50 response, then app resets (~50ms) into the BL
        send_recv_uds(tester, bytes([0x10, 0x02]))
    # poll: BL = 0x22 F190 -> NRC 0x11
    deadline = time.time() + 4.0
    while time.time() < deadline:
        kind, code = parse_uds(tester, send_recv_uds(tester, bytes([0x22, 0xF1, 0x90])), 0x22)
        if kind == 'nrc' and code == 0x11:
            # ensure programming session in the BL (no-op when arriving
            # via the handoff flag -- BL already set it at boot)
            send_recv_uds(tester, bytes([0x10, 0x02]))
            if wait_unlock_delay:
                time.sleep(1.2)   # 0x27 boot delay (NRC 0x37 window)
            return True
        if kind == 'pos':
            # app still answering: the 0x10 02 was lost on the wire -- retry
            send_recv_uds(tester, bytes([0x10, 0x02]))
        time.sleep(0.2)
    return False


def unlock(tester, report, name="0x27 unlock"):
    kind, code = parse_uds(tester, send_recv_uds(tester, bytes([0x27, 0x01])), 0x27)
    if kind == 'nrc' and code == 0x37:
        # security boot-delay (1s from BL boot-ready) not expired yet -- wait it out
        time.sleep(1.2)
        kind, code = parse_uds(tester, send_recv_uds(tester, bytes([0x27, 0x01])), 0x27)
    if kind != 'pos' or len(code) < 3:   # payload = [level, seed_hi, seed_lo]
        check(report, name, kind, code, 'pos', raw_tx="27 01")
        return False
    seed = (code[1] << 8) | code[2]
    key = tester.compute_key(seed)
    kind2, code2 = parse_uds(tester, send_recv_uds(tester, bytes([0x27, 0x02, key >> 8, key & 0xFF])), 0x27)
    return check(report, name, kind2, code2, 'pos',
                 raw_tx=f"27 02 key={key:04X}")


def request_download(tester, size):
    """0x34 with NRC 0x78 handling: the BL sends responsePending before the
    slot erase (~1s), so keep waiting for the final 0x74/NRC within one P2*
    style budget (5s) instead of returning on the first frame."""
    req = bytes([0x34, 0x44]) + APP_BASE.to_bytes(4, 'big') + size.to_bytes(4, 'big')
    deadline = time.time() + 5.0
    send_uds(tester, req)
    kind, code = parse_uds(tester, recv_uds(tester, deadline - time.time()), 0x34)
    while kind == 'nrc' and code == 0x78:
        kind, code = parse_uds(tester, recv_uds(tester, deadline - time.time()), 0x34)
    return kind, code


def transfer_blocks(tester, image: bytes, skip_block=None):
    """0x36 all blocks (seq from 1). skip_block=i zeroes block i's data (brick
    test): the block sequence stays contiguous (skipping a seq number would
    rightly earn 0x73) and the corrupted data breaks the image CRC instead."""
    seq = 1
    for off in range(0, len(image), MAX_BLOCK):
        block = image[off:off + MAX_BLOCK]
        if skip_block is not None and (off // MAX_BLOCK) == skip_block:
            block = bytes(len(block))   # same length, wrong data
        req = bytes([0x36, seq & 0xFF]) + block
        kind, code = parse_uds(tester, send_recv_uds(tester, req, timeout=5.0), 0x36)
        if kind != 'pos':
            return (kind, code)
        seq += 1
    return ('pos', None)


def check_memory(tester, version, crc):
    req = bytes([0x31, 0x01, 0x02, 0x03]) + version.to_bytes(4, 'big') + crc.to_bytes(4, 'big')
    return parse_uds(tester, send_recv_uds(tester, req, timeout=5.0), 0x31)


def flash_full_cycle(tester, report, image, version, tag=""):
    """0x34 -> 0x36* -> 0x37 -> 0x31 (CheckMemory). Returns True on success."""
    size = len(image)
    kind, code = request_download(tester, size)
    if not check(report, f"0x34 download start {tag}", kind, code, 'pos',
                 raw_tx=f"34 44 {APP_BASE:08X} {size:08X}"):
        return False
    kind, code = transfer_blocks(tester, image)
    if not check(report, f"0x36 transfer {size}B {tag}", kind, code, 'pos'):
        return False
    kind, code = parse_uds(tester, send_recv_uds(tester, bytes([0x37])), 0x37)
    if not check(report, f"0x37 transfer exit {tag}", kind, code, 'pos'):
        return False
    crc = binascii.crc32(image) & 0xFFFFFFFF
    kind, code = check_memory(tester, version, crc)
    return check(report, f"0x31 CheckMemory v{version} {tag}", kind, code, 'pos',
                 raw_tx=f"31 01 02 03 v={version} crc={crc:08X}")


#=======================================
# Test cases
#=======================================
def test_enter_and_gate(tester, report):
    print(Color.header("FBL: enter BL + gating"))
    ok = enter_bl(tester, report)
    check(report, "enter BL via 0x10 02 handoff",
          'pos' if ok else None, None, 'pos')

    # locked 0x34 -> NRC 0x33 (before unlock)
    kind, code = request_download(tester, os.path.getsize(APP_BIN))
    check(report, "0x34 locked -> 0x33", kind, code, 'nrc', 0x33)

    # 0x36 without 0x34 (after unlock) -> NRC 0x22
    if unlock(tester, report):
        kind, code = parse_uds(tester, send_recv_uds(tester, bytes([0x36, 0x01, 0xAA])), 0x36)
        check(report, "0x36 no download -> 0x22", kind, code, 'nrc', 0x22)

    # 0x34 wrong address -> NRC 0x31
    req = bytes([0x34, 0x44]) + (0x08000000).to_bytes(4, 'big') + (1024).to_bytes(4, 'big')
    kind, code = parse_uds(tester, send_recv_uds(tester, req, timeout=5.0), 0x34)
    check(report, "0x34 wrong addr (BL area) -> 0x31", kind, code, 'nrc', 0x31)


def test_update_cycle(tester, report, image):
    print(Color.header("FBL: full update cycle (happy path)"))
    if not flash_full_cycle(tester, report, image, PASS_VERSION, "(v-up)"):
        return
    # 0x11 -> reset -> app must answer VIN again
    kind, code = parse_uds(tester, send_recv_uds(tester, bytes([0x11, 0x01])), 0x11)
    check(report, "0x11 reset", kind, code, 'pos')
    deadline = time.time() + 4.0
    alive = False
    while time.time() < deadline:
        if in_app(tester):
            alive = True
            break
        time.sleep(0.2)
    check(report, "app boots after update (0x22 F190)", 'pos' if alive else None,
          None, 'pos')


def test_brick_and_recovery(tester, report, image):
    print(Color.header("FBL: brick path (CRC mismatch) + recovery"))
    if not enter_bl(tester, report) or not unlock(tester, report):
        return
    size = len(image)
    kind, code = request_download(tester, size)
    if not check(report, "0x34 download start (brick)", kind, code, 'pos'):
        return
    # write all blocks except block 0 -> CRC will mismatch
    kind, code = transfer_blocks(tester, image, skip_block=0)
    if not check(report, "0x36 transfer missing block 0", kind, code, 'pos'):
        return
    parse_uds(tester, send_recv_uds(tester, bytes([0x37])), 0x37)
    crc_full = binascii.crc32(image) & 0xFFFFFFFF
    kind, code = check_memory(tester, PASS_VERSION + 1, crc_full)
    check(report, "0x31 CRC mismatch -> 0x72", kind, code, 'nrc', 0x72)

    # reset -> boot gate must hold the ECU in BL (meta stuck UPDATING)
    parse_uds(tester, send_recv_uds(tester, bytes([0x11, 0x01])), 0x11)
    time.sleep(1.0)
    check(report, "after failed update: stays in BL (0x22 -> 0x11)",
          'nrc' if not in_app(tester) else 'pos', 0x11 if not in_app(tester) else None,
          'nrc', 0x11)

    # recovery: full cycle on top. enter_bl first: after the 0x11 reset the
    # resident BL boots WITHOUT the handoff flag -> session is Default, and
    # 0x34 would answer 0x33 until 0x10 02 re-enters programming.
    if enter_bl(tester, report) and unlock(tester, report, "0x27 unlock (recovery)"):
        if flash_full_cycle(tester, report, image, PASS_VERSION + 2, "(recovery)"):
            parse_uds(tester, send_recv_uds(tester, bytes([0x11, 0x01])), 0x11)
            time.sleep(1.0)
            check(report, "recovery flash boots app",
                  'pos' if in_app(tester) else None, None, 'pos')


def test_anti_rollback(tester, report, image):
    print(Color.header("FBL: anti-rollback + wrong sequence"))
    if not enter_bl(tester, report) or not unlock(tester, report):
        return
    size = len(image)
    kind, code = request_download(tester, size)
    if not check(report, "0x34 download start (rollback)", kind, code, 'pos'):
        return
    # wrong block sequence mid-transfer -> 0x73
    block = image[:MAX_BLOCK]
    kind, code = parse_uds(tester, send_recv_uds(tester, bytes([0x36, 0x01]) + block, timeout=5.0), 0x36)
    if check(report, "0x36 block #1", kind, code, 'pos'):
        kind, code = parse_uds(tester, send_recv_uds(tester, bytes([0x36, 0x05]) + image[MAX_BLOCK:2*MAX_BLOCK], timeout=5.0), 0x36)
        check(report, "0x36 wrong seq (5 after 1) -> 0x73", kind, code, 'nrc', 0x73)
    # resume: continue with correct seq (2..) to complete the image
    image_rest = image[MAX_BLOCK:]
    seq = 2
    for off in range(0, len(image_rest), MAX_BLOCK):
        chunk = image_rest[off:off + MAX_BLOCK]
        kind, code = parse_uds(tester, send_recv_uds(tester, bytes([0x36, seq & 0xFF]) + chunk, timeout=5.0), 0x36)
        if kind != 'pos':
            check(report, "0x36 resume transfer", kind, code, 'pos')
            return
        seq += 1
    check(report, "0x36 resume transfer", 'pos', None, 'pos')
    parse_uds(tester, send_recv_uds(tester, bytes([0x37])), 0x37)
    crc = binascii.crc32(image) & 0xFFFFFFFF
    kind, code = check_memory(tester, LOW_VERSION, crc)
    check(report, f"0x31 version {LOW_VERSION} < floor -> 0x31", kind, code, 'nrc', 0x31)
    # same image, valid version -> accepted
    kind, code = check_memory(tester, PASS_VERSION + 3, crc)
    if check(report, f"0x31 version {PASS_VERSION + 3} accepted", kind, code, 'pos'):
        parse_uds(tester, send_recv_uds(tester, bytes([0x11, 0x01])), 0x11)
        time.sleep(1.0)
        check(report, "app boots after rollback-blocked update",
              'pos' if in_app(tester) else None, None, 'pos')


#=======================================
# main
#=======================================
def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('interface', nargs='?', default='can0')
    args = ap.parse_args()

    if not os.path.exists(APP_BIN):
        print(f"app image not found: {APP_BIN} (run `make all`)")
        sys.exit(1)
    image = open(APP_BIN, 'rb').read()
    print(Color.info_msg(f"app image: {APP_BIN} ({len(image)}B, "
                         f"crc32={binascii.crc32(image) & 0xFFFFFFFF:08X})"))

    tester = UDSTester(args.interface)
    report = TestReport()
    tester.connect()
    try:
        if not in_app(tester) and not enter_bl(tester, report, wait_unlock_delay=False):
            # cold path: CRC-gated BL (or no app) -- fall through, gating test reports it
            print(Color.info_msg("app not responding; assuming BL is active"))
        test_enter_and_gate(tester, report)
        test_update_cycle(tester, report, image)
        test_brick_and_recovery(tester, report, image)
        test_anti_rollback(tester, report, image)
        tester.disconnect()
    except KeyboardInterrupt:
        print("\nUser interrupt")
        sys.exit(130)

    sys.exit(0 if report.print_summary() else 1)


if __name__ == '__main__':
    main()
