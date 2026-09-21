#!/usr/bin/env python3
"""
can_frames.py — MKS SERVO42D/57D CAN protocol (byte-exact port of the
frame builders from the original can_gui.py testing script).

Pure functions, no CAN/ROS dependencies, unit-testable:

    python3 can_frames.py   # runs byte-exact assertions

Protocol summary (from can_gui.py):
    crc:   crc8(can_id, data) = (can_id + sum(data)) & 0xFF
    frame: [can_id, *data, crc8]

Motion commands:
    F6  speed mode            [F6, dir|speed_hi, speed_lo, acc]
    FD  relative position     [FD, dir|speed_hi, speed_lo, acc, p_hi, p_mid, p_lo]
    FE  absolute position     [FE, speed_hi, speed_lo, acc, p_hi, p_mid, p_lo]
    F7  emergency stop        [F7]
    F3  enable/disable        [F3, 0/1]
Queries:
    F1  motor status          [F1]
    30/31 encoder / encoder+  [30] / [31]
    32  speed                 [32]
    33  pulses received       [33]
    39  shaft error           [39]
    3A  enable status         [3A]
    3B  zero status           [3B]
Homing:
    91  go home               [91]
    90  home parameters       [90, trig, dir, speed_hi, speed_lo, end_limit]
    92  set zero              [92]
    80  calibrate             [80, 00]
"""
from typing import List

MAX_SPEED = 3000      # RPM (per can_gui.py clamps)
MAX_ACC = 255
MAX_REL_PULSES = 0xFFFFFF
MAX_ABS_PULSES = 8388607


def crc8(can_id: int, data_bytes: List[int]) -> int:
    return (can_id + sum(data_bytes)) & 0xFF


def _frame(can_id: int, data: List[int]) -> List[int]:
    return [can_id] + data + [crc8(can_id, data)]


# ── Motion ─────────────────────────────────────────────────────────────────

def build_speed_cmd(can_id: int, direction: int, speed: int, acc: int) -> List[int]:
    speed = max(0, min(MAX_SPEED, speed))
    acc = max(0, min(MAX_ACC, acc))
    db = 0x80 if direction == 1 else 0x00
    b2 = db | ((speed >> 8) & 0x0F)
    b3 = speed & 0xFF
    return _frame(can_id, [0xF6, b2, b3, acc])


def build_position_rel_cmd(can_id: int, direction: int, speed: int, acc: int,
                           pulses: int) -> List[int]:
    speed = max(0, min(MAX_SPEED, speed))
    acc = max(0, min(MAX_ACC, acc))
    pulses = max(0, min(MAX_REL_PULSES, pulses))
    db = 0x80 if direction == 1 else 0x00
    b2 = db | ((speed >> 8) & 0x0F)
    b3 = speed & 0xFF
    p2, p1, p0 = (pulses >> 16) & 0xFF, (pulses >> 8) & 0xFF, pulses & 0xFF
    return _frame(can_id, [0xFD, b2, b3, acc, p2, p1, p0])


def build_abs_position_cmd(can_id: int, speed: int, acc: int,
                           abs_pulses: int) -> List[int]:
    speed = max(0, min(MAX_SPEED, speed))
    acc = max(0, min(MAX_ACC, acc))
    abs_pulses = max(-MAX_ABS_PULSES, min(MAX_ABS_PULSES, abs_pulses))
    packed = abs_pulses & 0xFFFFFF
    p2, p1, p0 = (packed >> 16) & 0xFF, (packed >> 8) & 0xFF, packed & 0xFF
    s1, s0 = (speed >> 8) & 0xFF, speed & 0xFF
    return _frame(can_id, [0xFE, s1, s0, acc, p2, p1, p0])


def build_stop_cmd(can_id: int, acc: int = 2) -> List[int]:
    return _frame(can_id, [0xF6, 0x00, 0x00, acc])


def build_emerg_stop_cmd(can_id: int) -> List[int]:
    return _frame(can_id, [0xF7])


def build_enable_cmd(can_id: int, enable: bool) -> List[int]:
    return _frame(can_id, [0xF3, 0x01 if enable else 0x00])


# ── Queries ────────────────────────────────────────────────────────────────

def build_query_status_cmd(can_id: int) -> List[int]:
    return _frame(can_id, [0xF1])


def build_read_encoder_cmd(can_id: int) -> List[int]:
    return _frame(can_id, [0x30])


def build_read_encoder_add_cmd(can_id: int) -> List[int]:
    return _frame(can_id, [0x31])


def build_read_speed_cmd(can_id: int) -> List[int]:
    return _frame(can_id, [0x32])


def build_read_pulses_cmd(can_id: int) -> List[int]:
    return _frame(can_id, [0x33])


def build_read_error_cmd(can_id: int) -> List[int]:
    return _frame(can_id, [0x39])


def build_read_en_status_cmd(can_id: int) -> List[int]:
    return _frame(can_id, [0x3A])


def build_read_zero_status_cmd(can_id: int) -> List[int]:
    return _frame(can_id, [0x3B])


# ── Homing ─────────────────────────────────────────────────────────────────

def build_go_home_cmd(can_id: int) -> List[int]:
    return _frame(can_id, [0x91])


def build_set_home_params_cmd(can_id: int, trig: int, direction: int,
                              speed_rpm: int, end_limit: int) -> List[int]:
    speed_rpm = max(0, min(MAX_SPEED, speed_rpm))
    s1, s0 = (speed_rpm >> 8) & 0xFF, speed_rpm & 0xFF
    return _frame(can_id, [0x90, trig & 0x01, direction & 0x01,
                           s1, s0, end_limit & 0x01])


def build_set_zero_cmd(can_id: int) -> List[int]:
    return _frame(can_id, [0x92])


def build_calibrate_cmd(can_id: int) -> List[int]:
    return _frame(can_id, [0x80, 0x00])


# ── Set-variables (safety-relevant subset) ─────────────────────────────────

def build_set_current_cmd(can_id: int, ma: int) -> List[int]:
    ma = max(0, min(5200, ma))
    return _frame(can_id, [0x83, (ma >> 8) & 0xFF, ma & 0xFF])


def build_set_protect_cmd(can_id: int, enable: bool) -> List[int]:
    return _frame(can_id, [0x88, 0x01 if enable else 0x00])


def build_set_dir_cmd(can_id: int, direction: int) -> List[int]:
    return _frame(can_id, [0x86, direction & 0xFF])


def build_set_subdivision_cmd(can_id: int, micstep: int) -> List[int]:
    return _frame(can_id, [0x84, micstep & 0xFF])


def build_set_can_bitrate_cmd(can_id: int, rate_idx: int) -> List[int]:
    return _frame(can_id, [0x8A, rate_idx & 0xFF])


def parse_raw_hex(hex_str: str) -> List[int]:
    try:
        return [int(p, 16) for p in hex_str.strip().split()]
    except Exception:
        return None


# ── Response parsing (from can_gui.py parse_query_response) ───────────────

import struct


def parse_query_response(parse_key: str, data: bytes):
    """Extract a numeric value from a CAN response frame's data bytes."""
    try:
        if parse_key == "status_f1":
            return float(data[1]) if len(data) >= 2 else None
        elif parse_key == "encoder_add":
            if len(data) < 7:
                return None
            return float(int.from_bytes(data[1:7], "big", signed=True))
        elif parse_key == "speed":
            if len(data) < 3:
                return None
            return float(struct.unpack(">h", bytes(data[1:3]))[0])
        elif parse_key == "pulses":
            if len(data) < 5:
                return None
            return float(struct.unpack(">i", bytes(data[1:5]))[0])
        elif parse_key == "error":
            if len(data) < 5:
                return None
            return float(struct.unpack(">i", bytes(data[1:5]))[0])
        elif parse_key == "zero_status":
            return float(data[1]) if len(data) >= 2 else None
    except Exception:
        return None


STATUS_LABELS = {0: "Query Fail", 1: "Stopped", 2: "Speeding Up",
                 3: "Slowing Down", 4: "Full Speed", 5: "Homing", 6: "Calibrating"}


def self_test() -> bool:
    """Byte-exact assertions (values hand-computed from can_gui.py logic)."""
    cases = [
        # hand-verified against can_gui.py logic: dir=1 sets the 0x80 bit in byte 2
        ("speed cw 500rpm acc2 id1", build_speed_cmd(1, 1, 500, 2),
         [1, 0xF6, 0x81, 0xF4, 0x02, (1 + 0xF6 + 0x81 + 0xF4 + 0x02) & 0xFF]),
        ("speed ccw 3000rpm id4", build_speed_cmd(4, 0, 3000, 5),
         [4, 0xF6, 0x0B, 0xB8, 0x05,
          (4 + 0xF6 + 0x0B + 0xB8 + 0x05) & 0xFF]),
        ("abs 100000 id2 spd100 acc2", build_abs_position_cmd(2, 100, 2, 100000),
         [2, 0xFE, 0x00, 0x64, 0x02, 0x01, 0x86, 0xA0, 0x8D]),
        ("abs negative -5000 id1", build_abs_position_cmd(1, 50, 1, -5000),
         [1, 0xFE, 0x00, 0x32, 0x01, 0xFF, 0xEC, 0x78, 0x95]),
        ("rel 12345 dir1 id1 spd1000 acc3",
         build_position_rel_cmd(1, 1, 1000, 3, 12345),
         [1, 0xFD, 0x83, 0xE8, 0x03, 0x00, 0x30, 0x39,
          (1 + 0xFD + 0x83 + 0xE8 + 0x03 + 0x00 + 0x30 + 0x39) & 0xFF]),
        ("estop id3", build_emerg_stop_cmd(3), [3, 0xF7, (3 + 0xF7) & 0xFF]),
        ("enable id2", build_enable_cmd(2, True), [2, 0xF3, 0x01, (2 + 0xF3 + 1) & 0xFF]),
        ("disable id2", build_enable_cmd(2, False), [2, 0xF3, 0x00, (2 + 0xF3) & 0xFF]),
        ("encoder_add id3", build_read_encoder_add_cmd(3), [3, 0x31, 0x34]),
        ("go home id5", build_go_home_cmd(5), [5, 0x91, (5 + 0x91) & 0xFF]),
        ("set zero id5", build_set_zero_cmd(5), [5, 0x92, (5 + 0x92) & 0xFF]),
        ("home params", build_set_home_params_cmd(1, 0, 1, 250, 1),
         [1, 0x90, 0x00, 0x01, 0x00, 0xFA, 0x01,
          (1 + 0x90 + 0x00 + 0x01 + 0x00 + 0xFA + 0x01) & 0xFF]),
        ("stall protect on", build_set_protect_cmd(7, True),
         [7, 0x88, 0x01, (7 + 0x88 + 1) & 0xFF]),
    ]
    ok = True
    for name, got, want in cases:
        match = got == want
        ok &= match
        print(f"  {'PASS' if match else 'FAIL'}  {name:32s} {' '.join(f'{b:02X}' for b in got)}"
              + ('' if match else f'  expected {' '.join(f"{b:02X}" for b in want)}'))

    # response parsing
    enc = b"\x31" + (123456).to_bytes(6, "big", signed=True)
    assert parse_query_response("encoder_add", enc) == 123456
    assert parse_query_response("status_f1", b"\xf1\x04") == 4.0
    print("  PASS  response parsing (encoder_add / status_f1)")
    return ok


if __name__ == "__main__":
    print("CAN protocol self-test:")
    import sys
    sys.exit(0 if self_test() else 1)
