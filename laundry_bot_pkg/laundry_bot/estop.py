#!/usr/bin/env python3
"""
estop.py — standalone emergency stop. NO ROS dependency, no startup delay.

Sends 0xF7 (emerg stop) + 0xF3 (disable) to all six MKS drivers over CAN.
Run it directly, bind it to a hotkey, or wire a hardware button to trigger:

    python3 estop.py                 # one-shot, all axes, default can0@500k
    python3 estop.py --interface candle --channel 0
    python3 estop.py --ids 1 2 3 4 5 6

Also usable as the command behind a hardware e-stop button (e.g. an
Arduino/GPIO expander that runs a fixed command on press).
"""
import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import can_frames as cf  # noqa: E402  (sibling module; standalone, no ROS needed)


def main():
    ap = argparse.ArgumentParser(description="Emergency stop for the laundry bot arm")
    ap.add_argument('--interface', default='socketcan')
    ap.add_argument('--channel', default='can0')
    ap.add_argument('--bitrate', type=int, default=500000)
    ap.add_argument('--ids', type=int, nargs='+', default=[1, 2, 3, 4, 5, 6])
    args = ap.parse_args()

    try:
        import can
        bus = can.Bus(iface=args.interface, channel=args.channel, bitrate=args.bitrate)
    except Exception as e:
        print(f"FATAL: cannot open CAN ({args.interface}:{args.channel}): {e}",
              file=sys.stderr)
        sys.exit(1)

    for cid in args.ids:
        try:
            bus.send(can.Message(arbitration_id=cid,
                                 data=cf.build_emerg_stop_cmd(cid)[1:-1]))
            bus.send(can.Message(arbitration_id=cid,
                                 data=cf.build_enable_cmd(cid, False)[1:-1]))
            print(f"  axis id={cid}: E-STOP + DISABLE sent")
        except Exception as e:
            print(f"  axis id={cid}: FAILED {e}", file=sys.stderr)
    bus.destroy()
    print("E-STOP complete.")


if __name__ == '__main__':
    main()
