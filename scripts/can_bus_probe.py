"""Bounded PCAN receive/status probe. Does not send motor command frames."""
import argparse
import collections
import json
import time

import can


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=5.0)
    args = parser.parse_args()
    if not 0 < args.seconds <= 30:
        parser.error("seconds must be in (0, 30]")
    counts = collections.Counter()
    statuses = collections.Counter()
    error_frames = 0
    failures = collections.Counter()
    bus = can.Bus(interface="pcan", channel="PCAN_USBBUS1", bitrate=1000000)
    try:
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            statuses[int(bus.status())] += 1
            try:
                message = bus.recv(timeout=0.05)
            except can.CanError as error:
                failures[str(error)] += 1
                time.sleep(0.01)
                continue
            if message is None:
                continue
            if message.is_error_frame:
                error_frames += 1
            else:
                counts[f"0x{message.arbitration_id:03X}"] += 1
    finally:
        bus.shutdown()
    print(json.dumps({"seconds": args.seconds, "frames": dict(counts),
                      "error_frames": error_frames, "status_samples": dict(statuses),
                      "receive_errors": dict(failures)}, indent=2))


if __name__ == "__main__":
    main()
