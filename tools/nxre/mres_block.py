"""Inspect one NX 2606 `.mres` record block against NX10 channels."""

from __future__ import annotations

import struct
import sys

from .motion_results import parse_rplt

MRES_HEADER = 29920
MRES_RECORD = 1520
MRES_SLOTS = MRES_RECORD // 4


def main() -> int:
    rplt, mres, start, end = (
        sys.argv[1],
        sys.argv[2],
        int(sys.argv[3]),
        int(sys.argv[4]),
    )
    results = parse_rplt(rplt)
    data = open(mres, "rb").read()
    record = list(
        struct.unpack(
            "<" + "f" * MRES_SLOTS,
            data[MRES_HEADER + MRES_RECORD : MRES_HEADER + 2 * MRES_RECORD],
        )
    )
    channels = [channel for channel in results.channels if channel.name != "TIME"]
    for slot in range(start, end):
        value = record[slot]
        matches = [
            channel.path
            for channel in channels
            if abs(channel.values[1] - value)
            <= max(1.0e-3, abs(channel.values[1]) * 2.0e-4)
            and abs(channel.values[1]) > 1.0e-9
        ]
        print(f"{slot:3d} {value: .9g} {matches}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
