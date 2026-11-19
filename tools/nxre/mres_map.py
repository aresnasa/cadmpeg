"""Map NX 2606 `.mres` float slots to NX10 `.rplt` channels."""

from __future__ import annotations

import struct
import sys

from .motion_results import parse_rplt

MRES_HEADER = 29920
MRES_RECORD = 1520
MRES_SLOTS = MRES_RECORD // 4


def load_records(path: str):
    data = open(path, "rb").read()
    records = []
    offset = MRES_HEADER
    while offset + MRES_RECORD <= len(data):
        records.append(
            list(
                struct.unpack(
                    "<" + "f" * MRES_SLOTS, data[offset : offset + MRES_RECORD]
                )
            )
        )
        offset += MRES_RECORD
    return records


def close(actual: float, expected: float) -> bool:
    return abs(actual - expected) <= max(1.0e-3, abs(expected) * 1.0e-4)


def main() -> int:
    rplt, mres = sys.argv[1], sys.argv[2]
    results = parse_rplt(rplt)
    records = load_records(mres)
    samples = min(len(records), len(results.times), 40)
    channels = [channel for channel in results.channels if channel.name != "TIME"]
    channels = [
        channel
        for channel in channels
        if max(abs(value) for value in channel.values[1:samples]) > 1.0e-9
    ]
    for slot in range(1, MRES_SLOTS):
        values = [record[slot] for record in records[1:samples]]
        if max(abs(value) for value in values) < 1.0e-9:
            continue
        matches = [
            channel.path
            for channel in channels
            if all(
                close(records[index][slot], channel.values[index])
                for index in range(1, samples)
            )
        ]
        if matches:
            print(slot, matches)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
