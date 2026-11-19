"""Match NX 2606 `.mres` float slots against NX10 `.rplt` channel series."""

from __future__ import annotations

import struct
import sys

from .motion_results import parse_rplt

MRES_HEADER = 29920
MRES_RECORD = 1520
MRES_SLOTS = MRES_RECORD // 4


def load_mres(path: str):
    data = open(path, "rb").read()
    records = []
    offset = MRES_HEADER
    while offset + MRES_RECORD <= len(data):
        records.append(
            list(struct.unpack("<" + "f" * MRES_SLOTS, data[offset : offset + MRES_RECORD]))
        )
        offset += MRES_RECORD
    return records


def main() -> int:
    rplt, mres = sys.argv[1], sys.argv[2]
    results = parse_rplt(rplt)
    records = load_mres(mres)
    print("records", len(records), "slots", len(records[0]))
    samples = min(len(records), len(results.times), 12)
    slots = len(records[0])
    for channel in results.channels:
        if channel.name == "TIME":
            continue
        best = None
        for slot in range(1, slots):
            error = 0.0
            for index in range(1, samples):
                expected = channel.values[index]
                actual = records[index][slot]
                error = max(error, abs(actual - expected))
                if error > 1.0e-3:
                    break
            if best is None or error < best[1]:
                best = (slot, error)
        if best is not None and best[1] <= 1.0e-3:
            print(f"{channel.path}\t{best[0]}\t{best[1]:.3e}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
