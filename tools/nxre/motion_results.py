"""Read NX Motion / RecurDyn time-history results.

RecurDyn (the NX Motion solver) writes several output files for a solved
motion:

* ``.rplt`` - plot channels (time history).  This is the file parsed here.
* ``.rad``  - animation data (model/geometry state), not plot channels.
* ``.rmd``  - the solver model (see :mod:`nxre.motion`).
* ``.msg``  - the solver log.
* ``.sim``  - the NX part container; it stores model entities, not results.

The ``.rplt`` layout was recovered from RecurDyn SDK 7.25.0 output:

* a small header with the unit list, model name and a recursive channel tree;
* 200 frame records of 85 little-endian float64 values.  Each record stores
  84 raw channel values followed by the record's time label;
* one trailing 84-value record holding the final state at the end time.

Derived channels (``*_TM``/``*_RM`` magnitudes) appear in the channel tree but
are not stored; they are recomputed from their vector components.

The time labels in the file are one output step ahead of the stored state
(record ``k`` has label ``(k+1) * dt`` but values at ``k * dt``).  This module
exposes corrected state times by default and records the applied shift.
"""

from __future__ import annotations

import csv
import json
import math
import re
import struct
from dataclasses import dataclass, field
from pathlib import Path
from statistics import median
from typing import Any

STRING_MARKER = b"\xff\xfe\xff"

UNIT_NAMES: dict[int, str] = {
    10: "s",
    20: "mm",
    30: "mm/s",
    40: "mm/s^2",
    50: "deg",
    70: "rad/s",
    80: "rad/s^2",
    90: "N",
    100: "N*mm",
}

DERIVED_CHANNELS: dict[str, tuple[str, str, str]] = {
    "Pos_TM": ("Pos_TX", "Pos_TY", "Pos_TZ"),
    "Vel_TM": ("Vel_TX", "Vel_TY", "Vel_TZ"),
    "Vel_RM": ("Vel_RX", "Vel_RY", "Vel_RZ"),
    "Acc_TM": ("Acc_TX", "Acc_TY", "Acc_TZ"),
    "Acc_RM": ("Acc_RX", "Acc_RY", "Acc_RZ"),
    "FM_Reaction_Force": (
        "FX_Reaction_Force",
        "FY_Reaction_Force",
        "FZ_Reaction_Force",
    ),
    "TM_Reaction_Force": (
        "TX_Reaction_Force",
        "TY_Reaction_Force",
        "TZ_Reaction_Force",
    ),
}


class MotionResultsError(ValueError):
    """Raised when a result file does not match the recovered layout."""


@dataclass
class ResultChannel:
    path: str
    name: str
    group: str
    owner: str
    kind: int
    unit_id: int
    unit: str
    derived: bool
    header_offset: int = 0
    values: list[float] = field(default_factory=list)

    def to_dict(self, include_values: bool = True) -> dict[str, Any]:
        data: dict[str, Any] = {
            "path": self.path,
            "name": self.name,
            "group": self.group,
            "owner": self.owner,
            "kind": self.kind,
            "unit_id": self.unit_id,
            "unit": self.unit,
            "derived": self.derived,
            "header_offset": self.header_offset,
        }
        if include_values:
            data["values"] = self.values
        return data


@dataclass
class MotionResults:
    path: str
    model_name: str
    units: dict[str, str]
    channels: list[ResultChannel]
    times: list[float]
    stored_times: list[float]
    time_step: float
    time_shift: float
    header_bytes: int
    data_offset: int
    frame_width: int
    frames_in_file: int
    final_state: bool
    final_offset: int | None

    def channel(self, path: str) -> ResultChannel:
        wanted = path if path.startswith("/") else "/" + path
        for channel in self.channels:
            if channel.path == wanted:
                return channel
        raise KeyError(path)

    def series(self, path: str) -> list[float]:
        return self.channel(path).values

    def table(self) -> dict[str, list[float]]:
        table: dict[str, list[float]] = {"time_s": list(self.times)}
        for channel in self.channels:
            table[channel.path.lstrip("/")] = list(channel.values)
        return table

    def to_dict(self, include_values: bool = True) -> dict[str, Any]:
        return {
            "summary": {
                "channel_count": len(self.channels),
                "derived_channel_count": sum(
                    1 for channel in self.channels if channel.derived
                ),
                "stored_values_per_frame": self.frame_width - 1,
                "sample_count": len(self.times),
                "body_count": len(
                    {channel.owner for channel in self.channels if channel.group == "Bodies"}
                ),
                "joint_count": len(
                    {channel.owner for channel in self.channels if channel.group == "Joints"}
                ),
                "coupler_count": len(
                    {channel.owner for channel in self.channels if channel.group == "Couplers"}
                ),
            },
            "source": {
                "path": self.path,
                "format": "RecurDyn RPLT",
                "model": self.model_name,
                "units": self.units,
                "header_bytes": self.header_bytes,
                "data_offset": self.data_offset,
                "frame_width_doubles": self.frame_width,
                "frames_in_file": self.frames_in_file,
                "final_state_record": self.final_state,
                "final_state_offset": self.final_offset,
            },
            "time": {
                "unit": "s",
                "count": len(self.times),
                "step": self.time_step,
                "shift_applied": self.time_shift,
                "stored_count": len(self.stored_times),
                "stored_range": [self.stored_times[0], self.stored_times[-1]]
                if self.stored_times
                else None,
                "stored_values": list(self.stored_times),
                "range": [self.times[0], self.times[-1]] if self.times else None,
                "values": list(self.times),
            },
            "channels": [
                channel.to_dict(include_values=include_values)
                for channel in self.channels
            ],
        }


def _read_name(data: bytes, offset: int) -> tuple[str, int]:
    if data[offset : offset + 3] != STRING_MARKER:
        raise MotionResultsError(
            f"expected string marker at 0x{offset:x}, got "
            f"{data[offset:offset + 4].hex()}"
        )
    length = data[offset + 3]
    start = offset + 4
    end = start + 2 * length
    if end > len(data):
        raise MotionResultsError(f"string at 0x{offset:x} runs past end of file")
    return data[start:end].decode("utf-16le"), end


def _channel_tree(data: bytes) -> tuple[list[ResultChannel], int, str, dict[str, str]]:
    offset = 0x14  # five 32-bit header words precede the unit strings
    units: dict[str, str] = {}
    unit_names = ["force", "mass", "length", "time", "angle"]
    for unit_name in unit_names:
        value, offset = _read_name(data, offset)
        units[unit_name] = value

    model_name, offset = _read_name(data, offset)
    root_count = struct.unpack_from("<i", data, offset)[0]
    offset += 4
    if root_count <= 0:
        raise MotionResultsError("result file has no channel groups")

    channels: list[ResultChannel] = []

    def walk(position: int, parent: str, count: int) -> int:
        for _ in range(count):
            leaf_offset = position
            name, position = _read_name(data, position)
            kind = struct.unpack_from("<i", data, position)[0]
            position += 4
            path = f"{parent}/{name}"
            if kind > 0:
                position = walk(position, path, kind)
            else:
                unit_id = struct.unpack_from("<i", data, position)[0]
                position += 4
                parts = path.strip("/").split("/")
                group = parts[0] if parts else ""
                owner = parts[1] if len(parts) > 2 else ""
                derived = kind < 0
                if derived and name not in DERIVED_CHANNELS:
                    raise MotionResultsError(
                        f"unknown derived channel {path} (kind {kind})"
                    )
                channels.append(
                    ResultChannel(
                        path=path,
                        name=name,
                        group=group,
                        owner=owner,
                        kind=kind,
                        unit_id=unit_id,
                        unit=UNIT_NAMES.get(unit_id, f"unit{unit_id}"),
                        derived=derived,
                        header_offset=leaf_offset,
                    )
                )
        return position

    offset = walk(offset, "", root_count)
    return channels, offset, model_name, units


def _raw_slots(channels: list[ResultChannel]) -> int:
    stored = [channel for channel in channels if not channel.derived]
    return len(stored)


def _derived_value(
    channel: ResultChannel, by_path: dict[str, ResultChannel], frame: int
) -> float:
    name = channel.name
    try:
        components = DERIVED_CHANNELS[name]
    except KeyError as error:  # pragma: no cover - guarded by the caller
        raise MotionResultsError(f"unsupported derived channel {name}") from error
    prefix = channel.path.rsplit("/", 1)[0]
    total = 0.0
    for component in components:
        sibling = f"{prefix}/{component}"
        try:
            total += by_path[sibling].values[frame] ** 2
        except KeyError as error:
            raise MotionResultsError(
                f"derived channel {channel.path} lacks component {sibling}"
            ) from error
    return math.sqrt(total)


def parse_rplt(path: str | Path, time_shift: float | None = None) -> MotionResults:
    """Parse a RecurDyn ``.rplt`` plot-result file.

    ``time_shift`` is added to the stored time labels.  ``None`` applies the
    recovered one-step correction (``-dt``) and uses the trailing final-state
    record; pass ``0.0`` to keep the labels exactly as written.
    """
    path = Path(path)
    data = path.read_bytes()
    if len(data) < 0x20:
        raise MotionResultsError(f"{path} is too small to be an RPLT file")
    channels, header_end, model_name, units = _channel_tree(data)

    if header_end + 8 > len(data):
        raise MotionResultsError("truncated RPLT header")
    separator = data[header_end : header_end + 8]
    if separator != b"\x00" * 8:
        raise MotionResultsError(
            f"unexpected data separator at 0x{header_end:x}: {separator.hex()}"
        )
    data_offset = header_end + 8

    width = _raw_slots(channels)
    if width < 2:
        raise MotionResultsError("RPLT channel tree has too few stored channels")
    payload = 8 * width
    body = data[data_offset:]
    frames, remainder = divmod(len(body), payload)
    if frames == 0:
        raise MotionResultsError("RPLT file has no complete frame records")

    stored_times: list[float] = []
    raw_frames: list[tuple[float, ...]] = []
    for index in range(frames):
        values = struct.unpack_from(f"<{width}d", body, index * payload)
        raw_frames.append(values[:-1])
        stored_times.append(values[-1])

    final_state = False
    final_offset: int | None = None
    final_values: tuple[float, ...] | None = None
    if remainder == 8 * (width - 1):
        final_state = True
        final_offset = data_offset + frames * payload
        final_values = struct.unpack_from(f"<{width - 1}d", body, frames * payload)
    elif remainder:
        raise MotionResultsError(
            f"{remainder} trailing bytes do not match a final-state record"
        )

    if len(stored_times) >= 2:
        measured_step = median(
            stored_times[index + 1] - stored_times[index]
            for index in range(len(stored_times) - 1)
        )
    else:
        measured_step = 0.0
    time_step = float(f"{measured_step:.12g}")
    auto_shift = time_shift is None
    if auto_shift:
        time_shift = -time_step

    # Stored channel order is the channel-tree order with derived channels and
    # TIME removed.  TIME is written last in each record.
    stored_channels = [
        channel for channel in channels if not channel.derived and channel.name != "TIME"
    ]
    if len(stored_channels) != width - 1:
        raise MotionResultsError(
            f"channel tree implies {width - 1} stored channels, "
            f"found {len(stored_channels)}"
        )
    by_path = {channel.path: channel for channel in channels}
    for slot, channel in enumerate(stored_channels):
        channel.values = [frame[slot] for frame in raw_frames]
        if final_values is not None:
            channel.values.append(final_values[slot])

    time_channel = next((channel for channel in channels if channel.name == "TIME"), None)
    if time_channel is None:
        raise MotionResultsError("RPLT channel tree has no TIME channel")
    if auto_shift:
        times = [index * time_step for index in range(frames)]
        if final_values is not None:
            times.append(frames * time_step)
    else:
        times = [value + time_shift for value in stored_times]
        if final_values is not None:
            times.append(stored_times[-1])
    time_channel.values = list(times)

    for channel in channels:
        if not channel.derived:
            continue
        count = len(raw_frames) + (1 if final_values is not None else 0)
        channel.values = [
            _derived_value(channel, by_path, frame) for frame in range(count)
        ]

    for channel in channels:
        expected = len(raw_frames) + (1 if final_values is not None else 0)
        if len(channel.values) != expected:
            raise MotionResultsError(
                f"channel {channel.path} has {len(channel.values)} values, "
                f"expected {expected}"
            )

    return MotionResults(
        path=str(path),
        model_name=model_name,
        units=units,
        channels=channels,
        times=times,
        stored_times=stored_times,
        time_step=time_step,
        time_shift=time_shift,
        header_bytes=header_end,
        data_offset=data_offset,
        frame_width=width,
        frames_in_file=frames,
        final_state=final_state,
        final_offset=final_offset,
    )


def _find_time_axis(
    data: bytes, step: float = 0.025, minimum: int = 10
) -> tuple[int, list[float]] | None:
    """Locate a monotonic time axis in a binary header without guessing."""
    limit = min(len(data) - 8 * minimum, 0x10000)
    for offset in range(0, limit, 4):
        try:
            previous = struct.unpack_from("<d", data, offset)[0]
            following = struct.unpack_from("<d", data, offset + 8)[0]
        except struct.error:
            break
        if abs(previous) > 1e-9 or abs((following - previous) - step) > 1e-9:
            continue
        values = [previous]
        position = offset + 8
        while position + 8 <= len(data):
            value = struct.unpack_from("<d", data, position)[0]
            if not math.isfinite(value) or value <= values[-1]:
                break
            if abs((value - values[-1]) - step) > 1e-9:
                break
            values.append(value)
            position += 8
        if len(values) >= minimum:
            return offset, values
    return None


def parse_rad(path: str | Path) -> dict[str, Any]:
    """Inspect a RecurDyn ``.rad`` animation file.

    ``.rad`` is written by ``OUTPUT/GRSAVE, RADSAVE=TRUE`` and contains
    animation/state data, not the plot channel table.  This function reports
    the recovered time axis and proves whether channel names exist; it does
    not pretend to decode the animation payload.
    """
    path = Path(path)
    data = path.read_bytes()
    found = _find_time_axis(data)
    token_names = [b"TIME", b"Pos_", b"Vel_", b"Acc_", b"Reaction_Force", b"J002", b"J003"]
    ascii_tokens = {
        token.decode(): [match.start() for match in re.finditer(token, data)]
        for token in token_names
    }
    ascii_tokens = {name: offsets for name, offsets in ascii_tokens.items() if offsets}
    utf16_runs = []
    channel_names: list[str] = []
    for match in re.finditer(rb"(?:[\x20-\x7e]\x00){4,}", data):
        text = match.group().decode("utf-16le")
        utf16_runs.append(match.start())
        if (
            text == "TIME"
            or re.match(r"^(Pos|Vel|Acc)_", text)
            or text.endswith(
                ("_Relative", "_Reaction_Force", "_Driving_Torque", "_Friction_Torque")
            )
        ):
            channel_names.append(text)
    return {
        "path": str(path),
        "size": len(data),
        "format": "RecurDyn RAD (animation/state data)",
        "time_axis": None
        if found is None
        else {
            "offset": found[0],
            "count": len(found[1]),
            "start": found[1][0],
            "end": found[1][-1],
            "step": found[1][1] - found[1][0] if len(found[1]) > 1 else None,
        },
        "channel_names": sorted(set(channel_names)),
        "ascii_channel_tokens": ascii_tokens,
        "utf16_string_runs": len(utf16_runs),
        "plot_channels_present": bool(channel_names),
        "note": (
            "RADSAVE animation data; plot channels are written to .rplt "
            "(OUTPUT/GRSAVE, RPLTSAVE=TRUE)"
        ),
    }


def inspect_sim(path: str | Path) -> dict[str, Any]:
    """Inventory an NX ``.sim`` OLE container.

    The ``.sim`` file is an NX part container (``UG_PART/UG_PART``) and does
    not contain solver result channels.  ``olefile`` is optional; when it is
    unavailable the function returns ``available: False``.
    """
    path = Path(path)
    try:
        import olefile
    except ImportError:
        return {
            "path": str(path),
            "available": False,
            "reason": "olefile is not installed",
        }
    data_prefix = path.read_bytes()[:8]
    if not data_prefix.startswith(b"\xd0\xcf\x11\xe0\xa1\xb1\x1a\xe1"):
        return {
            "path": str(path),
            "available": False,
            "reason": (
                "not an OLE2 structured storage file; NX 2606 writes this "
                "simulation as an SPLMSSTR container"
            ),
            "container": "splmsstr"
            if data_prefix.startswith(b"SPLMSSTR")
            else "unknown",
            "size": path.stat().st_size,
            "streams": [],
            "plot_channels_present": False,
            "result_tokens_found": {},
            "entity_name_strings": {},
            "has_ug_part_stream": False,
        }
    result: dict[str, Any] = {
        "path": str(path),
        "available": True,
        "size": path.stat().st_size,
        "streams": [],
        "plot_channels_present": False,
        "result_tokens_found": {},
    }
    channel_tokens = (
        b"TIME",
        b"Pos_",
        b"Vel_",
        b"Acc_",
        b"Reaction_Force",
        b"Driving_Torque",
        b"Friction_Torque",
    )
    with olefile.OleFileIO(str(path)) as ole:
        for entry in ole.listdir(streams=True, storages=True):
            name = "/".join(entry)
            try:
                size = ole.get_size(entry)
            except (OSError, TypeError):
                size = None
            result["streams"].append({"name": name, "size": size})
            if size is None:
                continue
            payload = ole.openstream(entry).read()
            for token in channel_tokens:
                offsets = []
                start = 0
                while True:
                    found = payload.find(token, start)
                    if found < 0:
                        break
                    offsets.append(found)
                    start = found + 1
                if offsets:
                    result["result_tokens_found"].setdefault(
                        token.decode(), {}
                    )[name] = offsets
        result["plot_channels_present"] = bool(result["result_tokens_found"])
        if ole.exists("UG_PART/UG_PART"):
            data = ole.openstream("UG_PART/UG_PART").read()
            tokens = {}
            for token in (b"J002", b"J003", b"J004"):
                offsets = []
                start = 0
                while True:
                    found = data.find(token, start)
                    if found < 0:
                        break
                    offsets.append(found)
                    start = found + 1
                if offsets:
                    tokens[token.decode()] = offsets
            result["entity_name_strings"] = tokens
            result["has_ug_part_stream"] = True
    return result


def validate_gear_ratio(
    results: MotionResults,
    driver: str = "J002",
    driven: str = "J003",
    driver_rate_deg_per_s: float = 180.0,
) -> dict[str, Any]:
    """Validate a two-gear coupler from the stored joint channels."""
    driver_pos = results.series(f"/Joints/{driver}/Pos1_Relative")
    driven_pos = results.series(f"/Joints/{driven}/Pos1_Relative")
    driver_vel = results.series(f"/Joints/{driver}/Vel1_Relative")
    driven_vel = results.series(f"/Joints/{driven}/Vel1_Relative")

    position_ratios = [
        driven / driver for driver, driven in zip(driver_pos, driven_pos) if abs(driver) > 1e-9
    ]
    velocity_ratios = [
        driven / driver for driver, driven in zip(driver_vel, driven_vel) if abs(driver) > 1e-9
    ]
    expected = [driver_rate_deg_per_s * value for value in results.times]
    rate_errors = [
        abs(actual - wanted) for actual, wanted in zip(driver_pos, expected)
    ]
    return {
        "driver_joint": driver,
        "driven_joint": driven,
        "sample_count": len(results.times),
        "position_units": results.channel(f"/Joints/{driver}/Pos1_Relative").unit,
        "velocity_units": results.channel(f"/Joints/{driver}/Vel1_Relative").unit,
        "position_ratio_median": median(position_ratios) if position_ratios else None,
        "velocity_ratio_median": median(velocity_ratios) if velocity_ratios else None,
        "position_ratio_max_error": max(
            (abs(value + 1.0) for value in position_ratios), default=None
        ),
        "velocity_ratio_max_error": max(
            (abs(value + 1.0) for value in velocity_ratios), default=None
        ),
        "driver_start_deg": driver_pos[0],
        "driver_end_deg": driver_pos[-1],
        "driven_start_deg": driven_pos[0],
        "driven_end_deg": driven_pos[-1],
        "driver_speed_rad_s": driver_vel[0],
        "driven_speed_rad_s": driven_vel[0],
        "driver_angle_max_error_deg": max(rate_errors, default=None),
        "passed": (
            bool(position_ratios)
            and bool(velocity_ratios)
            and abs(median(position_ratios) + 1.0) < 1e-9
            and abs(median(velocity_ratios) + 1.0) < 1e-9
            and max(rate_errors, default=0.0) < 1e-6
        ),
    }


def write_json(
    results: MotionResults,
    path: str | Path,
    validation: dict[str, Any] | None = None,
    rad: dict[str, Any] | None = None,
    sim: dict[str, Any] | None = None,
) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = results.to_dict()
    if validation is not None:
        payload["validation"] = validation
    if rad is not None:
        payload["rad"] = rad
    if sim is not None:
        payload["sim"] = sim
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")


def write_csv(results: MotionResults, path: str | Path, precision: int = 12) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    channels = [channel for channel in results.channels if channel.name != "TIME"]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["time_s"] + [channel.path.lstrip("/") for channel in channels])
        for index, time in enumerate(results.times):
            writer.writerow(
                [format(time, f".{precision}g")]
                + [
                    format(channel.values[index], f".{precision}g")
                    for channel in channels
                ]
            )
