"""Parse the NX 2606 RecurDyn `.minp` motion model.

NX 2606 writes the Motion solver input as a fixed-width RecurDyn text model
(`HIEROBJ` records, BODY/JOINT/GEAR/DRIVER sections, NAMEMAP aliases) instead
of the NX10 `.rmd` file.  The parsed shape follows `motion.MotionModel` so the
same Motion report can consume either generation.
"""

from __future__ import annotations

import re
from pathlib import Path

from .motion import Body, Driver, GearCoupler, Joint, Marker, MotionModel

FLOAT = re.compile(r"[-+]?\d+(?:\.\d*)?(?:[eE][-+]?\d+)?")


def _floats(line: str) -> list[float]:
    return [float(token) for token in FLOAT.findall(line)]


def _namemaps(lines: list[str]) -> dict[str, str]:
    mapping: dict[str, str] = {}
    for index, line in enumerate(lines):
        if not line.startswith("NAMEMAP"):
            continue
        parts = line.split()
        if len(parts) < 2:
            continue
        internal = parts[1]
        if index + 1 >= len(lines):
            continue
        display = lines[index + 1].strip()
        if display and not display.startswith("NAMEMAP"):
            mapping[internal] = display
    return mapping


def _entity_name(token: str) -> str:
    """Strip the RecurDyn element prefix from an entity reference."""
    cleaned = token.lstrip("0123456789")
    if cleaned.startswith(
        ("EBRACKET_", "EREVOLUTE_", "EGEAR_", "GDRIVER_")
    ):
        cleaned = cleaned[1:]
    return cleaned


def parse_minp(path: str | Path) -> MotionModel:
    """Parse an NX 2606 `.minp` into a MotionModel."""
    path = Path(path)
    text = path.read_text(encoding="utf-8", errors="replace")
    lines = text.splitlines()
    mapping = _namemaps(lines)

    bodies: list[Body] = []
    joints: list[Joint] = []
    drivers: list[Driver] = []
    gears: list[GearCoupler] = []
    markers: list[Marker] = []
    solver: dict[str, str] = {}
    gravity = [0.0, 0.0, 0.0]

    for index, line in enumerate(lines):
        stripped = line.strip()
        parts = stripped.split()
        if not parts:
            continue

        if parts[0] == "BODY" and len(parts) >= 2:
            internal = parts[1].lstrip("E")
            mass = None
            inertia: list[float] = []
            for follow in lines[index + 1 : index + 14]:
                values = _floats(follow)
                if len(values) == 4 and 0.0 < values[0] < 1.0e4:
                    mass = values[0]
                    inertia = values[1:4]
                    break
            bodies.append(
                Body(
                    id=len(bodies) + 1,
                    name=mapping.get(internal, internal),
                    mass=mass,
                    inertia=inertia,
                )
            )

        elif parts[0] in ("BRACKET", "REVOLUTE") and len(parts) >= 4:
            internal = _entity_name(parts[1])
            joints.append(
                Joint(
                    id=len(joints) + 1,
                    name=mapping.get(internal, internal),
                    i_marker=None,
                    j_marker=None,
                    kind="Fixed" if parts[0] == "BRACKET" else "Revolute",
                )
            )

        elif parts[0] == "GEAR" and len(parts) >= 4:
            internal = _entity_name(parts[1])
            joint_refs = [mapping.get(parts[2], parts[2]), mapping.get(parts[3], parts[3])]
            ratio: list[float] = []
            for follow in lines[index + 1 : index + 3]:
                values = _floats(follow)
                if len(values) >= 2:
                    ratio = values[:2]
                    break
            gears.append(
                GearCoupler(
                    id=len(gears) + 1,
                    name=mapping.get(internal, internal),
                    joints=joint_refs,
                    cv_marker=None,
                )
            )

        elif parts[0] == "DRIVER" and len(parts) >= 2:
            internal = _entity_name(parts[1])
            joint_name = None
            coefficients: list[float] = []
            for follow in lines[index + 1 : index + 12]:
                follow_parts = follow.split()
                if "POLYNOMIAL" in follow and follow_parts:
                    joint_name = mapping.get(
                        follow_parts[-1], follow_parts[-1]
                    )
                    continue
                if follow_parts and follow_parts[0] == "NONE" and "ACTIV" in follow:
                    break
                values = _floats(follow)
                if len(values) == 4:
                    # The last four-value line before ACTIV is the
                    # polynomial coefficient row; earlier four-value lines
                    # are marker/matrix rows.
                    coefficients = values[:4]
            function = ""
            if coefficients:
                function = (
                    f"0.5*({coefficients[0]:g})*TIME**2"
                    f"+({coefficients[1]:g})*TIME+({coefficients[2]:g})"
                )
            joint_id = next(
                (joint.id for joint in joints if joint.name == joint_name),
                None,
            )
            drivers.append(
                Driver(
                    id=len(drivers) + 1,
                    name=mapping.get(internal, internal),
                    joint=joint_id,
                    function=function,
                    rotation=True,
                    displacement=False,
                )
            )

        if "SDK_BUILD_INFO" in line:
            solver["sdk_build"] = stripped.split("=", 1)[-1].strip()
        if line.startswith("SET_IASIZE"):
            solver["ia_size"] = stripped.split("=", 1)[-1].strip().rstrip(";")
        if line.startswith("SET_DASIZE"):
            solver["da_size"] = stripped.split("=", 1)[-1].strip().rstrip(";")
        if line.startswith("BDFEPS"):
            solver["bdf_eps"] = stripped.split("=", 1)[-1].strip()
        if "9.80665" in line:
            values = _floats(line)
            if len(values) >= 4:
                gravity = [0.0, 0.0, -values[0]]

    bodies = [body for body in bodies if not body.name.startswith("the_ground")]
    for index, body in enumerate(bodies):
        body.id = index + 1

    return MotionModel(
        path=str(path),
        units={"length": "mm", "mass": "kg", "time": "s", "angle": "rad"},
        gravity=gravity,
        bodies=bodies,
        markers=markers,
        joints=joints,
        drivers=drivers,
        gears=gears,
        solver=solver,
        raw_sections={"hierobj": text.count("HIEROBJ"), "namemap": text.count("NAMEMAP")},
    )


def parse_minf(path: str | Path) -> dict:
    """Read the NX 2606 RecurDyn `.minf` solver log as text metadata."""
    path = Path(path)
    text = path.read_text(encoding="utf-8", errors="replace")
    lines = [line.rstrip() for line in text.splitlines() if line.strip()]
    result = {
        "path": str(path),
        "format": "RecurDyn MINF (NX 2606)",
        "header": lines[:20],
        "line_count": len(lines),
    }
    for line in lines:
        if line.startswith("Solver:"):
            result["solver_version"] = line.split(":", 1)[1].strip()
        elif line.startswith("Interface:"):
            result["interface"] = line.split(":", 1)[1].strip()
        elif line.startswith("License Server is"):
            result["license"] = line.split("is", 1)[-1].strip()
        elif line.startswith("Hostname:"):
            result["hostname"] = line.split(":", 1)[1].strip()
    return result


MRES_HEADER = 29920
MRES_RECORD = 1520
MRES_SLOTS = MRES_RECORD // 4
MRES_CHANNELS = (
    ("/Bodies/L001/Pos_TX", "mm", 20),
    ("/Bodies/L001/Pos_TY", "mm", 21),
    ("/Bodies/L001/Pos_TZ", "mm", 22),
    ("/Bodies/L002/Pos_TX", "mm", 60),
    ("/Bodies/L002/Pos_TY", "mm", 61),
    ("/Bodies/L002/Pos_TZ", "mm", 62),
    ("/Bodies/L003/Pos_TX", "mm", 100),
    ("/Bodies/L003/Pos_TY", "mm", 101),
    ("/Bodies/L003/Pos_TZ", "mm", 102),
    ("/Joints/J002/Vel1_Relative", "rad/s", 244),
    ("/Joints/J003/Vel1_Relative", "rad/s", 241),
    ("/Joints/J002/FX_Reaction_Force", "N", 182),
    ("/Joints/J002/FY_Reaction_Force", "N", 171),
    ("/Joints/J002/TX_Reaction_Force", "N*mm", 185),
    ("/Joints/J002/TY_Reaction_Force", "N*mm", 190),
    ("/Joints/J002/FM_Reaction_Force", "N", 188),
    ("/Joints/J002/Driving_Torque", "N*mm", 369),
)


def parse_mres(path: str | Path) -> dict:
    """Decode the verified channel subset of an NX 2606 `.mres` file.

    The 2606 result record is 1520 bytes of little-endian float32 per sample
    (201 samples for this scenario).  The slots below were matched against the
    NX10 RPLT channel series over all samples; the remaining slots still need
    a channel-table decode and are reported as pending.
    """
    import struct

    path = Path(path)
    data = path.read_bytes()
    records = []
    offset = MRES_HEADER
    while offset + MRES_RECORD <= len(data):
        records.append(
            struct.unpack(
                "<" + "f" * MRES_SLOTS, data[offset : offset + MRES_RECORD]
            )
        )
        offset += MRES_RECORD
    if not records:
        raise ValueError("no MRES records")
    times = [record[0] for record in records]
    channels = []
    for channel_path, unit, slot in MRES_CHANNELS:
        channels.append(
            {
                "path": channel_path,
                "unit": unit,
                "values": [record[slot] for record in records],
            }
        )
    by_path = {channel["path"]: channel["values"] for channel in channels}
    driver = by_path["/Joints/J002/Vel1_Relative"]
    driven = by_path["/Joints/J003/Vel1_Relative"]
    ratios = [
        driven_value / driver_value
        for driver_value, driven_value in zip(driver, driven)
        if abs(driver_value) > 1.0e-9
    ]
    ratio = sorted(ratios)[len(ratios) // 2] if ratios else None
    return {
        "path": str(path),
        "format": "RecurDyn MRES (NX 2606)",
        "coverage": "partial",
        "times": times,
        "time_step": (times[1] - times[0]) if len(times) > 1 else None,
        "channels": channels,
        "summary": {
            "channel_count": len(channels),
            "sample_count": len(times),
            "gear_ratio": ratio,
            "verified_slots": len(MRES_CHANNELS),
            "slot_count": MRES_SLOTS,
        },
        "note": (
            "Verified channel subset; the remaining MRES slots require the "
            "binary channel-table decode."
        ),
    }


def write_mres_csv(results: dict, path: str | Path) -> None:
    import csv

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    channels = results["channels"]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["time_s"] + [channel["path"].lstrip("/") for channel in channels])
        for index, time in enumerate(results["times"]):
            writer.writerow(
                [format(time, ".12g")]
                + [format(channel["values"][index], ".12g") for channel in channels]
            )


def parse_mres_header(path: str | Path) -> dict:
    """Read the header of an NX 2606 RecurDyn `.mres` result file.

    The 2606 result container is binary and does not carry the RPLT channel
    names or text headers.  The leading record does carry the source `.minp`
    path, timestamp, solver mode and counts, which is enough to tie the result
    file to a decoded model while channel decoding remains pending.
    """
    path = Path(path)
    data = path.read_bytes()
    header = data[:256].decode("ascii", errors="replace")
    model = re.search(r"([A-Za-z]:\\[^\s]+?\.minp)", header)
    timestamp = re.search(r"(\d{2}/\d{2}/\d{2}\s+\d{2}:\d{2}:\d{2})", header)
    return {
        "path": str(path),
        "size": len(data),
        "format": "RecurDyn MRES (NX 2606)",
        "input_model": model.group(1) if model else None,
        "timestamp": timestamp.group(1) if timestamp else None,
        "solver_mode": "DYNAMIC" if "DYNAMIC" in header else None,
        "channel_decoding": "pending",
    }
