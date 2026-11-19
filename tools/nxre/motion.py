"""Parse NX Motion (RecurDyn) model and message files.

NX Motion stores its solver model as a RecurDyn `.rmd` text file and the
solver log as `.msg`. This module reads the rigid-body, marker, joint, driver
and gear-coupler declarations without running RecurDyn.
"""

from __future__ import annotations

import json
import re
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any


@dataclass
class Marker:
    id: int
    name: str
    part: int | None
    qp: list[float]
    reuler: list[float]


@dataclass
class Body:
    id: int
    name: str
    mass: float | None = None
    center_of_mass: int | None = None
    inertia: list[float] = field(default_factory=list)
    qg: list[float] = field(default_factory=list)
    reuler: list[float] = field(default_factory=list)
    markers: list[int] = field(default_factory=list)


@dataclass
class Joint:
    id: int
    name: str
    i_marker: int | None
    j_marker: int | None
    kind: str


@dataclass
class Driver:
    id: int
    name: str
    joint: int | None
    function: str
    rotation: bool
    displacement: bool


@dataclass
class GearCoupler:
    id: int
    name: str
    joints: list[int]
    cv_marker: int | None


@dataclass
class MotionModel:
    path: str
    units: dict[str, str]
    gravity: list[float]
    bodies: list[Body]
    markers: list[Marker]
    joints: list[Joint]
    drivers: list[Driver]
    gears: list[GearCoupler]
    solver: dict[str, str]
    raw_sections: dict[str, int] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def _numbers(text: str) -> list[float]:
    return [float(value) for value in re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", text)]


def _values(block: str, name: str) -> list[float]:
    match = re.search(
        r",\s*" + re.escape(name) + r"\s*=\s*(.*?)(?=,\s*[A-Z_]{2,}\s*=|\n\s*!|\Z)",
        block,
        re.DOTALL,
    )
    return _numbers(match.group(1)) if match else []


def _blocks(text: str, keyword: str) -> list[str]:
    pattern = re.compile(r"^" + keyword + r"\s*/", re.MULTILINE)
    starts = [match.start() for match in pattern.finditer(text)]
    blocks = []
    for index, start in enumerate(starts):
        end = starts[index + 1] if index + 1 < len(starts) else len(text)
        blocks.append(text[start:end])
    return blocks


def _field(block: str, name: str) -> str | None:
    match = re.search(r",\s*" + re.escape(name) + r"\s*=\s*([^,\n]+)", block)
    return match.group(1).strip() if match else None


def _name(block: str) -> str:
    value = _field(block, "NAME")
    if value is None:
        return ""
    return value.strip().strip("'")


def _quoted_function(block: str) -> str:
    match = re.search(r"FUNCTION\s*=\s*(.*?)(?:\n|$)", block, re.DOTALL)
    if not match:
        return ""
    return match.group(1).strip().rstrip("\\").strip()


def parse_rmd(path: str | Path) -> MotionModel:
    path = Path(path)
    text = path.read_text(encoding="utf-8-sig", errors="replace")
    units: dict[str, str] = {}
    for block in _blocks(text, "UNITS"):
        for key, value in re.findall(r",\s*([A-Z_]+)\s*=\s*([^,\n]+)", block):
            units[key.strip()] = value.strip().strip("'")
    gravity = _numbers(_field(text, "KGRAV") or "")

    markers = []
    for block in _blocks(text, "MARKER"):
        identifier = _numbers(block.split("\n", 1)[0])[0]
        markers.append(
            Marker(
                id=int(identifier),
                name=_name(block),
                part=int(_numbers(_field(block, "PART") or "")[0])
                if _field(block, "PART")
                else None,
                qp=_values(block, "QP"),
                reuler=_values(block, "REULER"),
            )
        )
    bodies = []
    for block in _blocks(text, "PART"):
        header = _numbers(block.split("\n", 1)[0])
        identifier = int(header[0]) if header else 0
        kind = "GROUND" in block.split("\n", 1)[0].upper()
        if kind:
            continue
        mass_text = _field(block, "MASS")
        cm_text = _field(block, "CM")
        bodies.append(
            Body(
                id=identifier,
                name=_name(block),
                mass=float(mass_text) if mass_text else None,
                center_of_mass=int(_numbers(cm_text)[0]) if cm_text else None,
                inertia=_values(block, "IP"),
                qg=_values(block, "QG"),
                reuler=_values(block, "REULER"),
            )
        )
    for body in bodies:
        body.markers = [
            marker.id for marker in markers if marker.part == body.id
        ]

    joints = []
    for block in _blocks(text, "JOINT"):
        header = _numbers(block.split("\n", 1)[0])
        identifier = int(header[0]) if header else 0
        kind = "unknown"
        for candidate in ("Revolute", "Fixed", "Translational", "Cylindrical", "Spherical"):
            if re.search(r"\b" + candidate + r"\b", block):
                kind = candidate
                break
        i_text = _field(block, "I")
        j_text = _field(block, "J")
        joints.append(
            Joint(
                id=identifier,
                name=_name(block),
                i_marker=int(_numbers(i_text)[0]) if i_text else None,
                j_marker=int(_numbers(j_text)[0]) if j_text else None,
                kind=kind,
            )
        )

    drivers = []
    for block in _blocks(text, "MOTION"):
        header = _numbers(block.split("\n", 1)[0])
        identifier = int(header[0]) if header else 0
        joint_text = _field(block, "JOINT")
        drivers.append(
            Driver(
                id=identifier,
                name=_name(block),
                joint=int(_numbers(joint_text)[0]) if joint_text else None,
                function=_quoted_function(block),
                rotation="ROTATION" in block,
                displacement="DISPLACEMENT" in block,
            )
        )

    gears = []
    for block in _blocks(text, "GEAR"):
        header = _numbers(block.split("\n", 1)[0].replace("GEAR/", ""))
        identifier = int(header[0]) if header else 0
        joints_text = _values(block, "JOINTS")
        cv_text = _values(block, "CV")
        gears.append(
            GearCoupler(
                id=identifier,
                name=_name(block),
                joints=[int(value) for value in joints_text],
                cv_marker=int(cv_text[0]) if cv_text else None,
            )
        )

    solver = {}
    info = _field(text, "MODEL_NAME")
    if info:
        solver["model_name"] = info.strip().strip("'")
    current = _field(text, "CURRENT_SYSTEM_NAME")
    if current:
        solver["current_system_name"] = current.strip().strip("'")

    return MotionModel(
        path=str(path),
        units=units,
        gravity=gravity,
        bodies=bodies,
        markers=markers,
        joints=joints,
        drivers=drivers,
        gears=gears,
        solver=solver,
        raw_sections={
            "parts": len(_blocks(text, "PART")),
            "markers": len(markers),
            "joints": len(joints),
            "drivers": len(drivers),
            "gears": len(gears),
        },
    )


def parse_msg(path: str | Path) -> dict[str, Any]:
    path = Path(path)
    text = path.read_text(encoding="utf-8", errors="replace")
    result: dict[str, Any] = {"path": str(path), "solver_version": None, "parts": None, "joints": None}
    match = re.search(r"Solver Version\s*:\s*([^*\r\n]+)", text)
    if match:
        result["solver_version"] = match.group(1).strip()
    match = re.search(r"NO\. PART\s*=\s*(\d+)", text)
    if match:
        result["parts"] = int(match.group(1))
    match = re.search(r"NO\. JOINT\s*=\s*(\d+)", text)
    if match:
        result["joints"] = int(match.group(1))
    return result


def kinematics(model: MotionModel) -> dict[str, Any]:
    """Summarize the declared driver and gear-coupler ratios."""
    drivers = []
    for driver in model.drivers:
        drivers.append(
            {
                "name": driver.name,
                "joint": driver.joint,
                "function": driver.function,
                "type": "rotation" if driver.rotation else "translation",
            }
        )
    gears = []
    for gear in model.gears:
        gears.append(
            {
                "name": gear.name,
                "joints": gear.joints,
                "ratio": -1.0 if len(gear.joints) == 2 else None,
            }
        )
    return {"drivers": drivers, "gear_couplers": gears}


def write_motion_report(model: MotionModel, path: str | Path) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    data = {
        "model": model.to_dict(),
        "kinematics": kinematics(model),
    }
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
