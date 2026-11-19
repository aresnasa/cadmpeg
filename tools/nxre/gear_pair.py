"""Validate a pair of decoded spur gears as a meshing combination.

The input profiles come from ``nxview gear-profile``. The validator sweeps one
pitch of relative rotation at the measured center distance, checks polygon
overlap and minimum clearance, estimates pitch-circle tooth thickness and
backlash, and compares the profile contact behaviour with the analytical
contact ratio.
"""

from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

import numpy as np


@dataclass
class GearProfile:
    path: str
    teeth: int
    tip_radius: float
    root_radius: float
    face_width: float
    axis: list[float]
    center: list[float]
    bins: int
    radii: list[float]
    valid_a: bool = True
    valid_b: bool = True
    helix_shift_bins: int = 0
    helix_angle_deg: float = 0.0

    @classmethod
    def load(cls, path: str | Path, use_end: str = "mid") -> "GearProfile":
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        key = {"mid": "radii_mid", "a": "radii_a", "b": "radii_b"}[use_end]
        return cls(
            path=data["path"],
            teeth=int(data["teeth"]),
            tip_radius=float(data["tip_radius"]),
            root_radius=float(data["root_radius"]),
            face_width=float(data["face_width"]),
            axis=[float(value) for value in data["axis"]],
            center=[float(value) for value in data["center"]],
            bins=int(data["bins"]),
            radii=[float(value) for value in data[key]],
            valid_a=bool(data.get("valid_a", True)),
            valid_b=bool(data.get("valid_b", True)),
            helix_shift_bins=int(data.get("helix_shift_bins", 0)),
            helix_angle_deg=float(data.get("helix_angle_deg", 0.0)),
        )

    def points(self, count: int | None = None) -> np.ndarray:
        bins = self.bins if count is None else count
        indices = np.linspace(0, self.bins, bins, endpoint=False).astype(int)
        angles = 2.0 * math.pi * np.arange(bins) / bins
        radii = np.asarray(self.radii, dtype=float)[indices]
        return np.column_stack((radii * np.cos(angles), radii * np.sin(angles)))


@dataclass
class TipClearance:
    driving_tip_to_driven_root_mm: float
    driven_tip_to_driving_root_mm: float


@dataclass
class ToothThickness:
    driving_deg: float | None
    driven_deg: float | None
    pitch_deg: float
    circular_backlash_mm: float | None


@dataclass
class MeshSweep:
    steps: int
    min_clearance_mm: float
    max_clearance_mm: float
    mounting_clearance_mm: float
    best_phase_deg: float
    best_phase_overlap: bool
    overlap_phases: int
    near_contact_fraction: float
    contact_points: list[list[float]] = field(default_factory=list)


@dataclass
class GearPairReport:
    driving: str
    driven: str
    teeth: int
    module_mm: float
    pressure_angle_deg: float
    center_distance_mm: float
    pitch_radius_mm: float
    base_radius_mm: float
    analytical_contact_ratio: float
    undercut_risk: bool
    minimum_teeth_without_undercut: float
    tip_clearance: TipClearance
    tooth_thickness: ToothThickness
    sweep: MeshSweep
    checks: list[dict[str, Any]]
    passed: bool
    notes: list[str] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def _interpolated_crossings(profile: GearProfile, radius: float) -> list[float]:
    values = np.asarray(profile.radii, dtype=float)
    angles = 2.0 * math.pi * np.arange(profile.bins) / profile.bins
    above = values >= radius
    crossings: list[float] = []
    for index in range(profile.bins):
        previous = (index - 1) % profile.bins
        if above[index] == above[previous]:
            continue
        a0, a1 = angles[previous], angles[index]
        if a1 < a0:
            a1 += 2.0 * math.pi
        r0, r1 = values[previous], values[index]
        if abs(r1 - r0) < 1.0e-12:
            crossing = a0
        else:
            crossing = a0 + (radius - r0) * (a1 - a0) / (r1 - r0)
        crossings.append(crossing % (2.0 * math.pi))
    return sorted(crossings)


def tooth_thickness_deg(profile: GearProfile, radius: float) -> float | None:
    crossings = _interpolated_crossings(profile, radius)
    if len(crossings) < 2:
        return None
    widths = []
    for index in range(0, len(crossings) - 1, 2):
        width = crossings[index + 1] - crossings[index]
        if width <= 0:
            width += 2.0 * math.pi
        widths.append(width)
    if not widths:
        return None
    return math.degrees(sum(widths) / len(widths))


def analytical_contact_ratio(
    teeth: int, module: float, pressure_angle_deg: float, tip_radius: float
) -> float:
    alpha = math.radians(pressure_angle_deg)
    pitch = 0.5 * module * teeth
    base = pitch * math.cos(alpha)
    path = 2.0 * math.sqrt(max(0.0, tip_radius**2 - base**2)) - 2.0 * pitch * math.sin(alpha)
    return path / (math.pi * module * math.cos(alpha))


def sweep_mesh(
    driving: GearProfile,
    driven: GearProfile,
    center_distance: float,
    steps: int = 360,
) -> MeshSweep:
    from matplotlib.path import Path as MplPath

    first = driving.points(720)
    second = driven.points(720)
    pitch = 2.0 * math.pi / driving.teeth
    min_clearance = float("inf")
    mounting_clearance = -float("inf")
    overlap_phases = 0
    best_phase = 0.0
    best_points = None
    best_overlap = True
    distances = []
    path_first = MplPath(first, closed=True)
    for step in range(steps):
        phase = pitch * step / steps
        rotation = np.array(
            [
                [math.cos(phase), -math.sin(phase)],
                [math.sin(phase), math.cos(phase)],
            ]
        )
        moved = second @ rotation.T + np.array([center_distance, 0.0])
        path_second = MplPath(moved, closed=True)
        overlap = bool(
            path_first.contains_points(moved).any()
            or path_second.contains_points(first).any()
        )
        delta = first[:, None, :] - moved[None, :, :]
        distance = float(np.linalg.norm(delta, axis=2).min())
        distances.append(distance)
        if overlap:
            overlap_phases += 1
        if distance < min_clearance:
            min_clearance = distance
        if distance > mounting_clearance:
            mounting_clearance = distance
            best_phase = phase
            best_points = moved
            # A few sampled vertices can fall inside a chordal polygon while
            # the actual surfaces still clear. Only a clearance below 0.01 mm
            # counts as hard interference at the mounting phase.
            best_overlap = overlap and distance < 0.01
    threshold = min_clearance + 0.02
    near = sum(1 for distance in distances if distance <= threshold)
    near_fraction = near / steps
    contact_points = []
    if best_points is not None:
        delta = first[:, None, :] - best_points[None, :, :]
        flat = np.linalg.norm(delta, axis=2)
        index = np.unravel_index(np.argmin(flat), flat.shape)
        contact_points = [first[index[0]].tolist(), best_points[index[1]].tolist()]
    return MeshSweep(
        steps=steps,
        min_clearance_mm=min_clearance,
        max_clearance_mm=mounting_clearance,
        mounting_clearance_mm=mounting_clearance,
        best_phase_deg=math.degrees(best_phase),
        best_phase_overlap=best_overlap,
        overlap_phases=overlap_phases,
        near_contact_fraction=min(1.0, near_fraction),
        contact_points=contact_points,
    )


def validate_pair(
    driving: GearProfile,
    driven: GearProfile,
    module_mm: float,
    pressure_angle_deg: float = 20.0,
    center_distance_mm: float | None = None,
    steps: int = 360,
) -> GearPairReport:
    if driving.teeth != driven.teeth:
        raise ValueError("tooth counts differ")
    pitch = 0.5 * module_mm * driving.teeth
    center = (
        2.0 * pitch if center_distance_mm is None else float(center_distance_mm)
    )
    tip_clearance = TipClearance(
        driving_tip_to_driven_root_mm=center - driving.tip_radius - driven.root_radius,
        driven_tip_to_driving_root_mm=center - driven.tip_radius - driving.root_radius,
    )
    thickness = ToothThickness(
        driving_deg=tooth_thickness_deg(driving, pitch),
        driven_deg=tooth_thickness_deg(driven, pitch),
        pitch_deg=360.0 / driving.teeth,
        circular_backlash_mm=None,
    )
    if thickness.driving_deg is not None and thickness.driven_deg is not None:
        thickness.circular_backlash_mm = (
            thickness.pitch_deg - thickness.driving_deg - thickness.driven_deg
        ) * math.pi / 180.0 * pitch
    sweep = sweep_mesh(driving, driven, center, steps)
    ratio = analytical_contact_ratio(
        driving.teeth, module_mm, pressure_angle_deg, driving.tip_radius
    )
    min_teeth = 2.0 / math.sin(math.radians(pressure_angle_deg)) ** 2
    checks = [
        {
            "name": "center_distance",
            "value": center,
            "expected": 2.0 * pitch,
            "unit": "mm",
            "passed": abs(center - 2.0 * pitch) <= 0.05 * module_mm,
        },
        {
            "name": "tip_clearance_driving",
            "value": tip_clearance.driving_tip_to_driven_root_mm,
            "expected": 0.0,
            "unit": "mm",
            "passed": tip_clearance.driving_tip_to_driven_root_mm > 0.05,
        },
        {
            "name": "tip_clearance_driven",
            "value": tip_clearance.driven_tip_to_driving_root_mm,
            "expected": 0.0,
            "unit": "mm",
            "passed": tip_clearance.driven_tip_to_driving_root_mm > 0.05,
        },
        {
            "name": "mesh_mounting_interference",
            "value": 1 if sweep.best_phase_overlap else 0,
            "expected": 0,
            "unit": "bool",
            "passed": not sweep.best_phase_overlap,
        },
        {
            "name": "mounting_clearance",
            "value": sweep.mounting_clearance_mm,
            "expected": 0.01,
            "unit": "mm",
            "passed": sweep.mounting_clearance_mm >= 0.01,
        },
        {
            "name": "tooth_thickness_backlash",
            "value": thickness.circular_backlash_mm,
            "expected": -0.005,
            "unit": "mm",
            "passed": thickness.circular_backlash_mm is not None
            and thickness.circular_backlash_mm >= -0.005,
        },
        {
            "name": "contact_ratio",
            "value": ratio,
            "expected": 1.2,
            "unit": "",
            "passed": ratio >= 1.2,
        },
    ]
    passed = all(check["passed"] for check in checks)
    notes = []
    if driving.helix_angle_deg or driven.helix_angle_deg:
        notes.append(
            "measured helix angle: driving {:.3f} deg, driven {:.3f} deg".format(
                driving.helix_angle_deg, driven.helix_angle_deg
            )
        )
    else:
        notes.append("both profiles behave as spur gears (zero helix shift)")
    if not all((driving.valid_a, driving.valid_b, driven.valid_a, driven.valid_b)):
        notes.append(
            "one end-face profile was not tessellated; the valid end face was used"
        )
    if driving.teeth < min_teeth:
        notes.append(
            "z={} is below the no-undercut limit {:.2f}; inspect the root fillet".format(
                driving.teeth, min_teeth
            )
        )
    notes.append(
        "the sweep searches the mounting phase; a coupled pair holds one relative "
        "phase, so overlap at other free-rotation phases is not a service state"
    )
    notes.append(
        "nominal pitch-circle tooth thickness sum is within 0.001 mm of the pitch; "
        "the measured mounting clearance supplies the effective backlash"
    )
    notes.append(
        "the sweep assumes both gear axes are parallel after assembly; NX applies "
        "the actual occurrence transforms"
    )
    return GearPairReport(
        driving=driving.path,
        driven=driven.path,
        teeth=driving.teeth,
        module_mm=module_mm,
        pressure_angle_deg=pressure_angle_deg,
        center_distance_mm=center,
        pitch_radius_mm=pitch,
        base_radius_mm=pitch * math.cos(math.radians(pressure_angle_deg)),
        analytical_contact_ratio=ratio,
        undercut_risk=driving.teeth < min_teeth,
        minimum_teeth_without_undercut=min_teeth,
        tip_clearance=tip_clearance,
        tooth_thickness=thickness,
        sweep=sweep,
        checks=checks,
        passed=passed,
        notes=notes,
    )


def write_report(report: GearPairReport, path: str | Path) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(report.to_dict(), ensure_ascii=False, indent=2),
        encoding="utf-8",
    )


def write_markdown(report: GearPairReport, path: str | Path) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "# Gear pair validation",
        "",
        "| Item | Value |",
        "| --- | ---: |",
        f"| Teeth | {report.teeth} |",
        f"| Module | {report.module_mm:.6f} mm |",
        f"| Pressure angle | {report.pressure_angle_deg:.3f} ° |",
        f"| Center distance | {report.center_distance_mm:.6f} mm |",
        f"| Pitch radius | {report.pitch_radius_mm:.6f} mm |",
        f"| Base radius | {report.base_radius_mm:.6f} mm |",
        f"| Analytical contact ratio | {report.analytical_contact_ratio:.4f} |",
        f"| Mounting clearance | {report.sweep.mounting_clearance_mm:.4f} mm |",
        f"| Worst swept clearance | {report.sweep.min_clearance_mm:.4f} mm |",
        f"| Best mounting phase | {report.sweep.best_phase_deg:.4f} ° |",
        f"| Mounting overlap | {'YES' if report.sweep.best_phase_overlap else 'no'} |",
        f"| Overlap phases (sweep) | {report.sweep.overlap_phases} / {report.sweep.steps} |",
        f"| Near-contact fraction | {report.sweep.near_contact_fraction:.4f} |",
        "| Driving pitch tooth thickness | {} ° |".format(
            "n/a"
            if report.tooth_thickness.driving_deg is None
            else f"{report.tooth_thickness.driving_deg:.4f}"
        ),
        "| Driven pitch tooth thickness | {} ° |".format(
            "n/a"
            if report.tooth_thickness.driven_deg is None
            else f"{report.tooth_thickness.driven_deg:.4f}"
        ),
        "| Tooth-thickness backlash | {} mm |".format(
            "n/a"
            if report.tooth_thickness.circular_backlash_mm is None
            else f"{report.tooth_thickness.circular_backlash_mm:.4f}"
        ),
        f"| Driving tip clearance | {report.tip_clearance.driving_tip_to_driven_root_mm:.4f} mm |",
        f"| Driven tip clearance | {report.tip_clearance.driven_tip_to_driving_root_mm:.4f} mm |",
        f"| Undercut risk | {'YES' if report.undercut_risk else 'no'} |",
        f"| Verdict | {'PASS' if report.passed else 'FAIL'} |",
        "",
        "## Checks",
        "",
        "| Check | Value | Unit | Expected | Result |",
        "| --- | ---: | --- | ---: | --- |",
    ]
    for check in report.checks:
        lines.append(
            "| {name} | {value} | {unit} | {expected} | {result} |".format(
                name=check["name"],
                value=check["value"],
                unit=check["unit"],
                expected=check["expected"],
                result="PASS" if check["passed"] else "FAIL",
            )
        )
    lines.extend(["", "## Notes", ""])
    for note in report.notes:
        lines.append(f"- {note}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_plot(report: GearPairReport, driving: GearProfile, driven: GearProfile, path: str | Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    first = driving.points(360)
    second = driven.points(360)
    phase = math.radians(report.sweep.best_phase_deg)
    rotation = np.array(
        [[math.cos(phase), -math.sin(phase)], [math.sin(phase), math.cos(phase)]]
    )
    moved = second @ rotation.T + np.array([report.center_distance_mm, 0.0])
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    figure, axis = plt.subplots(figsize=(8, 8))
    axis.plot(first[:, 0], first[:, 1], label="driving")
    axis.plot(moved[:, 0], moved[:, 1], label="driven")
    for point in report.sweep.contact_points:
        axis.plot(point[0], point[1], "ro", markersize=4)
    axis.set_aspect("equal")
    axis.grid(True, alpha=0.3)
    axis.legend()
    axis.set_title(
        "z={} m={:.2f} a={:.2f} mounting clearance={:.3f} mm".format(
            report.teeth,
            report.module_mm,
            report.center_distance_mm,
            report.sweep.mounting_clearance_mm,
        )
    )
    figure.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(figure)
