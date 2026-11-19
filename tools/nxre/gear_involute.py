"""Fit and validate involute gear tooth flanks.

The module works from two evidence sources:

* a polar end-face envelope from ``nxview gear-profile`` (1440 bins by
  default), and
* the exact B-rep boundary curves retained in the cadmpeg model JSON.

Both sources describe the same transverse tooth profile.  The fit recovers the
base radius, the pressure angle at a reference radius, the angular tooth
thickness and the profile shift.  A second pass measures the root fillet,
the involute/fillet junction and the undercut state.
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Iterable, Sequence

import numpy as np

TAU = 2.0 * math.pi

# The NX10 corpus uses a 6-edge tooth: root arc, root fillet, flank, tip land,
# flank, root fillet.
EDGES_PER_TOOTH = 6


@dataclass
class InvoluteFit:
    base_radius_mm: float
    pressure_angle_deg: float
    reference_radius_mm: float
    phase_rad: float
    tooth_half_angle_deg: float | None
    profile_shift: float | None
    samples: int
    rejected: int
    r_min_mm: float
    r_max_mm: float
    rms_angular_rad: float
    rms_tangential_mm: float
    max_tangential_mm: float
    base_radius_stderr_mm: float
    pressure_angle_stderr_deg: float
    residual_bands: list[dict[str, float]] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class CircleFit:
    center: list[float]
    radius_mm: float
    rms_mm: float
    max_mm: float

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class FilletMeasurement:
    root_radius_mm: float
    flank_start_radius_mm: float
    angular_span_deg: float
    arc_length_mm: float
    circle: CircleFit
    curvature_radius_root_mm: float
    curvature_radius_flank_mm: float
    curvature_radius_min_mm: float
    curvature_radius_max_mm: float
    root_tangent_error_deg: float
    flank_tangent_error_deg: float
    circular: bool
    classification: str

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class UndercutAssessment:
    teeth: int
    pressure_angle_deg: float
    minimum_teeth_without_undercut: float
    below_undercut_limit: bool
    base_radius_mm: float
    involute_start_radius_mm: float
    involute_start_above_base_mm: float
    theoretical_start_radius_sharp_mm: float | None
    theoretical_start_radius_rounded_mm: float | None
    junction_kink_deg: float
    visible_undercut: bool
    notes: list[str] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class ToothMeasurement:
    tooth_index: int
    center_angle_deg: float
    tip_radius_mm: float
    tip_land_span_deg: float
    pitch_thickness_deg: float | None
    pitch_thickness_mm: float | None
    left_flank_start_radius_mm: float
    right_flank_start_radius_mm: float


@dataclass
class EndFaceAnalysis:
    face_id: str
    loop_id: str
    teeth: int
    center: list[float]
    normal: list[float]
    station_mm: float
    involute: InvoluteFit
    local_tangent_base_radius_mm: float
    local_tangent_pressure_angle_deg: float
    local_tangent_stderr_mm: float
    fillet: FilletMeasurement
    undercut: UndercutAssessment
    tip: CircleFit
    tooth_samples: list[ToothMeasurement] = field(default_factory=list)
    alternatives: dict[str, dict[str, float]] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        value = asdict(self)
        return value


@dataclass
class GearInvoluteReport:
    source: str
    teeth: int
    module_mm: float
    face_width_mm: float
    pitch_radius_mm: float
    tip_radius_mm: float
    root_radius_mm: float
    helix_angle_deg: float
    end_faces: list[EndFaceAnalysis]
    checks: list[dict[str, Any]]
    passed: bool
    notes: list[str] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def involute_inv(alpha: np.ndarray | float) -> np.ndarray | float:
    """Involute function ``tan(alpha) - alpha``."""

    return np.tan(alpha) - alpha


def involute_pressure_angle(radius: np.ndarray, base_radius: float) -> np.ndarray:
    """Pressure angle at ``radius`` for an involute of ``base_radius``."""

    return np.arccos(np.clip(base_radius / radius, -1.0, 1.0))


def involute_radius(base_radius: float, alpha: np.ndarray | float) -> np.ndarray | float:
    """Radius at involute roll angle ``alpha``."""

    return base_radius / np.cos(alpha)


def involute_point(
    base_radius: float, alpha: float, phase: float = 0.0, side: int = 1
) -> tuple[float, float]:
    """Cartesian point on one involute flank.

    ``alpha`` is the pressure angle at the point, ``phase`` is the tooth
    center angle, and ``side`` selects the left (-1) or right (+1) flank.
    """

    radius = base_radius / math.cos(alpha)
    angle = phase + side * (involute_inv(alpha))
    return radius * math.cos(angle), radius * math.sin(angle)


def _unit(vector: np.ndarray) -> np.ndarray:
    length = float(np.linalg.norm(vector))
    if length == 0.0:
        raise ValueError("cannot normalize a zero vector")
    return vector / length


def _plane_basis(normal: Sequence[float]) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    n = _unit(np.asarray(normal, dtype=float))
    helper = np.array([1.0, 0.0, 0.0])
    if abs(float(np.dot(n, helper))) > 0.9:
        helper = np.array([0.0, 1.0, 0.0])
    u = _unit(np.cross(n, helper))
    v = np.cross(n, u)
    return u, v, n


def _project(points: np.ndarray, normal: Sequence[float]) -> np.ndarray:
    u, v, _ = _plane_basis(normal)
    return np.column_stack((points @ u, points @ v))


def _angle_delta(value: np.ndarray, center: float) -> np.ndarray:
    return (value - center + math.pi) % TAU - math.pi


def _circular_mean(angles: np.ndarray) -> float:
    return math.atan2(float(np.sin(angles).mean()), float(np.cos(angles).mean())) % TAU


def circle_fit(points: np.ndarray) -> CircleFit:
    """Algebraic least-squares circle through 2-D points."""

    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or len(points) < 3:
        raise ValueError("circle_fit needs at least three 2-D points")
    x = points[:, 0]
    y = points[:, 1]
    matrix = np.column_stack((x, y, np.ones(len(points))))
    solution, *_ = np.linalg.lstsq(matrix, x * x + y * y, rcond=None)
    cx = 0.5 * solution[0]
    cy = 0.5 * solution[1]
    radius = math.sqrt(max(0.0, solution[2] + cx * cx + cy * cy))
    residual = np.hypot(x - cx, y - cy) - radius
    return CircleFit(
        center=[float(cx), float(cy)],
        radius_mm=float(radius),
        rms_mm=float(np.sqrt(np.mean(residual**2))),
        max_mm=float(np.max(np.abs(residual))),
    )


def curvature_radius(points: np.ndarray, resample: int = 2000) -> np.ndarray:
    """Curvature radius along an ordered 2-D polyline."""

    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or len(points) < 5:
        raise ValueError("curvature_radius needs an ordered 2-D polyline")
    segment = np.diff(points, axis=0)
    length = np.concatenate(([0.0], np.cumsum(np.linalg.norm(segment, axis=1))))
    total = float(length[-1])
    if total <= 0.0:
        raise ValueError("polyline has zero length")
    sample = np.linspace(0.0, total, resample)
    x = np.interp(sample, length, points[:, 0])
    y = np.interp(sample, length, points[:, 1])
    xy = np.column_stack((x, y))
    first = np.gradient(xy, axis=0)
    second = np.gradient(first, axis=0)
    numerator = np.abs(first[:, 0] * second[:, 1] - first[:, 1] * second[:, 0])
    denominator = np.power(np.hypot(first[:, 0], first[:, 1]), 3.0)
    with np.errstate(divide="ignore", invalid="ignore"):
        radius = np.where(numerator > 0.0, denominator / numerator, np.inf)
    return radius


def _golden_minimum(function, lower: float, upper: float, iterations: int = 120) -> float:
    ratio = (math.sqrt(5.0) - 1.0) / 2.0
    left = upper - ratio * (upper - lower)
    right = lower + ratio * (upper - lower)
    f_left = function(left)
    f_right = function(right)
    for _ in range(iterations):
        if f_left < f_right:
            upper = right
            right = left
            f_right = f_left
            left = upper - ratio * (upper - lower)
            f_left = function(left)
        else:
            lower = left
            left = right
            f_left = f_right
            right = lower + ratio * (upper - lower)
            f_right = function(right)
        if upper - lower <= 1.0e-12 * (1.0 + abs(lower) + abs(upper)):
            break
    return 0.5 * (lower + upper)


def _residual_bands(
    radii: np.ndarray, residual: np.ndarray, reference_radius: float, bins: int = 6
) -> list[dict[str, float]]:
    if len(radii) == 0:
        return []
    edges = np.linspace(float(radii.min()), float(radii.max()), bins + 1)
    bands = []
    for index in range(bins):
        lower, upper = edges[index], edges[index + 1]
        last = index == bins - 1
        mask = (radii >= lower) & ((radii <= upper) if last else (radii < upper))
        if not mask.any():
            continue
        values = residual[mask] * reference_radius
        bands.append(
            {
                "r_min_mm": float(lower),
                "r_max_mm": float(upper),
                "samples": int(mask.sum()),
                "mean_um": float(values.mean() * 1000.0),
                "rms_um": float(np.sqrt(np.mean(values**2)) * 1000.0),
                "max_um": float(np.max(np.abs(values)) * 1000.0),
            }
        )
    return bands


def fit_involute(
    radii: Iterable[float],
    angles: Iterable[float],
    sides: Iterable[int] | None = None,
    reference_radius: float | None = None,
    r_min: float | None = None,
    r_max: float | None = None,
    teeth: int | None = None,
    reject_sigma: float = 4.0,
) -> InvoluteFit:
    """Fit ``base_radius`` and tooth half-angle to centered flank samples.

    ``angles`` are polar angles in radians relative to the tooth center.  The
    model is ``abs(angle) = half_angle + inv(a_p) - inv(a)``.  ``sides`` is
    accepted for callers that carry the flank side; the magnitude removes the
    left/right sign.  ``phase_rad`` is the fitted constant
    ``half_angle + inv(a_p)``.
    """

    radius = np.asarray(list(radii), dtype=float)
    angle = np.asarray(list(angles), dtype=float)
    if radius.shape != angle.shape:
        raise ValueError("radii and angles must have equal length")
    if sides is not None:
        side = np.asarray(list(sides), dtype=float)
        if side.shape != radius.shape:
            raise ValueError("sides must have the same length as radii")
    else:
        side = np.sign(angle)
        side[side == 0.0] = 1.0
    if reference_radius is None:
        reference_radius = float(np.median(radius))
    reference_radius = float(reference_radius)

    lower = float(np.min(radius)) if r_min is None else float(r_min)
    upper = float(np.max(radius)) if r_max is None else float(r_max)
    mask = (
        np.isfinite(radius)
        & np.isfinite(angle)
        & (radius >= lower)
        & (radius <= upper)
    )
    if mask.sum() < 8:
        raise ValueError("fewer than eight flank samples in the fit window")
    r = radius[mask]
    magnitude = np.abs(angle[mask])

    def solve(rr: np.ndarray, uu: np.ndarray):
        def residual(rb: float) -> tuple[float, float, np.ndarray]:
            alpha = np.arccos(np.clip(rb / rr, -1.0, 1.0))
            inva = np.tan(alpha) - alpha
            constant = float(np.mean(uu + inva))
            values = uu + inva - constant
            return float(np.sum(values * values)), constant, values

        rb_upper = float(np.min(rr)) * 0.999999
        rb_lower = max(1.0e-6, 0.80 * reference_radius)
        if rb_lower >= rb_upper:
            raise ValueError("fit window starts at or below the base radius")
        grid = np.linspace(rb_lower, rb_upper, 4001)
        scores = np.array([residual(float(value))[0] for value in grid])
        best = int(np.argmin(scores))
        step = grid[1] - grid[0]
        low = max(rb_lower, float(grid[best] - 2.0 * step))
        high = min(rb_upper, float(grid[best] + 2.0 * step))
        rb = _golden_minimum(lambda value: residual(value)[0], low, high)
        sse, constant, values = residual(rb)
        return rb, constant, values, sse

    rejected = 0
    rb, phase, values, sse = solve(r, magnitude)
    for _ in range(2):
        sigma = math.sqrt(sse / max(1, len(values) - 2))
        if sigma <= 0.0:
            break
        keep = np.abs(values) <= reject_sigma * sigma
        if keep.all():
            break
        rejected += int((~keep).sum())
        r, magnitude = r[keep], magnitude[keep]
        rb, phase, values, sse = solve(r, magnitude)

    alpha = np.arccos(np.clip(rb / r, -1.0, 1.0))
    sigma2 = sse / max(1, len(values) - 2)
    d_inv_d_alpha = np.tan(alpha) ** 2
    d_alpha_d_rb = -1.0 / (r * np.sin(alpha))
    jacobian = np.column_stack((d_inv_d_alpha * d_alpha_d_rb, -np.ones(len(r))))
    try:
        covariance = sigma2 * np.linalg.inv(jacobian.T @ jacobian)
        rb_stderr = float(math.sqrt(max(0.0, covariance[0, 0])))
    except np.linalg.LinAlgError:
        rb_stderr = float("nan")

    alpha_ref = math.acos(min(1.0, rb / reference_radius))
    tooth_half = phase - involute_inv(alpha_ref)
    tooth_half_deg = math.degrees(tooth_half)
    profile_shift = None
    if teeth is not None and teeth > 0:
        profile_shift = (
            (tooth_half - math.pi / (2.0 * teeth))
            * teeth
            / (2.0 * math.tan(alpha_ref))
        )
    rms_angular = float(np.sqrt(np.mean(values**2)))
    return InvoluteFit(
        base_radius_mm=float(rb),
        pressure_angle_deg=math.degrees(alpha_ref),
        reference_radius_mm=reference_radius,
        phase_rad=float(phase),
        tooth_half_angle_deg=float(tooth_half_deg),
        profile_shift=None if profile_shift is None else float(profile_shift),
        samples=int(len(r)),
        rejected=rejected,
        r_min_mm=float(r.min()),
        r_max_mm=float(r.max()),
        rms_angular_rad=rms_angular,
        rms_tangential_mm=float(rms_angular * reference_radius),
        max_tangential_mm=float(np.max(np.abs(values)) * reference_radius),
        base_radius_stderr_mm=rb_stderr,
        pressure_angle_stderr_deg=float(
            math.degrees(rb_stderr / (reference_radius * math.sin(alpha_ref)))
        ),
        residual_bands=_residual_bands(r, values, reference_radius),
    )


def _tip_land_runs(radii: np.ndarray, tip_radius: float, tolerance: float) -> list[tuple[int, int]]:
    bins = len(radii)
    mask = radii >= tip_radius - tolerance
    indices = np.where(mask)[0]
    if len(indices) == 0:
        return []
    runs = []
    start = int(indices[0])
    previous = int(indices[0])
    for value in indices[1:]:
        value = int(value)
        if value == previous + 1:
            previous = value
            continue
        runs.append((start, previous))
        start = value
        previous = value
    runs.append((start, previous))
    if len(runs) > 1 and runs[0][0] == 0 and runs[-1][1] == bins - 1:
        start, end = runs.pop()
        runs[0] = (start - bins, runs[0][1])
    return runs


def fit_involute_profile(
    radii: Sequence[float],
    teeth: int,
    tip_radius: float,
    root_radius: float | None = None,
    reference_radius: float | None = None,
    r_min: float | None = None,
    r_max: float | None = None,
    tip_tolerance: float | None = None,
) -> tuple[InvoluteFit, dict[str, Any]]:
    """Fit an involute to a polar end-face envelope.

    ``radii`` is the maximum radius per angular bin.  The function folds the
    14 teeth, rejects the tip land and the root fillet, and returns the fit
    plus the detected tooth centers.
    """

    values = np.asarray(radii, dtype=float)
    bins = len(values)
    if bins < 32:
        raise ValueError("profile has too few bins")
    if teeth < 3:
        raise ValueError("teeth must be at least three")
    pitch = TAU / teeth
    if reference_radius is None:
        module = 2.0 * tip_radius / (teeth + 2.0)
        reference_radius = 0.5 * module * teeth
    reference_radius = float(reference_radius)
    if tip_tolerance is None:
        tip_tolerance = min(0.02, max(1.0e-4, 0.0005 * tip_radius))
    angles = TAU * np.arange(bins) / bins
    runs = _tip_land_runs(values, tip_radius, tip_tolerance)
    centers: list[float] = []
    if runs:
        for start, end in runs:
            indices = np.arange(start, end + 1) % bins
            centers.append(_circular_mean(angles[indices]))
        if len(centers) != teeth:
            phases = np.array([(center % pitch) for center in centers], dtype=float)
            scaled = TAU * phases / pitch
            mean_phase = _circular_mean(scaled) * pitch / TAU
            centers = [(mean_phase + index * pitch) % TAU for index in range(teeth)]
    if not centers:
        peak = int(np.argmax(values))
        centers = [(angles[peak] + index * pitch) % TAU for index in range(teeth)]
    centers = sorted(centers)

    folded_r = []
    folded_a = []
    for center in centers:
        delta = _angle_delta(angles, center)
        local = np.abs(delta) < 0.5 * pitch
        folded_r.append(values[local])
        folded_a.append(delta[local])
    radius = np.concatenate(folded_r)
    delta = np.concatenate(folded_a)

    if r_min is None:
        r_min = 0.94 * reference_radius
        if root_radius is not None:
            r_min = max(r_min, root_radius + 0.5 * (reference_radius - root_radius))
    if r_max is None:
        r_max = tip_radius - max(0.01, 0.0005 * tip_radius)
    fit = fit_involute(
        radius,
        delta,
        sides=np.sign(delta),
        reference_radius=reference_radius,
        r_min=r_min,
        r_max=r_max,
        teeth=teeth,
    )
    diagnostics = {
        "bins": bins,
        "tip_land_runs": len(runs),
        "tip_runs_merged": len(runs) != teeth,
        "tooth_centers_deg": [math.degrees(value) for value in centers],
        "r_min_mm": float(r_min),
        "r_max_mm": float(r_max),
        "folded_samples": int(len(radius)),
    }
    return fit, diagnostics


def _tangent_fit(radii: np.ndarray, tangent: np.ndarray, radial: np.ndarray) -> tuple[float, float, float]:
    """Base radius and pressure angle from local profile tangents."""

    dot = np.abs(np.sum(tangent * radial, axis=1))
    alpha = np.arccos(np.clip(dot, 0.0, 1.0))
    base = radii * np.cos(alpha)
    median = float(np.median(base))
    return median, float(np.median(np.degrees(alpha))), float(np.std(base))


def measure_fillet(
    points: np.ndarray,
    root_radius: float,
    flank_tangent: np.ndarray | None = None,
    flank_start_radius: float | None = None,
) -> FilletMeasurement:
    """Measure a root fillet from ordered root-to-flank points."""

    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or len(points) < 10:
        raise ValueError("fillet needs an ordered 2-D polyline")
    radii = np.hypot(points[:, 0], points[:, 1])
    if flank_start_radius is None:
        flank_start_radius = float(radii[-1])
    circle = circle_fit(points)
    curvature = curvature_radius(points, max(400, len(points)))
    window = max(3, len(curvature) // 40)
    root_curvature = float(np.median(curvature[:window]))
    flank_curvature = float(np.median(curvature[-window:]))
    interior = curvature[window:-window] if len(curvature) > 2 * window else curvature
    finite = interior[np.isfinite(interior)]
    minimum = float(np.min(finite)) if len(finite) else float("nan")
    maximum = float(np.max(finite)) if len(finite) else float("nan")

    first = points[1] - points[0]
    last = points[-1] - points[-2]
    root_tangent = np.array([-points[0, 1], points[0, 0]])
    root_error = math.degrees(
        math.acos(
            min(1.0, abs(float(np.dot(first, root_tangent)) / (np.linalg.norm(first) * np.linalg.norm(root_tangent))))
        )
    )
    if flank_tangent is None:
        flank_error = float("nan")
    else:
        flank_error = math.degrees(
            math.acos(
                min(
                    1.0,
                    abs(float(np.dot(last, flank_tangent)) / (np.linalg.norm(last) * np.linalg.norm(flank_tangent))),
                )
            )
        )
    angles = np.unwrap(np.arctan2(points[:, 1], points[:, 0]))
    span = abs(math.degrees(float(angles[-1] - angles[0])))
    length = float(np.sum(np.linalg.norm(np.diff(points, axis=0), axis=1)))
    variation = (maximum - minimum) / max(1.0e-12, 0.5 * (maximum + minimum))
    circular = circle.rms_mm < 5.0e-3 and variation < 0.25
    classification = "circular" if circular else "non-circular trochoid-like transition"
    return FilletMeasurement(
        root_radius_mm=float(root_radius),
        flank_start_radius_mm=float(flank_start_radius),
        angular_span_deg=float(span),
        arc_length_mm=length,
        circle=circle,
        curvature_radius_root_mm=root_curvature,
        curvature_radius_flank_mm=flank_curvature,
        curvature_radius_min_mm=minimum,
        curvature_radius_max_mm=maximum,
        root_tangent_error_deg=float(root_error),
        flank_tangent_error_deg=float(flank_error),
        circular=bool(circular),
        classification=classification,
    )


def assess_undercut(
    teeth: int,
    pressure_angle_deg: float,
    base_radius_mm: float,
    reference_radius_mm: float,
    involute_start_radius_mm: float,
    module_mm: float,
    junction_kink_deg: float,
    root_radius_mm: float,
) -> UndercutAssessment:
    """Compare the observed involute start with the rack undercut limit."""

    alpha = math.radians(pressure_angle_deg)
    minimum = 2.0 / math.sin(alpha) ** 2

    def start_radius(addendum_factor: float) -> float | None:
        tangent_offset = addendum_factor * module_mm / math.sin(alpha) - reference_radius_mm * math.sin(alpha)
        if tangent_offset <= 0.0:
            return None
        return math.hypot(base_radius_mm, tangent_offset)

    sharp = start_radius(1.0)
    rounded = start_radius(1.25)
    above_base = involute_start_radius_mm - base_radius_mm
    visible = junction_kink_deg > 0.5
    notes = []
    if teeth < minimum:
        notes.append(
            "z={} is below the standard full-depth limit {:.3f}; a sharp rack "
            "cutter would undercut the involute".format(teeth, minimum)
        )
    if visible:
        notes.append(
            "the involute meets the root fillet with a {:.2f} deg tangent "
            "discontinuity, which indicates an undercut/trochoid intersection "
            "rather than a smooth circular blend".format(junction_kink_deg)
        )
    if sharp is not None and involute_start_radius_mm < sharp - 0.01:
        notes.append(
            "the measured involute start {:.4f} mm is {:.4f} mm below the "
            "sharp-rack estimate {:.4f} mm; a rounded cutter shifts the "
            "intersection".format(
                involute_start_radius_mm,
                sharp - involute_start_radius_mm,
                sharp,
            )
        )
    return UndercutAssessment(
        teeth=teeth,
        pressure_angle_deg=pressure_angle_deg,
        minimum_teeth_without_undercut=minimum,
        below_undercut_limit=teeth < minimum,
        base_radius_mm=base_radius_mm,
        involute_start_radius_mm=involute_start_radius_mm,
        involute_start_above_base_mm=above_base,
        theoretical_start_radius_sharp_mm=sharp,
        theoretical_start_radius_rounded_mm=rounded,
        junction_kink_deg=junction_kink_deg,
        visible_undercut=visible,
        notes=notes,
    )


def _de_boor(knots: Sequence[float], degree: int, poles: np.ndarray, value: float) -> np.ndarray:
    count = len(poles) - 1
    if value >= knots[count + 1]:
        span = count
    elif value <= knots[degree]:
        span = degree
    else:
        span = degree
        for index in range(degree, count + 1):
            if knots[index] <= value < knots[index + 1]:
                span = index
                break
    work = [poles[span - degree + index].copy() for index in range(degree + 1)]
    for level in range(1, degree + 1):
        for index in range(degree, level - 1, -1):
            knot = span - degree + index
            denominator = knots[knot + degree - level + 1] - knots[knot]
            weight = 0.0 if denominator == 0.0 else (value - knots[knot]) / denominator
            work[index] = (1.0 - weight) * work[index - 1] + weight * work[index]
    return work[degree]


def evaluate_nurbs(curve: dict[str, Any], count: int = 400) -> np.ndarray:
    """Sample a polynomial or rational NURBS curve from the cadmpeg model."""

    geometry = curve.get("geometry", curve)
    if geometry.get("kind") != "nurbs":
        raise ValueError("curve is not a NURBS carrier")
    knots = [float(value) for value in geometry["knots"]]
    degree = int(geometry["degree"])
    pole_data = geometry["poles"]
    form = pole_data.get("form", "polynomial")
    points = pole_data["points"]
    poles = []
    weights = []
    for point in points:
        poles.append([float(point["x"]), float(point["y"]), float(point["z"])])
        weights.append(float(point.get("weight", 1.0)))
    poles_array = np.asarray(poles, dtype=float)
    lower = knots[degree]
    upper = knots[len(poles)]
    parameters = np.linspace(lower, upper, count)
    if form == "rational":
        homogeneous = np.column_stack((poles_array * np.asarray(weights)[:, None], weights))
        values = np.array(
            [_de_boor(knots, degree, homogeneous, value) for value in parameters]
        )
        return values[:, :3] / values[:, 3:4]
    return np.array([_de_boor(knots, degree, poles_array, value) for value in parameters])


def _arc_points(curve: dict[str, Any], start: dict[str, float], end: dict[str, float], count: int = 200) -> np.ndarray:
    geometry = curve["geometry"]
    center = np.array(
        [geometry["center"]["x"], geometry["center"]["y"], geometry["center"]["z"]], dtype=float
    )
    radius = float(geometry["radius"])
    axis = geometry.get("axis", {"x": 0.0, "y": 0.0, "z": 1.0})
    normal = _unit(np.array([axis["x"], axis["y"], axis["z"]], dtype=float))
    reference = geometry.get("ref_direction", {"x": 1.0, "y": 0.0, "z": 0.0})
    u = np.array([reference["x"], reference["y"], reference["z"]], dtype=float)
    u = _unit(u - normal * float(np.dot(u, normal)))
    v = np.cross(normal, u)
    first_point = np.array([start["x"], start["y"], start["z"]], dtype=float) - center
    last_point = np.array([end["x"], end["y"], end["z"]], dtype=float) - center
    first = math.atan2(float(np.dot(first_point, v)), float(np.dot(first_point, u)))
    last = math.atan2(float(np.dot(last_point, v)), float(np.dot(last_point, u)))
    delta = (last - first + math.pi) % TAU - math.pi
    angles = first + np.linspace(0.0, delta, count)
    return center + radius * (
        np.cos(angles)[:, None] * u[None, :] + np.sin(angles)[:, None] * v[None, :]
    )


class BrepModel:
    """Indexed view of a cadmpeg model JSON."""

    def __init__(self, model: dict[str, Any]):
        self.model = model
        self.loops = {item["id"]: item for item in model.get("loops", [])}
        self.coedges = {item["id"]: item for item in model.get("coedges", [])}
        self.edges = {item["id"]: item for item in model.get("edges", [])}
        self.curves = {item["id"]: item for item in model.get("curves", [])}
        self.vertices = {item["id"]: item for item in model.get("vertices", [])}
        self.points = {item["id"]: item for item in model.get("points", [])}
        self.surfaces = {item["id"]: item.get("geometry", {}) for item in model.get("surfaces", [])}

    @classmethod
    def load(cls, path: str | Path) -> "BrepModel":
        return cls(json.loads(Path(path).read_text(encoding="utf-8")))

    def vertex(self, identifier: str) -> dict[str, float]:
        return self.points[self.vertices[identifier]["point"]]["position"]

    def edge_points(self, coedge_id: str, count: int = 600) -> np.ndarray:
        coedge = self.coedges[coedge_id]
        edge = self.edges[coedge["edge"]]
        curve = self.curves[edge["carrier"]["curve"]]
        geometry = curve["geometry"]
        if geometry.get("kind") == "circle":
            points = _arc_points(curve, self.vertex(edge["start"]), self.vertex(edge["end"]), max(200, count // 2))
        elif geometry.get("kind") == "line":
            first = self.vertex(edge["start"])
            last = self.vertex(edge["end"])
            points = np.linspace(
                [first["x"], first["y"], first["z"]],
                [last["x"], last["y"], last["z"]],
                count,
            )
        else:
            points = evaluate_nurbs(curve, count)
        if coedge.get("sense") == "reversed":
            points = points[::-1]
        return points

    def ring_coedges(self, loop_id: str) -> list[str]:
        boundary = self.loops[loop_id]["boundary"]
        if boundary.get("kind") != "ring":
            raise ValueError("loop is not a ring")
        return list(boundary.get("coedges", []))

    def end_face_loops(self, minimum_edges: int = 18) -> list[tuple[dict[str, Any], str]]:
        result = []
        for face in self.model.get("faces", []):
            for loop_id in face.get("loops", {}).get("loops", []):
                boundary = self.loops[loop_id]["boundary"]
                count = len(boundary.get("coedges", []))
                if (
                    boundary.get("kind") == "ring"
                    and count >= minimum_edges
                    and count % EDGES_PER_TOOTH == 0
                ):
                    result.append((face, loop_id))
        return result


def _group_teeth(coedges: Sequence[str], model: BrepModel, normal: Sequence[float]) -> tuple[list[list[str]], int]:
    best_groups: list[list[str]] = []
    best_offset = 0
    for offset in range(EDGES_PER_TOOTH):
        shifted = list(coedges[offset:]) + list(coedges[:offset])
        groups = [
            shifted[index : index + EDGES_PER_TOOTH]
            for index in range(0, len(shifted), EDGES_PER_TOOTH)
        ]
        valid = 0
        for group in groups:
            radii = []
            for coedge_id in group:
                points = model.edge_points(coedge_id, 60)
                projected = _project(points, normal)
                radii.append(float(np.hypot(projected[:, 0], projected[:, 1]).max()))
            if max(radii) - min(radii) > 0.8:
                valid += 1
        if valid > best_offset:
            best_offset = valid
            best_groups = groups
    if not best_groups:
        best_groups = [
            list(coedges[index : index + EDGES_PER_TOOTH])
            for index in range(0, len(coedges), EDGES_PER_TOOTH)
        ]
    return best_groups, best_offset


def _radius_crossing(points: np.ndarray, radius: float) -> float | None:
    values = np.hypot(points[:, 0], points[:, 1])
    for index in range(1, len(values)):
        if (values[index - 1] - radius) * (values[index] - radius) <= 0.0:
            first = values[index - 1]
            last = values[index]
            if abs(last - first) < 1.0e-15:
                return float(math.atan2(points[index, 1], points[index, 0]))
            weight = (radius - first) / (last - first)
            x = points[index - 1, 0] + weight * (points[index, 0] - points[index - 1, 0])
            y = points[index - 1, 1] + weight * (points[index, 1] - points[index - 1, 1])
            return float(math.atan2(y, x))
    return None


def _canonical_axis(normal: Sequence[float]) -> np.ndarray:
    axis = np.asarray(normal, dtype=float).copy()
    for component in axis:
        if abs(float(component)) > 1.0e-9:
            if component < 0.0:
                axis = -axis
            break
    return axis


def _station(origin: dict[str, float], axis: Sequence[float]) -> float:
    return float(
        origin["x"] * axis[0] + origin["y"] * axis[1] + origin["z"] * axis[2]
    )


def analyze_end_face(
    model: BrepModel,
    face: dict[str, Any],
    loop_id: str,
    teeth: int,
    module_mm: float,
    reference_radius_mm: float,
    face_width_mm: float,
    flank_samples: int = 400,
) -> EndFaceAnalysis:
    surface = model.surfaces[face["surface"]]
    normal = surface.get("normal")
    if normal is None:
        raise ValueError("end face has no planar normal")
    normal_vector = [normal["x"], normal["y"], normal["z"]]
    coedges = model.ring_coedges(loop_id)
    groups, _ = _group_teeth(coedges, model, normal_vector)
    all_radius = []
    all_delta = []
    all_side = []
    flank_curves = []
    fillet_curves = []
    tips = []
    tooth_samples = []

    def classify(group: list[str]) -> tuple[list[np.ndarray], float, float]:
        polylines = []
        for coedge_id in group:
            points = model.edge_points(coedge_id, flank_samples)
            polylines.append(_project(points, normal_vector))
        radii = [np.hypot(item[:, 0], item[:, 1]) for item in polylines]
        tip_index = int(np.argmax([float(item.min()) for item in radii]))
        tip = polylines[tip_index]
        tip_radius = float(np.median(np.hypot(tip[:, 0], tip[:, 1])))
        center = _circular_mean(np.arctan2(tip[:, 1], tip[:, 0]))
        return polylines, center, tip_radius

    for tooth_index, group in enumerate(groups):
        polylines, center, tip_radius = classify(group)
        radii = [np.hypot(item[:, 0], item[:, 1]) for item in polylines]
        spans = [float(item.max() - item.min()) for item in radii]
        minimum_radius = min(float(item.min()) for item in radii)
        tip_candidates = [index for index, item in enumerate(radii) if item.min() > 0.999 * tip_radius]
        if len(tip_candidates) != 1:
            tip_candidates = [int(np.argmax([item.min() for item in radii]))]
        tip_index = tip_candidates[0]
        flank_indices = [
            index
            for index, span in enumerate(spans)
            if index != tip_index and span > 0.4 * (tip_radius - minimum_radius)
        ]
        if len(flank_indices) < 2:
            continue
        flank_indices = sorted(flank_indices, key=lambda index: -spans[index])[:2]
        fillet_indices = [
            index
            for index, span in enumerate(spans)
            if index not in flank_indices and index != tip_index and span > 0.2 * module_mm
        ]
        tip = polylines[tip_index]
        tip_radius_measured = float(np.median(np.hypot(tip[:, 0], tip[:, 1])))
        tips.append(tip)
        for index in flank_indices:
            points = polylines[index]
            radius = np.hypot(points[:, 0], points[:, 1])
            delta = _angle_delta(np.arctan2(points[:, 1], points[:, 0]), center)
            all_radius.append(radius)
            all_delta.append(delta)
            all_side.append(np.sign(delta))
            flank_curves.append(points)
        if fillet_indices:
            fillet_index = min(
                fillet_indices,
                key=lambda index: float(np.hypot(*polylines[index][-1])),
            )
            fillet_points = polylines[fillet_index]
            fillet_radii = np.hypot(fillet_points[:, 0], fillet_points[:, 1])
            if fillet_radii[0] > fillet_radii[-1]:
                fillet_points = fillet_points[::-1]
            fillet_curves.append(fillet_points)

        left = None
        right = None
        for index in flank_indices:
            crossing = _radius_crossing(polylines[index], reference_radius_mm)
            if crossing is None:
                continue
            delta = _angle_delta(np.array([crossing]), center)[0]
            if delta < 0.0:
                left = abs(float(delta))
            else:
                right = float(delta)
        thickness_deg = None
        thickness_mm = None
        if left is not None and right is not None:
            thickness_deg = math.degrees(left + right)
            thickness_mm = reference_radius_mm * math.radians(thickness_deg)
        left_start = None
        right_start = None
        for index in flank_indices:
            points = polylines[index]
            radii_here = np.hypot(points[:, 0], points[:, 1])
            start_index = int(np.argmin(radii_here))
            start_radius = float(radii_here[start_index])
            delta = _angle_delta(
                np.array([math.atan2(points[start_index, 1], points[start_index, 0])]), center
            )[0]
            if delta < 0.0:
                left_start = start_radius
            else:
                right_start = start_radius
        tooth_samples.append(
            ToothMeasurement(
                tooth_index=tooth_index,
                center_angle_deg=math.degrees(center),
                tip_radius_mm=tip_radius_measured,
                tip_land_span_deg=_span_deg(tip),
                pitch_thickness_deg=thickness_deg,
                pitch_thickness_mm=thickness_mm,
                left_flank_start_radius_mm=left_start if left_start is not None else float("nan"),
                right_flank_start_radius_mm=right_start if right_start is not None else float("nan"),
            )
        )

    if not all_radius:
        raise ValueError("end face loop has no flank edges")
    radius = np.concatenate(all_radius)
    delta = np.concatenate(all_delta)
    side = np.concatenate(all_side)
    selected = (radius >= 0.0) & (radius < 0.999 * max(tip_radius, float(radius.max())))
    fit = fit_involute(
        radius[selected],
        delta[selected],
        sides=side[selected],
        reference_radius=reference_radius_mm,
        teeth=teeth,
    )

    local_base = []
    for points in flank_curves:
        gradient = np.gradient(points, axis=0)
        tangent = gradient / np.linalg.norm(gradient, axis=1)[:, None]
        radial = points / np.hypot(points[:, 0], points[:, 1])[:, None]
        dot = np.abs(np.sum(tangent * radial, axis=1))
        alpha = np.arccos(np.clip(dot, 0.0, 1.0))
        base = np.hypot(points[:, 0], points[:, 1]) * np.cos(alpha)
        local_base.append(base)
    local_base_values = np.concatenate(local_base)
    local_median = float(np.median(local_base_values))
    local_alpha_deg = math.degrees(math.acos(min(1.0, local_median / reference_radius_mm)))
    local_stderr = float(np.std(local_base_values))

    fillet = None
    if fillet_curves:
        fillet_points = fillet_curves[0]
        fillet_radius = np.hypot(fillet_points[:, 0], fillet_points[:, 1])
        root_radius = float(root_radius_from_points(fillet_points))
        flank_tangent = None
        flank_start = float(fillet_radius[-1])
        fillet_end = fillet_points[-1]
        best_distance = float("inf")
        for candidate in flank_curves:
            candidate_radii = np.hypot(candidate[:, 0], candidate[:, 1])
            start_index = int(np.argmin(candidate_radii))
            distance = float(np.linalg.norm(candidate[start_index] - fillet_end))
            if distance >= best_distance:
                continue
            best_distance = distance
            if start_index == 0:
                flank_tangent = candidate[1] - candidate[0]
            else:
                flank_tangent = candidate[-2] - candidate[-1]
        fillet = measure_fillet(fillet_points, root_radius, flank_tangent, flank_start)
    if fillet is None:
        raise ValueError("end face loop has no root fillet edge")

    tip_points = np.concatenate(tips) if tips else np.empty((0, 2))
    if len(tip_points):
        tip_radii = np.hypot(tip_points[:, 0], tip_points[:, 1])
        tip = CircleFit(
            center=[0.0, 0.0],
            radius_mm=float(np.mean(tip_radii)),
            rms_mm=float(np.std(tip_radii)),
            max_mm=float(np.max(np.abs(tip_radii - np.mean(tip_radii)))),
        )
    else:
        tip = CircleFit([0.0, 0.0], float("nan"), float("nan"), float("nan"))

    undercut = assess_undercut(
        teeth=teeth,
        pressure_angle_deg=fit.pressure_angle_deg,
        base_radius_mm=fit.base_radius_mm,
        reference_radius_mm=reference_radius_mm,
        involute_start_radius_mm=fillet.flank_start_radius_mm,
        module_mm=module_mm,
        junction_kink_deg=fillet.flank_tangent_error_deg,
        root_radius_mm=fillet.root_radius_mm,
    )

    alternatives = _alternative_fits(flank_curves)
    alternatives["involute"] = {
        "rms_mm": fit.rms_tangential_mm,
        "max_mm": fit.max_tangential_mm,
    }
    axis = _canonical_axis(normal_vector)
    station = _station(surface["origin"], axis)
    return EndFaceAnalysis(
        face_id=face["id"],
        loop_id=loop_id,
        teeth=len(tooth_samples),
        center=[0.0, 0.0, 0.0],
        normal=normal_vector,
        station_mm=station,
        involute=fit,
        local_tangent_base_radius_mm=local_median,
        local_tangent_pressure_angle_deg=local_alpha_deg,
        local_tangent_stderr_mm=local_stderr,
        fillet=fillet,
        undercut=undercut,
        tip=tip,
        tooth_samples=tooth_samples,
        alternatives=alternatives,
    )


def _span_deg(points: np.ndarray) -> float:
    angles = np.unwrap(np.arctan2(points[:, 1], points[:, 0]))
    return abs(math.degrees(float(angles[-1] - angles[0])))


def root_radius_from_points(points: np.ndarray) -> float:
    return float(np.min(np.hypot(points[:, 0], points[:, 1])))


def _alternative_fits(flanks: Sequence[np.ndarray]) -> dict[str, dict[str, float]]:
    if not flanks:
        return {}
    points = np.asarray(flanks[0], dtype=float)
    center = points.mean(axis=0)
    shifted = points - center
    _, _, vectors = np.linalg.svd(shifted, full_matrices=False)
    direction = vectors[0]
    normal = np.array([-direction[1], direction[0]])
    line_distance = shifted @ normal
    line_rms = float(np.sqrt(np.mean(line_distance**2)))
    circle = circle_fit(points)
    return {
        "involute": {},
        "straight_line": {
            "rms_mm": line_rms,
            "max_mm": float(np.max(np.abs(line_distance))),
        },
        "circle": {
            "radius_mm": circle.radius_mm,
            "rms_mm": circle.rms_mm,
            "max_mm": circle.max_mm,
        },
    }


def analyze_model(
    path: str | Path,
    teeth: int | None = None,
    module_mm: float | None = None,
    face_width_mm: float | None = None,
    face_ids: Sequence[str] | None = None,
    flank_samples: int = 400,
) -> GearInvoluteReport:
    """Analyze every large end-face loop in a cadmpeg model JSON."""

    model = BrepModel.load(path)
    candidates = model.end_face_loops()
    if face_ids:
        candidates = [item for item in candidates if item[0]["id"] in set(face_ids)]
    if not candidates:
        raise ValueError("no end-face loops with a 6-edge tooth pattern")
    if teeth is None:
        teeth = max(len(model.ring_coedges(loop_id)) for _, loop_id in candidates) // EDGES_PER_TOOTH
    candidates = [
        item
        for item in candidates
        if len(model.ring_coedges(item[1])) == teeth * EDGES_PER_TOOTH
    ]
    if not candidates:
        raise ValueError("no end-face loop matches the tooth count")
    if module_mm is None:
        module_mm = 3.0
    stations = []
    for face, _ in candidates:
        surface = model.surfaces[face["surface"]]
        normal = surface["normal"]
        axis = _canonical_axis([normal["x"], normal["y"], normal["z"]])
        stations.append(_station(surface["origin"], axis))
    if face_width_mm is None:
        face_width_mm = float(max(stations) - min(stations)) if stations else float("nan")
    reference_radius = 0.5 * module_mm * teeth
    analyses = []
    for face, loop_id in candidates:
        try:
            analyses.append(
                analyze_end_face(
                    model,
                    face,
                    loop_id,
                    teeth=teeth,
                    module_mm=module_mm,
                    reference_radius_mm=reference_radius,
                    face_width_mm=face_width_mm,
                    flank_samples=flank_samples,
                )
            )
        except ValueError:
            continue
    if not analyses:
        raise ValueError("no end face produced a complete involute analysis")
    first = analyses[0]
    checks = []
    alpha = first.involute.pressure_angle_deg
    checks.append(
        {
            "name": "pressure_angle",
            "value": alpha,
            "expected": 20.0,
            "unit": "deg",
            "passed": abs(alpha - 20.0) <= 0.1,
        }
    )
    checks.append(
        {
            "name": "base_radius",
            "value": first.involute.base_radius_mm,
            "expected": reference_radius * math.cos(math.radians(20.0)),
            "unit": "mm",
            "passed": abs(first.involute.base_radius_mm - reference_radius * math.cos(math.radians(20.0))) <= 0.02,
        }
    )
    checks.append(
        {
            "name": "involute_residual",
            "value": first.involute.rms_tangential_mm,
            "expected": 0.005,
            "unit": "mm",
            "passed": first.involute.rms_tangential_mm <= 0.005,
        }
    )
    checks.append(
        {
            "name": "root_fillet_not_circular",
            "value": first.fillet.circle.rms_mm,
            "expected": 0.005,
            "unit": "mm",
            "passed": first.fillet.circle.rms_mm > 0.005,
        }
    )
    passed = all(check["passed"] for check in checks)
    notes = [
        "the fit uses the exact end-face boundary carriers; the tip land and root fillet are excluded",
        "the local tangent estimate uses the angle between the profile tangent and the radius vector",
    ]
    notes.extend(first.undercut.notes)
    return GearInvoluteReport(
        source=str(path),
        teeth=teeth,
        module_mm=module_mm,
        face_width_mm=face_width_mm,
        pitch_radius_mm=reference_radius,
        tip_radius_mm=first.tip.radius_mm,
        root_radius_mm=first.fillet.root_radius_mm,
        helix_angle_deg=0.0,
        end_faces=analyses,
        checks=checks,
        passed=passed,
        notes=notes,
    )


def analyze_profile(
    path: str | Path,
    teeth: int | None = None,
    module_mm: float | None = None,
    pressure_angle_deg: float = 20.0,
    use_end: str = "a",
) -> GearInvoluteReport:
    """Analyze an ``nxview gear-profile`` JSON file."""

    data = json.loads(Path(path).read_text(encoding="utf-8"))
    key = {"a": "radii_a", "b": "radii_b", "mid": "radii_mid"}[use_end]
    radii = data[key]
    if teeth is None:
        teeth = int(data["teeth"])
    if module_mm is None:
        module_mm = 2.0 * float(data["tip_radius"]) / (teeth + 2.0)
    pitch = 0.5 * module_mm * teeth
    measured_root = float(np.min(np.asarray(radii, dtype=float)))
    declared_root = float(data["root_radius"])
    root = measured_root
    fit, diagnostics = fit_involute_profile(
        radii,
        teeth=teeth,
        tip_radius=float(data["tip_radius"]),
        root_radius=root,
        reference_radius=pitch,
    )
    checks = [
        {
            "name": "pressure_angle",
            "value": fit.pressure_angle_deg,
            "expected": pressure_angle_deg,
            "unit": "deg",
            "passed": abs(fit.pressure_angle_deg - pressure_angle_deg) <= 0.2,
        },
        {
            "name": "base_radius",
            "value": fit.base_radius_mm,
            "expected": pitch * math.cos(math.radians(pressure_angle_deg)),
            "unit": "mm",
            "passed": abs(fit.base_radius_mm - pitch * math.cos(math.radians(pressure_angle_deg))) <= 0.05,
        },
        {
            "name": "involute_residual",
            "value": fit.rms_tangential_mm,
            "expected": 0.05,
            "unit": "mm",
            "passed": fit.rms_tangential_mm <= 0.05,
        },
    ]
    return GearInvoluteReport(
        source=str(path),
        teeth=teeth,
        module_mm=module_mm,
        face_width_mm=float(data["face_width"]),
        pitch_radius_mm=pitch,
        tip_radius_mm=float(data["tip_radius"]),
        root_radius_mm=root,
        helix_angle_deg=float(data.get("helix_angle_deg", 0.0)),
        end_faces=[],
        checks=checks,
        passed=all(check["passed"] for check in checks),
        notes=[
            "profile fit uses the {}-bin maximum-radius envelope".format(data.get("bins", len(radii))),
            "bin quantization limits the base-radius fit; wider bins increase the uncertainty",
            "profile sampling diagnostics: {}".format(json.dumps(diagnostics, ensure_ascii=False)),
            "measured tooth-space root radius {:.6f} mm; nxview gear root radius {:.6f} mm".format(
                root, declared_root
            ),
        ],
    )


def write_report(report: GearInvoluteReport, path: str | Path) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report.to_dict(), ensure_ascii=False, indent=2), encoding="utf-8")


def write_markdown(report: GearInvoluteReport, path: str | Path) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "# Gear involute validation",
        "",
        f"Source: `{report.source}`",
        "",
        "| Item | Value |",
        "| --- | ---: |",
        f"| Teeth | {report.teeth} |",
        f"| Module | {report.module_mm:.6f} mm |",
        f"| Face width | {report.face_width_mm:.6f} mm |",
        f"| Pitch radius | {report.pitch_radius_mm:.6f} mm |",
        f"| Tip radius | {report.tip_radius_mm:.6f} mm |",
        f"| Root radius | {report.root_radius_mm:.6f} mm |",
        f"| Helix angle | {report.helix_angle_deg:.3f} deg |",
        f"| Verdict | {'PASS' if report.passed else 'FAIL'} |",
        "",
        "## End faces",
        "",
    ]
    for face in report.end_faces:
        lines.extend(
            [
                f"### {face.face_id}",
                "",
                "| Item | Value |",
                "| --- | ---: |",
                f"| Base radius | {face.involute.base_radius_mm:.6f} mm |",
                f"| Pressure angle | {face.involute.pressure_angle_deg:.6f} deg |",
                f"| Local tangent base radius | {face.local_tangent_base_radius_mm:.6f} mm |",
                f"| Local tangent pressure angle | {face.local_tangent_pressure_angle_deg:.6f} deg |",
                f"| Profile shift | {face.involute.profile_shift if face.involute.profile_shift is not None else float('nan'):.6f} |",
                f"| Involute RMS | {face.involute.rms_tangential_mm * 1000.0:.3f} um |",
                f"| Involute max | {face.involute.max_tangential_mm * 1000.0:.3f} um |",
                f"| Root fillet circle radius | {face.fillet.circle.radius_mm:.6f} mm |",
                f"| Root fillet circle RMS | {face.fillet.circle.rms_mm * 1000.0:.3f} um |",
                f"| Fillet curvature at root | {face.fillet.curvature_radius_root_mm:.6f} mm |",
                f"| Fillet curvature at flank | {face.fillet.curvature_radius_flank_mm:.6f} mm |",
                f"| Fillet/flank kink | {face.fillet.flank_tangent_error_deg:.3f} deg |",
                f"| Undercut limit | {face.undercut.minimum_teeth_without_undercut:.3f} |",
                f"| Visible undercut | {'YES' if face.undercut.visible_undercut else 'no'} |",
                "",
            ]
        )
    lines.extend(["## Checks", "", "| Check | Value | Unit | Expected | Result |", "| --- | ---: | --- | ---: | --- |"])
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


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Fit involute gear flanks")
    parser.add_argument("--model", action="append", default=[], help="cadmpeg model JSON")
    parser.add_argument("--profile", action="append", default=[], help="nxview gear-profile JSON")
    parser.add_argument("--teeth", type=int)
    parser.add_argument("--module", type=float)
    parser.add_argument("--output", help="JSON report path")
    parser.add_argument("--markdown", help="Markdown report path")
    arguments = parser.parse_args(argv)
    reports = []
    for path in arguments.model:
        reports.append(analyze_model(path, teeth=arguments.teeth, module_mm=arguments.module))
    for path in arguments.profile:
        reports.append(analyze_profile(path, teeth=arguments.teeth, module_mm=arguments.module))
    if not reports:
        parser.error("give at least one --model or --profile")
    payload = [report.to_dict() for report in reports]
    if arguments.output:
        Path(arguments.output).write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    if arguments.markdown:
        for index, report in enumerate(reports):
            target = Path(arguments.markdown)
            if index:
                target = target.with_name(target.stem + f"_{index}" + target.suffix)
            write_markdown(report, target)
    print(json.dumps(payload[0], ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
