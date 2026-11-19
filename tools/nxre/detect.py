"""Detect the gear-pump model from decoded NX part geometry."""

from __future__ import annotations

import json
import os
import subprocess
from pathlib import Path
from typing import Any

from .pump import GearPumpModel, default_model, model_to_dict


def nxview_path() -> Path:
    return Path(os.environ.get("NXVIEW", r"C:\RE\build-cadmpeg\release\nxview.exe"))


def _run_json(command: list[str]) -> Any:
    result = subprocess.run(command, capture_output=True)
    text = result.stdout.decode("utf-8", "replace")
    if result.returncode != 0:
        error = result.stderr.decode("utf-8", "replace").strip()
        raise RuntimeError(error or f"{command[0]} failed")
    decoder = json.JSONDecoder()
    for start, character in enumerate(text):
        if character not in "[{":
            continue
        try:
            value, end = decoder.raw_decode(text[start:])
        except json.JSONDecodeError:
            continue
        if not text[start + end :].strip():
            return value
    raise RuntimeError("engine did not return JSON")


def detect_gear(part: str | Path, nxview: str | Path | None = None) -> dict[str, Any]:
    executable = str(nxview or nxview_path())
    report = _run_json([executable, "gear", str(part)])
    required = ("teeth", "tip_radius", "root_radius", "face_width", "axis", "center")
    missing = [key for key in required if key not in report]
    if missing:
        raise RuntimeError(f"gear report is missing {missing}")
    return report


def detect_body_bores(
    body: str | Path,
    tip_radius: float,
    center_distance: float,
    nxview: str | Path | None = None,
) -> dict[str, Any] | None:
    """Find a pair of housing bores with the gear tip radius and spacing."""
    executable = str(nxview or nxview_path())
    surfaces = _run_json([executable, "surfaces", str(body)])
    candidates = []
    for surface in surfaces:
        geometry = surface.get("geometry", {})
        if geometry.get("kind") != "cylinder":
            continue
        radius = float(geometry.get("radius", 0.0))
        if abs(radius - tip_radius) > max(0.2, 0.03 * tip_radius):
            continue
        origin = geometry["origin"]
        axis = geometry["axis"]
        candidates.append(
            {
                "radius": radius,
                "origin": [origin["x"], origin["y"], origin["z"]],
                "axis": [axis["x"], axis["y"], axis["z"]],
            }
        )
    best = None
    best_error = float("inf")
    for index, first in enumerate(candidates):
        for second in candidates[index + 1 :]:
            dot = abs(sum(a * b for a, b in zip(first["axis"], second["axis"])))
            if dot < 0.999:
                continue
            delta = [b - a for a, b in zip(first["origin"], second["origin"])]
            axis = first["axis"]
            along = sum(value * direction for value, direction in zip(delta, axis))
            perpendicular = [value - along * direction for value, direction in zip(delta, axis)]
            spacing = sum(value * value for value in perpendicular) ** 0.5
            error = abs(spacing - center_distance)
            if error < best_error:
                best_error = error
                best = {
                    "radius": first["radius"],
                    "axis": axis,
                    "centers": [first["origin"], second["origin"]],
                    "spacing": spacing,
                    "spacing_error": error,
                }
    if best is None or best_error > 0.5:
        return None
    return best


def detect_model(
    driving_gear: str | Path,
    driven_gear: str | Path,
    body: str | Path | None = None,
    nxview: str | Path | None = None,
) -> GearPumpModel:
    driving = detect_gear(driving_gear, nxview)
    driven = detect_gear(driven_gear, nxview)
    if driving["teeth"] != driven["teeth"]:
        raise RuntimeError(
            f"tooth counts differ: {driving['teeth']} and {driven['teeth']}"
        )
    tip = 0.5 * (driving["tip_radius"] + driven["tip_radius"])
    face_width = 0.5 * (driving["face_width"] + driven["face_width"])
    if abs(driving["tip_radius"] - driven["tip_radius"]) > 0.1:
        raise RuntimeError("tip radii differ by more than 0.1 mm")
    if abs(driving["face_width"] - driven["face_width"]) > 0.1:
        raise RuntimeError("face widths differ by more than 0.1 mm")
    teeth = int(driving["teeth"])
    module = 2.0 * tip / (teeth + 2.0)
    pitch = 0.5 * module * teeth
    root = pitch - 1.25 * module
    center_distance = 2.0 * pitch

    model = default_model()
    model.gear.teeth = teeth
    model.gear.module_mm = module
    model.gear.face_width_mm = face_width
    model.gear.pitch_radius_mm = pitch
    model.gear.tip_radius_mm = tip
    model.gear.root_radius_mm = root
    model.gear.center_distance_mm = center_distance
    model.provenance = {
        "corpus": str(Path(driving_gear).parent),
        "driving_gear": str(driving_gear),
        "driven_gear": str(driven_gear),
        "body": str(body) if body else None,
        "measured": {"driving": driving, "driven": driven},
        "method": (
            "nxview gear recovered the periodic tooth profile from the B-rep "
            "tessellation; module and pitch radius follow the measured tip "
            "radius and tooth count."
        ),
        "limitations": default_model().provenance["limitations"],
    }
    if body is not None:
        bores = detect_body_bores(body, tip, center_distance, nxview)
        model.provenance["housing_bores"] = bores
        if bores is None:
            raise RuntimeError(
                "housing has no bore pair matching the gear tip radius and "
                "center distance"
            )
    model.validate()
    return model


def write_model(model: GearPumpModel, path: str | Path) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(model_to_dict(model), ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
