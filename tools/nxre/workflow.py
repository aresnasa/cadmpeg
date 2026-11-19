"""End-to-end gear-pump workflow: catalog, detect, relink, simulate, validate."""

from __future__ import annotations

import json
import os
import shutil
import subprocess
import tempfile
from pathlib import Path
from typing import Any

from .catalog import build_catalog, write_catalog
from .detect import detect_model, nxview_path, write_model
from .gear_pair import (
    GearProfile,
    validate_pair,
    write_markdown as write_pair_markdown,
    write_plot as write_pair_plot,
    write_report as write_pair_report,
)
from .nx_env import ug_inspect_path
from .pump import (
    load_conditions,
    results_to_json,
    simulate,
    write_csv,
    write_markdown,
)


def _run(command: list[str]) -> tuple[int, str]:
    result = subprocess.run(command, capture_output=True)
    text = (result.stdout + result.stderr).decode("utf-8", "replace")
    return result.returncode, text


def _scan(path: Path) -> dict[str, Any]:
    ug = ug_inspect_path()
    if not Path(ug).is_file():
        return {"available": False, "ok": None, "output": "ug_inspect not found"}
    # ug_inspect opens ANSI paths; copy a non-ASCII assembly to an ASCII name.
    temporary = None
    target = path
    if not str(path).isascii():
        handle, name = tempfile.mkstemp(suffix=".prt", prefix="nxre_scan_")
        os.close(handle)
        temporary = Path(name)
        shutil.copy2(path, temporary)
        target = temporary
    try:
        code, text = _run([ug, "-scan", str(target)])
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
    return {
        "available": True,
        "ok": code == 0 and "No corruptions identified" in text,
        "output": text.strip(),
    }


def run_workflow(
    corpus: str | Path,
    workdir: str | Path,
    nxview: str | Path | None = None,
    assembly_name: str = "齿轮泵装配.prt",
    conditions_path: str | Path | None = None,
    edit_attributes: bool = True,
) -> dict[str, Any]:
    corpus = Path(corpus)
    workdir = Path(workdir)
    parts_dir = workdir / "parts"
    parts_dir.mkdir(parents=True, exist_ok=True)
    executable = str(nxview or nxview_path())

    catalog = build_catalog(corpus, executable)
    write_catalog(
        catalog,
        workdir / "gearpump_catalog.json",
        workdir / "gearpump_catalog.md",
    )

    driving = next(
        part for part in catalog["parts"] if part["role"] == "driving gear shaft"
    )
    driven = next(
        part for part in catalog["parts"] if part["role"] == "driven gear shaft"
    )
    housing = next(part for part in catalog["parts"] if part["role"] == "housing")
    model = detect_model(
        corpus / driving["file"],
        corpus / driven["file"],
        corpus / housing["file"],
        executable,
    )
    write_model(model, workdir / "gearpump_model.json")

    profiles = {}
    for role in ("driving gear shaft", "driven gear shaft"):
        part = next(part for part in catalog["parts"] if part["role"] == role)
        output = workdir / f"{role.replace(' ', '_')}_profile.json"
        code, text = _run(
            [
                executable,
                "gear-profile",
                str(corpus / part["file"]),
                "1440",
            ]
        )
        if code != 0:
            raise RuntimeError(text)
        output.write_text(text.strip(), encoding="utf-8")
        profiles[role] = output
    driving_profile = GearProfile.load(profiles["driving gear shaft"])
    driven_profile = GearProfile.load(profiles["driven gear shaft"])
    pair = validate_pair(
        driving_profile,
        driven_profile,
        model.gear.module_mm,
        model.gear.pressure_angle_deg,
        model.gear.center_distance_mm,
        720,
    )
    write_pair_report(pair, workdir / "gearpump_pair.json")
    write_pair_markdown(pair, workdir / "gearpump_pair.md")
    write_pair_plot(
        pair,
        driving_profile,
        driven_profile,
        workdir / "gearpump_pair.png",
    )

    edits = {
        "10泵体.prt": {
            "DB_PART_DESC": "GEAR PUMP BODY - RE WORKFLOW",
            "DB_PART_NO": "GP-10",
        },
        "9泵盖.prt": {
            "DB_PART_DESC": "GEAR PUMP COVER - RE WORKFLOW",
            "DB_PART_NO": "GP-9",
        },
    }
    edited = []
    for part in catalog["parts"]:
        name = part["file"]
        source = corpus / name
        destination = parts_dir / name
        if edit_attributes and name in edits:
            edit_path = workdir / (name + ".edits.json")
            edit_path.write_text(json.dumps(edits[name], ensure_ascii=False), encoding="utf-8")
            code, text = _run(
                [executable, "attr", "apply", str(source), str(destination), str(edit_path)]
            )
            if code != 0:
                raise RuntimeError(text)
            edited.append({"file": name, "attributes": edits[name], "scan": _scan(destination)})
        else:
            shutil.copy2(source, destination)

    assemblies = []
    assembly_files = [assembly_name] + [
        part["file"]
        for part in catalog["parts"]
        if part["role"] == "assembly" and part["file"] != assembly_name
    ]
    for name in assembly_files:
        assembly = corpus / name
        relinked = workdir / name
        code, relink_output = _run(
            [
                executable,
                "assembly",
                "relink",
                str(assembly),
                str(relinked),
                str(parts_dir),
            ]
        )
        if code != 0:
            raise RuntimeError(relink_output)
        assemblies.append(
            {
                "source": str(assembly),
                "assembly": str(relinked),
                "output": relink_output.strip(),
                "scan": _scan(relinked),
            }
        )
    scan = assemblies[0]["scan"]

    conditions = load_conditions(conditions_path)
    results = [simulate(model, condition) for condition in conditions]
    (workdir / "gearpump_sim.json").write_text(
        json.dumps(
            results_to_json(model, results),
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )
    write_csv(results, workdir / "gearpump_sim.csv")
    write_markdown(model, results, workdir / "gearpump_sim.md")

    report = {
        "corpus": str(corpus),
        "workdir": str(workdir),
        "catalog": catalog["summary"],
        "model": {
            "teeth": model.gear.teeth,
            "module_mm": model.gear.module_mm,
            "face_width_mm": model.gear.face_width_mm,
            "center_distance_mm": model.gear.center_distance_mm,
            "displacement_cc": model.gear.displacement_cc(),
        },
        "gear_pair": {
            "passed": pair.passed,
            "mounting_clearance_mm": pair.sweep.mounting_clearance_mm,
            "min_clearance_mm": pair.sweep.min_clearance_mm,
            "contact_ratio": pair.analytical_contact_ratio,
            "tooth_thickness_backlash_mm": pair.tooth_thickness.circular_backlash_mm,
            "undercut_risk": pair.undercut_risk,
        },
        "parts_directory": str(parts_dir),
        "edited_parts": edited,
        "relink": {
            "assemblies": assemblies,
            "scan": scan,
        },
        "conditions": {
            "count": len(results),
            "passed": sum(1 for result in results if result.passed),
            "failed": sum(1 for result in results if not result.passed),
        },
    }
    (workdir / "gearpump_workflow.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    return report
