"""Unified NX Motion module report for a `.sim` scenario.

The `.sim` file is the Motion container.  Its rigid-body model lives in a
companion RecurDyn `.rmd`, the solver log in `.msg`, and the result channels in
`.rplt`/`.rad`.  This module discovers those companions and presents the same
objects the NX Motion navigator shows: bodies, markers, joints, drivers, gear
couplers, the active solution, and the XY result channels.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

from .motion import parse_msg, parse_rmd
from .motion_minp import (
    parse_minf,
    parse_minp,
    parse_mres,
    parse_mres_header,
    write_mres_csv,
)
from .motion_results import (
    inspect_sim,
    parse_rad,
    parse_rplt,
    validate_gear_ratio,
)

COMPANION_EXTENSIONS = ("rmd", "msg", "rplt", "rad", "mres", "minf", "minp")


def _companions(sim: Path) -> dict[str, list[str]]:
    found: dict[str, list[str]] = {}
    for extension in COMPANION_EXTENSIONS:
        matches = sorted(
            str(path)
            for path in sim.parent.glob(f"{sim.stem}*.{extension}")
            if path.is_file()
        )
        found[extension] = matches
    return found


def _part_attributes(sim: Path) -> list[dict[str, str]]:
    try:
        import olefile
    except ImportError:
        return []
    try:
        with olefile.OleFileIO(str(sim)) as ole:
            if not ole.exists("part/attrs"):
                return []
            payload = ole.openstream("part/attrs").read()
    except Exception:
        return []
    try:
        root = ET.fromstring(payload)
    except ET.ParseError:
        return []
    result = []
    for item in root.findall("Attribute"):
        result.append(
            {
                "title": item.get("title", ""),
                "value": item.get("value", ""),
                "owner": item.get("owner", ""),
            }
        )
    return result


def build_sim_report(
    sim_path: str | Path,
    driver: str = "J002",
    driven: str = "J003",
    driver_rate_deg_per_s: float = 180.0,
) -> dict[str, Any]:
    """Build a Motion-module report for one `.sim` scenario."""
    sim = Path(sim_path)
    if not sim.is_file():
        raise FileNotFoundError(sim)
    companions = _companions(sim)
    report: dict[str, Any] = {
        "sim": {
            "path": str(sim),
            "size": sim.stat().st_size,
            "inventory": inspect_sim(sim),
            "part_attributes": _part_attributes(sim),
        },
        "companions": companions,
        "model": None,
        "solver": None,
        "results": None,
        "validation": None,
        "animation": None,
        "module": {
            "name": "Motion",
            "state": "container-only",
            "bodies": 0,
            "markers": 0,
            "joints": 0,
            "drivers": 0,
            "gears": 0,
            "channels": 0,
            "samples": 0,
            "gear_ratio": None,
            "passed": None,
            "results_format": None,
            "results_file": None,
        },
        "results_binary": None,
    }

    if companions["rmd"]:
        model = parse_rmd(companions["rmd"][0])
        report["model"] = model.to_dict()
        report["model"]["format"] = "RecurDyn RMD (NX10)"
    elif companions["minp"]:
        model = parse_minp(companions["minp"][0])
        report["model"] = model.to_dict()
        report["model"]["format"] = "RecurDyn MINP (NX 2606)"
    else:
        model = None
    if model is not None:
        report["module"].update(
            {
                "state": "model-decoded",
                "bodies": len(model.bodies),
                "markers": len(model.markers),
                "joints": len(model.joints),
                "drivers": len(model.drivers),
                "gears": len(model.gears),
            }
        )
    if companions["msg"]:
        report["solver"] = parse_msg(companions["msg"][0])
    elif companions["minf"]:
        report["solver"] = parse_minf(companions["minf"][0])
    if companions["rplt"]:
        results = parse_rplt(companions["rplt"][0])
        validation = validate_gear_ratio(
            results, driver, driven, driver_rate_deg_per_s
        )
        report["results"] = results.to_dict(include_values=False)
        report["validation"] = validation
        report["module"].update(
            {
                "state": "results-decoded",
                "channels": len(results.channels),
                "samples": len(results.times),
                "gear_ratio": validation["position_ratio_median"],
                "passed": validation["passed"],
                "results_format": "RPLT (NX10)",
                "results_file": companions["rplt"][0],
            }
        )
    elif companions["mres"]:
        mres = parse_mres(companions["mres"][0])
        report["results_binary"] = parse_mres_header(companions["mres"][0])
        report["results_mres"] = mres
        report["results"] = {
            "source": {
                "model": Path(mres["path"]).stem,
                "format": mres["format"],
            },
            "summary": {
                "channel_count": mres["summary"]["channel_count"],
                "sample_count": mres["summary"]["sample_count"],
            },
            "time": {
                "range": [mres["times"][0], mres["times"][-1]]
                if mres["times"]
                else None,
            },
            "channels": [
                {
                    "path": channel["path"],
                    "unit": channel["unit"],
                    "derived": False,
                }
                for channel in mres["channels"]
            ],
        }
        report["module"].update(
            {
                "state": "results-decoded-partial",
                "channels": mres["summary"]["channel_count"],
                "samples": mres["summary"]["sample_count"],
                "gear_ratio": mres["summary"]["gear_ratio"],
                "passed": (
                    abs(mres["summary"]["gear_ratio"] + 1.0) < 1.0e-6
                    if mres["summary"]["gear_ratio"] is not None
                    else None
                ),
                "results_format": "MRES (NX 2606, verified subset)",
                "results_file": companions["mres"][0],
            }
        )
    if companions["rad"]:
        report["animation"] = parse_rad(companions["rad"][0])
    return report


def write_markdown(report: dict[str, Any], path: str | Path) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    sim = report["sim"]["path"]
    module = report["module"]
    lines = [
        "# NX Motion module report",
        "",
        f"- `.sim`: `{sim}`",
        f"- state: **{module['state']}**",
        f"- bodies/joints/drivers/gears: "
        f"{module['bodies']} / {module['joints']} / "
        f"{module['drivers']} / {module['gears']}",
        f"- channels/samples: {module['channels']} / {module['samples']}",
        f"- result format: {module.get('results_format')}",
        f"- gear ratio: {module['gear_ratio']}",
        f"- validation passed: {module['passed']}",
        "",
        "## Motion navigator",
        "",
    ]
    model = report.get("model")
    if model:
        lines.append("### Bodies")
        lines.append("")
        for body in model["bodies"]:
            lines.append(
                f"- `{body['name']}` id={body['id']} "
                f"mass={body['mass']} inertia={body['inertia']}"
            )
        lines.append("")
        lines.append("### Joints")
        lines.append("")
        for joint in model["joints"]:
            lines.append(
                f"- `{joint['name']}` {joint['kind']} "
                f"{joint['i_marker']}->{joint['j_marker']}"
            )
        lines.append("")
        lines.append("### Drivers")
        lines.append("")
        for driver in model["drivers"]:
            lines.append(
                f"- `{driver['name']}` joint={driver['joint']} "
                f"function=`{driver['function']}`"
            )
        lines.append("")
        lines.append("### Gear couplers")
        lines.append("")
        for gear in model["gears"]:
            lines.append(f"- `{gear['name']}` joints={gear['joints']}")
        lines.append("")
    else:
        lines.append("_No companion `.rmd` was found._")
        lines.append("")

    solver = report.get("solver")
    if solver:
        lines.extend(["## Solver", ""])
        if "parts" in solver and "joints" in solver:
            lines.extend(
                [
                    f"- version: {solver.get('solver_version')}",
                    f"- parts: {solver.get('parts')}",
                    f"- joints: {solver.get('joints')}",
                ]
            )
        else:
            lines.append(f"- log: {solver.get('format')}")
            for key in ("solver_version", "interface", "license", "hostname"):
                if solver.get(key):
                    lines.append(f"- {key}: {solver[key]}")
            for header_line in solver.get("header", [])[:4]:
                lines.append(f"- `{header_line}`")
        lines.append("")

    results = report.get("results")
    if results:
        lines.extend(
            [
                "## XY results",
                "",
                f"- model: {results['source']['model']}",
                f"- channels: {results['summary']['channel_count']}",
                f"- samples: {results['summary']['sample_count']}",
                f"- time: {results['time']['range']} s",
                "",
                "| channel | unit | derived |",
                "| --- | --- | --- |",
            ]
        )
        for channel in results["channels"][:40]:
            lines.append(
                f"| `{channel['path']}` | {channel['unit']} | {channel['derived']} |"
            )
        if len(results["channels"]) > 40:
            lines.append(f"| … {len(results['channels']) - 40} more | | |")
        lines.append("")

    binary = report.get("results_binary")
    if binary:
        lines.extend(
            [
                "## XY results (NX 2606 binary)",
                "",
                f"- format: {binary['format']}",
                f"- input model: `{binary.get('input_model')}`",
                f"- timestamp: {binary.get('timestamp')}",
                f"- solver mode: {binary.get('solver_mode')}",
                "- channel decoding: {}".format(
                    "partial (verified subset; remaining slots pending)"
                    if report.get("results_mres")
                    else "pending (MRES binary table)"
                ),
                "",
            ]
        )

    lines.extend(["## Companion files", ""])
    for extension, files in report["companions"].items():
        for file in files:
            lines.append(f"- `{extension}`: `{file}`")
    lines.append("")
    path.write_text("\n".join(lines), encoding="utf-8")
