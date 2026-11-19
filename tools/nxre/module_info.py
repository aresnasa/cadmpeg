"""Inventory the NX modules represented in a part, simulation, or FE file.

The scan is deliberately format-level: it reports which module marker strings
and container streams are present, not whether the codec has a semantic
decoder for them.  It is the first pass of the NX 2606 module coverage matrix.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

MARKERS: dict[str, tuple[bytes, ...]] = {
    "modeling": (b"Parasolid", b"UGS::BODY", b"UGS::FACE", b"UGS::EDGE"),
    "assembly": (
        b"ExternalReferences",
        b"UGS::Occurrence",
        b"UGS::RM_part",
    ),
    "motion": (
        b"Mdm mechanism",
        b"Mdm joint",
        b"Mdm link",
        b"Motion",
        b"RMotion",
    ),
    "cae": (b"CaeSim", b"CaeFem", b"Cae::", b"FEM", b"Simulation"),
    "cam": (
        b"Cam::Operation",
        b"Cam::Tool",
        b"Cam::CutterDisplay",
        b"Tool display",
    ),
    "drafting": (b"DWG_", b"DWG_view", b"VIEW_base", b"LAYOUT_", b"Sheet", b"Drawing"),
    "pmi": (b"PMI", b"Dimension", b"Annotation"),
    "jt": (b"DisplayJT",),
    "expressions": (b"ESS", b"Expression"),
    "attributes": (b"part/attrs", b"ES_part_attribute"),
}


def _container_kind(data: bytes) -> str:
    if data.startswith(b"SPLMSSTR"):
        return "splmsstr"
    if data.startswith(b"\xd0\xcf\x11\xe0\xa1\xb1\x1a\xe1"):
        return "cfb"
    return "unknown"


def _streams(path: Path) -> list[dict[str, Any]]:
    try:
        import olefile
    except ImportError:
        return []
    try:
        with olefile.OleFileIO(str(path)) as ole:
            result = []
            for entry in ole.listdir(streams=True, storages=False):
                name = "/".join(entry)
                try:
                    size = ole.get_size(entry)
                except Exception:
                    size = None
                result.append({"name": name, "size": size})
            return result
    except Exception:
        return []


def inspect_modules(path: str | Path) -> dict[str, Any]:
    path = Path(path)
    data = path.read_bytes()
    streams = _streams(path)
    names = {entry["name"] for entry in streams}
    structural = {
        "display_jt": any("DisplayJT" in name for name in names),
        "external_references": any(
            name.endswith("ExternalReferences") for name in names
        ),
        "part_attributes": any(name.endswith("part/attrs") for name in names),
        "cae_streams": sorted(name for name in names if "Cae" in name or "FEM" in name),
        "motion_streams": sorted(
            name for name in names if "Mdm" in name or "motion" in name.lower()
        ),
        "cam_streams": sorted(name for name in names if "Cam" in name or "CAM" in name),
        "drafting_streams": sorted(
            name for name in names if "Draft" in name or "View" in name
        ),
    }
    result: dict[str, Any] = {
        "path": str(path),
        "size": len(data),
        "container": _container_kind(data),
        "streams": streams,
        "modules": {},
        "structural": structural,
    }
    for module, markers in MARKERS.items():
        counts = {}
        for marker in markers:
            ascii_count = data.count(marker)
            utf16 = marker.decode("ascii", "ignore").encode("utf-16-le")
            utf16_count = data.count(utf16) if utf16 else 0
            if ascii_count or utf16_count:
                counts[marker.decode("ascii", "ignore")] = {
                    "ascii": ascii_count,
                    "utf16": utf16_count,
                }
        total = sum(value["ascii"] + value["utf16"] for value in counts.values())
        result["modules"][module] = {
            "present": False,
            "confidence": "none",
            "marker_total": total,
            "markers": counts,
        }

    modules = result["modules"]
    rules = {
        "assembly": structural["external_references"],
        "attributes": structural["part_attributes"],
        "jt": structural["display_jt"],
    }
    for module, present in rules.items():
        modules[module]["present"] = present
        modules[module]["confidence"] = "structural" if present else "none"

    # Module-specific markers.  Common NX boilerplate (Cam::CutterDisplay,
    # DWG_*, PMI QAF metadata) appears in nearly every part, so those markers
    # alone must not claim a module.
    cae_markers = modules["cae"]["markers"]
    cae_marker_count = lambda name: (
        cae_markers.get(name, {}).get("ascii", 0)
        + cae_markers.get(name, {}).get("utf16", 0)
    )
    cae_present = cae_marker_count("CaeSim") > 0 or cae_marker_count("CaeFem") > 10
    modules["cae"]["present"] = cae_present
    modules["cae"]["confidence"] = "marker" if cae_present else "none"

    motion_markers = modules["motion"]["markers"]
    motion_count = sum(
        value["ascii"] + value["utf16"]
        for name, value in motion_markers.items()
        if name in ("Mdm mechanism", "Mdm joint", "Mdm link", "RMotion")
    )
    if not motion_count and path.suffix.lower() == ".sim":
        motion_count = sum(
            value["ascii"] + value["utf16"]
            for name, value in motion_markers.items()
            if name in ("Motion", "motion_")
        )
    motion_present = motion_count > 10 and not cae_present
    modules["motion"]["present"] = motion_present
    modules["motion"]["confidence"] = "marker" if motion_present else "none"

    cam_markers = modules["cam"]["markers"]
    cam_present = any(
        name in cam_markers
        for name in ("Cam::Operation", "Cam::Tool", "Tool display", "NC_PROGRAM")
    )
    modules["cam"]["present"] = cam_present
    modules["cam"]["confidence"] = "marker" if cam_present else "none"

    drafting_markers = modules["drafting"]["markers"]

    def marker_count(*names: str) -> int:
        return sum(
            value["ascii"] + value["utf16"]
            for name, value in drafting_markers.items()
            if name in names
        )

    drafting_present = (
        marker_count("DWG_view") > 0
        or (
            marker_count("Drawing") >= 20
            and marker_count("Sheet") >= 10
        )
    )
    modules["drafting"]["present"] = drafting_present
    modules["drafting"]["confidence"] = "marker" if drafting_present else "none"

    pmi_markers = modules["pmi"]["markers"]
    pmi_present = any(
        name in pmi_markers for name in ("PMI_", "PMI annotation")
    )
    modules["pmi"]["present"] = pmi_present
    modules["pmi"]["confidence"] = "marker" if pmi_present else "none"

    modeling_markers = modules["modeling"]["markers"]
    modeling_present = any(
        name in modeling_markers for name in ("Parasolid", "UGS::BODY", "UGS::FACE")
    )
    modules["modeling"]["present"] = modeling_present
    modules["modeling"]["confidence"] = "marker" if modeling_present else "none"
    return result


def write_markdown(report: dict[str, Any], path: str | Path) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "# NX module inventory",
        "",
        f"- file: `{report['path']}`",
        f"- container: `{report['container']}`",
        f"- size: {report['size']} bytes",
        f"- streams: {len(report['streams'])}",
        "",
        "| module | present | confidence | markers |",
        "| --- | --- | --- | --- |",
    ]
    for module, value in report["modules"].items():
        summary = ", ".join(
            f"`{marker}`×{counts['ascii'] + counts['utf16']}"
            for marker, counts in value["markers"].items()
        )
        lines.append(
            f"| {module} | {value['present']} | {value['confidence']} | {summary} |"
        )
    lines.append("")
    lines.append("## Structural signals")
    lines.append("")
    for key, value in report["structural"].items():
        lines.append(f"- `{key}`: {value}")
    lines.append("")
    path.write_text("\n".join(lines), encoding="utf-8")
