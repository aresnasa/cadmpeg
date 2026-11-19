"""Inventory NX module OM classes through `ug_inspect -full`."""

from __future__ import annotations

import json
import os
import re
import subprocess
from collections import defaultdict
from pathlib import Path
from typing import Any

DEFAULT_UGINSPECT = (
    r"C:\Program Files\Siemens\Designcenter2606\NXBIN\ug_inspect.exe"
)


def _decode(data: bytes) -> str:
    for encoding in ("utf-8", "gbk", "utf-16"):
        try:
            return data.decode(encoding)
        except UnicodeDecodeError:
            continue
    return data.decode("utf-8", "replace")


def _class_line(line: str) -> tuple[str, int] | None:
    parts = line.split()
    if len(parts) < 2 or not parts[0].startswith("UGS::"):
        return None
    try:
        count = int(parts[1])
    except ValueError:
        return None
    return parts[0], count


def module_of(class_name: str) -> str:
    if "CaeFem" in class_name or "CaeSim" in class_name:
        return "cae"
    if "Cam::" in class_name or class_name.startswith("UGS::CAM"):
        return "cam"
    if "DWG" in class_name or "VIEW" in class_name or "LAYOUT" in class_name:
        return "drafting"
    if "PMI" in class_name or "Annot" in class_name:
        return "pmi"
    if "Mdm" in class_name or "Motion" in class_name:
        return "motion"
    if "Part" in class_name or "Body" in class_name or "Face" in class_name:
        return "core"
    return "other"


def inspect_classes(
    path: str | Path, uginspect: str | Path | None = None
) -> dict[str, Any]:
    path = Path(path)
    executable = Path(
        uginspect
        or os.environ.get("NX_UG_INSPECT")
        or DEFAULT_UGINSPECT
    )
    result = {
        "path": str(path),
        "size": path.stat().st_size,
        "uginspect": str(executable),
        "release": None,
        "om_format": None,
        "object_count": None,
        "classes": [],
        "modules": {},
        "error": None,
    }
    if not executable.is_file():
        result["error"] = "ug_inspect not found"
        return result
    completed = subprocess.run(
        [str(executable), "-full", str(path)], capture_output=True
    )
    text = _decode(completed.stdout) + _decode(completed.stderr)
    if completed.returncode != 0:
        result["error"] = f"ug_inspect exit {completed.returncode}"
    for line in text.splitlines():
        stripped = line.strip()
        match = re.match(r"Release:\s*(.+)$", stripped)
        if match:
            result["release"] = match.group(1).strip()
        match = re.match(r"OM file format version:\s*(.+)$", stripped)
        if match:
            result["om_format"] = match.group(1).strip()
        match = re.match(r"Number of objects:\s*(\d+)", stripped)
        if match:
            result["object_count"] = max(
                result.get("object_count") or 0, int(match.group(1))
            )
        entry = _class_line(line)
        if entry:
            result["classes"].append({"name": entry[0], "count": entry[1]})

    grouped: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for entry in result["classes"]:
        grouped[module_of(entry["name"])].append(entry)
    result["modules"] = {
        module: {
            "class_count": len(entries),
            "object_count": sum(entry["count"] for entry in entries),
            "classes": sorted(entries, key=lambda entry: entry["count"], reverse=True),
        }
        for module, entries in sorted(grouped.items())
    }
    return result


def write_markdown(report: dict[str, Any], path: str | Path) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "# NX module class inventory",
        "",
        f"- file: `{report['path']}`",
        f"- release: {report.get('release')}",
        f"- OM format: {report.get('om_format')}",
        f"- objects: {report.get('object_count')}",
        f"- classes: {len(report.get('classes', []))}",
    ]
    if report.get("error"):
        lines.append(f"- error: {report['error']}")
    lines.append("")
    for module, value in report["modules"].items():
        lines.append(f"## {module}")
        lines.append("")
        lines.append(f"classes: {value['class_count']} / objects: {value['object_count']}")
        lines.append("")
        lines.append("| class | objects |")
        lines.append("| --- | ---: |")
        for entry in value["classes"][:40]:
            lines.append(f"| `{entry['name']}` | {entry['count']} |")
        lines.append("")
    path.write_text("\n".join(lines), encoding="utf-8")
