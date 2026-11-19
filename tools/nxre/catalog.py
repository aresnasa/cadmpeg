"""Build a catalog of decoded NX parts and assembly references."""

from __future__ import annotations

import json
import subprocess
import tempfile
from pathlib import Path
from typing import Any

from .detect import _run_json, detect_gear, nxview_path


ROLE_HINTS = (
    ("泵体", "housing"),
    ("泵盖", "cover"),
    ("主动齿轮", "driving gear shaft"),
    ("从动齿轮", "driven gear shaft"),
    ("锁紧螺母", "lock nut"),
    ("填料压盖", "packing gland"),
    ("垫片", "gasket"),
    ("螺塞", "plug"),
    ("定位圈", "bearing locator"),
    ("弹簧", "spring"),
    ("钢珠", "ball"),
    ("装配", "assembly"),
)


def _role(name: str) -> str:
    for token, role in ROLE_HINTS:
        if token in name:
            return role
    return "part"


def build_catalog(directory: str | Path, nxview: str | Path | None = None) -> dict[str, Any]:
    executable = str(nxview or nxview_path())
    directory = Path(directory)
    parts = []
    for path in sorted(directory.glob("*.prt")):
        info = _run_json([executable, "json", str(path)])
        entry = {
            "file": path.name,
            "path": str(path),
            "bytes": path.stat().st_size,
            "role": _role(path.name),
            "container": info.get("container"),
            "bodies": info.get("bodies"),
            "faces": info.get("faces"),
            "edges": info.get("edges"),
            "surfaces": info.get("surfaces"),
            "tessellations": info.get("tessellations"),
            "features": info.get("features"),
            "attributes": info.get("attributes_count"),
            "occurrences": info.get("occurrences"),
            "placements": len(info.get("placements", [])),
            "losses": len(info.get("losses", [])),
        }
        if entry["role"] in ("driving gear shaft", "driven gear shaft"):
            try:
                entry["gear"] = detect_gear(path, executable)
            except Exception as error:
                entry["gear_error"] = str(error)
        parts.append(entry)

    assemblies = []
    for entry in parts:
        if entry["role"] != "assembly":
            continue
        try:
            with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as temporary:
                temporary_path = Path(temporary.name)
            subprocess.run(
                [
                    executable,
                    "assembly",
                    "export",
                    entry["path"],
                    str(temporary_path),
                    "--root",
                    str(directory),
                ],
                check=True,
                capture_output=True,
            )
            manifest = json.loads(temporary_path.read_text(encoding="utf-8"))
            temporary_path.unlink(missing_ok=True)
        except Exception:
            manifest = None
        if manifest is not None:
            assemblies.append(
                {
                    "file": entry["file"],
                    "directory": manifest.get("directory"),
                    "components": manifest.get("components", []),
                }
            )
    return {
        "directory": str(directory),
        "parts": parts,
        "assemblies": assemblies,
        "summary": {
            "parts": len(parts),
            "assemblies": sum(1 for part in parts if part["role"] == "assembly"),
            "gear_parts": sum(1 for part in parts if "gear" in part),
            "total_faces": sum(part.get("faces") or 0 for part in parts),
            "missing_components": sum(
                1
                for assembly in assemblies
                for component in assembly["components"]
                if not component.get("present")
            ),
        },
    }


def write_catalog(catalog: dict[str, Any], out_json: str | Path, out_md: str | Path) -> None:
    out_json = Path(out_json)
    out_json.parent.mkdir(parents=True, exist_ok=True)
    out_json.write_text(
        json.dumps(catalog, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    lines = [
        "# Gear-pump NX10 part catalog",
        "",
        f"Directory: `{catalog['directory']}`",
        "",
        "| File | Role | Container | Bodies | Faces | Surfaces | Tess | Features | Attrs |",
        "| --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for part in catalog["parts"]:
        lines.append(
            "| {file} | {role} | {container} | {bodies} | {faces} | {surfaces} | "
            "{tessellations} | {features} | {attributes} |".format(**part)
        )
    lines.extend(["", "## Gear measurements", ""])
    for part in catalog["parts"]:
        gear = part.get("gear")
        if not gear:
            continue
        lines.append(
            "- `{file}`: z={teeth}, tip={tip_radius:.3f} mm, root={root_radius:.3f} mm, "
            "face={face_width:.3f} mm, axis={axis}, periodicity={periodicity:.3f}".format(
                file=part["file"], **gear
            )
        )
    lines.extend(["", "## Assemblies", ""])
    for assembly in catalog["assemblies"]:
        present = sum(1 for component in assembly["components"] if component["present"])
        lines.append(
            f"- `{assembly['file']}`: {present}/{len(assembly['components'])} "
            f"components resolve under `{assembly['directory']}`"
        )
    out_md = Path(out_md)
    out_md.parent.mkdir(parents=True, exist_ok=True)
    out_md.write_text("\n".join(lines) + "\n", encoding="utf-8")
