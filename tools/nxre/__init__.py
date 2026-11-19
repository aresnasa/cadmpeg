"""Python interface to the NX reverse-engineering engine.

The heavy lifting remains in Rust (`nxview`); this module provides a stable
Python API for reading, editing, and headless rendering NX `.prt` / `.fem`
files. Set `NXVIEW` to override the executable path.
"""

from __future__ import annotations

import ast
import json
import os
import subprocess
import tempfile
from pathlib import Path
from typing import Any

from .nx_env import ug_inspect_path


DEFAULT_NXVIEW = r"C:\RE\build-cadmpeg\release\nxview.exe"
NXVIEW = Path(os.environ.get("NXVIEW", DEFAULT_NXVIEW))


class NxError(RuntimeError):
    """Raised when the Rust engine rejects an operation."""


class NxPart:
    """A decoded NX part with read, edit, and render helpers."""

    def __init__(self, path: str | Path, info: dict[str, Any]):
        self.path = Path(path)
        self.info = info

    @classmethod
    def open(cls, path: str | Path) -> "NxPart":
        """Decode `path` with the Rust engine and return a part handle."""
        path = Path(path)
        code, text = _run([str(NXVIEW), "json", str(path)])
        if code != 0:
            raise NxError(text.strip() or f"failed to read {path}")
        try:
            info = json.loads(text)
        except json.JSONDecodeError as error:
            raise NxError(f"engine returned invalid JSON: {error}") from error
        return cls(path, info)

    @property
    def container(self) -> str:
        return self.info.get("container", "unknown")

    @property
    def attributes(self) -> dict[str, str]:
        return {
            attribute["title"]: attribute["value"]
            for attribute in self.info.get("attributes", [])
        }

    @property
    def placements(self) -> list[dict[str, Any]]:
        return list(self.info.get("placements", []))

    def attribute(self, title: str, default: str | None = None) -> str | None:
        return self.attributes.get(title, default)

    def set_attribute(
        self, output: str | Path, title: str, value: str
    ) -> dict[str, Any]:
        """Write a modified copy and return the engine's edit report."""
        output = Path(output)
        code, text = _run(
            [str(NXVIEW), "attr", "set", str(self.path), str(output), title, value]
        )
        if code != 0:
            raise NxError(text.strip() or "attribute edit failed")
        report: dict[str, Any] = {"output": str(output)}
        for line in text.splitlines():
            if line.startswith("changed_streams="):
                report["changed_streams"] = ast.literal_eval(
                    line.split("=", 1)[1]
                )
            elif line.startswith("source_sha256="):
                report["source_sha256"] = line.split("=", 1)[1]
            elif line.startswith("output_sha256="):
                report["output_sha256"] = line.split("=", 1)[1]
            elif line.startswith("byte_identical="):
                report["byte_identical"] = line.split("=", 1)[1].lower() == "true"
        return report

    def render_png(
        self,
        output: str | Path,
        width: int = 1100,
        height: int = 700,
    ) -> Path:
        """Render a headless PNG through the Rust software rasterizer."""
        output = Path(output)
        output.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile(
            suffix=".ppm", delete=False, dir=output.parent
        ) as temporary:
            ppm = Path(temporary.name)
        try:
            code, text = _run(
                [
                    str(NXVIEW),
                    "render",
                    str(self.path),
                    str(ppm),
                    str(width),
                    str(height),
                ]
            )
            if code != 0 or not ppm.exists():
                raise NxError(text.strip() or "render failed")
            try:
                from PIL import Image
            except ImportError as error:
                raise NxError("Pillow is required for render_png") from error
            with Image.open(ppm) as image:
                image.save(output, format="PNG")
        finally:
            ppm.unlink(missing_ok=True)
        return output

    def scan(self) -> tuple[bool, str]:
        """Run `ug_inspect -scan` when it is available."""
        ug = ug_inspect_path()
        if not Path(ug).exists():
            return False, "ug_inspect not available"
        code, text = _run([ug, "-scan", str(self.path)])
        return code == 0 and "No corruptions identified" in text, text

    def __repr__(self) -> str:
        return (
            f"NxPart({self.path.name!r}, container={self.container!r}, "
            f"bodies={self.info.get('bodies')}, faces={self.info.get('faces')})"
        )


def _run(command: list[str]) -> tuple[int, str]:
    result = subprocess.run(command, capture_output=True)
    return result.returncode, (result.stdout + result.stderr).decode(
        "utf-8", "replace"
    )


from .detect import detect_gear, detect_model  # noqa: E402
from .motion import MotionModel, parse_msg, parse_rmd  # noqa: E402
from .pump import (  # noqa: E402
    Condition,
    GearGeometry,
    GearPumpModel,
    SimulationResult,
    default_model,
    load_conditions,
    load_model,
    simulate,
    sweep,
)

__all__ = [
    "Condition",
    "GearGeometry",
    "GearPumpModel",
    "MotionModel",
    "NxError",
    "NxPart",
    "SimulationResult",
    "NXVIEW",
    "default_model",
    "detect_gear",
    "detect_model",
    "load_conditions",
    "load_model",
    "parse_msg",
    "parse_rmd",
    "simulate",
    "sweep",
]
