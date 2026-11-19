"""Resolve the NX installation used by the reverse-engineering harnesses.

NX 2606 at ``C:\\Program Files\\Siemens\\Designcenter2606`` is the current
target. Older evidence was captured from NX 2412 at ``W:\\UGNX``; that install
remains a fallback when the 2606 tree is absent.

Overrides, highest priority first:

``NX_INSTALL_ROOT`` / ``UGII_BASE_DIR``
    Installation root containing ``NXBIN``.
``NXBIN_DIR``
    ``NXBIN`` directory itself.
``UG_INSPECT`` / ``RUN_JOURNAL``
    Exact executable paths.
"""

from __future__ import annotations

import os
from pathlib import Path

PRIMARY_ROOT = Path(r"C:\Program Files\Siemens\Designcenter2606")
FALLBACK_ROOT = Path(r"W:\UGNX")


def nx_install_root() -> Path:
    """Return the NX installation root, preferring the 2606 install."""
    for variable in ("NX_INSTALL_ROOT", "UGII_BASE_DIR"):
        value = os.environ.get(variable)
        if value:
            return Path(value)
    if (PRIMARY_ROOT / "NXBIN").is_dir():
        return PRIMARY_ROOT
    return FALLBACK_ROOT


def nxbin_dir() -> Path:
    """Return the ``NXBIN`` directory holding the NX executables and DLLs."""
    value = os.environ.get("NXBIN_DIR")
    if value:
        return Path(value)
    return nx_install_root() / "NXBIN"


def ug_inspect_path() -> Path:
    """Return ``ug_inspect.exe``; ``UG_INSPECT`` overrides the derived path."""
    return Path(os.environ.get("UG_INSPECT", nxbin_dir() / "ug_inspect.exe"))


def run_journal_path() -> Path:
    """Return ``run_journal.exe``; ``RUN_JOURNAL`` overrides the derived path."""
    return Path(os.environ.get("RUN_JOURNAL", nxbin_dir() / "run_journal.exe"))


def ugopen_nxopen_dir() -> Path:
    """Return the NX Open header directory if the install ships one."""
    return nx_install_root() / "UGOPEN" / "NXOpen"
