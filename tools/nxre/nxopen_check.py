"""Static check of NX Open API names used by a journal against NX headers."""

from __future__ import annotations

import json
import re
from dataclasses import asdict, dataclass, field
from pathlib import Path


@dataclass
class NameCheck:
    name: str
    kind: str
    found: bool
    header: str | None = None


@dataclass
class JournalCheck:
    journal: str
    headers: str
    classes: list[NameCheck] = field(default_factory=list)
    methods: list[NameCheck] = field(default_factory=list)
    missing_classes: list[str] = field(default_factory=list)
    missing_methods: list[str] = field(default_factory=list)

    @property
    def passed(self) -> bool:
        return not self.missing_classes and not self.missing_methods

    def to_dict(self) -> dict:
        data = asdict(self)
        data["passed"] = self.passed
        return data


def _header_index(headers: Path) -> tuple[set[str], set[str], dict[str, str]]:
    available_classes: set[str] = set()
    available_methods: set[str] = set()
    class_sources: dict[str, str] = {}
    for path in headers.glob("*.hxx"):
        text = path.read_text(encoding="utf-8", errors="replace")
        available_classes.add(path.stem)
        for name in re.findall(r"\bclass\s+([A-Za-z_]\w*)\b", text):
            available_classes.add(name)
            class_sources.setdefault(name, str(path))
        available_methods.update(re.findall(r"\b([A-Za-z_]\w*)\s*\(", text))
    return available_classes, available_methods, class_sources


def check_journal(journal: str | Path, headers: str | Path) -> JournalCheck:
    journal_path = Path(journal)
    source = journal_path.read_text(encoding="utf-8", errors="replace")
    available_classes, available_methods, class_sources = _header_index(Path(headers))
    class_names = sorted(set(re.findall(r"NXOpen\.([A-Za-z_]\w*)", source)))
    method_names = sorted(set(re.findall(r"\.([A-Za-z_]\w*)\s*\(", source)))
    ignored_methods = {
        "append",
        "close",
        "format",
        "get",
        "getattr",
        "items",
        "join",
        "list",
        "open",
        "read",
        "replace",
        "split",
        "strip",
        "write",
        "abspath",
        "basename",
        "dirname",
        "isdir",
        "isfile",
        "makedirs",
        "dump",
        "print_exc",
        "splitext",
    }
    report = JournalCheck(str(journal_path), str(headers))
    for name in class_names:
        found = (
            name in available_classes
            or any(stem.startswith(name + "_") for stem in available_classes)
        )
        report.classes.append(
            NameCheck(name, "class", found, class_sources.get(name))
        )
        if not found:
            report.missing_classes.append(name)
    for name in method_names:
        if name in ignored_methods:
            continue
        found = name in available_methods
        report.methods.append(NameCheck(name, "method", found))
        if not found:
            report.missing_methods.append(name)
    return report


def write_report(report: JournalCheck, path: str | Path) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(report.to_dict(), ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
