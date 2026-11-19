# -*- coding: utf-8 -*-
# Read-only NX Open journal: open the gear-pump assembly combination and
# report what NX actually loaded.
#
# Run inside NX (File > Execute > NX Open) or:
#   run_journal.exe gear_pump_open_check.py --config gearpump_open_check.json
#
# This journal never saves, never replaces components and never edits parts.
# It is the acceptance check for "the .prt combination opens correctly".

from __future__ import print_function

import json
import os
import sys
import traceback

import NXOpen
import NXOpen.Assemblies

LOG = []
RESULT = {}


def log(message):
    LOG.append(str(message))
    print("[open-check] " + str(message))


def get(object_, name):
    """Return a property or a no-argument method result."""
    value = getattr(object_, name)
    if callable(value):
        try:
            return value()
        except TypeError:
            return value
    return value


def load_config():
    candidates = [
        value
        for value in sys.argv[1:]
        if value not in ("-args", "--args", "-config", "--config")
    ]
    if candidates:
        candidate = candidates[0]
    else:
        candidate = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "gearpump_open_check.json"
        )
    if not os.path.isfile(candidate):
        raise RuntimeError("configuration file not found: " + candidate)
    with open(candidate, "r", encoding="utf-8-sig") as handle:
        return json.load(handle)


def open_part(path):
    session = NXOpen.Session.GetSession()
    log("open " + path)
    value = session.Parts.OpenBaseDisplay(path)
    status = None
    if isinstance(value, tuple):
        if len(value) > 1:
            status = value[1]
        value = value[0]
    if value is None:
        log("open failed: " + str(status_text(status)))
    return value, status


def status_text(status):
    if status is None:
        return None
    for name in ("GetStatus", "Status"):
        try:
            return str(get(status, name))
        except Exception:
            continue
    return None


def walk_component(component, depth, rows):
    row = {"depth": depth}
    for key in ("DisplayName", "Name"):
        try:
            row[key] = str(get(component, key))
        except Exception as error:
            row[key] = None
            row[key + "_error"] = str(error)
    for key in ("IsSuppressed", "ReferenceSet"):
        try:
            value = get(component, key)
            row[key] = str(value)
        except Exception:
            pass
    rows.append(row)
    try:
        children = get(component, "GetChildren")
    except Exception as error:
        row["children_error"] = str(error)
        return
    for child in children:
        walk_component(child, depth + 1, rows)


def read_expressions(path, names):
    out = {}
    try:
        part, status = open_part(path)
    except Exception as error:
        return {"error": str(error)}
    out["opened"] = True
    out["load_status"] = status_text(status)
    try:
        collection = get(part, "Expressions")
        for name in names:
            try:
                expression = collection.FindObject(name)
                out[name] = str(expression.GetFormula())
            except Exception as error:
                out[name] = "missing: " + str(error)
    except Exception as error:
        out["expressions_error"] = str(error)
    return out


def main():
    config = load_config()
    assembly_path = config.get("assembly")
    if not assembly_path:
        raise RuntimeError("config has no 'assembly' path")

    session = NXOpen.Session.GetSession()
    part, load_status = open_part(assembly_path)
    if part is None:
        log("assembly did not open: " + str(status_text(load_status)))
        return
    log("assembly opened: " + str(get(part, "FullPath")))
    RESULT["assembly"] = str(get(part, "FullPath"))
    RESULT["load_status"] = status_text(load_status)

    rows = []
    try:
        component_assembly = get(part, "ComponentAssembly")
        root = get(component_assembly, "RootComponent")
    except Exception as error:
        log("no component assembly: " + str(error))
        component_assembly = None
        root = None

    if root is not None:
        for child in get(root, "GetChildren"):
            walk_component(child, 1, rows)
    RESULT["components"] = rows
    RESULT["component_count"] = len(rows)
    log("components: " + str(len(rows)))

    expression_names = config.get("expressions", [])
    RESULT["expression_parts"] = {}
    for name in config.get("expression_parts", []):
        path = os.path.join(config.get("parts_root", ""), name)
        RESULT["expression_parts"][name] = read_expressions(path, expression_names)

    RESULT["log"] = LOG
    log_path = config.get("log_path")
    if log_path:
        parent = os.path.dirname(log_path)
        if parent and not os.path.isdir(parent):
            os.makedirs(parent)
        with open(log_path, "w", encoding="utf-8") as handle:
            json.dump(RESULT, handle, ensure_ascii=False, indent=2)
        log("wrote " + log_path)

    if config.get("close_after_check"):
        try:
            part.Close(NXOpen.BasePart.CloseAfterSaveFalse)
        except Exception as error:
            log("close failed: " + str(error))


if __name__ == "__main__":
    try:
        main()
    except Exception:
        traceback.print_exc()
        raise
