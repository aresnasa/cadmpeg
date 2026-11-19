# -*- coding: utf-8 -*-
# NX Open journal for the NX10 gear-pump corpus.
#
# Run inside NX: File > Execute > NX Open > select this file, or
#   run_journal.exe gear_pump_workbench.py --config gearpump_nxopen.json
#
# The journal opens the assembly, relinks missing components, edits part
# attributes and expressions, updates the session, saves a working copy, and
# runs the existing motion scenario when one is present. Every optional stage
# is logged and does not stop the later stages.
#
# API names follow the NX 2412 NXOpen headers. NX10 exposes the same object
# model for the calls below; newer calls are guarded.

from __future__ import print_function

import json
import math
import os
import sys
import traceback

import NXOpen
import NXOpen.Assemblies
import NXOpen.Motion


LOG = []


def log(message):
    LOG.append(str(message))
    print("[gearpump] " + str(message))


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
            os.path.dirname(os.path.abspath(__file__)), "gearpump_nxopen.json"
        )
    if not os.path.isfile(candidate):
        raise RuntimeError("configuration file not found: " + candidate)
    with open(candidate, "r", encoding="utf-8-sig") as handle:
        return json.load(handle)


def open_part(path):
    session = NXOpen.Session.GetSession()
    log("open " + path)
    part = session.Parts.OpenBaseDisplay(path)
    if isinstance(part, tuple):
        part = part[0]
    return part


def save_part(part, path):
    if not path:
        return
    log("save as " + path)
    status = part.SaveAs(path)
    if status is not None and hasattr(status, "Dispose"):
        status.Dispose()


def find_component(part, name):
    if part is None:
        return None
    try:
        component_assembly = get(part, "ComponentAssembly")
    except Exception:
        return None
    if component_assembly is None:
        return None
    root = get(component_assembly, "RootComponent")
    if root is None:
        return None
    return _find_component_below(root, name)


def _find_component_below(component, name):
    try:
        if get(component, "DisplayName") == name or get(component, "Name") == name:
            return component
    except Exception:
        pass
    try:
        children = get(component, "GetChildren")
    except Exception:
        return None
    for child in children:
        found = _find_component_below(child, name)
        if found is not None:
            return found
    return None


def relink(assembly, root):
    if not root:
        return
    log("relink component root " + root)
    try:
        manager = get(assembly, "AssemblyManager")
        builder = manager.CreateReplaceComponentBuilder()
    except Exception as error:
        log("relink builder unavailable: " + str(error))
        return
    try:
        component_assembly = get(assembly, "ComponentAssembly")
        root_component = get(component_assembly, "RootComponent")
        for component in get(root_component, "GetChildren"):
            try:
                prototype = get(component, "Prototype")
                old_path = str(get(prototype, "FullPath"))
            except Exception:
                continue
            name = os.path.basename(old_path)
            new_path = os.path.join(root, name)
            if not os.path.isfile(new_path):
                log("relink skip missing " + new_path)
                continue
            try:
                get(builder, "ComponentsToReplace").Add(component)
                builder.SetReplacementPart(new_path)
                builder.SetReplaceAllOccurrences(False)
                builder.SetMaintainRelationships(True)
                builder.Commit()
                builder.Destroy()
                log("relinked " + name)
                builder = manager.CreateReplaceComponentBuilder()
            except Exception as error:
                log("relink failed for " + name + ": " + str(error))
                try:
                    builder.Destroy()
                except Exception:
                    pass
                builder = manager.CreateReplaceComponentBuilder()
    finally:
        try:
            builder.Destroy()
        except Exception:
            pass


def apply_attributes(assembly, root, attributes):
    if not attributes:
        return
    session = NXOpen.Session.GetSession()
    for name, values in attributes.items():
        path = os.path.join(root, name) if root else name
        if not os.path.isfile(path):
            log("attribute part missing " + path)
            continue
        part = open_part(path)
        for title, value in values.items():
            try:
                part.SetUserAttribute(title, 0, value, NXOpen.Update.Option.Now)
                log("attribute " + name + " " + title + "=" + value)
            except Exception as error:
                log("attribute failed " + name + " " + title + ": " + str(error))
        try:
            part.Save(
                NXOpen.BasePart.SaveComponentsTrue,
                NXOpen.BasePart.CloseAfterSaveFalse,
            )
        except Exception as error:
            log("save failed " + name + ": " + str(error))
    get(session, "UpdateManager").DoUpdate()


def apply_expressions(assembly, root, expressions):
    if not expressions:
        return
    session = NXOpen.Session.GetSession()
    for name, values in expressions.items():
        path = os.path.join(root, name) if root else name
        if not os.path.isfile(path):
            log("expression part missing " + path)
            continue
        part = open_part(path)
        for expression_name, formula in values.items():
            try:
                expression = get(part, "Expressions").FindObject(expression_name)
                expression.SetFormula(formula)
                log("expression " + name + " " + expression_name + "=" + formula)
            except Exception as error:
                log(
                    "expression failed "
                    + name
                    + " "
                    + expression_name
                    + ": "
                    + str(error)
                )
        try:
            part.Save(
                NXOpen.BasePart.SaveComponentsTrue,
                NXOpen.BasePart.CloseAfterSaveFalse,
            )
        except Exception as error:
            log("save failed " + name + ": " + str(error))
    get(session, "UpdateManager").DoUpdate()


def expression_formula(expression):
    try:
        return expression.GetFormula()
    except Exception:
        return get(expression, "Formula")


def set_driver_speed(motion_part, driver_name, operation_name, rpm):
    drivers = get(get(motion_part, "MotionManager"), "JointDrivers")
    driver = drivers.FindObject(driver_name)
    builder = drivers.CreateJointDriverBuilder(driver)
    try:
        operations = get(builder, "DriverMultiOperations")
        operation = get(operations, operation_name)
        expression = get(operation, "DisplacementExpression")
        original = expression_formula(expression)
        # .rplt validates this model: pi*TIME -> 900 deg over 5 s, so the
        # rotational displacement expression is in radians.
        omega_rad_s = rpm * 2.0 * math.pi / 60.0
        expression.SetFormula("{0:.12g}*TIME".format(omega_rad_s))
        builder.Commit()
        return original
    finally:
        try:
            builder.Destroy()
        except Exception:
            pass


def solve_and_export(solution, csv_path):
    log("solve motion solution")
    solution.SolveNormalRunSolution()
    if not csv_path:
        return
    export = solution.GetSolutionSpreadsheetExport()
    try:
        export.SetFileName(csv_path)
    except AttributeError:
        export.FileName = csv_path
    export.ExportDriverResult()
    export.SaveSpreadsheet()
    export.DisconnectFromSpreadsheet()
    export.CloseSpreadsheet()
    log("wrote motion csv " + csv_path)


def run_motion(config, root, master=None):
    motion = config.get("motion")
    if not motion:
        return
    session = NXOpen.Session.GetSession()
    session_motion = get(session, "MotionSession")
    if master is None:
        master = open_part(config["assembly"])
    raw_scenarios = list(session_motion.GetMotionScenarios(master))
    names = []
    for item in raw_scenarios:
        if isinstance(item, (list, tuple)):
            names.append(str(item[0]))
        else:
            names.append(str(item))
    log("motion scenarios: " + ", ".join(names))
    scenario = motion.get("scenario")
    if scenario not in names:
        log("motion scenario not found: " + str(scenario))
        return
    motion_path = session_motion.GetFullName(master, scenario)
    motion_part = open_part(motion_path)
    session_motion.InitializeSimulation(motion_part)
    solutions = get(get(motion_part, "MotionManager"), "MotionSolutions")
    solution_name = motion.get("solution")
    solution = None
    if solution_name:
        try:
            solution = solutions.FindObject(solution_name)
        except Exception as error:
            log("solution lookup failed: " + str(error))
    if solution is None:
        try:
            solution = solutions.GetActiveSolution()
            log("using active motion solution")
        except Exception as error:
            log("active solution lookup failed: " + str(error))
            return
    case_speeds = motion.get("case_speeds_rpm") or []
    if not case_speeds:
        solve_and_export(solution, motion.get("export_csv"))
        return
    driver_name = motion.get("driver_name", "J002.RMotion")
    operation_name = motion.get("driver_operation", "MotionEulerAngle1")
    original = None
    try:
        for rpm in case_speeds:
            original = set_driver_speed(motion_part, driver_name, operation_name, rpm)
            get(session, "UpdateManager").DoUpdate()
            csv_path = motion.get("export_csv")
            if csv_path:
                stem = os.path.splitext(csv_path)[0]
                csv_path = "{0}_{1:g}rpm.csv".format(stem, rpm)
            solve_and_export(solution, csv_path)
    except Exception as error:
        log("motion case sweep failed: " + str(error))
    finally:
        if original is not None:
            try:
                drivers = get(get(motion_part, "MotionManager"), "JointDrivers")
                driver = drivers.FindObject(driver_name)
                builder = drivers.CreateJointDriverBuilder(driver)
                operations = get(builder, "DriverMultiOperations")
                operation = get(operations, operation_name)
                get(operation, "DisplacementExpression").SetFormula(original)
                builder.Commit()
                builder.Destroy()
                log("restored motion driver " + driver_name)
            except Exception as error:
                log("restore motion driver failed: " + str(error))


def run_cae(config):
    cae = config.get("cae")
    if not cae:
        return
    log(
        "CAE stage is optional; open the prepared .sim part and run its solution "
        "inside NX. Configured file: " + str(cae.get("sim_file"))
    )


def main():
    config = load_config()
    assembly_path = config["assembly"]
    root = config.get("component_root") or os.path.dirname(assembly_path)
    assembly = open_part(assembly_path)
    if config.get("relink", True):
        relink(assembly, root)
    apply_attributes(assembly, root, config.get("attributes", {}))
    apply_expressions(assembly, root, config.get("expressions", {}))
    try:
        get(NXOpen.Session.GetSession(), "UpdateManager").DoUpdate()
    except Exception as error:
        log("assembly update failed: " + str(error))
    output = config.get("output_assembly")
    if output:
        save_part(assembly, output)
    run_motion(config, root, assembly)
    run_cae(config)
    log_path = config.get("log_path")
    if log_path:
        with open(log_path, "w", encoding="utf-8") as handle:
            handle.write("\n".join(LOG))
    log("done")


if __name__ == "__main__":
    try:
        main()
    except Exception:
        traceback.print_exc()
        raise
