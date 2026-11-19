"""Command line entry points for the nxre Python package."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from .catalog import build_catalog, write_catalog
from .detect import detect_model, write_model
from .gear_involute import main as gear_involute_main
from .gear_pair import (
    GearProfile,
    validate_pair,
    write_markdown as write_pair_markdown,
    write_plot,
    write_report as write_pair_report,
)
from .motion import parse_msg, parse_rmd, write_motion_report
from .module_info import inspect_modules, write_markdown as write_module_markdown
from .motion_minp import write_mres_csv
from .motion_module import build_sim_report, write_markdown as write_sim_markdown
from .motion_rad import main as motion_rad_main
from .motion_results import (
    inspect_sim,
    parse_rad,
    parse_rplt,
    validate_gear_ratio,
    write_csv as write_motion_csv,
    write_json as write_motion_json,
)
from .nx_env import ugopen_nxopen_dir
from .nxopen_check import check_journal, write_report as write_nxopen_check
from .workflow import run_workflow
from .pump import (
    load_conditions,
    load_model,
    results_to_json,
    simulate,
    sweep,
    write_csv,
    write_markdown,
)


def _pump_detect(args: argparse.Namespace) -> int:
    model = detect_model(args.driving, args.driven, args.body, args.nxview)
    write_model(model, args.out)
    print(f"wrote {args.out}")
    print(
        f"teeth={model.gear.teeth} module={model.gear.module_mm:.6f} mm "
        f"face_width={model.gear.face_width_mm:.6f} mm "
        f"center_distance={model.gear.center_distance_mm:.6f} mm"
    )
    return 0


def _pump_sim(args: argparse.Namespace) -> int:
    model = load_model(args.model)
    results = load_conditions(args.conditions)
    if args.sweep:
        results = sweep(
            model,
            speeds_rpm=args.speeds,
            pressures_bar=args.pressures,
            temperature_c=args.temperature,
        )
    else:
        results = [simulate(model, condition) for condition in results]
    if args.out:
        output = Path(args.out)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(
            json.dumps(
                results_to_json(model, results),
                ensure_ascii=False,
                indent=2,
            ),
            encoding="utf-8",
        )
        print(f"wrote {output}")
    if args.csv:
        write_csv(results, args.csv)
        print(f"wrote {args.csv}")
    if args.markdown:
        write_markdown(model, results, args.markdown)
        print(f"wrote {args.markdown}")
    failed = sum(1 for result in results if not result.passed)
    print(
        f"conditions={len(results)} passed={len(results) - failed} failed={failed}"
    )
    for result in results:
        if result.passed:
            continue
        for check in result.checks:
            if not check.passed:
                print(
                    f"FAIL {result.condition.name} {check.name}: "
                    f"{check.value:.3f} {check.unit} limit {check.limit:.3f}"
                )
    return 0 if failed == 0 else 2


def _workflow(args: argparse.Namespace) -> int:
    report = run_workflow(
        args.corpus,
        args.workdir,
        args.nxview,
        args.assembly,
        args.conditions,
    )
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0


def _motion_rad(args: argparse.Namespace) -> int:
    command = [args.rad]
    if args.json:
        command.extend(["--json", args.json])
    for item in args.entity:
        command.extend(["--entity", item])
    if args.no_matrices:
        command.append("--no-matrices")
    return motion_rad_main(command)


def _motion_results(args: argparse.Namespace) -> int:
    results = parse_rplt(args.rplt, args.time_shift)
    validation = validate_gear_ratio(
        results,
        args.driver,
        args.driven,
        args.driver_rate,
    )
    rad = parse_rad(args.rad) if args.rad else None
    sim = inspect_sim(args.sim) if args.sim else None
    write_motion_json(results, args.out, validation, rad, sim)
    if args.csv:
        write_motion_csv(results, args.csv)
    print(
        "channels={} samples={} ratio={:.12f} passed={}".format(
            len(results.channels),
            len(results.times),
            validation["position_ratio_median"],
            validation["passed"],
        )
    )
    return 0 if validation["passed"] else 2


def _sim_report(args: argparse.Namespace) -> int:
    report = build_sim_report(
        args.sim, args.driver, args.driven, args.driver_rate
    )
    if args.out:
        Path(args.out).parent.mkdir(parents=True, exist_ok=True)
        Path(args.out).write_text(
            json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
        )
    if args.markdown:
        write_sim_markdown(report, args.markdown)
    if args.csv:
        rplt = report["companions"]["rplt"]
        if rplt:
            write_motion_csv(parse_rplt(rplt[0]), args.csv)
        elif report.get("results_mres"):
            write_mres_csv(report["results_mres"], args.csv)
    module = report["module"]
    print(
        "state={} bodies={} joints={} drivers={} gears={} "
        "channels={} samples={} ratio={} passed={}".format(
            module["state"],
            module["bodies"],
            module["joints"],
            module["drivers"],
            module["gears"],
            module["channels"],
            module["samples"],
            module["gear_ratio"],
            module["passed"],
        )
    )
    return 0


def _module_info(args: argparse.Namespace) -> int:
    report = inspect_modules(args.file)
    if args.out:
        Path(args.out).parent.mkdir(parents=True, exist_ok=True)
        Path(args.out).write_text(
            json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
        )
    if args.markdown:
        write_module_markdown(report, args.markdown)
    present = [name for name, value in report["modules"].items() if value["present"]]
    print(
        "container={} streams={} modules={}".format(
            report["container"], len(report["streams"]), ",".join(present)
        )
    )
    return 0


def _nxopen_check(args: argparse.Namespace) -> int:
    report = check_journal(args.journal, args.headers)
    write_nxopen_check(report, args.out)
    print(
        "classes={} methods={} missing_classes={} missing_methods={} passed={}".format(
            len(report.classes),
            len(report.methods),
            len(report.missing_classes),
            len(report.missing_methods),
            report.passed,
        )
    )
    for name in report.missing_classes:
        print("missing class", name)
    for name in report.missing_methods:
        print("missing method", name)
    return 0 if report.passed else 2


def _gear_involute(args: argparse.Namespace) -> int:
    command = []
    for path in args.model:
        command.extend(["--model", path])
    for path in args.profile:
        command.extend(["--profile", path])
    if args.teeth:
        command.extend(["--teeth", str(args.teeth)])
    if args.module:
        command.extend(["--module", str(args.module)])
    if args.output:
        command.extend(["--output", args.output])
    if args.markdown:
        command.extend(["--markdown", args.markdown])
    return gear_involute_main(command)


def _gear_pair(args: argparse.Namespace) -> int:
    driving = GearProfile.load(args.driving)
    driven = GearProfile.load(args.driven)
    report = validate_pair(
        driving,
        driven,
        args.module,
        args.pressure_angle,
        args.center_distance,
        args.steps,
    )
    write_pair_report(report, args.out)
    write_pair_markdown(report, args.markdown)
    if args.plot:
        write_plot(report, driving, driven, args.plot)
    print(
        "teeth={} a={:.4f} min_clearance={:.4f} overlap_phases={} "
        "contact_ratio={:.4f} backlash={} verdict={}".format(
            report.teeth,
            report.center_distance_mm,
            report.sweep.min_clearance_mm,
            report.sweep.overlap_phases,
            report.analytical_contact_ratio,
            "n/a"
            if report.tooth_thickness.circular_backlash_mm is None
            else "{:.4f}".format(report.tooth_thickness.circular_backlash_mm),
            "PASS" if report.passed else "FAIL",
        )
    )
    for check in report.checks:
        if not check["passed"]:
            print(
                "FAIL {}: {} {} expected {}".format(
                    check["name"], check["value"], check["unit"], check["expected"]
                )
            )
    return 0 if report.passed else 2


def _motion_info(args: argparse.Namespace) -> int:
    model = parse_rmd(args.rmd)
    print(
        f"bodies={len(model.bodies)} markers={len(model.markers)} "
        f"joints={len(model.joints)} drivers={len(model.drivers)} gears={len(model.gears)}"
    )
    for body in model.bodies:
        print(f"  body {body.name}: mass={body.mass} inertia={body.inertia}")
    for joint in model.joints:
        print(f"  joint {joint.name}: {joint.kind} {joint.i_marker}->{joint.j_marker}")
    for driver in model.drivers:
        print(f"  driver {driver.name}: joint={driver.joint} function={driver.function}")
    for gear in model.gears:
        print(f"  gear {gear.name}: joints={gear.joints}")
    if args.msg:
        print(f"solver={parse_msg(args.msg)}")
    if args.out:
        write_motion_report(model, args.out)
        print(f"wrote {args.out}")
    return 0


def _catalog(args: argparse.Namespace) -> int:
    catalog = build_catalog(args.directory, args.nxview)
    write_catalog(catalog, args.out, args.markdown)
    print(
        "parts={parts} assemblies={assemblies} total_faces={total_faces} "
        "missing_components={missing_components}".format(**catalog["summary"])
    )
    print(f"wrote {args.out}")
    print(f"wrote {args.markdown}")
    return 0


def _assembly(args: argparse.Namespace) -> int:
    from . import NXVIEW
    import subprocess

    command = [str(NXVIEW), "assembly", args.action]
    command.extend(args.arguments)
    return subprocess.call(command)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="nxre")
    sub = parser.add_subparsers(dest="command", required=True)

    detect = sub.add_parser(
        "pump-detect",
        help="measure a gear-pump model from decoded NX parts",
    )
    detect.add_argument("--driving", required=True)
    detect.add_argument("--driven", required=True)
    detect.add_argument("--body")
    detect.add_argument("--nxview")
    detect.add_argument("--out", required=True)
    detect.set_defaults(function=_pump_detect)

    simulate_parser = sub.add_parser(
        "pump-sim",
        help="run steady-state pump conditions",
    )
    simulate_parser.add_argument("--model")
    simulate_parser.add_argument("--conditions")
    simulate_parser.add_argument("--out")
    simulate_parser.add_argument("--csv")
    simulate_parser.add_argument("--markdown")
    simulate_parser.add_argument("--sweep", action="store_true")
    simulate_parser.add_argument(
        "--speeds",
        type=float,
        nargs="+",
        default=[500, 1000, 1500, 2000, 2500, 3000],
    )
    simulate_parser.add_argument(
        "--pressures",
        type=float,
        nargs="+",
        default=[0, 25, 63, 100, 160],
    )
    simulate_parser.add_argument("--temperature", type=float, default=40.0)
    simulate_parser.set_defaults(function=_pump_sim)

    workflow = sub.add_parser(
        "gearpump-workflow",
        help="catalog, detect, relink, simulate and validate the gear pump",
    )
    workflow.add_argument("--corpus", required=True)
    workflow.add_argument("--workdir", required=True)
    workflow.add_argument("--assembly", default="齿轮泵装配.prt")
    workflow.add_argument("--conditions")
    workflow.add_argument("--nxview")
    workflow.set_defaults(function=_workflow)

    motion_rad = sub.add_parser(
        "motion-rad",
        help="decode NX Motion .rad 4x4 animation tracks",
    )
    motion_rad.add_argument("rad")
    motion_rad.add_argument("--json", dest="json")
    motion_rad.add_argument("--entity", action="append", default=[])
    motion_rad.add_argument("--no-matrices", action="store_true")
    motion_rad.set_defaults(function=_motion_rad)

    motion_results = sub.add_parser(
        "motion-results",
        help="read NX Motion .rplt channels and validate the gear ratio",
    )
    motion_results.add_argument("--rplt", required=True)
    motion_results.add_argument("--rad")
    motion_results.add_argument("--sim")
    motion_results.add_argument("--driver", default="J002")
    motion_results.add_argument("--driven", default="J003")
    motion_results.add_argument("--driver-rate", type=float, default=180.0)
    motion_results.add_argument("--time-shift", type=float)
    motion_results.add_argument("--out", required=True)
    motion_results.add_argument("--csv")
    motion_results.set_defaults(function=_motion_results)

    nxopen_check = sub.add_parser(
        "nxopen-check",
        help="check journal API names against NX headers",
    )
    nxopen_check.add_argument("--journal", required=True)
    nxopen_check.add_argument(
        "--headers", default=str(ugopen_nxopen_dir())
    )
    nxopen_check.add_argument("--out", required=True)
    nxopen_check.set_defaults(function=_nxopen_check)

    involute = sub.add_parser(
        "gear-involute",
        help="fit involute flanks and measure the root fillet",
    )
    involute.add_argument("--model", action="append", default=[])
    involute.add_argument("--profile", action="append", default=[])
    involute.add_argument("--teeth", type=int)
    involute.add_argument("--module", type=float)
    involute.add_argument("--output")
    involute.add_argument("--markdown")
    involute.set_defaults(function=_gear_involute)

    pair = sub.add_parser(
        "gear-pair",
        help="validate meshing of two decoded gear profiles",
    )
    pair.add_argument("--driving", required=True)
    pair.add_argument("--driven", required=True)
    pair.add_argument("--module", type=float, default=3.0)
    pair.add_argument("--pressure-angle", type=float, default=20.0)
    pair.add_argument("--center-distance", type=float)
    pair.add_argument("--steps", type=int, default=360)
    pair.add_argument("--out", required=True)
    pair.add_argument("--markdown", required=True)
    pair.add_argument("--plot")
    pair.set_defaults(function=_gear_pair)

    motion = sub.add_parser("motion-info", help="parse an NX Motion .rmd model")
    motion.add_argument("rmd")
    motion.add_argument("--msg")
    motion.add_argument("--out")
    motion.set_defaults(function=_motion_info)

    sim_report = sub.add_parser(
        "sim-report",
        help="report the complete NX Motion module for a .sim scenario",
    )
    sim_report.add_argument("sim")
    sim_report.add_argument("--out")
    sim_report.add_argument("--markdown")
    sim_report.add_argument("--csv")
    sim_report.add_argument("--driver", default="J002")
    sim_report.add_argument("--driven", default="J003")
    sim_report.add_argument("--driver-rate", type=float, default=180.0)
    sim_report.set_defaults(function=_sim_report)

    module_info = sub.add_parser(
        "module-info",
        help="inventory the NX modules represented in a part, sim, or FE file",
    )
    module_info.add_argument("file")
    module_info.add_argument("--out")
    module_info.add_argument("--markdown")
    module_info.set_defaults(function=_module_info)

    catalog = sub.add_parser(
        "catalog",
        help="decode every part in a directory and write a catalog",
    )
    catalog.add_argument("--directory", required=True)
    catalog.add_argument("--out", required=True)
    catalog.add_argument("--markdown", required=True)
    catalog.add_argument("--nxview")
    catalog.set_defaults(function=_catalog)

    assembly = sub.add_parser("assembly", help="inspect or relink an NX assembly")
    assembly.add_argument("action", choices=["info", "relink", "export"])
    assembly.add_argument("arguments", nargs=argparse.REMAINDER)
    assembly.set_defaults(function=_assembly)

    args = parser.parse_args(argv)
    return args.function(args)


if __name__ == "__main__":
    sys.exit(main())
