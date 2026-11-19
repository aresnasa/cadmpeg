# NX Open workbench for the NX10 gear-pump corpus

`gear_pump_workbench.py` is an NX Open journal. It opens the assembly, relinks
components, edits part attributes and expressions, updates the assembly, saves
a working copy, and runs the existing motion scenario.

## Prerequisites

- NX with the Assemblies and Mechanisms licenses.
- The assembly and child parts copied to a writable working directory.
- A motion scenario already present in the assembly, for example
  `motion_1.sim` from `齿轮泵UG10.0\齿轮泵装配`.
- Python 3 for NX 2412 journals. NX 10 journals use Python 2.7; the source
  uses `from __future__ import print_function` and avoids f-strings, but test
  it in the target NX release.

## Run

1. Copy `gearpump_nxopen.json` to the working directory and edit the paths.
2. In NX: **File > Execute > NX Open** and select
   `gear_pump_workbench.py`.
3. Inspect `log_path` and the saved assembly.
4. Check **Assemblies > Update Structure** and **Motion > Solve**.

Command-line form:

```text
run_journal.exe gear_pump_workbench.py --config gearpump_nxopen.json
```

## What the journal changes

The default `gearpump_nxopen.json` points at
`C:\RE\artifacts\gearpump_open` — a relinked copy that also contains the
`齿轮泵装配\motion_1.sim` folder. Running the journal edits that copy, not
the source corpus. Re-run `C:\RE\artifacts\build_open_package.py` to restore
the pristine copy. `relink` is `false` because this copy is already relinked;
set it to `true` when pointing `component_root` at a different directory.

| Stage | API | Effect |
| --- | --- | --- |
| Relink | `Assemblies.ReplaceComponentBuilder` | replaces each top-level component with the same file name under `component_root` |
| Attributes | `NXObject.SetUserAttribute` | writes part attributes and saves each part |
| Expressions | `Expression.SetFormula` | edits configured expressions and updates the session |
| Assembly update | `Update.DoUpdate` | rebuilds the assembly after child edits |
| Save | `BasePart.SaveAs` | writes `output_assembly` |
| Motion | `MotionSession`, `MotionSolution.SolveNormalRunSolution` | solves the existing scenario and exports the driver result |
| Motion cases | `JointDriverCollection.CreateJointDriverBuilder`, `DriverOperation.DisplacementExpression` | rewrites the driver displacement for each `case_speeds_rpm`, solves, exports one CSV per speed, and restores the original formula |

## Limits

- The journal does not create joints, links, contacts, meshes, or loads. It
  runs a motion setup that already exists in the assembly.
- The motion case sweep uses **radians/second** for the driver displacement.
  The saved `π*TIME` driver produces 900° over 5 s in `.rplt`, which confirms
  the rotational expression is in radians. Check the logged scenario/driver
  names before relying on the CSV files.
- The `cae` stage only logs the configured `.sim` file. Prepare the FEM and
  solution in Simcenter 3D, then solve it in NX.
- A missing component or a failed expression is logged and the run continues.
  Inspect the log before using the saved assembly.
- No `GEAR`/`PATTERN` feature exists in the NX10 gear shafts: the 14 teeth
  are 14 BREP-copy + META-unite pairs. `gear_module` / `gear_face_width` are
  KF/ESS metadata and are not referenced by any feature record, so editing
  them does not rebuild the tooth B-rep. Geometry changes require NX
  Knowledge Fusion or NX Open to re-run the gear generator.
- Verify every saved part with `ug_inspect -scan` and open the assembly in NX
  before production use.

## Read-only open check

`gear_pump_open_check.py` + `gearpump_open_check.json` open the assembly
combination and report the component tree and gear expressions without
saving or editing anything:

```text
run_journal.exe gear_pump_open_check.py --config gearpump_open_check.json
```

The default config points at `C:\RE\artifacts\gearpump_open\齿轮泵装配.prt`
and writes `C:\RE\artifacts\nx_open_check.json`. Expected result: 11 loaded
components and `gear_module=3`, `gear_teeth_number=14`,
`gear_face_width=25` in both gear shafts.

## Offline checks

The repository-side checks do not need an NX license:

```powershell
python -m py_compile python\nxre\nxopen\gear_pump_workbench.py
python -m nxre assembly info C:\RE\artifacts\齿轮泵UG10.0\齿轮泵装配.prt --root C:\RE\artifacts\齿轮泵UG10.0
python -m nxre assembly relink C:\RE\artifacts\齿轮泵UG10.0\齿轮泵装配.prt C:\RE\artifacts\gearpump_work\齿轮泵装配_relinked.prt C:\RE\artifacts\齿轮泵UG10.0
```
