# nxre — NX reverse-engineering CLI

Python command-line tooling used to inspect NX parts, assemblies, Motion
scenarios and module payloads. It complements the Rust codecs and the
`nxview` probe; it is not required to build or test the Rust workspace.

## Setup

```powershell
python -m pip install -r tools/nxre/requirements.txt
$env:PYTHONPATH = "tools/nxre;$env:PYTHONPATH"
```

The NX 2606 paths can be overridden with:

```text
NX_UGII_BASE_DIR=C:\Program Files\Siemens\Designcenter2606
NX_UG_INSPECT=...\NXBIN\ug_inspect.exe
NX_UGOPEN_DIR=...\NXBIN\managed
```

## Commands

```powershell
# Motion module report for a .sim (NX10 .rmd/.rplt and NX 2606 .minp/.mres)
python -m nxre sim-report motion_1.sim --out motion.json --markdown motion.md --csv motion.csv

# NX module marker/stream inventory for a .prt, .sim or .fem
python -m nxre module-info part.prt --out modules.json --markdown modules.md

# OM class inventory through the installed ug_inspect
python -m nxre module-classes part.prt --out classes.json --markdown classes.md

# NX10 Motion model and result files
python -m nxre motion-info model.rmd --msg solver.msg
python -m nxre motion-results --rplt result.rplt --rad animation.rad

# NX assembly manifest / relink
python -m nxre assembly info assembly.prt --root parts
```

## Motion 2606 formats

`motion_minp.py` decodes the NX 2606 RecurDyn text model (`BODY`, `JOINT`,
`GEAR`, `DRIVER`, `NAMEMAP`), the solver log (`.minf`) and the verified subset
of the binary `.mres` result records (201 samples × 1520-byte float32 records).
The verified result slots are listed in `MRES_CHANNELS`; the remaining slots
require the full result channel table.

## Notes

- The codec work lives in the Rust workspace; this directory only contains the
  Python RE tooling.
- Some scripts default to the local artifact paths under `C:\RE\artifacts`;
  pass explicit paths on the command line for reproducible runs.
