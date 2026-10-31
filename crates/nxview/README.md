# nxview — Siemens NX `.prt` reader and viewer (Rust)

`nxview` is a thin, cross-platform NX part engine built on the
[`cadmpeg`](../..) Rust codec. It opens NX `.prt` files, decodes the
Parasolid B-rep and embedded display data, tessellates faces, and either
exports Wavefront OBJ or opens an interactive 3D window. The viewer uses a
small pure-Rust software rasterizer (`minifb` + `glam`), so it does not need
OpenGL, Vulkan, a GPU, or a vendor SDK.

This is the first working slice of the larger NX reader. It is intentionally
split from NX UI/CAM/CAE semantics: opening and rendering a part does not
require the NX application, and the file format is decoded from bytes only.

## Status (NX 2412 samples)

| Input | Result |
|---|---|
| Legacy CFB `facetted_hood.prt` | 18 bodies, 93 faces, 266 edges; **48 faces tessellated / 45 refused** |
| Legacy CFB `glass.prt` | 7 surfaces, 6 curves; topology graph incomplete, no face mesh |
| Legacy CFB assembly `ufun_disp_mattex6_test.prt` | external child-part references; legacy JT index decoded, geometry is JT type-31/ULP and not yet meshed |
| Legacy CFB CAM `hsm_setup.prt` | 8 Path OM sections, **25,124 moves** (7,627 linear / 17,449 circular / 48 helical); operations `RESTMILL_ALL`, `ZLEVEL_POSTS`; CAM points overlay in the viewer |

The 45 refusals are concentrated in faces whose loops do not close in the
surface parameter plane or whose procedural edge has no evaluable parameter.
These are codec/tessellator gaps, not container gaps. The next two fixes are:

1. Reconcile pcurve parameter ranges with the coedge traversal (seam and
   reversed-pcurve cases) so loops close exactly.
2. Extend procedural curve evaluation for legacy `edge-parametric` curves
   (surface-parametric and intersection constructions) to cover the remaining
   edges.

## Build

Requires Rust 1.88+. The viewer has no GPU dependency; it only needs a
desktop window (Win32, X11/Wayland, or Cocoa).

```sh
cargo build -p nxview --release
```

## Usage

```sh
# Container and model census, no tessellation
nxview info part.prt

# Tessellate B-rep faces and write part.obj (vertices + normals + triangles)
nxview obj part.prt

# Open an interactive orbit viewer
nxview view part.prt

# Extract NX CAM move endpoints from legacy CFB Path OM sections
nxview cam part.prt
```

`cam` writes `<name>.cam.ply` (millimetre point cloud) and reports the
`PATH_3x_linear` / `PATH_3x_circular` / `PATH_3x_helical` move counts and
operation-like names. `view` overlays the same CAM points in orange on the CAD
mesh.

Controls in the viewer: left-drag orbit, wheel zoom, Esc close.

`NXVIEW_CHORD` controls the tessellation chord tolerance (millimetres). The
viewer defaults to `0.5`; `obj` defaults to `0.01`. Lower is finer and slower:

```sh
NXVIEW_CHORD=2.0 nxview view part.prt
```

## Architecture

```
.prt bytes
  │
  ├─ CFB (legacy) / SPLMSSTR (modern) container
  │     └─ /Root/UG_PART/UG_PART
  │           ├─ NX object-model sections
  │           └─ Parasolid partition/deltas streams (zlib)
  │
  ├─ Parasolid X_B decode ──► cadmpeg-ir B-rep (bodies → faces → loops → edges)
  │                              └─ cadmpeg-tessellate → triangles
  │                                    └─ software rasterizer (z-buffer, orbit camera)
  │
  ├─ embedded JT (DisplayJT / legacy UG_PART/JT)
  │     └─ JT 9.x LSG + shape LOD / tri-strip or ULP
  │
  └─ assets: preview JPEG, attrs XML, materials, external references
```

`nxview` depends on the `cadmpeg-codec-nx` codec and the
`cadmpeg-tessellate` crate. The local fork currently carries two fixes:

- Legacy CFB `UG_PART/JT` / `UG_PART/DisplayJT` index support (4-byte row
  variant) in the NX codec.
- Procedural curve interval lookup in the tessellator (surface-parametric and
  intersection constructions), which raised the sample from 14 to 48 faces.

## Coverage and roadmap

- **P0 container** — CFB and SPLMSSTR directory/stream handling: done.
- **P0 B-rep** — Parasolid decode, analytic/NURBS carriers, topology: partial,
  driven by `cadmpeg-codec-nx`.
- **P1 tessellation** — planar/analytic/NURBS face tessellation: partial;
  pcurve-loop closure and legacy procedural curves remain.
- **P1 display JT** — modern DisplayJT path supported by the codec; legacy
  type-31/ULP geometry needs a decoder.
- **P1 assembly** — external references and placement frames are retained;
  child-part resolution and occurrence transforms are not yet composed.
- **P2 CAM** — legacy Path OM sections: 3D move extraction and overlay done
  (point cloud). Operation ordering, circular/helical arc interpolation, tool
  and setup records, and modern SPLMSSTR Path sections remain.
- **P2 CAE** — mesh/solver/result records are separate object-model families;
  the current IR has no FEM arena.
- **P3 UI** — out of scope for file opening. NX UI state is stored in streams;
  reimplementing NX's UI is a separate application project.

## Legal / provenance

`cadmpeg` is Apache-2.0 and uses clean-room rules: format knowledge comes from
legally possessed files and public documentation, not vendor SDKs or
decompiled binaries. Do not use this project to bypass NX licensing or
activation. Parasolid is a separately licensed kernel; this reader parses
transmit data and does not link or redistribute Parasolid.
