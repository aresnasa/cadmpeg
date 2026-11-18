# cadmpeg-codec-nx

`cadmpeg-codec-nx` reads Siemens NX `.prt` files stored as SPLMSSTR containers
into [`CadIr`][ir]. It detects the container by its `SPLMSSTR` signature,
extracts zlib-compressed Parasolid neutral-binary streams from the canonical
part payload, and decodes supported geometry and topology.

<!-- generated: capability nx -->

Support: L1 ([ladder](https://github.com/cadmpeg/cadmpeg/blob/main/docs/format-support.md#siemens-nx-prt)).

<!-- /generated: capability nx -->

Connected B-rep on selected or terminal-lineage-resolved body images shows as
extras. `RMFastLoad` retains every body whose complete
nonempty topology node-ID set is covered by the active object-ID set; when no
body has complete membership, selection declines and terminal lineage can
resolve the images.

## Install

```sh
cargo add cadmpeg-codec-nx cadmpeg-ir
```

## Decode

```rust,no_run
use cadmpeg_codec_nx::NxCodec;
use cadmpeg_ir::{Codec, DecodeOptions};
use std::fs::File;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut input = File::open("part.prt")?;
    let result = NxCodec.decode(&mut input, &DecodeOptions::default())?;

    println!(
        "{} bodies, {} surfaces",
        result.ir().model.bodies.len(),
        result.ir().model.surfaces.len()
    );
    Ok(())
}
```

The result holds the decoded `CadIr` and a [`DecodeReport`][report]. Read
`report.losses` before trusting geometry. Call `NxCodec::inspect` for container
metadata without entity decode. It lists SPLMSSTR directory entries and
classifies embedded streams as partition, deltas, plain cached body, or preview
data. Set `DecodeOptions::depth` to `DecodeDepth::Container` for metadata IR
without entity decoding.

Run the capability profiler against a directory of `.prt` files for a
deterministic JSON census:

```sh
cargo run -p cadmpeg-codec-nx --bin nx_profile -- FIXTURES OUTPUT.json
```

## Byte-preserving edits

Legacy CFB and SPLMSSTR parts expose the `part/attrs` XML writer:

```rust
use cadmpeg_codec_nx::set_part_attribute;

set_part_attribute(
    std::path::Path::new("in.prt"),
    std::path::Path::new("out.prt"),
    "DB_PART_DESC",
    "REV B",
)?;
```

Assembly relinking replaces only the trailing `ExternalReferences` string
table and keeps every indexed occurrence record, every other stream, and the
geometry bytes unchanged:

```rust
use cadmpeg_codec_nx::{read_assembly_manifest, relink_assembly};

let manifest = read_assembly_manifest(std::path::Path::new("assembly.prt"), None)?;
println!("{} components", manifest.components.len());
relink_assembly(
    std::path::Path::new("assembly.prt"),
    std::path::Path::new("assembly_relinked.prt"),
    std::path::Path::new(r"D:\parts"),
)?;
```

CFB output rebuilds the compound file and preserves each storage's CLSID,
creation time, modification time, and state bits. Stream payloads are
unchanged; a repeated edit of the same input produces the same output.
SPLMSSTR output repacks the directory, shifts later payloads, and recomputes
the header and footer XXH32 checksums. A no-op relink copies the source byte
for byte.

The decoder does not infer an occurrence 4x4 placement when the assembly
stores only child names and paths.

## Data model

NX stores part geometry in one or more Parasolid streams inside an SPLMSSTR
container. The decoder inflates each stream, converts Parasolid metre values to
the millimetre-based IR, and retains the inflated stream as an unknown record
for provenance and passthrough.

Typed transfer covers analytic carriers, NURBS, selected trimmed curves, and
connected topology when fixed-record references resolve. Geometry that cannot
be attached remains available through derived free topology. Partitions retain
their own input. Every paired or unpaired Deltas stream contributes no semantic
bytes while aggregate selection remains unresolved. A sequential source owner
retains supported physical events, shared schema metadata, generic field values,
and the exact suffix at its first unsupported boundary. `RMFastLoad` intersects
membership IDs with topology node
IDs and retains every image whose complete nonempty set is covered; otherwise
selection declines and complete primary-writer lineage falls back to terminal
partition images. Assembly files can contain only external child-part
references and produce no inline geometry.

Complete assembly placement frames transfer to the native `assembly_placements`
arena with exact bytes, row-major matrices, labels, identity texts, and source
offsets. NX 10 stores an opaque identity string where NX 12 and later store a
canonical UUID. Native validation checks the derived fields against those
bytes. Translation values retain their serialized unit; child-occurrence
binding and unit assignment remain unresolved.

Embedded JT shape-LOD segments transfer as display tessellation. A mesh that
declares a normal lane absent from the decoded vertex records transfers without
a normal lane. Validated
embedded JPEG previews and TIFF material textures transfer as exact document
assets. Ordered feature-operation records, body dependencies, Boolean
operations, sketch record lanes, named arrangements and configurations, part
attributes, external dependency inspection, and numeric expressions transfer.
Coverage detail lives in the [format-support profile][support]. Byte semantics
live in the [format notes][spec].

## Documentation

- [API documentation][docs]
- [Format support][support]
- [Format notes][spec]
- [Clean-room and legal policy][legal]
- [Repository][repo]

Requires Rust 1.88 or later. Licensed under Apache-2.0.

Siemens NX and Parasolid and other product names are trademarks of their
respective owners. cadmpeg uses them only to identify the file formats this
codec targets and is not affiliated with, endorsed by, or sponsored by any CAD
vendor. See the [clean-room and legal policy][legal].

[docs]: https://docs.rs/cadmpeg-codec-nx
[ir]: https://docs.rs/cadmpeg-ir/latest/cadmpeg_ir/document/struct.CadIr.html
[legal]: https://github.com/cadmpeg/cadmpeg/blob/main/LEGAL.md
[repo]: https://github.com/cadmpeg/cadmpeg
[report]: https://docs.rs/cadmpeg-ir/latest/cadmpeg_ir/report/struct.DecodeReport.html
[spec]: https://github.com/cadmpeg/cadmpeg/blob/main/docs/formats/siemens_nx.md
[support]: https://github.com/cadmpeg/cadmpeg/blob/main/docs/format-support.md#siemens-nx-prt
