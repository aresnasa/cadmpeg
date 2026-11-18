// SPDX-License-Identifier: Apache-2.0
//! Cross-platform NX `.prt` probe: decode, tessellate, export OBJ, or view
//! the result with a small pure-Rust software viewer.

use std::fmt::Write as _;
use std::fs::{self, File};
use std::io::{self, BufRead, Write as _};
use std::path::{Path, PathBuf};

use anyhow::{bail, Context};
use cadmpeg_codec_nx::{
    assembly_placements, read_assembly_manifest, read_part_attributes, relink_assembly,
    set_part_attribute, NxCodec,
};
use cadmpeg_ir::codec::{Codec, DecodeOptions};
use cadmpeg_ir::math::{Point3, Vector3};
use cadmpeg_tessellate::{Options, Tessellator};
use glam::camera::rh::{proj::opengl, view};
use glam::{Vec3, Vec4};
use serde_json::json;

#[cfg(feature = "viewer")]
use minifb::{Key, KeyRepeat, MouseButton, MouseMode, Window, WindowOptions};

mod assembly_mesh;
mod cam;

#[derive(Default)]
pub(crate) struct MeshData {
    pub(crate) positions: Vec<[f32; 3]>,
    pub(crate) normals: Vec<[f32; 3]>,
    pub(crate) indices: Vec<u32>,
    pub(crate) cam_points: Vec<[f32; 3]>,
    pub(crate) cam_sections: Vec<usize>,
    pub(crate) cam_edges: Vec<[u32; 2]>,
    pub(crate) cam_rapid: Vec<bool>,
    pub(crate) faces_tessellated: usize,
    pub(crate) faces_refused: usize,
}

impl MeshData {
    pub(crate) fn add(
        &mut self,
        positions: &[Point3],
        normals: &[Vector3],
        triangles: &[[u32; 3]],
        source: &str,
    ) {
        if positions.is_empty() || triangles.is_empty() {
            return;
        }
        let base = u32::try_from(self.positions.len()).unwrap_or(u32::MAX);
        if base == u32::MAX {
            return;
        }
        for point in positions {
            self.positions
                .push([point.x as f32, point.y as f32, point.z as f32]);
        }
        if normals.len() == positions.len() {
            for normal in normals {
                self.normals
                    .push([normal.x as f32, normal.y as f32, normal.z as f32]);
            }
        } else {
            let computed = compute_normals(&self.positions[base as usize..], triangles);
            self.normals.extend(computed);
        }
        for triangle in triangles {
            self.indices.push(base + triangle[0]);
            self.indices.push(base + triangle[1]);
            self.indices.push(base + triangle[2]);
        }
        self.faces_tessellated += 1;
        let _ = source;
    }
}

fn compute_normals(positions: &[[f32; 3]], triangles: &[[u32; 3]]) -> Vec<[f32; 3]> {
    let mut normals = vec![[0.0_f32; 3]; positions.len()];
    for triangle in triangles {
        let [a, b, c] = triangle.map(|index| index as usize);
        let (Some(a), Some(b), Some(c)) = (positions.get(a), positions.get(b), positions.get(c))
        else {
            continue;
        };
        let u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
        let v = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
        let n = [
            u[1] * v[2] - u[2] * v[1],
            u[2] * v[0] - u[0] * v[2],
            u[0] * v[1] - u[1] * v[0],
        ];
        for index in triangle {
            let normal = &mut normals[*index as usize];
            normal[0] += n[0];
            normal[1] += n[1];
            normal[2] += n[2];
        }
    }
    for normal in &mut normals {
        let length = (normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]).sqrt();
        if length > 0.0 {
            normal[0] /= length;
            normal[1] /= length;
            normal[2] /= length;
        } else {
            *normal = [0.0, 1.0, 0.0];
        }
    }
    normals
}

fn main() -> anyhow::Result<()> {
    #[cfg(all(windows, feature = "viewer"))]
    unsafe {
        // minifb does not set DPI awareness. Without this, a 1100x700 window
        // is scaled to 2200x1400 physical pixels on a 200% display and the
        // model appears outside the visible screen area.
        windows_sys::Win32::UI::WindowsAndMessaging::SetProcessDPIAware();
    }

    let mut args = std::env::args().skip(1);
    let first = args.next().unwrap_or_else(|| "info".to_string());
    match first.as_str() {
        "info" => {
            let paths: Vec<PathBuf> = args.map(PathBuf::from).collect();
            if paths.is_empty() {
                bail!("usage: nxview info <part.prt>...");
            }
            for path in paths {
                info(&path)?;
            }
        }
        "obj" => {
            let paths: Vec<PathBuf> = args.map(PathBuf::from).collect();
            if paths.is_empty() {
                bail!("usage: nxview obj <part.prt>...");
            }
            for path in paths {
                let mesh = load_mesh(&path, chord(0.01))?;
                // Write into the current directory, not next to the (often
                // read-only) NX installation.
                let obj_path = PathBuf::from(
                    path.file_name()
                        .unwrap_or_default()
                        .to_string_lossy()
                        .to_string(),
                )
                .with_extension("obj");
                write_obj(&obj_path, &mesh)?;
                println!("wrote {}", obj_path.display());
            }
        }
        #[cfg(feature = "viewer")]
        "view" => {
            let path = args.next().context("usage: nxview view <part.prt>")?;
            let path = PathBuf::from(path);
            let mesh = load_mesh(&path, chord(0.5))?;
            if mesh.indices.is_empty() && mesh.cam_points.is_empty() {
                bail!(
                    "{} has no renderable B-rep, JT, or CAM geometry",
                    path.display()
                );
            }
            view(&path, &mesh)?;
        }
        #[cfg(not(feature = "viewer"))]
        "view" => {
            bail!("viewer not compiled: build with --features viewer")
        }
        "edit" => {
            let path = PathBuf::from(args.next().context("usage: nxview edit <part.prt>")?);
            edit_interactive(&path)?;
        }
        "cam" => {
            let path = args.next().context("usage: nxview cam <part.prt>")?;
            let path = PathBuf::from(path);
            let cam = cam::load(&path)?;
            let (min, max) = cam.bounds();
            println!(
                "path-sections={} moves={} linear={} circular={} helical={}",
                cam.path_sections,
                cam.moves.len(),
                cam.linear(),
                cam.circular(),
                cam.helical()
            );
            println!(
                "bbox=({}, {}, {})..({}, {}, {})",
                min[0], min[1], min[2], max[0], max[1], max[2]
            );
            println!("operations={:?}", cam.operations);
            for section in &cam.sections {
                println!(
                    "  section {} offset={:#x} moves={} operations={:?}",
                    section.index, section.stream_offset, section.moves, section.operations
                );
            }
            let ply_path = PathBuf::from(
                path.file_name()
                    .unwrap_or_default()
                    .to_string_lossy()
                    .to_string(),
            )
            .with_extension("cam.ply");
            cam.write_ply(&ply_path)?;
            println!("wrote {}", ply_path.display());
        }
        "json" => {
            let path = PathBuf::from(args.next().context("usage: nxview json <part.prt>")?);
            let mut input = File::open(&path)?;
            let result = NxCodec.decode(&mut input, &DecodeOptions::default())?;
            let ir = result.ir();
            let attributes = cadmpeg_codec_nx::read_part_attributes(&path).unwrap_or_default();
            let placements = cadmpeg_codec_nx::assembly_placements(ir).unwrap_or_default();
            let losses: Vec<String> = result
                .report()
                .losses
                .iter()
                .map(|loss| format!("{:?}: {}", loss.severity, loss.message))
                .collect();
            let value = json!({
                "path": path.display().to_string(),
                "container": container_kind(&path),
                "bodies": ir.model.bodies.len(),
                "regions": ir.model.regions.len(),
                "shells": ir.model.shells.len(),
                "faces": ir.model.faces.len(),
                "edges": ir.model.edges.len(),
                "vertices": ir.model.vertices.len(),
                "surfaces": ir.model.surfaces.len(),
                "curves": ir.model.curves.len(),
                "tessellations": ir.model.tessellations.len(),
                "features": ir.model.features.len(),
                "attributes_count": ir.model.attributes.len(),
                "occurrences": ir.model.occurrences.len(),
                "attributes": attributes
                    .iter()
                    .map(|attribute| json!({
                        "title": attribute.title,
                        "value": attribute.value,
                    }))
                    .collect::<Vec<_>>(),
                "placements": placements
                    .iter()
                    .map(|placement| json!({
                        "label": placement.label,
                        "uuid": placement.uuid,
                        "matrix": placement.matrix,
                        "source_offset": placement.source_offset,
                    }))
                    .collect::<Vec<_>>(),
                "losses": losses,
            });
            println!("{}", serde_json::to_string_pretty(&value)?);
        }
        "render" => {
            let path = PathBuf::from(
                args.next()
                    .context("usage: nxview render <part.prt> <out.ppm> [width] [height]")?,
            );
            let output = PathBuf::from(
                args.next()
                    .context("usage: nxview render <part.prt> <out.ppm> [width] [height]")?,
            );
            let width = args
                .next()
                .and_then(|value| value.parse::<usize>().ok())
                .unwrap_or(1100);
            let height = args
                .next()
                .and_then(|value| value.parse::<usize>().ok())
                .unwrap_or(700);
            if width == 0 || height == 0 || width > 8192 || height > 8192 {
                bail!("render dimensions must be between 1 and 8192");
            }
            let buffer = render_image(&path, width, height)?;
            write_ppm(&output, &buffer, width, height)?;
            println!("wrote {} ({}x{})", output.display(), width, height);
        }
        "placements" => {
            let path = PathBuf::from(args.next().context("usage: nxview placements <part.prt>")?);
            let mut input = File::open(&path)?;
            let result = NxCodec.decode(&mut input, &DecodeOptions::default())?;
            let placements =
                cadmpeg_codec_nx::assembly_placements(result.ir()).map_err(anyhow::Error::msg)?;
            println!("placements={}", placements.len());
            for placement in placements {
                println!(
                    "{} uuid={} offset={:#x}",
                    placement.label, placement.uuid, placement.source_offset
                );
                for row in 0..3 {
                    let base = row * 4;
                    println!(
                        "  [{}, {}, {}, {}]",
                        placement.matrix[base],
                        placement.matrix[base + 1],
                        placement.matrix[base + 2],
                        placement.matrix[base + 3]
                    );
                }
            }
        }
        "assembly" => {
            assembly_command(&mut args)?;
        }
        "gear" => {
            let path = PathBuf::from(args.next().context("usage: nxview gear <part.prt>")?);
            gear_report(&path)?;
        }
        "gear-profile" => {
            let path = PathBuf::from(
                args.next()
                    .context("usage: nxview gear-profile <part.prt> [bins]")?,
            );
            let bins = args
                .next()
                .and_then(|value| value.parse::<usize>().ok())
                .unwrap_or(1440);
            gear_profile_report(&path, bins)?;
        }
        "surfaces" => {
            let path = PathBuf::from(args.next().context("usage: nxview surfaces <part.prt>")?);
            let mut input = File::open(&path)?;
            let result = NxCodec.decode(&mut input, &DecodeOptions::default())?;
            let mut surfaces = Vec::new();
            for surface in &result.ir().model.surfaces {
                let geometry = surface.geometry.solved().map_or_else(
                    || serde_json::to_value(&surface.geometry),
                    serde_json::to_value,
                )?;
                surfaces.push(json!({
                    "id": surface.id.as_str(),
                    "source_object": surface.source_object,
                    "geometry": geometry,
                }));
            }
            println!("{}", serde_json::to_string_pretty(&surfaces)?);
        }
        "attr" => {
            let action = args.next().context(
                "usage: nxview attr get <part.prt> | nxview attr set <in.prt> <out.prt> <title> <value>",
            )?;
            match action.as_str() {
                "get" => {
                    let path =
                        PathBuf::from(args.next().context("usage: nxview attr get <part.prt>")?);
                    for attribute in cadmpeg_codec_nx::read_part_attributes(&path)? {
                        println!("{}\t{}", attribute.title, attribute.value);
                    }
                }
                "apply" => {
                    let input = PathBuf::from(
                        args.next()
                            .context("usage: nxview attr apply <in.prt> <out.prt> <edits.json>")?,
                    );
                    let output = PathBuf::from(
                        args.next()
                            .context("usage: nxview attr apply <in.prt> <out.prt> <edits.json>")?,
                    );
                    let edits_path = PathBuf::from(
                        args.next()
                            .context("usage: nxview attr apply <in.prt> <out.prt> <edits.json>")?,
                    );
                    let edits: serde_json::Map<String, serde_json::Value> =
                        serde_json::from_str(&fs::read_to_string(&edits_path)?)?;
                    let edits: Vec<(String, String)> = edits
                        .into_iter()
                        .map(|(title, value)| {
                            let value = match value {
                                serde_json::Value::String(text) => text,
                                other => other.to_string(),
                            };
                            (title, value)
                        })
                        .collect();
                    apply_pending(&input, &output, &edits)?;
                    println!("wrote {}", output.display());
                }
                "set" => {
                    let input = PathBuf::from(
                        args.next()
                            .context("usage: nxview attr set <in.prt> <out.prt> <title> <value>")?,
                    );
                    let output = PathBuf::from(
                        args.next()
                            .context("usage: nxview attr set <in.prt> <out.prt> <title> <value>")?,
                    );
                    let title = args
                        .next()
                        .context("usage: nxview attr set <in.prt> <out.prt> <title> <value>")?;
                    let value = args
                        .next()
                        .context("usage: nxview attr set <in.prt> <out.prt> <title> <value>")?;
                    let report =
                        cadmpeg_codec_nx::set_part_attribute(&input, &output, &title, &value)?;
                    println!("wrote {}", report.output.display());
                    println!("changed_streams={:?}", report.changed_streams);
                    println!("source_sha256={}", report.source_sha256);
                    println!("output_sha256={}", report.output_sha256);
                    println!("byte_identical={}", report.byte_identical);
                }
                other => bail!("unknown attr action: {other}"),
            }
        }
        "stream" => {
            let action = args.next().context(
                "usage: nxview stream replace <in.prt> <out.prt> <stream-path> <payload-file>",
            )?;
            if action != "replace" {
                bail!(
                    "usage: nxview stream replace <in.prt> <out.prt> <stream-path> <payload-file>"
                );
            }
            let input = PathBuf::from(args.next().context(
                "usage: nxview stream replace <in.prt> <out.prt> <stream-path> <payload-file>",
            )?);
            let output = PathBuf::from(args.next().context(
                "usage: nxview stream replace <in.prt> <out.prt> <stream-path> <payload-file>",
            )?);
            let stream = args.next().context(
                "usage: nxview stream replace <in.prt> <out.prt> <stream-path> <payload-file>",
            )?;
            let payload_path = PathBuf::from(args.next().context(
                "usage: nxview stream replace <in.prt> <out.prt> <stream-path> <payload-file>",
            )?);
            let payload = fs::read(&payload_path)?;
            let report = cadmpeg_codec_nx::replace_cfb_stream(&input, &output, &stream, &payload)?;
            println!("wrote {}", report.output.display());
            println!("changed_streams={:?}", report.changed_streams);
            println!("source_sha256={}", report.source_sha256);
            println!("output_sha256={}", report.output_sha256);
        }
        other => {
            // Treat the first argument as a path for convenience.
            let path = PathBuf::from(other);
            info(&path)?;
        }
    }
    Ok(())
}

fn assembly_command(args: &mut impl Iterator<Item = String>) -> anyhow::Result<()> {
    let action = args
        .next()
        .context("usage: nxview assembly <info|export|relink|check|obj|render> ...")?;
    match action.as_str() {
        "info" => {
            let mut path = None;
            let mut root = None;
            while let Some(argument) = args.next() {
                match argument.as_str() {
                    "--root" => {
                        root = Some(PathBuf::from(
                            args.next().context("usage: --root <directory>")?,
                        ));
                    }
                    other => {
                        if path.replace(PathBuf::from(other)).is_some() {
                            bail!("usage: nxview assembly info <assembly.prt> [--root <dir>]");
                        }
                    }
                }
            }
            let path = path.context("usage: nxview assembly info <assembly.prt> [--root <dir>]")?;
            print_manifest(&path, root.as_deref())?;
        }
        "export" => {
            let path = PathBuf::from(args.next().context(
                "usage: nxview assembly export <assembly.prt> <out.json> [--root <dir>]",
            )?);
            let output = PathBuf::from(args.next().context(
                "usage: nxview assembly export <assembly.prt> <out.json> [--root <dir>]",
            )?);
            let mut root = None;
            while let Some(argument) = args.next() {
                if argument == "--root" {
                    root = Some(PathBuf::from(
                        args.next().context("usage: --root <directory>")?,
                    ));
                } else {
                    bail!("usage: nxview assembly export <assembly.prt> <out.json> [--root <dir>]");
                }
            }
            let manifest = read_assembly_manifest(&path, root.as_deref())?;
            let value = manifest_json(&manifest);
            fs::write(&output, serde_json::to_string_pretty(&value)?)?;
            println!("wrote {}", output.display());
        }
        "check" => {
            let (positionals, root) = assembly_positionals(args)?;
            let path = positionals
                .first()
                .context("usage: nxview assembly check <assembly.prt> [--root <dir>]")?;
            let loaded =
                crate::assembly_mesh::load_assembly_mesh(path, root.as_deref(), chord(0.5))?;
            println!("{}", serde_json::to_string_pretty(&loaded.report)?);
        }
        "obj" => {
            let (positionals, root) = assembly_positionals(args)?;
            let path = positionals
                .first()
                .context("usage: nxview assembly obj <assembly.prt> <out.obj> [--root <dir>]")?;
            let output = positionals
                .get(1)
                .context("usage: nxview assembly obj <assembly.prt> <out.obj> [--root <dir>]")?;
            let loaded =
                crate::assembly_mesh::load_assembly_mesh(path, root.as_deref(), chord(0.5))?;
            if loaded.mesh.indices.is_empty() {
                bail!("assembly has no renderable geometry");
            }
            write_obj(output, &loaded.mesh)?;
            println!("wrote {}", output.display());
            println!("{}", serde_json::to_string_pretty(&loaded.report)?);
        }
        "render" => {
            let (positionals, root) = assembly_positionals(args)?;
            let path = positionals.first().context(
                "usage: nxview assembly render <assembly.prt> <out.ppm> [width] [height] [--root <dir>]",
            )?;
            let output = positionals.get(1).context(
                "usage: nxview assembly render <assembly.prt> <out.ppm> [width] [height] [--root <dir>]",
            )?;
            let width = positionals
                .get(2)
                .and_then(|value| value.to_str())
                .and_then(|value| value.parse::<usize>().ok())
                .unwrap_or(1100);
            let height = positionals
                .get(3)
                .and_then(|value| value.to_str())
                .and_then(|value| value.parse::<usize>().ok())
                .unwrap_or(700);
            if width == 0 || height == 0 || width > 8192 || height > 8192 {
                bail!("render dimensions must be between 1 and 8192");
            }
            let loaded =
                crate::assembly_mesh::load_assembly_mesh(path, root.as_deref(), chord(0.5))?;
            let buffer = render_mesh_image(&loaded.mesh, width, height)?;
            write_ppm(output, &buffer, width, height)?;
            println!("wrote {} ({}x{})", output.display(), width, height);
            println!("{}", serde_json::to_string_pretty(&loaded.report)?);
        }
        "relink" => {
            let input = PathBuf::from(
                args.next()
                    .context("usage: nxview assembly relink <in.prt> <out.prt> <directory>")?,
            );
            let output = PathBuf::from(
                args.next()
                    .context("usage: nxview assembly relink <in.prt> <out.prt> <directory>")?,
            );
            let directory = PathBuf::from(
                args.next()
                    .context("usage: nxview assembly relink <in.prt> <out.prt> <directory>")?,
            );
            let report = relink_assembly(&input, &output, &directory)?;
            println!("wrote {}", report.output.display());
            println!("stream={}", report.stream);
            println!(
                "old_directory={}",
                report.old_directory.as_deref().unwrap_or("")
            );
            println!("new_directory={}", report.new_directory);
            println!("changed_strings={}", report.changed_strings.len());
            println!("source_sha256={}", report.source_sha256);
            println!("output_sha256={}", report.output_sha256);
            println!("byte_identical={}", report.byte_identical);
            let manifest = read_assembly_manifest(&output, Some(&directory))?;
            let present = manifest
                .components
                .iter()
                .filter(|component| component.present)
                .count();
            println!(
                "components={} present={} missing={}",
                manifest.components.len(),
                present,
                manifest.components.len() - present
            );
        }
        other => bail!("unknown assembly action: {other}"),
    }
    Ok(())
}

fn assembly_positionals(
    args: &mut impl Iterator<Item = String>,
) -> anyhow::Result<(Vec<PathBuf>, Option<PathBuf>)> {
    let mut positionals = Vec::new();
    let mut root = None;
    while let Some(argument) = args.next() {
        if argument == "--root" {
            root = Some(PathBuf::from(
                args.next().context("usage: --root <directory>")?,
            ));
        } else {
            positionals.push(PathBuf::from(argument));
        }
    }
    Ok((positionals, root))
}

fn print_manifest(path: &Path, root: Option<&Path>) -> anyhow::Result<()> {
    let manifest = read_assembly_manifest(path, root)?;
    println!("source={}", manifest.source.display());
    println!("stream={}", manifest.stream);
    println!(
        "directory={}",
        manifest.directory.as_deref().unwrap_or("(none)")
    );
    println!("components={}", manifest.components.len());
    for component in &manifest.components {
        println!(
            "  [{}] {} present={} path={}",
            component.string_ordinal,
            component.name,
            component.present,
            component.path.display()
        );
    }
    Ok(())
}

fn manifest_json(manifest: &cadmpeg_codec_nx::AssemblyManifest) -> serde_json::Value {
    json!({
        "source": manifest.source.display().to_string(),
        "stream": manifest.stream,
        "directory": manifest.directory,
        "components": manifest.components.iter().map(|component| json!({
            "name": component.name,
            "string_ordinal": component.string_ordinal,
            "path": component.path.display().to_string(),
            "present": component.present,
        })).collect::<Vec<_>>(),
        "strings": manifest.strings,
    })
}

fn load_vertices(path: &Path) -> anyhow::Result<Vec<[f64; 3]>> {
    let mut input = File::open(path)?;
    let result = NxCodec.decode(&mut input, &DecodeOptions::default())?;
    let mut vertices = Vec::new();
    for tessellation in &result.ir().model.tessellations {
        for point in tessellation.vertices() {
            let point = point.get();
            vertices.push([point.x, point.y, point.z]);
        }
    }
    if vertices.is_empty() && !result.ir().model.faces.is_empty() {
        let options = Options {
            chord: 0.5,
            ..Options::default()
        };
        let mut tessellator = Tessellator::new(result.ir(), options);
        for face in &result.ir().model.faces {
            if let Ok(face_mesh) = tessellator.face(&face.id) {
                for point in face_mesh.positions {
                    vertices.push([point.x, point.y, point.z]);
                }
            }
        }
    }
    if vertices.is_empty() {
        bail!("{} has no tessellation vertices", path.display());
    }
    Ok(vertices)
}

fn gear_report(path: &Path) -> anyhow::Result<()> {
    let vertices = load_vertices(path)?;
    let report = analyze_gear(&vertices).context("no periodic gear profile found")?;
    let value = json!({
        "path": path.display().to_string(),
        "vertices": vertices.len(),
        "axis": report.axis,
        "center": report.center,
        "teeth": report.teeth,
        "tip_radius": report.tip_radius,
        "root_radius": report.root_radius,
        "face_width": report.face_width,
        "face_min": report.face_min,
        "face_max": report.face_max,
        "tooth_ratio": report.tooth_ratio,
        "periodicity": report.periodicity,
    });
    println!("{}", serde_json::to_string_pretty(&value)?);
    Ok(())
}

struct GearReport {
    axis: [f64; 3],
    center: [f64; 3],
    teeth: usize,
    tip_radius: f64,
    root_radius: f64,
    face_width: f64,
    face_min: f64,
    face_max: f64,
    tooth_ratio: f64,
    periodicity: f64,
}

fn folded_tooth_count(profile: &[f64; 720], root_radius: f64) -> Option<(usize, f64)> {
    const SLOTS: usize = 24;
    let mut candidates = Vec::new();
    for teeth in 6..=40usize {
        let pitch = 720.0 / teeth as f64;
        let mut sums = [0.0_f64; SLOTS];
        let mut counts = [0usize; SLOTS];
        for (index, profile_value) in profile.iter().enumerate() {
            let value = if *profile_value > 0.0 {
                *profile_value
            } else {
                root_radius
            };
            let phase = (index as f64 / pitch).fract();
            let slot = ((phase * SLOTS as f64) as usize).min(SLOTS - 1);
            sums[slot] += value;
            counts[slot] += 1;
        }
        let mut minimum = f64::INFINITY;
        let mut maximum = f64::NEG_INFINITY;
        for (sum, count) in sums.iter().zip(counts) {
            if count == 0 {
                continue;
            }
            let average = sum / count as f64;
            minimum = minimum.min(average);
            maximum = maximum.max(average);
        }
        candidates.push((teeth, maximum - minimum));
    }
    let maximum = candidates
        .iter()
        .map(|(_, contrast)| *contrast)
        .fold(0.0_f64, f64::max);
    let minimum = candidates
        .iter()
        .map(|(_, contrast)| *contrast)
        .fold(f64::INFINITY, f64::min);
    if maximum <= 0.25 * (profile.iter().fold(0.0_f64, |a, b| a.max(*b)) - root_radius) {
        return None;
    }
    // Divisors of the tooth count produce the same contrast; the largest
    // candidate near the maximum is the fundamental tooth count.
    candidates
        .into_iter()
        .filter(|(_, contrast)| *contrast >= maximum - 0.02 * (maximum - minimum).max(1.0e-9))
        .max_by_key(|(teeth, _)| *teeth)
}

fn analyze_gear(vertices: &[[f64; 3]]) -> Option<GearReport> {
    const BINS: usize = 720;
    let axes = [
        ([0.0, 0.0, 1.0], 0usize, 1usize, 2usize),
        ([0.0, 1.0, 0.0], 2usize, 0usize, 1usize),
        ([1.0, 0.0, 0.0], 1usize, 2usize, 0usize),
    ];
    let mut best: Option<(f64, GearReport)> = None;
    for (axis, u, v, w) in axes {
        let mut radial = Vec::new();
        let mut max_radius = 0.0_f64;
        for point in vertices {
            let radius = point[u].hypot(point[v]);
            radial.push((radius, point[w]));
            max_radius = max_radius.max(radius);
        }
        if max_radius <= 0.0 {
            continue;
        }
        let floor = 0.65 * max_radius;
        let mut min_tooth = f64::INFINITY;
        for &(radius, _) in &radial {
            if radius > floor {
                min_tooth = min_tooth.min(radius);
            }
        }
        let root_radius = dominant_radius(radial.iter().map(|(radius, _)| *radius), max_radius)
            .unwrap_or(min_tooth);
        if !root_radius.is_finite() || max_radius / root_radius < 1.08 {
            continue;
        }
        let threshold = 0.5 * (root_radius + max_radius);
        let mut profile = [0.0_f64; BINS];
        for point in vertices {
            let du = point[u];
            let dv = point[v];
            let radius = du.hypot(dv);
            if radius <= threshold {
                continue;
            }
            let angle = dv.atan2(du).rem_euclid(std::f64::consts::TAU);
            let bin = ((angle / std::f64::consts::TAU) * BINS as f64) as usize % BINS;
            if radius > profile[bin] {
                profile[bin] = radius;
            }
        }
        let mask: Vec<bool> = profile.iter().map(|value| *value > threshold).collect();
        let mask_count = mask.iter().filter(|value| **value).count();
        if mask_count < BINS / 8 {
            continue;
        }
        let min_lag = BINS / 40;
        let max_lag = BINS / 6;
        let mut best_correlation = 0.0_f64;
        for lag in min_lag..=max_lag {
            let matches = (0..BINS)
                .filter(|index| mask[*index] && mask[(index + lag) % BINS])
                .count();
            let correlation = matches as f64 / mask_count as f64;
            best_correlation = best_correlation.max(correlation);
        }
        let Some((teeth, _)) = folded_tooth_count(&profile, root_radius) else {
            continue;
        };
        let tooth_ratio = max_radius / root_radius;
        if best_correlation < 0.8 || !(1.08..=1.65).contains(&tooth_ratio) {
            continue;
        }

        let mut min_u = f64::INFINITY;
        let mut max_u = f64::NEG_INFINITY;
        let mut min_v = f64::INFINITY;
        let mut max_v = f64::NEG_INFINITY;
        let mut min_w = f64::INFINITY;
        let mut max_w = f64::NEG_INFINITY;
        for point in vertices {
            let radius = point[u].hypot(point[v]);
            if radius > 0.8 * max_radius {
                min_u = min_u.min(point[u]);
                max_u = max_u.max(point[u]);
                min_v = min_v.min(point[v]);
                max_v = max_v.max(point[v]);
                min_w = min_w.min(point[w]);
                max_w = max_w.max(point[w]);
            }
        }
        let face_width = max_w - min_w;
        if !face_width.is_finite() || face_width <= 0.0 {
            continue;
        }
        let center = match axis {
            [0.0, 0.0, 1.0] => [0.5 * (min_u + max_u), 0.5 * (min_v + max_v), 0.0],
            [0.0, 1.0, 0.0] => [0.5 * (min_u + max_u), 0.0, 0.5 * (min_v + max_v)],
            _ => [0.0, 0.5 * (min_u + max_u), 0.5 * (min_v + max_v)],
        };
        let face_ratio = face_width / (2.0 * max_radius);
        let score = best_correlation + 0.25 * (tooth_ratio - 1.0) - 0.2 * face_ratio;
        let report = GearReport {
            axis,
            center,
            teeth,
            tip_radius: max_radius,
            root_radius,
            face_width,
            face_min: min_w,
            face_max: max_w,
            tooth_ratio,
            periodicity: best_correlation,
        };
        if best.as_ref().is_none_or(|(current, _)| score > *current) {
            best = Some((score, report));
        }
    }
    best.map(|(_, report)| report)
}

fn gear_profile_report(path: &Path, bins: usize) -> anyhow::Result<()> {
    let bins = bins.clamp(360, 4096);
    let vertices = load_vertices(path)?;
    let report = analyze_gear(&vertices).context("no periodic gear profile found")?;
    let (u, v, w) = match report.axis {
        [0.0, 0.0, 1.0] => (0usize, 1usize, 2usize),
        [0.0, 1.0, 0.0] => (2usize, 0usize, 1usize),
        _ => (1usize, 2usize, 0usize),
    };
    // The tooth end faces carry the dense profile; the side flanks are not
    // present in the B-rep tessellation of this NX10 corpus.
    let station_a = report.face_min;
    let station_b = report.face_max;
    let station_mid = report.face_min + 0.5 * report.face_width;
    let stations = [station_a, station_b];
    let mut profiles = [vec![0.0_f64; bins], vec![0.0_f64; bins]];
    let mut input = File::open(path)?;
    let result = NxCodec.decode(&mut input, &DecodeOptions::default())?;
    let options = Options {
        chord: 0.5,
        ..Options::default()
    };
    let mut tessellator = Tessellator::new(result.ir(), options);
    for face in &result.ir().model.faces {
        let Ok(face_mesh) = tessellator.face(&face.id) else {
            continue;
        };
        for triangle in &face_mesh.triangles {
            let points = [
                face_mesh.positions[triangle[0]],
                face_mesh.positions[triangle[1]],
                face_mesh.positions[triangle[2]],
            ];
            for (profile, station) in profiles.iter_mut().zip(stations) {
                section_triangle_into_profile(profile, &points, w, station, u, v, bins, &report);
            }
        }
    }
    for profile in &mut profiles {
        for value in profile {
            if *value <= 0.0 {
                *value = report.root_radius;
            }
        }
    }
    let mut radii_a = profiles[0].clone();
    let mut radii_b = profiles[1].clone();
    let profile_root = dominant_radius(radii_a.iter().chain(&radii_b).copied(), report.tip_radius)
        .unwrap_or(report.root_radius);
    fill_profile_gaps(&mut radii_a, profile_root);
    fill_profile_gaps(&mut radii_b, profile_root);
    let valid_a = radii_a.iter().fold(0.0_f64, |a, b| a.max(*b)) > 0.95 * report.tip_radius;
    let valid_b = radii_b.iter().fold(0.0_f64, |a, b| a.max(*b)) > 0.95 * report.tip_radius;
    let radii_mid: Vec<f64> = match (valid_a, valid_b) {
        (true, true) => radii_a
            .iter()
            .zip(&radii_b)
            .map(|(a, b)| 0.5 * (a + b))
            .collect(),
        (true, false) => radii_a.clone(),
        (false, true) => radii_b.clone(),
        (false, false) => vec![report.root_radius; bins],
    };
    let shift = if valid_a && valid_b {
        profile_shift(&radii_a, &radii_b, report.root_radius)
    } else {
        0
    };
    let pitch_radius = 0.5 * (report.tip_radius + report.root_radius);
    let helix_angle = if valid_a && valid_b && report.face_width > 0.0 {
        let arc = (shift as f64 / bins as f64) * std::f64::consts::TAU * pitch_radius;
        arc.atan2(report.face_width).to_degrees()
    } else {
        0.0
    };
    let value = json!({
        "path": path.display().to_string(),
        "bins": bins,
        "axis": report.axis,
        "center": report.center,
        "teeth": report.teeth,
        "tip_radius": report.tip_radius,
        "root_radius": profile_root,
        "face_min": report.face_min,
        "face_max": report.face_max,
        "face_width": report.face_width,
        "station_a": station_a,
        "station_b": station_b,
        "station_mid": station_mid,
        "valid_a": valid_a,
        "valid_b": valid_b,
        "radii_a": radii_a,
        "radii_b": radii_b,
        "radii_mid": radii_mid,
        "helix_shift_bins": shift,
        "helix_angle_deg": helix_angle,
    });
    println!("{}", serde_json::to_string(&value)?);
    Ok(())
}

fn section_triangle_into_profile(
    profile: &mut [f64],
    points: &[Point3; 3],
    w: usize,
    station: f64,
    u: usize,
    v: usize,
    bins: usize,
    report: &GearReport,
) {
    let distances = points.map(|point| match w {
        0 => point.x - station,
        1 => point.y - station,
        _ => point.z - station,
    });
    let mut intersections = Vec::new();
    for index in 0..3 {
        if distances[index].abs() <= 1.0e-9 {
            intersections.push(points[index]);
        }
    }
    for (first, second) in [(0usize, 1usize), (1, 2), (2, 0)] {
        if (distances[first] < -1.0e-9 && distances[second] > 1.0e-9)
            || (distances[first] > 1.0e-9 && distances[second] < -1.0e-9)
        {
            let t = distances[first] / (distances[first] - distances[second]);
            intersections.push(Point3 {
                x: points[first].x + t * (points[second].x - points[first].x),
                y: points[first].y + t * (points[second].y - points[first].y),
                z: points[first].z + t * (points[second].z - points[first].z),
            });
        }
    }
    for point in intersections {
        let (du, dv) = match (u, v) {
            (0, 1) => (point.x, point.y),
            (1, 2) => (point.y, point.z),
            _ => (point.z, point.x),
        };
        let radius = du.hypot(dv);
        if radius < 0.5 * report.tip_radius {
            continue;
        }
        let angle = dv.atan2(du).rem_euclid(std::f64::consts::TAU);
        let bin = ((angle / std::f64::consts::TAU) * bins as f64) as usize % bins;
        if radius > profile[bin] {
            profile[bin] = radius;
        }
    }
}

fn dominant_radius(values: impl Iterator<Item = f64>, tip_radius: f64) -> Option<f64> {
    let low = 0.6 * tip_radius;
    let high = 0.75 * tip_radius;
    let mut buckets: Vec<(i64, usize)> = Vec::new();
    for value in values {
        if !(low..=high).contains(&value) {
            continue;
        }
        let key = (value * 1000.0).round() as i64;
        if let Some((_, count)) = buckets.iter_mut().find(|(bucket, _)| *bucket == key) {
            *count += 1;
        } else {
            buckets.push((key, 1));
        }
    }
    buckets
        .into_iter()
        .max_by_key(|(_, count)| *count)
        .map(|(key, _)| key as f64 / 1000.0)
}

fn fill_profile_gaps(profile: &mut [f64], root_radius: f64) {
    const MAX_GAP: usize = 8;
    let bins = profile.len();
    let is_root = |value: f64| value <= root_radius * 1.02;
    for index in 0..bins {
        if !is_root(profile[index]) {
            continue;
        }
        let left = (1..=MAX_GAP)
            .map(|offset| (index + bins - offset) % bins)
            .find(|other| !is_root(profile[*other]));
        let right = (1..=MAX_GAP)
            .map(|offset| (index + offset) % bins)
            .find(|other| !is_root(profile[*other]));
        let (Some(left), Some(right)) = (left, right) else {
            continue;
        };
        let span = (right + bins - left) % bins;
        if span == 0 || span > 2 * MAX_GAP {
            continue;
        }
        let start = profile[left];
        let end = profile[right];
        for step in 1..span {
            let other = (left + step) % bins;
            let t = step as f64 / span as f64;
            profile[other] = start + t * (end - start);
        }
    }
}

fn profile_shift(a: &[f64], b: &[f64], root_radius: f64) -> isize {
    let bins = a.len().min(b.len());
    if bins == 0 {
        return 0;
    }
    let limit = (bins / 4) as isize;
    let mut best_shift = 0_isize;
    let mut best_score = f64::NEG_INFINITY;
    for shift in -limit..=limit {
        let mut score = 0.0;
        for index in 0..bins {
            let other = (index as isize + shift).rem_euclid(bins as isize) as usize;
            score += (a[index] - root_radius) * (b[other] - root_radius);
        }
        if score > best_score {
            best_score = score;
            best_shift = shift;
        }
    }
    best_shift
}

fn container_kind(path: &Path) -> &'static str {
    let Ok(mut file) = File::open(path) else {
        return "unknown";
    };
    let mut magic = [0_u8; 8];
    if std::io::Read::read_exact(&mut file, &mut magic).is_err() {
        return "unknown";
    }
    if magic == [0xD0, 0xCF, 0x11, 0xE0, 0xA1, 0xB1, 0x1A, 0xE1] {
        "cfb"
    } else if &magic == b"SPLMSSTR" {
        "splmsstr"
    } else {
        "unknown"
    }
}

fn render_image(path: &Path, width: usize, height: usize) -> anyhow::Result<Vec<u32>> {
    let mesh = load_mesh(path, chord(0.5))?;
    if mesh.indices.is_empty() && mesh.cam_points.is_empty() {
        bail!(
            "{} has no renderable B-rep, JT, or CAM geometry",
            path.display()
        );
    }
    render_mesh_image(&mesh, width, height)
}

fn render_mesh_image(mesh: &MeshData, width: usize, height: usize) -> anyhow::Result<Vec<u32>> {
    if mesh.indices.is_empty() && mesh.cam_points.is_empty() {
        bail!("model has no renderable B-rep, JT, or CAM geometry");
    }
    let mut positions = mesh.positions.clone();
    positions.extend(mesh.cam_points.iter().copied());
    let (min, max, center) = view_bounds(&positions);
    let target = Vec3::new(center[0], center[1], center[2]);
    let extent = (max[0] - min[0])
        .max(max[1] - min[1])
        .max(max[2] - min[2])
        .max(1e-3);
    let mut buffer = vec![0_u32; width * height];
    let mut zbuf = vec![f32::INFINITY; width * height];
    render_software(
        &mesh,
        &mesh.cam_points,
        &mesh.cam_sections,
        &mesh.cam_edges,
        &mesh.cam_rapid,
        true,
        width,
        height,
        target,
        0.65,
        0.35,
        extent * 2.6,
        extent,
        &mut buffer,
        &mut zbuf,
    );
    Ok(buffer)
}

fn write_ppm(path: &Path, buffer: &[u32], width: usize, height: usize) -> anyhow::Result<()> {
    let mut file = File::create(path)?;
    write!(&mut file, "P6\n{width} {height}\n255\n")?;
    let mut rgb = Vec::with_capacity(buffer.len() * 3);
    for pixel in buffer {
        rgb.push(((pixel >> 16) & 0xFF) as u8);
        rgb.push(((pixel >> 8) & 0xFF) as u8);
        rgb.push((pixel & 0xFF) as u8);
    }
    file.write_all(&rgb)?;
    Ok(())
}

fn edit_interactive(path: &Path) -> anyhow::Result<()> {
    let mut pending: Vec<(String, String)> = Vec::new();
    println!("editing {}", path.display());
    println!("commands: info, attrs, get <title>, set <title> <value>, edits,");
    println!("          discard, save [out.prt], render <out.ppm> [w] [h], reload, quit");

    let stdin = io::stdin();
    let mut lines = stdin.lock().lines();
    loop {
        print!("nx> ");
        io::stdout().flush()?;
        let Some(Ok(line)) = lines.next() else {
            break;
        };
        let line = line.trim();
        if line.is_empty() {
            continue;
        }
        let (command, rest) = line
            .split_once(char::is_whitespace)
            .map_or((line, ""), |(command, rest)| (command, rest.trim()));
        match command {
            "help" | "?" => {
                println!("info                  part census");
                println!("attrs                 list attributes");
                println!("get <title>           show one attribute");
                println!("set <title> <value>   stage an attribute edit");
                println!("edits                 list staged edits");
                println!("discard               drop staged edits");
                println!("save [out.prt]        write the staged edits");
                println!("render <out.ppm> [w] [h]  render the source part");
                println!("reload                drop staged edits");
                println!("quit | exit           leave the editor");
            }
            "info" => edit_summary(path)?,
            "attrs" => {
                for attribute in read_part_attributes(path)? {
                    println!("{}\t{}", attribute.title, attribute.value);
                }
            }
            "get" => {
                if rest.is_empty() {
                    println!("usage: get <title>");
                    continue;
                }
                let attributes = read_part_attributes(path)?;
                match attributes.iter().find(|attribute| attribute.title == rest) {
                    Some(attribute) => println!("{}", attribute.value),
                    None => println!("(missing)"),
                }
            }
            "set" => {
                let Some((title, value)) = rest.split_once(char::is_whitespace) else {
                    println!("usage: set <title> <value>");
                    continue;
                };
                let title = title.trim().to_string();
                let value = value.trim_start().to_string();
                if title.is_empty() {
                    println!("attribute title is empty");
                    continue;
                }
                if let Some(existing) = pending.iter_mut().find(|(key, _)| *key == title) {
                    existing.1 = value;
                } else {
                    pending.push((title, value));
                }
                println!("staged");
            }
            "edits" => {
                if pending.is_empty() {
                    println!("(none)");
                }
                for (title, value) in &pending {
                    println!("{title}\t{value}");
                }
            }
            "discard" | "reload" => {
                pending.clear();
                println!("staged edits cleared");
            }
            "save" => {
                let output = if rest.is_empty() {
                    let stem = path
                        .file_stem()
                        .and_then(|stem| stem.to_str())
                        .unwrap_or("part");
                    path.with_file_name(format!("{stem}_edited.prt"))
                } else {
                    PathBuf::from(rest)
                };
                apply_pending(path, &output, &pending)?;
                println!("wrote {}", output.display());
                pending.clear();
            }
            "render" => {
                let mut fields = rest.split_whitespace();
                let Some(output) = fields.next() else {
                    println!("usage: render <out.ppm> [w] [h]");
                    continue;
                };
                let width = fields
                    .next()
                    .and_then(|value| value.parse::<usize>().ok())
                    .unwrap_or(800);
                let height = fields
                    .next()
                    .and_then(|value| value.parse::<usize>().ok())
                    .unwrap_or(600);
                let buffer = render_image(path, width, height)?;
                write_ppm(Path::new(output), &buffer, width, height)?;
                println!("wrote {output}");
            }
            "quit" | "exit" => break,
            other => println!("unknown command: {other}"),
        }
    }
    Ok(())
}

fn edit_summary(path: &Path) -> anyhow::Result<()> {
    let mut input = File::open(path)?;
    let result = NxCodec.decode(&mut input, &DecodeOptions::default())?;
    let ir = result.ir();
    let placements = assembly_placements(ir).unwrap_or_default();
    println!(
        "bodies={} faces={} edges={} attributes={} occurrences={} placements={}",
        ir.model.bodies.len(),
        ir.model.faces.len(),
        ir.model.edges.len(),
        ir.model.attributes.len(),
        ir.model.occurrences.len(),
        placements.len()
    );
    Ok(())
}

fn apply_pending(input: &Path, output: &Path, edits: &[(String, String)]) -> anyhow::Result<()> {
    if edits.is_empty() {
        fs::copy(input, output)?;
        println!("no staged edits; copied source");
        return Ok(());
    }
    if input == output {
        bail!("save output must differ from the source path");
    }
    let mut current = input.to_path_buf();
    let mut temporaries = Vec::new();
    let result = (|| -> anyhow::Result<()> {
        for (index, (title, value)) in edits.iter().enumerate() {
            let next = if index + 1 == edits.len() {
                output.to_path_buf()
            } else {
                std::env::temp_dir().join(format!("nxview-edit-{}-{index}.prt", std::process::id()))
            };
            let report = set_part_attribute(&current, &next, title, value)?;
            println!("updated {title:?}: {}", report.changed_streams.join(", "));
            if index > 0 {
                let _ = fs::remove_file(&current);
            }
            temporaries.push(next.clone());
            current = next;
        }
        Ok(())
    })();
    if result.is_err() {
        for temporary in temporaries {
            let _ = fs::remove_file(temporary);
        }
    }
    result
}

fn info(path: &Path) -> anyhow::Result<()> {
    println!("== {}", path.display());
    let mut input = File::open(path)?;
    let result = NxCodec.decode(&mut input, &DecodeOptions::default())?;
    let ir = result.ir();
    println!(
        "bodies={} regions={} shells={} faces={} loops={} coedges={} edges={} vertices={}",
        ir.model.bodies.len(),
        ir.model.regions.len(),
        ir.model.shells.len(),
        ir.model.faces.len(),
        ir.model.loops.len(),
        ir.model.coedges.len(),
        ir.model.edges.len(),
        ir.model.vertices.len(),
    );
    println!(
        "surfaces={} curves={} tessellations={} features={} attributes={} occurrences={}",
        ir.model.surfaces.len(),
        ir.model.curves.len(),
        ir.model.tessellations.len(),
        ir.model.features.len(),
        ir.model.attributes.len(),
        ir.model.occurrences.len(),
    );
    let report = result.report();
    println!("losses={}", report.losses.len());
    for loss in report.losses.iter().take(12) {
        println!("  {:?}: {}", loss.severity, loss.message);
    }
    for tess in &ir.model.tessellations {
        println!(
            "  embedded tessellation {}: vertices={} triangles={}",
            tess.id,
            tess.mesh().vertex_count(),
            tess.mesh().triangle_count()
        );
    }
    Ok(())
}

fn chord(default: f64) -> f64 {
    std::env::var("NXVIEW_CHORD")
        .ok()
        .and_then(|value| value.parse::<f64>().ok())
        .filter(|value| value.is_finite() && *value > 0.0)
        .unwrap_or(default)
}

pub(crate) fn load_mesh(path: &Path, chord: f64) -> anyhow::Result<MeshData> {
    println!("== {}", path.display());
    let mut input = File::open(path)?;
    let result = NxCodec.decode(&mut input, &DecodeOptions::default())?;
    let ir = result.ir();
    println!(
        "bodies={} faces={} edges={} vertices={} tessellations={}",
        ir.model.bodies.len(),
        ir.model.faces.len(),
        ir.model.edges.len(),
        ir.model.vertices.len(),
        ir.model.tessellations.len(),
    );
    for loss in result.report().losses.iter().take(8) {
        println!("  {:?}: {}", loss.severity, loss.message);
    }

    let mut mesh = MeshData::default();

    // Source tessellation, when the part carries a decodable display stream.
    for tessellation in &ir.model.tessellations {
        let source = tessellation.mesh();
        let positions: Vec<Point3> = source
            .vertices()
            .into_iter()
            .map(cadmpeg_ir::features::FinitePoint3::get)
            .collect();
        let normals: Vec<Vector3> = source
            .vertex_normals()
            .into_iter()
            .map(cadmpeg_ir::features::FiniteVector3::get)
            .collect();
        let triangles = source.triangles();
        mesh.add(&positions, &normals, &triangles, "embedded");
    }

    // Exact B-rep faces, tessellated in the surface parameter plane.
    if !ir.model.faces.is_empty() {
        let options = Options {
            chord,
            ..Options::default()
        };
        let mut tessellator = Tessellator::new(ir, options);
        for face in &ir.model.faces {
            match tessellator.face(&face.id) {
                Ok(face_mesh) => {
                    let triangles: Vec<[u32; 3]> = face_mesh
                        .triangles
                        .iter()
                        .map(|triangle| {
                            [triangle[0] as u32, triangle[1] as u32, triangle[2] as u32]
                        })
                        .collect();
                    mesh.add(&face_mesh.positions, &face_mesh.normals, &triangles, "brep");
                }
                Err(error) => {
                    mesh.faces_refused += 1;
                    let limit = if std::env::var("NXVIEW_FACE_ERRORS").is_ok() {
                        usize::MAX
                    } else {
                        10
                    };
                    if mesh.faces_refused <= limit {
                        let surface = ir
                            .model
                            .surfaces
                            .iter()
                            .find(|surface| surface.id == face.surface)
                            .map(|surface| &surface.geometry);
                        println!(
                            "  refused face {} (surface {:?}): {}",
                            face.id, surface, error
                        );
                    }
                }
            }
        }
    }

    if let Ok(cam) = cam::load(path) {
        if !cam.moves.is_empty() {
            let (samples, edges) = cam.polyline();
            mesh.cam_points = samples
                .iter()
                .map(|sample| {
                    [
                        sample.point[0] as f32,
                        sample.point[1] as f32,
                        sample.point[2] as f32,
                    ]
                })
                .collect();
            mesh.cam_sections = samples.iter().map(|sample| sample.section).collect();
            mesh.cam_rapid = samples.iter().map(|sample| sample.rapid).collect();
            mesh.cam_edges = edges;
            println!(
                "CAM: sections={} moves={} samples={} operations={:?}",
                cam.path_sections,
                cam.moves.len(),
                samples.len(),
                cam.operations
            );
        }
    }

    println!(
        "render mesh: faces={} refused={} vertices={} triangles={} cam-points={}",
        mesh.faces_tessellated,
        mesh.faces_refused,
        mesh.positions.len(),
        mesh.indices.len() / 3,
        mesh.cam_points.len()
    );
    Ok(mesh)
}

fn write_obj(path: &Path, mesh: &MeshData) -> anyhow::Result<()> {
    let mut obj = String::new();
    for position in &mesh.positions {
        let _ = writeln!(obj, "v {} {} {}", position[0], position[1], position[2]);
    }
    for normal in &mesh.normals {
        let _ = writeln!(obj, "vn {} {} {}", normal[0], normal[1], normal[2]);
    }
    for triangle in mesh.indices.chunks_exact(3) {
        let [a, b, c] = [triangle[0] + 1, triangle[1] + 1, triangle[2] + 1];
        let _ = writeln!(obj, "f {a}//{a} {b}//{b} {c}//{c}");
    }
    fs::write(path, obj)?;
    Ok(())
}

#[derive(Clone, Copy)]
struct CameraState {
    target: Vec3,
    yaw: f32,
    pitch: f32,
    distance: f32,
}

impl CameraState {
    fn new(target: Vec3, extent: f32) -> Self {
        Self {
            target,
            yaw: 0.65,
            pitch: 0.35,
            distance: extent * 4.0,
        }
    }

    fn eye(self) -> Vec3 {
        self.target
            + Vec3::new(
                self.distance * self.pitch.cos() * self.yaw.sin(),
                self.distance * self.pitch.sin(),
                self.distance * self.pitch.cos() * self.yaw.cos(),
            )
    }

    fn orbit(&mut self, dx: f32, dy: f32) {
        self.yaw -= dx * 0.008;
        self.pitch = (self.pitch + dy * 0.008).clamp(-1.45, 1.45);
    }

    fn pan(&mut self, dx: f32, dy: f32, height: usize) {
        let forward = (self.target - self.eye()).normalize_or_zero();
        let right = forward.cross(Vec3::Y).normalize_or_zero();
        let up = right.cross(forward).normalize_or_zero();
        let scale = self.distance * 2.0 * (22.5_f32.to_radians()).tan() / height.max(1) as f32;
        self.target += right * (-dx * scale) + up * (dy * scale);
    }

    fn zoom(&mut self, scroll: f32, extent: f32) {
        self.distance = (self.distance * (1.0 - scroll * 0.12)).clamp(extent * 0.05, extent * 50.0);
    }
}

#[cfg(feature = "viewer")]
fn view(path: &Path, mesh: &MeshData) -> anyhow::Result<()> {
    let mut bounds_positions = mesh.positions.clone();
    bounds_positions.extend(mesh.cam_points.iter().copied());
    let (min, max, _center) = view_bounds(&bounds_positions);
    let mut mean = [0.0_f64; 3];
    for position in &bounds_positions {
        for axis in 0..3 {
            mean[axis] += f64::from(position[axis]);
        }
    }
    let count = bounds_positions.len().max(1) as f64;
    let target = Vec3::new(
        (mean[0] / count) as f32,
        (mean[1] / count) as f32,
        (mean[2] / count) as f32,
    );
    let extent = (max[0] - min[0])
        .max(max[1] - min[1])
        .max(max[2] - min[2])
        .max(1e-3);

    let mut window = Window::new(
        &format!("NX Viewer — {}", path.display()),
        1100,
        700,
        WindowOptions {
            resize: true,
            ..Default::default()
        },
    )?;
    window.set_target_fps(60);

    let initial = CameraState::new(target, extent);
    let mut camera = initial;
    let mut buffer: Vec<u32> = Vec::new();
    let mut zbuf: Vec<f32> = Vec::new();
    let mut last_orbit: Option<(f32, f32)> = None;
    let mut last_pan: Option<(f32, f32)> = None;
    let mut show_cam = true;
    let mut screenshot = 0usize;

    println!("controls: left-drag orbit, right/middle-drag pan, wheel zoom");
    println!("          C CAM on/off, R reset, F fit, S screenshot, P state, Esc close");

    while window.is_open() && !window.is_key_down(Key::Escape) {
        let (width, height) = window.get_size();
        if width == 0 || height == 0 {
            continue;
        }
        if buffer.len() != width * height {
            buffer = vec![0; width * height];
            zbuf = vec![f32::INFINITY; width * height];
        }

        if window.is_key_pressed(Key::C, KeyRepeat::No) {
            show_cam = !show_cam;
            println!("CAM overlay: {}", if show_cam { "on" } else { "off" });
        }
        if window.is_key_pressed(Key::R, KeyRepeat::No) {
            camera = initial;
            println!("camera reset");
        }
        if window.is_key_pressed(Key::F, KeyRepeat::No) {
            camera.target = target;
            camera.distance = extent * 4.0;
            println!("camera fit");
        }
        if window.is_key_pressed(Key::P, KeyRepeat::No) {
            println!(
                "camera target=({:.3}, {:.3}, {:.3}) yaw={:.3} pitch={:.3} distance={:.3}",
                camera.target.x,
                camera.target.y,
                camera.target.z,
                camera.yaw,
                camera.pitch,
                camera.distance
            );
        }
        if window.is_key_pressed(Key::S, KeyRepeat::No) {
            let path = PathBuf::from(format!("nxview-screenshot-{screenshot}.ppm"));
            write_ppm(&path, &buffer, width, height)?;
            println!("wrote {}", path.display());
            screenshot += 1;
        }

        if let Some((mx, my)) = window.get_mouse_pos(MouseMode::Discard) {
            if window.get_mouse_down(MouseButton::Left) {
                if let Some((lx, ly)) = last_orbit {
                    camera.orbit(mx - lx, my - ly);
                }
                last_orbit = Some((mx, my));
            } else {
                last_orbit = None;
            }
            if window.get_mouse_down(MouseButton::Right)
                || window.get_mouse_down(MouseButton::Middle)
            {
                if let Some((lx, ly)) = last_pan {
                    camera.pan(mx - lx, my - ly, height);
                }
                last_pan = Some((mx, my));
            } else {
                last_pan = None;
            }
        }

        if let Some((_, scroll)) = window.get_scroll_wheel() {
            camera.zoom(scroll, extent);
        }

        render_software(
            mesh,
            &mesh.cam_points,
            &mesh.cam_sections,
            &mesh.cam_edges,
            &mesh.cam_rapid,
            show_cam,
            width,
            height,
            camera.target,
            camera.yaw,
            camera.pitch,
            camera.distance,
            extent,
            &mut buffer,
            &mut zbuf,
        );
        window.update_with_buffer(&buffer, width, height)?;
    }
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn render_software(
    mesh: &MeshData,
    cam_points: &[[f32; 3]],
    cam_sections: &[usize],
    cam_edges: &[[u32; 2]],
    cam_rapid: &[bool],
    show_cam: bool,
    width: usize,
    height: usize,
    target: Vec3,
    yaw: f32,
    pitch: f32,
    distance: f32,
    extent: f32,
    buffer: &mut [u32],
    zbuf: &mut [f32],
) {
    buffer.fill(0x000b1119);
    zbuf.fill(f32::INFINITY);

    let eye = target
        + Vec3::new(
            distance * pitch.cos() * yaw.sin(),
            distance * pitch.sin(),
            distance * pitch.cos() * yaw.cos(),
        );
    let aspect = width as f32 / height as f32;
    let near = (extent * 0.005).max(1e-4);
    let far = (extent * 200.0).max(near * 10.0);
    let view_matrix = view::look_at_mat4(eye, target, Vec3::Y);
    let projection = opengl::perspective(45_f32.to_radians(), aspect, near, far);
    let mvp = projection * view_matrix;
    if std::env::var("NXVIEW_PROJ_DEBUG").is_ok() {
        let clip = mvp * Vec4::new(target.x, target.y, target.z, 1.0);
        eprintln!(
            "target={target:?} eye={eye:?} extent={extent} clip={clip:?} screen=({}, {})",
            (clip.x / clip.w * 0.5 + 0.5) * width as f32,
            (1.0 - (clip.y / clip.w * 0.5 + 0.5)) * height as f32
        );
    }

    let mut screen = vec![Vec3::ZERO; mesh.positions.len()];
    let mut visible = vec![false; mesh.positions.len()];
    for (index, position) in mesh.positions.iter().enumerate() {
        let clip = mvp * Vec4::new(position[0], position[1], position[2], 1.0);
        if clip.w <= 1e-6 {
            continue;
        }
        let inv_w = 1.0 / clip.w;
        screen[index] = Vec3::new(
            (clip.x * inv_w * 0.5 + 0.5) * width as f32,
            (1.0 - (clip.y * inv_w * 0.5 + 0.5)) * height as f32,
            clip.z * inv_w * 0.5 + 0.5,
        );
        visible[index] = true;
    }

    let light = Vec3::new(-0.4, 0.78, 0.55).normalize();
    let base = Vec3::new(0.62, 0.70, 0.82);

    for triangle in mesh.indices.chunks_exact(3) {
        let indices = [
            triangle[0] as usize,
            triangle[1] as usize,
            triangle[2] as usize,
        ];
        if indices.iter().any(|index| !visible[*index]) {
            continue;
        }
        let a = screen[indices[0]];
        let b = screen[indices[1]];
        let c = screen[indices[2]];
        let area = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
        if area.abs() < 1e-6 {
            continue;
        }
        let inv_area = 1.0 / area;

        let min_x = a.x.min(b.x).min(c.x).floor().max(0.0) as usize;
        let max_x = (a.x.max(b.x).max(c.x).ceil() as usize).min(width - 1);
        let min_y = a.y.min(b.y).min(c.y).floor().max(0.0) as usize;
        let max_y = (a.y.max(b.y).max(c.y).ceil() as usize).min(height - 1);
        if min_x > max_x || min_y > max_y {
            continue;
        }

        let n0 = Vec3::from(mesh.normals[indices[0]]);
        let n1 = Vec3::from(mesh.normals[indices[1]]);
        let n2 = Vec3::from(mesh.normals[indices[2]]);

        for y in min_y..=max_y {
            let py = y as f32 + 0.5;
            for x in min_x..=max_x {
                let px = x as f32 + 0.5;
                let w0 = ((b.x - px) * (c.y - py) - (b.y - py) * (c.x - px)) * inv_area;
                let w1 = ((c.x - px) * (a.y - py) - (c.y - py) * (a.x - px)) * inv_area;
                let w2 = 1.0 - w0 - w1;
                if w0 < 0.0 || w1 < 0.0 || w2 < 0.0 {
                    continue;
                }
                let z = w0 * a.z + w1 * b.z + w2 * c.z;
                let offset = y * width + x;
                if z >= zbuf[offset] {
                    continue;
                }
                zbuf[offset] = z;

                let normal = (n0 * w0 + n1 * w1 + n2 * w2).normalize_or_zero();
                let diffuse = normal.dot(light).abs();
                // A little depth cue keeps silhouette edges readable.
                let fog = (1.0 - z).clamp(0.0, 1.0) * 0.10;
                let shade = 0.24 + 0.76 * diffuse + fog;
                let r = (base.x * shade * 255.0).clamp(0.0, 255.0) as u32;
                let g = (base.y * shade * 255.0).clamp(0.0, 255.0) as u32;
                let b = (base.z * shade * 255.0).clamp(0.0, 255.0) as u32;
                buffer[offset] = (r << 16) | (g << 8) | b;
            }
        }
    }

    // CAM toolpath moves. A small depth bias keeps them visible on top of the
    // CAD surface they are cutting. Arc and helical moves have already been
    // expanded by cam.rs; draw the resulting segments as lines, not just dots.
    const CAM_COLORS: [u32; 8] = [
        0x00ffb347, 0x004fc3f7, 0x0081c784, 0x00ba68c8, 0x00ffd54f, 0x004db6ac, 0x00ff8a65,
        0x009575cd,
    ];
    const CAM_RAPID_COLOR: u32 = 0x00ff5252;

    if show_cam {
        let mut cam_screen = vec![Vec3::ZERO; cam_points.len()];
        let mut cam_visible = vec![false; cam_points.len()];
        for (index, point) in cam_points.iter().enumerate() {
            let clip = mvp * Vec4::new(point[0], point[1], point[2], 1.0);
            if clip.w <= 1e-6 {
                continue;
            }
            let inv_w = 1.0 / clip.w;
            cam_screen[index] = Vec3::new(
                (clip.x * inv_w * 0.5 + 0.5) * width as f32,
                (1.0 - (clip.y * inv_w * 0.5 + 0.5)) * height as f32,
                clip.z * inv_w * 0.5 + 0.5,
            );
            cam_visible[index] = (0.0..=1.0).contains(&cam_screen[index].z);
        }

        for edge in cam_edges {
            let [a, b] = [edge[0] as usize, edge[1] as usize];
            if a >= cam_visible.len()
                || b >= cam_visible.len()
                || !cam_visible[a]
                || !cam_visible[b]
            {
                continue;
            }
            let section = cam_sections.get(a).copied().unwrap_or(0);
            let color = if cam_rapid.get(a).copied().unwrap_or(false) {
                CAM_RAPID_COLOR
            } else {
                CAM_COLORS[section % CAM_COLORS.len()]
            };
            draw_cam_line(
                buffer,
                zbuf,
                width,
                height,
                cam_screen[a],
                cam_screen[b],
                color,
            );
        }

        for (move_index, screen) in cam_screen.iter().enumerate() {
            if !cam_visible[move_index] {
                continue;
            }
            let section = cam_sections.get(move_index).copied().unwrap_or(0);
            let color = if cam_rapid.get(move_index).copied().unwrap_or(false) {
                CAM_RAPID_COLOR
            } else {
                CAM_COLORS[section % CAM_COLORS.len()]
            };
            let depth = (screen.z - 0.0008).max(0.0);
            let xi = screen.x as isize;
            let yi = screen.y as isize;
            for dy in 0..2 {
                for dx in 0..2 {
                    let px = xi + dx;
                    let py = yi + dy;
                    if px < 0 || py < 0 || px >= width as isize || py >= height as isize {
                        continue;
                    }
                    let offset = py as usize * width + px as usize;
                    if depth < zbuf[offset] {
                        zbuf[offset] = depth;
                        buffer[offset] = color;
                    }
                }
            }
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn draw_cam_line(
    buffer: &mut [u32],
    zbuf: &mut [f32],
    width: usize,
    height: usize,
    a: Vec3,
    b: Vec3,
    color: u32,
) {
    let dx = b.x - a.x;
    let dy = b.y - a.y;
    let steps = dx.abs().max(dy.abs()).ceil().max(1.0) as usize;
    for step in 0..=steps {
        let t = step as f32 / steps as f32;
        let x = a.x + dx * t;
        let y = a.y + dy * t;
        let z = a.z + (b.z - a.z) * t;
        let depth = (z - 0.0008).max(0.0);
        let xi = x as isize;
        let yi = y as isize;
        for oy in 0..2 {
            for ox in 0..2 {
                let px = xi + ox;
                let py = yi + oy;
                if px < 0 || py < 0 || px >= width as isize || py >= height as isize {
                    continue;
                }
                let offset = py as usize * width + px as usize;
                if depth < zbuf[offset] {
                    zbuf[offset] = depth;
                    buffer[offset] = color;
                }
            }
        }
    }
}

/// A 2nd–98th percentile box keeps stray construction faces from shrinking the
/// whole model to a dot in the viewer. Exact bounds remain the fallback.
fn view_bounds(positions: &[[f32; 3]]) -> ([f32; 3], [f32; 3], [f32; 3]) {
    if positions.is_empty() {
        return ([-1.0; 3], [1.0; 3], [0.0; 3]);
    }
    let mut min = [f32::INFINITY; 3];
    let mut max = [f32::NEG_INFINITY; 3];
    let mut center = [0.0_f32; 3];
    for axis in 0..3 {
        let mut values: Vec<f32> = positions.iter().map(|position| position[axis]).collect();
        values.sort_by(|a, b| a.total_cmp(b));
        let low = values[(values.len() * 2 / 100).min(values.len() - 1)];
        let high = values[(values.len() * 98 / 100).min(values.len() - 1)];
        center[axis] = values[values.len() / 2];
        if low.is_finite() && high.is_finite() && low < high {
            min[axis] = low;
            max[axis] = high;
        } else {
            min[axis] = values[0];
            max[axis] = values[values.len() - 1];
        }
    }
    let margin = [
        (max[0] - min[0]).max(1e-3) * 0.05,
        (max[1] - min[1]).max(1e-3) * 0.05,
        (max[2] - min[2]).max(1e-3) * 0.05,
    ];
    (
        [min[0] - margin[0], min[1] - margin[1], min[2] - margin[2]],
        [max[0] + margin[0], max[1] + margin[1], max[2] + margin[2]],
        center,
    )
}

#[cfg(test)]
mod interactive_tests {
    use super::CameraState;
    use glam::Vec3;

    fn camera() -> CameraState {
        CameraState::new(Vec3::new(10.0, 20.0, 30.0), 100.0)
    }

    #[test]
    fn eye_is_at_the_configured_distance() {
        let camera = camera();
        assert!((camera.eye().distance(camera.target) - camera.distance).abs() < 1e-4);
    }

    #[test]
    fn orbit_clamps_pitch_and_changes_yaw() {
        let mut camera = camera();
        camera.orbit(100.0, 1000.0);
        assert!(camera.pitch <= 1.45);
        assert!((camera.yaw + 0.15).abs() < 1e-4);
    }

    #[test]
    fn zoom_stays_inside_fit_limits() {
        let mut camera = camera();
        camera.zoom(1000.0, 100.0);
        assert!((camera.distance - 5.0).abs() < 1e-4);
        camera.zoom(-1000.0, 100.0);
        assert!((camera.distance - 605.0).abs() < 1e-2);
    }

    #[test]
    fn pan_moves_the_target_in_the_view_plane() {
        let mut camera = camera();
        let before = camera.target;
        camera.pan(50.0, -25.0, 700);
        let moved = camera.target - before;
        assert!(moved.length() > 0.0);
        let forward = (camera.target - camera.eye()).normalize_or_zero();
        assert!(moved.dot(forward).abs() < 1e-3);
    }

    #[test]
    fn reset_restores_the_initial_frame() {
        let initial = camera();
        let mut camera = initial;
        camera.orbit(50.0, 25.0);
        camera.pan(10.0, 10.0, 700);
        camera.zoom(-5.0, 100.0);
        camera = initial;
        assert_eq!(camera.yaw, initial.yaw);
        assert_eq!(camera.pitch, initial.pitch);
        assert_eq!(camera.distance, initial.distance);
        assert_eq!(camera.target, initial.target);
    }
}

#[cfg(test)]
mod gear_tests {
    use super::analyze_gear;

    fn gear_vertices(teeth: usize) -> Vec<[f64; 3]> {
        let mut vertices = Vec::new();
        let tip = 24.0;
        let root = 17.25;
        let pitch = std::f64::consts::TAU / teeth as f64;
        let duty = 0.38;
        for z in [18.0, 43.0] {
            for index in 0..720 {
                let angle = std::f64::consts::TAU * index as f64 / 720.0;
                let phase = angle.rem_euclid(pitch) / pitch;
                let radius = if phase < duty { tip } else { root };
                vertices.push([radius * angle.cos(), radius * angle.sin(), z]);
            }
        }
        vertices
    }

    #[test]
    fn counts_synthetic_tooth_edges() {
        let mut profile = [17.25; 720];
        let pitch = 720.0 / 14.0;
        for (index, value) in profile.iter_mut().enumerate() {
            let phase = (index as f64 % pitch) / pitch;
            if phase < 0.38 {
                *value = 24.0;
            }
        }
        let (teeth, contrast) = super::folded_tooth_count(&profile, 17.25).expect("folded count");
        assert_eq!(teeth, 14);
        assert!(contrast > 6.0);
    }

    #[test]
    fn detects_a_spur_gear_profile() {
        let report = analyze_gear(&gear_vertices(14)).expect("gear profile");
        assert_eq!(report.teeth, 14);
        assert!((report.tip_radius - 24.0).abs() < 1e-9);
        assert!((report.root_radius - 17.25).abs() < 1e-9);
        assert!((report.face_width - 25.0).abs() < 1e-9);
        assert_eq!(report.axis, [0.0, 0.0, 1.0]);
    }

    #[test]
    fn refuses_a_smooth_cylinder() {
        let mut vertices = Vec::new();
        for z in [0.0, 20.0] {
            for index in 0..720 {
                let angle = std::f64::consts::TAU * index as f64 / 720.0;
                vertices.push([20.0 * angle.cos(), 20.0 * angle.sin(), z]);
            }
        }
        assert!(analyze_gear(&vertices).is_none());
    }
}
