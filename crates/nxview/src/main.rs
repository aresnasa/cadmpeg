// SPDX-License-Identifier: Apache-2.0
//! Cross-platform NX `.prt` probe: decode, tessellate, export OBJ, or view
//! the result with a small pure-Rust software viewer.

use std::fmt::Write as _;
use std::fs::{self, File};
use std::path::{Path, PathBuf};

use anyhow::{bail, Context};
use cadmpeg_codec_nx::NxCodec;
use cadmpeg_ir::codec::{Codec, DecodeOptions};
use cadmpeg_ir::math::{Point3, Vector3};
use cadmpeg_tessellate::{Options, Tessellator};
use glam::camera::rh::{proj::opengl, view};
use glam::{Vec3, Vec4};
use minifb::{Key, MouseButton, MouseMode, Window, WindowOptions};

mod cam;

#[derive(Default)]
struct MeshData {
    positions: Vec<[f32; 3]>,
    normals: Vec<[f32; 3]>,
    indices: Vec<u32>,
    cam_points: Vec<[f32; 3]>,
    cam_sections: Vec<usize>,
    faces_tessellated: usize,
    faces_refused: usize,
}

impl MeshData {
    fn add(
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
        other => {
            // Treat the first argument as a path for convenience.
            let path = PathBuf::from(other);
            info(&path)?;
        }
    }
    Ok(())
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

fn load_mesh(path: &Path, chord: f64) -> anyhow::Result<MeshData> {
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
                    if mesh.faces_refused <= 10 {
                        println!("  refused face {}: {}", face.id, error);
                    }
                }
            }
        }
    }

    if let Ok(cam) = cam::load(path) {
        if !cam.moves.is_empty() {
            mesh.cam_points = cam
                .moves
                .iter()
                .map(|mv| [mv.end[0] as f32, mv.end[1] as f32, mv.end[2] as f32])
                .collect();
            mesh.cam_sections = cam.moves.iter().map(|mv| mv.section).collect();
            println!(
                "CAM: sections={} moves={} operations={:?}",
                cam.path_sections,
                cam.moves.len(),
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

fn view(path: &Path, mesh: &MeshData) -> anyhow::Result<()> {
    let mut bounds_positions = mesh.positions.clone();
    bounds_positions.extend(mesh.cam_points.iter().copied());
    let (min, max, center) = view_bounds(&bounds_positions);
    let target = Vec3::from(center);
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

    let mut yaw = 0.65_f32;
    let mut pitch = 0.35_f32;
    let mut distance = extent * 3.2;
    let mut buffer: Vec<u32> = Vec::new();
    let mut zbuf: Vec<f32> = Vec::new();
    let mut last_mouse: Option<(f32, f32)> = None;

    println!("controls: left-drag orbit, wheel zoom, Esc close");

    while window.is_open() && !window.is_key_down(Key::Escape) {
        let (width, height) = window.get_size();
        if width == 0 || height == 0 {
            continue;
        }
        if buffer.len() != width * height {
            buffer = vec![0; width * height];
            zbuf = vec![f32::INFINITY; width * height];
        }

        if let Some((mx, my)) = window.get_mouse_pos(MouseMode::Discard) {
            if window.get_mouse_down(MouseButton::Left) {
                if let Some((lx, ly)) = last_mouse {
                    yaw -= (mx - lx) * 0.008;
                    pitch = (pitch + (my - ly) * 0.008).clamp(-1.45, 1.45);
                }
                last_mouse = Some((mx, my));
            } else {
                last_mouse = None;
            }
        }

        if let Some((_, scroll)) = window.get_scroll_wheel() {
            distance = (distance * (1.0 - scroll * 0.12)).clamp(extent * 0.05, extent * 50.0);
        }

        render_software(
            mesh,
            &mesh.cam_points,
            &mesh.cam_sections,
            width,
            height,
            target,
            yaw,
            pitch,
            distance,
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

    // CAM toolpath move endpoints. A small depth bias keeps them visible on
    // top of the CAD surface they are cutting.
    const CAM_COLORS: [u32; 8] = [
        0x00ffb347, 0x004fc3f7, 0x0081c784, 0x00ba68c8, 0x00ffd54f, 0x004db6ac, 0x00ff8a65,
        0x009575cd,
    ];
    for (move_index, point) in cam_points.iter().enumerate() {
        let section = cam_sections.get(move_index).copied().unwrap_or(0);
        let color = CAM_COLORS[section % CAM_COLORS.len()];
        let clip = mvp * Vec4::new(point[0], point[1], point[2], 1.0);
        if clip.w <= 1e-6 {
            continue;
        }
        let inv_w = 1.0 / clip.w;
        let x = (clip.x * inv_w * 0.5 + 0.5) * width as f32;
        let y = (1.0 - (clip.y * inv_w * 0.5 + 0.5)) * height as f32;
        let z = clip.z * inv_w * 0.5 + 0.5;
        if !(0.0..=1.0).contains(&z) || x < 0.0 || y < 0.0 {
            continue;
        }
        let depth = (z - 0.0008).max(0.0);
        let xi = x as isize;
        let yi = y as isize;
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
