// SPDX-License-Identifier: Apache-2.0
//! Assembly scene loading: placement frames, part meshes, and motion tracks.
//!
//! A scene is built from an assembly `.prt` (or a Motion `.sim` that links
//! one) plus the placement frames stored in that assembly. Components are
//! joined to their placement frame by the object-manager UID string, which is
//! character-identical in both files.

use std::collections::HashMap;
use std::fs;
use std::path::{Path, PathBuf};

use anyhow::bail;
use cadmpeg_codec_nx::{
    assembly_placements, read_assembly_manifest, AssemblyManifest, NxCodec, PlacementInfo,
};
use cadmpeg_ir::codec::{Codec, DecodeOptions};

use crate::motion::{parse_rad, RadAnimation};

const PLACEMENT_SEARCH_ROOTS: usize = 3;

/// One component of the scene.
pub struct ScenePart {
    /// Component file name.
    pub label: String,
    /// UID text of the placement frame that positions this part.
    pub uuid: String,
    /// Current placement matrix (row-major three-by-four).
    pub matrix: [f64; 12],
    /// Placement matrix read from the file (undo target).
    pub base_matrix: [f64; 12],
    /// Part-local positions.
    pub local: Vec<[f32; 3]>,
    /// Part-local normals.
    pub local_normals: Vec<[f32; 3]>,
    /// World positions at the reference frame.
    pub rest: Vec<[f32; 3]>,
    /// World normals at the reference frame.
    pub rest_normals: Vec<[f32; 3]>,
    /// Positions of the displayed frame.
    pub positions: Vec<[f32; 3]>,
    /// Normals of the displayed frame.
    pub normals: Vec<[f32; 3]>,
    /// Triangle indices into `positions`.
    pub indices: Vec<u32>,
    /// Flat base colour.
    pub color: [f32; 3],
    /// Moving `.rad` track driving this part, when present.
    pub track: Option<usize>,
    /// Z coordinate of the part's placement translation.
    pub placement_z: f64,
    /// Whether the part is drawn.
    pub visible: bool,
    /// Moving body that drives this part.
    pub moving: bool,
}

impl ScenePart {
    /// Triangle count.
    #[must_use]
    pub fn triangles(&self) -> usize {
        self.indices.len() / 3
    }

    /// Rebuild the reference-frame geometry from `matrix`.
    pub fn apply_matrix(&mut self) {
        let rotation = placement_rotation(&self.matrix);
        let translation = placement_translation(&self.matrix);
        for (position, local) in self.rest.iter_mut().zip(&self.local) {
            *position = apply(rotation, translation, *local);
        }
        for (normal, local) in self.rest_normals.iter_mut().zip(&self.local_normals) {
            *normal = rotate(rotation, *local);
        }
        self.positions.copy_from_slice(&self.rest);
        self.normals.copy_from_slice(&self.rest_normals);
    }
}

/// Rotation part of a row-major three-by-four placement matrix.
#[must_use]
pub fn placement_rotation(matrix: &[f64; 12]) -> [[f32; 3]; 3] {
    [
        [matrix[0] as f32, matrix[1] as f32, matrix[2] as f32],
        [matrix[4] as f32, matrix[5] as f32, matrix[6] as f32],
        [matrix[8] as f32, matrix[9] as f32, matrix[10] as f32],
    ]
}

/// Translation part of a row-major three-by-four placement matrix.
#[must_use]
pub fn placement_translation(matrix: &[f64; 12]) -> [f32; 3] {
    [matrix[3] as f32, matrix[7] as f32, matrix[11] as f32]
}

/// Camera and display state persisted next to a scene.
#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
pub struct SceneViewState {
    /// Orbit target.
    pub target: [f32; 3],
    /// Camera yaw in radians.
    pub yaw: f32,
    /// Camera pitch in radians.
    pub pitch: f32,
    /// Camera distance from the target.
    pub distance: f32,
}

/// Sidecar state written next to a `.sim` or assembly.
#[derive(Debug, Clone, Default, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct SceneState {
    /// Format version.
    pub version: u32,
    /// Displayed animation frame.
    pub frame: usize,
    /// Whether playback was running.
    pub playing: bool,
    /// Whether static parts were hidden.
    pub hide_static: bool,
    /// Playback speed multiplier; zero means 1.0.
    pub speed: f64,
    /// Camera state, when the file was saved from the viewer.
    pub view: Option<SceneViewState>,
    /// Per-part visibility by component file name.
    pub visible: std::collections::BTreeMap<String, bool>,
}

impl SceneState {
    /// Playback speed with the default applied.
    #[must_use]
    pub fn speed_or_default(&self) -> f64 {
        if self.speed > 0.0 {
            self.speed
        } else {
            1.0
        }
    }
}

/// Default sidecar path: `<source>.nxview.json`.
#[must_use]
pub fn default_state_path(source: &Path) -> PathBuf {
    let mut name = source
        .file_name()
        .map_or_else(|| std::ffi::OsString::from("scene"), |name| name.to_owned());
    name.push(".nxview.json");
    source.with_file_name(name)
}

/// Read a sidecar state, returning `None` when the file is absent or invalid.
#[must_use]
pub fn load_state(path: &Path) -> Option<SceneState> {
    let text = fs::read_to_string(path).ok()?;
    serde_json::from_str(&text).ok()
}

/// Write a sidecar state.
pub fn save_state(path: &Path, state: &SceneState) -> anyhow::Result<()> {
    let text = serde_json::to_string_pretty(state)?;
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent)?;
    }
    fs::write(path, text)?;
    Ok(())
}

/// Build a scene state from the current scene.
#[must_use]
pub fn state_from_scene(scene: &Scene) -> SceneState {
    SceneState {
        version: 1,
        frame: scene.frame,
        playing: scene.playing,
        hide_static: scene.parts.iter().all(|part| !part.visible || part.moving),
        speed: scene.speed,
        view: None,
        visible: scene
            .parts
            .iter()
            .map(|part| (part.label.clone(), part.visible))
            .collect(),
    }
}

/// Apply a sidecar state to a loaded scene.
pub fn apply_state(scene: &mut Scene, state: &SceneState) {
    scene.speed = state.speed_or_default();
    if state.frame > 0 {
        scene.set_frame(state.frame);
    }
    scene.playing = state.playing;
    for part in &mut scene.parts {
        if let Some(visible) = state.visible.get(&part.label) {
            part.visible = *visible;
        }
    }
}

/// Everything the scene viewer needs.
pub struct Scene {
    /// File the scene was opened from.
    pub source: PathBuf,
    /// Assembly that carries the placements.
    pub assembly: PathBuf,
    /// Root directory used to resolve component paths.
    pub root: PathBuf,
    /// Loaded components.
    pub parts: Vec<ScenePart>,
    /// Motion animation, when a `.rad` file was found.
    pub animation: Option<RadAnimation>,
    /// RecurDyn text model, when a `.rmd` file was found.
    pub motion_model: Option<crate::motion_model::MotionModel>,
    /// Path of the RecurDyn text model.
    pub rmd_path: Option<PathBuf>,
    /// Driver name used for the editable rotation speed.
    pub driver: Option<String>,
    /// Driver velocity stored in the model.
    pub base_velocity: Option<f64>,
    /// Driver velocity currently used for playback.
    pub driver_velocity: f64,
    /// Displayed frame.
    pub frame: usize,
    /// Whether the animation is playing.
    pub playing: bool,
    /// Diagnostics worth printing.
    pub notes: Vec<String>,
    /// Playback speed multiplier.
    pub speed: f64,
}

impl Scene {
    /// Total triangle count.
    #[must_use]
    pub fn triangles(&self) -> usize {
        self.parts.iter().map(ScenePart::triangles).sum()
    }

    /// Frame count of the animation.
    #[must_use]
    pub fn frame_count(&self) -> usize {
        self.animation.as_ref().map_or(1, RadAnimation::frame_count)
    }

    /// Time of the displayed frame.
    #[must_use]
    pub fn time(&self) -> f64 {
        self.animation
            .as_ref()
            .map_or(0.0, |animation| animation.time(self.frame))
    }

    /// Apply `frame` to every moving part.
    pub fn set_frame(&mut self, frame: usize) {
        let count = self.frame_count().max(1);
        self.frame = frame.min(count - 1);
        for part in &mut self.parts {
            let Some(track) = part.track else {
                continue;
            };
            let Some(animation) = self.animation.as_ref() else {
                continue;
            };
            let Some(matrix) = animation.matrix(track, self.frame) else {
                continue;
            };
            let rotation = [
                [
                    matrix[0][0] as f32,
                    matrix[0][1] as f32,
                    matrix[0][2] as f32,
                ],
                [
                    matrix[1][0] as f32,
                    matrix[1][1] as f32,
                    matrix[1][2] as f32,
                ],
                [
                    matrix[2][0] as f32,
                    matrix[2][1] as f32,
                    matrix[2][2] as f32,
                ],
            ];
            let translation = [
                matrix[0][3] as f32,
                matrix[1][3] as f32,
                matrix[2][3] as f32,
            ];
            for (position, rest) in part.positions.iter_mut().zip(&part.rest) {
                *position = apply(rotation, translation, *rest);
            }
            for (normal, rest) in part.normals.iter_mut().zip(&part.rest_normals) {
                *normal = rotate(rotation, *rest);
            }
        }
    }

    /// Replace one part's placement matrix and rebuild its geometry.
    pub fn set_placement(&mut self, index: usize, matrix: [f64; 12]) {
        if let Some(part) = self.parts.get_mut(index) {
            part.matrix = matrix;
            part.apply_matrix();
        }
        self.set_frame(self.frame);
    }

    /// Placement edits relative to the file contents.
    #[must_use]
    pub fn edited_placements(&self) -> Vec<cadmpeg_codec_nx::PlacementUpdate> {
        self.parts
            .iter()
            .filter(|part| part.matrix != part.base_matrix)
            .map(|part| cadmpeg_codec_nx::PlacementUpdate {
                uuid: part.uuid.clone(),
                matrix: part.matrix,
            })
            .collect()
    }

    /// Write edited placements into a new assembly file.
    pub fn save_placements(&self, output: &Path) -> anyhow::Result<usize> {
        let updates = self.edited_placements();
        if updates.is_empty() {
            anyhow::bail!("no placement was edited");
        }
        cadmpeg_codec_nx::set_assembly_placements(&self.assembly, output, &updates)?;
        Ok(updates.len())
    }

    /// Suggested output path for edited placements.
    #[must_use]
    pub fn edited_assembly_path(&self) -> PathBuf {
        let stem = self.assembly.file_stem().map_or_else(
            || std::ffi::OsString::from("assembly"),
            |stem| stem.to_owned(),
        );
        let mut name = stem;
        name.push("-edited.prt");
        self.source.with_file_name(name)
    }
}

fn apply(rotation: [[f32; 3]; 3], translation: [f32; 3], point: [f32; 3]) -> [f32; 3] {
    let mut out = rotate(rotation, point);
    for axis in 0..3 {
        out[axis] += translation[axis];
    }
    out
}

fn rotate(rotation: [[f32; 3]; 3], vector: [f32; 3]) -> [f32; 3] {
    [
        rotation[0][0] * vector[0] + rotation[0][1] * vector[1] + rotation[0][2] * vector[2],
        rotation[1][0] * vector[0] + rotation[1][1] * vector[1] + rotation[1][2] * vector[2],
        rotation[2][0] * vector[0] + rotation[2][1] * vector[1] + rotation[2][2] * vector[2],
    ]
}

fn component_color(name: &str) -> [f32; 3] {
    if name.contains("齿轮轴") {
        [0.80, 0.62, 0.30]
    } else if name.contains("泵体") || name.contains("泵盖") {
        [0.52, 0.58, 0.66]
    } else if name.contains("弹簧") {
        [0.42, 0.40, 0.37]
    } else if name.contains("钢珠") {
        [0.72, 0.74, 0.78]
    } else {
        [0.66, 0.68, 0.72]
    }
}

fn is_sim(path: &Path) -> bool {
    path.extension()
        .is_some_and(|extension| extension.eq_ignore_ascii_case("sim"))
}

fn manifest_root_candidates(sim: &Path, explicit: Option<&Path>) -> Vec<PathBuf> {
    let mut roots = Vec::new();
    if let Some(root) = explicit {
        roots.push(root.to_path_buf());
    }
    let parent = sim.parent().unwrap_or_else(|| Path::new("."));
    roots.push(parent.to_path_buf());
    if let Some(grand) = parent.parent() {
        roots.push(grand.to_path_buf());
    }
    roots.truncate(PLACEMENT_SEARCH_ROOTS);
    roots
}

/// Pick the root that resolves the most components of `manifest`.
fn best_root(sim: &Path, explicit: Option<&Path>) -> anyhow::Result<(AssemblyManifest, PathBuf)> {
    let mut best: Option<(AssemblyManifest, PathBuf, usize)> = None;
    for root in manifest_root_candidates(sim, explicit) {
        let Ok(manifest) = read_assembly_manifest(sim, Some(&root)) else {
            continue;
        };
        let present = manifest.components.iter().filter(|c| c.present).count();
        if best.as_ref().is_none_or(|(_, _, count)| present > *count) {
            best = Some((manifest, root, present));
        }
    }
    match best {
        Some((manifest, root, _)) => Ok((manifest, root)),
        None => bail!("{} has no readable external-reference table", sim.display()),
    }
}

fn has_placements(path: &Path) -> Option<Vec<PlacementInfo>> {
    let mut input = fs::File::open(path).ok()?;
    let result = NxCodec.decode(&mut input, &DecodeOptions::default()).ok()?;
    assembly_placements(result.ir())
        .ok()
        .filter(|p| !p.is_empty())
}

fn find_assembly(manifest: &AssemblyManifest) -> anyhow::Result<(PathBuf, Vec<PlacementInfo>)> {
    let mut candidates: Vec<&Path> = manifest
        .components
        .iter()
        .filter(|component| component.present)
        .map(|component| component.path.as_path())
        .collect();
    candidates.sort_by_key(|path| {
        let name = path.file_name().unwrap_or_default().to_string_lossy();
        let preferred = !(name.contains("装配") || name.to_lowercase().contains("assembly"));
        (
            preferred,
            fs::metadata(path)
                .map(|meta| meta.len())
                .unwrap_or(u64::MAX),
        )
    });
    for path in candidates {
        if let Some(placements) = has_placements(path) {
            return Ok((path.to_path_buf(), placements));
        }
    }
    bail!(
        "no assembly placement frames in the {} present component(s)",
        manifest.components.len()
    )
}

/// Find the `.rad` file that belongs to a Motion `.sim`.
fn find_rad(sim: &Path) -> Option<PathBuf> {
    let directory = sim.parent()?;
    let stem = sim.file_stem()?.to_string_lossy().to_string();
    let mut candidates: Vec<PathBuf> = fs::read_dir(directory)
        .ok()?
        .filter_map(Result::ok)
        .map(|entry| entry.path())
        .filter(|path| {
            path.extension()
                .is_some_and(|extension| extension.eq_ignore_ascii_case("rad"))
                && path
                    .file_stem()
                    .is_some_and(|name| name.to_string_lossy().starts_with(&stem))
        })
        .collect();
    if candidates.is_empty() {
        candidates = fs::read_dir(directory)
            .ok()?
            .filter_map(Result::ok)
            .map(|entry| entry.path())
            .filter(|path| {
                path.extension()
                    .is_some_and(|extension| extension.eq_ignore_ascii_case("rad"))
            })
            .collect();
    }
    candidates.sort();
    candidates.into_iter().next()
}

/// Find the RecurDyn `.rmd` model that belongs to a Motion `.sim`.
fn find_rmd(sim: &Path) -> Option<PathBuf> {
    let directory = sim.parent()?;
    let stem = sim.file_stem()?.to_string_lossy().to_string();
    let mut candidates: Vec<PathBuf> = fs::read_dir(directory)
        .ok()?
        .filter_map(Result::ok)
        .map(|entry| entry.path())
        .filter(|path| {
            path.extension()
                .is_some_and(|extension| extension.eq_ignore_ascii_case("rmd"))
                && path
                    .file_stem()
                    .is_some_and(|name| name.to_string_lossy().starts_with(&stem))
        })
        .collect();
    candidates.sort();
    candidates.into_iter().next()
}

fn placement_uid_bytes(uuid: &str) -> Vec<u8> {
    uuid.as_bytes().to_vec()
}

/// Build a scene from an assembly or Motion file.
pub fn load(path: &Path, root: Option<&Path>, chord: f64) -> anyhow::Result<Scene> {
    let source = path.to_path_buf();
    let mut notes = Vec::new();
    let (assembly, placements, manifest, root) = if is_sim(path) {
        let (sim_manifest, root) = best_root(path, root)?;
        let (assembly, placements) = find_assembly(&sim_manifest)?;
        notes.push(format!(
            "sim links {} component(s); assembly {} carries {} placement(s)",
            sim_manifest.components.len(),
            assembly.file_name().unwrap_or_default().to_string_lossy(),
            placements.len()
        ));
        let manifest =
            read_assembly_manifest(&assembly, Some(&root)).unwrap_or_else(|_| sim_manifest.clone());
        (assembly, placements, manifest, root)
    } else {
        let root = root.map_or_else(
            || {
                path.parent()
                    .map_or_else(|| PathBuf::from("."), Path::to_path_buf)
            },
            Path::to_path_buf,
        );
        let manifest = read_assembly_manifest(path, Some(&root))?;
        let placements = has_placements(path)
            .ok_or_else(|| anyhow::anyhow!("{} has no placement frames", path.display()))?;
        (path.to_path_buf(), placements, manifest, root)
    };

    // The placement UID is character-identical to the child part's own OM UID,
    // so the join key is the raw text.
    let mut part_bytes: HashMap<String, Vec<u8>> = HashMap::new();
    for component in &manifest.components {
        if !component.present {
            continue;
        }
        if let Ok(bytes) = fs::read(&component.path) {
            part_bytes.insert(component.name.clone(), bytes);
        }
    }

    let mut parts = Vec::new();

    // Tessellation dominates the load time and is independent per component.
    let mut loaded_meshes: Vec<Option<(Vec<[f32; 3]>, Vec<[f32; 3]>, Vec<u32>)>> =
        vec![None; manifest.components.len()];
    let mut load_errors: Vec<Option<String>> = vec![None; manifest.components.len()];
    std::thread::scope(|scope| {
        let handles: Vec<_> = manifest
            .components
            .iter()
            .enumerate()
            .filter(|(_, component)| component.present)
            .map(|(index, component)| {
                let path = component.path.clone();
                scope.spawn(move || (index, crate::load_part_geometry(&path, chord)))
            })
            .collect();
        for handle in handles {
            let (index, result) = handle.join().expect("part loader panicked");
            match result {
                Ok(mesh) => loaded_meshes[index] = mesh,
                Err(error) => load_errors[index] = Some(error.to_string()),
            }
        }
    });

    for (index, component) in manifest.components.iter().enumerate() {
        if !component.present {
            notes.push(format!("missing {}", component.name));
            continue;
        }
        let bytes = match part_bytes.get(&component.name) {
            Some(bytes) => bytes,
            None => continue,
        };
        let placement = placements
            .iter()
            .find(|placement| find_bytes(bytes, &placement_uid_bytes(&placement.uuid)).is_some());
        let Some(placement) = placement else {
            notes.push(format!("no placement for {}", component.name));
            continue;
        };
        if let Some(error) = &load_errors[index] {
            notes.push(format!("{}: {error}", component.name));
            continue;
        }
        let Some((local, local_normals, indices)) = loaded_meshes[index].take() else {
            notes.push(format!("no renderable geometry in {}", component.name));
            continue;
        };
        let mut part = ScenePart {
            label: component.name.clone(),
            uuid: placement.uuid.clone(),
            matrix: placement.matrix,
            base_matrix: placement.matrix,
            positions: local.clone(),
            normals: local_normals.clone(),
            rest: local.clone(),
            rest_normals: local_normals.clone(),
            local,
            local_normals,
            indices,
            color: component_color(&component.name),
            track: None,
            placement_z: placement.matrix[11],
            visible: true,
            moving: false,
        };
        part.apply_matrix();
        parts.push(part);
    }
    if parts.is_empty() {
        bail!("no component of {} could be loaded", assembly.display());
    }

    let mut scene = Scene {
        source,
        assembly,
        root,
        parts,
        animation: None,
        motion_model: None,
        rmd_path: None,
        driver: None,
        base_velocity: None,
        driver_velocity: 0.0,
        frame: 0,
        playing: true,
        notes,
        speed: 1.0,
    };

    let rad = find_rad(&scene.source);
    if let Some(rad) = rad {
        match parse_rad(&rad) {
            Ok(animation) => {
                let moving = animation.moving_tracks();
                if !moving.is_empty() {
                    let assignments: Vec<(usize, f64)> = moving
                        .iter()
                        .map(|track| (*track, animation.axis_z(*track).unwrap_or(f64::NAN)))
                        .collect();
                    let mut taken = vec![false; assignments.len()];
                    for part in &mut scene.parts {
                        if !part.label.contains("齿轮轴") {
                            continue;
                        }
                        let mut best: Option<(usize, f64)> = None;
                        for (index, (_, z)) in assignments.iter().enumerate() {
                            if taken[index] || !z.is_finite() {
                                continue;
                            }
                            let distance = (z - part.placement_z).abs();
                            if best.is_none_or(|(_, value)| distance < value) {
                                best = Some((index, distance));
                            }
                        }
                        if let Some((index, _)) = best {
                            taken[index] = true;
                            part.track = Some(assignments[index].0);
                            part.moving = true;
                            scene.notes.push(format!(
                                "{} driven by RAD track {} (axis z {:.3})",
                                part.label, assignments[index].0, assignments[index].1
                            ));
                        }
                    }
                }
                scene.notes.push(format!(
                    "motion {}: {} frame(s), {} track(s), {} moving",
                    rad.file_name().unwrap_or_default().to_string_lossy(),
                    animation.frame_count(),
                    animation.tracks.len(),
                    animation.moving_tracks().len()
                ));
                scene.animation = Some(animation);
            }
            Err(error) => scene.notes.push(format!("RAD: {error}")),
        }
    } else if is_sim(&scene.source) {
        scene
            .notes
            .push("no .rad animation next to the sim; parts show the reference pose".to_string());
    }

    scene.set_frame(0);

    // The RecurDyn text model exposes the driver function that produced the
    // RAD animation, so it can be edited and re-exported.
    if is_sim(&scene.source) {
        if let Some(rmd) = find_rmd(&scene.source) {
            match crate::motion_model::MotionModel::parse(&rmd) {
                Ok(model) => {
                    if let Some(driver) = model.drivers.first() {
                        scene.driver = Some(driver.name.clone());
                        scene.base_velocity = driver.velocity;
                        scene.driver_velocity = driver.velocity.unwrap_or(0.0);
                        scene.notes.push(format!(
                            "driver {}: velocity {:?} (acceleration {:?})",
                            driver.name, driver.velocity, driver.acceleration
                        ));
                    }
                    scene.rmd_path = Some(rmd);
                    scene.motion_model = Some(model);
                }
                Err(error) => scene.notes.push(format!("RMD: {error}")),
            }
        }
    }

    Ok(scene)
}

fn find_bytes(haystack: &[u8], needle: &[u8]) -> Option<usize> {
    if needle.is_empty() || needle.len() > haystack.len() {
        return None;
    }
    haystack
        .windows(needle.len())
        .position(|window| window == needle)
}

/// Print a one-screen summary of a loaded scene.
pub fn print_summary(scene: &Scene) {
    println!("== {}", scene.source.display());
    println!(
        "assembly={} root={}",
        scene.assembly.display(),
        scene.root.display()
    );
    let moving = scene.parts.iter().filter(|part| part.moving).count();
    println!(
        "parts={} triangles={} moving={} frames={}",
        scene.parts.len(),
        scene.triangles(),
        moving,
        scene.frame_count()
    );
    for part in &scene.parts {
        println!(
            "  {:<20} {:>8} tris  {}",
            part.label,
            part.triangles(),
            if part.moving { "animated" } else { "static" }
        );
    }
    for note in &scene.notes {
        println!("  note: {note}");
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn state_round_trips_through_json() {
        let state = SceneState {
            version: 1,
            frame: 42,
            playing: true,
            hide_static: false,
            speed: 2.5,
            view: Some(SceneViewState {
                target: [1.0, 2.0, 3.0],
                yaw: 0.5,
                pitch: 0.25,
                distance: 120.0,
            }),
            visible: [("2.主动齿轮轴.prt".to_string(), false)]
                .into_iter()
                .collect(),
        };
        let text = serde_json::to_string(&state).unwrap();
        let back: SceneState = serde_json::from_str(&text).unwrap();
        assert_eq!(back.frame, 42);
        assert_eq!(back.speed_or_default(), 2.5);
        assert_eq!(back.visible.len(), 1);
        assert_eq!(back.view.unwrap().target, [1.0, 2.0, 3.0]);
    }

    #[test]
    fn placement_edit_updates_geometry_and_collects_updates() {
        let mut matrix = [0.0; 12];
        matrix[0] = 1.0;
        matrix[5] = 1.0;
        matrix[10] = 1.0;
        let local = vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
        let normals = vec![[0.0, 0.0, 1.0]; 3];
        let part = ScenePart {
            label: "part.prt".to_string(),
            uuid: "uid".to_string(),
            matrix,
            base_matrix: matrix,
            positions: local.clone(),
            normals: normals.clone(),
            rest: local.clone(),
            rest_normals: normals.clone(),
            local,
            local_normals: normals,
            indices: vec![0, 1, 2],
            color: [1.0, 1.0, 1.0],
            track: None,
            placement_z: 0.0,
            visible: true,
            moving: false,
        };
        let mut scene = Scene {
            source: PathBuf::from("/tmp/motion_1.sim"),
            assembly: PathBuf::from("/tmp/assembly.prt"),
            root: PathBuf::from("/tmp"),
            parts: vec![part],
            animation: None,
            motion_model: None,
            rmd_path: None,
            driver: None,
            base_velocity: None,
            driver_velocity: 0.0,
            frame: 0,
            playing: false,
            notes: Vec::new(),
            speed: 1.0,
        };
        let mut moved = matrix;
        moved[3] = 5.0;
        scene.set_placement(0, moved);
        assert_eq!(scene.parts[0].rest[0], [5.0, 0.0, 0.0]);
        let updates = scene.edited_placements();
        assert_eq!(updates.len(), 1);
        assert_eq!(updates[0].uuid, "uid");
        assert_eq!(updates[0].matrix[3], 5.0);
        assert_eq!(
            scene.edited_assembly_path(),
            PathBuf::from("/tmp/assembly-edited.prt")
        );
    }

    #[test]
    fn default_state_path_appends_the_suffix() {
        assert_eq!(
            default_state_path(Path::new("/tmp/motion_1.sim")),
            PathBuf::from("/tmp/motion_1.sim.nxview.json")
        );
    }
}
