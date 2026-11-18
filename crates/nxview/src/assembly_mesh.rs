// SPDX-License-Identifier: Apache-2.0
//! Compose the external child geometry of an NX assembly.

use std::collections::{BTreeSet, HashMap};
use std::fs::{self, File};
use std::io::Read;
use std::path::Path;

use anyhow::{bail, Context};
use cadmpeg_codec_nx::{assembly_placements, read_assembly_manifest, NxCodec};
use cadmpeg_ir::codec::{Codec, DecodeOptions};
use serde_json::json;

use crate::{load_mesh, MeshData};

const CFB_MAGIC: [u8; 8] = [0xD0, 0xCF, 0x11, 0xE0, 0xA1, 0xB1, 0x1A, 0xE1];
const UG_PART_STREAM: &str = "UG_PART/UG_PART";

pub(crate) struct AssemblyLoad {
    pub(crate) mesh: MeshData,
    pub(crate) report: serde_json::Value,
}

pub(crate) fn load_assembly_mesh(
    assembly: &Path,
    root: Option<&Path>,
    chord: f64,
) -> anyhow::Result<AssemblyLoad> {
    let manifest = read_assembly_manifest(assembly, root)?;
    let mut input = File::open(assembly)?;
    let result = NxCodec.decode(&mut input, &DecodeOptions::default())?;
    let placements = assembly_placements(result.ir()).map_err(anyhow::Error::msg)?;
    if manifest.components.is_empty() {
        bail!("{} has no external components", assembly.display());
    }
    if placements.is_empty() {
        bail!("{} has no assembly placement frames", assembly.display());
    }

    let bytes: Vec<Option<Vec<u8>>> = manifest
        .components
        .iter()
        .map(|component| {
            component
                .present
                .then(|| fs::read(&component.path).ok())
                .flatten()
        })
        .collect();
    let mut by_identity: HashMap<String, usize> = HashMap::new();
    for (index, component) in manifest.components.iter().enumerate() {
        let Some(data) = bytes[index].as_deref() else {
            continue;
        };
        for placement in &placements {
            if contains_identity(&component.path, data, &placement.uuid) {
                by_identity.entry(placement.uuid.clone()).or_insert(index);
                break;
            }
        }
    }

    let mut assignments = Vec::new();
    let mut unmatched = Vec::new();
    for placement in &placements {
        match by_identity.get(&placement.uuid) {
            Some(index) => assignments.push((*index, placement)),
            None => unmatched.push(json!({
                "label": placement.label,
                "identity": placement.uuid,
                "source_offset": placement.source_offset,
            })),
        }
    }
    if assignments.is_empty() {
        bail!("no placement identity matched an external component");
    }

    let mut combined = MeshData::default();
    let mut parts = Vec::new();
    let mut min = [f32::INFINITY; 3];
    let mut max = [f32::NEG_INFINITY; 3];
    for (index, placement) in &assignments {
        let component = &manifest.components[*index];
        let mut child = load_mesh(&component.path, chord)
            .with_context(|| format!("compose {}", component.path.display()))?;
        transform_mesh(&mut child, &placement.matrix);
        let vertices = child.positions.len();
        let triangles = child.indices.len() / 3;
        add_mesh(&mut combined, &child)?;
        for position in &child.positions {
            for axis in 0..3 {
                min[axis] = min[axis].min(position[axis]);
                max[axis] = max[axis].max(position[axis]);
            }
        }
        parts.push(json!({
            "name": component.name,
            "path": component.path.display().to_string(),
            "identity": placement.uuid,
            "matrix": placement.matrix,
            "translation": [
                placement.matrix[3],
                placement.matrix[7],
                placement.matrix[11],
            ],
            "vertices": vertices,
            "triangles": triangles,
            "faces_tessellated": child.faces_tessellated,
            "faces_refused": child.faces_refused,
        }));
    }

    let placed: BTreeSet<usize> = assignments.iter().map(|(index, _)| *index).collect();
    let present = manifest
        .components
        .iter()
        .filter(|component| component.present)
        .count();
    let unplaced: Vec<_> = manifest
        .components
        .iter()
        .enumerate()
        .filter(|(index, component)| component.present && !placed.contains(index))
        .map(|(_, component)| component.name.clone())
        .collect();
    let report = json!({
        "assembly": assembly.display().to_string(),
        "root": manifest.directory,
        "components": manifest.components.len(),
        "present": present,
        "placements": placements.len(),
        "placed_instances": parts.len(),
        "vertices": combined.positions.len(),
        "triangles": combined.indices.len() / 3,
        "faces_tessellated": combined.faces_tessellated,
        "faces_refused": combined.faces_refused,
        "bounds": {
            "min": min,
            "max": max,
        },
        "parts": parts,
        "unmatched_placements": unmatched,
        "unplaced_components": unplaced,
    });
    Ok(AssemblyLoad {
        mesh: combined,
        report,
    })
}

fn contains_identity(path: &Path, data: &[u8], identity: &str) -> bool {
    let needle = identity.as_bytes();
    if needle.is_empty() {
        return false;
    }
    if data.windows(needle.len()).any(|window| window == needle) {
        return true;
    }
    if !data.starts_with(&CFB_MAGIC) {
        return false;
    }
    let Ok(mut compound) = cfb::open(path) else {
        return false;
    };
    let stream_path = compound
        .walk()
        .find(|entry| {
            entry.is_stream()
                && entry
                    .path()
                    .to_string_lossy()
                    .replace('\\', "/")
                    .ends_with(UG_PART_STREAM)
        })
        .map(|entry| entry.path().to_path_buf());
    let Some(stream_path) = stream_path else {
        return false;
    };
    let Ok(stream) = compound.open_stream(&stream_path) else {
        return false;
    };
    let mut prefix = Vec::new();
    if stream.take(0x2000).read_to_end(&mut prefix).is_err() {
        return false;
    }
    prefix.windows(needle.len()).any(|window| window == needle)
}

fn transform_mesh(mesh: &mut MeshData, matrix: &[f64; 12]) {
    for position in &mut mesh.positions {
        *position = transform_position(matrix, *position);
    }
    for normal in &mut mesh.normals {
        let x = matrix[0] * f64::from(normal[0])
            + matrix[1] * f64::from(normal[1])
            + matrix[2] * f64::from(normal[2]);
        let y = matrix[4] * f64::from(normal[0])
            + matrix[5] * f64::from(normal[1])
            + matrix[6] * f64::from(normal[2]);
        let z = matrix[8] * f64::from(normal[0])
            + matrix[9] * f64::from(normal[1])
            + matrix[10] * f64::from(normal[2]);
        let length = (x * x + y * y + z * z).sqrt();
        *normal = if length > 0.0 {
            [
                (x / length) as f32,
                (y / length) as f32,
                (z / length) as f32,
            ]
        } else {
            [0.0, 1.0, 0.0]
        };
    }
    for point in &mut mesh.cam_points {
        *point = transform_position(matrix, *point);
    }
}

fn transform_position(matrix: &[f64; 12], position: [f32; 3]) -> [f32; 3] {
    let x = matrix[0] * f64::from(position[0])
        + matrix[1] * f64::from(position[1])
        + matrix[2] * f64::from(position[2])
        + matrix[3];
    let y = matrix[4] * f64::from(position[0])
        + matrix[5] * f64::from(position[1])
        + matrix[6] * f64::from(position[2])
        + matrix[7];
    let z = matrix[8] * f64::from(position[0])
        + matrix[9] * f64::from(position[1])
        + matrix[10] * f64::from(position[2])
        + matrix[11];
    [x as f32, y as f32, z as f32]
}

fn add_mesh(target: &mut MeshData, source: &MeshData) -> anyhow::Result<()> {
    let base = u32::try_from(target.positions.len())
        .map_err(|_| anyhow::anyhow!("assembly vertex count exceeds u32"))?;
    target.positions.extend_from_slice(&source.positions);
    target.normals.extend_from_slice(&source.normals);
    target
        .indices
        .extend(source.indices.iter().map(|index| base + index));
    target.faces_tessellated += source.faces_tessellated;
    target.faces_refused += source.faces_refused;
    let cam_base = u32::try_from(target.cam_points.len())
        .map_err(|_| anyhow::anyhow!("assembly CAM sample count exceeds u32"))?;
    target.cam_points.extend_from_slice(&source.cam_points);
    target.cam_sections.extend_from_slice(&source.cam_sections);
    target.cam_rapid.extend_from_slice(&source.cam_rapid);
    target.cam_edges.extend(
        source
            .cam_edges
            .iter()
            .map(|edge| [cam_base + edge[0], cam_base + edge[1]]),
    );
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::{add_mesh, transform_position};
    use crate::MeshData;

    #[test]
    fn transform_uses_row_major_three_by_four_matrix() {
        let matrix = [
            1.0, 0.0, 0.0, 10.0, 0.0, 0.0, 1.0, 20.0, 0.0, -1.0, 0.0, 30.0,
        ];
        assert_eq!(
            transform_position(&matrix, [2.0, 3.0, 4.0]),
            [12.0, 24.0, 27.0]
        );
    }

    #[test]
    fn add_mesh_rebases_indices_and_counts() {
        let mut target = MeshData {
            positions: vec![[0.0; 3]],
            normals: vec![[0.0, 0.0, 1.0]],
            indices: vec![0, 0, 0],
            ..MeshData::default()
        };
        let source = MeshData {
            positions: vec![[1.0; 3]; 3],
            normals: vec![[0.0, 0.0, 1.0]; 3],
            indices: vec![0, 1, 2],
            faces_tessellated: 1,
            ..MeshData::default()
        };
        add_mesh(&mut target, &source).expect("merge");
        assert_eq!(target.positions.len(), 4);
        assert_eq!(target.indices, vec![0, 0, 0, 1, 2, 3]);
        assert_eq!(target.faces_tessellated, 1);
    }
}
