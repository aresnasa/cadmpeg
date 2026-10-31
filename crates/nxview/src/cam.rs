// SPDX-License-Identifier: Apache-2.0
//! Minimal NX CAM path extraction from the legacy CFB `UG_PART/UG_PART`
//! stream. The Path OM sections store 3D linear/circular/helical move records
//! with a per-section monotone offset table.

use std::collections::BTreeSet;
use std::fs::File;
use std::io::{Read, Write};
use std::path::Path;

use anyhow::{bail, Context};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MoveKind {
    Linear,
    Circular,
    Helical,
}

#[derive(Debug, Clone)]
#[allow(dead_code)]
pub struct CamMove {
    pub id: u32,
    pub kind: MoveKind,
    /// End point in millimetres (NX legacy Path records store mm here).
    pub end: [f64; 3],
    pub center: Option<[f64; 3]>,
}

#[derive(Debug, Default)]
pub struct CamPart {
    pub path_sections: usize,
    pub moves: Vec<CamMove>,
    pub operations: Vec<String>,
}

impl CamPart {
    pub fn linear(&self) -> usize {
        self.moves
            .iter()
            .filter(|m| m.kind == MoveKind::Linear)
            .count()
    }
    pub fn circular(&self) -> usize {
        self.moves
            .iter()
            .filter(|m| m.kind == MoveKind::Circular)
            .count()
    }
    pub fn helical(&self) -> usize {
        self.moves
            .iter()
            .filter(|m| m.kind == MoveKind::Helical)
            .count()
    }
    pub fn bounds(&self) -> ([f64; 3], [f64; 3]) {
        let mut min = [f64::INFINITY; 3];
        let mut max = [f64::NEG_INFINITY; 3];
        for mv in &self.moves {
            for axis in 0..3 {
                min[axis] = min[axis].min(mv.end[axis]);
                max[axis] = max[axis].max(mv.end[axis]);
            }
        }
        (min, max)
    }
}

pub fn load(path: &Path) -> anyhow::Result<CamPart> {
    let started = std::time::Instant::now();
    let data = read_ug_part(path)?;
    let debug = std::env::var("NXVIEW_CAM_DEBUG").is_ok();
    if debug {
        eprintln!("UG_PART bytes={} read={:?}", data.len(), started.elapsed());
    }
    let mut part = CamPart::default();

    let mut cursor = 0usize;
    while cursor + 4 < data.len() {
        let Some(relative) = data[cursor..]
            .windows(4)
            .position(|window| window == b"OM\x01\x01")
        else {
            break;
        };
        let offset = cursor + relative;
        cursor = offset + 4;
        if debug {
            eprintln!(
                "OM candidate at {offset} count={:?} size={:?}",
                u32_at(&data, offset + 0x36),
                u32_at(&data, offset + 0x52)
            );
        }
        let section_started = std::time::Instant::now();
        if let Some(section) = parse_path_section(&data, offset) {
            if debug {
                eprintln!(
                    "  path section moves={} ops={:?} took={:?}",
                    section.moves.len(),
                    section.operations,
                    section_started.elapsed()
                );
            }
            part.path_sections += 1;
            part.moves.extend(section.moves);
            part.operations.extend(section.operations);
        } else if debug {
            eprintln!("  rejected section took={:?}", section_started.elapsed());
        }
    }

    part.operations.sort();
    part.operations.dedup();
    if debug {
        eprintln!(
            "cam scan={:?} sections={} moves={}",
            started.elapsed(),
            part.path_sections,
            part.moves.len()
        );
    }
    Ok(part)
}

fn read_ug_part(path: &Path) -> anyhow::Result<Vec<u8>> {
    let mut compound =
        cfb::open(path).with_context(|| format!("{} is not a legacy CFB part", path.display()))?;
    let mut stream = match compound.open_stream("/UG_PART/UG_PART") {
        Ok(stream) => stream,
        Err(_) => compound
            .open_stream("UG_PART/UG_PART")
            .context("missing UG_PART/UG_PART stream")?,
    };
    let mut data = Vec::new();
    stream.read_to_end(&mut data)?;
    if data.len() < 8 || &data[..6] != b"\r\x01UGII" {
        bail!("UG_PART/UG_PART does not carry a UGII payload");
    }
    if std::env::var("NXVIEW_CAM_DEBUG").is_ok() {
        let _ = std::fs::write(r"C:\RE\cfb_ugpart.bin", &data);
    }
    Ok(data)
}

struct Section {
    moves: Vec<CamMove>,
    operations: Vec<String>,
}

fn parse_path_section(data: &[u8], offset: usize) -> Option<Section> {
    let debug = std::env::var("NXVIEW_CAM_DEBUG").is_ok();
    let section_end = offset
        .checked_add(8)?
        .checked_add(u32_at(data, offset + 0x52)? as usize)?;
    if debug {
        eprintln!(
            "  parse off={offset} end={section_end} len={}",
            section_end.saturating_sub(offset)
        );
    }
    if section_end > data.len() || section_end <= offset {
        return None;
    }
    let section = data.get(offset..section_end)?;
    let header = section.get(..section.len().min(0x3000))?;
    if section.len() < 0x2000
        || !header.windows(9).any(|window| window == b"UGS::PATH")
        || !(header.windows(14).any(|window| window == b"PATH_3x_linear")
            || header
                .windows(16)
                .any(|window| window == b"PATH_3x_circular")
            || header
                .windows(15)
                .any(|window| window == b"PATH_3x_helical"))
    {
        if debug {
            eprintln!("  no 3D PATH class marker / too short");
        }
        return None;
    }
    let count = u32_at(section, 0x36)? as usize;
    if debug {
        eprintln!("  path count={count}");
    }
    if count == 0 || count > 50_000 {
        return None;
    }

    let index_start = find_entity_index(section, count)?;
    if debug {
        eprintln!("  index start={index_start:#x}");
    }
    let index = |slot: usize| u32_at(section, index_start + slot * 4).map(|value| value as usize);
    let mut moves = Vec::new();
    let mut operations = BTreeSet::new();

    for ordinal in 0..count {
        let start = index(ordinal)?;
        let end = index(ordinal + 1).unwrap_or(section.len());
        if start == 0 || start >= section.len() {
            continue;
        }
        let end = if end > start && end <= section.len() {
            end
        } else {
            section.len()
        };
        let record = section.get(start..end)?;
        if record.len() < 4 {
            continue;
        }
        let id = u32_at(record, 0)?;

        match record.len() {
            31 => {
                // 4-byte id, 3 flag bytes, three little-endian f64 millimetres.
                if let Some(end) = point_at(record, 7) {
                    moves.push(CamMove {
                        id,
                        kind: MoveKind::Linear,
                        end,
                        center: None,
                    });
                }
            }
            96 => {
                if let Some(end) = point_at(record, 7) {
                    moves.push(CamMove {
                        id,
                        kind: MoveKind::Circular,
                        end,
                        center: point_at(record, 55),
                    });
                }
            }
            104 => {
                if let Some(end) = point_at(record, 7) {
                    moves.push(CamMove {
                        id,
                        kind: MoveKind::Helical,
                        end,
                        center: point_at(record, 55),
                    });
                }
            }
            16 | 38 | 57 => {
                for text in printable_runs(record) {
                    if plausible_operation_name(&text) {
                        operations.insert(text);
                    }
                }
            }
            _ => {}
        }
    }

    Some(Section {
        moves,
        operations: operations.into_iter().collect(),
    })
}

/// The Path section stores a `(count + 1)` u32 offset table. Its values are
/// section-relative and monotone; locate it by scanning the small header area.
fn find_entity_index(section: &[u8], count: usize) -> Option<usize> {
    let deadline = std::time::Instant::now() + std::time::Duration::from_millis(150);
    let table_bytes = count.checked_add(1)?.checked_mul(4)?;
    let max_start = section.len().checked_sub(table_bytes)?.min(0x8000);
    for start in 0x300..max_start {
        if std::time::Instant::now() > deadline {
            return None;
        }
        let Some(first) = u32_at(section, start) else {
            continue;
        };
        let first = first as usize;
        if first < start + table_bytes || first >= section.len() {
            continue;
        }
        let mut previous = first;
        let mut good = true;
        let quick = count.min(32);
        for slot in 1..quick {
            let Some(value) = u32_at(section, start + slot * 4) else {
                good = false;
                break;
            };
            let value = value as usize;
            if value < previous || value >= section.len() {
                good = false;
                break;
            }
            previous = value;
        }
        if !good {
            continue;
        }
        for slot in quick..count {
            let Some(value) = u32_at(section, start + slot * 4) else {
                good = false;
                break;
            };
            let value = value as usize;
            if value < previous || value >= section.len() {
                good = false;
                break;
            }
            previous = value;
        }
        if good {
            return Some(start);
        }
    }
    None
}

fn u32_at(bytes: &[u8], offset: usize) -> Option<u32> {
    let slice = bytes.get(offset..offset.checked_add(4)?)?;
    Some(u32::from_le_bytes(slice.try_into().ok()?))
}

fn point_at(bytes: &[u8], offset: usize) -> Option<[f64; 3]> {
    let slice = bytes.get(offset..offset.checked_add(24)?)?;
    let x = f64::from_le_bytes(slice[0..8].try_into().ok()?);
    let y = f64::from_le_bytes(slice[8..16].try_into().ok()?);
    let z = f64::from_le_bytes(slice[16..24].try_into().ok()?);
    // Path partitions also contain typed IPW and setup records whose 31-byte
    // neighbours are not moves. Keep only plausible CAM coordinates.
    [x, y, z]
        .iter()
        .all(|value| value.is_finite() && value.abs() < 1.0e6)
        .then_some([x, y, z])
}

fn plausible_operation_name(text: &str) -> bool {
    (4..=64).contains(&text.len())
        && text
            .chars()
            .next()
            .is_some_and(|ch| ch.is_ascii_alphabetic())
        && text
            .chars()
            .all(|ch| ch.is_ascii_alphanumeric() || matches!(ch, '_' | '-' | ' ' | '.' | '/'))
        && text.chars().filter(|ch| ch.is_ascii_alphabetic()).count() >= 3
}

fn printable_runs(bytes: &[u8]) -> Vec<String> {
    let mut runs = Vec::new();
    let mut current = String::new();
    for &byte in bytes {
        if (0x20..=0x7e).contains(&byte) {
            current.push(byte as char);
        } else {
            if current.len() >= 4 {
                runs.push(std::mem::take(&mut current));
            } else {
                current.clear();
            }
        }
    }
    if current.len() >= 4 {
        runs.push(current);
    }
    runs
}

impl CamPart {
    pub fn write_ply(&self, path: &Path) -> anyhow::Result<()> {
        let mut file = File::create(path)?;
        writeln!(file, "ply")?;
        writeln!(file, "format ascii 1.0")?;
        writeln!(file, "comment NX CAM move endpoints (mm)")?;
        writeln!(file, "element vertex {}", self.moves.len())?;
        writeln!(file, "property float x")?;
        writeln!(file, "property float y")?;
        writeln!(file, "property float z")?;
        writeln!(file, "end_header")?;
        for mv in &self.moves {
            writeln!(
                file,
                "{} {} {}",
                mv.end[0] as f32, mv.end[1] as f32, mv.end[2] as f32
            )?;
        }
        Ok(())
    }
}
