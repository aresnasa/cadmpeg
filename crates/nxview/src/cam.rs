// SPDX-License-Identifier: Apache-2.0
//! Minimal NX CAM path extraction from the legacy CFB `UG_PART/UG_PART`
//! stream. The Path OM sections store 3D linear/circular/helical move records
//! with a per-section monotone offset table.
//!
//! The offset table is grouped by record class, not by machining order. Each
//! move carries a monotonically increasing event id, so sorting by id within a
//! section restores the local path order. Circular and helical records also
//! carry the tool axis, arc centre, radius and a direction flag; those fields
//! are used to expand the moves into a sampled polyline for display and PLY
//! export.

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
pub struct CamMove {
    pub id: u32,
    pub section: usize,
    pub kind: MoveKind,
    /// Start point in millimetres. Filled from the previous event in the same
    /// section after the offset table has been sorted by event id.
    pub start: [f64; 3],
    /// End point in millimetres (NX legacy Path records store mm here).
    pub end: [f64; 3],
    pub center: Option<[f64; 3]>,
    /// Unit tool axis for circular/helical records.
    pub axis: Option<[f64; 3]>,
    pub radius: Option<f64>,
    pub clockwise: bool,
    /// Raw motion/event byte. 1 is rapid, 2 is linear feed and 3 is arc feed in
    /// the samples inspected so far; 4/6 are retract/plunge moves.
    pub motion: u8,
}

#[derive(Debug, Clone, Copy)]
pub struct CamSample {
    pub point: [f64; 3],
    pub section: usize,
    pub rapid: bool,
}

#[derive(Debug, Clone, Copy)]
pub struct CamSegment {
    pub a: [f64; 3],
    pub b: [f64; 3],
    pub section: usize,
    pub rapid: bool,
}

#[derive(Debug, Clone)]
pub struct CamSection {
    pub index: usize,
    pub stream_offset: usize,
    pub moves: usize,
    pub operations: Vec<String>,
}

#[derive(Debug, Default)]
pub struct CamPart {
    pub path_sections: usize,
    pub sections: Vec<CamSection>,
    pub moves: Vec<CamMove>,
    pub segments: Vec<CamSegment>,
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

    /// Return the sampled polyline and the edge list indexing it. Consecutive
    /// segments share their endpoint, so the vertex count stays proportional
    /// to the number of generated arc samples rather than twice the segment
    /// count.
    pub fn polyline(&self) -> (Vec<CamSample>, Vec<[u32; 2]>) {
        let mut samples: Vec<CamSample> = Vec::new();
        let mut edges: Vec<[u32; 2]> = Vec::new();
        for segment in &self.segments {
            let start = match samples.last() {
                Some(sample) if sample.point == segment.a => samples.len() as u32 - 1,
                _ => {
                    samples.push(CamSample {
                        point: segment.a,
                        section: segment.section,
                        rapid: segment.rapid,
                    });
                    samples.len() as u32 - 1
                }
            };
            samples.push(CamSample {
                point: segment.b,
                section: segment.section,
                rapid: segment.rapid,
            });
            let end = samples.len() as u32 - 1;
            edges.push([start, end]);
        }
        (samples, edges)
    }

    pub fn bounds(&self) -> ([f64; 3], [f64; 3]) {
        let mut min = [f64::INFINITY; 3];
        let mut max = [f64::NEG_INFINITY; 3];
        if self.segments.is_empty() {
            for mv in &self.moves {
                for axis in 0..3 {
                    min[axis] = min[axis].min(mv.end[axis]);
                    max[axis] = max[axis].max(mv.end[axis]);
                }
            }
        } else {
            for segment in &self.segments {
                for point in [segment.a, segment.b] {
                    for axis in 0..3 {
                        min[axis] = min[axis].min(point[axis]);
                        max[axis] = max[axis].max(point[axis]);
                    }
                }
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
            let index = part.sections.len();
            let mut section = section;
            // Class-grouped offset table -> event-id order.
            section.moves.sort_by_key(|mv| mv.id);
            let move_count = section.moves.len();
            let mut previous: Option<[f64; 3]> = None;
            for mv in &mut section.moves {
                mv.section = index;
                if let Some(start) = previous {
                    mv.start = start;
                    append_move_segments(&mut part.segments, mv, index);
                }
                previous = Some(mv.end);
            }
            part.path_sections += 1;
            part.moves.extend(section.moves);
            part.operations.extend(section.operations.iter().cloned());
            part.sections.push(CamSection {
                index,
                stream_offset: offset,
                moves: move_count,
                operations: section.operations,
            });
        } else if debug {
            eprintln!("  rejected section took={:?}", section_started.elapsed());
        }
    }

    part.operations.sort();
    part.operations.dedup();
    if debug {
        eprintln!(
            "cam scan={:?} sections={} moves={} segments={}",
            started.elapsed(),
            part.path_sections,
            part.moves.len(),
            part.segments.len()
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
        let motion = record.get(5).copied().unwrap_or(0);

        match record.len() {
            31 => {
                // 4-byte id, 3 flag bytes, three little-endian f64 millimetres.
                if let Some(end) = point_at(record, 7) {
                    moves.push(CamMove {
                        id,
                        section: 0,
                        kind: MoveKind::Linear,
                        start: end,
                        end,
                        center: None,
                        axis: None,
                        radius: None,
                        clockwise: false,
                        motion,
                    });
                }
            }
            96 => {
                if let Some(end) = point_at(record, 7) {
                    moves.push(CamMove {
                        id,
                        section: 0,
                        kind: MoveKind::Circular,
                        start: end,
                        end,
                        center: point_at(record, 55),
                        axis: point_at(record, 31),
                        radius: f64_at(record, 79),
                        // Observed values: 1 = clockwise, 2 = counter-clockwise,
                        // 6 = full circle / helical marker.
                        clockwise: record.get(95).copied() == Some(1),
                        motion,
                    });
                }
            }
            104 => {
                if let Some(end) = point_at(record, 7) {
                    moves.push(CamMove {
                        id,
                        section: 0,
                        kind: MoveKind::Helical,
                        start: end,
                        end,
                        center: point_at(record, 55),
                        axis: point_at(record, 31),
                        radius: f64_at(record, 79),
                        // Helical records use byte 95 as a class marker (6),
                        // not the circular direction flag. Treat the sweep as
                        // counter-clockwise; the endpoint determines the pitch.
                        clockwise: false,
                        motion,
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

/// Expand one move into straight or arc segments. Circular/helical records are
/// only expanded when the previous endpoint actually lies on the recorded
/// circle; otherwise a missing link or a section boundary makes the arc start
/// unknown and a straight segment is the honest fallback.
fn append_move_segments(out: &mut Vec<CamSegment>, mv: &CamMove, section: usize) {
    let rapid = !matches!(mv.motion, 2 | 3 | 7);
    if matches!(mv.kind, MoveKind::Circular | MoveKind::Helical) {
        if let (Some(center), Some(axis), Some(radius)) = (mv.center, mv.axis, mv.radius) {
            let start_radius = distance(mv.start, center);
            let end_radius = distance(mv.end, center);
            let tolerance = (radius.abs() * 0.02).max(0.05);
            let axis_length = dot(axis, axis).sqrt();
            if axis_length > 1e-9
                && (start_radius - radius).abs() <= tolerance
                && (end_radius - radius).abs() <= tolerance
            {
                let axis = [
                    axis[0] / axis_length,
                    axis[1] / axis_length,
                    axis[2] / axis_length,
                ];
                let samples = sample_arc(mv, center, axis, radius);
                for pair in samples.windows(2) {
                    out.push(CamSegment {
                        a: pair[0],
                        b: pair[1],
                        section,
                        rapid,
                    });
                }
                return;
            }
        }
    }
    if distance(mv.start, mv.end) > 1e-9 {
        out.push(CamSegment {
            a: mv.start,
            b: mv.end,
            section,
            rapid,
        });
    }
}

fn sample_arc(mv: &CamMove, center: [f64; 3], axis: [f64; 3], radius: f64) -> Vec<[f64; 3]> {
    let radius = radius.abs();
    let start = sub(mv.start, center);
    let end = sub(mv.end, center);
    // Build a stable in-plane basis. `u` points at the start, `v` is 90 degrees
    // around the tool axis.
    let mut u = sub(start, scale(axis, dot(start, axis)));
    let u_len = dot(u, u).sqrt();
    if u_len < 1e-9 {
        u = perpendicular(axis);
    } else {
        u = scale(u, 1.0 / u_len);
    }
    let v = cross(axis, u);
    let start_axial = dot(start, axis);
    let end_axial = dot(end, axis);
    let theta0 = 0.0_f64;
    let theta1 = dot(end, v).atan2(dot(end, u));
    let mut raw_sweep = theta1 - theta0;
    let tau = std::f64::consts::TAU;
    let pi = std::f64::consts::PI;
    while raw_sweep > pi {
        raw_sweep -= tau;
    }
    while raw_sweep < -pi {
        raw_sweep += tau;
    }
    let mut sweep = raw_sweep;
    if mv.kind == MoveKind::Helical {
        // Helical records carry a class marker rather than the circular
        // direction flag. Keep the geometric sweep and treat a closed XY loop
        // with a Z change as one full turn.
        if sweep.abs() < 1e-6 && (end_axial - start_axial).abs() > 1e-6 {
            sweep = tau;
        }
    } else if mv.clockwise {
        if sweep >= 0.0 {
            sweep -= tau;
        }
    } else {
        if sweep <= 0.0 {
            sweep += tau;
        }
    }
    // A shallow arc almost always means the previous event belongs to another
    // region and the direction flag no longer describes a real full turn.
    // Drawing the short geometric sweep avoids multi-kilometre circles around
    // a far-away centre.
    if !mv.clockwise
        && sweep.abs() > 1.5 * pi
        && raw_sweep.abs() < 0.5 * pi
        && distance(mv.start, mv.end) > 1e-6
    {
        sweep = raw_sweep;
    }
    if mv.clockwise
        && sweep.abs() > 1.5 * pi
        && raw_sweep.abs() < 0.5 * pi
        && distance(mv.start, mv.end) > 1e-6
    {
        sweep = raw_sweep;
    }

    let steps = ((sweep.abs() / (15.0_f64.to_radians())).ceil() as usize).clamp(2, 96);
    let mut samples = Vec::with_capacity(steps + 1);
    for step in 0..=steps {
        let t = step as f64 / steps as f64;
        let angle = theta0 + sweep * t;
        let axial = start_axial + (end_axial - start_axial) * t;
        let point = add(
            add(center, scale(u, radius * angle.cos())),
            add(scale(v, radius * angle.sin()), scale(axis, axial)),
        );
        samples.push(point);
    }
    if let Some(first) = samples.first_mut() {
        *first = mv.start;
    }
    if let Some(last) = samples.last_mut() {
        *last = mv.end;
    }
    samples
}

fn perpendicular(axis: [f64; 3]) -> [f64; 3] {
    let candidate = if axis[0].abs() < 0.9 {
        [1.0, 0.0, 0.0]
    } else {
        [0.0, 1.0, 0.0]
    };
    let result = cross(axis, candidate);
    let length = dot(result, result).sqrt().max(1e-12);
    scale(result, 1.0 / length)
}

fn add(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn scale(a: [f64; 3], factor: f64) -> [f64; 3] {
    [a[0] * factor, a[1] * factor, a[2] * factor]
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn distance(a: [f64; 3], b: [f64; 3]) -> f64 {
    dot(sub(a, b), sub(a, b)).sqrt()
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

fn f64_at(bytes: &[u8], offset: usize) -> Option<f64> {
    let slice = bytes.get(offset..offset.checked_add(8)?)?;
    let value = f64::from_le_bytes(slice.try_into().ok()?);
    value.is_finite().then_some(value)
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
        let (samples, edges) = self.polyline();
        let mut file = File::create(path)?;
        writeln!(file, "ply")?;
        writeln!(file, "format ascii 1.0")?;
        writeln!(file, "comment NX CAM sampled toolpath (mm)")?;
        writeln!(file, "element vertex {}", samples.len())?;
        writeln!(file, "property float x")?;
        writeln!(file, "property float y")?;
        writeln!(file, "property float z")?;
        writeln!(file, "property uchar rapid")?;
        writeln!(file, "element edge {}", edges.len())?;
        writeln!(file, "property int vertex1")?;
        writeln!(file, "property int vertex2")?;
        writeln!(file, "end_header")?;
        for sample in &samples {
            writeln!(
                file,
                "{} {} {} {}",
                sample.point[0] as f32,
                sample.point[1] as f32,
                sample.point[2] as f32,
                u8::from(sample.rapid)
            )?;
        }
        for edge in &edges {
            writeln!(file, "{} {}", edge[0], edge[1])?;
        }
        Ok(())
    }
}
