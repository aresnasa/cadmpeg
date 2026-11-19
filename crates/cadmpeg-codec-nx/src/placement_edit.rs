// SPDX-License-Identifier: Apache-2.0
//! Byte-preserving assembly placement edits for NX `.prt` files.
//!
//! A placement frame is `45 01 04 01` (or `ff 00 45 01 04 01` on NX12),
//! twelve self-delimiting matrix atoms, a fixed trailer, a label frame, and a
//! UID frame. The writer re-encodes only the twelve atoms, keeps every other
//! byte of the frame, and rewrites the containing stream (and container) with
//! the existing copy-on-write helpers.

use std::collections::HashMap;
use std::fs;
use std::io::{Cursor, Read};
use std::path::{Path, PathBuf};

use crate::edit::{
    copy_cfb_with_replacements, replace_file, sha256_hex, temporary_path, EditError,
};
use crate::native::assembly_placement::{frames_in_bytes, ParsedFrameData};

const CFB_MAGIC: [u8; 8] = [0xD0, 0xCF, 0x11, 0xE0, 0xA1, 0xB1, 0x1A, 0xE1];
const PLACEMENT_STREAMS: [&str; 3] = ["UG_PART/UG_PART", "UG_PART/DisplayJT", "UG_PART/JT"];

/// One placement to replace, identified by the frame's UID text.
#[derive(Debug, Clone, PartialEq)]
pub struct PlacementUpdate {
    /// UID text carried by the frame (canonical UUID or NX OM UID).
    pub uuid: String,
    /// New row-major three-by-four matrix.
    pub matrix: [f64; 12],
}

/// Result of rewriting placement frames.
#[derive(Debug, Clone)]
pub struct PlacementEditReport {
    /// Original assembly path.
    pub source: PathBuf,
    /// Written assembly path.
    pub output: PathBuf,
    /// UIDs that were rewritten.
    pub updated: Vec<String>,
    /// Streams that differ from the source.
    pub changed_streams: Vec<String>,
    /// SHA-256 of the original file.
    pub source_sha256: String,
    /// SHA-256 of the written file.
    pub output_sha256: String,
    /// True when the writer reproduced the source byte for byte.
    pub byte_identical: bool,
}

/// One placement frame as read from an assembly.
#[derive(Debug, Clone, PartialEq)]
pub struct PlacementFrameInfo {
    /// UID text carried by the frame.
    pub uuid: String,
    /// Label text carried by the frame.
    pub label: String,
    /// Row-major three-by-four matrix.
    pub matrix: [f64; 12],
    /// Stream that carries the frame.
    pub stream: String,
    /// Stream offset in the file, when known.
    pub stream_offset: u64,
    /// Frame offset within the stream.
    pub frame_offset: usize,
    /// Frame byte length.
    pub frame_length: usize,
}

/// Placement frames of one stream.
struct StreamFrames {
    /// Stream name without a leading `/Root` or `/`.
    stream: String,
    /// Stream payload.
    bytes: Vec<u8>,
    /// Frames as `(offset, frame)` pairs inside `bytes`.
    frames: Vec<(usize, ParsedFrameData)>,
    /// File offset of the payload, when known.
    offset: u64,
}

/// Encode one matrix atom with the narrowest encodable form.
fn encode_atom(value: f64, out: &mut Vec<u8>) -> Result<(), EditError> {
    if value == 0.0 {
        out.push(if value.is_sign_negative() { 0x80 } else { 0x00 });
        return Ok(());
    }
    if value == 1.0 {
        out.push(0x01);
        return Ok(());
    }
    if value == -1.0 {
        out.push(0x81);
        return Ok(());
    }
    let scaled = value * 4.0;
    if !scaled.is_finite() {
        return Err(EditError::InvalidPlacement(format!(
            "matrix entry {value} is not finite when scaled"
        )));
    }
    let bytes = scaled.to_be_bytes();
    let lead = bytes[0].wrapping_sub(0x10);
    if matches!(lead, 0x20..=0x3f | 0xa0..=0xbf) {
        out.push(lead);
        out.extend_from_slice(&bytes[1..]);
        Ok(())
    } else {
        Err(EditError::InvalidPlacement(format!(
            "matrix entry {value} cannot be encoded as a shifted binary64 atom"
        )))
    }
}

/// Rebuild one frame with a new matrix, keeping its prefix, trailer, label and
/// UID bytes intact.
fn rebuild_frame(raw: &[u8], matrix: &[f64; 12]) -> Result<Vec<u8>, EditError> {
    let prefix_len = if raw.starts_with(&[0xff, 0x00, 0x45, 0x01, 0x04, 0x01]) {
        6
    } else if raw.starts_with(&[0x45, 0x01, 0x04, 0x01]) {
        4
    } else {
        return Err(EditError::InvalidPlacement(
            "frame does not start with a placement prefix".to_string(),
        ));
    };
    // Locate the end of the twelve atoms by walking the original frame.
    let mut cursor = prefix_len;
    for _ in 0..12 {
        cursor = skip_atom(raw, cursor)?;
    }
    let mut frame = Vec::with_capacity(raw.len() + 12);
    frame.extend_from_slice(&raw[..prefix_len]);
    for value in matrix {
        encode_atom(*value, &mut frame)?;
    }
    frame.extend_from_slice(&raw[cursor..]);
    Ok(frame)
}

fn skip_atom(raw: &[u8], offset: usize) -> Result<usize, EditError> {
    let marker = *raw
        .get(offset)
        .ok_or_else(|| EditError::InvalidPlacement("truncated matrix atom".to_string()))?;
    let width = match marker {
        0 | 1 | 0x80 | 0x81 => 1,
        0x20..=0x3f | 0xa0..=0xbf => 8,
        0x40..=0x5f | 0xc0..=0xdf => 4,
        other => {
            return Err(EditError::InvalidPlacement(format!(
                "invalid matrix atom marker 0x{other:02x}"
            )))
        }
    };
    offset
        .checked_add(width)
        .filter(|end| *end <= raw.len())
        .ok_or_else(|| EditError::InvalidPlacement("truncated matrix atom".to_string()))
}

fn collect_stream_frames(source: &[u8]) -> Result<Vec<StreamFrames>, EditError> {
    if source.starts_with(b"SPLMSSTR") {
        let container = crate::container::scan_bytes(source)
            .map_err(|error| EditError::MalformedContainer(error.to_string()))?;
        let mut streams = Vec::new();
        for entry in container.entries.iter().filter(|entry| {
            matches!(
                entry.content(),
                crate::container::EntryContent::PartPayload
                    | crate::container::EntryContent::DisplayJt
            )
        }) {
            let Some((offset, length)) = entry.file_span() else {
                continue;
            };
            let offset = usize::try_from(offset)
                .map_err(|_| EditError::MalformedContainer("stream offset overflow".into()))?;
            let length = usize::try_from(length)
                .map_err(|_| EditError::MalformedContainer("stream length overflow".into()))?;
            let bytes = source
                .get(offset..offset.saturating_add(length))
                .ok_or_else(|| EditError::MalformedContainer("stream span".into()))?
                .to_vec();
            let frames = frames_in_bytes(&bytes);
            if !frames.is_empty() {
                streams.push(StreamFrames {
                    stream: entry.name.trim_start_matches("/Root/").to_string(),
                    bytes,
                    frames,
                    offset: u64::try_from(offset).unwrap_or(0),
                });
            }
        }
        return Ok(streams);
    }
    if source.get(..8) == Some(&CFB_MAGIC) {
        let mut compound = cfb::CompoundFile::open(Cursor::new(source))
            .map_err(|error| EditError::MalformedContainer(error.to_string()))?;
        let paths: Vec<PathBuf> = compound
            .walk()
            .filter(|entry| entry.is_stream())
            .map(|entry| entry.path().to_path_buf())
            .collect();
        let mut streams = Vec::new();
        for path in paths {
            let name = path.to_string_lossy().replace('\\', "/");
            let name = name.trim_start_matches('/');
            if !PLACEMENT_STREAMS.contains(&name) {
                continue;
            }
            let mut stream = compound
                .open_stream(&path)
                .map_err(|_| EditError::MissingStream(name.to_string()))?;
            let mut bytes = Vec::new();
            stream.read_to_end(&mut bytes)?;
            let frames = frames_in_bytes(&bytes);
            if !frames.is_empty() {
                streams.push(StreamFrames {
                    stream: name.to_string(),
                    bytes,
                    frames,
                    offset: 0,
                });
            }
        }
        return Ok(streams);
    }
    Err(EditError::UnsupportedContainer(
        "only legacy CFB and SPLMSSTR .prt files are supported".to_string(),
    ))
}

/// Rewrite placement frames of `input` into `output`.
pub fn set_assembly_placements(
    input: &Path,
    output: &Path,
    updates: &[PlacementUpdate],
) -> Result<PlacementEditReport, EditError> {
    if input == output {
        return Err(EditError::Io(std::io::Error::new(
            std::io::ErrorKind::InvalidInput,
            "input and output must be different paths",
        )));
    }
    let source_bytes = fs::read(input)?;
    let source_sha256 = sha256_hex(&source_bytes);
    let streams = collect_stream_frames(&source_bytes)?;

    let mut available = Vec::new();
    let mut replacements: HashMap<String, Vec<u8>> = HashMap::new();
    let mut changed_streams = Vec::new();
    for stream in &streams {
        for (_, frame) in &stream.frames {
            available.push(frame.uuid.clone());
        }
        let mut edits: Vec<(usize, usize, Vec<u8>)> = Vec::new();
        for update in updates {
            for (offset, frame) in &stream.frames {
                if frame.uuid != update.uuid {
                    continue;
                }
                let raw = stream
                    .bytes
                    .get(*offset..offset.saturating_add(frame.length))
                    .ok_or_else(|| EditError::MalformedContainer("frame span".into()))?;
                edits.push((*offset, frame.length, rebuild_frame(raw, &update.matrix)?));
            }
        }
        if edits.is_empty() {
            continue;
        }
        edits.sort_by_key(|(offset, _, _)| std::cmp::Reverse(*offset));
        let mut bytes = stream.bytes.clone();
        for (offset, length, frame) in edits {
            bytes.splice(offset..offset + length, frame);
        }
        replacements.insert(stream.stream.clone(), bytes);
        changed_streams.push(stream.stream.clone());
    }
    for update in updates {
        if !available.iter().any(|uuid| uuid == &update.uuid) {
            return Err(EditError::InvalidPlacement(format!(
                "no placement frame for {}; available: {}",
                update.uuid,
                available.join(", ")
            )));
        }
    }

    let output_bytes = if replacements.is_empty() {
        None
    } else if source_bytes.starts_with(b"SPLMSSTR") {
        let mut current = source_bytes.clone();
        for (stream, payload) in &replacements {
            let container = crate::container::scan_bytes(current.as_slice())
                .map_err(|error| EditError::MalformedContainer(error.to_string()))?;
            let entry = container
                .entries
                .iter()
                .find(|entry| entry.name.trim_start_matches("/Root/") == stream)
                .ok_or_else(|| EditError::MissingStream(stream.clone()))?;
            let (offset, length) = entry
                .file_span()
                .ok_or_else(|| EditError::MissingStream(stream.clone()))?;
            let offset = usize::try_from(offset)
                .map_err(|_| EditError::MalformedContainer("stream offset overflow".into()))?;
            let length = usize::try_from(length)
                .map_err(|_| EditError::MalformedContainer("stream length overflow".into()))?;
            current = crate::edit::repack_splmsstr_payload(&current, offset, length, payload)?;
        }
        Some(current)
    } else {
        let scratch = temporary_path(output);
        let mut compound = cfb::CompoundFile::open(Cursor::new(source_bytes.as_slice()))
            .map_err(|error| EditError::MalformedContainer(error.to_string()))?;
        if let Err(error) = copy_cfb_with_replacements(&mut compound, &scratch, &replacements) {
            let _ = fs::remove_file(&scratch);
            return Err(error);
        }
        let bytes = fs::read(&scratch)?;
        let _ = fs::remove_file(&scratch);
        Some(bytes)
    };

    let Some(output_bytes) = output_bytes else {
        let temp = temporary_path(output);
        if let Err(error) = fs::copy(input, &temp) {
            let _ = fs::remove_file(&temp);
            return Err(error.into());
        }
        replace_file(&temp, output)?;
        let written = fs::read(output)?;
        return Ok(PlacementEditReport {
            source: input.to_path_buf(),
            output: output.to_path_buf(),
            updated: Vec::new(),
            changed_streams: Vec::new(),
            source_sha256,
            output_sha256: sha256_hex(&written),
            byte_identical: true,
        });
    };
    let temp = temporary_path(output);
    if let Err(error) = fs::write(&temp, &output_bytes) {
        let _ = fs::remove_file(&temp);
        return Err(error.into());
    }
    replace_file(&temp, output)?;

    // Re-read the written file and check every updated matrix round-trips.
    let written = fs::read(output)?;
    let written_frames = read_placement_frames(output)?;
    for update in updates {
        let Some(frame) = written_frames
            .iter()
            .find(|frame| frame.uuid == update.uuid)
        else {
            return Err(EditError::InvalidPlacement(format!(
                "written file lost placement {}",
                update.uuid
            )));
        };
        if frame
            .matrix
            .iter()
            .zip(update.matrix)
            .any(|(a, b)| (a - b).abs() > 1e-6)
        {
            return Err(EditError::InvalidPlacement(format!(
                "written placement {} does not match the requested matrix",
                update.uuid
            )));
        }
    }

    Ok(PlacementEditReport {
        source: input.to_path_buf(),
        output: output.to_path_buf(),
        updated: updates.iter().map(|update| update.uuid.clone()).collect(),
        changed_streams,
        source_sha256,
        output_sha256: sha256_hex(&written),
        byte_identical: false,
    })
}

/// Read the `UG_PART/UG_PART` payload of a part, whatever the container.
///
/// Legacy CFB streams are stored in sector chains, so reading the raw file
/// bytes is not enough to find a UID; callers must read the payload instead.
pub fn read_part_payload(path: &Path) -> Result<Vec<u8>, EditError> {
    let bytes = fs::read(path)?;
    if bytes.starts_with(b"SPLMSSTR") {
        let container = crate::container::scan_bytes(bytes.as_slice())
            .map_err(|error| EditError::MalformedContainer(error.to_string()))?;
        let entry = container
            .entries
            .iter()
            .find(|entry| entry.name.trim_start_matches("/Root/") == "UG_PART/UG_PART")
            .ok_or_else(|| EditError::MissingStream("UG_PART/UG_PART".to_string()))?;
        let (offset, length) = entry
            .file_span()
            .ok_or_else(|| EditError::MissingStream("UG_PART/UG_PART".to_string()))?;
        let offset = usize::try_from(offset)
            .map_err(|_| EditError::MalformedContainer("stream offset overflow".into()))?;
        let length = usize::try_from(length)
            .map_err(|_| EditError::MalformedContainer("stream length overflow".into()))?;
        return bytes
            .get(offset..offset.saturating_add(length))
            .map(<[u8]>::to_vec)
            .ok_or_else(|| EditError::MalformedContainer("stream span".into()));
    }
    if bytes.get(..8) == Some(&CFB_MAGIC) {
        let mut compound = cfb::CompoundFile::open(Cursor::new(bytes.as_slice()))
            .map_err(|error| EditError::MalformedContainer(error.to_string()))?;
        for name in ["/UG_PART/UG_PART", "UG_PART/UG_PART"] {
            let Ok(mut stream) = compound.open_stream(name) else {
                continue;
            };
            let mut payload = Vec::new();
            stream.read_to_end(&mut payload)?;
            return Ok(payload);
        }
        return Err(EditError::MissingStream("UG_PART/UG_PART".to_string()));
    }
    Ok(bytes)
}

/// Read one assembly's placements as full frames (UID, label, matrix).
pub fn read_placement_frames(path: &Path) -> Result<Vec<PlacementFrameInfo>, EditError> {
    let bytes = fs::read(path)?;
    let mut frames = Vec::new();
    for stream in collect_stream_frames(&bytes)? {
        for (offset, frame) in &stream.frames {
            frames.push(PlacementFrameInfo {
                uuid: frame.uuid.clone(),
                label: frame.label.clone(),
                matrix: frame.matrix,
                stream: stream.stream.clone(),
                stream_offset: stream.offset,
                frame_offset: *offset,
                frame_length: frame.length,
            });
        }
    }
    Ok(frames)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn decode_atom(raw: &[u8]) -> f64 {
        match raw[0] {
            0 => 0.0,
            1 => 1.0,
            0x80 => -0.0,
            0x81 => -1.0,
            0x20..=0x3f | 0xa0..=0xbf => {
                let mut bytes = [0_u8; 8];
                bytes.copy_from_slice(&raw[..8]);
                bytes[0] += 0x10;
                f64::from_be_bytes(bytes) / 4.0
            }
            0x40..=0x5f | 0xc0..=0xdf => {
                let mut bytes = [0_u8; 8];
                bytes[..4].copy_from_slice(&raw[..4]);
                bytes[0] -= 0x10;
                f64::from_be_bytes(bytes) / 4.0
            }
            other => panic!("unexpected marker 0x{other:02x}"),
        }
    }

    #[test]
    fn atoms_round_trip_through_the_decoder() {
        for value in [0.0, -0.0, 1.0, -1.0, 0.5, -1.5, 92.0, 1234.5, 1e-6] {
            let mut encoded = Vec::new();
            encode_atom(value, &mut encoded).unwrap();
            assert_eq!(decode_atom(&encoded).to_bits(), value.to_bits(), "{value}");
        }
    }

    #[test]
    fn frame_rebuild_keeps_trailer_and_label() {
        let mut frame = vec![0xff, 0x00, 0x45, 0x01, 0x04, 0x01];
        frame.extend([1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0]);
        frame.extend([0, 0, 0, 1, 63, 0xf0, 0, 0, 0]);
        frame.extend(b"\x03\x03A\0\x03\x26laptop.GenuineInt.1.2.3\0");
        let mut matrix = [0.0; 12];
        matrix[0] = 1.0;
        matrix[5] = 1.0;
        matrix[10] = 1.0;
        matrix[3] = 10.0;
        let rebuilt = rebuild_frame(&frame, &matrix).unwrap();
        // Prefix, trailer, label and UID are byte-identical; only the atoms move.
        assert!(rebuilt.starts_with(&frame[..6]));
        assert!(rebuilt.ends_with(&frame[frame.len() - 34..]));
    }
}
