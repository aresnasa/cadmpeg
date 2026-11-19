// SPDX-License-Identifier: Apache-2.0
//! Byte-preserving edits for NX `.prt` files.
//!
//! The first supported edit is the legacy CFB `part/attrs` XML stream. The
//! writer copies every other stream and storage verbatim, so unknown data does
//! not pass through the NX object model during a metadata edit.

use std::collections::HashMap;
use std::fs::{self, File};
use std::io::{Read, Seek, Write};
use std::path::{Path, PathBuf};

use sha2::{Digest, Sha256};

/// Name of the legacy CFB stream that stores part attributes as XML.
pub const PART_ATTRS_STREAM: &str = "part/attrs";

const CFB_MAGIC: [u8; 8] = [0xD0, 0xCF, 0x11, 0xE0, 0xA1, 0xB1, 0x1A, 0xE1];

/// One `<Attribute>` entry from the `part/attrs` XML stream.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PartAttribute {
    /// Display title / key.
    pub title: String,
    /// Stored string value.
    pub value: String,
}

/// Result of a byte-preserving edit.
#[derive(Debug, Clone)]
pub struct EditReport {
    /// Original part path.
    pub source: PathBuf,
    /// Written part path.
    pub output: PathBuf,
    /// Streams that differ from the source.
    pub changed_streams: Vec<String>,
    /// SHA-256 of the original file.
    pub source_sha256: String,
    /// SHA-256 of the written file.
    pub output_sha256: String,
    /// True when the writer reproduced the source byte for byte.
    pub byte_identical: bool,
}

/// Errors returned by the NX edit APIs.
#[derive(Debug, thiserror::Error)]
pub enum EditError {
    /// The file is not a legacy CFB part, or its container writer is not yet
    /// implemented.
    #[error("unsupported container: {0}")]
    UnsupportedContainer(String),
    /// A required CFB stream is absent.
    #[error("missing stream: {0}")]
    MissingStream(String),
    /// The container header or directory is malformed.
    #[error("malformed container: {0}")]
    MalformedContainer(String),
    /// The `part/attrs` payload is not the expected XML document.
    #[error("invalid part/attrs XML: {0}")]
    InvalidAttributeXml(String),
    /// An existing attribute has an incompatible owner.
    #[error("attribute already exists with a different owner: {0}")]
    ConflictingAttribute(String),
    /// A placement frame or matrix value cannot be written.
    #[error("invalid placement: {0}")]
    InvalidPlacement(String),
    /// Underlying I/O or CFB failure.
    #[error(transparent)]
    Io(#[from] std::io::Error),
}

/// Read the `part/attrs` XML of a legacy CFB part.
///
/// This is a small, side-effect-free read used by the editor and by tests. It
/// does not open the NX object model.
pub fn read_part_attributes(path: &Path) -> Result<Vec<PartAttribute>, EditError> {
    let bytes = fs::read(path)?;
    if bytes.starts_with(b"SPLMSSTR") {
        let (offset, length) = splmsstr_attrs_span(&bytes)?;
        let end = offset
            .checked_add(length)
            .filter(|end| *end <= bytes.len())
            .ok_or_else(|| EditError::MalformedContainer("part/attrs span exceeds file".into()))?;
        return parse_attributes(&bytes[offset..end]);
    }
    let mut compound = cfb::open(path)?;
    let xml = read_stream(&mut compound, PART_ATTRS_STREAM)?;
    parse_attributes(&xml)
}

fn splmsstr_attrs_span(bytes: &[u8]) -> Result<(usize, usize), EditError> {
    let container = crate::container::scan_bytes(bytes)
        .map_err(|error| EditError::MalformedContainer(error.to_string()))?;
    let entry = container
        .entries
        .iter()
        .find(|entry| entry.name == "/Root/part/attrs" || entry.name == "part/attrs")
        .ok_or_else(|| EditError::MissingStream(PART_ATTRS_STREAM.to_string()))?;
    let (offset, length) = entry
        .file_span()
        .ok_or_else(|| EditError::MissingStream(PART_ATTRS_STREAM.to_string()))?;
    Ok((
        usize::try_from(offset)
            .map_err(|_| EditError::MalformedContainer("offset overflow".into()))?,
        usize::try_from(length)
            .map_err(|_| EditError::MalformedContainer("length overflow".into()))?,
    ))
}

/// Set one part attribute and write the result to `output`.
///
/// All streams other than `part/attrs` are copied verbatim. If the value is
/// already present, the output is still written but `changed_streams` is empty
/// and `byte_identical` is expected to be true.
pub fn set_part_attribute(
    input: &Path,
    output: &Path,
    title: &str,
    value: &str,
) -> Result<EditReport, EditError> {
    let source_bytes = fs::read(input)?;
    let source_sha256 = sha256_hex(&source_bytes);
    if source_bytes.starts_with(b"SPLMSSTR") {
        return set_splmsstr_attribute(input, output, &source_bytes, title, value, source_sha256);
    }
    if source_bytes.len() < 8 || source_bytes[..8] != CFB_MAGIC {
        return Err(EditError::UnsupportedContainer(
            "only legacy CFB and SPLMSSTR .prt files are supported".to_string(),
        ));
    }
    if input == output {
        return Err(EditError::Io(std::io::Error::new(
            std::io::ErrorKind::InvalidInput,
            "input and output must be different paths",
        )));
    }

    let mut compound = cfb::open(input)?;
    let old_xml = read_stream(&mut compound, PART_ATTRS_STREAM)?;
    let (new_xml, changed) = rewrite_attribute(&old_xml, title, value)?;

    let temp = temporary_path(output);
    if !changed {
        // A no-op edit must not disturb the CFB allocator, directory entry
        // timestamps, or sector layout. Copy the source bytes instead of
        // rebuilding the container.
        if let Err(error) = fs::copy(input, &temp) {
            let _ = fs::remove_file(&temp);
            return Err(error.into());
        }
        replace_file(&temp, output)?;
        let output_bytes = fs::read(output)?;
        let output_sha256 = sha256_hex(&output_bytes);
        let byte_identical = source_sha256 == output_sha256;
        return Ok(EditReport {
            source: input.to_path_buf(),
            output: output.to_path_buf(),
            changed_streams: Vec::new(),
            source_sha256,
            output_sha256,
            byte_identical,
        });
    }

    if let Err(error) = copy_cfb_with_replacement(&mut compound, &temp, PART_ATTRS_STREAM, &new_xml)
    {
        let _ = fs::remove_file(&temp);
        return Err(error);
    }
    replace_file(&temp, output)?;

    let output_bytes = fs::read(output)?;
    let output_sha256 = sha256_hex(&output_bytes);
    let byte_identical = source_sha256 == output_sha256;
    let mut changed_streams = Vec::new();
    if changed {
        changed_streams.push(PART_ATTRS_STREAM.to_string());
    }

    // Verify the stream is still parseable before returning. This catches
    // truncated writes and malformed replacements early.
    let mut written = cfb::open(output)?;
    let written_xml = read_stream(&mut written, PART_ATTRS_STREAM)?;
    let _ = parse_attributes(&written_xml)?;

    Ok(EditReport {
        source: input.to_path_buf(),
        output: output.to_path_buf(),
        changed_streams,
        source_sha256,
        output_sha256,
        byte_identical,
    })
}

/// Replace one CFB stream with exact bytes, copying every other stream and
/// storage verbatim.
///
/// This is the byte-preserving primitive behind the higher-level edits. It
/// refuses SPLMSSTR containers, which have their own repack path, and verifies
/// the replacement reads back exactly before reporting success.
pub fn replace_cfb_stream(
    input: &Path,
    output: &Path,
    stream: &str,
    payload: &[u8],
) -> Result<EditReport, EditError> {
    let source_bytes = fs::read(input)?;
    let source_sha256 = sha256_hex(&source_bytes);
    if source_bytes.len() < 8 || source_bytes[..8] != CFB_MAGIC {
        return Err(EditError::UnsupportedContainer(
            "stream replacement is implemented for legacy CFB parts".to_string(),
        ));
    }
    if input == output {
        return Err(EditError::Io(std::io::Error::new(
            std::io::ErrorKind::InvalidInput,
            "input and output must be different paths",
        )));
    }

    let mut compound = cfb::open(input)?;
    let _existing = read_stream(&mut compound, stream)?;

    let temp = temporary_path(output);
    if let Err(error) = copy_cfb_with_replacement(&mut compound, &temp, stream, payload) {
        let _ = fs::remove_file(&temp);
        return Err(error);
    }
    replace_file(&temp, output)?;

    let mut written = cfb::open(output)?;
    let written_payload = read_stream(&mut written, stream)?;
    if written_payload != payload {
        return Err(EditError::MalformedContainer(
            "replaced stream did not read back byte-exactly".to_string(),
        ));
    }

    let output_bytes = fs::read(output)?;
    let output_sha256 = sha256_hex(&output_bytes);
    Ok(EditReport {
        source: input.to_path_buf(),
        output: output.to_path_buf(),
        changed_streams: vec![stream.to_string()],
        source_sha256,
        output_sha256,
        byte_identical: false,
    })
}

fn set_splmsstr_attribute(
    input: &Path,
    output: &Path,
    source_bytes: &[u8],
    title: &str,
    value: &str,
    source_sha256: String,
) -> Result<EditReport, EditError> {
    let (offset, length) = {
        let container = crate::container::scan_bytes(source_bytes)
            .map_err(|error| EditError::MalformedContainer(error.to_string()))?;
        let entry = container
            .entries
            .iter()
            .find(|entry| entry.name == "/Root/part/attrs" || entry.name == "part/attrs")
            .ok_or_else(|| EditError::MissingStream(PART_ATTRS_STREAM.to_string()))?;
        let (offset, length) = entry
            .file_span()
            .ok_or_else(|| EditError::MissingStream(PART_ATTRS_STREAM.to_string()))?;
        (
            usize::try_from(offset)
                .map_err(|_| EditError::MalformedContainer("offset overflow".into()))?,
            usize::try_from(length)
                .map_err(|_| EditError::MalformedContainer("length overflow".into()))?,
        )
    };
    let end = offset
        .checked_add(length)
        .filter(|end| *end <= source_bytes.len())
        .ok_or_else(|| EditError::MalformedContainer("part/attrs span exceeds file".into()))?;
    let old_xml = &source_bytes[offset..end];
    let (new_xml, changed) = rewrite_attribute(old_xml, title, value)?;

    if !changed {
        let temp = temporary_path(output);
        if let Err(error) = fs::copy(input, &temp) {
            let _ = fs::remove_file(&temp);
            return Err(error.into());
        }
        replace_file(&temp, output)?;
        let output_bytes = fs::read(output)?;
        let output_sha256 = sha256_hex(&output_bytes);
        return Ok(EditReport {
            source: input.to_path_buf(),
            output: output.to_path_buf(),
            changed_streams: Vec::new(),
            source_sha256,
            output_sha256,
            byte_identical: true,
        });
    }

    let output_bytes = if new_xml.len() <= length {
        let mut output_bytes = source_bytes.to_vec();
        output_bytes[offset..offset + new_xml.len()].copy_from_slice(&new_xml);
        let padding = if old_xml.last().copied() == Some(0) {
            0
        } else {
            b' '
        };
        output_bytes[offset + new_xml.len()..end].fill(padding);
        output_bytes
    } else {
        repack_splmsstr_growth(source_bytes, offset, length, &new_xml)?
    };

    let temp = temporary_path(output);
    if let Err(error) = fs::write(&temp, &output_bytes) {
        let _ = fs::remove_file(&temp);
        return Err(error.into());
    }
    replace_file(&temp, output)?;

    // Re-scan and read the edited stream before returning.
    let written = fs::read(output)?;
    {
        let container = crate::container::scan_bytes(written.as_slice())
            .map_err(|error| EditError::MalformedContainer(error.to_string()))?;
        let entry = container
            .entries
            .iter()
            .find(|entry| entry.name == "/Root/part/attrs" || entry.name == "part/attrs")
            .ok_or_else(|| EditError::MissingStream(PART_ATTRS_STREAM.to_string()))?;
        let (offset, length) = entry
            .file_span()
            .ok_or_else(|| EditError::MissingStream(PART_ATTRS_STREAM.to_string()))?;
        let offset = usize::try_from(offset)
            .map_err(|_| EditError::MalformedContainer("offset overflow".into()))?;
        let length = usize::try_from(length)
            .map_err(|_| EditError::MalformedContainer("length overflow".into()))?;
        let end = offset
            .checked_add(length)
            .filter(|end| *end <= written.len())
            .ok_or_else(|| EditError::MalformedContainer("part/attrs span exceeds file".into()))?;
        parse_attributes(&written[offset..end])?;
    }

    Ok(EditReport {
        source: input.to_path_buf(),
        output: output.to_path_buf(),
        changed_streams: vec![PART_ATTRS_STREAM.to_string()],
        source_sha256,
        output_sha256: sha256_hex(&written),
        byte_identical: false,
    })
}

#[derive(Debug, Clone, Copy)]
struct SplmsstrField {
    offset_position: usize,
    offset: u64,
    size: u64,
}

/// Grow a file payload and shift only the bytes after it. Directory entries in
/// both the HEADER and FOOTER regions are updated in place; unrelated payload
/// bytes and gaps are preserved exactly.
fn repack_splmsstr_growth(
    source: &[u8],
    offset: usize,
    old_length: usize,
    new_payload: &[u8],
) -> Result<Vec<u8>, EditError> {
    if new_payload.len() <= old_length {
        return Err(EditError::MalformedContainer(
            "growth repack received a shorter payload".into(),
        ));
    }
    repack_splmsstr_payload(source, offset, old_length, new_payload)
}

/// Replace one file payload in a SPLMSSTR container and shift every later
/// payload. Header and footer directory rows keep their absolute payload
/// offsets, so both regions are rewritten when the replacement changes the
/// file length. The fixed header and footer XXH32 checksums are recomputed.
pub(crate) fn repack_splmsstr_payload(
    source: &[u8],
    offset: usize,
    old_length: usize,
    new_payload: &[u8],
) -> Result<Vec<u8>, EditError> {
    let insertion = offset
        .checked_add(old_length)
        .filter(|end| *end <= source.len())
        .ok_or_else(|| EditError::MalformedContainer("payload span exceeds file".into()))?;
    let insertion_u64 = u64::try_from(insertion)
        .map_err(|_| EditError::MalformedContainer("payload offset exceeds u64".into()))?;
    let offset_u64 = u64::try_from(offset)
        .map_err(|_| EditError::MalformedContainer("payload offset exceeds u64".into()))?;
    let old_length_u64 = u64::try_from(old_length)
        .map_err(|_| EditError::MalformedContainer("payload length exceeds u64".into()))?;
    let new_length_u64 = u64::try_from(new_payload.len())
        .map_err(|_| EditError::MalformedContainer("payload length exceeds u64".into()))?;
    let delta = i64::try_from(new_payload.len())
        .ok()
        .and_then(|new| {
            i64::try_from(old_length)
                .ok()
                .and_then(|old| new.checked_sub(old))
        })
        .ok_or_else(|| EditError::MalformedContainer("payload size overflow".into()))?;

    let footer_offset = read_u48(source, 0x11)
        .ok_or_else(|| EditError::MalformedContainer("missing SPLMSSTR footer offset".into()))?;
    let has_footer = footer_offset != 0x0000_ffff_ffff_ffff;
    let header_count = read_u32(source, 0x1f)
        .ok_or_else(|| EditError::MalformedContainer("missing HEADER entry count".into()))?;
    let mut fields = Vec::new();
    let header_end = parse_splmsstr_fields(source, 0x23, header_count, &mut fields)?;

    let footer = if has_footer {
        let footer = usize::try_from(footer_offset)
            .map_err(|_| EditError::MalformedContainer("footer offset overflow".into()))?;
        if source.get(footer..footer + 6) != Some(b"FOOTER") {
            return Err(EditError::MalformedContainer(
                "missing FOOTER marker at footer offset".into(),
            ));
        }
        let count = read_u32(source, footer + 6)
            .ok_or_else(|| EditError::MalformedContainer("missing FOOTER entry count".into()))?;
        parse_splmsstr_fields(source, footer + 10, count, &mut fields)?;
        Some(footer)
    } else {
        None
    };

    let output_len = i64::try_from(source.len())
        .ok()
        .and_then(|len| len.checked_add(delta))
        .and_then(|len| usize::try_from(len).ok())
        .ok_or_else(|| EditError::MalformedContainer("output size overflow".into()))?;
    let mut output = Vec::with_capacity(output_len);
    output.extend_from_slice(&source[..offset]);
    output.extend_from_slice(new_payload);
    output.extend_from_slice(&source[insertion..]);

    for field in fields {
        let write_base = if footer.is_some_and(|footer| field.offset_position >= footer) {
            delta
        } else {
            0
        };
        let write_position = i64::try_from(field.offset_position)
            .ok()
            .and_then(|position| position.checked_add(write_base))
            .and_then(|position| usize::try_from(position).ok())
            .ok_or_else(|| EditError::MalformedContainer("directory offset overflow".into()))?;
        let new_offset = if field.offset >= insertion_u64 {
            i64::try_from(field.offset)
                .ok()
                .and_then(|value| value.checked_add(delta))
                .and_then(|value| u64::try_from(value).ok())
                .ok_or_else(|| EditError::MalformedContainer("offset overflow".into()))?
        } else {
            field.offset
        };
        let new_size = if field.offset == offset_u64 && field.size == old_length_u64 {
            new_length_u64
        } else {
            field.size
        };
        write_u64(&mut output, write_position, new_offset)?;
        write_u64(&mut output, write_position + 8, new_size)?;
    }

    if footer.is_some() {
        let new_footer_offset = i64::try_from(footer_offset)
            .ok()
            .and_then(|value| value.checked_add(delta))
            .and_then(|value| u64::try_from(value).ok())
            .ok_or_else(|| EditError::MalformedContainer("footer offset overflow".into()))?;
        write_u48(&mut output, 0x11, new_footer_offset)?;
        if source.len() >= 4 && read_u32(source, source.len() - 4).is_some_and(|value| value != 0) {
            let footer_start = usize::try_from(new_footer_offset)
                .map_err(|_| EditError::MalformedContainer("footer offset overflow".into()))?;
            let checksum_end = output.len() - 4;
            if footer_start > checksum_end {
                return Err(EditError::MalformedContainer(
                    "footer checksum range exceeds file".into(),
                ));
            }
            let checksum = xxh32(&output[footer_start..checksum_end], 0);
            write_u32(&mut output, checksum_end, checksum)?;
        }
    }

    // The fixed HEADER/index extension places an XXH32 checksum at
    // `header_end + 0x21e`. It is present only when the source field is
    // nonzero.
    let checksum_offset = header_end
        .checked_add(0x21E)
        .ok_or_else(|| EditError::MalformedContainer("checksum offset overflow".into()))?;
    if read_u32(source, checksum_offset).is_some_and(|value| value != 0) {
        checksum_offset
            .checked_add(4)
            .filter(|end| *end <= output.len())
            .ok_or_else(|| EditError::MalformedContainer("checksum field exceeds file".into()))?;
        let checksum = xxh32(&output[0x19..checksum_offset], 0);
        write_u32(&mut output, checksum_offset, checksum)?;
    }

    Ok(output)
}

fn parse_splmsstr_fields(
    bytes: &[u8],
    mut position: usize,
    count: u32,
    fields: &mut Vec<SplmsstrField>,
) -> Result<usize, EditError> {
    for _ in 0..count {
        let name_length = read_u32(bytes, position)
            .ok_or_else(|| EditError::MalformedContainer("truncated directory entry".into()))?
            as usize;
        if !(6..=128).contains(&name_length) {
            return Err(EditError::MalformedContainer(
                "directory name length is out of range".into(),
            ));
        }
        position = position
            .checked_add(4)
            .and_then(|position| position.checked_add(name_length))
            .ok_or_else(|| EditError::MalformedContainer("directory entry overflow".into()))?;
        let offset_position = position;
        let offset = read_u64(bytes, position)
            .ok_or_else(|| EditError::MalformedContainer("missing payload offset".into()))?;
        position += 8;
        let size = read_u64(bytes, position)
            .ok_or_else(|| EditError::MalformedContainer("missing payload size".into()))?;
        position += 8;
        if size != 0 {
            fields.push(SplmsstrField {
                offset_position,
                offset,
                size,
            });
        }
    }
    Ok(position)
}

fn read_u32(bytes: &[u8], offset: usize) -> Option<u32> {
    let value = bytes.get(offset..offset.checked_add(4)?)?;
    Some(u32::from_le_bytes(value.try_into().ok()?))
}

fn read_u64(bytes: &[u8], offset: usize) -> Option<u64> {
    let value = bytes.get(offset..offset.checked_add(8)?)?;
    Some(u64::from_le_bytes(value.try_into().ok()?))
}

fn read_u48(bytes: &[u8], offset: usize) -> Option<u64> {
    let value = bytes.get(offset..offset.checked_add(6)?)?;
    let mut result = 0_u64;
    for (index, byte) in value.iter().enumerate() {
        result |= u64::from(*byte) << (8 * index);
    }
    Some(result)
}

fn write_u32(bytes: &mut [u8], offset: usize, value: u32) -> Result<(), EditError> {
    let target = bytes
        .get_mut(
            offset
                ..offset.checked_add(4).ok_or_else(|| {
                    EditError::MalformedContainer("checksum field overflow".into())
                })?,
        )
        .ok_or_else(|| EditError::MalformedContainer("checksum field out of bounds".into()))?;
    target.copy_from_slice(&value.to_le_bytes());
    Ok(())
}

fn xxh32(data: &[u8], seed: u32) -> u32 {
    const P1: u32 = 2_654_435_761;
    const P2: u32 = 2_246_822_519;
    const P3: u32 = 3_266_489_917;
    const P4: u32 = 668_265_263;
    const P5: u32 = 374_761_393;

    let length = data.len();
    let mut hash;
    if length >= 16 {
        let mut values = [
            seed.wrapping_add(P1).wrapping_add(P2),
            seed.wrapping_add(P2),
            seed,
            seed.wrapping_sub(P1),
        ];
        for chunk in data[..length & !15].chunks_exact(16) {
            for (index, value) in values.iter_mut().enumerate() {
                let lane = read_u32(chunk, index * 4).unwrap_or(0);
                *value = value
                    .wrapping_add(lane.wrapping_mul(P2))
                    .rotate_left(13)
                    .wrapping_mul(P1);
            }
        }
        hash = values[0]
            .rotate_left(1)
            .wrapping_add(values[1].rotate_left(7))
            .wrapping_add(values[2].rotate_left(12))
            .wrapping_add(values[3].rotate_left(18));
    } else {
        hash = seed.wrapping_add(P5);
    }
    hash = hash.wrapping_add(length as u32);

    let mut index = length & !15;
    while index + 4 <= length {
        let lane = read_u32(data, index).unwrap_or(0);
        hash = hash
            .wrapping_add(lane.wrapping_mul(P3))
            .rotate_left(17)
            .wrapping_mul(P4);
        index += 4;
    }
    while index < length {
        hash = hash
            .wrapping_add(u32::from(data[index]).wrapping_mul(P5))
            .rotate_left(11)
            .wrapping_mul(P1);
        index += 1;
    }
    hash ^= hash >> 15;
    hash = hash.wrapping_mul(P2);
    hash ^= hash >> 13;
    hash = hash.wrapping_mul(P3);
    hash ^= hash >> 16;
    hash
}

fn write_u64(bytes: &mut [u8], offset: usize, value: u64) -> Result<(), EditError> {
    let target = bytes
        .get_mut(
            offset
                ..offset.checked_add(8).ok_or_else(|| {
                    EditError::MalformedContainer("directory field overflow".into())
                })?,
        )
        .ok_or_else(|| EditError::MalformedContainer("directory field out of bounds".into()))?;
    target.copy_from_slice(&value.to_le_bytes());
    Ok(())
}

fn write_u48(bytes: &mut [u8], offset: usize, value: u64) -> Result<(), EditError> {
    let target = bytes
        .get_mut(
            offset
                ..offset
                    .checked_add(6)
                    .ok_or_else(|| EditError::MalformedContainer("footer field overflow".into()))?,
        )
        .ok_or_else(|| EditError::MalformedContainer("footer field out of bounds".into()))?;
    for (index, byte) in target.iter_mut().enumerate() {
        *byte = (value >> (8 * index)) as u8;
    }
    Ok(())
}

fn read_stream(compound: &mut cfb::CompoundFile<File>, name: &str) -> Result<Vec<u8>, EditError> {
    let mut stream = compound
        .open_stream(name)
        .map_err(|_| EditError::MissingStream(name.to_string()))?;
    let mut data = Vec::new();
    stream.read_to_end(&mut data)?;
    Ok(data)
}

struct AttributeTag {
    start: usize,
    end: usize,
    attributes: Vec<(String, String)>,
}

impl AttributeTag {
    fn get(&self, name: &str) -> Option<&str> {
        self.attributes
            .iter()
            .find(|(key, _)| key == name)
            .map(|(_, value)| value.as_str())
    }

    fn title(&self) -> Option<&str> {
        self.get("title").or_else(|| self.get("utf8title"))
    }

    fn value(&self) -> Option<&str> {
        self.get("value").or_else(|| self.get("utf8value"))
    }
}

/// NX writes `xsi:type` in `part/attrs` without declaring the `xsi` prefix in
/// some releases, so a namespace-aware XML parser rejects otherwise valid NX
/// files. This scanner handles the small, regular subset NX emits.
fn parse_attributes(xml: &[u8]) -> Result<Vec<PartAttribute>, EditError> {
    let text = std::str::from_utf8(xml)
        .map_err(|error| EditError::InvalidAttributeXml(error.to_string()))?;
    let text = text
        .trim_end_matches(|character: char| character == '\0' || character.is_ascii_whitespace());
    if !text.contains("<UgAttributes") {
        return Err(EditError::InvalidAttributeXml(
            "missing UgAttributes root".to_string(),
        ));
    }
    Ok(scan_tags(text)
        .into_iter()
        .filter_map(|tag| {
            let title = tag.title()?.to_string();
            let value = tag.value().unwrap_or_default().to_string();
            Some(PartAttribute { title, value })
        })
        .collect())
}

fn rewrite_attribute(xml: &[u8], title: &str, value: &str) -> Result<(Vec<u8>, bool), EditError> {
    let had_nul = xml.last().copied() == Some(0);
    let text = std::str::from_utf8(xml)
        .map_err(|error| EditError::InvalidAttributeXml(error.to_string()))?;
    let text = text
        .trim_end_matches(|character: char| character == '\0' || character.is_ascii_whitespace());
    if !text.contains("<UgAttributes") {
        return Err(EditError::InvalidAttributeXml(
            "missing UgAttributes root".to_string(),
        ));
    }

    let tags = scan_tags(text);
    let existing = tags
        .iter()
        .find(|tag| tag.title().is_some_and(|candidate| candidate == title));
    if existing.is_some_and(|tag| tag.value().is_some_and(|candidate| candidate == value)) {
        return Ok((xml.to_vec(), false));
    }

    let mut output = String::with_capacity(text.len() + 128);
    if let Some(tag) = existing {
        let mut attributes = tag.attributes.clone();
        let mut has_value = false;
        for (name, attribute_value) in &mut attributes {
            match name.as_str() {
                "title" | "utf8title" => *attribute_value = title.to_string(),
                "value" | "utf8value" => {
                    *attribute_value = value.to_string();
                    has_value = true;
                }
                _ => {}
            }
        }
        if !has_value {
            attributes.push(("value".to_string(), value.to_string()));
        }
        output.push_str(&text[..tag.start]);
        output.push_str(&render_tag("Attribute", &attributes));
        output.push_str(&text[tag.end..]);
    } else {
        let attributes = new_attribute_attributes(text, title, value);
        if let Some(close) = text.rfind("</UgAttributes>") {
            output.push_str(&text[..close]);
            if !output.ends_with('\n') {
                output.push('\n');
            }
            output.push_str("  ");
            output.push_str(&render_tag("Attribute", &attributes));
            output.push('\n');
            output.push_str(&text[close..]);
        } else {
            let Some(start) = text.find("<UgAttributes") else {
                return Err(EditError::InvalidAttributeXml(
                    "missing UgAttributes root".to_string(),
                ));
            };
            let Some(end) = find_tag_end(text, start) else {
                return Err(EditError::InvalidAttributeXml(
                    "unterminated UgAttributes root".to_string(),
                ));
            };
            let root = &text[start..end];
            let Some(root_open) = root.strip_suffix("/>") else {
                return Err(EditError::InvalidAttributeXml(
                    "UgAttributes root is neither open nor self-closing".to_string(),
                ));
            };
            output.push_str(&text[..start]);
            output.push_str(root_open);
            output.push_str(">\n  ");
            output.push_str(&render_tag("Attribute", &attributes));
            output.push_str("\n</UgAttributes>");
            output.push_str(&text[end..]);
        }
    }

    if had_nul {
        output.push('\0');
    }
    Ok((output.into_bytes(), true))
}

fn new_attribute_attributes(text: &str, title: &str, value: &str) -> Vec<(String, String)> {
    let uses_utf8 = text.contains("utf8title=") || text.contains("version=\"4\"");
    let mut attributes = vec![("owner".to_string(), "part".to_string())];
    if uses_utf8 {
        attributes.push(("pdmBased".to_string(), "false".to_string()));
    }
    attributes.push(("title".to_string(), title.to_string()));
    attributes.push(("value".to_string(), value.to_string()));
    if uses_utf8 {
        attributes.push(("version".to_string(), "3".to_string()));
        attributes.push(("utf8title".to_string(), title.to_string()));
        attributes.push(("utf8value".to_string(), value.to_string()));
    }
    attributes.push(("xsi:type".to_string(), "StringAttributeType".to_string()));
    attributes
}

fn scan_tags(text: &str) -> Vec<AttributeTag> {
    let mut tags = Vec::new();
    let mut search = 0;
    const OPEN: &str = "<Attribute";
    while let Some(relative) = text[search..].find(OPEN) {
        let start = search + relative;
        let next = text[start + OPEN.len()..].chars().next();
        if !matches!(next, Some(character) if character.is_ascii_whitespace() || character == '/' || character == '>')
        {
            search = start + OPEN.len();
            continue;
        }
        let Some(end) = find_tag_end(text, start) else {
            break;
        };
        let raw = &text[start..end];
        tags.push(AttributeTag {
            start,
            end,
            attributes: parse_tag_attributes(raw),
        });
        search = end;
    }
    tags
}

fn find_tag_end(text: &str, start: usize) -> Option<usize> {
    let bytes = text.as_bytes();
    let mut quote = None;
    let mut index = start;
    while index < bytes.len() {
        let byte = bytes[index];
        match quote {
            Some(expected) if byte == expected => quote = None,
            Some(_) => {}
            None if byte == b'"' || byte == b'\'' => quote = Some(byte),
            None if byte == b'>' => return Some(index + 1),
            None => {}
        }
        index += 1;
    }
    None
}

fn parse_tag_attributes(raw: &str) -> Vec<(String, String)> {
    let bytes = raw.as_bytes();
    let mut attributes = Vec::new();
    let mut index = "<Attribute".len();
    while index < bytes.len() {
        while index < bytes.len() && bytes[index].is_ascii_whitespace() {
            index += 1;
        }
        if index >= bytes.len()
            || bytes[index] == b'>'
            || (bytes[index] == b'/' && bytes.get(index + 1) == Some(&b'>'))
        {
            break;
        }
        let name_start = index;
        while index < bytes.len()
            && bytes[index] != b'='
            && !bytes[index].is_ascii_whitespace()
            && bytes[index] != b'>'
            && bytes[index] != b'/'
        {
            index += 1;
        }
        let name = raw[name_start..index].trim().to_string();
        while index < bytes.len() && bytes[index].is_ascii_whitespace() {
            index += 1;
        }
        if index >= bytes.len() || bytes[index] != b'=' {
            break;
        }
        index += 1;
        while index < bytes.len() && bytes[index].is_ascii_whitespace() {
            index += 1;
        }
        if index >= bytes.len() || (bytes[index] != b'"' && bytes[index] != b'\'') {
            break;
        }
        let quote = bytes[index];
        index += 1;
        let value_start = index;
        while index < bytes.len() && bytes[index] != quote {
            index += 1;
        }
        if index >= bytes.len() {
            break;
        }
        let value = decode_entities(&raw[value_start..index]);
        if !name.is_empty() {
            attributes.push((name, value));
        }
        index += 1;
    }
    attributes
}

fn render_tag(name: &str, attributes: &[(String, String)]) -> String {
    let mut output = String::new();
    output.push('<');
    output.push_str(name);
    for (key, value) in attributes {
        output.push(' ');
        output.push_str(key);
        output.push_str("=\"");
        output.push_str(&escape(value));
        output.push('"');
    }
    output.push_str("/>");
    output
}

fn decode_entities(value: &str) -> String {
    let mut decoded = String::with_capacity(value.len());
    let mut rest = value;
    while let Some(position) = rest.find('&') {
        decoded.push_str(&rest[..position]);
        let Some(end) = rest[position..].find(';') else {
            decoded.push_str(&rest[position..]);
            return decoded;
        };
        let entity = &rest[position + 1..position + end];
        let replacement = match entity {
            "amp" => Some('&'),
            "lt" => Some('<'),
            "gt" => Some('>'),
            "quot" => Some('"'),
            "apos" => Some('\''),
            _ if entity.starts_with("#x") => u32::from_str_radix(&entity[2..], 16)
                .ok()
                .and_then(char::from_u32),
            _ if entity.starts_with('#') => {
                entity[1..].parse::<u32>().ok().and_then(char::from_u32)
            }
            _ => None,
        };
        match replacement {
            Some(character) => decoded.push(character),
            None => decoded.push_str(&rest[position..=position + end]),
        }
        rest = &rest[position + end + 1..];
    }
    decoded.push_str(rest);
    decoded
}

fn escape(value: &str) -> String {
    let mut escaped = String::with_capacity(value.len());
    for character in value.chars() {
        match character {
            '&' => escaped.push_str("&amp;"),
            '<' => escaped.push_str("&lt;"),
            '>' => escaped.push_str("&gt;"),
            '"' => escaped.push_str("&quot;"),
            '\'' => escaped.push_str("&apos;"),
            other => escaped.push(other),
        }
    }
    escaped
}

pub(crate) fn copy_cfb_with_replacement(
    source: &mut cfb::CompoundFile<File>,
    output: &Path,
    replaced_stream: &str,
    replacement: &[u8],
) -> Result<(), EditError> {
    copy_cfb_with_replacements(
        source,
        output,
        &HashMap::from([(replaced_stream.to_string(), replacement.to_vec())]),
    )
}

/// Copy every CFB entry, replacing the named streams with new payloads.
///
/// Names are compared without the leading slash that `cfb` crate paths use.
pub(crate) fn copy_cfb_with_replacements<R: Read + Seek>(
    source: &mut cfb::CompoundFile<R>,
    output: &Path,
    replacements: &HashMap<String, Vec<u8>>,
) -> Result<(), EditError> {
    let entries: Vec<_> = source
        .walk()
        .map(|entry| {
            (
                entry.path().to_path_buf(),
                entry.is_stream(),
                entry.state_bits(),
                *entry.clsid(),
                entry.created(),
                entry.modified(),
            )
        })
        .collect();
    let file = File::create(output)?;
    let mut destination = cfb::CompoundFile::create(file)?;
    for (path, is_stream, state_bits, clsid, created, modified) in entries {
        let name = path.to_string_lossy().replace('\\', "/");
        if name == "/" || name.is_empty() {
            continue;
        }
        if is_stream {
            let key = name.trim_start_matches('/');
            let data = if let Some(replacement) = replacements.get(key) {
                replacement.clone()
            } else {
                let mut stream = source
                    .open_stream(&path)
                    .map_err(|_| EditError::MissingStream(name.clone()))?;
                let mut data = Vec::new();
                stream.read_to_end(&mut data)?;
                data
            };
            let mut stream = destination.create_stream(&path)?;
            stream.write_all(&data)?;
        } else {
            destination.create_storage(&path)?;
            destination.set_storage_clsid(&path, clsid)?;
            destination.set_created_time(&path, created)?;
            destination.set_modified_time(&path, modified)?;
        }
        destination.set_state_bits(&path, state_bits)?;
    }
    destination.flush()?;
    Ok(())
}

pub(crate) fn temporary_path(output: &Path) -> PathBuf {
    let mut name = output
        .file_name()
        .map(|name| name.to_os_string())
        .unwrap_or_else(|| "nxedit".into());
    name.push(format!(".{}.tmp", std::process::id()));
    output.with_file_name(name)
}

pub(crate) fn replace_file(temp: &Path, output: &Path) -> Result<(), EditError> {
    if output.exists() {
        fs::remove_file(output)?;
    }
    fs::rename(temp, output)?;
    Ok(())
}

pub(crate) fn sha256_hex(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    let mut text = String::with_capacity(64);
    for byte in digest {
        use std::fmt::Write as _;
        let _ = write!(text, "{byte:02x}");
    }
    text
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn escapes_xml_attribute_values() {
        assert_eq!(escape("a&b<c>\"d'"), "a&amp;b&lt;c&gt;&quot;d&apos;");
    }

    #[test]
    fn xxh32_matches_reference_vectors() {
        assert_eq!(xxh32(b"", 0), 0x02CC_5D05);
        assert_eq!(xxh32(b"a", 0), 0x550D_7456);
        assert_eq!(xxh32(b"hello", 0), 0xFB00_77F9);
    }

    #[test]
    fn repacks_splmsstr_growth_and_shifts_later_payloads() {
        let mut data = vec![0_u8; 0x460];
        data[..8].copy_from_slice(b"SPLMSSTR");
        data[8] = 0x06;
        data[0x11..0x17].copy_from_slice(&[0xFF; 6]);
        data[0x19..0x1F].copy_from_slice(b"HEADER");
        data[0x1F..0x23].copy_from_slice(&2_u32.to_le_bytes());

        let write_entry =
            |data: &mut Vec<u8>, position: usize, name: &str, offset: u64, size: u64| {
                let mut cursor = position;
                data[cursor..cursor + 4].copy_from_slice(&(name.len() as u32).to_le_bytes());
                cursor += 4;
                data[cursor..cursor + name.len()].copy_from_slice(name.as_bytes());
                cursor += name.len();
                data[cursor..cursor + 8].copy_from_slice(&offset.to_le_bytes());
                cursor += 8;
                data[cursor..cursor + 8].copy_from_slice(&size.to_le_bytes());
                cursor += 8;
                cursor
            };

        let after_first = write_entry(&mut data, 0x23, "/Root/part/attrs", 0x460, 10);
        write_entry(&mut data, after_first, "/Root/other", 0x46A, 5);
        data.extend_from_slice(b"0123456789");
        data.extend_from_slice(b"abcde");

        let output = repack_splmsstr_growth(&data, 0x460, 10, b"0123456789ABCDEF").unwrap();
        assert_eq!(output.len(), data.len() + 6);
        assert_eq!(&output[0x460..0x470], b"0123456789ABCDEF");
        assert_eq!(&output[0x470..0x475], b"abcde");
        let mut fields = Vec::new();
        parse_splmsstr_fields(&output, 0x23, 2, &mut fields).unwrap();
        assert!(fields
            .iter()
            .any(|field| field.offset == 0x470 && field.size == 5));
    }

    #[test]
    fn decodes_xml_entities() {
        assert_eq!(
            decode_entities("a&amp;b&lt;c&gt;&quot;d&apos;e&#65;&#x42;"),
            "a&b<c>\"d'eAB"
        );
    }

    #[test]
    fn accepts_trailing_padding_after_the_root() {
        let xml = b"<UgAttributes><Attribute title=\"A\" value=\"B\"/></UgAttributes>   \0";
        let (output, changed) = rewrite_attribute(xml, "A", "C").unwrap();
        assert!(changed);
        assert_eq!(
            parse_attributes(&output).unwrap(),
            vec![PartAttribute {
                title: "A".to_string(),
                value: "C".to_string()
            }]
        );
    }

    #[test]
    fn rewrites_existing_attribute() {
        let xml = br#"<?xml version="1.0" encoding="UTF-8"?>
<UgAttributes version="2" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"><Attribute owner="part" title="Material" value="Steel" version="2" xsi:type="StringAttributeType" utf8title="Material" utf8value="Steel"/></UgAttributes>"#;
        let (output, changed) = rewrite_attribute(xml, "Material", "Aluminum").unwrap();
        assert!(changed);
        let values = parse_attributes(&output).unwrap();
        assert_eq!(
            values,
            vec![PartAttribute {
                title: "Material".to_string(),
                value: "Aluminum".to_string()
            }]
        );
    }

    #[test]
    fn appends_missing_attribute() {
        let xml = br#"<?xml version="1.0" encoding="UTF-8"?>
<UgAttributes version="2" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"></UgAttributes>"#;
        let (output, changed) = rewrite_attribute(xml, "PartNumber", "NX-001").unwrap();
        assert!(changed);
        assert!(parse_attributes(&output)
            .unwrap()
            .iter()
            .any(|attribute| attribute.title == "PartNumber" && attribute.value == "NX-001"));
    }

    #[test]
    fn unchanged_value_keeps_source_bytes() {
        let xml = br#"<UgAttributes version="2"><Attribute title="A" value="B"/></UgAttributes>"#;
        let (output, changed) = rewrite_attribute(xml, "A", "B").unwrap();
        assert!(!changed);
        assert_eq!(output, xml);
    }

    #[test]
    fn edits_a_legacy_cfb_part_and_preserves_unknown_streams() {
        let unique = format!(
            "{}-{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        );
        let directory = std::env::temp_dir();
        let input = directory.join(format!("nxedit-test-{unique}.prt"));
        let edited = directory.join(format!("nxedit-test-{unique}-edited.prt"));
        let same = directory.join(format!("nxedit-test-{unique}-same.prt"));

        let xml = br#"<?xml version="1.0" encoding="UTF-8"?>
<UgAttributes><Attribute owner="part" title="DB_PART_DESC" value="old" xsi:type="StringAttributeType"/></UgAttributes>"#;
        {
            let file = File::create(&input).unwrap();
            let mut compound = cfb::CompoundFile::create(file).unwrap();
            compound.create_storage("part").unwrap();
            compound
                .create_stream("part/attrs")
                .unwrap()
                .write_all(xml)
                .unwrap();
            compound.create_storage("UG_PART").unwrap();
            compound
                .create_stream("UG_PART/UG_PART")
                .unwrap()
                .write_all(b"\r\x01UGII  payload")
                .unwrap();
            compound.flush().unwrap();
        }

        let report = set_part_attribute(&input, &edited, "DB_PART_DESC", "new").unwrap();
        assert_eq!(report.changed_streams, vec![PART_ATTRS_STREAM.to_string()]);
        assert!(!report.byte_identical);
        let attributes = read_part_attributes(&edited).unwrap();
        assert_eq!(
            attributes,
            vec![PartAttribute {
                title: "DB_PART_DESC".to_string(),
                value: "new".to_string()
            }]
        );

        let same_report = set_part_attribute(&input, &same, "DB_PART_DESC", "old").unwrap();
        assert!(same_report.changed_streams.is_empty());
        assert!(same_report.byte_identical);

        let _ = fs::remove_file(input);
        let _ = fs::remove_file(edited);
        let _ = fs::remove_file(same);
    }
}

#[test]
fn replaces_one_cfb_stream_and_preserves_other_streams() {
    let unique = format!(
        "{}-{}",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos()
    );
    let directory = std::env::temp_dir();
    let input = directory.join(format!("nxstream-test-{unique}.prt"));
    let edited = directory.join(format!("nxstream-test-{unique}-edited.prt"));
    let xml = br#"<?xml version="1.0" encoding="UTF-8"?>
<UgAttributes><Attribute owner="part" title="DB_PART_DESC" value="old" xsi:type="StringAttributeType"/></UgAttributes>"#;
    let original = b"\r\x01UGII  original payload";
    let replacement = b"\r\x01UGII  replacement payload";
    {
        let file = File::create(&input).unwrap();
        let mut compound = cfb::CompoundFile::create(file).unwrap();
        compound.create_storage("part").unwrap();
        compound
            .create_stream("part/attrs")
            .unwrap()
            .write_all(xml)
            .unwrap();
        compound.create_storage("UG_PART").unwrap();
        compound
            .create_stream("UG_PART/UG_PART")
            .unwrap()
            .write_all(original)
            .unwrap();
        compound.flush().unwrap();
    }

    let report = replace_cfb_stream(&input, &edited, "UG_PART/UG_PART", replacement).unwrap();
    assert_eq!(report.changed_streams, vec!["UG_PART/UG_PART".to_string()]);
    assert!(!report.byte_identical);

    let mut compound = cfb::open(&edited).unwrap();
    let mut stream = compound.open_stream("UG_PART/UG_PART").unwrap();
    let mut written = Vec::new();
    stream.read_to_end(&mut written).unwrap();
    assert_eq!(written, replacement);
    let mut attrs = compound.open_stream("part/attrs").unwrap();
    let mut written_xml = Vec::new();
    attrs.read_to_end(&mut written_xml).unwrap();
    assert_eq!(written_xml, xml);

    let missing = replace_cfb_stream(&input, &edited, "UG_PART/MISSING", replacement);
    assert!(matches!(missing, Err(EditError::MissingStream(_))));

    let _ = fs::remove_file(input);
    let _ = fs::remove_file(edited);
}
