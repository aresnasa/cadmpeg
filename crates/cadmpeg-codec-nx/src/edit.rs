// SPDX-License-Identifier: Apache-2.0
//! Byte-preserving edits for NX `.prt` files.
//!
//! The first supported edit is the legacy CFB `part/attrs` XML stream. The
//! writer copies every other stream and storage verbatim, so unknown data does
//! not pass through the NX object model during a metadata edit.

use std::fs::{self, File};
use std::io::{Read, Write};
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

    if new_xml.len() > length {
        return Err(EditError::UnsupportedContainer(format!(
            "SPLMSSTR in-place edit cannot grow part/attrs from {length} to {} bytes; \
             a container repack is required",
            new_xml.len()
        )));
    }

    let mut output_bytes = source_bytes.to_vec();
    output_bytes[offset..offset + new_xml.len()].copy_from_slice(&new_xml);
    let padding = if old_xml.last().copied() == Some(0) {
        0
    } else {
        b' '
    };
    output_bytes[offset + new_xml.len()..end].fill(padding);

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
    let text = text.trim_end_matches('\0');
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
    let text = text.trim_end_matches('\0');
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
    let uses_utf8 = text.contains("utf8title=") || text.contains("version=\"2\"");
    let mut attributes = vec![
        ("owner".to_string(), "part".to_string()),
        ("title".to_string(), title.to_string()),
        ("value".to_string(), value.to_string()),
    ];
    if uses_utf8 {
        attributes.push(("version".to_string(), "2".to_string()));
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

fn copy_cfb_with_replacement(
    source: &mut cfb::CompoundFile<File>,
    output: &Path,
    replaced_stream: &str,
    replacement: &[u8],
) -> Result<(), EditError> {
    let entries: Vec<(PathBuf, bool, u32)> = source
        .walk()
        .map(|entry| {
            (
                entry.path().to_path_buf(),
                entry.is_stream(),
                entry.state_bits(),
            )
        })
        .collect();
    let file = File::create(output)?;
    let mut destination = cfb::CompoundFile::create(file)?;
    for (path, is_stream, state_bits) in entries {
        let name = path.to_string_lossy().replace('\\', "/");
        if name == "/" || name.is_empty() {
            continue;
        }
        if is_stream {
            let data = if name.trim_start_matches('/') == replaced_stream {
                replacement.to_vec()
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
        }
        destination.set_state_bits(&path, state_bits)?;
    }
    destination.flush()?;
    Ok(())
}

fn temporary_path(output: &Path) -> PathBuf {
    let mut name = output
        .file_name()
        .map(|name| name.to_os_string())
        .unwrap_or_else(|| "nxedit".into());
    name.push(format!(".{}.tmp", std::process::id()));
    output.with_file_name(name)
}

fn replace_file(temp: &Path, output: &Path) -> Result<(), EditError> {
    if output.exists() {
        fs::remove_file(output)?;
    }
    fs::rename(temp, output)?;
    Ok(())
}

fn sha256_hex(bytes: &[u8]) -> String {
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
