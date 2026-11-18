// SPDX-License-Identifier: Apache-2.0
//! External-reference inspection and relinking for NX assemblies.
//!
//! An assembly `.prt` names its child parts in the `ExternalReferences`
//! stream. The trailing string table is a self-delimiting
//! `01 count:u32 (len:u16 utf8)*` frame. Rewriting that frame preserves every
//! indexed occurrence record and every other container stream.

use std::fs;
use std::io::Read;
use std::path::{Path, PathBuf};

use crate::container;
use crate::edit::{
    copy_cfb_with_replacement, repack_splmsstr_payload, replace_file, sha256_hex, temporary_path,
    EditError,
};

const CFB_MAGIC: [u8; 8] = [0xD0, 0xCF, 0x11, 0xE0, 0xA1, 0xB1, 0x1A, 0xE1];
const EXTREF_MARKER: &str = "ExternalReferences";

/// The decoded trailing string table of one `ExternalReferences` stream.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ExternalReferenceTable {
    /// Directory entry that owns the stream.
    pub stream: String,
    /// Absolute file offset of the payload for SPLMSSTR containers; zero for
    /// legacy CFB streams whose bytes are addressed by the compound file.
    pub payload_offset: u64,
    /// Byte offset of the `01 count:u32` marker within the payload.
    pub table_offset: usize,
    /// Table strings in serialized order.
    pub strings: Vec<String>,
}

/// One external child part named by an assembly.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AssemblyComponent {
    /// File name as stored in the assembly.
    pub name: String,
    /// Ordinal in the trailing string table.
    pub string_ordinal: usize,
    /// Resolved local path using the stored directory or a caller override.
    pub path: PathBuf,
    /// Whether the resolved path exists as a file.
    pub present: bool,
}

/// Assembly components and the stored directory prefix.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AssemblyManifest {
    /// Assembly file.
    pub source: PathBuf,
    /// Owning stream.
    pub stream: String,
    /// Stored directory string, when the table has one.
    pub directory: Option<String>,
    /// Child components in string-table order.
    pub components: Vec<AssemblyComponent>,
    /// Complete string table.
    pub strings: Vec<String>,
}

/// Result of rewriting the external-reference directory.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RelinkReport {
    /// Source assembly.
    pub source: PathBuf,
    /// Written assembly.
    pub output: PathBuf,
    /// Directory string found in the source, when present.
    pub old_directory: Option<String>,
    /// Replacement directory string.
    pub new_directory: String,
    /// Rewritten strings, in table order.
    pub changed_strings: Vec<String>,
    /// Owning stream.
    pub stream: String,
    /// SHA-256 of the source file.
    pub source_sha256: String,
    /// SHA-256 of the output file.
    pub output_sha256: String,
    /// True when the output is byte-identical to the source.
    pub byte_identical: bool,
}

struct ExtrefPayload {
    stream: String,
    payload: Vec<u8>,
    span: Option<(usize, usize)>,
}

impl ExtrefPayload {
    fn read(path: &Path, bytes: &[u8]) -> Result<Self, EditError> {
        if bytes.starts_with(b"SPLMSSTR") {
            let container = container::scan_bytes(bytes)
                .map_err(|error| EditError::MalformedContainer(error.to_string()))?;
            let entry = container
                .entries
                .iter()
                .find(|entry| entry.name.contains(EXTREF_MARKER) && entry.file_span().is_some())
                .ok_or_else(|| EditError::MissingStream(EXTREF_MARKER.to_string()))?;
            let (offset, length) = entry
                .file_span()
                .ok_or_else(|| EditError::MissingStream(EXTREF_MARKER.to_string()))?;
            let offset = usize::try_from(offset).map_err(|_| {
                EditError::MalformedContainer("external-reference offset overflow".into())
            })?;
            let length = usize::try_from(length).map_err(|_| {
                EditError::MalformedContainer("external-reference length overflow".into())
            })?;
            let end = offset
                .checked_add(length)
                .filter(|end| *end <= bytes.len())
                .ok_or_else(|| {
                    EditError::MalformedContainer("external-reference span exceeds file".into())
                })?;
            return Ok(Self {
                stream: entry.name.clone(),
                payload: bytes[offset..end].to_vec(),
                span: Some((offset, length)),
            });
        }
        if bytes.len() >= 8 && bytes[..8] == CFB_MAGIC {
            let mut compound = cfb::open(path)?;
            let stream_path = compound
                .walk()
                .find(|entry| {
                    entry.is_stream() && entry.path().to_string_lossy().contains(EXTREF_MARKER)
                })
                .map(|entry| entry.path().to_path_buf())
                .ok_or_else(|| EditError::MissingStream(EXTREF_MARKER.to_string()))?;
            let mut payload = Vec::new();
            compound
                .open_stream(&stream_path)
                .map_err(|_| EditError::MissingStream(EXTREF_MARKER.to_string()))?
                .read_to_end(&mut payload)?;
            return Ok(Self {
                stream: stream_path.to_string_lossy().replace('\\', "/"),
                payload,
                span: None,
            });
        }
        Err(EditError::UnsupportedContainer(
            "only legacy CFB and SPLMSSTR .prt files are supported".to_string(),
        ))
    }
}

/// Read the trailing string table of the assembly's `ExternalReferences`
/// stream.
pub fn read_external_reference_table(path: &Path) -> Result<ExternalReferenceTable, EditError> {
    let bytes = fs::read(path)?;
    let source = ExtrefPayload::read(path, &bytes)?;
    let (table_offset, strings) =
        locate_assembly_string_table(&source.payload).ok_or_else(|| {
            EditError::MalformedContainer(format!(
                "external-reference string table is malformed (stream {}, {} bytes, tail {})",
                source.stream,
                source.payload.len(),
                hex_tail(&source.payload, 16)
            ))
        })?;
    Ok(ExternalReferenceTable {
        stream: source.stream,
        payload_offset: source
            .span
            .map_or(0, |(offset, _)| u64::try_from(offset).unwrap_or(0)),
        table_offset,
        strings,
    })
}

/// Locate the trailing string table. The shared container locator rejects
/// empty strings because they cannot prove a complete table; NX10 assemblies
/// store one empty ordinal, so this reader accepts empty UTF-8 entries and
/// requires at least one child name plus one directory-shaped string.
fn locate_assembly_string_table(payload: &[u8]) -> Option<(usize, Vec<String>)> {
    let start = payload.len().saturating_sub(2 * 1024 * 1024);
    let mut fallback = None;
    for at in (start..payload.len().saturating_sub(5)).rev() {
        if payload[at] != 1 {
            continue;
        }
        let count = u32::from_le_bytes(payload[at + 1..at + 5].try_into().ok()?);
        if count == 0 || count > 65_535 {
            continue;
        }
        let mut position = at + 5;
        let mut strings = Vec::new();
        let mut valid = true;
        for _ in 0..count {
            let Some(length) = payload
                .get(position..position + 2)
                .map(|value| usize::from(u16::from_le_bytes([value[0], value[1]])))
            else {
                valid = false;
                break;
            };
            let text_start = position + 2;
            let Some(text_end) = text_start.checked_add(length) else {
                valid = false;
                break;
            };
            let Some(raw) = payload.get(text_start..text_end) else {
                valid = false;
                break;
            };
            let Ok(value) = std::str::from_utf8(raw) else {
                valid = false;
                break;
            };
            if value
                .chars()
                .any(|character| character == '\0' || character.is_control())
            {
                valid = false;
                break;
            }
            strings.push(value.to_owned());
            position = text_end;
        }
        if !valid || position != payload.len() {
            continue;
        }
        if strings.iter().any(|value| is_component_name(value))
            && strings.iter().any(|value| is_directory_string(value))
        {
            return Some((at, strings));
        }
        fallback.get_or_insert((at, strings));
    }
    fallback
}

/// Resolve the assembly's external child components.
///
/// The string table also carries the assembly's own file name; a string equal
/// to the input file name is omitted from `components`.
pub fn read_assembly_manifest(
    path: &Path,
    root_override: Option<&Path>,
) -> Result<AssemblyManifest, EditError> {
    let table = read_external_reference_table(path)?;
    let assembly_name = path
        .file_name()
        .and_then(|name| name.to_str())
        .unwrap_or_default();
    let directory = root_override
        .map(|root| root.to_string_lossy().to_string())
        .or_else(|| {
            table
                .strings
                .iter()
                .rev()
                .find(|value| is_directory_string(value))
                .cloned()
        });
    let base = path.parent().unwrap_or_else(|| Path::new("."));
    let mut components = Vec::new();
    for (ordinal, value) in table.strings.iter().enumerate() {
        if !is_component_name(value) || value.eq_ignore_ascii_case(assembly_name) {
            continue;
        }
        let resolved = directory
            .as_deref()
            .map_or_else(|| base.join(value), |dir| portable_path(dir).join(value));
        let present = resolved.is_file();
        components.push(AssemblyComponent {
            name: value.clone(),
            string_ordinal: ordinal,
            path: resolved,
            present,
        });
    }
    Ok(AssemblyManifest {
        source: path.to_path_buf(),
        stream: table.stream,
        directory,
        components,
        strings: table.strings,
    })
}

/// Replace the stored component directory with `new_root`.
///
/// Component names and their string ordinals stay unchanged, so indexed
/// occurrence records that point into the table keep their meaning.
pub fn relink_assembly(
    input: &Path,
    output: &Path,
    new_root: &Path,
) -> Result<RelinkReport, EditError> {
    if input == output {
        return Err(EditError::Io(std::io::Error::new(
            std::io::ErrorKind::InvalidInput,
            "input and output must be different paths",
        )));
    }
    let source_bytes = fs::read(input)?;
    let source_sha256 = sha256_hex(&source_bytes);
    let source = ExtrefPayload::read(input, &source_bytes)?;
    let table = read_external_reference_table(input)?;
    let new_directory = new_root
        .to_string_lossy()
        .trim_end_matches(['\\', '/'])
        .to_string();
    if new_directory.is_empty() {
        return Err(EditError::MalformedContainer(
            "replacement directory is empty".into(),
        ));
    }
    let old_directory = table
        .strings
        .iter()
        .rev()
        .find(|value| is_directory_string(value))
        .cloned();
    if old_directory.is_none() {
        return Err(EditError::MalformedContainer(
            "external-reference table has no directory string".into(),
        ));
    }
    let mut strings = table.strings.clone();
    let mut changed_strings = Vec::new();
    for value in &mut strings {
        if is_directory_string(value) && *value != new_directory {
            value.clone_from(&new_directory);
            changed_strings.push(value.clone());
        }
    }
    if changed_strings.is_empty() {
        let temp = temporary_path(output);
        if let Err(error) = fs::copy(input, &temp) {
            drop(fs::remove_file(&temp));
            return Err(error.into());
        }
        replace_file(&temp, output)?;
        let output_sha256 = sha256_hex(&fs::read(output)?);
        return Ok(RelinkReport {
            source: input.to_path_buf(),
            output: output.to_path_buf(),
            old_directory,
            new_directory,
            changed_strings,
            stream: table.stream,
            source_sha256: source_sha256.clone(),
            output_sha256: output_sha256.clone(),
            byte_identical: source_sha256 == output_sha256,
        });
    }

    let new_payload = rebuild_string_table(&source.payload, table.table_offset, &strings)?;
    let temp = temporary_path(output);
    let write = if source_bytes.starts_with(b"SPLMSSTR") {
        let (offset, length) = source.span.ok_or_else(|| {
            EditError::MalformedContainer("SPLMSSTR external-reference span is absent".into())
        })?;
        let new_bytes = repack_splmsstr_payload(&source_bytes, offset, length, &new_payload)?;
        fs::write(&temp, new_bytes).map_err(EditError::from)
    } else if source_bytes.len() >= 8 && source_bytes[..8] == CFB_MAGIC {
        let mut compound = cfb::open(input)?;
        copy_cfb_with_replacement(
            &mut compound,
            &temp,
            source.stream.trim_start_matches('/'),
            &new_payload,
        )
    } else {
        Err(EditError::UnsupportedContainer(
            "only legacy CFB and SPLMSSTR .prt files are supported".to_string(),
        ))
    };
    if let Err(error) = write {
        drop(fs::remove_file(&temp));
        return Err(error);
    }
    replace_file(&temp, output)?;

    let written = read_external_reference_table(output)?;
    if written.strings != strings {
        drop(fs::remove_file(output));
        return Err(EditError::MalformedContainer(
            "written external-reference table does not read back".into(),
        ));
    }
    let output_sha256 = sha256_hex(&fs::read(output)?);
    Ok(RelinkReport {
        source: input.to_path_buf(),
        output: output.to_path_buf(),
        old_directory,
        new_directory,
        changed_strings,
        stream: table.stream,
        source_sha256: source_sha256.clone(),
        output_sha256: output_sha256.clone(),
        byte_identical: source_sha256 == output_sha256,
    })
}

fn rebuild_string_table(
    payload: &[u8],
    table_offset: usize,
    strings: &[String],
) -> Result<Vec<u8>, EditError> {
    let original_count_u32 = payload
        .get(table_offset + 1..table_offset + 5)
        .map(|value| u32::from_le_bytes([value[0], value[1], value[2], value[3]]))
        .ok_or_else(|| EditError::MalformedContainer("string table count is truncated".into()))?;
    let original_count = usize::try_from(original_count_u32)
        .map_err(|_| EditError::MalformedContainer("string table count exceeds usize".into()))?;
    if strings.len() != original_count {
        return Err(EditError::MalformedContainer(
            "replacement string count differs from the source table".into(),
        ));
    }
    let mut output = Vec::new();
    output.extend_from_slice(&payload[..table_offset]);
    output.push(1);
    output.extend_from_slice(&original_count_u32.to_le_bytes());
    for value in strings {
        let length = u16::try_from(value.len()).map_err(|_| {
            EditError::MalformedContainer("external-reference string exceeds u16".into())
        })?;
        output.extend_from_slice(&length.to_le_bytes());
        output.extend_from_slice(value.as_bytes());
    }
    Ok(output)
}

fn hex_tail(bytes: &[u8], length: usize) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let start = bytes.len().saturating_sub(length);
    let mut text = String::with_capacity(length * 2);
    for byte in &bytes[start..] {
        text.push(char::from(HEX[usize::from(byte >> 4)]));
        text.push(char::from(HEX[usize::from(byte & 0x0f)]));
    }
    text
}

fn is_component_name(value: &str) -> bool {
    value.to_ascii_lowercase().ends_with(".prt")
}

fn is_directory_string(value: &str) -> bool {
    !value.is_empty()
        && !is_component_name(value)
        && (value.contains(':')
            || value.starts_with('/')
            || value.starts_with('\\')
            || value.contains('/')
            || value.contains('\\'))
}

fn portable_path(value: &str) -> PathBuf {
    #[cfg(windows)]
    {
        PathBuf::from(value)
    }
    #[cfg(not(windows))]
    {
        PathBuf::from(value.replace('\\', "/"))
    }
}

#[cfg(test)]
mod tests {
    use std::fs;
    use std::io::{Read, Write};
    use std::path::{Path, PathBuf};
    use std::sync::atomic::{AtomicU64, Ordering};

    use super::{
        read_assembly_manifest, read_external_reference_table, relink_assembly, AssemblyManifest,
    };
    use crate::container;
    use crate::test_support::test_prt::prt_with_named_payloads;

    static NEXT_TEMP: AtomicU64 = AtomicU64::new(1);

    fn temp_path(stem: &str) -> PathBuf {
        let ordinal = NEXT_TEMP.fetch_add(1, Ordering::Relaxed);
        std::env::temp_dir().join(format!(
            "nx-assembly-{}-{ordinal}-{stem}",
            std::process::id()
        ))
    }

    fn extref_payload(directory: &str, names: &[&str]) -> Vec<u8> {
        let mut strings = vec!["assembly.prt", "C", directory, "CM", "O", "RM"];
        strings.extend_from_slice(names);
        strings.push("");
        let mut payload = b"EXTREFSTREAM".to_vec();
        payload.extend_from_slice(&3u32.to_le_bytes());
        payload.extend_from_slice(&0u32.to_le_bytes());
        payload.extend_from_slice(&[0; 4]);
        payload.push(0);
        payload.extend_from_slice(&0u32.to_le_bytes());
        payload.push(1);
        payload.extend_from_slice(
            &u32::try_from(strings.len())
                .expect("fixture count fits u32")
                .to_le_bytes(),
        );
        for value in strings {
            payload.extend_from_slice(
                &u16::try_from(value.len())
                    .expect("fixture string fits u16")
                    .to_le_bytes(),
            );
            payload.extend_from_slice(value.as_bytes());
        }
        payload
    }

    fn splmsstr_assembly(directory: &str, names: &[&str]) -> Vec<u8> {
        prt_with_named_payloads(&[
            (
                "/Root/FastLoad/ExternalReferences",
                extref_payload(directory, names),
            ),
            ("/Root/part/attrs", b"<UgAttributes/>".to_vec()),
        ])
    }

    fn cfb_assembly(path: &Path, directory: &str, names: &[&str]) {
        let file = fs::File::create(path).expect("create CFB");
        let mut compound = cfb::CompoundFile::create(file).expect("open CFB");
        compound.create_storage("/part").expect("part storage");
        let mut attrs = compound.create_stream("/part/attrs").expect("attrs stream");
        attrs.write_all(b"<UgAttributes/>").expect("attrs bytes");
        attrs.flush().expect("flush attrs");
        compound
            .create_storage("/UG_PART")
            .expect("UG_PART storage");
        let mut extref = compound
            .create_stream("/UG_PART/ExternalReferences")
            .expect("extref stream");
        extref
            .write_all(&extref_payload(directory, names))
            .expect("extref bytes");
        extref.flush().expect("flush extref");
        compound.flush().expect("flush CFB");
    }

    fn payloads(path: &Path) -> Vec<(String, Vec<u8>)> {
        let bytes = fs::read(path).expect("read part");
        let container = container::scan_bytes(bytes.as_slice()).expect("scan part");
        container
            .entries
            .iter()
            .filter_map(|entry| {
                let (offset, length) = entry.file_span()?;
                let offset = usize::try_from(offset).ok()?;
                let length = usize::try_from(length).ok()?;
                Some((
                    entry.name.clone(),
                    bytes.get(offset..offset + length)?.to_vec(),
                ))
            })
            .collect()
    }

    fn assert_manifest(manifest: &AssemblyManifest, directory: &str, names: &[&str]) {
        assert_eq!(manifest.directory.as_deref(), Some(directory));
        assert_eq!(manifest.components.len(), names.len());
        for (component, name) in manifest.components.iter().zip(names) {
            assert_eq!(&component.name, name);
        }
    }

    #[test]
    fn manifest_reads_directory_and_components() {
        let directory = r"C:\pump";
        let bytes = splmsstr_assembly(directory, &["10泵体.prt", "9泵盖.prt"]);
        let folder = temp_path("manifest-dir");
        fs::create_dir_all(&folder).expect("create folder");
        let path = folder.join("assembly.prt");
        fs::write(&path, bytes).expect("write part");
        let manifest = read_assembly_manifest(&path, None).expect("manifest");
        assert_manifest(&manifest, directory, &["10泵体.prt", "9泵盖.prt"]);
        assert!(!manifest.components[0].present);
        drop(fs::remove_dir_all(folder));
    }

    #[test]
    fn relink_splmsstr_rewrites_directory_and_keeps_other_payloads() {
        let source = temp_path("splmsstr-in.prt");
        let output = temp_path("splmsstr-out.prt");
        fs::write(
            &source,
            splmsstr_assembly(r"Z:\old", &["10泵体.prt", "9泵盖.prt"]),
        )
        .expect("write source");
        let report = relink_assembly(&source, &output, Path::new(r"C:\new\pump")).expect("relink");
        assert_eq!(report.old_directory.as_deref(), Some(r"Z:\old"));
        assert_eq!(report.new_directory, r"C:\new\pump");
        assert_eq!(report.changed_strings, vec![r"C:\new\pump".to_string()]);
        let table = read_external_reference_table(&output).expect("read table");
        assert_eq!(table.strings[2], r"C:\new\pump");
        let before = payloads(&source);
        let after = payloads(&output);
        assert_eq!(before.len(), after.len());
        for ((before_name, before_bytes), (after_name, after_bytes)) in
            before.iter().zip(after.iter())
        {
            assert_eq!(before_name, after_name);
            if before_name.ends_with("ExternalReferences") {
                assert_ne!(before_bytes, after_bytes);
            } else {
                assert_eq!(before_bytes, after_bytes, "{before_name}");
            }
        }
        drop(fs::remove_file(source));
        drop(fs::remove_file(output));
    }

    #[test]
    fn relink_cfb_rewrites_directory_and_keeps_other_streams() {
        let source = temp_path("cfb-in.prt");
        let output = temp_path("cfb-out.prt");
        cfb_assembly(&source, r"E:\zsj\齿轮泵", &["10泵体.prt", "9泵盖.prt"]);
        let report = relink_assembly(&source, &output, Path::new(r"C:\RE\pump")).expect("relink");
        assert_eq!(report.old_directory.as_deref(), Some(r"E:\zsj\齿轮泵"));
        assert_eq!(report.changed_strings, vec![r"C:\RE\pump".to_string()]);
        let mut source_compound = cfb::open(&source).expect("open source");
        let mut output_compound = cfb::open(&output).expect("open output");
        let mut source_attrs = Vec::new();
        source_compound
            .open_stream("/part/attrs")
            .expect("source attrs")
            .read_to_end(&mut source_attrs)
            .expect("read source attrs");
        let mut output_attrs = Vec::new();
        output_compound
            .open_stream("/part/attrs")
            .expect("output attrs")
            .read_to_end(&mut output_attrs)
            .expect("read output attrs");
        assert_eq!(source_attrs, output_attrs);
        let manifest = read_assembly_manifest(&output, None).expect("manifest");
        assert_eq!(manifest.directory.as_deref(), Some(r"C:\RE\pump"));
        drop(fs::remove_file(source));
        drop(fs::remove_file(output));
    }

    #[test]
    fn relink_splmsstr_growth_shifts_later_payloads() {
        let source = temp_path("grow-in.prt");
        let output = temp_path("grow-out.prt");
        fs::write(&source, splmsstr_assembly(r"C:\p", &["10泵体.prt"])).expect("write source");
        let before = payloads(&source);
        let report = relink_assembly(
            &source,
            &output,
            Path::new(r"C:\RE\artifacts\gearpump_work\parts_extended_directory_name"),
        )
        .expect("relink");
        assert!(
            fs::metadata(&output).expect("metadata").len()
                > fs::metadata(&source).expect("metadata").len()
        );
        let after = payloads(&output);
        assert_eq!(before.len(), after.len());
        for ((before_name, before_bytes), (after_name, after_bytes)) in
            before.iter().zip(after.iter())
        {
            assert_eq!(before_name, after_name);
            if !before_name.ends_with("ExternalReferences") {
                assert_eq!(before_bytes, after_bytes, "{before_name}");
            }
        }
        assert_eq!(report.changed_strings.len(), 1);
        drop(fs::remove_file(source));
        drop(fs::remove_file(output));
    }

    #[test]
    fn relink_noop_is_byte_identical() {
        let source = temp_path("noop-in.prt");
        let output = temp_path("noop-out.prt");
        fs::write(&source, splmsstr_assembly(r"C:\pump", &["10泵体.prt"])).expect("write source");
        let report = relink_assembly(&source, &output, Path::new(r"C:\pump")).expect("relink");
        assert!(report.changed_strings.is_empty());
        assert!(report.byte_identical);
        assert_eq!(
            fs::read(&source).expect("source"),
            fs::read(&output).expect("output")
        );
        drop(fs::remove_file(source));
        drop(fs::remove_file(output));
    }

    #[test]
    fn relink_rejects_same_path() {
        let source = temp_path("same.prt");
        fs::write(&source, splmsstr_assembly(r"C:\pump", &["10泵体.prt"])).expect("write");
        assert!(relink_assembly(&source, &source, Path::new(r"C:\pump")).is_err());
        drop(fs::remove_file(source));
    }
}
