// SPDX-License-Identifier: Apache-2.0
//! Placement atom, frame, and wire admission invariants.

use super::*;

fn frame() -> Vec<u8> {
    let mut bytes = MAGIC_VALUE.to_vec();
    bytes.extend([1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0]);
    bytes.extend([0, 0, 0, 1, 63, 0xf0, 0, 0, 0]);
    bytes.extend(b"\x03\x03A\0\x03\x2601234567-89ab-cdef-0123-456789abcdef\0");
    bytes
}

fn legacy_frame() -> Vec<u8> {
    let mut bytes = LEGACY_MAGIC_VALUE.to_vec();
    bytes.extend_from_slice(&frame()[MAGIC_VALUE.len()..]);
    bytes
}

fn frame_with_identity(prefix: &[u8], identity: &str) -> Vec<u8> {
    let mut bytes = prefix.to_vec();
    bytes.extend([1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0]);
    bytes.extend([0, 0, 0, 1, 63, 0xf0, 0, 0, 0]);
    bytes.extend(b"\x03\x03A\0");
    bytes.push(3);
    bytes.push(u8::try_from(identity.len() + 2).unwrap());
    bytes.extend_from_slice(identity.as_bytes());
    bytes.push(0);
    bytes
}

#[test]
fn placement_atoms_use_truncated_binary64_and_preserve_signed_zero() {
    for (raw, value) in [
        (&[0][..], 0.0_f64),
        (&[1][..], 1.0),
        (&[0x80][..], -0.0),
        (&[0x81][..], -1.0),
        (&[0xd0, 0, 0, 0][..], -0.5),
        (&[0x50, 0x18, 0, 0][..], 1.5),
        (&[0x30, 0x28, 0, 0, 0, 0, 0, 0][..], 3.0),
    ] {
        let mut view = View::over_retained(raw);
        assert_eq!(matrix_atom(&mut view).unwrap().to_bits(), value.to_bits());
        assert!(view.is_empty());
        for length in 0..raw.len() {
            assert!(matrix_atom(&mut View::over_retained(&raw[..length])).is_none());
        }
    }
    assert!(matrix_atom(&mut View::over_retained(&[0x10])).is_none());
    assert!(matrix_atom(&mut View::over_retained(&[0x6f, 0xff, 0, 0])).is_none());
}

#[test]
fn placement_frame_requires_complete_text_terminators_and_proper_rotation() {
    let bytes = frame();
    let parsed = parse_frame(View::over_retained(&bytes)).unwrap();
    assert_eq!(parsed.length, bytes.len());
    assert_eq!(parsed.label, "A");
    assert_eq!(
        parsed.matrix,
        [1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0.]
    );
    for length in 0..bytes.len() {
        assert!(parse_frame(View::over_retained(&bytes[..length])).is_none());
    }
    let mut reflection = bytes.clone();
    reflection[6] = 0x81;
    assert!(parse_frame(View::over_retained(&reflection)).is_none());
    let mut malformed = bytes;
    *malformed.last_mut().unwrap() = 1;
    assert!(parse_frame(View::over_retained(&malformed)).is_none());
}

#[test]
fn placement_accepts_nx10_legacy_prefix_with_the_same_matrix() {
    let modern = frame();
    let legacy = legacy_frame();
    assert_eq!(legacy.len() + 2, modern.len());
    let parsed = parse_frame(View::over_retained(&legacy)).unwrap();
    let reference = parse_frame(View::over_retained(&modern)).unwrap();
    assert_eq!(parsed.length, legacy.len());
    assert_eq!(parsed.matrix, reference.matrix);
    assert_eq!(parsed.label, reference.label);
    assert_eq!(parsed.uuid, reference.uuid);
}

#[test]
fn placement_accepts_opaque_nx10_identity_text() {
    let identity = "laptop-fc1.GenuineInt.1558328259.0.18420.8";
    let bytes = frame_with_identity(&LEGACY_MAGIC_VALUE, identity);
    let parsed = parse_frame(View::over_retained(&bytes)).unwrap();
    assert_eq!(parsed.uuid, identity);
}

#[test]
fn placement_wire_rejects_derived_field_and_source_identity_changes() {
    let raw = frame();
    let wire = serde_json::json!({
        "id":"nx:assembly:placement#123", "raw_frame":NativeBytes::new(raw.as_slice()),
        "matrix":[1.,0.,0.,0., 0.,1.,0.,0., 0.,0.,1.,0.],
        "label":"A", "uuid":"01234567-89ab-cdef-0123-456789abcdef",
        "source_entry":ENTRY_NAME, "source_offset":123,
    });
    let record: AssemblyPlacement = serde_json::from_value(wire.clone()).unwrap();
    assert_eq!(serde_json::to_value(record).unwrap(), wire);
    for (field, value) in [
        ("id", serde_json::json!("nx:assembly:placement#124")),
        (
            "source_entry",
            serde_json::json!("/Root/FastLoad/Structure"),
        ),
        ("label", serde_json::json!("B")),
        (
            "uuid",
            serde_json::json!("11234567-89ab-cdef-0123-456789abcdef"),
        ),
    ] {
        let mut changed = wire.clone();
        changed[field] = value;
        assert!(serde_json::from_value::<AssemblyPlacement>(changed).is_err());
    }
    let mut changed = wire;
    changed["matrix"][3] = serde_json::json!(1.0);
    assert!(serde_json::from_value::<AssemblyPlacement>(changed).is_err());
}
