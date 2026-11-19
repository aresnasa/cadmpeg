// SPDX-License-Identifier: Apache-2.0
//! RecurDyn `.rad` animation frames: one rigid transform track per body.
//!
//! The layout is the recovered one documented in the NX Motion notes: the
//! file holds a run of constant-step frame times followed by self-describing
//! transform arrays. Each array header is `1, count, 1, 4, 4` (little-endian
//! `u32`) and each record is `1, 4, 4` plus a row-major `4x4` `f64` matrix.

use std::fs;
use std::path::Path;

use anyhow::{bail, Context};

const ARRAY_HEADER_BYTES: usize = 20;
const RECORD_SIZE: usize = 140;
const RECORD_PREFIX_BYTES: usize = 12;

/// One body's per-frame transform.
#[derive(Debug, Clone)]
pub struct RadTrack {
    /// Row-major `4x4` matrices, one per frame.
    pub matrices: Vec<[[f64; 4]; 4]>,
    /// True when every frame carries the same matrix.
    pub fixed: bool,
}

/// Parsed `.rad` animation.
#[derive(Debug, Clone)]
pub struct RadAnimation {
    /// Frame times in seconds.
    pub times: Vec<f64>,
    /// Body transform tracks.
    pub tracks: Vec<RadTrack>,
}

impl RadAnimation {
    /// Number of stored frames.
    #[must_use]
    pub fn frame_count(&self) -> usize {
        self.tracks.first().map_or(0, |track| track.matrices.len())
    }

    /// Time of `frame`, clamped to the stored range.
    #[must_use]
    pub fn time(&self, frame: usize) -> f64 {
        self.times
            .get(frame)
            .or_else(|| self.times.last())
            .copied()
            .unwrap_or(0.0)
    }

    /// Indices of tracks whose matrix changes over time.
    #[must_use]
    pub fn moving_tracks(&self) -> Vec<usize> {
        self.tracks
            .iter()
            .enumerate()
            .filter(|(_, track)| !track.fixed)
            .map(|(index, _)| index)
            .collect()
    }

    /// Matrix of `track` at `frame`, clamped to the stored range.
    #[must_use]
    pub fn matrix(&self, track: usize, frame: usize) -> Option<&[[f64; 4]; 4]> {
        let track = self.tracks.get(track)?;
        track.matrices.get(frame).or_else(|| track.matrices.last())
    }

    /// Z coordinate of the rotation axis of `track`, derived from the frame
    /// with the largest rotation angle and an axis assumed parallel to X.
    #[must_use]
    pub fn axis_z(&self, track: usize) -> Option<f64> {
        let track = self.tracks.get(track)?;
        let first = track.matrices.first()?;
        let mut best: Option<(f64, &[[f64; 4]; 4])> = None;
        for matrix in &track.matrices {
            let trace = matrix[0][0] + matrix[1][1] + matrix[2][2];
            let angle = ((trace - 1.0) / 2.0).clamp(-1.0, 1.0).acos();
            if best.is_none_or(|(value, _)| angle > value) {
                best = Some((angle, matrix));
            }
        }
        let _ = first;
        let (angle, matrix) = best?;
        let one_minus_cos = 1.0 - angle.cos();
        if one_minus_cos < 1e-9 {
            return None;
        }
        Some(matrix[2][3] / one_minus_cos)
    }
}

fn read_u32(data: &[u8], offset: usize) -> Option<u32> {
    let bytes = data.get(offset..offset.checked_add(4)?)?;
    Some(u32::from_le_bytes(bytes.try_into().ok()?))
}

fn read_f64(data: &[u8], offset: usize) -> Option<f64> {
    let bytes = data.get(offset..offset.checked_add(8)?)?;
    Some(f64::from_le_bytes(bytes.try_into().ok()?))
}

fn read_matrix(data: &[u8], offset: usize) -> Option<[[f64; 4]; 4]> {
    let mut matrix = [[0.0; 4]; 4];
    for (row, values) in matrix.iter_mut().enumerate() {
        for (column, value) in values.iter_mut().enumerate() {
            *value = read_f64(data, offset + (row * 4 + column) * 8)?;
        }
    }
    Some(matrix)
}

fn valid_matrix(matrix: &[[f64; 4]; 4]) -> bool {
    if !matrix.iter().flatten().all(|value| value.is_finite()) {
        return false;
    }
    if matrix[3] != [0.0, 0.0, 0.0, 1.0] {
        return false;
    }
    for row in 0..3 {
        for other in 0..3 {
            let dot: f64 = (0..3).map(|c| matrix[row][c] * matrix[other][c]).sum();
            let expected = if row == other { 1.0 } else { 0.0 };
            if (dot - expected).abs() > 1e-4 {
                return false;
            }
        }
    }
    true
}

fn transform_arrays(data: &[u8]) -> Vec<(usize, usize)> {
    let mut arrays = Vec::new();
    let mut offset = 0;
    while offset + ARRAY_HEADER_BYTES <= data.len() {
        if read_u32(data, offset) == Some(1) {
            if let Some(count) = read_u32(data, offset + 4).map(|value| value as usize) {
                if count > 0
                    && count <= 200_000
                    && read_u32(data, offset + 8) == Some(1)
                    && read_u32(data, offset + 12) == Some(4)
                    && read_u32(data, offset + 16) == Some(4)
                {
                    let end = offset + 8 + count * RECORD_SIZE;
                    if end <= data.len() {
                        let first = read_matrix(data, offset + ARRAY_HEADER_BYTES);
                        let last = read_matrix(
                            data,
                            offset + ARRAY_HEADER_BYTES + (count - 1) * RECORD_SIZE,
                        );
                        let mut accepted = first.as_ref().is_some_and(valid_matrix)
                            && last.as_ref().is_some_and(valid_matrix);
                        for index in [1, count / 2, count - 1] {
                            let prefix = offset + ARRAY_HEADER_BYTES + index * RECORD_SIZE
                                - RECORD_PREFIX_BYTES;
                            let words = (
                                read_u32(data, prefix),
                                read_u32(data, prefix + 4),
                                read_u32(data, prefix + 8),
                            );
                            if words != (Some(1), Some(4), Some(4)) {
                                accepted = false;
                                break;
                            }
                        }
                        if accepted {
                            arrays.push((offset, count));
                        }
                    }
                }
            }
        }
        offset += 2;
    }
    arrays
}

/// Longest run of constant-step frame times before `stop`.
fn time_axis(data: &[u8], stop: usize, count: usize) -> Vec<f64> {
    let mut best: Vec<f64> = Vec::new();
    let mut start = 0;
    while start + 16 <= stop.min(data.len()) {
        let Some(first) = read_f64(data, start) else {
            start += 2;
            continue;
        };
        if !first.is_finite() || first.abs() > 1e7 || first < 0.0 {
            start += 2;
            continue;
        }
        let mut times = vec![first];
        let mut step = None;
        let mut offset = start + 8;
        while times.len() < count {
            let Some(value) = read_f64(data, offset) else {
                break;
            };
            if !value.is_finite() || value.abs() > 1e7 {
                break;
            }
            let delta = value - times[times.len() - 1];
            match step {
                None => {
                    if !(1e-6..=100.0).contains(&delta) {
                        break;
                    }
                    step = Some(delta);
                }
                Some(step) => {
                    if (delta - step).abs() > 1e-9 * step.abs().max(1.0) {
                        break;
                    }
                }
            }
            times.push(value);
            offset += 8;
        }
        if times.len() > best.len() && times.len() >= 2 {
            best = times;
        }
        start += 2;
    }
    best
}

/// Parse a RecurDyn `.rad` animation.
pub fn parse_rad(path: &Path) -> anyhow::Result<RadAnimation> {
    let data = fs::read(path).with_context(|| format!("read {}", path.display()))?;
    if data.len() < 0x400 {
        bail!("{} is too small to be a RAD file", path.display());
    }
    let arrays = transform_arrays(&data);
    if arrays.is_empty() {
        bail!("no transform arrays found in {}", path.display());
    }
    let frame_count = arrays.iter().map(|(_, count)| *count).max().unwrap_or(0);
    let times = time_axis(&data, arrays[0].0, frame_count);
    let tracks = arrays
        .iter()
        .map(|(offset, count)| {
            let mut matrices = Vec::with_capacity(*count);
            for index in 0..*count {
                let matrix = read_matrix(&data, offset + ARRAY_HEADER_BYTES + index * RECORD_SIZE)
                    .context("truncated RAD transform record")?;
                if !valid_matrix(&matrix) {
                    bail!("invalid RAD transform at 0x{:x}", offset);
                }
                matrices.push(matrix);
            }
            let fixed = matrices.first().is_none_or(|first| {
                matrices.iter().all(|matrix| {
                    matrix
                        .iter()
                        .flatten()
                        .zip(first.iter().flatten())
                        .all(|(a, b)| (a - b).abs() < 1e-9)
                })
            });
            Ok(RadTrack { matrices, fixed })
        })
        .collect::<anyhow::Result<Vec<_>>>()?;
    Ok(RadAnimation { times, tracks })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn transform_arrays_reads_the_documented_header_and_records() {
        let mut data = vec![0u8; 0x40];
        data.extend(1u32.to_le_bytes());
        data.extend(3u32.to_le_bytes());
        data.extend(1u32.to_le_bytes());
        data.extend(4u32.to_le_bytes());
        data.extend(4u32.to_le_bytes());
        for index in 0..3 {
            // The first record shares the array header's `1, 4, 4` prefix, so
            // only later records carry their own twelve-byte prefix.
            if index > 0 {
                data.extend(1u32.to_le_bytes());
                data.extend(4u32.to_le_bytes());
                data.extend(4u32.to_le_bytes());
            }
            let mut matrix = [0.0_f64; 16];
            matrix[0] = 1.0;
            matrix[5] = 1.0;
            matrix[10] = 1.0;
            matrix[15] = 1.0;
            matrix[3] = index as f64;
            for value in matrix {
                data.extend(value.to_le_bytes());
            }
        }
        assert_eq!(transform_arrays(&data), vec![(0x40, 3)]);
    }

    #[test]
    fn matrix_axis_z_matches_a_pure_x_rotation() {
        let angle = 0.7_f64;
        let (sin, cos) = angle.sin_cos();
        let z = 50.0;
        let track = RadTrack {
            matrices: vec![
                [
                    [1.0, 0.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0, 0.0],
                    [0.0, 0.0, 1.0, 0.0],
                    [0.0, 0.0, 0.0, 1.0],
                ],
                [
                    [1.0, 0.0, 0.0, 0.0],
                    [0.0, cos, -sin, sin * z],
                    [0.0, sin, cos, (1.0 - cos) * z],
                    [0.0, 0.0, 0.0, 1.0],
                ],
            ],
            fixed: false,
        };
        let animation = RadAnimation {
            times: vec![0.0, 0.1],
            tracks: vec![track],
        };
        assert!((animation.axis_z(0).unwrap() - z).abs() < 1e-9);
    }
}
