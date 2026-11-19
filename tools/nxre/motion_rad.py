"""Read NX Motion / RecurDyn ``.rad`` animation files.

RecurDyn (the NX Motion solver) writes ``.rad`` animation data next to the
``.rplt`` plot channels.  The layout recovered from the gear-pump sample
(RecurDyn SDK 7.25.0, 201 states, 5.0 s) is:

* ``0x000``  small header.  The first words are ``4, 3``, which coincide with
  the ``.msg`` part/joint counts; the declared frame count is at ``0x054``
  (``202``), a 200-entry frame-id list (``2 .. 201``) starts at ``0x060``,
  the final frame id (``202``) is at ``0x380`` and the reference-frame time
  sentinel ``-99999.0`` is at ``0x384``.
* ``0x38C``  201 little-endian float64 time values ``0.0 .. 5.0`` with a
  ``0.025`` step (frames ``2 .. 202``).
* A sequence of self-describing transform arrays.  Each array starts with a
  20-byte header::

      u32 1, u32 count, u32 1, u32 4, u32 4

  followed by ``count`` records of 140 bytes::

      u32 1, u32 4, u32 4, float64[16]   # row-major 4x4 homogeneous matrix

  The first record's ``1, 4, 4`` prefix is shared with the array header, so
  matrices live at ``header + 20 + i * 140``.  The sample contains five
  arrays of 202 frames each (frames 1..202; frame 1 is a reference frame and
  record 1 maps to ``t = 0``).
* One standalone 140-byte transform record outside the arrays.
* A tail block with a 52-byte descriptor, 201 time labels and a 12-byte
  trailer ``u32 1, u32 rplt_header_bytes, u32 rplt_frame_width``.  The 201
  labels are ``0.0`` followed by the 200 stored ``.rplt`` record labels.

The 4x4 matrices are standard row-major homogeneous transforms
(``m[12:16] == [0, 0, 0, 1]``) with the translation in ``m[3], m[7], m[11]``.
The two time-varying tracks in the gear-pump file are the rigid-body
transforms of ``L002`` and ``L003``; their translations are
``(I - R) * (0, 0, 92)`` and ``(I - R) * (0, 0, 50)``, matching the revolute
joint pivots and the ``+/-900 deg`` (``+/-pi rad/s``) driver.

The file carries no names or entity ids, so the mapping of the static tracks
to ground/fixed bodies is inferred from order and from the model; only the
two rotating tracks can be validated independently against ``.rplt`` and
``.rmd``.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import struct
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable, Sequence

MATRIX_DOUBLES = 16
MATRIX_BYTES = 128
RECORD_PREFIX = (1, 4, 4)
RECORD_PREFIX_BYTES = 12
RECORD_SIZE = 140
ARRAY_HEADER_BYTES = 20


class MotionRadError(ValueError):
    """Raised when a ``.rad`` file does not match the recovered layout."""


# ---------------------------------------------------------------------------
# small binary helpers
# ---------------------------------------------------------------------------


def _u32(data: bytes, offset: int) -> int | None:
    if offset < 0 or offset + 4 > len(data):
        return None
    return struct.unpack_from("<I", data, offset)[0]


def _i32(data: bytes, offset: int) -> int | None:
    if offset < 0 or offset + 4 > len(data):
        return None
    return struct.unpack_from("<i", data, offset)[0]


def _f64(data: bytes, offset: int) -> float | None:
    if offset < 0 or offset + 8 > len(data):
        return None
    return struct.unpack_from("<d", data, offset)[0]


def _matrix_at(data: bytes, offset: int) -> tuple[float, ...]:
    if offset < 0 or offset + MATRIX_BYTES > len(data):
        raise MotionRadError(f"matrix at 0x{offset:x} runs past end of file")
    return struct.unpack_from("<16d", data, offset)


def _valid_matrix(matrix: Sequence[float], tolerance: float = 1e-6) -> bool:
    if len(matrix) != MATRIX_DOUBLES:
        return False
    if not all(math.isfinite(value) for value in matrix):
        return False
    if any(abs(matrix[index]) > 1e-9 for index in (12, 13, 14)):
        return False
    if abs(matrix[15] - 1.0) > 1e-9:
        return False
    rows = (matrix[0:3], matrix[4:7], matrix[8:11])
    for first in range(3):
        norm = sum(value * value for value in rows[first])
        if abs(norm - 1.0) > tolerance:
            return False
        for second in range(first + 1, 3):
            dot = sum(a * b for a, b in zip(rows[first], rows[second]))
            if abs(dot) > tolerance:
                return False
    return True


def _mat3(matrix: Sequence[float]) -> tuple[tuple[float, ...], ...]:
    return (
        (matrix[0], matrix[1], matrix[2]),
        (matrix[4], matrix[5], matrix[6]),
        (matrix[8], matrix[9], matrix[10]),
    )


def _mat3_mul(
    left: Sequence[Sequence[float]], right: Sequence[Sequence[float]]
) -> list[list[float]]:
    return [
        [sum(left[row][k] * right[k][column] for k in range(3)) for column in range(3)]
        for row in range(3)
    ]


def _mat3_transpose(matrix: Sequence[Sequence[float]]) -> list[list[float]]:
    return [[matrix[column][row] for column in range(3)] for row in range(3)]


def _rotation_vector_deg(
    before: Sequence[float], after: Sequence[float]
) -> tuple[float, float, float]:
    """Return the axis-angle vector (degrees) from ``before`` to ``after``."""
    delta = _mat3_mul(_mat3_transpose(_mat3(before)), _mat3(after))
    trace = delta[0][0] + delta[1][1] + delta[2][2]
    cosine = max(-1.0, min(1.0, (trace - 1.0) / 2.0))
    angle = math.acos(cosine)
    if angle < 1e-12:
        return (0.0, 0.0, 0.0)
    sine = math.sin(angle)
    axis = (
        (delta[2][1] - delta[1][2]) / (2.0 * sine),
        (delta[0][2] - delta[2][0]) / (2.0 * sine),
        (delta[1][0] - delta[0][1]) / (2.0 * sine),
    )
    scale = math.degrees(angle)
    return (axis[0] * scale, axis[1] * scale, axis[2] * scale)


def _canonical_axis(vector: Sequence[float]) -> tuple[float, float, float] | None:
    length = math.sqrt(sum(value * value for value in vector))
    if length < 1e-12:
        return None
    axis = [value / length for value in vector]
    for value in axis:
        if abs(value) > 1e-12:
            if value < 0.0:
                axis = [-component for component in axis]
            break
    # Strip floating-point dust so a pure X/Y/Z axis is reported cleanly.
    axis = [0.0 if abs(value) < 1e-12 else value for value in axis]
    return (axis[0], axis[1], axis[2])


def _perpendicular_basis(
    axis: Sequence[float],
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """Return a right-handed orthonormal basis perpendicular to ``axis``."""
    helper = (1.0, 0.0, 0.0)
    if abs(axis[0]) > 0.9:
        helper = (0.0, 1.0, 0.0)
    first = (
        axis[1] * helper[2] - axis[2] * helper[1],
        axis[2] * helper[0] - axis[0] * helper[2],
        axis[0] * helper[1] - axis[1] * helper[0],
    )
    norm = math.sqrt(sum(value * value for value in first))
    first = tuple(value / norm for value in first)
    second = (
        axis[1] * first[2] - axis[2] * first[1],
        axis[2] * first[0] - axis[0] * first[2],
        axis[0] * first[1] - axis[1] * first[0],
    )
    return first, second  # type: ignore[return-value]


def _estimate_axis_point(
    matrices: Sequence[Sequence[float]], axis: Sequence[float]
) -> tuple[float, float, float] | None:
    """Least-squares point on the rotation axis: ``t = (I - R) * p``.

    The component of ``p`` along the rotation axis is unobservable, so the
    solution is returned in the plane perpendicular to the axis (axis
    component zero).
    """
    first, second = _perpendicular_basis(axis)
    m11 = m12 = m22 = r1 = r2 = 0.0
    for matrix in matrices:
        rotation = _mat3(matrix)
        translation = (matrix[3], matrix[7], matrix[11])
        # A = I - R
        a = [
            [
                (1.0 if row == column else 0.0) - rotation[row][column]
                for column in range(3)
            ]
            for row in range(3)
        ]
        a_first = tuple(
            sum(a[row][column] * first[column] for column in range(3))
            for row in range(3)
        )
        a_second = tuple(
            sum(a[row][column] * second[column] for column in range(3))
            for row in range(3)
        )
        m11 += sum(value * value for value in a_first)
        m12 += sum(x * y for x, y in zip(a_first, a_second))
        m22 += sum(value * value for value in a_second)
        r1 += sum(x * y for x, y in zip(a_first, translation))
        r2 += sum(x * y for x, y in zip(a_second, translation))
    determinant = m11 * m22 - m12 * m12
    if abs(determinant) < 1e-18:
        return None
    p1 = (r1 * m22 - r2 * m12) / determinant
    p2 = (m11 * r2 - m12 * r1) / determinant
    point = [p1 * first[index] + p2 * second[index] for index in range(3)]
    point = [0.0 if abs(value) < 1e-9 else value for value in point]
    return (point[0], point[1], point[2])


# ---------------------------------------------------------------------------
# data model
# ---------------------------------------------------------------------------


@dataclass
class MatrixTrack:
    """One time series of row-major 4x4 homogeneous matrices."""

    index: int
    header_offset: int
    count: int
    matrices: list[tuple[float, ...]]
    matrix_offsets: list[int]
    record_size: int = RECORD_SIZE
    static: bool = True
    rotation_axis: tuple[float, float, float] | None = None
    signed_rotation_deg: float = 0.0
    rate_rad_s: float = 0.0
    angle_samples_deg: list[float] = field(default_factory=list)
    translations: list[tuple[float, float, float]] = field(default_factory=list)
    axis_point: tuple[float, float, float] | None = None
    inferred_entity: str | None = None

    @property
    def first_matrix_offset(self) -> int:
        return self.matrix_offsets[0] if self.matrix_offsets else self.header_offset

    @property
    def end_offset(self) -> int:
        return self.header_offset + 8 + self.count * self.record_size

    def to_dict(
        self,
        include_matrices: bool = True,
        include_samples: bool = True,
        entity_name: str | None = None,
    ) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "index": self.index,
            "header_offset": self.header_offset,
            "first_matrix_offset": self.first_matrix_offset,
            "end_offset": self.end_offset,
            "record_count": self.count,
            "record_size": self.record_size,
            "matrix_doubles": MATRIX_DOUBLES,
            "encoding": "float64 row-major 4x4 homogeneous (translation at 3/7/11)",
            "static": self.static,
            "inferred_entity": (
                entity_name if entity_name is not None else self.inferred_entity
            ),
            "rotation": {
                "axis": list(self.rotation_axis)
                if self.rotation_axis is not None
                else None,
                "signed_angle_deg": self.signed_rotation_deg,
                "turns": self.signed_rotation_deg / 360.0,
                "rate_rad_s": self.rate_rad_s,
                "axis_point_mm": list(self.axis_point)
                if self.axis_point is not None
                else None,
            },
            "translation": {
                "first_mm": list(self.translations[0]) if self.translations else None,
                "last_mm": list(self.translations[-1]) if self.translations else None,
            },
        }
        if include_samples:
            payload["rotation"]["angle_samples_deg"] = list(self.angle_samples_deg)
            payload["translation"]["samples_mm"] = [
                list(value) for value in self.translations
            ]
        if include_matrices:
            payload["matrices"] = [
                [list(matrix[0:4]), list(matrix[4:8]), list(matrix[8:12]), list(matrix[12:16])]
                for matrix in self.matrices
            ]
        return payload


@dataclass
class TailInfo:
    descriptor_offset: int
    descriptor_words: list[int]
    time_offset: int
    time_labels: list[float]
    trailer_offset: int
    trailer_words: list[int]

    @property
    def linked_rplt_header_bytes(self) -> int | None:
        return self.trailer_words[1] if len(self.trailer_words) >= 3 else None

    @property
    def linked_rplt_frame_width(self) -> int | None:
        return self.trailer_words[2] if len(self.trailer_words) >= 3 else None

    def to_dict(self) -> dict[str, Any]:
        return {
            "descriptor_offset": self.descriptor_offset,
            "descriptor_words": list(self.descriptor_words),
            "time_offset": self.time_offset,
            "time_count": len(self.time_labels),
            "time_labels": list(self.time_labels),
            "trailer_offset": self.trailer_offset,
            "trailer_words": list(self.trailer_words),
            "linked_rplt_header_bytes": self.linked_rplt_header_bytes,
            "linked_rplt_frame_width": self.linked_rplt_frame_width,
        }


@dataclass
class StandaloneRecord:
    offset: int
    header_words: list[int]
    matrix: tuple[float, ...]

    def to_dict(self) -> dict[str, Any]:
        return {
            "offset": self.offset,
            "header_words": list(self.header_words),
            "matrix": [
                list(self.matrix[0:4]),
                list(self.matrix[4:8]),
                list(self.matrix[8:12]),
                list(self.matrix[12:16]),
            ],
        }


@dataclass
class RadAnimation:
    path: str
    size: int
    sha256: str
    declared_frame_count: int
    frame_ids: list[int]
    final_frame_id: int | None
    reference_time_sentinel: float | None
    time_offset: int
    times: list[float]
    time_step: float
    tracks: list[MatrixTrack]
    standalone_records: list[StandaloneRecord]
    tail: TailInfo | None
    leading_header_words: list[int]
    time_axis_scan: dict[str, Any] = field(default_factory=dict)

    @property
    def track_count(self) -> int:
        return len(self.tracks)

    @property
    def frame_count(self) -> int:
        if self.tracks:
            return max(track.count for track in self.tracks)
        return self.declared_frame_count

    @property
    def duration_s(self) -> float:
        if len(self.times) < 2:
            return 0.0
        return self.times[-1] - self.times[0]

    def frame_times(self) -> list[float | None]:
        """Times for the animation frames, including the reference frame."""
        return [None, *self.times]

    def to_dict(
        self,
        include_matrices: bool = True,
        include_samples: bool = True,
        entity_map: dict[int, str] | None = None,
        extra: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        tracks = []
        for track in self.tracks:
            tracks.append(
                track.to_dict(
                    include_matrices=include_matrices,
                    include_samples=include_samples,
                    entity_name=(
                        entity_map.get(track.index) if entity_map else None
                    ),
                )
            )
        payload = {
            "format": "RecurDyn RAD (NX Motion animation)",
            "source": {
                "path": self.path,
                "size_bytes": self.size,
                "sha256": self.sha256,
            },
            "summary": {
                "track_count": self.track_count,
                "frame_count": self.frame_count,
                "time_sample_count": len(self.times),
                "duration_s": self.duration_s,
                "time_step_s": self.time_step,
                "standalone_record_count": len(self.standalone_records),
                "rotating_track_count": sum(1 for track in self.tracks if not track.static),
                "static_track_count": sum(1 for track in self.tracks if track.static),
            },
            "layout": {
                "record_size_bytes": RECORD_SIZE,
                "matrix_bytes": MATRIX_BYTES,
                "record_prefix": list(RECORD_PREFIX),
                "array_header_bytes": ARRAY_HEADER_BYTES,
                "array_header_grammar": "[u32 1][u32 count][u32 1][u32 4][u32 4]",
                "matrix_encoding": "float64 row-major 4x4 homogeneous; translation at indices 3,7,11",
                "frame_mapping": (
                    "track record 0 = reference frame (no stored time); "
                    "records 1..count-1 map to times[0..count-2]"
                ),
            },
            "header": {
                "leading_words": list(self.leading_header_words),
                "declared_frame_count_offset": 0x54,
                "declared_frame_count": self.declared_frame_count,
                "frame_id_list_offset": 0x60,
                "frame_ids": list(self.frame_ids),
                "final_frame_id_offset": 0x380,
                "final_frame_id": self.final_frame_id,
                "reference_time_sentinel_offset": 0x384,
                "reference_time_sentinel": self.reference_time_sentinel,
            },
            "time": {
                "unit": "s",
                "offset": self.time_offset,
                "count": len(self.times),
                "step_s": self.time_step,
                "values": list(self.times),
                "reference_frame_time_s": None,
                "frame_times_s": self.frame_times(),
            },
            "tracks": tracks,
            "standalone_records": [record.to_dict() for record in self.standalone_records],
            "tail": self.tail.to_dict() if self.tail is not None else None,
            "time_axis_scan": dict(self.time_axis_scan),
        }
        if extra:
            payload.update(extra)
        return payload


# ---------------------------------------------------------------------------
# parsing
# ---------------------------------------------------------------------------


def _find_transform_arrays(data: bytes) -> list[tuple[int, int, int]]:
    """Locate ``(1, count, 1, 4, 4)`` transform-array headers.

    Returns ``(header_offset, record_count, end_offset)`` tuples sorted by
    offset.  ``end_offset`` is the first byte after the last 128-byte matrix.
    """
    length = len(data)
    arrays: list[tuple[int, int, int]] = []
    for offset in range(0, max(0, length - ARRAY_HEADER_BYTES), 2):
        if _u32(data, offset) != 1:
            continue
        count = _u32(data, offset + 4)
        if not count or count > 200_000:
            continue
        if (_u32(data, offset + 8), _u32(data, offset + 12), _u32(data, offset + 16)) != (
            1,
            4,
            4,
        ):
            continue
        end = offset + 8 + count * RECORD_SIZE
        if end > length:
            continue
        first = _matrix_at(data, offset + 20)
        last = _matrix_at(data, offset + 20 + (count - 1) * RECORD_SIZE)
        if not _valid_matrix(first) or not _valid_matrix(last):
            continue
        sample_indices = {1, count // 2, count - 1}
        valid = True
        for index in sample_indices:
            prefix = offset + 20 + index * RECORD_SIZE - RECORD_PREFIX_BYTES
            if (
                _u32(data, prefix),
                _u32(data, prefix + 4),
                _u32(data, prefix + 8),
            ) != RECORD_PREFIX:
                valid = False
                break
        if valid:
            arrays.append((offset, count, end))
    arrays.sort(key=lambda item: item[0])
    return arrays


def _find_time_axis(
    data: bytes, stop: int
) -> tuple[int, list[float], float, dict[str, Any]]:
    """Find the longest constant-step float64 run before ``stop``."""
    best: tuple[int, int, float] | None = None
    for start in range(0, max(0, stop - 8 * 8), 2):
        first = _f64(data, start)
        if first is None or not math.isfinite(first) or abs(first) > 1e7:
            continue
        second = _f64(data, start + 8)
        if second is None:
            continue
        step = second - first
        if not math.isfinite(step) or step <= 0.0 or step > 1e6:
            continue
        count = 1
        previous = first
        while True:
            offset = start + count * 8
            if offset + 8 > stop:
                break
            value = _f64(data, offset)
            if value is None or not math.isfinite(value):
                break
            if abs((value - previous) - step) > 1e-9 * max(1.0, abs(step)):
                break
            count += 1
            previous = value
        if count >= 32:
            if (
                best is None
                or count > best[1]
                or (count == best[1] and start < best[0])
            ):
                best = (start, count, step)
    if best is None:
        raise MotionRadError("could not locate the animation time axis")
    start, count, step = best
    values = list(struct.unpack_from(f"<{count}d", data, start))
    scan = {
        "candidate_count": count,
        "rules": "longest constant positive-step float64 run before the first transform array",
    }
    return start, values, step, scan


def _find_standalone_records(
    data: bytes, arrays: Sequence[tuple[int, int, int]]
) -> list[StandaloneRecord]:
    """Find ``(1, 4, 4) + 4x4`` records outside any transform array."""
    length = len(data)
    ranges = [(offset, end) for offset, _count, end in arrays]
    records: list[StandaloneRecord] = []
    for offset in range(0, max(0, length - RECORD_SIZE), 2):
        if (_u32(data, offset), _u32(data, offset + 4), _u32(data, offset + 8)) != (
            RECORD_PREFIX
        ):
            continue
        matrix_offset = offset + RECORD_PREFIX_BYTES
        if matrix_offset + MATRIX_BYTES > length:
            continue
        if any(start <= offset < end for start, end in ranges):
            continue
        matrix = _matrix_at(data, matrix_offset)
        if not _valid_matrix(matrix):
            continue
        records.append(
            StandaloneRecord(
                offset=offset,
                header_words=[1, 4, 4],
                matrix=matrix,
            )
        )
    return records


def _parse_tail(data: bytes) -> TailInfo | None:
    """Parse the optional RPLT-link tail block."""
    length = len(data)
    if length < 12 + 16 + 8:
        return None
    trailer_offset = length - 12
    trailer = list(struct.unpack_from("<3I", data, trailer_offset))
    if trailer[0] == 0 or trailer[1] == 0 or trailer[1] > length * 16:
        return None
    if trailer[2] == 0 or trailer[2] > 10_000:
        return None

    end = trailer_offset
    last = _f64(data, end - 8)
    previous = _f64(data, end - 16)
    if last is None or previous is None:
        return None
    step = last - previous
    if not math.isfinite(step) or step <= 0.0:
        return None
    tolerance = 1e-9 * max(1.0, abs(step))
    count = 2
    while True:
        start = end - (count + 1) * 8
        if start < 0:
            break
        value = _f64(data, start)
        following = _f64(data, start + 8)
        if (
            value is None
            or following is None
            or not math.isfinite(value)
            or abs((following - value) - step) > tolerance
        ):
            break
        count += 1
    time_offset = end - count * 8
    if count < 8:
        return None
    time_labels = list(struct.unpack_from(f"<{count}d", data, time_offset))
    descriptor_offset = time_offset - 52
    descriptor_words: list[int] = []
    if descriptor_offset >= 0:
        words = struct.unpack_from("<13I", data, descriptor_offset)
        if words[-1] == count:
            descriptor_words = list(words)
    return TailInfo(
        descriptor_offset=descriptor_offset,
        descriptor_words=descriptor_words,
        time_offset=time_offset,
        time_labels=time_labels,
        trailer_offset=trailer_offset,
        trailer_words=trailer,
    )


def _analyze_track(
    index: int,
    header_offset: int,
    count: int,
    matrices: list[tuple[float, ...]],
    matrix_offsets: list[int],
    times: Sequence[float],
) -> MatrixTrack:
    static = True
    first = matrices[0]
    for matrix in matrices[1:]:
        if max(abs(matrix[i] - first[i]) for i in range(MATRIX_DOUBLES)) > 1e-9:
            static = False
            break

    rotation_vectors = [
        _rotation_vector_deg(matrices[i], matrices[i + 1]) for i in range(count - 1)
    ]
    total = [0.0, 0.0, 0.0]
    for vector in rotation_vectors:
        for component in range(3):
            total[component] += vector[component]

    axis = _canonical_axis(total)
    signed_rotation = 0.0
    angle_samples = [0.0]
    if axis is not None:
        signed_rotation = sum(
            sum(vector[component] * axis[component] for component in range(3))
            for vector in rotation_vectors
        )
        cumulative = 0.0
        for vector in rotation_vectors:
            cumulative += sum(
                vector[component] * axis[component] for component in range(3)
            )
            angle_samples.append(cumulative)

    duration = times[-1] - times[0] if len(times) >= 2 else 0.0
    rate = math.radians(signed_rotation) / duration if duration > 0 else 0.0
    translations = [(matrix[3], matrix[7], matrix[11]) for matrix in matrices]
    axis_point = _estimate_axis_point(matrices, axis) if axis is not None else None
    return MatrixTrack(
        index=index,
        header_offset=header_offset,
        count=count,
        matrices=matrices,
        matrix_offsets=matrix_offsets,
        static=static,
        rotation_axis=axis,
        signed_rotation_deg=signed_rotation,
        rate_rad_s=rate,
        angle_samples_deg=angle_samples if axis is not None else [],
        translations=translations,
        axis_point=axis_point,
    )


def parse_rad(path: str | Path) -> RadAnimation:
    """Parse a RecurDyn ``.rad`` animation file."""
    path = Path(path)
    data = path.read_bytes()
    if len(data) < 0x400:
        raise MotionRadError(f"{path} is too small to be a RAD file")

    arrays = _find_transform_arrays(data)
    if not arrays:
        raise MotionRadError("no transform arrays found in RAD file")

    time_offset, times, time_step, time_scan = _find_time_axis(data, arrays[0][0])
    declared_frame_count = _u32(data, 0x54) or max(count for _offset, count, _end in arrays)
    if declared_frame_count < 2:
        declared_frame_count = max(count for _offset, count, _end in arrays)

    # The header lists frame ids 2..201 and then the final id 202.
    frame_ids: list[int] = []
    for offset in range(0x60, 0x380, 4):
        value = _u32(data, offset)
        if value is None:
            break
        frame_ids.append(value)
    final_frame_id = _i32(data, 0x380)
    reference_sentinel = _f64(data, 0x384)

    tracks: list[MatrixTrack] = []
    for index, (header_offset, count, _end) in enumerate(arrays):
        matrices: list[tuple[float, ...]] = []
        matrix_offsets: list[int] = []
        for record in range(count):
            matrix_offset = header_offset + 20 + record * RECORD_SIZE
            matrix_offsets.append(matrix_offset)
            matrix = _matrix_at(data, matrix_offset)
            if not _valid_matrix(matrix):
                raise MotionRadError(
                    f"invalid transform at array {index} record {record} "
                    f"(0x{matrix_offset:x})"
                )
            matrices.append(matrix)
        tracks.append(
            _analyze_track(
                index=index,
                header_offset=header_offset,
                count=count,
                matrices=matrices,
                matrix_offsets=matrix_offsets,
                times=times,
            )
        )

    leading_words = [
        value
        for value in (_i32(data, offset) for offset in range(0, 0x60, 4))
        if value is not None
    ]

    return RadAnimation(
        path=str(path),
        size=len(data),
        sha256=hashlib.sha256(data).hexdigest(),
        declared_frame_count=declared_frame_count,
        frame_ids=frame_ids,
        final_frame_id=final_frame_id,
        reference_time_sentinel=reference_sentinel,
        time_offset=time_offset,
        times=times,
        time_step=time_step,
        tracks=tracks,
        standalone_records=_find_standalone_records(data, arrays),
        tail=_parse_tail(data),
        leading_header_words=leading_words,
        time_axis_scan=time_scan,
    )


def write_json(
    animation: RadAnimation,
    output: str | Path,
    include_matrices: bool = True,
    include_samples: bool = True,
    entity_map: dict[int, str] | None = None,
    extra: dict[str, Any] | None = None,
) -> Path:
    """Write a JSON representation and return the output path."""
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    payload = animation.to_dict(
        include_matrices=include_matrices,
        include_samples=include_samples,
        entity_map=entity_map,
        extra=extra,
    )
    output.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False, allow_nan=False),
        encoding="utf-8",
    )
    return output


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("rad", help="path to a RecurDyn .rad file")
    parser.add_argument(
        "--json",
        dest="json_path",
        help="write a JSON export to this path",
    )
    parser.add_argument(
        "--no-matrices",
        action="store_true",
        help="omit the 202x16 matrix values from the JSON",
    )
    parser.add_argument(
        "--entity",
        action="append",
        default=[],
        metavar="INDEX=NAME",
        help="annotate a track with an entity name, e.g. --entity 3=L002",
    )
    args = parser.parse_args(list(argv) if argv is not None else None)

    animation = parse_rad(args.rad)
    entity_map: dict[int, str] = {}
    for item in args.entity:
        if "=" not in item:
            parser.error(f"invalid --entity value: {item!r}")
        index_text, name = item.split("=", 1)
        entity_map[int(index_text)] = name

    print(f"tracks={animation.track_count} frames={animation.frame_count} "
          f"time_samples={len(animation.times)} step={animation.time_step:g}s")
    for track in animation.tracks:
        print(
            f"  track {track.index}: header=0x{track.header_offset:x} "
            f"records={track.count} static={track.static} "
            f"axis={track.rotation_axis} angle={track.signed_rotation_deg:.6f}deg "
            f"rate={track.rate_rad_s:.9f}rad/s"
        )
    if animation.tail is not None:
        print(
            f"  tail: labels={len(animation.tail.time_labels)} "
            f"rplt_header_bytes={animation.tail.linked_rplt_header_bytes} "
            f"rplt_frame_width={animation.tail.linked_rplt_frame_width}"
        )
    if args.json_path:
        output = write_json(
            animation,
            args.json_path,
            include_matrices=not args.no_matrices,
            entity_map=entity_map or None,
        )
        print(f"wrote {output}")
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
