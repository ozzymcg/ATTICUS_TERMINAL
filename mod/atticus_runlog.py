"""Binary runlog reader/writer structures for Atticus Efficiency Pack."""

from __future__ import annotations

from dataclasses import dataclass
import os
import struct
from typing import Dict, Iterable, List, Optional, Tuple


RUNLOG_MAGIC = b"ATLG"
RUNLOG_SCHEMA_MAJOR = 1
RUNLOG_SCHEMA_MINOR = 1

# Header:
# magic[4], major(u16), minor(u16), header_size(u16), frame_size(u16),
# session_id(u32), config_hash32(u32), map_hash32(u32), reserved(u32)
HEADER_STRUCT = struct.Struct("<4sHHHHIIII")

# Frame record prefix (little endian):
# time_ms(u32), seg_idx(i16), trig_idx(i16), flags(u32),
# pose/fused/target -> x,y,theta triplets (9 floats),
# alpha/conf/slip/late/particle_count/neff/ess_ratio/peakedness (8 floats)
# flags lower bits include correction/slip/lost state; high bits include
# correction gate reasons (blocked invalid/disabled/no-correction/confidence/
# lost/safe-window/jump/stale) and writeback-attempt marker.
FRAME_STRUCT = struct.Struct("<IhhI" + "f" * 17)


@dataclass
class RunlogHeader:
    """Parsed runlog header."""

    major: int
    minor: int
    header_size: int
    frame_size: int
    session_id: int
    config_hash32: int
    map_hash32: int


@dataclass
class RunlogFrame:
    """Parsed runlog frame."""

    time_ms: int
    seg_idx: int
    trig_idx: int
    flags: int
    pose_x: float
    pose_y: float
    pose_theta: float
    fused_x: float
    fused_y: float
    fused_theta: float
    target_x: float
    target_y: float
    target_theta: float
    alpha: float
    confidence: float
    slip: float
    late_ms: float
    particle_count: float
    neff: float
    ess_ratio: float
    peakedness: float


class RunlogSchemaError(RuntimeError):
    """Raised when runlog schema is incompatible."""


def parse_runlog(path: str) -> Tuple[RunlogHeader, List[RunlogFrame]]:
    """Parse runlog binary file."""
    with open(path, "rb") as f:
        raw = f.read()
    if len(raw) < HEADER_STRUCT.size:
        raise RunlogSchemaError("Runlog is too small to contain a valid header.")

    head = HEADER_STRUCT.unpack_from(raw, 0)
    magic = head[0]
    if magic != RUNLOG_MAGIC:
        raise RunlogSchemaError("Invalid runlog magic.")

    major, minor, header_size, frame_size = head[1], head[2], head[3], head[4]
    if major != RUNLOG_SCHEMA_MAJOR:
        raise RunlogSchemaError(
            f"Incompatible runlog schema major {major}; expected {RUNLOG_SCHEMA_MAJOR}."
        )
    if frame_size < FRAME_STRUCT.size:
        raise RunlogSchemaError(
            f"Unexpected frame size {frame_size}; expected at least {FRAME_STRUCT.size}."
        )
    if header_size < HEADER_STRUCT.size:
        raise RunlogSchemaError("Invalid header size in runlog.")

    header = RunlogHeader(
        major=major,
        minor=minor,
        header_size=header_size,
        frame_size=frame_size,
        session_id=head[5],
        config_hash32=head[6],
        map_hash32=head[7],
    )

    frames: List[RunlogFrame] = []
    off = header_size
    limit = len(raw)
    while off + frame_size <= limit:
        vals = FRAME_STRUCT.unpack_from(raw, off)
        off += frame_size
        frames.append(
            RunlogFrame(
                time_ms=int(vals[0]),
                seg_idx=int(vals[1]),
                trig_idx=int(vals[2]),
                flags=int(vals[3]),
                pose_x=float(vals[4]),
                pose_y=float(vals[5]),
                pose_theta=float(vals[6]),
                fused_x=float(vals[7]),
                fused_y=float(vals[8]),
                fused_theta=float(vals[9]),
                target_x=float(vals[10]),
                target_y=float(vals[11]),
                target_theta=float(vals[12]),
                alpha=float(vals[13]),
                confidence=float(vals[14]),
                slip=float(vals[15]),
                late_ms=float(vals[16]),
                particle_count=float(vals[17]),
                neff=float(vals[18]),
                ess_ratio=float(vals[19]),
                peakedness=float(vals[20]),
            )
        )

    return header, frames


def runlog_summary(path: str) -> Dict[str, float]:
    """Return lightweight summary for UI diagnostics."""
    header, frames = parse_runlog(path)
    if not frames:
        return {
            "schema_major": float(header.major),
            "schema_minor": float(header.minor),
            "frames": 0.0,
            "duration_s": 0.0,
            "avg_conf": 0.0,
            "avg_peakedness": 0.0,
            "avg_ess_ratio": 0.0,
        }

    t0 = frames[0].time_ms
    t1 = frames[-1].time_ms
    n = float(len(frames))
    avg_conf = sum(f.confidence for f in frames) / n
    avg_peak = sum(f.peakedness for f in frames) / n
    avg_ess = sum(f.ess_ratio for f in frames) / n

    return {
        "schema_major": float(header.major),
        "schema_minor": float(header.minor),
        "frames": n,
        "duration_s": max(0.0, (t1 - t0) / 1000.0),
        "avg_conf": avg_conf,
        "avg_peakedness": avg_peak,
        "avg_ess_ratio": avg_ess,
    }
