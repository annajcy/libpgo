"""Stress field statistics utilities."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class FrameStressStats:
    frame: int
    time: float
    count: int
    min: float
    mean: float
    stddev: float
    median: float
    p99: float
    max: float


@dataclass(frozen=True)
class StressFieldStats:
    stress_type: str
    location: str
    source_dir: str
    prefix: str
    frame_start: int
    frame_end: int
    frames: list[FrameStressStats]

    @property
    def num_frames(self) -> int:
        return len(self.frames)

    def to_dict(self) -> dict:
        return {
            "stress_type": self.stress_type,
            "location": self.location,
            "source_dir": self.source_dir,
            "prefix": self.prefix,
            "frame_start": self.frame_start,
            "frame_end": self.frame_end,
            "num_frames": self.num_frames,
            "frames": [
                {
                    "frame": s.frame,
                    "time": s.time,
                    "count": s.count,
                    "min": s.min,
                    "mean": s.mean,
                    "stddev": s.stddev,
                    "median": s.median,
                    "p99": s.p99,
                    "max": s.max,
                }
                for s in self.frames
            ],
        }

    def save(self, path: str | Path) -> None:
        """Write aggregated stats to a JSON file (same format as the C++ tool)."""
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, "w") as f:
            json.dump(self.to_dict(), f, indent=2)
            f.write("\n")


def _frame_path(stress_dir: Path, prefix: str, frame: int) -> Path:
    return stress_dir / f"{prefix}{frame:04d}.json"


def _detect_frame_end(stress_dir: Path, prefix: str, frame_start: int) -> int:
    frame = frame_start
    while _frame_path(stress_dir, prefix, frame).exists():
        frame += 1
    return frame


def _compute_frame_stats(doc: dict, source: Path) -> FrameStressStats:
    values = doc.get("values")
    if not isinstance(values, list) or len(values) == 0:
        raise ValueError(f"Missing or empty 'values' array in {source}")

    v = np.asarray(values, dtype=np.float64)
    return FrameStressStats(
        frame=int(doc.get("frame", 0)),
        time=float(doc.get("time", 0.0)),
        count=len(v),
        min=float(v.min()),
        max=float(v.max()),
        mean=float(v.mean()),
        stddev=float(v.std()),          # population std, matches C++ tool
        median=float(np.quantile(v, 0.50)),
        p99=float(np.quantile(v, 0.99)),
    )


def compute_stress_field_stats(
    stress_dir: str | Path,
    *,
    prefix: str = "von_mises",
    frame_start: int = 0,
    frame_end: int = -1,
) -> StressFieldStats:
    """Aggregate per-frame statistics over a sequence of stress JSON files.

    Reads ``{stress_dir}/{prefix}{frame:04d}.json`` for each frame and
    computes min, max, mean, stddev, median, p99 — matching the output of the
    C++ ``computeStressFieldStats`` tool exactly.

    Parameters
    ----------
    stress_dir:
        Folder containing per-frame stress JSON files.
    prefix:
        Filename prefix; files are read as ``{prefix}{frame:04d}.json``.
    frame_start:
        First frame index (inclusive).
    frame_end:
        Last frame index (exclusive). ``-1`` auto-detects from files on disk.
    """
    stress_dir = Path(stress_dir)
    if not stress_dir.is_dir():
        raise FileNotFoundError(f"stress directory does not exist: {stress_dir}")

    if frame_end < 0:
        frame_end = _detect_frame_end(stress_dir, prefix, frame_start)
        print(f"Auto-detected frame range: [{frame_start}, {frame_end})")

    if frame_end <= frame_start:
        raise ValueError(f"No frames found in {stress_dir} starting at frame {frame_start}")

    stress_type = ""
    location = ""
    frame_stats: list[FrameStressStats] = []

    for frame in range(frame_start, frame_end):
        path = _frame_path(stress_dir, prefix, frame)
        with open(path) as f:
            doc = json.load(f)

        if frame == frame_start:
            stress_type = doc.get("stress_type", "")
            location = doc.get("location", "")

        frame_stats.append(_compute_frame_stats(doc, path))

    return StressFieldStats(
        stress_type=stress_type,
        location=location,
        source_dir=str(stress_dir.resolve()),
        prefix=prefix,
        frame_start=frame_start,
        frame_end=frame_end,
        frames=frame_stats,
    )
