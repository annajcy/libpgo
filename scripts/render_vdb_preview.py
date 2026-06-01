#!/usr/bin/env python3
"""Render a VDB stress field animation preview with Blender and encode it as a GIF.

Config JSON format
------------------
{
  "vdb":              "relative/or/absolute/vonMises####.vdb",
  "frames_dir":       "relative/or/absolute/frames/",
  "output_gif":       "relative/or/absolute/stress.gif",
  "frame_start":      0,
  "frame_end":        23,
  "frame_step":       1,
  "fps":              24,
  "gif_fps":          12,
  "width":            960,
  "height":           540,
  "samples":          128,
  "background":       [0.05, 0.05, 0.05],
  "grid_name":        "von_mises",
  "low_color":        [0.0, 0.0, 1.0],
  "high_color":       [1.0, 0.0, 0.0],
  "emission_strength": 5.0,
  "density":          0.1,
  "camera": {
    "view":                  [0.0, -1.0, 0.35],
    "ortho_scale_multiplier": 2.4
  }
}
"""

from __future__ import annotations

import argparse
import json
import os
import shlex
import shutil
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
BLENDER_RENDER_SCRIPT = REPO_ROOT / "scripts" / "blender_render_vdb.py"
DEFAULT_BLENDER_MACOS = Path("/Applications/Blender.app/Contents/MacOS/Blender")


@dataclass(frozen=True)
class CameraConfig:
    view: tuple[float, float, float]
    ortho_scale_multiplier: float


@dataclass(frozen=True)
class RenderConfig:
    config_path: Path
    vdb: Path
    frames_dir: Path
    output_gif: Path
    frame_start: int
    frame_end: int
    frame_step: int
    fps: int
    gif_fps: int
    width: int
    height: int
    samples: int
    background: tuple[float, float, float]
    grid_name: str
    low_color: tuple[float, float, float]
    high_color: tuple[float, float, float]
    emission_strength: float
    density: float
    camera: CameraConfig


@dataclass(frozen=True)
class CommandSpec:
    label: str
    argv: list[str]


def require_mapping(value: Any, context: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{context} must be an object")
    return value


def require_string(value: Any, context: str) -> str:
    if not isinstance(value, str) or not value:
        raise ValueError(f"{context} must be a non-empty string")
    return value


def require_int(value: Any, context: str) -> int:
    if not isinstance(value, int):
        raise ValueError(f"{context} must be an integer")
    return value


def require_number(value: Any, context: str) -> float:
    if not isinstance(value, (int, float)):
        raise ValueError(f"{context} must be a number")
    return float(value)


def require_number_list(value: Any, context: str, length: int) -> tuple[float, ...]:
    if not isinstance(value, list) or len(value) != length:
        raise ValueError(f"{context} must be an array of {length} numbers")
    out: list[float] = []
    for index, item in enumerate(value):
        if not isinstance(item, (int, float)):
            raise ValueError(f"{context}[{index}] must be a number")
        out.append(float(item))
    return tuple(out)


def resolve_config_path(path_text: str, config_path: Path) -> Path:
    path = Path(path_text)
    return (path if path.is_absolute() else config_path.parent / path).resolve()


def load_config(config_path: Path) -> RenderConfig:
    with config_path.open("r", encoding="utf-8") as fin:
        config = require_mapping(json.load(fin), str(config_path))

    camera_cfg = require_mapping(config.get("camera", {}), f"{config_path}.camera")
    camera = CameraConfig(
        view=require_number_list(
            camera_cfg.get("view", [0.0, -1.0, 0.35]),
            f"{config_path}.camera.view", 3,
        ),
        ortho_scale_multiplier=float(camera_cfg.get("ortho_scale_multiplier", 2.4)),
    )

    rc = RenderConfig(
        config_path=config_path,
        vdb=resolve_config_path(
            require_string(config.get("vdb"), f"{config_path}.vdb"), config_path
        ),
        frames_dir=resolve_config_path(
            require_string(config.get("frames_dir"), f"{config_path}.frames_dir"), config_path
        ),
        output_gif=resolve_config_path(
            require_string(config.get("output_gif"), f"{config_path}.output_gif"), config_path
        ),
        frame_start=require_int(config.get("frame_start", 0),   f"{config_path}.frame_start"),
        frame_end=  require_int(config.get("frame_end",   23),  f"{config_path}.frame_end"),
        frame_step= require_int(config.get("frame_step",   1),  f"{config_path}.frame_step"),
        fps=        require_int(config.get("fps",          24), f"{config_path}.fps"),
        gif_fps=    require_int(config.get("gif_fps",      12), f"{config_path}.gif_fps"),
        width=      require_int(config.get("width",       960), f"{config_path}.width"),
        height=     require_int(config.get("height",      540), f"{config_path}.height"),
        samples=    require_int(config.get("samples",     128), f"{config_path}.samples"),
        background= require_number_list(
            config.get("background", [0.05, 0.05, 0.05]),
            f"{config_path}.background", 3,
        ),
        grid_name=require_string(config.get("grid_name", "von_mises"), f"{config_path}.grid_name"),
        low_color= require_number_list(
            config.get("low_color",  [0.0, 0.0, 1.0]),
            f"{config_path}.low_color", 3,
        ),
        high_color=require_number_list(
            config.get("high_color", [1.0, 0.0, 0.0]),
            f"{config_path}.high_color", 3,
        ),
        emission_strength=require_number(
            config.get("emission_strength", 5.0), f"{config_path}.emission_strength"
        ),
        density=require_number(config.get("density", 0.1), f"{config_path}.density"),
        camera=camera,
    )

    if rc.frame_start > rc.frame_end:
        raise ValueError(f"{config_path}.frame_start must be <= frame_end")
    if rc.frame_step <= 0:
        raise ValueError(f"{config_path}.frame_step must be positive")
    if rc.fps <= 0 or rc.gif_fps <= 0:
        raise ValueError(f"{config_path}: fps and gif_fps must be positive")
    if rc.width <= 0 or rc.height <= 0:
        raise ValueError(f"{config_path}: width and height must be positive")
    if rc.samples <= 0:
        raise ValueError(f"{config_path}.samples must be positive")
    if rc.density <= 0:
        raise ValueError(f"{config_path}.density must be positive")
    return rc


def default_blender_path() -> Path | None:
    env_value = os.environ.get("BLENDER")
    if env_value:
        return Path(env_value)
    found = shutil.which("blender")
    if found:
        return Path(found)
    if DEFAULT_BLENDER_MACOS.exists():
        return DEFAULT_BLENDER_MACOS
    return None


def default_ffmpeg_path() -> Path | None:
    env_value = os.environ.get("FFMPEG")
    if env_value:
        return Path(env_value)
    found = shutil.which("ffmpeg")
    return Path(found) if found else None


def build_commands(config: RenderConfig, blender: Path, ffmpeg: Path) -> list[CommandSpec]:
    frame_pattern = str(config.frames_dir / "frame_*.png")
    palette_path = config.frames_dir / "palette.png"

    blender_cmd = [
        str(blender), "-b",
        "--python", str(BLENDER_RENDER_SCRIPT),
        "--",
        "--vdb",          str(config.vdb),
        "--out",          str(config.frames_dir),
        "--frame-start",  str(config.frame_start),
        "--frame-end",    str(config.frame_end),
        "--frame-step",   str(config.frame_step),
        "--fps",          str(config.fps),
        "--width",        str(config.width),
        "--height",       str(config.height),
        "--samples",      str(config.samples),
        "--background",   ",".join(f"{v:g}" for v in config.background),
        "--grid-name",    config.grid_name,
        "--low-color",    ",".join(f"{v:g}" for v in config.low_color),
        "--high-color",   ",".join(f"{v:g}" for v in config.high_color),
        "--emission-strength", f"{config.emission_strength:g}",
        "--density",      f"{config.density:g}",
        "--camera-view",  ",".join(f"{v:g}" for v in config.camera.view),
        "--ortho-scale-multiplier", f"{config.camera.ortho_scale_multiplier:g}",
    ]
    palette_cmd = [
        str(ffmpeg), "-v", "error", "-y",
        "-framerate", str(config.gif_fps),
        "-pattern_type", "glob", "-i", frame_pattern,
        "-vf", "palettegen=stats_mode=diff",
        str(palette_path),
    ]
    gif_cmd = [
        str(ffmpeg), "-v", "error", "-y",
        "-framerate", str(config.gif_fps),
        "-pattern_type", "glob", "-i", frame_pattern,
        "-i", str(palette_path),
        "-lavfi", "paletteuse=dither=bayer:bayer_scale=5:diff_mode=rectangle",
        "-loop", "0",
        str(config.output_gif),
    ]
    return [
        CommandSpec("blender", blender_cmd),
        CommandSpec("palette", palette_cmd),
        CommandSpec("gif",     gif_cmd),
    ]


def check_inputs(config: RenderConfig, blender: Path, ffmpeg: Path) -> None:
    vdb_first = str(config.vdb).replace("####", f"{config.frame_start:04d}")
    if not Path(vdb_first).exists():
        raise FileNotFoundError(f"first VDB frame not found: {vdb_first}")
    if not BLENDER_RENDER_SCRIPT.exists():
        raise FileNotFoundError(BLENDER_RENDER_SCRIPT)
    if not blender.exists():
        raise FileNotFoundError(blender)
    if not ffmpeg.exists():
        raise FileNotFoundError(ffmpeg)


def prepare_outputs(config: RenderConfig, overwrite: bool) -> None:
    if config.output_gif.exists():
        if not overwrite:
            raise FileExistsError(f"output already exists: {config.output_gif}; pass --overwrite")
        config.output_gif.unlink()
    if config.frames_dir.exists():
        if not overwrite and any(config.frames_dir.iterdir()):
            raise FileExistsError(
                f"frames_dir already exists and is not empty: {config.frames_dir}; pass --overwrite"
            )
        shutil.rmtree(config.frames_dir)
    config.frames_dir.mkdir(parents=True, exist_ok=True)
    config.output_gif.parent.mkdir(parents=True, exist_ok=True)


def run_command(command: CommandSpec, dry_run: bool) -> None:
    print(f"+ {shlex.join(command.argv)}")
    if dry_run:
        return
    subprocess.run(command.argv, check=True, cwd=REPO_ROOT)


def render_from_config(
    config_path: Path,
    *,
    blender: Path | None = None,
    ffmpeg: Path | None = None,
    overwrite: bool = False,
    dry_run: bool = False,
) -> Path:
    config = load_config(config_path)
    blender = blender or default_blender_path()
    ffmpeg  = ffmpeg  or default_ffmpeg_path()
    if blender is None:
        raise FileNotFoundError("Blender not found; pass --blender or set BLENDER env var")
    if ffmpeg is None:
        raise FileNotFoundError("ffmpeg not found; pass --ffmpeg or set FFMPEG env var")
    check_inputs(config, blender, ffmpeg)
    commands = build_commands(config, blender, ffmpeg)
    if not dry_run:
        prepare_outputs(config, overwrite)
    for command in commands:
        run_command(command, dry_run)
    return config.output_gif


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Render a VDB stress field animation preview with Blender and ffmpeg."
    )
    parser.add_argument("--config",    type=Path, required=True, help="JSON render config.")
    parser.add_argument("--blender",   type=Path, help="Blender executable. Defaults to BLENDER, PATH, macOS app.")
    parser.add_argument("--ffmpeg",    type=Path, help="ffmpeg executable. Defaults to FFMPEG, then PATH.")
    parser.add_argument("--overwrite", action="store_true", help="Replace existing GIF and frames.")
    parser.add_argument("--dry-run",   action="store_true", help="Print commands without running them.")
    args = parser.parse_args()
    args.config = args.config if args.config.is_absolute() else REPO_ROOT / args.config
    return args


def main() -> int:
    args = parse_args()
    try:
        output = render_from_config(
            args.config,
            blender=args.blender,
            ffmpeg=args.ffmpeg,
            overwrite=args.overwrite,
            dry_run=args.dry_run,
        )
        print(f"wrote {output}")
        return 0
    except (FileNotFoundError, FileExistsError, ValueError,
            json.JSONDecodeError, subprocess.CalledProcessError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
