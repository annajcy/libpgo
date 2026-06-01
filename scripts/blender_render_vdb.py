#!/usr/bin/env python3
"""Blender-side VDB stress field renderer used by render_vdb_preview.py.

Run via render_vdb_preview.py, not directly.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import bpy
from mathutils import Vector


def parse_vec(text: str, length: int) -> tuple[float, ...]:
    values = tuple(float(part) for part in text.split(","))
    if len(values) != length:
        raise ValueError(f"expected {length} comma-separated values, got {text}")
    return values


def parse_args() -> argparse.Namespace:
    argv = sys.argv
    argv = argv[argv.index("--") + 1:] if "--" in argv else []
    parser = argparse.ArgumentParser(description="Render a VDB stress field sequence to PNG frames.")
    parser.add_argument("--vdb", type=Path, required=True,
                        help="VDB path with #### frame pattern, e.g. /out/vonMises####.vdb")
    parser.add_argument("--out", type=Path, required=True,
                        help="Output directory for rendered PNG frames")
    parser.add_argument("--frame-start", type=int, required=True)
    parser.add_argument("--frame-end",   type=int, required=True)
    parser.add_argument("--frame-step",  type=int, default=1)
    parser.add_argument("--fps",         type=int, default=24)
    parser.add_argument("--width",       type=int, default=960)
    parser.add_argument("--height",      type=int, default=540)
    parser.add_argument("--samples",     type=int, default=128,
                        help="Cycles render samples (volumes need more than meshes)")
    parser.add_argument("--background",  default="0.05,0.05,0.05",
                        help="World background RGB, default near-black")
    parser.add_argument("--grid-name",   default="von_mises",
                        help="Name of the scalar VDB grid to visualise")
    parser.add_argument("--low-color",   default="0,0,1",
                        help="Emission color for minimum stress value (RGB)")
    parser.add_argument("--high-color",  default="1,0,0",
                        help="Emission color for maximum stress value (RGB)")
    parser.add_argument("--emission-strength", type=float, default=5.0)
    parser.add_argument("--density",     type=float, default=0.1,
                        help="Volume density (controls opacity)")
    parser.add_argument("--camera-view", default="0,-1,0.35",
                        help="Camera look-from direction XYZ (normalised internally)")
    parser.add_argument("--ortho-scale-multiplier", type=float, default=2.4)
    args = parser.parse_args(argv)
    args.background = parse_vec(args.background, 3)
    args.low_color  = parse_vec(args.low_color,  3)
    args.high_color = parse_vec(args.high_color, 3)
    args.camera_view = parse_vec(args.camera_view, 3)
    return args


def clear_scene() -> None:
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()


def import_vdb_sequence(vdb_path: Path, frame_start: int, frame_end: int) -> bpy.types.Object:
    """Import the first VDB frame, then configure the object as an animated sequence."""
    first = str(vdb_path).replace("####", f"{frame_start:04d}")
    bpy.ops.object.volume_import(filepath=first, files=[])
    vol_obj = bpy.context.active_object
    if vol_obj is None or vol_obj.type != "VOLUME":
        raise RuntimeError(f"VDB import did not produce a Volume object (got {vol_obj})")

    vol = vol_obj.data
    vol.filepath = str(vdb_path)
    vol.is_sequence = True
    vol.frame_start = frame_start
    vol.frame_duration = frame_end - frame_start + 1
    vol.frame_offset = 0
    return vol_obj


def setup_volume_material(
    vol_obj: bpy.types.Object,
    grid_name: str,
    low_color: tuple[float, float, float],
    high_color: tuple[float, float, float],
    emission_strength: float,
    density: float,
) -> None:
    """Build a Principled Volume shader driven by the named VDB scalar grid."""
    mat = bpy.data.materials.new("StressMaterial")
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    links = mat.node_tree.links
    nodes.clear()

    # Reads the named VDB grid; Fac output carries the scalar value [0, 1]
    attr = nodes.new("ShaderNodeAttribute")
    attr.attribute_name = grid_name
    attr.attribute_type = "GEOMETRY"
    attr.location = (-600, 0)

    # Blue (low) → red (high) colormap
    ramp = nodes.new("ShaderNodeValToRGB")
    ramp.location = (-300, 0)
    ramp.color_ramp.interpolation = "LINEAR"
    ramp.color_ramp.elements[0].position = 0.0
    ramp.color_ramp.elements[0].color = (*low_color, 1.0)
    ramp.color_ramp.elements[1].position = 1.0
    ramp.color_ramp.elements[1].color = (*high_color, 1.0)

    vol_shader = nodes.new("ShaderNodeVolumePrincipled")
    vol_shader.location = (0, 0)
    vol_shader.inputs["Emission Strength"].default_value = emission_strength
    vol_shader.inputs["Density"].default_value = density

    output = nodes.new("ShaderNodeOutputMaterial")
    output.location = (300, 0)

    links.new(attr.outputs["Fac"],       ramp.inputs["Fac"])
    links.new(ramp.outputs["Color"],     vol_shader.inputs["Emission Color"])
    links.new(vol_shader.outputs["Volume"], output.inputs["Volume"])

    vol_obj.data.materials.clear()
    vol_obj.data.materials.append(mat)


def volume_bounds(vol_obj: bpy.types.Object) -> tuple[Vector, float]:
    corners = [vol_obj.matrix_world @ Vector(c) for c in vol_obj.bound_box]
    mins = Vector((min(c.x for c in corners), min(c.y for c in corners), min(c.z for c in corners)))
    maxs = Vector((max(c.x for c in corners), max(c.y for c in corners), max(c.z for c in corners)))
    center = (mins + maxs) * 0.5
    radius = max((maxs - mins).length * 0.5, 1e-6)
    return center, radius


def setup_camera(center: Vector, radius: float, view: tuple[float, float, float], scale_multiplier: float) -> None:
    view_vector = Vector(view)
    if view_vector.length == 0:
        raise ValueError("camera view vector must be non-zero")
    view_vector.normalize()

    camera_data = bpy.data.cameras.new("Camera")
    camera = bpy.data.objects.new("Camera", camera_data)
    bpy.context.collection.objects.link(camera)
    camera.location = center + view_vector * (3.0 * radius)
    direction = center - camera.location
    camera.rotation_euler = direction.to_track_quat("-Z", "Y").to_euler()
    camera_data.type = "ORTHO"
    camera_data.ortho_scale = radius * scale_multiplier
    bpy.context.scene.camera = camera


def setup_render(args: argparse.Namespace) -> None:
    scene = bpy.context.scene
    scene.frame_start = args.frame_start
    scene.frame_end   = args.frame_end
    scene.frame_step  = args.frame_step
    scene.render.fps  = args.fps
    scene.render.resolution_x = args.width
    scene.render.resolution_y = args.height
    scene.render.film_transparent = False
    scene.render.image_settings.file_format = "PNG"
    scene.render.filepath = str(args.out / "frame_")

    # Cycles is required for accurate volumetric rendering
    scene.render.engine = "CYCLES"
    scene.cycles.samples = args.samples
    scene.cycles.use_denoising = True

    world = scene.world or bpy.data.worlds.new("World")
    scene.world = world
    world.use_nodes = True
    bg = world.node_tree.nodes.get("Background")
    if bg:
        bg.inputs["Color"].default_value = (*args.background, 1.0)
        bg.inputs["Strength"].default_value = 0.3


def main() -> None:
    args = parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    clear_scene()
    vol_obj = import_vdb_sequence(args.vdb, args.frame_start, args.frame_end)
    setup_volume_material(
        vol_obj,
        args.grid_name,
        args.low_color,
        args.high_color,
        args.emission_strength,
        args.density,
    )
    center, radius = volume_bounds(vol_obj)
    setup_camera(center, radius, args.camera_view, args.ortho_scale_multiplier)
    setup_render(args)
    bpy.ops.render.render(animation=True)


if __name__ == "__main__":
    main()
