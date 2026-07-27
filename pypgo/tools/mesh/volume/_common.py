"""Shared helpers for mesh CLI tools."""

from __future__ import annotations

import argparse

import pypgo.mesh as _mesh
from pypgo.mesh.volume import ENuMaterial, VegFile, VolumeMesh, write_veg


def add_material_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--E", type=float, default=1e6, help="Young's modulus")
    parser.add_argument("--nu", type=float, default=0.45, help="Poisson ratio")
    parser.add_argument("--density", type=float, default=1000.0, help="material density")


def material_from_args(args) -> ENuMaterial:
    return ENuMaterial(density=float(args.density), E=float(args.E), nu=float(args.nu))


def write_volume_outputs(mesh_data, args) -> None:
    veg = VegFile.from_single_material(mesh_data, material_from_args(args))
    write_veg(args.output_veg, veg)
    if args.output_surface:
        volume = VolumeMesh(veg)
        _mesh.write_obj(args.output_surface, volume.extract_surface_mesh())
