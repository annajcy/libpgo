#!/usr/bin/env python3
"""Generate reproducible tet and cubic simulation assets from source OBJs.

The source OBJ files are the canonical assets. Generated VEG files and their
manifest are local build artifacts and are intentionally ignored by Git.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable

import numpy as np

ASSET_DIR = Path(__file__).resolve().parent
REPO_ROOT = ASSET_DIR.parents[1]
sys.path.insert(0, str(REPO_ROOT))

import pypgo as pgo  # noqa: E402
from pypgo.mesh.volume import ENuMaterial, VegFile, read_veg, write_veg  # noqa: E402


DEFAULT_OUTPUT_ROOT = ASSET_DIR / "veg"
SOURCE_OBJS = {
    "box": ASSET_DIR / "obj" / "box.obj",
    "box-with-sphere": ASSET_DIR / "obj" / "box-with-sphere.obj",
    "bunny": ASSET_DIR / "obj" / "bunny.obj",
    "dragon": ASSET_DIR / "obj" / "dragon.obj",
}


@dataclass(frozen=True)
class GeneratedAsset:
    name: str
    kind: str
    source_obj: str
    source_sha256: str
    output_veg: str
    parameters: dict
    num_vertices: int
    num_elements: int
    num_element_vertices: int
    total_volume: float


def _surface_volume(surface: pgo.mesh.TriMeshData) -> float:
    vertices = np.asarray(surface.vertices, dtype=np.float64)
    triangles = vertices[np.asarray(surface.elements, dtype=np.int64)]
    signed_six_volume = np.einsum(
        "ij,ij->",
        triangles[:, 0],
        np.cross(triangles[:, 1], triangles[:, 2]),
    )
    return float(abs(signed_six_volume) / 6.0)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _check_surface(surface, source: Path, *, strict: bool) -> None:
    report = pgo.mesh.check_surface_quality(surface)
    if report.is_clean:
        print(f"[surface] {source.name}: clean")
        return

    message = (
        f"{source} has surface-quality findings: "
        f"degenerate={len(report.degenerate_tris)}, "
        f"nonmanifold={len(report.non_manifold_edges)}, "
        f"flipped={len(report.flipped_tris)}, "
        f"self_intersections={report.has_self_intersections}"
    )
    if strict:
        raise RuntimeError(message)
    print(f"[surface] WARNING: {message}")


def _atomic_write_veg(path: Path, mesh, material: ENuMaterial) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    try:
        write_veg(
            str(temporary),
            VegFile.from_single_material(mesh, material),
        )
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)


def _asset_record(
    *,
    name: str,
    kind: str,
    source: Path,
    output: Path,
    parameters: dict,
) -> GeneratedAsset:
    info = pgo.mesh.volume_mesh_info(read_veg(str(output)))
    return GeneratedAsset(
        name=name,
        kind=kind,
        source_obj=str(source),
        source_sha256=_sha256(source),
        output_veg=str(output),
        parameters=parameters,
        num_vertices=info.num_vertices,
        num_elements=info.num_elements,
        num_element_vertices=info.num_element_vertices,
        total_volume=info.total_volume,
    )


def _generate_cubic(
    *,
    name: str,
    source: Path,
    surface,
    output_root: Path,
    resolution: int,
    occupancy: str,
    material: ENuMaterial,
) -> GeneratedAsset:
    output = output_root / "cubic" / f"{name}.veg"
    print(
        f"[cubic] {name}: resolution={resolution}, "
        f"occupancy={occupancy} -> {output}"
    )
    mesh = pgo.mesh.cubic_mesher(
        surface,
        resolution=resolution,
        occupancy=occupancy,
        E=material.E,
        nu=material.nu,
        density=material.density,
    )
    _atomic_write_veg(output, mesh, material)
    return _asset_record(
        name=name,
        kind="cubic",
        source=source,
        output=output,
        parameters={
            "resolution": resolution,
            "occupancy": occupancy,
        },
    )


def _generate_tet(
    *,
    name: str,
    source: Path,
    surface,
    output_root: Path,
    backend: str,
    target_elements: int,
    tetwild_lr: float,
    material: ENuMaterial,
    output_name: str | None = None,
) -> GeneratedAsset:
    output_name = output_name or name
    output = output_root / "tet" / f"{output_name}.veg"

    if backend == "tetgen":
        volume = _surface_volume(surface)
        if volume <= 0.0:
            raise RuntimeError(f"{source} has non-positive enclosed volume")
        max_tet_volume = volume / target_elements
        command = f"pq1.414a{max_tet_volume:.17g}"
        config = {"command": command}
        parameters = {
            "backend": backend,
            "target_elements": target_elements,
            "surface_volume": volume,
            "max_tet_volume": max_tet_volume,
            "command": command,
        }
    else:
        config = {
            "lr": tetwild_lr,
            "epsr": 0.001,
            "stop_energy": 10.0,
            "max_threads": 0,
        }
        parameters = {
            "backend": backend,
            **config,
        }

    print(f"[tet] {output_name}: {parameters} -> {output}")
    mesh = pgo.mesh.tet_mesher(surface, backend=backend, config=config)
    _atomic_write_veg(output, mesh, material)
    return _asset_record(
        name=output_name,
        kind="tet",
        source=source,
        output=output,
        parameters=parameters,
    )


def _write_manifest(path: Path, records: Iterable[GeneratedAsset], args) -> None:
    payload = {
        "schema_version": 1,
        "generator": str(Path(__file__).resolve()),
        "settings": {
            "assets": args.assets,
            "kind": args.kind,
            "cubic_resolution": args.cubic_resolution,
            "cubic_occupancy": args.cubic_occupancy,
            "tet_backend": args.tet_backend,
            "tet_target_elements": args.tet_target_elements,
            "static_dragon_target_elements": args.static_dragon_target_elements,
            "tetwild_lr": args.tetwild_lr,
            "static_dragon_tetwild_lr": args.static_dragon_tetwild_lr,
            "max_parallelism": args.max_parallelism,
            "E": args.E,
            "nu": args.nu,
            "density": args.density,
        },
        "assets": [asdict(record) for record in records],
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    try:
        temporary.write_text(json.dumps(payload, indent=2) + "\n")
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--assets",
        nargs="+",
        choices=(*SOURCE_OBJS, "all"),
        default=["all"],
        help="source assets to generate (default: all)",
    )
    parser.add_argument(
        "--kind",
        choices=("both", "cubic", "tet"),
        default="both",
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=DEFAULT_OUTPUT_ROOT,
    )
    parser.add_argument("--cubic-resolution", type=int, default=12)
    parser.add_argument(
        "--cubic-occupancy",
        choices=("conservative", "center"),
        default="conservative",
    )
    parser.add_argument(
        "--tet-backend",
        choices=("tetgen", "tetwild"),
        default="tetwild",
    )
    parser.add_argument("--tet-target-elements", type=int, default=10_000)
    parser.add_argument(
        "--static-dragon-target-elements",
        type=int,
        default=50_000,
        help="TetGen target for tet/dragon_big.veg; 0 disables it",
    )
    parser.add_argument("--tetwild-lr", type=float, default=0.05)
    parser.add_argument(
        "--static-dragon-tetwild-lr",
        type=float,
        default=0.025,
        help="relative edge length for tet/dragon_big.veg; 0 disables it",
    )
    parser.add_argument("--E", type=float, default=1e6)
    parser.add_argument("--nu", type=float, default=0.45)
    parser.add_argument("--density", type=float, default=1000.0)
    parser.add_argument(
        "--max-parallelism",
        type=int,
        default=0,
        help="pypgo process-wide parallelism limit; 0 uses the runtime default",
    )
    parser.add_argument(
        "--strict-surface-quality",
        action="store_true",
        help="fail instead of warning when surface-quality findings are present",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    if args.cubic_resolution < 1:
        raise ValueError("--cubic-resolution must be positive")
    if args.tet_target_elements < 1:
        raise ValueError("--tet-target-elements must be positive")
    if args.static_dragon_target_elements < 0:
        raise ValueError("--static-dragon-target-elements must be non-negative")
    if args.tetwild_lr <= 0.0:
        raise ValueError("--tetwild-lr must be positive")
    if args.static_dragon_tetwild_lr < 0.0:
        raise ValueError("--static-dragon-tetwild-lr must be non-negative")
    if args.max_parallelism < 0:
        raise ValueError("--max-parallelism must be non-negative")
    if args.tet_backend == "tetwild" and not pgo.mesh.has_tetwild():
        raise RuntimeError("this pypgo build does not include the tetwild backend")

    if args.max_parallelism > 0:
        with pgo.parallel.GlobalTbbControl(args.max_parallelism):
            return _generate_requested_assets(args)
    return _generate_requested_assets(args)


def _generate_requested_assets(args) -> int:
    requested = list(SOURCE_OBJS) if "all" in args.assets else args.assets
    material = ENuMaterial(E=args.E, nu=args.nu, density=args.density)
    records: list[GeneratedAsset] = []

    for name in requested:
        source = SOURCE_OBJS[name]
        if not source.is_file():
            raise FileNotFoundError(source)
        surface = pgo.mesh.read_obj(str(source))
        _check_surface(surface, source, strict=args.strict_surface_quality)

        if args.kind in ("both", "cubic"):
            records.append(
                _generate_cubic(
                    name=name,
                    source=source,
                    surface=surface,
                    output_root=args.output_root,
                    resolution=args.cubic_resolution,
                    occupancy=args.cubic_occupancy,
                    material=material,
                )
            )

        if args.kind in ("both", "tet"):
            records.append(
                _generate_tet(
                    name=name,
                    source=source,
                    surface=surface,
                    output_root=args.output_root,
                    backend=args.tet_backend,
                    target_elements=args.tet_target_elements,
                    tetwild_lr=args.tetwild_lr,
                    material=material,
                )
            )

            generate_static_dragon = (
                name == "dragon"
                and (
                    (
                        args.tet_backend == "tetgen"
                        and args.static_dragon_target_elements > 0
                    )
                    or (
                        args.tet_backend == "tetwild"
                        and args.static_dragon_tetwild_lr > 0.0
                    )
                )
            )
            if generate_static_dragon:
                records.append(
                    _generate_tet(
                        name=name,
                        output_name="dragon_big",
                        source=source,
                        surface=surface,
                        output_root=args.output_root,
                        backend=args.tet_backend,
                        target_elements=args.static_dragon_target_elements,
                        tetwild_lr=args.static_dragon_tetwild_lr,
                        material=material,
                    )
                )

    manifest = args.output_root / "manifest.json"
    _write_manifest(manifest, records, args)
    print(f"wrote manifest: {manifest}")
    for record in records:
        print(
            f"{record.kind:5s} {record.name:20s} "
            f"vertices={record.num_vertices:8d} "
            f"elements={record.num_elements:8d}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
