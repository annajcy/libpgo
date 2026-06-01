"""Animation sequence loading and Alembic export pipeline.

Replicates the convertAnimation CLI tool in pure Python + pypgo bindings.

Sequence types
--------------
"objmesh"  : per-frame .obj files; displacement = frame_positions - rest_positions
"u"        : per-frame Eigen binary files (.u); each stores a (3*n_verts, 1) column vector

Scale convention  (matches C++ tool's  "scale": "embedded,embedding,displacement")
-----------------
embedded_scale      : applied to display mesh vertices at load time
embedding_scale     : applied to driving mesh vertices at load time
displacement_scale  : multiplied into all per-frame displacements after loading
"""

from __future__ import annotations

import json
import struct
import warnings
from dataclasses import dataclass, field
from pathlib import Path
from typing import Literal

import numpy as np

import pypgo._core as _core
from pypgo.mesh import TriMeshData, read_obj
from pypgo.mesh.geo import BarycentricEmbedding
from pypgo.mesh.veg import VolumeMesh, read_veg


# ---------------------------------------------------------------------------
# Eigen binary matrix I/O
# ---------------------------------------------------------------------------

def read_u_file(path: str | Path) -> np.ndarray:
    """Read an Eigen binary matrix .u file.

    Returns a (nrows, ncols) float64 array (column-major, matching Eigen default).
    """
    with open(path, "rb") as f:
        nrows, ncols, entry_size = struct.unpack("iii", f.read(12))
        dtype = np.float64 if entry_size == 8 else np.float32
        data = np.frombuffer(f.read(nrows * ncols * entry_size), dtype=dtype)
    return np.ascontiguousarray(data.reshape((nrows, ncols), order="F"))


def write_u_file(path: str | Path, mat: np.ndarray) -> None:
    """Write a 2-D array as an Eigen binary matrix .u file (float64, column-major)."""
    mat = np.asfortranarray(mat, dtype=np.float64)
    if mat.ndim == 1:
        mat = mat[:, None]
    nrows, ncols = mat.shape
    with open(path, "wb") as f:
        f.write(struct.pack("iii", nrows, ncols, 8))
        f.write(mat.tobytes(order="F"))


# ---------------------------------------------------------------------------
# Surface closest-point barycentric (for tri-driving + separate display mesh)
# ---------------------------------------------------------------------------

def _closest_point_on_triangle(
    p: np.ndarray, a: np.ndarray, b: np.ndarray, c: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """Project point p onto triangle (a, b, c).

    Returns (closest_point, barycentric_weights [wa, wb, wc]).
    Implements Ericson's Real-Time Collision Detection §5.1.5.
    """
    ab, ac, ap = b - a, c - a, p - a
    d1, d2 = float(ab @ ap), float(ac @ ap)
    if d1 <= 0 and d2 <= 0:
        return a, np.array([1.0, 0.0, 0.0])

    bp = p - b
    d3, d4 = float(ab @ bp), float(ac @ bp)
    if d3 >= 0 and d4 <= d3:
        return b, np.array([0.0, 1.0, 0.0])

    cp = p - c
    d5, d6 = float(ab @ cp), float(ac @ cp)
    if d6 >= 0 and d5 <= d6:
        return c, np.array([0.0, 0.0, 1.0])

    vc = d1 * d4 - d3 * d2
    if vc <= 0 and d1 >= 0 and d3 <= 0:
        v = d1 / (d1 - d3)
        return a + v * ab, np.array([1.0 - v, v, 0.0])

    vb = d5 * d2 - d1 * d6
    if vb <= 0 and d2 >= 0 and d6 <= 0:
        w = d2 / (d2 - d6)
        return a + w * ac, np.array([1.0 - w, 0.0, w])

    va = d3 * d6 - d5 * d4
    if va <= 0 and (d4 - d3) >= 0 and (d5 - d6) >= 0:
        w = (d4 - d3) / ((d4 - d3) + (d5 - d6))
        return b + w * (c - b), np.array([0.0, 1.0 - w, w])

    denom = 1.0 / (va + vb + vc)
    v = vb * denom
    w = vc * denom
    return a + ab * v + ac * w, np.array([1.0 - v - w, v, w])


def _surface_embedding(
    display_verts: np.ndarray,
    driving_verts: np.ndarray,
    driving_tris: np.ndarray,
    k_neighbors: int = 8,
) -> tuple[np.ndarray, np.ndarray]:
    """For each display vertex find the closest point on the driving surface.

    Returns (tri_indices, bary_weights) arrays of shape (n_display,) and
    (n_display, 3) respectively.
    Requires scipy (pip install scipy).
    """
    try:
        from scipy.spatial import cKDTree
    except ImportError as e:
        raise ImportError(
            "Surface-to-surface embedding requires scipy: pip install scipy"
        ) from e

    tree = cKDTree(driving_verts)
    k = min(k_neighbors, len(driving_verts))
    _, close_vtx = tree.query(display_verts, k=k)  # (n_disp, k)

    # vertex → incident triangle index list
    vtx_to_tris: list[list[int]] = [[] for _ in range(len(driving_verts))]
    for ti, tri in enumerate(driving_tris):
        for v in tri:
            vtx_to_tris[v].append(ti)

    n_display = len(display_verts)
    tri_indices = np.zeros(n_display, dtype=np.int64)
    bary_weights = np.zeros((n_display, 3))

    for di, p in enumerate(display_verts):
        best_dist2 = np.inf
        best_tri = 0
        best_bary = np.array([1.0, 0.0, 0.0])

        candidates: set[int] = set()
        neighbors = close_vtx[di] if close_vtx.ndim == 2 else [close_vtx[di]]
        for vtx in neighbors:
            candidates.update(vtx_to_tris[vtx])

        for ti in candidates:
            a, b, c = (driving_verts[driving_tris[ti, j]] for j in range(3))
            proj, bary = _closest_point_on_triangle(p, a, b, c)
            dist2 = float(np.sum((p - proj) ** 2))
            if dist2 < best_dist2:
                best_dist2 = dist2
                best_tri = ti
                best_bary = bary

        tri_indices[di] = best_tri
        bary_weights[di] = best_bary

    return tri_indices, bary_weights


def _apply_surface_embedding(
    tri_indices: np.ndarray,
    bary_weights: np.ndarray,
    driving_tris: np.ndarray,
    driving_disps: np.ndarray,
) -> np.ndarray:
    """Apply surface barycentric embedding to driving displacements.

    driving_disps : (3*n_driving, n_frames)
    Returns        : (3*n_display, n_frames)
    """
    n_display = len(tri_indices)
    n_frames = driving_disps.shape[1]
    display_disps = np.zeros((n_display * 3, n_frames))
    nd = driving_disps.reshape(-1, 3, n_frames)  # (n_driving, 3, n_frames)

    for di in range(n_display):
        ti = int(tri_indices[di])
        w = bary_weights[di]  # (3,)
        vi0, vi1, vi2 = driving_tris[ti]
        disp = w[0] * nd[vi0] + w[1] * nd[vi1] + w[2] * nd[vi2]  # (3, n_frames)
        display_disps[di * 3: di * 3 + 3] = disp

    return display_disps


# ---------------------------------------------------------------------------
# AnimationSequence dataclass
# ---------------------------------------------------------------------------

@dataclass
class AnimationSequence:
    """One named mesh + frame sequence to be exported as an Alembic object."""

    name: str
    driving_mesh: str
    sequence: str
    sequence_type: Literal["objmesh", "u"]
    sequence_range: tuple[int, int]
    display_mesh: str = ""
    gap: int = 1
    embedded_scale: float = 1.0
    embedding_scale: float = 1.0
    displacement_scale: float = 1.0

    @classmethod
    def from_dict(cls, d: dict, base_dir: Path) -> "AnimationSequence":
        """Build from a JSON config dict, resolving paths relative to base_dir."""
        def resolve(p: str) -> str:
            return str(base_dir / p) if p and not Path(p).is_absolute() else p

        scale_str = d.get("scale", "1,1,1")
        parts = [float(x) for x in scale_str.split(",")]
        embedded_scale, embedding_scale, displacement_scale = parts if len(parts) == 3 else (1.0, 1.0, 1.0)

        seq_range = d["sequence-range"]

        return cls(
            name=d["name"],
            driving_mesh=resolve(d["driving-mesh"]),
            display_mesh=resolve(d.get("display-mesh", "")),
            sequence=resolve(d["sequence"]),
            sequence_type=d["sequence-type"],
            sequence_range=(seq_range[0], seq_range[1]),
            gap=d.get("gap", 1),
            embedded_scale=embedded_scale,
            embedding_scale=embedding_scale,
            displacement_scale=displacement_scale,
        )


# ---------------------------------------------------------------------------
# Core pipeline functions
# ---------------------------------------------------------------------------

def _load_driving_displacements(seq: AnimationSequence) -> tuple[np.ndarray, np.ndarray]:
    """Load rest positions and per-frame displacements for the driving mesh.

    Returns (rest_positions, driving_disps):
      rest_positions : (3*n_driving,) float64
      driving_disps  : (3*n_driving, n_frames) float64
    """
    driving_path = Path(seq.driving_mesh)

    if seq.sequence_type == "objmesh":
        rest_mesh = read_obj(str(driving_path))
        rest_pos = (rest_mesh.vertices * seq.embedding_scale).ravel()

        disps: list[np.ndarray] = []
        for i in range(seq.sequence_range[0], seq.sequence_range[1], seq.gap):
            frame_path = seq.sequence.format(i)
            try:
                frame_mesh = read_obj(frame_path)
                disps.append(frame_mesh.vertices.ravel() - rest_pos)
            except Exception:
                warnings.warn(f"Skipping frame {frame_path}: failed to load")

        if not disps:
            raise RuntimeError(f"No frames loaded for sequence {seq.name!r}")
        return rest_pos, np.column_stack(disps) * seq.displacement_scale

    elif seq.sequence_type == "u":
        driving_path_str = str(driving_path)
        if driving_path_str.endswith(".veg"):
            veg = read_veg(driving_path_str)
            md = veg.mesh_data
            rest_pos = (md.vertices * seq.embedding_scale).ravel()
            n_driving = md.num_vertices
        else:
            rest_mesh = read_obj(driving_path_str)
            rest_pos = (rest_mesh.vertices * seq.embedding_scale).ravel()
            n_driving = rest_mesh.num_vertices

        expected_rows = n_driving * 3
        disps = []
        for i in range(seq.sequence_range[0], seq.sequence_range[1], seq.gap):
            u_path = seq.sequence.format(i)
            try:
                u = read_u_file(u_path)
                col = u[:, 0] if u.ndim == 2 else u
                if len(col) != expected_rows:
                    warnings.warn(
                        f"Skipping {u_path}: expected {expected_rows} rows, got {len(col)}"
                    )
                    continue
                disps.append(col)
            except Exception:
                warnings.warn(f"Skipping {u_path}: failed to load")

        if not disps:
            raise RuntimeError(f"No u files loaded for sequence {seq.name!r}")
        return rest_pos, np.column_stack(disps) * seq.displacement_scale

    else:
        raise ValueError(f"Unknown sequence_type: {seq.sequence_type!r}")


def _build_display_mesh(
    seq: AnimationSequence,
    driving_rest_pos: np.ndarray,
    driving_disps: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Compute display mesh rest positions and per-frame displacements.

    Returns (display_rest_pos, display_disps, triangles):
      display_rest_pos : (3*n_display,)  float64, flat
      display_disps    : (3*n_display, n_frames) float64
      triangles        : (n_tri, 3) int32
    """
    driving_path = Path(seq.driving_mesh)
    is_tet = str(driving_path).endswith(".veg")

    if seq.display_mesh:
        # Load separate high-res display mesh
        display_data = read_obj(seq.display_mesh)
        display_verts = display_data.vertices * seq.embedded_scale  # (n_disp, 3)
        triangles = display_data.elements.astype(np.int32)          # (n_tri, 3)
        display_rest_pos = display_verts.ravel()

        if is_tet:
            # Volume barycentric interpolation
            veg = read_veg(str(driving_path))
            vol = VolumeMesh.from_veg_file(veg)
            embedding = BarycentricEmbedding(display_verts, vol)
            n_frames = driving_disps.shape[1]
            display_disps = np.zeros((len(display_verts) * 3, n_frames))
            for f in range(n_frames):
                display_disps[:, f] = embedding.deform(driving_disps[:, f])
        else:
            # Surface barycentric (closest-point projection)
            driving_mesh = read_obj(str(driving_path))
            driving_verts = driving_mesh.vertices * seq.embedding_scale
            driving_tris = driving_mesh.elements

            tri_idx, bary_w = _surface_embedding(display_verts, driving_verts, driving_tris)
            display_disps = _apply_surface_embedding(tri_idx, bary_w, driving_tris, driving_disps)

    else:
        # No separate display mesh — use driving mesh surface
        if is_tet:
            veg = read_veg(str(driving_path))
            vol = VolumeMesh.from_veg_file(veg)
            surf = vol.extract_surface_mesh()
            display_verts = surf.vertices * seq.embedded_scale
            triangles = surf.elements.astype(np.int32)
            display_rest_pos = display_verts.ravel()

            embedding = BarycentricEmbedding(display_verts, vol)
            n_frames = driving_disps.shape[1]
            display_disps = np.zeros((len(display_verts) * 3, n_frames))
            for f in range(n_frames):
                display_disps[:, f] = embedding.deform(driving_disps[:, f])
        else:
            # Driving is a surface mesh — display == driving
            driving_mesh = read_obj(str(driving_path))
            display_verts = driving_mesh.vertices * seq.embedded_scale
            triangles = driving_mesh.elements.astype(np.int32)
            display_rest_pos = display_verts.ravel()
            display_disps = driving_disps

    return display_rest_pos.astype(np.float32), display_disps.astype(np.float32), triangles


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def process_sequence(seq: AnimationSequence, output_folder: str | Path) -> None:
    """Process a single AnimationSequence and write an Alembic .abc file."""
    if not _core.has_animation_io():
        raise RuntimeError("Alembic is not available in this build.")

    output_folder = Path(output_folder)
    output_folder.mkdir(parents=True, exist_ok=True)

    print(f"[{seq.name}] loading displacements ...")
    rest_pos, driving_disps = _load_driving_displacements(seq)

    print(f"[{seq.name}] {driving_disps.shape[1]} frames loaded, building display mesh ...")
    display_rest, display_disps, triangles = _build_display_mesh(seq, rest_pos, driving_disps)

    print(f"[{seq.name}] writing ABC ({display_disps.shape[1]} frames) ...")
    n_frames = display_disps.shape[1]
    n_dof = display_disps.shape[0]

    _core.dump_abc(
        str(output_folder / f"{seq.name}.abc"),
        seq.name,
        display_rest.tolist(),
        [display_disps[:, f].tolist() for f in range(n_frames)],
        triangles.tolist(),
    )
    print(f"[{seq.name}] done → {output_folder / seq.name}.abc")


def convert_animation(
    config: str | Path | dict,
    output_folder: str | Path | None = None,
) -> None:
    """Convert all animation sequences in a JSON config to Alembic .abc files.

    Parameters
    ----------
    config
        Path to a JSON config file (same format as the C++ convertAnimation tool),
        or a pre-parsed dict. When a path is given, relative paths inside the
        config are resolved relative to its parent directory.
    output_folder
        Destination for the .abc files. Defaults to the config file's directory
        (for dict input, defaults to the current directory).
    """
    if isinstance(config, dict):
        base_dir = Path(".")
        cfg = config
    else:
        config_path = Path(config)
        base_dir = config_path.parent
        with open(config_path) as f:
            cfg = json.load(f)

    if output_folder is None:
        output_folder = cfg.get("output-folder", str(base_dir))
        output_folder = base_dir / output_folder if not Path(output_folder).is_absolute() else Path(output_folder)
    else:
        output_folder = Path(output_folder)

    sequences = [AnimationSequence.from_dict(m, base_dir) for m in cfg["meshes"]]
    for seq in sequences:
        process_sequence(seq, output_folder)


def has_stress_vdb_export() -> bool:
    """Return True if the Alembic + OpenVDB stress VDB export is available."""
    return bool(_core.has_stress_vdb_export())


def dump_stress_vdb(
    veg_path: str | Path,
    sim_output: str | Path,
    output_dir: str | Path,
    *,
    prefix: str = "vonMises",
    voxel_size: float = 0.0,
    frame_start: int = 0,
    frame_end: int = -1,
) -> int:
    """Splat per-tet von Mises stress into a per-frame OpenVDB sequence.

    Mirrors the ``dumpStressVDB`` CLI tool. Reads the standard runIPCSim output
    layout::

        {sim_output}/states/deform{frame:04d}.u
        {sim_output}/stress/von_mises{frame:04d}.json

    and writes ``{output_dir}/{prefix}{frame:04d}.vdb`` for each frame.

    Parameters
    ----------
    veg_path:    Path to the `.veg` rest-configuration tet mesh.
    sim_output:  runIPCSim output folder (must contain ``states/`` and ``stress/``).
    output_dir:  Destination folder for the ``.vdb`` sequence.
    prefix:      Output filename prefix (default ``"vonMises"``).
    voxel_size:  VDB voxel size in world units. ``<= 0`` auto-derives ~½ rest edge length.
    frame_start: First frame index (inclusive).
    frame_end:   Last frame index (exclusive). ``-1`` auto-detects from files on disk.

    Returns the number of frames written.
    """
    if not has_stress_vdb_export():
        raise RuntimeError(
            "Stress VDB export requires OpenVDB support. "
            "Reconfigure the build with PGO_ENABLE_OPENVDB=ON."
        )

    sim_output = Path(sim_output)
    states_dir = sim_output / "states"
    stress_dir = sim_output / "stress"

    if not states_dir.is_dir():
        raise FileNotFoundError(f"states folder not found: {states_dir}")
    if not stress_dir.is_dir():
        raise FileNotFoundError(f"stress folder not found: {stress_dir}")

    if frame_end < 0:
        frame = frame_start
        while (
            (states_dir / f"deform{frame:04d}.u").exists()
            and (stress_dir / f"von_mises{frame:04d}.json").exists()
        ):
            frame += 1
        frame_end = frame
        print(f"Auto-detected frame range: [{frame_start}, {frame_end})")

    if frame_end <= frame_start:
        raise ValueError(
            f"No frames found in {sim_output} starting at frame {frame_start}"
        )

    Path(output_dir).mkdir(parents=True, exist_ok=True)

    exporter = _core.PyStressFieldVDBExporter()

    if exporter.load_tet_mesh(str(veg_path)) != 0:
        raise RuntimeError(f"Failed to load tet mesh: {veg_path}")
    if exporter.load_deformation_sequence(
        str(states_dir), "deform{:04d}.u", frame_start, frame_end
    ) != 0:
        raise RuntimeError(f"Failed to load deformation sequence from {states_dir}")
    if exporter.load_von_mises_sequence(
        str(stress_dir), "von_mises{:04d}.json", frame_start, frame_end
    ) != 0:
        raise RuntimeError(f"Failed to load von Mises sequence from {stress_dir}")
    if exporter.export_animation_vdb(str(output_dir), prefix, voxel_size) != 0:
        raise RuntimeError(f"Failed to export VDB sequence to {output_dir}")

    return exporter.num_frames()
