"""Alembic (.abc) animation export and Eigen binary matrix I/O."""

from __future__ import annotations

import struct
from pathlib import Path

import numpy as np

import pypgo._core as _core


# ---------------------------------------------------------------------------
# Alembic I/O capability check
# ---------------------------------------------------------------------------

def has_animation_io() -> bool:
    """Return True if Alembic I/O is available in this build."""
    return bool(_core.has_animation_io())


# ---------------------------------------------------------------------------
# Eigen binary matrix I/O (.u files)
# ---------------------------------------------------------------------------

def read_u_file(path: str | Path) -> np.ndarray:
    """Read an Eigen binary matrix ``.u`` file.

    Returns a ``(nrows, ncols)`` float64 array (column-major, matching Eigen
    default). The C++ animation pipeline consumes this format for per-frame
    displacements.
    """
    with open(path, "rb") as f:
        nrows, ncols, entry_size = struct.unpack("iii", f.read(12))
        dtype = np.float64 if entry_size == 8 else np.float32
        data = np.frombuffer(f.read(nrows * ncols * entry_size), dtype=dtype)
    return np.ascontiguousarray(data.reshape((nrows, ncols), order="F"))


def write_u_file(path: str | Path, mat: np.ndarray) -> None:
    """Write a 2-D array as an Eigen binary matrix ``.u`` file.

    Data is stored as float64 in column-major order with a 12-byte header.
    Use this to write per-frame displacement data for the animation pipeline.
    """
    mat = np.asfortranarray(mat, dtype=np.float64)
    if mat.ndim == 1:
        mat = mat[:, None]
    nrows, ncols = mat.shape
    with open(path, "wb") as f:
        f.write(struct.pack("iii", nrows, ncols, 8))
        f.write(mat.tobytes(order="F"))


# ---------------------------------------------------------------------------
# Alembic writer (in-memory → .abc)
# ---------------------------------------------------------------------------

class AbcWriter:
    """Incremental Alembic (``.abc``) writer for a single mesh.

    Accumulates displacement frames in memory and writes the archive in one
    shot. Supports the context-manager protocol so the file is written
    automatically on exit.

    Usage
    -----

        # One-shot (all frames known up front)
        AbcWriter.dump("out.abc", "my_mesh",
                        rest_positions=rest, triangles=tris,
                        displacements=disps)

        # Incremental (add frames as they are computed)
        with AbcWriter("out.abc", "my_mesh",
                       rest_positions=rest, triangles=tris) as w:
            for disp in displacements:
                w.add_frame(disp)
        # writes on context exit

    Parameters
    ----------
    path :
        Output ``.abc`` file path.
    name :
        Alembic object name.
    rest_positions :
        Flat ``(3 * n_verts,)`` float64 array of rest positions.
    triangles :
        Flat ``(3 * n_tris,)`` int array of face indices.
    fps :
        Frame rate written into the Alembic archive (default 24).
    """

    def __init__(
        self,
        path: str | Path,
        name: str,
        *,
        rest_positions: np.ndarray,
        triangles: np.ndarray,
        fps: float = 24.0,
    ) -> None:
        if not has_animation_io():
            raise RuntimeError("Alembic I/O is not available in this build.")

        self._path = str(path)
        self._name = name
        self._fps = float(fps)
        self._rest = np.asarray(rest_positions, dtype=np.float64).ravel()
        self._tris = np.asarray(triangles, dtype=np.int32).ravel()
        self._frames: list[np.ndarray] = []

    # -- builder API ----------------------------------------------------------

    def add_frame(self, displacement: np.ndarray) -> None:
        """Append one frame of displacement data.

        *displacement* must be a flat ``(3 * n_verts,)`` array (float64 or
        convertible).
        """
        self._frames.append(
            np.asarray(displacement, dtype=np.float64).ravel()
        )

    def write(self) -> None:
        """Flush all accumulated frames to the ``.abc`` file.

        Safe to call multiple times; each call overwrites the file with the
        current set of frames.  Raises ``RuntimeError`` if the archive
        cannot be written.
        """
        n_tris = len(self._tris) // 3
        _core.dump_abc(
            self._path,
            self._name,
            self._rest.tolist(),
            [f.tolist() for f in self._frames],
            [self._tris[i * 3:(i + 1) * 3].tolist() for i in range(n_tris)],
            self._fps,
        )

    # -- context manager ------------------------------------------------------

    def __enter__(self) -> "AbcWriter":
        return self

    def __exit__(self, *_: object) -> None:
        if self._frames:
            self.write()

    # -- one-shot convenience ---------------------------------------------------

    @classmethod
    def dump(
        cls,
        path: str | Path,
        name: str,
        *,
        rest_positions: np.ndarray,
        triangles: np.ndarray,
        displacements: list[np.ndarray],
        fps: float = 24.0,
    ) -> None:
        """Write an ``.abc`` file from all frames at once.

        Equivalent to constructing an ``AbcWriter``, calling
        ``add_frame`` for each displacement, then ``write()``.
        """
        with cls(
            path, name,
            rest_positions=rest_positions, triangles=triangles, fps=fps,
        ) as w:
            for d in displacements:
                w.add_frame(d)


# ---------------------------------------------------------------------------
# AnimationLoader — JSON-config-driven pipeline
# ---------------------------------------------------------------------------

class AnimationLoader:
    """Load a JSON animation config and export Alembic (``.abc``) files.

    The config format mirrors the C++ ``AnimationLoader`` JSON schema::

        {
            "meshes": [
                {
                    "name": "my_mesh",
                    "driving-mesh": "path/to/rest.obj",
                    "sequence": "path/to/frame_{:04d}.u",
                    "sequence-type": "u",
                    "sequence-range": [0, 100]
                }
            ]
        }

    Relative paths in the config are resolved against the config file's
    parent directory by the C++ backend.

    For the common one-shot case, use :func:`dump_animation` instead of
    managing the loader directly.
    """

    def __init__(self) -> None:
        if not has_animation_io():
            raise RuntimeError("Alembic I/O is not available in this build.")
        self._loader = _core.PyAnimationLoader()

    def load(self, config_path: str | Path) -> None:
        """Load and parse a JSON animation config file.

        Raises ``RuntimeError`` on failure.
        """
        if self._loader.load(str(config_path)) != 0:
            raise RuntimeError(f"AnimationLoader failed to load config: {config_path}")

    def save_abc(self, output_folder: str | Path) -> None:
        """Write one ``.abc`` file per mesh to *output_folder*.

        The directory is created if it does not exist. Raises
        ``RuntimeError`` on failure.
        """
        output_folder = Path(output_folder)
        output_folder.mkdir(parents=True, exist_ok=True)
        if self._loader.save_abc(str(output_folder)) != 0:
            raise RuntimeError(
                f"AnimationLoader failed to save ABC to: {output_folder}"
            )


# ---------------------------------------------------------------------------
# Convenience function
# ---------------------------------------------------------------------------

def dump_animation(
    config_path: str | Path,
    output_folder: str | Path | None = None,
) -> None:
    """Load a JSON animation config and export Alembic ``.abc`` files.

    This is the one-shot equivalent of::

        loader = AnimationLoader()
        loader.load(config_path)
        loader.save_abc(output_folder)

    Parameters
    ----------
    config_path : Path to a JSON config file.
    output_folder :
        Destination directory. Defaults to the config's ``output-folder``
        field, or ``./output``.
    """
    import json

    config_path = Path(config_path)

    if output_folder is None:
        with open(config_path) as f:
            cfg = json.load(f)
        configured_output = cfg.get("output-folder")
        if configured_output is None:
            output_folder = Path("output")
        else:
            output_folder = Path(configured_output)
            if not output_folder.is_absolute():
                output_folder = config_path.parent / output_folder

    loader = AnimationLoader()
    loader.load(config_path)
    loader.save_abc(output_folder)
