"""OpenVDB stress-field export.

Splats per-tet von Mises stress into per-frame ``.vdb`` sequences.
"""

from __future__ import annotations

from pathlib import Path

import pypgo._core as _core


def has_stress_vdb_export() -> bool:
    """Return True if OpenVDB stress VDB export is available in this build."""
    return bool(_core.has_stress_vdb_export())


# ---------------------------------------------------------------------------
# StressFieldVDBExporter
# ---------------------------------------------------------------------------

class StressFieldVDBExporter:
    """Splat per-tet von Mises stress into a per-frame OpenVDB sequence.

    Reads the standard simulation stress-output layout::

        {sim_output}/states/deform{frame:04d}.u
        {sim_output}/stress/von_mises{frame:04d}.json

    and writes ``{output_dir}/{prefix}{frame:04d}.vdb`` for each frame.

    Requires an OpenVDB-enabled build (``PGO_ENABLE_OPENVDB=ON``).

    Example
    -------

        exporter = StressFieldVDBExporter()
        exporter.load_tet_mesh("bunny.veg")
        exporter.load_deformation_sequence("sim/states", "deform{:04d}.u", 0, 24)
        exporter.load_von_mises_sequence("sim/stress", "von_mises{:04d}.json", 0, 24)
        exporter.export_animation_vdb("output/", "vonMises")
        print(f"Wrote {exporter.num_frames} frame(s)")
    """

    def __init__(self) -> None:
        if not has_stress_vdb_export():
            raise RuntimeError(
                "Stress VDB export requires OpenVDB support. "
                "Reconfigure the build with PGO_ENABLE_OPENVDB=ON."
            )
        self._exporter = _core.PyStressFieldVDBExporter()

    def load_tet_mesh(self, veg_path: str | Path) -> None:
        """Load the rest-configuration tet mesh from a ``.veg`` file.

        Raises ``RuntimeError`` on failure.
        """
        if self._exporter.load_tet_mesh(str(veg_path)) != 0:
            raise RuntimeError(f"Failed to load tet mesh: {veg_path}")

    def load_deformation_sequence(
        self,
        folder: str | Path,
        pattern: str,
        frame_start: int,
        frame_end: int,
    ) -> None:
        """Load per-frame displacement ``.u`` files.

        Parameters
        ----------
        folder : Path to the ``states/`` directory.
        pattern : printf-style pattern, e.g. ``"deform{:04d}.u"``.
        frame_start : First frame index (inclusive).
        frame_end : Last frame index (exclusive).

        Raises ``RuntimeError`` on failure.
        """
        if self._exporter.load_deformation_sequence(
            str(folder), pattern, frame_start, frame_end
        ) != 0:
            raise RuntimeError(
                f"Failed to load deformation sequence from {folder}"
            )

    def load_von_mises_sequence(
        self,
        folder: str | Path,
        pattern: str,
        frame_start: int,
        frame_end: int,
    ) -> None:
        """Load per-frame von Mises stress JSON files.

        Parameters
        ----------
        folder : Path to the ``stress/`` directory.
        pattern : printf-style pattern, e.g. ``"von_mises{:04d}.json"``.
        frame_start : First frame index (inclusive).
        frame_end : Last frame index (exclusive).

        Raises ``RuntimeError`` on failure.
        """
        if self._exporter.load_von_mises_sequence(
            str(folder), pattern, frame_start, frame_end
        ) != 0:
            raise RuntimeError(
                f"Failed to load von Mises sequence from {folder}"
            )

    def export_animation_vdb(
        self,
        output_dir: str | Path,
        prefix: str = "vonMises",
        voxel_size: float = 0.0,
    ) -> None:
        """Write the VDB sequence to *output_dir*.

        Parameters
        ----------
        output_dir : Destination folder for ``.vdb`` files.
        prefix : Output filename prefix (default ``"vonMises"``).
        voxel_size : VDB voxel size. ``<= 0`` auto-derives from the tet mesh.

        Raises ``RuntimeError`` on failure.
        """
        if self._exporter.export_animation_vdb(str(output_dir), prefix, voxel_size) != 0:
            raise RuntimeError(f"Failed to export VDB sequence to {output_dir}")

    @property
    def num_frames(self) -> int:
        """Number of frames loaded."""
        return self._exporter.num_frames()


# ---------------------------------------------------------------------------
# Convenience function
# ---------------------------------------------------------------------------

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
    """Convenience function that auto-detects frame range and exports VDB.

    See :class:`StressFieldVDBExporter` for the underlying pipeline and
    the expected file layout.

    Returns the number of frames written.
    """
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

    exporter = StressFieldVDBExporter()
    exporter.load_tet_mesh(veg_path)
    exporter.load_deformation_sequence(states_dir, "deform{:04d}.u", frame_start, frame_end)
    exporter.load_von_mises_sequence(stress_dir, "von_mises{:04d}.json", frame_start, frame_end)
    exporter.export_animation_vdb(output_dir, prefix, voxel_size)
    return exporter.num_frames
