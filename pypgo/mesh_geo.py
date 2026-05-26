"""Geometry-only mesh types for pypgo.

Exposes three CellMeshGeo instantiations and the CellMeshType enum.
"""

import numpy as np
import pypgo._core as _core
from pypgo._arrays import float_matrix, index_matrix

# Re-export the enum directly from _core
CellMeshType = _core.CellMeshType


class TriCellMeshGeo:
    """Triangle surface mesh geometry (CellMeshGeo<3>)."""

    def __init__(self, vertices, cells=None):
        if isinstance(vertices, _core.TriCellMeshGeoCore) and cells is None:
            # Internal construction from pre-existing _core object
            self._core_obj = vertices
        else:
            v_arr = float_matrix("vertices", vertices, 3)
            c_arr = index_matrix("cells", cells, 3, num_vertices=v_arr.shape[0])
            v_flat = v_arr.ravel().tolist()
            c_flat = c_arr.ravel().tolist()
            self._core_obj = _core.create_tricellmesh_geo(v_flat, c_flat)

    @property
    def vertices(self) -> np.ndarray:
        return np.array(self._core_obj.vertices(), dtype=np.float64).reshape(-1, 3)

    @property
    def cells(self) -> np.ndarray:
        return np.array(self._core_obj.cells(), dtype=np.int64).reshape(-1, 3)

    @property
    def cell_type(self):
        return self._core_obj.cell_type()

    @property
    def num_vertices(self) -> int:
        return self._core_obj.num_vertices()

    @property
    def num_cells(self) -> int:
        return self._core_obj.num_cells()


class TetCellMeshGeo:
    """Tetrahedral volume mesh geometry (CellMeshGeo<4>)."""

    def __init__(self, vertices, cells=None):
        if isinstance(vertices, _core.TetCellMeshGeoCore) and cells is None:
            self._core_obj = vertices
        else:
            v_arr = float_matrix("vertices", vertices, 3)
            c_arr = index_matrix("cells", cells, 4, num_vertices=v_arr.shape[0])
            v_flat = v_arr.ravel().tolist()
            c_flat = c_arr.ravel().tolist()
            self._core_obj = _core.create_tetcellmesh_geo(v_flat, c_flat)

    @property
    def vertices(self) -> np.ndarray:
        return np.array(self._core_obj.vertices(), dtype=np.float64).reshape(-1, 3)

    @property
    def cells(self) -> np.ndarray:
        return np.array(self._core_obj.cells(), dtype=np.int64).reshape(-1, 4)

    @property
    def cell_type(self):
        return self._core_obj.cell_type()

    @property
    def num_vertices(self) -> int:
        return self._core_obj.num_vertices()

    @property
    def num_cells(self) -> int:
        return self._core_obj.num_cells()


class CubicCellMeshGeo:
    """Cubic/hexahedral volume mesh geometry (CellMeshGeo<8>)."""

    def __init__(self, vertices, cells=None):
        if isinstance(vertices, _core.CubicCellMeshGeoCore) and cells is None:
            self._core_obj = vertices
        else:
            v_arr = float_matrix("vertices", vertices, 3)
            c_arr = index_matrix("cells", cells, 8, num_vertices=v_arr.shape[0])
            v_flat = v_arr.ravel().tolist()
            c_flat = c_arr.ravel().tolist()
            self._core_obj = _core.create_cubiccellmesh_geo(v_flat, c_flat)

    @property
    def vertices(self) -> np.ndarray:
        return np.array(self._core_obj.vertices(), dtype=np.float64).reshape(-1, 3)

    @property
    def cells(self) -> np.ndarray:
        return np.array(self._core_obj.cells(), dtype=np.int64).reshape(-1, 8)

    @property
    def cell_type(self):
        return self._core_obj.cell_type()

    @property
    def num_vertices(self) -> int:
        return self._core_obj.num_vertices()

    @property
    def num_cells(self) -> int:
        return self._core_obj.num_cells()
