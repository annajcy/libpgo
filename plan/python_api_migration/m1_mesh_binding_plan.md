# M1 MeshData / MeshGeo 绑定状态与收尾计划

> **状态更新时间：** 2026-05-27（mesher wrapper + lightweight `.veg` adapter 收口）
> **当前方向：** `MeshData<K>` 是唯一中间转换数据结构；Python 公开 `MeshData` 数据容器和 `MeshGeo` façade 两套对象，但 I/O 与 `VolumeMesh` 边界只接受/返回 `MeshData`。

## 本轮收口状态

- `pypgo.tools.mesh.cubic_mesher` / `tet_mesher` / `has_tetwild` 已落地；Python wrapper 接收 `TriMeshData`，返回 `CubicMeshData` / `TetMeshData`，并在 tetwild 未编译时提前抛出清晰 `RuntimeError`。
- 新增 `src/core/volumetricMesh/vegFile.h/cpp` POD adapter；`_core.read_veg` / `_core.write_veg` 已改为桥接 `readVegFile` / `writeVegFile`，读 ASCII `.veg` 时不再构造完整 `TetMesh` / `CubicMesh` 后导出 payload。
- Python 测试新增 mesher wrapper 覆盖，以及多 material `.veg` roundtrip 覆盖。

## 当前 API

```python
import pypgo as pgo

tri_data = pgo.mesh_geo.TriMeshData(vertices, triangles)
tet_data = pgo.mesh_geo.TetMeshData(vertices, tets)
cubic_data = pgo.mesh_geo.CubicMeshData(vertices, cubes)

tri_geo = pgo.mesh_geo.TriMeshGeo.from_mesh_data(tri_data)
tet_geo = pgo.mesh_geo.TetMeshGeo.from_mesh_data(tet_data)
cubic_geo = pgo.mesh_geo.CubicMeshGeo.from_mesh_data(cubic_data)

material = pgo.mesh.MaterialSpec(E=1e9, nu=0.45, density=1000.0)
volume = pgo.mesh.VolumeMesh(tet_data, material)

data_from_file, material_from_file = pgo.io.read_veg_geo("box.veg")
surface_data = pgo.io.read_obj_geo("box.obj")
volume_from_file = pgo.mesh.VolumeMesh.load("box.veg")
```

Public modules:

- `pypgo.mesh_geo`: geometry-only API with `TriMeshData`, `TetMeshData`, `CubicMeshData`, plus `TriMeshGeo`, `TetMeshGeo`, and `CubicMeshGeo`.
- `pypgo.mesh`: `MaterialSpec` and `VolumeMesh`.
- `pypgo.io`: canonical I/O helpers whose public boundary is `MeshData`.
- `pypgo._core`: private nanobind module.

## 目标模块布局（Phase 2/4 完成后）

```python
import pypgo as pgo

tet_data = pgo.mesh.TetMeshData(vertices, tets)
tri_data = pgo.mesh.read_obj("surface.obj")

tri_geo = pgo.mesh.geo.TriMeshGeo.from_mesh_data(tri_data)

veg = pgo.mesh.veg.read_veg("box.veg")
volume = pgo.mesh.veg.VolumeMesh(
    veg.mesh_data,
    regions=veg.to_volume_regions(),
)

sim_volume = pgo.sim.SimulationMesh.create_volumetric(volume)
shell_mat = pgo.sim.KoiterStVKShellMaterial(
    "cloth", thickness=0.001, E_membrane=1e6, nu_membrane=0.4)
sim_shell = pgo.sim.SimulationMesh.create_shell(
    tri_data,
    shell_mat,
)

pgo.sim.write_shell("cloth.shell.json", tri_data, shell_mat)
tri_from_file, shell_mat_from_file = pgo.sim.read_shell("cloth.shell.json")
```

Target public modules:

- `pypgo.mesh`: `MeshData` 数据容器、OBJ I/O、shape factory，以及 pure-Python mesh data 属性。
- `pypgo.mesh.geo`: `TriMeshGeo` / `TetMeshGeo` / `CubicMeshGeo` façade、geometry-only algorithm，以及 `BarycentricEmbedding`。
- `pypgo.mesh.veg`: Vega volume wrapper：`VolumeMesh`、`.veg` I/O、volume materials、sets/regions、volume surface extraction。
- `pypgo.sim`: solver-ready `SimulationMesh`、simulation material / factory API，以及 shell spec I/O。
- `pypgo._core`: private nanobind module.

## Boundary Rules

- `MeshData` is the only intermediate conversion representation.
- `MeshGeo -> MeshData` uses explicit `.to_mesh_data()`.
- `MeshData -> MeshGeo` uses explicit `.from_mesh_data(data)`.
- `VolumeMesh` accepts only `TetMeshData` or `CubicMeshData`.
- `read_obj_geo()` returns `TriMeshData`.
- `read_veg_geo()` returns `(TetMeshData | CubicMeshData, MaterialSpec)`.
- `write_obj_geo()` accepts only `TriMeshData`.
- `write_veg_geo()` accepts only `TetMeshData` or `CubicMeshData`.
- Old public names are removed: no `TriCellMeshGeo`, `TetCellMeshGeo`, `CubicCellMeshGeo`, or `CellMeshType`.

## C++ Naming

- `src/core/mesh/meshData.h`
- `src/core/mesh/meshData.cpp`
- `pgo::Mesh::MeshData<K>`
- `pgo::Mesh::MeshDataType`
- `pgo::Mesh::ElementView<K>`
- `pgo::Mesh::ElementsView<K>`
- `pgo::Mesh::TriMeshData`
- `pgo::Mesh::TetMeshData`
- `pgo::Mesh::CubicMeshData`

Core method names:

- `meshType()`
- `numElements()`
- `elements()`
- `elementsFlat()`
- `elementVtxID(elementID, localVertexID)`
- `verticesPerElement()`
- `fromElements(...)`
- `fromFlatElements(...)`

Legacy façade bridge names:

- `TriMeshGeo::toMeshData()`
- `TetMeshGeo::toMeshData()`
- `CubicMeshGeo::toMeshData()`
- `TriMeshGeo(const MeshData<3>&)`
- `TetMeshGeo(const MeshData<4>&)`
- `CubicMeshGeo(const MeshData<8>&)`

## Python Binding Surface

Private `_core` names:

- `MeshDataType`
- `TriMeshDataCore`
- `TetMeshDataCore`
- `CubicMeshDataCore`
- `TriMeshGeoCore`
- `TetMeshGeoCore`
- `CubicMeshGeoCore`
- `MaterialSpecCore`
- `VolumeMeshCore`
- `create_tri_mesh_data(vertices, elements)`
- `create_tet_mesh_data(vertices, elements)`
- `create_cubic_mesh_data(vertices, elements)`
- `create_tri_mesh_geo(vertices, triangles)`
- `create_tet_mesh_geo(vertices, tets)`
- `create_cubic_mesh_geo(vertices, cubes)`
- `create_volume_mesh(mesh_data, material_spec)`
- `load_volume_mesh(path)`
- `save_volume_mesh(path, volume_mesh)`
- `read_veg_geo(path)`
- `write_veg_geo(path, mesh_data, material_spec)`
- `read_obj_geo(path)`
- `write_obj_geo(path, surface_data)`

## Tests And Verification

Required C++ tests:

- `MeshDataGTest` covers construction, validation, and `ElementsView` indexing.
- `TriMeshGeoGTest`, `TetMeshGeoGTest`, and `CubicMeshGeoGTest` cover `toMeshData()` and reconstruction from `MeshData`.
- `TetMesh` and `CubicMesh` constructor tests use `MeshData`.

Required Python tests:

- `TriMeshData`, `TetMeshData`, and `CubicMeshData` construction, validation, copy behavior, and properties.
- `TriMeshGeo`, `TetMeshGeo`, and `CubicMeshGeo` common queries and explicit conversion.
- `VolumeMesh` accepts volume `MeshData` and rejects `MeshGeo` / `TriMeshData`.
- I/O reads and writes `MeshData`, and rejects `MeshGeo` façade objects.
- Old public names are absent.

Run:

```bash
# Python build (pypgo_core only, no C++ tests)
conda run -n libpgo cmake --preset pypgo
conda run -n libpgo cmake --build --preset pypgo -j 8

# C++ tests — use the base preset (PGO_BUILD_TESTING=ON)
conda run -n libpgo cmake --preset base
conda run -n libpgo cmake --build --preset base -j 8
conda run -n libpgo ctest --test-dir build/base -R "MeshData|TriMeshGeo|TetMeshGeo|CubicMeshGeo" --output-on-failure

# Or run gtest binaries directly from the base build
conda run -n libpgo build/base/tests/src/core/mesh/MeshDataGTest
conda run -n libpgo build/base/tests/src/core/mesh/TriMeshGeoGTest
conda run -n libpgo build/base/tests/src/core/mesh/TetMeshGeoGTest
conda run -n libpgo build/base/tests/src/core/mesh/CubicMeshGeoGTest

# Python tests
conda run -n libpgo python -m pytest -q tests/pypgo
```

## Remaining Policy Note

本节以上描述的是 Phase 1 / M1 当前状态快照：`.veg` material policy 尚未展开，`MaterialSpec` 仍是当前 exposed Python material value object。下面 Phase 2 会把 full material / region / set modeling 正式纳入 M1 收尾计划，并用 `ENuMaterial` / `MooneyRivlinMaterial` / `OrthotropicMaterial` 替代 `MaterialSpec` 的长期 public role。

---

## Cross-Phase Binding Infrastructure（新增）

> **状态：** 设计中，尚未实现

这三项不是独立用户功能，但会决定 Phase 2/3/4 绑定是否可维护。纳入 M1 plan，作为 mesh pipeline 的底层交付物，而不是散落在各个 binding TU 里临时实现。

| 项目 | 状态 | M1 交付边界 |
|------|------|-------------|
| Eigen dense vector/matrix ↔ NumPy conversion helpers | Not done | 新增共享 binding helper，统一 safe zero-copy input map、copy fallback、owned output / capsule-owned view 规则 |
| Eigen sparse matrix wrapper + COO export | Not done | 新增最小 `pypgo.sparse.SparseMatrix`，只承诺 shape / nnz / COO export；不在 M1 做 sparse algebra |
| `gil_scoped_release` policy for long-running kernels | Not done | 规定所有长耗时 C++ kernel 的 GIL 释放边界，Phase 2/3/4 新 binding 必须遵守 |

### Dense Eigen ↔ NumPy helper

新增共享头文件：

```cpp
// src/python/pypgo/bindings/eigen_numpy.h

#pragma once

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <Eigen/Dense>

namespace pgo::python {

using ConstVectorMapXd = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>;
using ConstMatrixMapXd = Eigen::Map<
    const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>,
    Eigen::Unaligned>;

ConstVectorMapXd ndarrayToVectorMapXd(nb::ndarray<nb::numpy, const double> array);
ConstMatrixMapXd ndarrayToRowMajorMatrixMapXd(
    nb::ndarray<nb::numpy, const double> array);

Eigen::VectorXd ndarrayToVectorXd(nb::ndarray<nb::numpy, const double> array);
Eigen::MatrixXd ndarrayToMatrixXd(nb::ndarray<nb::numpy, const double> array);

template<int Rows, int Cols>
Eigen::Matrix<double, Rows, Cols> ndarrayToFixedMatrix(
    nb::ndarray<nb::numpy, const double> array);

nb::ndarray<nb::numpy, double> vectorXdToNdarray(Eigen::VectorXd values);
nb::ndarray<nb::numpy, double> matrixXdToNdarray(Eigen::MatrixXd values);

}  // namespace pgo::python
```

设计规则：

- **M1 helper 支持 safe zero-copy input map for compatible NumPy arrays。** 对 dtype、维度、contiguous/stride 兼容的只读 NumPy 输入，binding 可以用 `Eigen::Map` 在当前 C++ 调用期间零拷贝读取。这个 map 不得存入 core object，也不得跨过原 Python `ndarray` lifetime。
- **Copy fallback 必须显式。** 当输入不是 `float64`、维度不匹配、stride 不能安全表达，或 C++ kernel 需要长期持有数据时，Python wrapper 先 `np.asarray(..., dtype=np.float64)` 归一化，或 C++ helper 返回 owned `Eigen::VectorXd/MatrixXd` copy。不要在调用点散写 ad-hoc stride 解析。
- **输出默认 owned copy / capsule-owned view。** C++ 返回 NumPy 时，不能返回指向局部 Eigen 对象的 view；允许两种安全路径：直接 copy 到新的 NumPy array，或把 `Eigen::MatrixXd` / `std::vector<double>` move 到 heap backing storage，并用 Python capsule 管 lifetime 后返回 view。
- **长期 core object 的 mutable zero-copy view 暂不公开。** `MeshData.vertices/elements`、`SparseMatrix` 内部数组、未来 solver state 等如果暴露 mutable alias，用户修改 NumPy view 可能破坏 C++ 不变量。M1 不承诺这种长期双向共享；需要共享时先做只读 view 或 copy。
- MeshData 的 `vertices/elements` 现有 Python wrapper 可以继续走 Python NumPy reshape；后续 solver/deformation binding 若直接收发 `Eigen::VectorXd/MatrixXd`，必须复用本 helper，不在各 TU 复制 `nb::ndarray` 解析逻辑。

测试要求：

- `VectorXd` roundtrip：shape `(n,)`，dtype `float64`，值一致
- compatible `float64` C-contiguous vector/matrix 输入走 zero-copy map；测试可通过修改原 NumPy array 后再次调用只读 kernel 验证 map 观察到新值
- `MatrixXd` roundtrip：shape `(m, n)`，C-contiguous / non-contiguous slice 输入都能被明确 map、拷贝或抛清楚错误
- fixed-size matrix 输入维度错误时抛 `ValueError`
- 返回数组 owns data 或 capsule owns backing storage：删除原 C++ temporary 后 Python 侧读取仍有效

### SparseMatrix wrapper + COO export

Phase 2 的 `BarycentricEmbedding` 会产生插值矩阵；M1 不应只返回裸 COO tuple 后把 sparse wrapper 推到 M2。改为在 M1 暴露最小 sparse wrapper：

```python
# pypgo/sparse.py

class SparseMatrix:
    @property
    def shape(self) -> tuple[int, int]: ...

    @property
    def nnz(self) -> int: ...

    def to_coo(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Return row, col, value arrays with 0-based indices."""
```

```cpp
// src/python/pypgo/bindings/sparse_bindings.cpp 中新增

class SparseMatrixCore {
public:
  explicit SparseMatrixCore(EigenSupport::SpMatD matrix);

  int rows() const;
  int cols() const;
  int nnz() const;

  // row-major Eigen sparse -> sorted COO arrays
  std::tuple<std::vector<int>, std::vector<int>, std::vector<double>> toCOO() const;

private:
  EigenSupport::SpMatD matrix_;
};
```

设计规则：

- M1 只支持 `double` value 和 0-based COO export；不做 CSR/CSC、SciPy adapter、matrix multiply、factorization 或 slicing。
- `SparseMatrixCore` owns `EigenSupport::SpMatD`，避免 Python 拿到悬垂引用。
- COO export 保持 deterministic：按 Eigen row-major 外层顺序输出；同一 `(row, col)` 重复项应在 core 构造前合并，或在 wrapper 构造时 `makeCompressed()` 后导出。
- `BarycentricEmbedding.interpolation_matrix` 返回 `pypgo.sparse.SparseMatrix`；`interpolation_matrix_coo()` 保留为 convenience alias，内部调用 `.interpolation_matrix.to_coo()`，避免破坏已写入 Phase 2 plan 的示例。

测试要求：

- `SparseMatrix.shape/nnz/to_coo()` 对小矩阵返回正确
- COO index dtype 为 Python 侧 `np.int64` 或 `np.int32` 之一并在文档中固定；value dtype 为 `float64`
- `BarycentricEmbedding.interpolation_matrix` 返回 `SparseMatrix`
- `BarycentricEmbedding.interpolation_matrix_coo()` 与 `.interpolation_matrix.to_coo()` 完全一致

### GIL release policy

所有长耗时 C++ kernel binding 必须按同一模式释放 GIL：

```cpp
m.def("heavy_call", [](const Input &input) {
  // 1. 在持有 GIL 时完成 nb::object / ndarray 解析和 Python 参数校验
  auto coreInput = copyAndValidate(input);

  // 2. 只在纯 C++ 计算段释放 GIL
  Result result;
  {
    nb::gil_scoped_release release;
    result = runPureCppKernel(coreInput);
  }

  // 3. 回到持有 GIL 状态后做 nb::cast / Python object 构造
  return wrapResult(std::move(result));
});
```

必须释放 GIL 的 M1 binding：

- `.veg` / `.obj` read-write：`readVegFile`、`writeVegFile`、`read_obj`、`write_obj`
- mesher / shape factory 中可能耗时的生成：`cubic_mesher`、`tet_mesher`
- surface / embedding：`extract_surface_mesh`、`BarycentricEmbeddingCore` 构造、`.interpolation_matrix` 构造、`.deform`
- Phase 3 自相交：`check_self_intersections`
- Phase 4 solver-ready 构造：`create_simulation_mesh_from_volume`、`create_simulation_mesh_from_shell`

不得释放 GIL 的区域：

- 访问 `nb::object`、`nb::ndarray` Python buffer metadata、`nb::cast`、Python callback 或 Python exception 构造
- 返回对象包装和 dataclass/material payload bridge

测试要求：

- 对至少一个可控长任务加 Python smoke test：后台 Python thread 递增 counter，主线程调用释放 GIL 的 `_core` kernel，确认 counter 在 kernel 执行期间有进展
- 所有 release-GIL wrapper 的异常路径仍能变成 Python exception，不能 terminate process

---

# Phase 2: Multi-Material VolumeMesh + Surface + Embedding

> **状态：** 设计中，尚未实现

## 目标

在 Phase 1 单材料 `VolumeMesh` 基础上，暴露：

1. 多 material / set / region 的 `VolumeMesh` 构造
2. Surface mesh extraction
3. Barycentric embedding / interpolation matrix（surface ↔ volume DOF 映射）+ 最小 `pypgo.sparse.SparseMatrix` COO 互操作
4. Mesher（tetgen / tetwild / cubic voxelizer）+ shape factory（box / sphere / cylinder / torus）

## 设计决策（review 后修订）

本节定义本 Phase 的边界。每一条对应一个曾经考虑过、但被放弃的方案。

1. **Python material 是纯数据载体，不绑 C++ Material 类。** Phase 2 公开 `ENuMaterial`、`MooneyRivlinMaterial`、`OrthotropicMaterial` 三个 dataclass，对应 Vega 现有 `Material::ENU / MOONEYRIVLIN / ORTHOTROPIC`。binding 函数按值接收这些 payload，由 C++ core adapter 构造对应 Vega material；Python 不持有任何 C++ Material 指针。这避免 Python 同时绑两个 C++ 类型体系（Vega 层 `VolumetricMesh::Material` vs Simulation 层 material），后者由 solver 使用。
2. **不重写 `.veg` parser，统一 I/O 命名。** 保留 C++ `VolumetricMeshParser` 作为唯一权威。C++ core 侧新增薄 adapter `readVegFile` / `writeVegFile`（**唯一** POD `.veg` reader/writer），一次性吐 / 写 `(TetMeshData | CubicMeshData, materials, sets, regions)` POD，立即释放 Vega 对象；Python binding 只负责把这个 POD adapter 映射到 `pypgo.mesh.veg.read_veg` / `write_veg`。同时移除 M1 的 `read_veg_geo` / `write_veg_geo`（单材料快捷 API），把 `read_obj_geo` / `write_obj_geo` rename 为 `pypgo.mesh.read_obj` / `write_obj`。Phase 2 完成后 `.veg` 入口只在 `pypgo.mesh.veg`，surface OBJ 入口只在 `pypgo.mesh`。避免 `.15g` 精度、binary `.vegb`、`*INCLUDE` 等 edge case 重新踩雷。
3. **不提取嵌套类。** `VolumetricMesh::Set/Material/Region` 保持嵌套。nanobind 完全支持嵌套类，binding TU 用 `using` alias 简化书写。外部调用点几乎为零，机械 rename 无收益、反而引入 Vega 老代码改动风险。
4. **不引入 Builder pattern。** `VolumeMesh(data, regions=[...])` 单构造函数支持多材料；partition 校验在 Python `__init__` 内做。Builder 是 Java 风格，Python 用户不需要 fluent chain，所有 regions 一次列出反而更利于 code review。
5. **Python `read_veg` 返回 `VegFile`，不返回裸 tuple。** `VegFile` 包含 `mesh_data/materials/sets/regions`，其中 `materials` 保留具体 material dataclass 类型，`sets` 包含隐式 `allElements`，`regions` 使用 0-based `material_index/set_index`，与 C++ adapter/Vega 语义一致。`VegFile.first_material()` 是单材料便捷路径；`VegFile.to_volume_regions()` 是多材料构造便捷路径，返回 `VolumeMesh(mesh_data, regions=...)` 所需的 `list[tuple[str, MaterialLike, list[int]]]`。
6. **`VolumeMesh` 当前 wrap Vega 层 `VolumetricMeshes::TetMesh`/`CubicMesh`，不是 `SimulationMesh`。** solver-ready 接入统一放到 Phase 4 的 `pypgo.sim.SimulationMesh` factory API：`SimulationMesh.create_volumetric(volume_mesh)` 显式把 Vega volume 转成 Simulation 层对象。`VolumeMesh` 不暴露 `.to_simulation_mesh()`，避免 Vega wrapper 泄露 Simulation 层细节。docstring 需明确："这是 Vega FEM 层 wrapper；需要 solver-ready mesh 时调用 `pypgo.sim.SimulationMesh.create_volumetric(volume)`。"
7. **Vega 层的杂乱不在本 Phase 整理范围。** `volumetricMesh.h/cpp` 2770 行、两套 Material 体系、嵌套类、C-style API ——这些都不阻塞 Python binding，因为 binding 接触面已经被 `MeshData<K>` 屏蔽。Material 体系统一是独立 C++ 重构任务，与本 Phase 解耦。
8. **Binding 基础设施纳入 M1，不推迟到 M2。** Dense Eigen/NumPy helper、最小 `SparseMatrix` wrapper、以及 `gil_scoped_release` policy 是 Phase 2/3/4 的共同依赖。Phase 2 的 embedding 直接返回 `pypgo.sparse.SparseMatrix`，COO 作为互操作格式保留；长耗时 C++ binding 必须显式释放 GIL。

## C++ 两层 Architecture（背景）

Python API 涉及的 C++ 对象分为两层，`load*Mesh()` factory 桥接：

```
Vega FEM 层 (VolumetricMeshes::)           Simulation 层 (SolidDeformationModel::)
────────────────────────────               ─────────────────────────────────────
TetMesh   (geometry + Material)       →    loadTetMesh()    → SimulationMesh (TET)
CubicMesh (geometry + Material)       →    loadCubicMesh()  → SimulationMesh (CUBIC)
                                           loadShellMesh(TriMeshGeo, mat)
                                                            → SimulationMesh (SHELL)
```

- Vega 层有 `.veg` I/O 和 multi-material（Set/Region/Material 嵌套类）
- Simulation 层是 solver/FEM/energy 的入口
- 两层各有 ENu material 类型，**互不复用**（`VolumetricMeshes::ENuMaterial` ≠ `SimulationMeshENuMaterial`）
- 本 Phase `VolumeMesh` 仅 wrap Vega 层；Phase 4 在不改变 Vega wrapper 语义的前提下新增 `pypgo.sim.SimulationMesh.create_volumetric(volume)`。Shell 没有 Vega 层，Phase 4 用 `pypgo.sim.SimulationMesh.create_shell(surface, material)` 直接生成 `SimulationMesh`。

## C++ 侧改动

控制在最小范围：新增一个 C++ POD `.veg` adapter 和对应 binding。不动 `volumetricMesh.h/cpp`，不重写 parser，不提取嵌套类。

### 2.1 `create_volume_mesh_multi` binding

`TetMesh` / `CubicMesh` 已有多材料 ctor（`tetMesh.h:91-95`、`cubicMesh.h:103-107`）。binding 层加 thin wrapper，并复用 2.2 的 core POD 类型：

```cpp
// src/python/pypgo/bindings/mesh_bindings.cpp 中新增

namespace VM = pgo::VolumetricMeshes;
using MeshSet = VM::VolumetricMesh::Set;        // 复用嵌套类
using MeshRegion = VM::VolumetricMesh::Region;
using MaterialPayload = VM::VegMaterialPayload;
using SetPayload = VM::VegSetPayload;
using RegionPayload = VM::VegRegionPayload;

std::shared_ptr<VolumeMeshCore> create_volume_mesh_multi(
    const nb::object &meshDataObj,
    const std::vector<MaterialPayload> &materials,
    const std::vector<SetPayload> &sets,
    const std::vector<RegionPayload> &regions);
```

实现步骤：

1. 用 `std::visit(VegMaterialPayload)` 构造 `std::vector<std::unique_ptr<VM::VolumetricMesh::Material>>`
2. 把 `SetPayload` 转换成 `std::vector<MeshSet>`
3. 把 `RegionPayload` 转换成 `std::vector<MeshRegion>`
4. 调现有多材料 `TetMesh(...)` / `CubicMesh(...)` ctor（ctor 内部 deep-copy 这些临时对象）
5. 返回 `VolumeMeshCore` 持有 `unique_ptr<TetMesh/CubicMesh>`

### 2.2 C++ `readVegFile` / `writeVegFile` POD adapter

新增 core adapter，位置建议为：

- `src/core/volumetricMesh/vegFile.h`
- `src/core/volumetricMesh/vegFile.cpp`
- `src/core/volumetricMesh/CMakeLists.txt` 加入对应源文件

这个 adapter 是 C++ API，不依赖 nanobind，不返回 `VolumetricMesh` 指针，也不把 Vega 嵌套类暴露给调用者。它只负责 path ↔ POD，方便 C++ 单测和未来非 Python 调用点复用。

设计决策：`readVegFile` 不构造完整 `TetMesh` / `CubicMesh`，而是实现一个 lightweight payload loader，直接读取到 `VegFilePayload`，避免 `.veg` I/O 先构造 Vega mesh 再导出 POD 的峰值内存和重复拷贝。这个 lightweight loader 必须参考现有 `VolumetricMesh::loadFromAscii` / `loadFromBinaryGeneric` 的语义实现，不重新设计 grammar，也不偏离现有 loader 对 `.veg` / `.vegb` / `*INCLUDE` / orthotropic subtype / set-region fallback 的行为。

`writeVegFile` 暂时保留 thin adapter：从 `VegFilePayload` 构造临时 `TetMesh` / `CubicMesh`，再调用 `saveToAscii(path.c_str())`。写路径先复用 Vega writer，避免同步维护 ASCII serialization；如果后续写入成为性能瓶颈，再单独提取 POD writer。

```cpp
// src/core/volumetricMesh/vegFile.h 中新增

#include "meshData.h"

#include <array>
#include <filesystem>
#include <string>
#include <variant>
#include <vector>

namespace pgo::VolumetricMeshes
{
struct VegENuMaterialPayload {
  std::string name;
  double density = 1000.0;
  double E = 1e9;
  double nu = 0.45;
};

struct VegMooneyRivlinMaterialPayload {
  std::string name;
  double density = 1000.0;
  double mu01 = 0.0;
  double mu10 = 0.0;
  double v1 = 0.0;
};

struct VegOrthotropicMaterialPayload {
  std::string name;
  double density = 1000.0;
  double E1 = 0.0;
  double E2 = 0.0;
  double E3 = 0.0;
  double nu12 = 0.0;
  double nu23 = 0.0;
  double nu31 = 0.0;
  double G12 = 0.0;
  double G23 = 0.0;
  double G31 = 0.0;
  std::array<double, 9> R{
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0,
  };
};

using VegMaterialPayload = std::variant<
  VegENuMaterialPayload,
  VegMooneyRivlinMaterialPayload,
  VegOrthotropicMaterialPayload>;

struct VegSetPayload {
  std::string name;
  std::vector<int> elements;
};

struct VegRegionPayload {
  int materialIndex = 0;
  int setIndex = 0;
};

using VegMeshData = std::variant<pgo::Mesh::TetMeshData, pgo::Mesh::CubicMeshData>;

struct VegFilePayload {
  VegMeshData meshData;
  std::vector<VegMaterialPayload> materials;
  std::vector<VegSetPayload> sets;
  std::vector<VegRegionPayload> regions;
};

VegFilePayload readVegFile(const std::filesystem::path &path);
void writeVegFile(const std::filesystem::path &path, const VegFilePayload &payload);
}  // namespace pgo::VolumetricMeshes
```

实现：

- `readVegFile`：用 `VolumetricMesh::getFileFormatTypeByExt(path.string().c_str())` 决定 ASCII / BINARY，未知扩展按现有 `VolumetricMesh` 行为当作 ASCII；再用轻量 reader 直接填充 `VegFilePayload`。ASCII reader 参考 `VolumetricMesh::loadFromAscii`，复用 `VolumetricMeshParser` 以保留 `*INCLUDE` 行为；binary reader 参考 `VolumetricMesh::loadFromBinaryGeneric` 的字段顺序和 validation。读入结束后执行与 `assignMaterialsToElements` 等价的 unassigned fallback：如果 regions 没覆盖全部元素，按现有 loader 行为补默认 material / set / region，而不是把现有合法文件读成错误。
- `readVegFile` material 拷贝规则：`Material::ENU` 通过 `downcastENuMaterial` 追加 `VegENuMaterialPayload`；`Material::MOONEYRIVLIN` 通过 `downcastMooneyRivlinMaterial` 追加 `VegMooneyRivlinMaterialPayload`；`Material::ORTHOTROPIC` 通过 `downcastOrthotropicMaterial` 追加 `VegOrthotropicMaterialPayload`。未知 material type 抛 `std::runtime_error`。
- `readVegFile` 不调用 `TetMesh(path)` / `CubicMesh(path)`。测试需要用一个足够大的 synthetic fixture 或 instrumentation 证明 reader 不走完整 mesh constructor 路径；行为一致性用现有 `TetMesh(path)` / `CubicMesh(path)` 作为 oracle 比较 geometry/material/set/region POD。
- `writeVegFile`：对 `VegMaterialPayload` 做 `std::visit`，分派构造 `VM::ENuMaterial`、`VM::MooneyRivlinMaterial` 或 `VM::OrthotropicMaterial`，再构造 `std::vector<VM::VolumetricMesh::Set>`、`std::vector<VM::VolumetricMesh::Region>`，根据 `std::variant` 中的 `TetMeshData` / `CubicMeshData` 构造临时 `TetMesh` / `CubicMesh`，调用 `saveToAscii(path.string().c_str())`，立即释放。
- 错误统一抛 `std::runtime_error`，包括未知 element type、读取失败、保存失败、material / set / region index 越界；catch 现有 parser/loader 风格的 integer throw 并转成带 path 的 `runtime_error`。

### 2.3 `read_veg` / `write_veg` binding（唯一 Python veg I/O 入口）

Python binding 调用 2.2 的 `readVegFile` / `writeVegFile`，不在 binding TU 内重复实现 parser、material/set/region 拷贝规则或 `TetMesh` / `CubicMesh` 构造分派。

```cpp
// src/python/pypgo/bindings/mesh_geo_bindings.cpp 中新增
// 需要 include <nanobind/stl/variant.h>，让 std::variant material payload
// 能按 concrete alternative 转成对应 Python core payload object。

struct VegPayload {
    nb::object mesh_data;             // TetMeshDataCore 或 CubicMeshDataCore
    std::vector<VM::VegMaterialPayload> materials;
    std::vector<VM::VegSetPayload> sets;
    std::vector<VM::VegRegionPayload> regions;
};

VegPayload read_veg(const std::string &path);

void write_veg(
    const std::string &path,
    const nb::object &meshDataObj,
    const std::vector<VM::VegMaterialPayload> &materials,
    const std::vector<VM::VegSetPayload> &sets,
    const std::vector<VM::VegRegionPayload> &regions);
```

实现：

- `read_veg`：调用 `VM::readVegFile(path)`，用 `std::visit` 把 `VegMeshData` cast 成 `TetMeshDataCore` 或 `CubicMeshDataCore`，其余 POD 字段按值返回给 Python。
- material payload binding 暴露三个 concrete core classes：`VegENuMaterialPayloadCore`、`VegMooneyRivlinMaterialPayloadCore`、`VegOrthotropicMaterialPayloadCore`。C++ core 仍使用 `std::variant`；Python wrapper 通过 `isinstance` 区分，不使用 `type` enum 字段。
- `write_veg`：从 `nb::object` 判定 `TetMeshDataCore` / `CubicMeshDataCore`，组装 `VM::VegFilePayload`，调用 `VM::writeVegFile(path, payload)`。
- **M1 的 `read_veg_geo` / `write_veg_geo` binding 一并删除**——`read_veg` 是 veg 文件唯一入口，单材料用户拿到 `VegFile` 后用 `.first_material()` 提取。

**OBJ 也同步 rename**：`read_obj_geo` / `write_obj_geo` → `read_obj` / `write_obj`（接受/返回 `TriMeshData`，签名不变）。

### 2.4 Surface extraction + Barycentric embedding binding

```cpp
// src/python/pypgo/bindings/mesh_bindings.cpp 中新增

Mesh::MeshData<3> extract_surface_mesh(const VolumeMeshCore &vm, bool triangulate);

class BarycentricEmbeddingCore {
public:
    BarycentricEmbeddingCore(
        const std::vector<double> &targetLocationsFlat,  // 3 * m
        const VolumeMeshCore &volumeMesh);

    SparseMatrixCore interpolation_matrix() const;

    // convenience alias，内部等价于 interpolation_matrix().toCOO()
    std::tuple<std::vector<int>, std::vector<int>, std::vector<double>>
    interpolation_matrix_coo() const;

    std::vector<double> deform(const std::vector<double> &volumeDispFlat) const;

private:
    std::unique_ptr<InterpolationCoordinates::BarycentricCoordinates> coords_;
    int numTargetLocations_;
    int numVolumeVertices_;
};
```

底层：

- `extract_surface_mesh` → `VolumetricMeshes::GenerateSurfaceMesh::computeMesh()` → `TriMeshGeo` → `toMeshData()`
- `BarycentricEmbeddingCore` ctor → `BarycentricCoordinates(numLoc, locFlat, vm.getVM())`
- `interpolation_matrix` → `generateInterpolationMatrix()` → owned `SparseMatrixCore`
- `interpolation_matrix_coo` → `interpolation_matrix().toCOO()`
- `deform` → 现有 `deform()` 接口

### 2.5 Mesher + shape factory binding

直接 wrap 现有静态库入口，零 C++ 改动。

```cpp
// src/python/pypgo/bindings/mesh_geo_bindings.cpp 中新增

m.def("cubic_mesher", &cubic_mesher_bind);    // (triMeshDataCore, resolution)
                                              //   → CubicMeshDataCore
m.def("tet_mesher",   &tet_mesher_bind);      // (triMeshDataCore, backend:str,
                                              //    config:dict)
                                              //   → TetMeshDataCore
m.def("has_tetwild",  []() {
    return static_cast<bool>(PGO_TET_MESHER_USE_TET_WILD);
});

m.def("create_box_mesh",      &create_box_bind);
m.def("create_sphere_mesh",   &create_sphere_bind);
m.def("create_cylinder_mesh", &create_cylinder_bind);
m.def("create_torus_mesh",    &create_torus_bind);
```

- `tet_mesher_bind` 接 `backend: str` + `config: dict`，分派到 `tet_mesher::generateTetgenMesh` / `generateTetwildMesh`。tetwild 不可用时（`PGO_TET_MESHER_USE_TET_WILD=0`）抛 `std::runtime_error("tetwild backend is not available")`。
- `create_*` 调用 `Mesh::createBoxMesh(...)` 等 `createTriMesh.h` 入口，结果 `.toMeshData()` 返回 `MeshData<3>`。

## Python 侧设计

### 2.6 Material 数据载体

```python
# pypgo/mesh.py
from dataclasses import dataclass
from typing import ClassVar, Literal

@dataclass
class ENuMaterial:
    """Linear isotropic material — pure Python data carrier.

    属性值在 VolumeMesh 构造或 write_veg 时按值传递到 _core，
    binding 在 TU 内构造对应的 Vega 或 Simulation Material。
    Python 不持有任何 C++ Material 指针。
    """
    name: str = "defaultMaterial"
    density: float = 1000.0
    E: float = 1e9
    nu: float = 0.45
    type: ClassVar[Literal["enu"]] = "enu"

    @property
    def lam(self) -> float:
        return self.E * self.nu / ((1 + self.nu) * (1 - 2 * self.nu))

    @property
    def mu(self) -> float:
        return self.E / (2 * (1 + self.nu))

    def __repr__(self):
        return (f"ENuMaterial({self.name!r}, E={self.E:.3g}, "
                f"nu={self.nu}, density={self.density})")

@dataclass
class MooneyRivlinMaterial:
    """Mooney-Rivlin hyperelastic material — pure Python data carrier."""

    name: str = "mooneyRivlinMaterial"
    density: float = 1000.0
    mu01: float = 0.0
    mu10: float = 0.0
    v1: float = 0.0
    type: ClassVar[Literal["mooney_rivlin"]] = "mooney_rivlin"

@dataclass
class OrthotropicMaterial:
    """Orthotropic material — pure Python data carrier."""

    name: str = "orthotropicMaterial"
    density: float = 1000.0
    E1: float = 0.0
    E2: float = 0.0
    E3: float = 0.0
    nu12: float = 0.0
    nu23: float = 0.0
    nu31: float = 0.0
    G12: float = 0.0
    G23: float = 0.0
    G31: float = 0.0
    R: tuple[float, float, float, float, float, float, float, float, float] = (
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0,
    )
    type: ClassVar[Literal["orthotropic"]] = "orthotropic"

MaterialLike = ENuMaterial | MooneyRivlinMaterial | OrthotropicMaterial

def _wrap_material_payload(m) -> MaterialLike:
    if isinstance(m, _core.VegENuMaterialPayloadCore):
        return ENuMaterial(m.name, density=m.density, E=m.E, nu=m.nu)
    if isinstance(m, _core.VegMooneyRivlinMaterialPayloadCore):
        return MooneyRivlinMaterial(
            m.name, density=m.density, mu01=m.mu01, mu10=m.mu10, v1=m.v1)
    if isinstance(m, _core.VegOrthotropicMaterialPayloadCore):
        return OrthotropicMaterial(
            m.name, density=m.density,
            E1=m.E1, E2=m.E2, E3=m.E3,
            nu12=m.nu12, nu23=m.nu23, nu31=m.nu31,
            G12=m.G12, G23=m.G23, G31=m.G31,
            R=tuple(m.R))
    raise RuntimeError(f"Unexpected material payload from _core: {type(m).__name__}")

def _material_to_core_payload(m: MaterialLike):
    if isinstance(m, ENuMaterial):
        return _core.create_enu_material_payload(m.name, m.density, m.E, m.nu)
    if isinstance(m, MooneyRivlinMaterial):
        return _core.create_mooney_rivlin_material_payload(
            m.name, m.density, m.mu01, m.mu10, m.v1)
    if isinstance(m, OrthotropicMaterial):
        if len(m.R) != 9:
            raise ValueError("OrthotropicMaterial.R must contain 9 row-major values")
        return _core.create_orthotropic_material_payload(
            m.name, m.density,
            m.E1, m.E2, m.E3,
            m.nu12, m.nu23, m.nu31,
            m.G12, m.G23, m.G31,
            m.R)
    raise TypeError(f"unsupported material type: {type(m).__name__}")
```

- **没有 `_core_obj`**——material 只是 Python dataclass
- **不引入持有 C++ 指针的 `MeshMaterial` 基类**——类型分派按 dataclass type 转成 `_core.Veg*MaterialPayloadCore`，不通过 Python 虚函数或 C++ 对象生命周期
- `_material_to_core_payload(material)` 和 `_wrap_material_payload(payload)` 放在 `pypgo.mesh.veg` 内部，供 `VolumeMesh` 与 `read_veg` / `write_veg` 共用，避免多个 Python module 复制 material 转换逻辑
- `MaterialSpec` 作为 M1 兼容名，alias 到 `ENuMaterial` 一个 minor 版本

### 2.7 `MeshSet` / `MeshRegion`（dataclass）

只在 `VegFile` 字段中暴露。普通用户构造 `VolumeMesh` 不直接接触它们。

```python
from dataclasses import dataclass

@dataclass
class MeshSet:
    name: str
    elements: list[int]  # 0-based; __post_init__ 内 sorted + unique

    def __post_init__(self):
        self.elements = sorted(set(self.elements))

@dataclass
class MeshRegion:
    material_index: int
    set_index: int
```

### 2.8 `VolumeMesh` 多材料构造

```python
from pypgo.mesh.veg import ENuMaterial, MooneyRivlinMaterial, VolumeMesh

# 单材料（M1 行为，保持不变）
vol = VolumeMesh(tet_data, ENuMaterial("rubber", density=1000, E=1e9, nu=0.45))

# 多材料：传 regions list[tuple]；material 可以是 ENu / Mooney-Rivlin / Orthotropic
vol = VolumeMesh(tet_data, regions=[
    ("body",   ENuMaterial("rubber", density=1000, E=1e9,  nu=0.45), [0, 1, 2, 3]),
    ("insert", MooneyRivlinMaterial("insert", density=1200, mu01=1e5, mu10=2e5, v1=0.49), [4, 5]),
])
```

`__init__` 签名：

```python
def __init__(self, mesh_data, material=None, *, regions=None):
    if regions is None and material is not None:
        # 单材料路径：支持 ENu / Mooney-Rivlin / Orthotropic
        self._core_obj = _core.create_volume_mesh_multi(
            mesh_data._core_obj,
            [_material_to_core_payload(material)],
            [("allElements", list(range(mesh_data.num_elements)))],
            [(0, 0)],
        )
    elif regions is not None and material is None:
        # 多材料路径：partition 校验 → POD → _core
        materials_pod, sets_pod, regions_pod = _validate_and_split_regions(
            regions, mesh_data.num_elements)
        self._core_obj = _core.create_volume_mesh_multi(
            mesh_data._core_obj, materials_pod, sets_pod, regions_pod)
    else:
        raise TypeError(
            "VolumeMesh requires exactly one of `material=` or `regions=`")
```

**Partition 校验**（Python 层，`_validate_and_split_regions`）：

```
1. 同一 element 出现在两个 region   → ValueError("element {id} assigned to both '{a}' and '{b}'")
2. element 漏（不在任何 region）    → ValueError("element {id} not assigned to any region")
3. region 名字重复                   → ValueError("duplicate region name '{name}'")
4. 元素 index 超出范围               → ValueError("region '{name}' references element {id} out of [0, {N}))")
```

校验通过后，把 regions 拆成三个 POD list 喂给 `_core.create_volume_mesh_multi`。`_validate_and_split_regions` 接受的 material 类型为 `MaterialLike`，并用 `_material_to_core_payload` 转成 `_core.VegENuMaterialPayloadCore` / `_core.VegMooneyRivlinMaterialPayloadCore` / `_core.VegOrthotropicMaterialPayloadCore`；不允许传入裸 C++ material 或未注册 Python 类型。

**`VolumeMesh` 当前语义**：`._core_obj` wrap `VolumetricMeshes::TetMesh`/`CubicMesh`（Vega 层）。Phase 4 会增加 `pypgo.sim.SimulationMesh.create_volumetric(volume)`，但不会让 `VolumeMesh` lazy 持有 `SimulationMesh`，也不会在 `VolumeMesh` 上挂 `.to_simulation_mesh()`。本 Phase 不涉及 solver-ready 转换。docstring 必须明确这一点。

### 2.9 Surface Mesh Extraction

```python
surface_data = vol.extract_surface_mesh(triangulate=True)
# 返回 TriMeshData，直接 write_obj 或喂给 TriMeshGeo
```

实现：

```python
class VolumeMesh:
    def extract_surface_mesh(self, *, triangulate: bool = True) -> "TriMeshData":
        core_data = _core.extract_surface_mesh(self._core_obj, triangulate)
        return TriMeshData(core_data)
```

### 2.10 Barycentric Embedding / Interpolation Matrix

```python
from pypgo.mesh.geo import BarycentricEmbedding

embedding = BarycentricEmbedding(
    target_locations=surface_data.vertices,  # (m, 3) float64
    volume_mesh=vol,
)

# M1：最小 sparse wrapper
P = embedding.interpolation_matrix          # pypgo.sparse.SparseMatrix
rows, cols, values = P.to_coo()

# convenience alias，内部等价于 embedding.interpolation_matrix.to_coo()
rows, cols, values = embedding.interpolation_matrix_coo()

# 直接 deform（不显式构造 matrix）
surface_disp = embedding.deform(volume_disp)  # (3*n,) → (3*m,)
```

Python wrapper 在 `pypgo.mesh.geo.BarycentricEmbedding`，内部 own `_core.BarycentricEmbeddingCore`。`interpolation_matrix` 返回 `pypgo.sparse.SparseMatrix`；COO tuple 只作为低层互操作和测试入口保留。

### 2.11 Mesh Package Layout + VEG / OBJ I/O API

```python
# pypgo/mesh/veg.py
from dataclasses import dataclass
from pypgo.mesh import TetMeshData, CubicMeshData

@dataclass
class VegFile:
    mesh_data: TetMeshData | CubicMeshData
    materials: list[MaterialLike]
    sets: list[MeshSet]
    regions: list[MeshRegion]

    def first_material(self) -> MaterialLike:
        if len(self.materials) != 1:
            raise ValueError(
                f"expected exactly one material, got {len(self.materials)}")
        return self.materials[0]

    def to_volume_regions(self) -> list[tuple[str, MaterialLike, list[int]]]:
        return [
            (
                self.sets[region.set_index].name,
                self.materials[region.material_index],
                list(self.sets[region.set_index].elements),
            )
            for region in self.regions
        ]


def read_veg(path: str) -> VegFile:
    """Read .veg with full multi-material data via C++ parser.

    Returns a VegFile, not a tuple. The returned sets include Vega's implicit
    allElements set, and regions use 0-based material/set indices.
    """
    payload = _core.read_veg(str(path))
    return VegFile(
        mesh_data=_wrap_mesh_data_core(payload.mesh_data),
        materials=[_wrap_material_payload(m) for m in payload.materials],
        sets=[MeshSet(s.name, list(s.elements)) for s in payload.sets],
        regions=[MeshRegion(r.material_index, r.set_index)
                 for r in payload.regions],
    )


def write_veg(path: str, veg: VegFile) -> None:
    _core.write_veg(
        str(path),
        veg.mesh_data._core_obj,
        [_material_to_core_payload(m) for m in veg.materials],
        [(s.name, s.elements) for s in veg.sets],
        [(r.material_index, r.set_index) for r in veg.regions],
    )


# pypgo/mesh/__init__.py
def read_obj(path: str) -> TriMeshData:
    """Read .obj surface mesh (rename from M1's read_obj_geo)."""
    return TriMeshData(_core.read_obj(str(path)))


def write_obj(path: str, surface_data: TriMeshData) -> None:
    """Write .obj surface mesh (rename from M1's write_obj_geo)."""
    if not isinstance(surface_data, TriMeshData):
        raise TypeError(...)
    _core.write_obj(str(path), surface_data._core_obj)
```

**移除的 M1 API**（破坏性变更，无 alias 保留）：

- `pypgo.io.read_veg_geo` — 由 `pypgo.mesh.veg.read_veg(path).first_material()` 替代
- `pypgo.io.write_veg_geo` — 调用方自己构造单材料 `VegFile` 后 `pypgo.mesh.veg.write_veg`
- `pypgo.io.read_obj_geo` / `write_obj_geo` — rename 为 `pypgo.mesh.read_obj` / `write_obj`
- `pypgo._core.read_veg_geo` / `write_veg_geo` / `read_obj_geo` / `write_obj_geo` — 同步从 binding 删除

M1 用户需要做的迁移：

```python
# Before (M1)
data, mat = pgo.io.read_veg_geo("box.veg")
tri = pgo.io.read_obj_geo("box.obj")

# After (Phase 2)
veg = pgo.mesh.veg.read_veg("box.veg")
data, mat = veg.mesh_data, veg.first_material()
volume = pgo.mesh.veg.VolumeMesh(veg.mesh_data, regions=veg.to_volume_regions())
tri = pgo.mesh.read_obj("box.obj")
```

### 2.12 Mesher API（`pypgo.tools.mesh`）

```python
from pypgo.tools.mesh import cubic_mesher, tet_mesher, has_tetwild

cubic_data = cubic_mesher(tri_data, resolution=20)         # → CubicMeshData
tet_data   = tet_mesher(tri_data, backend="tetgen",
                        config={"command": "pq1.2aY"})     # → TetMeshData

if has_tetwild():
    tet_data = tet_mesher(tri_data, backend="tetwild",
                          config={"lr": 0.1, "epsr": 1e-6,
                                  "stop_energy": 10, "max_threads": 8})
```

`backend="tetwild"` 在 `PGO_TET_MESHER_USE_TET_WILD=0` 时抛 `RuntimeError("tetwild backend not available")`。

### 2.13 createTriMesh 形状工厂（`pypgo.mesh`）

```python
from pypgo.mesh import create_box, create_sphere, create_cylinder, create_torus

tri_data = create_box(bmin=(0, 0, 0), bmax=(1, 1, 1))
tri_data = create_sphere(radius=1.0, axis_subdiv=32, height_subdiv=16)
tri_data = create_cylinder(radius=1.0, height=2.0, axis_subdiv=32, height_subdiv=1)
tri_data = create_torus(radial_res=32, tubular_res=16, radius=1.0, thickness=0.3)
```

全部返回 `TriMeshData`。

## 绑定清单

### Private `_core` 新增

```
# Multi-material 入口
VegENuMaterialPayloadCore
VegMooneyRivlinMaterialPayloadCore
VegOrthotropicMaterialPayloadCore
create_enu_material_payload(name, density, E, nu) → VegENuMaterialPayloadCore
create_mooney_rivlin_material_payload(name, density, mu01, mu10, v1) → VegMooneyRivlinMaterialPayloadCore
create_orthotropic_material_payload(name, density, E1, E2, E3, nu12, nu23, nu31, G12, G23, G31, R9) → VegOrthotropicMaterialPayloadCore
create_volume_mesh_multi(meshDataCore, materials, sets, regions) → VolumeMeshCore
read_veg(path) → VegPayload   # (mesh_data, materials, sets, regions) POD
write_veg(path, meshDataCore, materials, sets, regions)

# Surface + Embedding
SparseMatrixCore
  .rows() / .cols() / .nnz()
  .to_coo() → (rows, cols, values)
extract_surface_mesh(volumeMeshCore, triangulate) → TriMeshDataCore
BarycentricEmbeddingCore
  .interpolation_matrix() → SparseMatrixCore
  .interpolation_matrix_coo() → (rows, cols, values)  # convenience alias
  .deform(volume_disp_flat) → surface_disp_flat

# Mesher + shape factory
cubic_mesher(triMeshDataCore, resolution) → CubicMeshDataCore
tet_mesher(triMeshDataCore, backend:str, config:dict) → TetMeshDataCore
has_tetwild() → bool
create_box_mesh(bmin, bmax) → TriMeshDataCore
create_sphere_mesh(radius, axis_subdiv, height_subdiv) → TriMeshDataCore
create_cylinder_mesh(radius, height, axis_subdiv, height_subdiv) → TriMeshDataCore
create_torus_mesh(radial_res, tubular_res, radius, thickness) → TriMeshDataCore
```

### Private `_core` 保留

- `MaterialSpecCore`、`create_volume_mesh`、`load_volume_mesh`、`save_volume_mesh` 保留作为单材料路径的 thin wrapper

### Private `_core` 删除（M1 → Phase 2 破坏性变更）

- `read_veg_geo` / `write_veg_geo` —— 由 `read_veg` / `write_veg` 覆盖
- `read_obj_geo` / `write_obj_geo` —— rename 为 `read_obj` / `write_obj`（C++ 函数体内容不变，只改导出名）

### Public Python 模块

```python
pypgo.mesh:       TriMeshData, TetMeshData, CubicMeshData, MeshDataType,
                  create_box, create_sphere, create_cylinder, create_torus,
                  read_obj, write_obj
pypgo.mesh.geo:   TriMeshGeo, TetMeshGeo, CubicMeshGeo,
                  BarycentricEmbedding
pypgo.mesh.veg:   ENuMaterial, MooneyRivlinMaterial, OrthotropicMaterial,
                  MaterialLike, MeshSet, MeshRegion, VegFile,
                  VolumeMesh (扩展多材料构造),
                  read_veg, write_veg
pypgo.sparse:     SparseMatrix
pypgo.tools.mesh: cubic_mesher, tet_mesher, has_tetwild
pypgo._core:      上述 Core 类型 + factory 函数
```

注：`pypgo.mesh.MaterialSpec`（M1 单材料 value object）由 `pypgo.mesh.veg.ENuMaterial` 取代，但保留作为 alias 一个 minor 版本，避免破坏 M1 用户的代码。`pypgo.mesh_geo`、`read_veg_geo` / `write_veg_geo` / `read_obj_geo` / `write_obj_geo` **不**保留 alias——这是清晰的破坏性变更。

## 改动影响范围

| 改动 | 类型 | 影响 |
|------|------|------|
| `src/python/pypgo/mesh.py` 改成 `src/python/pypgo/mesh/__init__.py` | Python package layout | `pypgo.mesh` 继续公开 `MeshData`、OBJ I/O、shape factory；为 `pypgo.mesh.geo` / `pypgo.mesh.veg` 留出子模块 |
| `src/python/pypgo/mesh/geo.py` 新增 | Python package layout | 承载 `TriMeshGeo` / `TetMeshGeo` / `CubicMeshGeo` 和 `BarycentricEmbedding` |
| `src/python/pypgo/mesh/veg.py` 新增 | Python package layout | 承载 `VolumeMesh`、`.veg` I/O、volume materials、`VegFile`、`MeshSet`、`MeshRegion` |
| `src/python/pypgo/sim.py` 新增 | Python package layout | 承载 `SimulationMesh`、`KoiterStVKShellMaterial`、`ShellMaterialLike` 和 classmethod factory |
| `src/python/pypgo/mesh_geo.py` 删除 | Python package layout | 不保留 alias；迁移到 `pypgo.mesh` / `pypgo.mesh.geo` |
| `src/core/volumetricMesh/vegFile.h/cpp` 新增 `VegFilePayload`、lightweight `readVegFile`、thin `writeVegFile` | 新增 core adapter | read 直接填 POD，不构造 `TetMesh/CubicMesh`；write 暂时构造临时 Vega mesh 并复用 `saveToAscii` |
| `src/core/volumetricMesh/CMakeLists.txt` 加入 `vegFile.cpp` | 构建配置 | 让 core 和 Python binding 都能复用 adapter |
| `src/python/pypgo/bindings/eigen_numpy.h` 新增 dense Eigen/NumPy helper | 新增 binding 基础设施 | Phase 2/3/4 直接收发 Eigen dense 数据时统一 copy/shape/dtype 规则 |
| `src/python/pypgo/bindings/sparse_bindings.cpp` 新增 `SparseMatrixCore` | 新增 binding 基础设施 | `BarycentricEmbedding.interpolation_matrix` 返回 owned sparse wrapper，支持 deterministic COO export |
| `src/python/pypgo/bindings/module.cpp` 增加 `init_sparse_bindings(m)` | 扩展现有文件 | `_core` 统一导出 `SparseMatrixCore` |
| `src/python/pypgo/CMakeLists.txt` 加入 `sparse_bindings.cpp` 并链接 `eigenSupport`（如当前 target 未传递） | 构建配置 | sparse wrapper 使用 `EigenSupport::SpMatD` |
| `mesh_bindings.cpp` 追加 `create_volume_mesh_multi`、`extract_surface_mesh`、`BarycentricEmbeddingCore`、`SimulationMeshCore` 与 `create_simulation_mesh_from_*`（Phase 4） | 扩展现有文件 | 仅追加 nanobind 定义 |
| `mesh_geo_bindings.cpp` 追加 `read_veg`/`write_veg`、`create_*`、`cubic_mesher`/`tet_mesher`/`has_tetwild` | 扩展现有文件 | `read_veg`/`write_veg` 只桥接 `readVegFile`/`writeVegFile`，其余仅追加绑定 |
| Vega 层（`volumetricMesh.h/cpp` 等） | **不改** | 零 C++ 重构 |
| 嵌套类 `Set`/`Material`/`Region` | **不重构** | binding TU 内 `using` alias 即可 |
| `volumetricMeshParser.cpp` | **不改** | lightweight ASCII reader 复用 `VolumetricMeshParser`，但不修改 parser 本身 |

## 测试清单

### C++ 测试

- binding helper smoke test：dense Eigen/NumPy helper 对 `VectorXd`、`MatrixXd`、fixed-size matrix 的 shape/dtype 校验和 roundtrip 正确
- `SparseMatrixCore` 小矩阵 COO export：shape、nnz、0-based row/col/value 顺序 deterministic
- `create_volume_mesh_multi` 多材料 mesh 构造：material/set/region 数量、元素归属正确
- `VegFileGTest` 覆盖 `readVegFile` 多材料 `.veg` 文件 parse 正确（用 fixture 文件），并分别断言 ENu / Mooney-Rivlin / Orthotropic material 字段
- `VegFileGTest` 对同一 ASCII fixture 比较 `readVegFile(path)` 与 `TetMesh(path)` / `CubicMesh(path)` 导出的 geometry/material/set/region POD 一致，证明 lightweight reader 行为对齐现有 loader
- `VegFileGTest` 覆盖 `.vegb` fixture 或由测试临时生成 `.vegb` 后读取，确保 binary reader 字段顺序与 `loadFromBinaryGeneric` 一致
- `VegFileGTest` 覆盖没有完整 region 覆盖的合法 fixture，确认 lightweight reader 执行与 `assignMaterialsToElements` 等价的 unassigned fallback
- `VegFileGTest` 通过链接替身、计数 hook 或专门注释约束检查确认 `readVegFile` 不调用 `TetMesh(path)` / `CubicMesh(path)` 完整构造路径
- `VegFileGTest` 覆盖 `writeVegFile` → `readVegFile` roundtrip 数据一致，至少包含一个 ENu、一个 Mooney-Rivlin、一个 Orthotropic material
- binding 层 `read_veg` / `write_veg` 只需在 Python 测试中覆盖类型桥接，不重复测 parser 细节
- `extract_surface_mesh` 输出 vertex/triangle 数与 `GenerateSurfaceMesh::computeMesh` 一致
- `BarycentricEmbeddingCore::interpolation_matrix` 返回 owned `SparseMatrixCore`，shape `(3*m, 3*n)`、非零数 = m × element_K
- `BarycentricEmbeddingCore::deform` 输出长度 = 3*m
- `cubic_mesher` 体素化输出维度正确
- `tet_mesher` tetgen 输出维度正确；tetwild（如编译开启）输出维度正确
- `create_box/sphere/cylinder/torus` 顶点 / 三角形数符合参数

### Python 测试

- `pypgo.mesh.veg.ENuMaterial`、`MooneyRivlinMaterial`、`OrthotropicMaterial` 是 pure Python dataclass：无 `_core_obj` 属性；`ENuMaterial.lam/.mu` 公式正确
- `_material_to_core_payload(OrthotropicMaterial(..., R=bad_length))` 抛 `ValueError("OrthotropicMaterial.R must contain 9 row-major values")`
- `pypgo.mesh.veg.VolumeMesh(data, mat)` 单材料行为不变（M1 回归测试）
- `pypgo.mesh.veg.VolumeMesh(data, regions=[...])` 多材料构造正确
- `pypgo.mesh.veg.VolumeMesh(data)` 或 `VolumeMesh(data, mat, regions=...)` 抛 `TypeError`
- Partition 校验：
  - 冲突元素 → `ValueError`，错误消息包含 element id 和两个 region 名
  - 遗漏元素 → `ValueError`，错误消息包含 element id
  - 重名 → `ValueError`，错误消息包含 region 名
  - 元素 index 超界 → `ValueError`
- `pypgo.mesh.veg.read_veg(path)` 返回 `VegFile`，多材料 fixture 字段正确，并保留 material 具体 Python 类型
- `pypgo.mesh.veg.read_veg(path)` 不返回裸 tuple；`sets` 包含隐式 `allElements`，`regions` 使用 0-based `material_index/set_index`
- `pypgo.mesh.veg.write_veg(path, veg)` → `pypgo.mesh.veg.read_veg(path)` roundtrip 数据一致（ENU 的 E/nu/density、Mooney-Rivlin 的 mu01/mu10/v1、Orthotropic 的 E*/nu*/G*/R、set 名/元素列表、region 索引）
- `VegFile.first_material()` 单材料 mesh 正常；多材料 mesh 抛 `ValueError`
- `VegFile.to_volume_regions()` 单材料 mesh 返回一个覆盖 `allElements` 的 region tuple
- `VegFile.to_volume_regions()` 多材料 mesh 保留 set 名、material 值和 0-based element list；`VolumeMesh(veg.mesh_data, regions=veg.to_volume_regions())` 构造成功
- `pypgo.mesh.read_obj(path)` 返回 `TriMeshData`（验证 rename 后行为与 M1 `read_obj_geo` 等价）
- `pypgo.mesh.write_obj(path, tri_data)` 写出后 `pypgo.mesh.read_obj` roundtrip 一致
- `pypgo.mesh_geo` import 应抛 `ModuleNotFoundError` 或 `AttributeError`；`pypgo.io.read_veg_geo` / `write_veg_geo` / `read_obj_geo` / `write_obj_geo` import 应抛 `AttributeError`（验证清理完整）
- `vol.extract_surface_mesh()` 返回 `TriMeshData`，vertex/face count > 0
- `BarycentricEmbedding.interpolation_matrix` 返回 `pypgo.sparse.SparseMatrix`，`shape == (3*m, 3*n)`，`nnz` 与 COO 长度一致
- `BarycentricEmbedding.interpolation_matrix.to_coo()` 返回 0-based row/col/value NumPy arrays，value dtype 为 `float64`
- `BarycentricEmbedding.interpolation_matrix_coo()` 与 `.interpolation_matrix.to_coo()` 完全一致
- `BarycentricEmbedding.deform` 输出 shape 与目标点数一致
- 长耗时 `_core` binding 的 GIL smoke test：后台 Python thread 在主线程执行一个可控 C++ kernel 时能继续推进 counter
- `cubic_mesher(tri_data, resolution=8)` 输出 `CubicMeshData` 有效
- `tet_mesher(tri_data, backend="tetgen", config={...})` 输出 `TetMeshData` 有效
- `tet_mesher(backend="tetwild")` 在 tetwild 不可用时抛 `RuntimeError`
- `has_tetwild()` 返回值与编译配置一致
- `create_box/sphere/cylinder/torus` 输出 `TriMeshData` 有效

---

# Phase 3: Mesh Info & Quality Check

> **状态：** 设计中，尚未实现

## 目标

在 Phase 1 `MeshData` / `MeshGeo` 属性绑定基础上，用 Python + NumPy 实现体积信息查询和表面网格质量检查。除自相交检测需保留 C++ binding 外，全部 Python 原生。

## 3.1 Volume Info（MeshData 属性）

`TetMeshData` 和 `CubicMeshData` 新增 `volume` 和 `center_of_mass` 两个只读 property，纯 NumPy 向量化实现。

### TetMeshData.volume

```python
# pypgo/mesh_geo.py 的 TetMeshData / CubicMeshData 类中追加

@property
def volume(self) -> float:
    """Total enclosed volume of the mesh."""
    v = self.vertices[self.elements]          # (m, K, 3)
    if self._element_width == 4:  # tet
        edges = v[:, 1:] - v[:, 0:1]         # (m, 3, 3)
        vols = np.abs(np.linalg.det(edges)) / 6.0
    else:  # cubic, K=8
        # decompose each cube into 5 tets
        ...
    return float(vols.sum())
```

### TetMeshData.center_of_mass

```python
@property
def center_of_mass(self) -> np.ndarray:
    """Center of mass (assuming uniform density)."""
    return self.vertices.mean(axis=0)  # (3,)
```

或者更精确：按单元质心 + 体积加权平均。

**设计决策：** 这两个属性放在 `_MeshDataBase` 的 Python 层，不绑 C++。NumPy 向量化实现 3-5 行，100 万元素 < 0.1s。

### MeshData.bbox

所有 `MeshData` 类型共用的 bounding box。

```python
# _MeshDataBase 中追加

@property
def bbox(self) -> tuple[np.ndarray, np.ndarray]:
    """Axis-aligned bounding box (bmin, bmax)."""
    v = self.vertices
    return v.min(axis=0), v.max(axis=0)  # → ((3,), (3,))
```

### MeshData.take_elements

提取部分元素，返回同类型的新 `MeshData`。

```python
# _MeshDataBase 中追加

def take_elements(self, indices) -> "Self":
    """Return a new MeshData with only the specified elements.

    Args:
        indices: list or int array of element indices (0-based).
    Returns:
        Same type as self (TriMeshData / TetMeshData / CubicMeshData).
    """
    idx = np.asarray(indices, dtype=np.int64)
    return self.__class__(self.vertices, self.elements[idx])
```

### TriMeshData.concatenate

拼接多个同类型 mesh。

```python
# TriMeshData, TetMeshData, CubicMeshData 各自加 static method

@staticmethod
def concatenate(meshes: list["TriMeshData"]) -> "TriMeshData":
    """Merge multiple meshes into one. Vertices are concatenated, element indices are offset."""
    all_v = [m.vertices for m in meshes]
    vertices = np.concatenate(all_v, axis=0)
    offsets = np.cumsum([0] + [len(v) for v in all_v[:-1]], dtype=np.int64)
    elements = np.concatenate([
        m.elements + o for m, o in zip(meshes, offsets)
    ], axis=0)
    return TriMeshData(vertices, elements)
```

### TriMeshGeo face_areas / face_normals / vertex_normals

纯 `TriMeshGeo` 的 property（面 mesh 才有）。

```python
# TriMeshGeo 中追加

@property
def face_areas(self) -> np.ndarray:
    """Area of each triangle, shape (m,)."""
    v = self.vertices[self.triangles]          # (m, 3, 3)
    cross = np.cross(v[:, 1] - v[:, 0], v[:, 2] - v[:, 0])
    return np.linalg.norm(cross, axis=1) / 2.0

@property
def face_normals(self) -> np.ndarray:
    """Unit normal of each triangle, shape (m, 3)."""
    v = self.vertices[self.triangles]
    normals = np.cross(v[:, 1] - v[:, 0], v[:, 2] - v[:, 0])
    lengths = np.linalg.norm(normals, axis=1, keepdims=True)
    lengths[lengths == 0] = 1.0  # degenerate → zero normal
    return normals / lengths

@property
def vertex_normals(self) -> np.ndarray:
    """Per-vertex normals, area-weighted sum of adjacent face normals, shape (n, 3)."""
    fn = self.face_normals * self.face_areas[:, None]  # (m, 3) weighted
    vn = np.zeros((self.num_vertices, 3), dtype=np.float64)
    np.add.at(vn, self.triangles[:, 0], fn)
    np.add.at(vn, self.triangles[:, 1], fn)
    np.add.at(vn, self.triangles[:, 2], fn)
    lengths = np.linalg.norm(vn, axis=1, keepdims=True)
    lengths[lengths == 0] = 1.0
    return vn / lengths
```

全部纯 NumPy，每个 property 3-5 行，100 万面 < 0.5s。

## 3.2 Surface Mesh Quality Check

`check_surface_quality()` 函数，大部分检查纯 Python，自相交走 C++ binding。

```python
from pypgo.tools.mesh import check_surface_quality, QualityReport

report = check_surface_quality(tri_data, short_edge_threshold=1e-6)
```

### QualityReport

```python
@dataclass
class QualityReport:
    is_clean: bool                     # 通过全部检查
    degenerate_tris: list[int]         # 退化三角形索引（面积≈0）
    short_edges: list[tuple[int, int]] # 短边 (vtx_a, vtx_b)
    non_manifold_edges: list[tuple[int, int]]  # 非流形边
    flipped_tris: list[int]            # 法向量翻转的三角形
    has_self_intersections: bool       # 自相交（C++ CGAL exact-count）
```

### 各项实现方式

| 检查项 | 实现 | 复杂度 |
|--------|------|--------|
| 退化三角形 | `np.linalg.norm(np.cross(edges[:,0], edges[:,1]), axis=1) < eps` | O(m) |
| 短边 | 遍历每条边的两个端点，`np.linalg.norm` 算长度 | O(m) |
| 非流形边 | 构建 `{UEdgeKey: count}` dict，count > 2 为非流形 | O(m) |
| 法向量翻转 | 检测相邻三角形法向量点积 < 0 | O(m) |
| 自相交 | `_core.check_self_intersections(tri_data)` → bool | C++ CGAL |

## 3.3 绑定清单（Phase 3 新增）

### Private `_core` 新增

```
check_self_intersections(triMeshGeo) → bool   # CGAL exact-count，pypgo 必开启
```

### Public Python 新增

```python
# pypgo.mesh (MeshData 新增)
_MeshDataBase.bbox               → (bmin, bmax) tuple of (3,) arrays
_MeshDataBase.take_elements(ids) → SameType
TriMeshData.concatenate(meshes)  → TriMeshData
TetMeshData.concatenate(meshes)  → TetMeshData
CubicMeshData.concatenate(meshes) → CubicMeshData
TetMeshData.volume               → float
TetMeshData.center_of_mass       → (3,) np.ndarray
CubicMeshData.volume             → float
CubicMeshData.center_of_mass     → (3,) np.ndarray

# pypgo.mesh.geo (MeshGeo 新增)
TriMeshGeo.face_areas            → (m,) np.ndarray
TriMeshGeo.face_normals          → (m, 3) np.ndarray
TriMeshGeo.vertex_normals        → (n, 3) np.ndarray

# pypgo.tools.mesh 新增
check_surface_quality(tri_data, short_edge_threshold) → QualityReport
```

## 3.4 C++ 改动影响范围（Phase 3）

| 改动 | 类型 | 影响 |
|------|------|------|
| `check_self_intersections` binding | 新 binding | CGAL exact-count，pypgo 必开启 |

## 3.5 测试清单（Phase 3）

- `tri_data.bbox` 输出形状 `(2,3)`，bmin < bmax
- `tri_data.take_elements([0, 2])` 返回同类型，元素数正确
- `TriMeshData.concatenate([a, b])` 顶点拼接 + 索引偏移正确
- `tet_data.volume` 输出与 C++ `volumetricMeshInfo` 一致
- `cubic_data.volume` 输出正确
- `tet_data.center_of_mass` 形状 `(3,)`，值合理
- `tri_geo.face_areas` 形状 `(m,)`，面积 > 0
- `tri_geo.face_normals` 形状 `(m, 3)`，单位长度
- `tri_geo.vertex_normals` 形状 `(n, 3)`，单位长度
- `check_surface_quality(tri_data)` 对干净 mesh 返回 `is_clean=True`
- `check_surface_quality` 对退化 mesh 检测出 `degenerate_tris`
- `check_surface_quality` 检测出自相交 mesh 的 `has_self_intersections=True`

---

# Phase 4: Tet / Cubic / Shell Simulation Mesh API

> **状态：** 设计中，尚未实现

## 目标

在 Phase 2/3 的 mesh pipeline 之后，补齐 solver-ready 层：

1. 公开统一的 `SimulationMesh` Python wrapper，对应 C++ `SolidDeformationModel::SimulationMesh`
2. `pypgo.sim.SimulationMesh.create_volumetric(volume_mesh)`：Tet/Cubic Vega volume → Simulation mesh
3. `pypgo.sim.SimulationMesh.create_shell(surface, material)`：Tri surface + `KoiterStVKShellMaterial` → Simulation shell mesh

目标是让 Python 用户从 I/O / meshing / material region 一路走到 solver-ready mesh，而不把 Vega `VolumeMesh` 和 Simulation `SimulationMesh` 混成同一个对象。

## 4.1 关键设计决策

**统一暴露 `pypgo.sim.SimulationMesh`，但保留两层语义。** `pypgo.mesh.veg.VolumeMesh` 继续 wrap Vega 层 `VolumetricMeshes::TetMesh` / `CubicMesh`，负责 `.veg` I/O、sets/regions/material payload 和 embedding。Phase 4 新增 `SimulationMesh` classmethod factory，返回 solver-ready 的 `SimulationMesh`：

```
现有 C++ 路径（保持不变）：
TetMesh/CubicMesh (Vega) → loadTetMesh/loadCubicMesh → SimulationMesh (TET/CUBIC)
TriMeshGeo + shell mat  → loadShellMesh              → SimulationMesh (SHELL)
```

**Shell 没有 Vega 层，也不新增 public `ShellMesh` wrapper。** Vega 层从未有 `ShellMesh`，因为 Shell 没有 `.veg` I/O 需求。Python 侧直接用 `SimulationMesh.create_shell(surface, KoiterStVKShellMaterial(...))` 创建 solver-ready shell mesh；surface geometry 继续用 `TriMeshData` 表示。

**不新增 `VolumetricMeshes::ShellMesh`。** 强行造对称类只会增加无意义的 C++ wrapper。需要统一的是 public Python 的 solver-ready API，而不是底层 C++ 类型体系。

**`SimulationMesh` 不直接 public-construct。** 用户通过 `SimulationMesh.create_volumetric(volume_mesh)` 或 `SimulationMesh.create_shell(surface, material)` 创建它；`SimulationMesh(...)` 构造函数保持 private/internal，用于包装 `_core_obj`。这让 solver-ready conversion 明确属于 `pypgo.sim`，不会把 Simulation 层细节挂到 `VolumeMesh` 上。

**`pypgo.io` 不进入目标 public API。** 文件 I/O 按 domain 就近放置：OBJ surface mesh 用 `pypgo.mesh.read_obj/write_obj`，Vega volume 用 `pypgo.mesh.veg.read_veg/write_veg`，shell spec 用 `pypgo.sim.read_shell/write_shell`。不保留 `pypgo.io` alias，避免同一个文件 I/O namespace 同时承载 mesh data、Vega volume 和 simulation spec 三种语义。

**`.shell.json` 作为 lightweight shell spec I/O 保留。** 但它不是 `ShellMesh` wrapper，也不直接生成 `SimulationMesh`。`pypgo.sim.read_shell(path)` 返回 `(TriMeshData, ShellMaterialLike)`；`pypgo.sim.write_shell(path, surface, material)` 写出 `.shell.json` 和对应 `.obj`。用户再显式调用 `SimulationMesh.create_shell(surface, material)` 进入 solver-ready 层。

**Shell material model 必须显式。** `thickness/E/nu` 只是参数，不足以表达 solver 要用哪个 shell elastic model。Phase 4 初版公开 `KoiterStVKShellMaterial`，其 concrete dataclass / C++ payload alternative 就是模型选择；不要把 `create_simulation_mesh_from_shell(...)` 设计成一串裸 float。未来增加 fabric shell 时，追加新的 shell material dataclass 和 `ShellMaterialPayload` variant alternative。

**其他 shell material model 审计结果。** 当前 C++ `DeformationModelElasticMaterial` 里 shell 相关项只有 `KOITER_STVK` 和 `KOITER_FABRIC`。`KOITER_STVK` 有完整 `ElasticModel2DFundamentalFormsSTVK`、5 个 elastic parameters、`compute_d2psi_*_dparam` 导数，以及启用的 fd test 覆盖。`KOITER_FABRIC` 虽然有 `ElasticModel2DFundamentalFormsFabric` 和 12 个 elastic parameters，但当前实现仍不适合作为稳定 Python API 一起公开：`SimulationMeshKoiterFabricMaterial` 相关读取在 `DeformationModelManager` 里是注释状态，warp/weft 方向被硬编码为 `(1,0)/(0,1)`，fd test 中 `KOITER_FABRIC` 被注释掉，且 fabric model 没有 override `compute_d2psi_da_dparam` / `compute_d2psi_db_dparam`，所以 material-parameter Hessian coupling 会退化为空实现。Phase 4 stable API 只暴露 `koiter_stvk`；`koiter_fabric` 暂列 experimental/future work，等 C++ 侧参数载体、方向配置和 fd test 补齐后再公开。

**Volume material 支持范围必须显式。** Phase 2 的 `.veg` I/O 和 `VolumeMesh` 支持 `ENuMaterial` / `MooneyRivlinMaterial` / `OrthotropicMaterial`，但当前 C++ `loadTetMesh()` / `loadCubicMesh()` 只把 Vega `ENuMaterial` 转成 `SimulationMeshENuMaterial`。Phase 4 初版只承诺 ENu volume material 可转换；遇到 Mooney-Rivlin / Orthotropic volume material 时，`SimulationMesh.create_volumetric(volume_mesh)` 必须抛清楚的错误，不能让 C++ `downcastENuMaterial` 空指针路径崩溃。后续若要支持 Mooney-Rivlin simulation material，再单独扩展 material mapping。

## 4.2 Volume vs Shell Pipeline（背景）

| | Volume | Shell |
|---|---|---|
| Python pre-solver wrapper | `pypgo.mesh.veg.VolumeMesh` | 无；直接使用 `TriMeshData + KoiterStVKShellMaterial` |
| 中间 C++ 层 | Vega `TetMesh`/`CubicMesh` | 无 Vega 层 |
| Simulation conversion | `SimulationMesh.create_volumetric(volume_mesh)` | `SimulationMesh.create_shell(surface, material)` |
| C++ factory | `loadTetMesh` / `loadCubicMesh` | `loadShellMesh` |
| Solver material | `SimulationMeshENuMaterial`（Phase 4 初版仅 ENu volume） | `SimulationMeshENuhMaterial` |
| 变形度量 | 3D 变形梯度 F (3x3) | 2D 基本形式 (a + b) |
| 单元拓扑 | 4 节点 tet / 8 节点 cubic | 6 节点三角形壳元 |
| 质量来源 | 体积 × density | 面网格面积 × scale |
| Python material | `ENuMaterial` / `MooneyRivlinMaterial` / `OrthotropicMaterial` | `KoiterStVKShellMaterial` |

Python volume material dataclass（`ENuMaterial` / `MooneyRivlinMaterial` / `OrthotropicMaterial`）和 `KoiterStVKShellMaterial` **不共享基类**——属性集不同，应用层不同。`SimulationMesh.create_shell(...)` 只接受 `KoiterStVKShellMaterial`，类型系统天然阻止把 volume material 当作 shell material 使用。

## 4.3 `KoiterStVKShellMaterial`（模型明确的数据载体）

```python
# pypgo/sim.py
from dataclasses import dataclass
from typing import ClassVar, Literal

@dataclass
class KoiterStVKShellMaterial:
    """Koiter-StVK shell material — pure Python data carrier.

    model 明确对应 C++ DeformationModelElasticMaterial::KOITER_STVK。
    属性按值传递到 _core.create_simulation_mesh_from_shell，binding 在 TU 内构造
    SimulationMeshENuhMaterial。膜/弯曲刚度默认相同。
    """
    name: str
    thickness: float
    E_membrane: float
    nu_membrane: float
    E_bending: float | None = None
    nu_bending: float | None = None
    model: ClassVar[Literal["koiter_stvk"]] = "koiter_stvk"

    def __post_init__(self):
        self.name = str(self.name)
        self.thickness = float(self.thickness)
        self.E_membrane = float(self.E_membrane)
        self.nu_membrane = float(self.nu_membrane)
        self.E_bending = float(self.E_bending) if self.E_bending is not None else self.E_membrane
        self.nu_bending = float(self.nu_bending) if self.nu_bending is not None else self.nu_membrane

    def __repr__(self):
        return (f"KoiterStVKShellMaterial({self.name!r}, thickness={self.thickness}, "
                f"E_mem={self.E_membrane:.3g}, nu_mem={self.nu_membrane}, "
                f"E_bend={self.E_bending:.3g}, nu_bend={self.nu_bending})")

ShellMaterialLike = KoiterStVKShellMaterial

def _shell_material_to_core_payload(m: ShellMaterialLike):
    if isinstance(m, KoiterStVKShellMaterial):
        return _core.create_koiter_stvk_shell_material_payload(
            m.name,
            m.thickness,
            m.E_membrane, m.nu_membrane,
            m.E_bending, m.nu_bending)
    raise TypeError(f"unsupported shell material type: {type(m).__name__}")
```

**不引入 `ShellMaterial` 基类**——Phase 4 stable API 只有一个 concrete material，加基类是 YAGNI。类型扩展用 `ShellMaterialLike` union 和 `_core.ShellMaterialPayloadCore` variant；需要 fabric / 其他模型时再追加 dataclass。

`KoiterFabricShellMaterial` 暂不进入 public API。升格条件：

1. C++ 侧恢复/新增明确的 fabric simulation material payload，或在 Python shell payload 中显式携带 warp/weft 方向和 12 个 elastic parameters
2. 不再硬编码 warp/weft 方向，至少支持全局方向配置；更理想是 per-element direction
3. 补 `KOITER_FABRIC` fd test，并决定是否必须实现 `compute_d2psi_da_dparam` / `compute_d2psi_db_dparam`
4. Python 侧增加 `model == "koiter_fabric"` 的 solver construction 测试；如果届时新增 shell spec 持久化，再覆盖对应 roundtrip

## 4.4 `SimulationMesh`（solver-ready wrapper）

`SimulationMesh` 是 Phase 4 的统一 solver-ready Python wrapper。它不替代 `VolumeMesh` 或 `TriMeshData`，只表示已经进入 `SolidDeformationModel::SimulationMesh` 层的对象。

```python
veg = pypgo.mesh.veg.read_veg("box.veg")
volume = pypgo.mesh.veg.VolumeMesh(veg.mesh_data, regions=veg.to_volume_regions())
sim_volume = pypgo.sim.SimulationMesh.create_volumetric(volume)

shell_mat = pypgo.sim.KoiterStVKShellMaterial(
    "fabric", thickness=0.001, E_membrane=1e6, nu_membrane=0.4)
sim_shell = pypgo.sim.SimulationMesh.create_shell(surface_data, shell_mat)

sim_volume.mesh_type              # "tet" or "cubic"
sim_shell.mesh_type               # "shell"
sim_volume.num_vertices
sim_volume.num_elements
sim_volume.num_element_vertices   # 4 / 8 / 6
```

Python API 草案：

```python
class SimulationMesh:
    """Solver-ready mesh backed by SolidDeformationModel::SimulationMesh."""

    def __init__(self, _core_obj, *,
                 default_elastic_model: str | None = None,
                 default_elastic_parameters: tuple[float, ...] | None = None):
        self._core_obj = _core_obj
        self.default_elastic_model = default_elastic_model
        self.default_elastic_parameters = default_elastic_parameters

    @classmethod
    def create_volumetric(cls, volume_mesh: "pypgo.mesh.veg.VolumeMesh") -> "SimulationMesh":
        if not isinstance(volume_mesh, pypgo.mesh.veg.VolumeMesh):
            raise TypeError("create_volumetric expects pypgo.mesh.veg.VolumeMesh")
        return cls(_core.create_simulation_mesh_from_volume(volume_mesh._core_obj))

    @classmethod
    def create_shell(cls, surface: "pypgo.mesh.TriMeshData",
                     material: ShellMaterialLike) -> "SimulationMesh":
        if not isinstance(surface, pypgo.mesh.TriMeshData):
            raise TypeError("create_shell expects pypgo.mesh.TriMeshData")
        material_payload = _shell_material_to_core_payload(material)
        return cls(
            _core.create_simulation_mesh_from_shell(surface._core_obj, material_payload),
            default_elastic_model=material.model,
            default_elastic_parameters=(
                material.E_membrane, material.nu_membrane,
                material.E_bending, material.nu_bending,
                material.thickness))

    @property
    def mesh_type(self) -> str: ...

    @property
    def num_vertices(self) -> int: ...

    @property
    def num_elements(self) -> int: ...

    @property
    def num_element_vertices(self) -> int: ...
```

`SimulationMesh` 的 public 构造入口只通过 `create_volumetric(...)` / `create_shell(...)` 暴露；不要支持 `SimulationMesh(mesh_data, material)` 这种直接构造签名，避免用户绕过 factory 校验。`default_elastic_model/default_elastic_parameters` 是 Python metadata，给后续 deformation model / solver API 作为默认值；C++ `SimulationMesh` 本身仍不保存 `DeformationModelElasticMaterial`。

### C++ binding

```cpp
// src/python/pypgo/bindings/mesh_bindings.cpp 中新增

class SimulationMeshCore {
public:
    explicit SimulationMeshCore(
        std::unique_ptr<SolidDeformationModel::SimulationMesh> sim);

    int numVertices() const;
    int numElements() const;
    int numElementVertices() const;
    SolidDeformationModel::SimulationMeshType meshType() const;

    SolidDeformationModel::SimulationMesh *get();
    const SolidDeformationModel::SimulationMesh *get() const;

private:
    std::unique_ptr<SolidDeformationModel::SimulationMesh> sim_;
};

struct KoiterStVKShellMaterialPayload {
    std::string name;
    double thickness = 1e-3;
    double E_membrane = 1e6;
    double nu_membrane = 0.4;
    double E_bending = 1e6;
    double nu_bending = 0.4;
};

using ShellMaterialPayload = std::variant<
    KoiterStVKShellMaterialPayload>;

std::shared_ptr<SimulationMeshCore> create_simulation_mesh_from_volume(
    const VolumeMeshCore &volume);

std::shared_ptr<SimulationMeshCore> create_simulation_mesh_from_shell(
    const Mesh::MeshData<3> &data,
    const ShellMaterialPayload &material);
```

实现要求：

- `create_simulation_mesh_from_volume` 从 `VolumeMeshCore` 取出底层 `VolumetricMeshes::VolumetricMesh`，按 element type 分派到 `loadTetMesh` / `loadCubicMesh`（或复用现有 `makeSimulationMesh`）。
- 分派前必须扫描所有 element material：只有 `VolumetricMesh::Material::ENU` 可进入当前转换；其他类型抛 `std::runtime_error("SimulationMesh.create_volumetric currently supports only ENuMaterial; element {i} uses {type}")`。不要让 `loadTetMesh` / `loadCubicMesh` 内部的 `downcastENuMaterial` 空指针路径承担错误处理。
- `create_simulation_mesh_from_shell` 对 `ShellMaterialPayload` 做 `std::visit`。Phase 4 初版只支持 `KoiterStVKShellMaterialPayload`，构造 `Mesh::TriMeshGeo(data)`，用 `SimulationMeshENuhMaterial(E_membrane, nu_membrane, thickness)` 调用 `loadShellMesh`。
- 当前 `loadShellMesh` 不区分 membrane / bending 参数；`E_bending, nu_bending` 暂时只保存在 Python `KoiterStVKShellMaterial`，binding 用 membrane 参数进入 `SimulationMeshENuhMaterial`。未来 C++ 若支持分离 bending，再扩展此 adapter。
- `KoiterStVKShellMaterialPayload` 的 concrete type 就是 shell material model。不要额外传 `"koiter_stvk"` string 到 C++ factory；如果需要运行时字符串，只在 Python I/O 边界解析成 concrete dataclass。
- 不把 `KOITER_FABRIC` 塞进这个 variant，直到上述升格条件满足。当前 C++ `KOITER_FABRIC` 可以作为内部研究入口保留，但 Python stable API 不承诺它。

## 4.5 `SimulationMesh.create_volumetric(volume_mesh)`

```python
sim = pypgo.sim.SimulationMesh.create_volumetric(volume)
```

语义：

- `volume_mesh` 必须是 `pypgo.mesh.veg.VolumeMesh`，否则抛 `TypeError`
- 不改变 `VolumeMesh._core_obj`，也不 lazy 缓存 `SimulationMesh`
- 每次调用返回新的 `SimulationMesh` wrapper，底层 C++ `SimulationMesh` 独立持有
- Tet → `mesh_type == "tet"`，`num_element_vertices == 4`
- Cubic → `mesh_type == "cubic"`，`num_element_vertices == 8`
- 非 ENu volume material 明确抛错；这是 Phase 4 初版限制，不影响 Phase 2 的 `.veg` I/O 和 `VolumeMesh` 构造能力
- `VolumeMesh` 不新增 `.to_simulation_mesh()`，避免 pre-solver Vega wrapper 暴露 Simulation 层 API

## 4.6 `SimulationMesh.create_shell(surface, material)`

```python
from pypgo.sim import KoiterStVKShellMaterial, SimulationMesh

mat = KoiterStVKShellMaterial("fabric", thickness=0.001,
                              E_membrane=1e6, nu_membrane=0.4)
sim_shell = SimulationMesh.create_shell(tri_data, mat)

tri_data.num_vertices # int
tri_data.num_elements # int  (input triangles)
sim_shell.num_element_vertices # 6（Simulation shell element）
```

factory 校验（Python 层）：

- `surface` 必须是 `pypgo.mesh.TriMeshData` → 否则 `TypeError`
- `material` 必须是 `ShellMaterialLike`（Phase 4 初版即 `KoiterStVKShellMaterial`）→ 否则 `TypeError`（隐含阻止误传 `ENuMaterial`）

Python API 草案：

```python
class SimulationMesh:
    @classmethod
    def create_shell(cls, surface: TriMeshData,
                     material: ShellMaterialLike) -> "SimulationMesh":
        if not isinstance(surface, TriMeshData):
            raise TypeError("create_shell expects pypgo.mesh.TriMeshData")
        material_payload = _shell_material_to_core_payload(material)
        m = material
        return cls(_core.create_simulation_mesh_from_shell(
            surface._core_obj,
            material_payload),
            default_elastic_model=m.model,
            default_elastic_parameters=(
                m.E_membrane, m.nu_membrane,
                m.E_bending, m.nu_bending,
                m.thickness))
```

不新增 `ShellMesh` / `ShellMeshCore`。shell path 的 pre-solver geometry 就是 `TriMeshData`；C++ solver-ready 对象只在 `SimulationMesh.create_shell(...)` 时创建。

## 4.7 Shell spec I/O：`read_shell` / `write_shell`

OBJ 只存 geometry，thickness + material 需要额外存储。M1 保留 `.shell.json`，但只作为 lightweight spec I/O，不新增 `ShellMesh` / `ShellMeshCore`，也不让 read path 自动创建 solver-ready mesh。

推荐 public API：

```python
from pypgo.sim import KoiterStVKShellMaterial, SimulationMesh, read_shell, write_shell

mat = KoiterStVKShellMaterial("fabric", thickness=0.001,
                              E_membrane=1e6, nu_membrane=0.4)

write_shell("fabric.shell.json", tri_data, mat)

surface, mat = read_shell("fabric.shell.json")
sim_shell = SimulationMesh.create_shell(surface, mat)
```

Python API 草案：

```python
def read_shell(path: str | os.PathLike) -> tuple[pypgo.mesh.TriMeshData, ShellMaterialLike]:
    """Read a .shell.json spec and its referenced OBJ geometry."""
    spec_path = Path(path)
    spec = json.loads(spec_path.read_text())
    _validate_shell_spec_version(spec)

    geometry_path = (spec_path.parent / spec["geometry"]).resolve()
    surface = pypgo.mesh.read_obj(geometry_path)
    material = _shell_material_from_json(spec["material"])
    return surface, material


def write_shell(
    path: str | os.PathLike,
    surface: pypgo.mesh.TriMeshData,
    material: ShellMaterialLike,
    *,
    geometry_path: str | os.PathLike | None = None,
) -> None:
    """Write a .shell.json spec plus OBJ geometry."""
    spec_path = Path(path)
    geometry_path = _default_shell_geometry_path(spec_path, geometry_path)
    pypgo.mesh.write_obj(geometry_path, surface)
    spec = {
        "version": 1,
        "geometry": _relative_path_for_json(spec_path.parent, geometry_path),
        "material": _shell_material_to_json(material),
    }
    spec_path.write_text(json.dumps(spec, indent=2) + "\n")
```

默认 geometry 命名：

- `write_shell("fabric.shell.json", surface, mat)` 默认写 `fabric.obj`，JSON 中保存 `"geometry": "fabric.obj"`。
- `write_shell("scene/fabric.json", surface, mat)` 默认写 `scene/fabric.obj`。
- 如果传入 `geometry_path=...`，按该路径写 OBJ；JSON 优先保存相对 `.shell.json` 所在目录的 relative path，避免把用户机器上的绝对路径写进可复现实验文件。

`.shell.json` schema v1：

```json
{
  "version": 1,
  "geometry": "fabric.obj",
  "material": {
    "model": "koiter_stvk",
    "name": "fabric",
    "thickness": 0.001,
    "E_membrane": 1000000.0,
    "nu_membrane": 0.4,
    "E_bending": 1000000.0,
    "nu_bending": 0.4
  }
}
```

语义和错误处理：

- `read_shell` 只返回 `(surface, material)`，不返回 `SimulationMesh`，不隐藏 solver-ready conversion。
- `write_shell` 只接受 `TriMeshData` 和 `ShellMaterialLike`；误传 `TetMeshData` 或 volume material 时抛 `TypeError`。
- `material.model` 未知时抛 `ValueError("unsupported shell material model: ...")`。
- `version` 缺失或不是 `1` 时抛 `ValueError("unsupported shell spec version: ...")`。
- `geometry` 路径相对 `.shell.json` 所在目录解析；读不到 OBJ 时抛带完整 path 的 `FileNotFoundError` 或 `ValueError`。
- 这部分纯 Python 实现，复用 `pypgo.mesh.read_obj/write_obj`，无需新增 C++ binding。

## 4.8 绑定清单（Phase 4 新增）

### Private `_core` 新增

```
SimulationMeshCore
  .num_vertices() → int
  .num_elements() → int
  .num_element_vertices() → int
  .mesh_type() → SimulationMeshType
KoiterStVKShellMaterialPayloadCore
create_simulation_mesh_from_volume(volumeMeshCore)
  → SimulationMeshCore
create_koiter_stvk_shell_material_payload(name, thickness, E_mem, nu_mem, E_bend, nu_bend)
  → KoiterStVKShellMaterialPayloadCore
create_simulation_mesh_from_shell(triMeshData, shellMaterialPayload)
  → SimulationMeshCore
```

### Public Python 新增

```python
pypgo.sim:  SimulationMesh, KoiterStVKShellMaterial, ShellMaterialLike,
            read_shell, write_shell
```

**不新增任何 Vega 层 C++ 类、不修改 `loadTetMesh` / `loadCubicMesh` / `loadShellMesh` 签名。**

## 4.9 测试清单（Phase 4）

- `KoiterStVKShellMaterial.model == "koiter_stvk"`
- `KoiterStVKShellMaterial` 默认 `E_bending=E_membrane`, `nu_bending=nu_membrane`
- `KoiterStVKShellMaterial` 显式分离 bending 参数正常工作
- `_shell_material_to_core_payload(KoiterStVKShellMaterial(...))` 生成 Koiter-STVK shell payload，不丢 model/参数
- `KoiterFabricShellMaterial` 不存在于 stable public API
- `SimulationMesh.create_shell(tri_data, KoiterStVKShellMaterial(...))` 返回 `SimulationMesh`，`mesh_type == "shell"`，`num_element_vertices == 6`
- `SimulationMesh.create_shell(tri_data, ENuMaterial(...))` 抛 `TypeError`
- `SimulationMesh.create_shell(tet_data, KoiterStVKShellMaterial(...))` 抛 `TypeError`（非 TriMeshData）
- `SimulationMesh.create_shell(...).default_elastic_model == "koiter_stvk"`，`default_elastic_parameters == (E_mem, nu_mem, E_bend, nu_bend, thickness)`
- `write_shell("fabric.shell.json", tri_data, KoiterStVKShellMaterial(...))` 写出 `fabric.shell.json` 和默认 `fabric.obj`
- `read_shell("fabric.shell.json")` 返回 `(TriMeshData, KoiterStVKShellMaterial)`，不构造 `SimulationMesh`
- `read_shell` / `write_shell` roundtrip 保留 geometry、`model == "koiter_stvk"` 和 5 个 shell material 参数
- `read_shell` 遇到未知 `material.model` 或不支持的 `version` 抛 `ValueError`
- `write_shell(..., tet_data, KoiterStVKShellMaterial(...))` 抛 `TypeError`
- `SimulationMesh.create_volumetric(VolumeMesh(tet_data, ENuMaterial(...)))` 返回 `SimulationMesh`，`mesh_type == "tet"`，`num_element_vertices == 4`
- `SimulationMesh.create_volumetric(VolumeMesh(cubic_data, ENuMaterial(...)))` 返回 `SimulationMesh`，`mesh_type == "cubic"`，`num_element_vertices == 8`
- `SimulationMesh.create_volumetric(VolumeMesh(..., MooneyRivlinMaterial(...)))` 抛清楚错误，说明 Phase 4 初版仅支持 ENu volume simulation conversion
- multi-region ENu `.veg` → `VolumeMesh(..., regions=veg.to_volume_regions())` → `SimulationMesh.create_volumetric(...)` 构造成功
- `VolumeMesh` 不存在 `.to_simulation_mesh`
- `ShellMesh` 不存在于 stable public API
- `SimulationMesh` 不能直接用 `SimulationMesh(mesh_data, material)` public 构造

## 4.10 模块总览（最终，Phase 2 + 3 + 4 完成后）

```python
pypgo.mesh:       TriMeshData(.bbox, .take_elements, .concatenate),
                  TetMeshData(.volume, .center_of_mass, .bbox, .take_elements, .concatenate),
                  CubicMeshData(.volume, .center_of_mass, .bbox, .take_elements, .concatenate),
                  MeshDataType,
                  create_box, create_sphere, create_cylinder, create_torus,
                  read_obj, write_obj,
                  check_surface_quality, QualityReport
pypgo.mesh.geo:   TriMeshGeo(.face_areas, .face_normals, .vertex_normals),
                  TetMeshGeo, CubicMeshGeo,
                  BarycentricEmbedding
pypgo.mesh.veg:   ENuMaterial, MooneyRivlinMaterial, OrthotropicMaterial,
                  MaterialLike, MeshSet, MeshRegion,
                  VegFile, read_veg, write_veg,
                  VolumeMesh
pypgo.sim:        SimulationMesh, KoiterStVKShellMaterial, ShellMaterialLike
                  read_shell, write_shell
pypgo.sparse:     SparseMatrix
pypgo.tools.mesh: cubic_mesher, tet_mesher, has_tetwild,
pypgo._core:      VolumeMeshCore, SimulationMeshCore, MaterialSpecCore,
                  SparseMatrixCore,
                  KoiterStVKShellMaterialPayloadCore,
                  *MeshDataCore, *MeshGeoCore,
                  + create_*, read_*, write_*, check_*
```

## 4.11 示例 Notebook 更新

Phase 4 实现完成后必须同步更新 `pypgo/examples/mesh_api_demo.ipynb`，把 notebook 从旧 mesh API demo 升级为完整 M1 mesh pipeline demo。这个更新是交付物的一部分，不作为可选文档清理。

Notebook 至少覆盖：

- `pypgo.mesh.veg.read_veg(path)` 返回 `VegFile`，展示 `veg.mesh_data` / `veg.materials` / `veg.sets` / `veg.regions`
- 单材料路径：`VolumeMesh(veg.mesh_data, veg.first_material())`
- 多材料路径：`VolumeMesh(veg.mesh_data, regions=veg.to_volume_regions())`
- generic material 示例：`ENuMaterial` / `MooneyRivlinMaterial` / `OrthotropicMaterial` 的 dataclass 表达；说明 Phase 4 volume simulation conversion 初版仅支持 ENu
- `BarycentricEmbedding.interpolation_matrix` 返回 `pypgo.sparse.SparseMatrix`，展示 `.shape/.nnz/.to_coo()`
- `pypgo.sim.SimulationMesh.create_volumetric(volume)`：Tet/Cubic ENu volume → `SimulationMesh`
- `pypgo.sim.KoiterStVKShellMaterial` + `pypgo.sim.SimulationMesh.create_shell(surface, material)`：Tri surface → solver-ready shell
- `.shell.json` roundtrip：`pypgo.sim.write_shell(path, surface, material)` / `pypgo.sim.read_shell(path)`，再显式 `SimulationMesh.create_shell(surface, material)`
- 新 package layout：`pypgo.mesh` 放 `MeshData` / OBJ / shape factory，`pypgo.mesh.geo` 放 geometry façade，`pypgo.mesh.veg` 放 Vega volume，`pypgo.sim` 放 solver-ready mesh

更新后验证：

- 用项目推荐的 notebook 执行方式跑通 `pypgo/examples/mesh_api_demo.ipynb`（例如 `jupyter nbconvert --execute` 或 repo 现有 notebook 测试命令）
- notebook 不依赖用户本地私有路径；示例 mesh fixture 必须来自 repo 内测试/示例数据，或在 notebook 中用 public API 生成
- 如果 `tetwild` 等外部 mesher 不可用，相关 cell 必须 graceful skip，不影响 notebook 主流程

---

# 统一执行顺序

把 Phase 3（pure-Python NumPy 属性）提前——零 C++ 风险、立即提供 user value、且可在不重新 build C++ 的情况下完成。Phase 4 的 `SimulationMeshCore` / Shell path 可与 Phase 2 并行；Phase 4 的 multi-region volume conversion 测试依赖 Phase 2 完成。

```
─── Cross-Phase Binding Infrastructure ───
I-1.  新增 dense Eigen/NumPy helper，并补 roundtrip/shape/dtype 测试
I-2.  新增 SparseMatrixCore + pypgo.sparse.SparseMatrix，并补 COO export 测试
I-3.  为 Phase 2/3/4 新增长耗时 binding 套用 gil_scoped_release policy，并补 smoke test

─── Phase 3 (pure-Python NumPy properties) ───
P3-1.  _MeshDataBase.bbox / .take_elements
P3-2.  TriMeshData / TetMeshData / CubicMeshData.concatenate
P3-3.  TetMeshData.volume / .center_of_mass
P3-4.  CubicMeshData.volume / .center_of_mass
P3-5.  TriMeshGeo.face_areas / .face_normals / .vertex_normals
P3-6.  check_surface_quality + QualityReport（pure Python 检查项）
P3-7.  _core 绑定：check_self_intersections（CGAL exact-count）
P3-8.  Phase 3 测试

─── Phase 2 (multi-material + mesher + embedding) ───
P2-1.  _core 绑定：create_box/sphere/cylinder/torus
P2-2.  _core 绑定：cubic_mesher / tet_mesher / has_tetwild
P2-3.  C++ core adapter + _core 绑定：readVegFile/writeVegFile + read_veg/write_veg（同时 rename read_obj_geo→read_obj, write_obj_geo→write_obj；删 read_veg_geo / write_veg_geo）
P2-4.  _core 绑定：create_volume_mesh_multi
P2-5.  _core 绑定：extract_surface_mesh, BarycentricEmbeddingCore（返回 SparseMatrixCore）
P2-6.  Python package layout：pypgo.mesh package 化，新增 pypgo.mesh.geo / pypgo.mesh.veg（pypgo.mesh_geo 不保留 alias）
P2-7.  Python 层：ENuMaterial, MooneyRivlinMaterial, OrthotropicMaterial, MeshSet, MeshRegion, VegFile（dataclass，位于 pypgo.mesh.veg）
P2-8.  Python 层：VolumeMesh 多 region 构造 + partition 校验（位于 pypgo.mesh.veg）
P2-9.  Python 层：pypgo.mesh.veg.read_veg / write_veg、pypgo.mesh.read_obj / write_obj（破坏性变更，无 alias）
P2-10. Python 层：BarycentricEmbedding wrapper + pypgo.sparse.SparseMatrix exposure（位于 pypgo.mesh.geo）
P2-11. Python 层：cubic_mesher / tet_mesher / has_tetwild wrapper
P2-12. Python 层：create_box/sphere/cylinder/torus wrapper（位于 pypgo.mesh）
P2-13. Phase 2 测试（含 multi-material .veg fixture）

─── Phase 4 (Simulation Mesh API: Tet/Cubic/Shell) ───
P4-1.  _core 绑定：SimulationMeshCore + create_simulation_mesh_from_volume / create_simulation_mesh_from_shell
P4-2.  Python 层：pypgo.sim.SimulationMesh wrapper + create_volumetric / create_shell classmethod factories
P4-3.  Python 层：pypgo.sim.KoiterStVKShellMaterial + ShellMaterialLike（模型明确的数据载体）
P4-4.  Python 层：pypgo.sim.read_shell / write_shell（.shell.json + OBJ，纯 Python spec I/O）
P4-5.  Phase 4 测试（Tet/Cubic/Shell 都覆盖；确认 VolumeMesh 没有 to_simulation_mesh、ShellMesh 不进入 stable API）
P4-6.  更新并执行验证 pypgo/examples/mesh_api_demo.ipynb（完整 M1 mesh pipeline demo）
```

## 跨 Phase 注意事项

- **Material 体系**：Python `ENuMaterial` / `MooneyRivlinMaterial` / `OrthotropicMaterial` 和 `KoiterStVKShellMaterial` 都是数据载体，**不持有 C++ Material 指针**。Volume material 通过 `MaterialLike` union 转成 Vega material payload；Phase 4 的 volume simulation conversion 初版只支持 ENu。Shell material 通过 `ShellMaterialLike` / payload variant 显式携带 shell material model。
- **模块布局**：`pypgo.mesh` 是 mesh data / OBJ / shape factory 层；`pypgo.mesh.geo` 是 geometry façade / geometry algorithm 层；`pypgo.mesh.veg` 是 Vega volume / `.veg` / volume material 层；`pypgo.sim` 是 solver-ready simulation mesh 层。`pypgo.mesh_geo` 不保留 alias。
- **VolumeMesh 当前 wrap Vega 层**，不是 `SimulationMesh`。Phase 4 只在 `pypgo.sim.SimulationMesh` 增加 `create_volumetric(volume)`；`VolumeMesh` 本身仍负责 `.veg` / region / embedding 等 Vega 层能力，不暴露 `.to_simulation_mesh()`。`extract_surface_mesh` 和 `BarycentricEmbedding` 继续通过 Vega `VolumetricMesh` 接口工作。
- **SimulationMesh 是 solver-ready 层**：`SimulationMesh` 不做 `.veg` I/O，不保存 material payload schema，也不替代 `VolumeMesh`。`.shell.json` 是 `pypgo.sim.read_shell/write_shell` 的 lightweight spec I/O，返回/接收 `TriMeshData + ShellMaterialLike`，不会自动构造 `SimulationMesh`。后续 deformation model / solver API 应接受 `SimulationMesh`。
- **Koiter Fabric 暂不公开**：C++ 有 `KOITER_FABRIC` 和 `ElasticModel2DFundamentalFormsFabric`，但仍缺少稳定 public 参数载体、方向配置和启用测试。Phase 4 只公开 `KoiterStVKShellMaterial`。
- **示例 notebook 是交付物**：完成 Phase 2/3/4 后同步 `pypgo/examples/mesh_api_demo.ipynb`，并用 repo 内 fixture 或 public API 生成数据，确保用户能从 notebook 走完整 M1 mesh pipeline。
- **Dense Eigen/NumPy 转换统一入口**：M1 新增 `eigen_numpy.h` helper。兼容 NumPy 输入支持 safe zero-copy `Eigen::Map`；需要长期持有或 stride/dtype 不兼容时显式 copy；输出默认 owned copy 或 capsule-owned view；长期 core object 的 mutable zero-copy view 暂不公开。后续直接接收/返回 `Eigen::VectorXd/MatrixXd` 的 binding 不再各自手写 `nb::ndarray` 解析和 lifetime 管理。
- **Sparse matrix 表示**：M1 新增最小 `pypgo.sparse.SparseMatrix`。Phase 2 的 `BarycentricEmbedding.interpolation_matrix` 返回 wrapper；`.interpolation_matrix_coo()` 保留为 convenience alias，COO 接口继续作为 SciPy/测试互操作底层访问。
- **GIL 释放策略**：所有新增长耗时 C++ binding 必须在纯 C++ 段使用 `nb::gil_scoped_release`；进入/离开 Python object、NumPy buffer metadata、`nb::cast` 和 Python exception 构造时必须持有 GIL。
- **C++ 重构与 binding 解耦**：本文档不规划任何 Vega 层 C++ 重构。两套 Material 体系合并、`volumetricMesh.cpp` 拆分等清理工作是独立的 C++ refactor 任务，与 Python binding 进度无关。
