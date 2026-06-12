# `pypgo/mesh/volume/core.py` — VEG 数据模型、VolumeMesh 与 .veg I/O

> 源文件：`pypgo/mesh/volume/core.py`（239 行）。子包架构见 [overview.md](overview.md)。
>
> 两级抽象：`VegFile`（纯 Python 数据，自由编辑）↔ `VolumeMesh`（构造校验后的 C++ Vega `VolumetricMesh` 句柄）。I/O 与句柄构造全部委托绑定层 `src/python/pypgo/mesh/volume/core.cpp`。

## VEG 数据模型

`.veg` 是 Vega FEM 的体网格格式：几何 + 命名材料 + 命名元素集合 + 区域（材料→集合的赋值）。三个 dataclass 一一对应其段落：

## class `MeshSet`（dataclass）

```python
MeshSet(name, elements)
```

| 字段 | 含义 |
|---|---|
| `name` | 集合名（`.veg` 的 `*SET` 段） |
| `elements` | 元素索引列表 |

`__post_init__` 自动**排序去重**并转 int（28-29 行）——构造后 `elements` 总是有序无重复。索引为 0 基（`.veg` 磁盘格式是 1 基，I/O 层转换）。

## class `MeshRegion`（dataclass）

```python
MeshRegion(material_index, set_index)
```

把 `materials[material_index]` 赋给 `sets[set_index]` 的全部元素（`.veg` 的 `*REGION` 段）。

## class `VegFile`（dataclass）

```python
VegFile(mesh_data, materials, sets, regions)
```

| 字段 | 类型 | 含义 |
|---|---|---|
| `mesh_data` | `TetMeshData \| CubicMeshData` | 几何（[../data.md](../data.md)） |
| `materials` | `list[MaterialLike]` | 材料表（[material.md](material.md)） |
| `sets` | `list[MeshSet]` | 元素集合表 |
| `regions` | `list[MeshRegion]` | 区域赋值表 |

纯数据：构造**不校验**分区完整性（校验推迟到 `VolumeMesh`），因此可作中间编辑态。

### classmethod `from_single_material(mesh_data, material)`

单材料便捷构造：建一个覆盖全部元素的 `"allElements"` 集合 + 一个 `MeshRegion(0, 0)`。非体网格容器抛 `TypeError`。

### `first_material() -> MaterialLike`

断言恰有一个材料并返回之（多材料抛 `ValueError`）——单材料工作流的安全取数。

### `to_volume_regions() -> list[(name, material, elements)]`

把 `(materials, sets, regions)` 三表连接展开成 `VolumeMesh.__init__` 期望的扁平 region 三元组列表。

---

## class `VolumeMesh`

```python
VolumeMesh(mesh_data, regions)
```

Vega 体网格句柄。`regions` 是 `(name, material, elements)` 三元组的可迭代；构造时经 `_validate_and_split_regions`（77-110 行）严格校验后调 `_core.create_volume_mesh_multi`（C++ 侧逐 region 构造 `VM::Material`/`VM::Set`/`VM::Region` 再建 `TetMesh`/`CubicMesh`，`volume/core.cpp:362-413`）。

**分区校验规则**（违反抛 `ValueError`/`TypeError`）：

1. region 名互不相同；
2. 元素索引 $\in[0, m)$；
3. 每个元素**至多**属一个 region（重复赋值报双归属错误）；
4. 每个元素**至少**属一个 region（完全覆盖检查，106-109 行）。

合起来即"regions 构成元素集的划分"——保证 FEM 装配时每个元素有唯一材料。

### classmethod `create_from_single_material(mesh_data, material)`

单材料快捷构造（跳过 Python 级校验直接走 C++，覆盖式 `allElements` 集合）。`material` 必须是 `ENuMaterial` 或 `MooneyRivlinMaterial`。

### classmethod `from_veg_file(veg)`

`VolumeMesh(veg.mesh_data, veg.to_volume_regions())` 的别名——数据层 → 句柄层的标准入口。

### `extract_surface_mesh(*, triangulate=True) -> TriMeshData`

提取体网格边界面（C++ `GenerateSurfaceMesh::computeMesh`）。六面体网格的边界四边形需 `triangulate=True` 拆成三角形（否则绑定层遇到非三角面抛错，`volume/core.cpp:505-523`）。结果常作 [`SurfaceEmbedding`](../geometry/core.md) 的目标表面。

### `_mass_matrix(*, inflate3dim=True)`（内部）

一致质量矩阵 $M=\int_\Omega \rho\,N_i N_j\,dV$（C++ `GenerateMassMatrix::computeMassMatrix`；`inflate3dim` 把 $n\times n$ 标量模式按坐标展开为 $3n\times3n$）。**前导下划线含义**：优先用 formulation 级 API `pypgo.fem.VolumeDensity` + `formulation.mass_matrix(...)`（[../../fem/mass.md](../../fem/mass.md)），本方法仅供对照调试。

### 属性 `num_vertices` / `num_elements` / `mesh_type`

计数与 [`MeshDataType`](../data.md) 枚举（委托 C++）。

### 属性 `mesh_data` / `geometry`

返回几何容器（`TetMeshData`/`CubicMeshData`）。优先返回构造时缓存的对象；缓存缺失时从 C++ 导出重建（177-180 行）。`geometry` 是 `mesh_data` 的别名。

### 属性 `material` / `material_spec`

导出**第一个**材料为 dataclass（`export_material_payload`；无材料时返回默认 `ENuMaterial`）。多材料网格请改用 `to_veg_file().materials` 取全表。`material_spec` 是别名。

### `to_veg_file() -> VegFile`

从 C++ 句柄完整导出四元组（含全部材料/集合/区域）——`VolumeMesh` → 可编辑数据层的逆向通道，与 `from_veg_file` 构成往返。

---

## I/O 函数

## func `read_msh(path) -> TetMeshData`

读 Gmsh `.msh`（C++ `loadMshFile`），**只取几何**——返回无材料的 `TetMeshData`，需自行配材料再建 `VolumeMesh`。

## func `read_veg(path) -> VegFile`

读 `.veg` 为数据层对象：几何按元素类型自动包装为 `TetMeshData` 或 `CubicMeshData`，材料还原为 dataclass（含 orthotropic 材料的文件会在还原时抛 `RuntimeError`，见 [material.md](material.md)）。

## func `write_veg(path, veg) -> None`

把 `VegFile` 写盘（C++ `writeVegFile`；集合元素 1 基化、材料按类型写 `ENU`/`MOONEYRIVLIN` 行）。`mesh_data` 类型不符抛 `TypeError`。

## 用法示例

```python
import numpy as np
import pypgo
from pypgo.mesh.volume import ENuMaterial, MeshRegion, MeshSet, VegFile, VolumeMesh

tets = pypgo.mesh.tet_mesher(
    pypgo.mesh.create_box(bmin=[0,0,0], bmax=[2,1,1]), backend="tetgen")

# 双材料：左半软、右半硬（按元素质心 x 分组）
cx = tets.vertices[tets.elements].mean(axis=1)[:, 0]
left, right = np.flatnonzero(cx < 1.0), np.flatnonzero(cx >= 1.0)

vol = VolumeMesh(tets, [
    ("soft", ENuMaterial(name="soft", E=1e4, nu=0.45), left.tolist()),
    ("hard", ENuMaterial(name="hard", E=1e7, nu=0.45), right.tolist()),
])

# 持久化与往返
veg = vol.to_veg_file()
pypgo.mesh.volume.write_veg("bimaterial.veg", veg)
veg2 = pypgo.mesh.volume.read_veg("bimaterial.veg")
assert len(veg2.materials) == 2

# 进入 FEM（仅 ENu 材料）
sim_mesh = pypgo.fem.SimulationMesh.create_volumetric(vol)
```

## 交叉链接

- 材料字段与 Lamé 换算：[material.md](material.md)
- 几何容器：[../data.md](../data.md)
- 下游消费：[../../fem/mesh.md](../../fem/mesh.md)（`SimulationMesh.create_volumetric`）、[../geometry/core.md](../geometry/core.md)（`BarycentricEmbedding` 以 `VolumeMesh` 为宿主）
- 体网格来源：[../processing/volume.md](../processing/volume.md)（`tet_mesher`/`cubic_mesher`）
