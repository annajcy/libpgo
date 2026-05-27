# M1 MeshData / MeshGeo 绑定状态与收尾计划

> **状态更新时间：** 2026-05-26  
> **当前方向：** `MeshData<K>` 是唯一中间转换数据结构；Python 公开 `MeshData` 数据容器和 `MeshGeo` façade 两套对象，但 I/O 与 `VolumeMesh` 边界只接受/返回 `MeshData`。

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
conda run -n libpgo cmake --preset python-build
conda run -n libpgo cmake --build --preset python-build -j 8

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

`.veg` material policy is unchanged in this milestone: `MaterialSpec` remains the exposed Python material value object, and full material / region / set modeling remains future work.

---

# Phase 2: Multi-Material VolumeMesh + Surface + Embedding

> **状态：** 设计中，尚未实现

## 目标

在 Phase 1 单材料 `VolumeMesh` 基础上，暴露：
1. 多 material / set / region 的 `VolumeMesh` 构造（链式 builder 模式）
2. Surface mesh extraction
3. Barycentric embedding / interpolation matrix（surface ↔ volume DOF 映射）

## C++ 两层 Architecture 概述

Python API 涉及的 C++ 对象分为两层，它们之间通过 `load*Mesh()` factory 函数桥接：

```
Vega FEM 层 (VolumetricMeshes::)           Simulation 层 (SimulationMesh)
────────────────────────────               ─────────────────────────────
TetMesh   (geometry + Material)       →    loadTetMesh()     → SimulationMesh (TET)
CubicMesh (geometry + Material)       →    loadCubicMesh()   → SimulationMesh (CUBIC)
ShellMesh (geometry + 5 STVK params)  →    loadShellMesh()   → SimulationMesh (SHELL)  ← NEW
```

- **Vega 层** 存 geometry + material，负责 `.veg` I/O 和 Python 侧的构造/导出
- **Simulation 层** 是 solver/FEM/energy 的入口，由 `load*Mesh()` 统一创建
- Python `VolumeMesh` 和 `ShellMesh` 都是 Vega 层的 wrapper，对称设计
- Python material 类型各自独立：`ENuMaterial` 对应 `VolumetricMeshes::Material`，`StVKShellMaterial` 对应 `SimulationMeshENuhMaterial` 的参数集

## C++ 前置重构

### 2.1 Material / Set / Region 从嵌套类移出

当前这三个是 `VolumetricMesh` 的内部嵌套类，nanobind 绑定嵌套类很麻烦。把它们提到 `pgo::VolumetricMeshes` namespace 级别，`VolumetricMesh` 内部加 `using` 别名保持向后兼容。

```cpp
// Before — 嵌套在 VolumetricMesh 内部
class VolumetricMesh {
public:
  class Set { ... };
  class Material { ... };
  class Region { ... };
};

// After — 提到 namespace 级别
namespace pgo::VolumetricMeshes {

class MeshSet {
public:
  MeshSet();
  explicit MeshSet(const std::string &name);
  MeshSet(const std::string &name, const std::set<int> &elements);

  const std::string &getName() const;
  int getNumElements() const;
  const std::set<int> &getElements() const;
  bool isMember(int element) const;
  void insert(int element);
  void clear();

private:
  std::string name_;
  std::set<int> elements_;
};

class MeshMaterial {  // 原 Material，rename 避免和 Python Material 混淆
public:
  MeshMaterial(const std::string &name, double density);
  virtual ~MeshMaterial();
  virtual MeshMaterial *clone() const = 0;
  virtual materialType getType() const = 0;

  const std::string &getName() const;
  double getDensity() const;
  void setName(const std::string &name);
  void setDensity(double density);

private:
  std::string name_;
  double density_;
};

class ENuMaterial : public MeshMaterial { ... };       // 原 ENuMaterial
class MooneyRivlinMaterial : public MeshMaterial { ... };
class OrthotropicMaterial : public MeshMaterial { ... };

class MeshRegion {
public:
  MeshRegion();
  MeshRegion(int materialIndex, int setIndex);

  int getMaterialIndex() const;
  int getSetIndex() const;
  void setMaterialIndex(int index);
  void setSetIndex(int index);

private:
  int setIndex_ = -1;
  int materialIndex_ = -1;
};

}  // namespace pgo::VolumetricMeshes

// VolumetricMesh 内部保持向后兼容
class VolumetricMesh {
public:
  using Set = MeshSet;
  using Material = MeshMaterial;
  using Region = MeshRegion;
  // ... 其余不变
};
```

**改动量：** 纯机械操作：把三个嵌套类的定义和实现移到 namespace 级别，原嵌套类位置放 `using` 别名。`downcastENuMaterial` 等辅助函数签名不变。不改变任何调用方。

### 2.2 新增 `buildVolumeMesh` factory 函数

`TetMesh` / `CubicMesh` 已有接受 `(vertices, elements, numMaterials, materials, numSets, sets, numRegions, regions)` 的构造函数（`tetMesh.h:91-95`，`cubicMesh.h:103-107`）。只需要一个薄的 bridge 函数把 `MeshData` + material/set/region vectors 转成这个调用。

```cpp
// 新文件 volumetricMeshBuilder.h — 纯加法，不修改现有代码

std::unique_ptr<VolumetricMeshes::TetMesh> buildTetMesh(
    const Mesh::MeshData<4> &data,
    const std::vector<std::unique_ptr<MeshMaterial>> &materials,
    const std::vector<MeshSet> &sets,
    const std::vector<MeshRegion> &regions);

std::unique_ptr<VolumetricMeshes::CubicMesh> buildCubicMesh(
    const Mesh::MeshData<8> &data,
    const std::vector<std::unique_ptr<MeshMaterial>> &materials,
    const std::vector<MeshSet> &sets,
    const std::vector<MeshRegion> &regions);
```

实现就是调现有的多材料构造函数。

### 2.3 新增 `buildBarycentricEmbedding` bridge

```cpp
// surface mesh extraction — 已有自由函数，直接 wrap
// GenerateSurfaceMesh::computeMesh(...) → (vertices, faces)

// interpolation matrix — 已有 BarycentricCoordinates，直接 wrap
// InterpolationCoordinates::BarycentricCoordinates(locations, volumetricMesh)
//   → generateInterpolationMatrix() → SpMatD
```

## Python API 设计

### 3.1 Material 类型

Python 侧建立 `MeshMaterial` 基类，`ENuMaterial` 等为子类。这是**纯数据类型系统**，不绑定 C++ `Material` 虚函数。Python 对象仅作为数据载体，`.build()` 时把属性序列化成 `_core.ENuMaterialCore(...)` 传给 C++。

```python
# pypgo/mesh.py

class MeshMaterial:
    """Base class for volumetric mesh materials.

    All material subtypes are plain data carriers.  No virtual dispatch to C++
    happens at the Python level — attribute values are passed by value to the
    C++ constructor at ``.build()`` time.
    """
    def __init__(self, name: str, density: float):
        self._name = name
        self._density = density

    @property
    def name(self) -> str: return self._name

    @property
    def density(self) -> float: return self._density

    @property
    def material_type(self) -> str:
        """'ENU', 'MOONEYRIVLIN', or 'ORTHOTROPIC'."""
        raise NotImplementedError


class ENuMaterial(MeshMaterial):
    """Linear isotropic material (Young's modulus + Poisson ratio)."""
    def __init__(self, name: str, *, density: float, E: float, nu: float):
        super().__init__(name, density)
        self._E = E
        self._nu = nu

    @property
    def material_type(self) -> str: return "ENU"

    @property
    def E(self) -> float: return self._E

    @property
    def nu(self) -> float: return self._nu

    # derived Lame parameters (convenience, computed inline)
    @property
    def lam(self) -> float: return self.E * self.nu / ((1 + self.nu) * (1 - 2 * self.nu))

    @property
    def mu(self) -> float: return self.E / (2 * (1 + self.nu))

    def __repr__(self):
        return f"ENuMaterial({self.name!r}, E={self.E:.3g}, nu={self.nu}, density={self.density})"


# Future subtypes (same pattern — data carrier only):
# class MooneyRivlinMaterial(MeshMaterial):   # density, mu01, mu10, v1
# class OrthotropicMaterial(MeshMaterial):    # density, E1..E3, nu12..nu31, G12..G31, R
```

**为什么建基类：**

1. `.region(name, material, elements)` 类型检查只需 `isinstance(material, MeshMaterial)`
2. VEG parser 根据 `*MATERIAL` section 头的类型字符串（`ENU` / `MOONEYRIVLIN` / `ORTHOTROPIC`）分派到对应 Python class
3. 和 `MeshData` / `MeshGeo` 的设计一致，都是 Python wrapper 层有层次结构

**`.build()` 时数据流：**

```
ENuMaterial(name="rubber", density=1000, E=1e9, nu=0.45)  # Python 对象，存几个 float
         ↓
.region("body", material, [0..3])    # isinstance(material, MeshMaterial) → True
         ↓
.build()                              # 内联取值 material.E, material.nu → 传参数
         ↓
_core.build_volume_mesh(             # nanobind 调用 C++ 构造
    data,
    materials=[ENuMaterialCore(name, density, E, nu)],
    sets=[...], regions=[...])
```

**性能：** `material_type` 和 `lam`/`mu` 等派生属性只在冷路径（parser 分派、debug 输出）被调用。FEM assembly/solver 全程在 C++ 侧，不接触 Python material 对象。

### 3.2 MeshSet / MeshRegion

```python
from pypgo.mesh import MeshSet, MeshRegion

body_set   = MeshSet("body",   [0, 1, 2, 3])
insert_set = MeshSet("insert", [4, 5])

# MeshSet("body", [3,2,1,0]) ≡ MeshSet("body", [0,1,2,3])  # std::set 去序

region = MeshRegion(material_index=0, set_index=0)
```

### 3.3 VolumeMesh 构造：Builder 为唯一入口

**设计决策：** `VolumeMeshBuilder` 是 `VolumeMesh` 构造的**唯一内部路径**。`VolumeMesh(tet_data, material)` 和 `VolumeMesh.load(path)` 都是语法糖，底层等价于 builder + 自动 region。

```python
from pypgo.mesh import VolumeMesh, ENuMaterial

# === 单材料（语法糖，底层走 builder）===
vol = VolumeMesh(tet_data, ENuMaterial("rubber", density=1000, E=1e9, nu=0.45))
# 等价于：
# vol = (VolumeMesh.builder(tet_data)
#     .region("all", ENuMaterial("rubber", ...), range(tet_data.num_elements))
#     .build())

# === 多材料（显式 builder）===
vol = (VolumeMesh.builder(tet_data)
    .region("body",   ENuMaterial("rubber", density=1000, E=1e9, nu=0.45),
            [0, 1, 2, 3])
    .region("insert", ENuMaterial("steel",  density=7800, E=2e11, nu=0.3),
            [4, 5])
    .build())
```

**Builder 语义：**

`VolumeMeshBuilder` 是 `VolumeMesh` 构造的**唯一入口**。初始化 builder 时需要 `mesh_data`（`TetMeshData` 或 `CubicMeshData`）。

每调用一次 `.region(name, material, elements)` 相当于：
1. `MeshSet(name, elements)` — 创建命名的元素集合
2. 将 material 追加到 materials 列表
3. 将 set 追加到 sets 列表
4. `MeshRegion(material_index, set_index)` — 绑定两者（index 由追加顺序自然确定）

`.build()` 时做校验后调 C++ 构造：

```
校验规则（partition check）：
1. 所有 region 的元素集合互不相交
   → 同一个 element 不能出现在两个 region 里
2. 所有 region 的元素集合的并集 == 全部 element（0..num_elements-1）
   → 不能有 element 漏掉
3. build() 校验失败 → ValueError，精确报告冲突/遗漏的 element IDs

单材料语法糖 VolumeMesh(tet_data, material)：
  内部等价于 builder(tet_data).region("all", material, all_elements).build()
  即自动创建一个覆盖全部 element 的默认 region。

冲突示例：
  .region("a", mat1, [0, 1, 2])
  .region("b", mat2, [2, 3, 4])
  # build() → ValueError: "element 2 assigned to both 'a' and 'b'"

遗漏示例：
  .region("a", mat1, [0, 1])
  .region("b", mat2, [3, 4])
  # build() → ValueError: "element 2 not assigned to any region"
```

**Python 层实现：** Builder 是纯 Python class，`.region()` 累积 `(name, material, elements)` 列表，`.build()` 做 partition 校验后调 `_core.build_volume_mesh(data, materials, sets, regions)`。C++ 侧只管构造，不管校验。

### 3.4 Surface Mesh Extraction

```python
# VolumeMesh → TriMeshData (extract boundary surface)
surface_data = vol.extract_surface_mesh(triangulate=True)
# surface_data 是 TriMeshData，可以直接 write_obj_geo 或用 TriMeshGeo 查询

# 底层调 GenerateSurfaceMesh::computeMesh() + TriMeshGeo → toMeshData()
```

### 3.5 Barycentric Embedding / Interpolation Matrix

```python
from pypgo.mesh import BarycentricEmbedding

# surface positions → volume embedding
embedding = BarycentricEmbedding(
    target_locations=surface_data.vertices,  # (m, 3) float64
    volume_mesh=vol,
)

# 显式 sparse interpolation matrix
S = embedding.interpolation_matrix
# S.shape == (3 * m_surface, 3 * n_volume)
# S 是 pypgo.sparse.SparseMatrix（M2 会提供），当前阶段先用 COO tuple
rows, cols, values = S.to_coo()

# 直接 deform（不用显式 matrix）
surface_disp = embedding.deform(volume_disp)  # volume_disp: (3*n,) → surface_disp: (3*m,)
```

**C++ 映射：** `BarycentricEmbedding` 是 Python class，内部构造 `InterpolationCoordinates::BarycentricCoordinates`，`.interpolation_matrix` → `generateInterpolationMatrix()`，`.deform()` → `deform()`。

### 3.6 Python-Native VEG Parser & Writer

**设计决策：** `.veg` 文件的解析和写入完全用 Python 实现，不再走 C++ `VolumetricMeshParser` / `saveToAscii` 路径。

**理由：**
- `.veg` 是简单 ASCII 格式（section header + CSV 数据），Python `split()` + `np.array()` 即可处理
- 性能不是瓶颈：100 万元素文件 ~50 MB，Python 解析 <0.5s，瓶颈在磁盘 I/O
- 当前 C++ 路径构造完整 `VolumetricMesh` 后再丢弃 materials/sets/regions，属于浪费
- Python parser 直接拿到 `MeshData` + materials + sets + regions，正好衔接 builder

**Parser（只支持 ASCII）：**

```python
# pypgo/io.py 中实现

@dataclass
class VegFile:
    """Parsed .veg file."""
    mesh_data: TetMeshData | CubicMeshData
    materials: list[ENuMaterial]
    sets: list[MeshSet]
    regions: list[MeshRegion]

def parse_veg(path: str) -> VegFile:
    """解析 .veg 文件，返回完整的 mesh data + material/set/region 信息。
    
    Format:
      *VERTICES
      <n> 3 0 0
      <id> <x> <y> <z>
      *ELEMENTS
      TET|CUBIC
      <n> <K> 0
      <id> <v1> ... <vK>
      *MATERIAL <name>
      ENU, <density>, <E>, <nu>
      *SET <name>     (optional)
      <id1>,<id2>,...
      *REGION
      <setName>, <materialName>
      *INCLUDE <path>  (not supported initially)
    """
    sections = _split_sections(open(path).read())
    elem_type = _parse_elements_header(sections["ELEMENTS"])
    
    vertices = np.array(sections["VERTICES"], dtype=np.float64)     # (n, 3)
    elements = np.array(sections["ELEMENTS"], dtype=np.int64) - 1   # 1-based → 0-based
    
    if elem_type == "TET":
        data = TetMeshData(vertices, elements)
    else:
        data = CubicMeshData(vertices, elements)
    
    materials = [_parse_material(sec) for sec in sections.get("MATERIAL", [])]
    sets = _parse_sets(sections.get("SET", []))  # element IDs → 0-based
    regions = _parse_regions(sections["REGION"], materials, sets)
    
    return VegFile(data, materials, sets, regions)
```

**Writer：**

```python
def write_veg(path: str, veg: VegFile) -> None:
    """写入 .veg 文件。0-based → 1-based 索引转换，.15g 精度保证 roundtrip。"""
    lines = [
        f"# Vega mesh file.",
        f"# {veg.mesh_data.num_vertices} vertices, {veg.mesh_data.num_elements} elements",
        "",
        "*VERTICES",
        f"{veg.mesh_data.num_vertices} 3 0 0",
    ]
    for i, v in enumerate(veg.mesh_data.vertices, 1):
        lines.append(f"{i} {v[0]:.15g} {v[1]:.15g} {v[2]:.15g}")
    
    k = {"TET": 4, "CUBIC": 8}[veg.mesh_data.mesh_type.name.upper()]
    elem_type = "TET" if k == 4 else "CUBIC"
    lines += ["", "*ELEMENTS", elem_type, f"{veg.mesh_data.num_elements} {k} 0"]
    for i, e in enumerate(veg.mesh_data.elements, 1):
        lines.append(f"{i} " + " ".join(str(v + 1) for v in e))
    
    for mat in veg.materials:
        lines += ["", f"*MATERIAL {mat.name}", f"ENU, {mat.density}, {mat.E}, {mat.nu}"]
    
    for s in veg.sets:
        ids = ",".join(str(e + 1) for e in sorted(s.elements))
        lines += ["", f"*SET {s.name}", ids]
    
    for r in veg.regions:
        set_name = veg.sets[r.set_index].name
        mat_name = veg.materials[r.material_index].name
        lines += ["", "*REGION", f"{set_name}, {mat_name}"]
    
    lines.append("")
    open(path, "w").write("\n".join(lines))
```

**风险：**
- 二进制 `.veg` 极少用，先不支持，遇到时按需添加
- `*INCLUDE` 指令暂不支持，发现时报 `NotImplementedError`

### 3.7 更新后的 I/O API

```python
from pypgo import io

# VEG：返回完整 VegFile（替代当前 read_veg_geo 只返回 (data, material)）
veg = io.read_veg("box.veg")
# veg.mesh_data   → TetMeshData | CubicMeshData
# veg.materials   → list[ENuMaterial]
# veg.sets        → list[MeshSet]
# veg.regions     → list[MeshRegion]

# 直接构造多材料 VolumeMesh
vol = (VolumeMesh.builder(veg.mesh_data)
    .region("body",   veg.materials[0], veg.sets[0].elements)
    .region("insert", veg.materials[1], veg.sets[1].elements)
    .build())

# OBJ：保持不变
tri_data = io.read_obj("box.obj")
io.write_obj("box.obj", tri_data)
```

### 3.8 Mesher API

模块归属 `pypgo.tools.mesh`：cubic/tet mesher 是"从表面构建体积网格"的工具，语义上是构造管线的一部分。

#### cubicMesher

简洁函数，只暴露 resolution。material 留给 builder。

```python
from pypgo.tools.mesh import cubic_mesher

cubic_data = cubic_mesher(tri_data, resolution=20)
# tri_data: TriMeshData
# resolution: 沿 AABB 最短边的体素数量
# → CubicMeshData

# 然后走 builder
vol = VolumeMesh(cubic_data, ENuMaterial("rubber", density=1000, E=1e6, nu=0.45))
```

**C++ 映射：** `cubicMesherCore` 已编译为静态库，绑定其入口函数，内部走 bounding volume hierarchy + inside/outside 查询。

**参数说明：**
- `resolution: int` — 沿 AABB 最短边的体素数量。分辨率越高，网格越密。
- 当前只暴露这一个参数。如需 padding、min/max voxel count 等高级参数，后续按需追加。

#### tetMesher

dict config 灵活传递 backend 参数。

```python
from pypgo.tools.mesh import tet_mesher

# tetgen backend
tet_data = tet_mesher(tri_data, backend="tetgen",
    config={"command": "pq1.2aY"})

# tetwild backend（需要编译时开启 PGO_TET_MESHER_USE_TET_WILD）
tet_data = tet_mesher(tri_data, backend="tetwild",
    config={"lr": 0.1, "la": 0.01, "epsr": 1e-6, "stop_energy": 10, "max_threads": 8})
# → TetMeshData
```

**backend 可用性检查：**

`backend="tetgen"` 总是可用（TetGen 是命令行调用，编译时不依赖 TetGen 库）。`backend="tetwild"` 可用性由 C++ 编译宏 `PGO_TET_MESHER_USE_TET_WILD` 控制。

```python
from pypgo.tools.mesh import has_tetwild

if has_tetwild():
    tet_data = tet_mesher(tri_data, backend="tetwild", config={...})
else:
    tet_data = tet_mesher(tri_data, backend="tetgen", config={"command": "pq1.2aY"})
```

调用 `tet_mesher(backend="tetwild")` 且 tetwild 不可用时，抛出 `RuntimeError("tetwild backend is not available. Rebuild with PGO_TET_MESHER_USE_TET_WILD=ON.")`。

**C++ 映射：** tetMesher 已有 CLI tool + `tetMesherBackend` 体系。绑定 `tetMesherBackend` 的 compute 函数：传入 `TriMeshGeo` + backend 类型 + config map → 返回 `TetMeshGeo` → `toMeshData()`。

#### createTriMesh 形状工厂

```python
from pypgo.mesh_geo import create_box, create_sphere, create_cylinder, create_torus

tri_data = create_box(bmin=(0, 0, 0), bmax=(1, 1, 1))
tri_data = create_sphere(radius=1.0, axis_subdiv=32, height_subdiv=16)
tri_data = create_cylinder(radius=1.0, height=2.0, axis_subdiv=32, height_subdiv=1)
tri_data = create_torus(radial_res=32, tubular_res=16, radius=1.0, thickness=0.3)
```

全部返回 `TriMeshData`，直接喂给 mesher 或 I/O。C++ 映射 `pgo::Mesh::createBoxMesh(...)` 等 `createTriMesh.h` 中的函数 → `toMeshData()`。

#### 模块总览

```python
pypgo.tools.mesh:
    cubic_mesher(tri_data, resolution) → CubicMeshData
    tet_mesher(tri_data, backend, config) → TetMeshData
    has_tetwild() → bool

pypgo.mesh_geo (新增):
    create_box(bmin, bmax) → TriMeshData
    create_sphere(radius, axis_subdiv, height_subdiv) → TriMeshData
    create_cylinder(radius, height, axis_subdiv, height_subdiv) → TriMeshData
    create_torus(radial_res, tubular_res, radius, thickness) → TriMeshData
```

## 绑定清单

### Private `_core` 新增

```
# Material types
ENuMaterialCore(name, density, E, nu)
MooneyRivlinMaterialCore(name, density, mu01, mu10, v1)
OrthotropicMaterialCore(name, density, E1, E2, E3, nu12, nu23, nu31, G12, G23, G31, R)

# Set / Region
MeshSetCore(name, elements)
MeshRegionCore(material_index, set_index)

# Builder
build_volume_mesh(data, materials, sets, regions) → VolumeMeshCore

# Surface extraction
extract_surface_mesh(volume_mesh_core, triangulate) → TriMeshDataCore

# Barycentric embedding
BarycentricEmbeddingCore(target_locations, volume_mesh_core)
  .interpolation_matrix() → SpMatD (or COO triplet)
  .deform(volume_disp) → surface_disp
```

### Private `_core` 移除

```
read_veg_geo / write_veg_geo  — 改为 Python 原生 parser/writer
```

### Private `_core` 新增（Mesher）

```
cubicMesherCore(triMeshGeo) → TetMeshGeo → toMeshData() → TetMeshDataCore 等价物
tetMesherBackendCore(triMeshGeo, backend, config) → TetMeshGeo → toMeshData()
has_tetwild() → bool
```

### Public Python 模块

```python
pypgo.mesh:      ENuMaterial, MeshSet, MeshRegion, VolumeMesh (扩展 builder),
                 VolumeMeshBuilder, BarycentricEmbedding
pypgo.mesh_geo:  TriMeshData, TetMeshData, CubicMeshData,
                 TriMeshGeo, TetMeshGeo, CubicMeshGeo, MeshDataType,
                 create_box, create_sphere, create_cylinder, create_torus
pypgo.io:        read_veg / write_veg / VegFile  (Python 原生),
                 read_obj / write_obj  (保持 _core)
pypgo.tools.mesh: cubic_mesher, tet_mesher, has_tetwild
pypgo._core:     上述 Core 类型 + factory 函数
```

## C++ 改动影响范围

| 改动 | 类型 | 影响 |
|------|------|------|
| Material/Set/Region 移出嵌套类 | 机械重构 | `VolumetricMesh` 内部加 using，调用方无感知 |
| `volumetricMeshBuilder.h` | 新文件 | 零影响，纯加法 |
| `mesh_bindings.cpp` 追加绑定 | 扩展现有文件 | 追加 nanobind 定义，不影响现有绑定 |
| `mesh_geo_bindings.cpp` 追加 `create_*` + mesher bridge | 扩展现有文件 | 追加绑定 |
| `read_veg_geo` / `write_veg_geo` 移除 | 删除 C++ binding | Python 原生 parser/writer 替代 |
| `cubicMesherCore` / `tetMesherBackend` 绑定 | 扩展现有文件 | 绑定已有静态库入口，零 C++ 改动 |

## 测试清单

### C++ 测试

- `MeshSet` construction, elements query, isMember
- `ENuMaterial` clone, getType, lambda/mu computation
- `buildTetMesh` / `buildCubicMesh` with multiple materials/sets/regions
- `GenerateSurfaceMesh::computeMesh` 输出顶点一致性和面数量
- `BarycentricCoordinates::generateInterpolationMatrix` 维度正确
- `cubicMesherCore` 体素化输出维度正确
- `tetMesherBackend` tetgen/tetwild 输出维度正确

### Python 测试

- Python VEG parser: tet/cubic mesh 正确解析，1-based → 0-based 索引转换正确
- Python VEG writer: roundtrip `VegFile → write → parse` 数据一致
- `VolumeMesh.builder().region(...).build()` 正确构造多材料 mesh
- Builder partition 校验：冲突元素报错、遗漏元素报错
- `vol.extract_surface_mesh()` 返回有效 `TriMeshData`
- `BarycentricEmbedding` 维度正确 + `deform()` 输出 shape 正确
- 单材料 `VolumeMesh(tet_data, mat)` 行为不变（回归）
- 旧 `read_veg_geo` / `write_veg_geo` 从 `_core` 移除，无 import 残留
- `cubic_mesher(tri_data, resolution)` 输出 `CubicMeshData` 有效
- `tet_mesher(tri_data, backend, config)` tetgen 输出 `TetMeshData` 有效
- `tet_mesher(backend="tetwild")` 在 tetwild 不可用时抛 `RuntimeError`
- `has_tetwild()` 返回值与编译配置一致
- `create_box/sphere/cylinder/torus` 输出 `TriMeshData` 有效

## 执行顺序

```
1.  C++ 提取 Material/Set/Region 到 namespace 级别
2.  C++ 新增 volumetricMeshBuilder.h（含 surface extraction bridge）
3.  _core 绑定：ENuMaterialCore, MeshSetCore, MeshRegionCore
4.  _core 绑定：build_volume_mesh, extract_surface_mesh, BarycentricEmbeddingCore
5.  _core 绑定：cubicMesherCore, tetMesherBackend, has_tetwild
6.  _core 移除：read_veg_geo / write_veg_geo C++ bindings
7.  Python 层：ENuMaterial, MeshSet, MeshRegion, VolumeMeshBuilder
8.  Python 层：VegFile, parse_veg, write_veg（纯 Python，不走 C++）
9.  Python 层：BarycentricEmbedding
10. Python 层：cubic_mesher, tet_mesher, has_tetwild (pypgo.tools.mesh)
11. Python 层：create_box/sphere/cylinder/torus (pypgo.mesh_geo)
12. Python 层：read_veg / write_veg API 对齐（替代 read_veg_geo / write_veg_geo）
13. 测试
```

---

# Phase 3: Mesh Info & Quality Check

> **状态：** 设计中，尚未实现

## 目标

在 Phase 2 完成 `MeshData` / `MeshGeo` 属性绑定 + mesher 工具后，用 Python + NumPy 实现体积信息查询和表面网格质量检查。除自相交检测需保留 C++ binding 外，全部 Python 原生。

## 4.1 Volume Info（MeshData 属性）

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

## 4.2 Surface Mesh Quality Check

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

## 4.3 绑定清单（Phase 3 新增）

### Private `_core` 新增

```
check_self_intersections(triMeshGeo) → bool   # CGAL exact-count，python-build 必开启
```

### Public Python 新增

```python
# pypgo.mesh_geo (MeshData 新增)
_MeshDataBase.bbox               → (bmin, bmax) tuple of (3,) arrays
_MeshDataBase.take_elements(ids) → SameType
TriMeshData.concatenate(meshes)  → TriMeshData
TetMeshData.concatenate(meshes)  → TetMeshData
CubicMeshData.concatenate(meshes) → CubicMeshData
TetMeshData.volume               → float
TetMeshData.center_of_mass       → (3,) np.ndarray
CubicMeshData.volume             → float
CubicMeshData.center_of_mass     → (3,) np.ndarray

# pypgo.mesh_geo (MeshGeo 新增)
TriMeshGeo.face_areas            → (m,) np.ndarray
TriMeshGeo.face_normals          → (m, 3) np.ndarray
TriMeshGeo.vertex_normals        → (n, 3) np.ndarray

# pypgo.tools.mesh 新增
check_surface_quality(tri_data, short_edge_threshold) → QualityReport
```

### 模块总览（Phase 2 + Phase 3）

```python
pypgo.mesh:       ENuMaterial, MeshSet, MeshRegion, VolumeMesh,
                  VolumeMeshBuilder, BarycentricEmbedding
pypgo.mesh_geo:   TriMeshData(.bbox, .take_elements, .concatenate),
                  TetMeshData(.volume, .center_of_mass, .bbox, .take_elements, .concatenate),
                  CubicMeshData(.volume, .center_of_mass, .bbox, .take_elements, .concatenate),
                  TriMeshGeo(.face_areas, .face_normals, .vertex_normals),
                  TetMeshGeo, CubicMeshGeo, MeshDataType,
                  create_box, create_sphere, create_cylinder, create_torus
pypgo.io:         read_veg, write_veg, VegFile, read_obj, write_obj
pypgo.tools.mesh: cubic_mesher, tet_mesher, has_tetwild,
                  check_surface_quality
pypgo._core:      上述 Core 类型 + factory 函数
```

## 4.4 C++ 改动影响范围（Phase 3 新增）

| 改动 | 类型 | 影响 |
|------|------|------|
| `check_self_intersections` binding | 新 binding | CGAL exact-count，python-build 必开启 |

## 4.5 测试清单（Phase 3 新增）

- `tri_data.bbox` 输出形状 (2,3)，bmin < bmax
- `tri_data.take_elements([0, 2])` 返回同类型，元素数正确
- `TriMeshData.concatenate([a, b])` 顶点拼接 + 索引偏移正确
- `tet_data.volume` 输出与 C++ `volumetricMeshInfo` 一致
- `cubic_data.volume` 输出正确
- `tet_data.center_of_mass` 形状 (3,)，值合理
- `tri_geo.face_areas` 形状 (m,)，面积 > 0
- `tri_geo.face_normals` 形状 (m, 3)，单位长度
- `tri_geo.vertex_normals` 形状 (n, 3)，单位长度
- `check_surface_quality(tri_data)` 对干净 mesh 返回 `is_clean=True`
- `check_surface_quality` 对退化 mesh 检测出 `degenerate_tris`
- `check_surface_quality` 检测出自相交 mesh 的 `has_self_intersections=True`

## 4.6 执行顺序（Phase 3）

```
14. Python 层：bbox, take_elements, concatenate（纯 NumPy）
15. Python 层：TetMeshData.volume / .center_of_mass（纯 NumPy）
16. Python 层：CubicMeshData.volume / .center_of_mass（纯 NumPy）
17. Python 层：TriMeshGeo.face_areas / .face_normals / .vertex_normals（纯 NumPy）
18. Python 层：check_surface_quality + QualityReport（纯 Python + NumPy）
19. _core 绑定：check_self_intersections（CGAL exact-count）
20. 测试
```

---

# Phase 4: Shell Simulation Mesh API

> **状态：** 设计中，尚未实现

## 目标

在 Volume 仿真 API（Phase 2）基础上，暴露壳仿真 API：`ShellMaterial` 材料体系 + `ShellMesh` 构造。

## 5.1 Shell vs Volume 材料差异

C++ 侧两者是完全独立的体系：

| | Volume | Shell |
|---|---|---|
| 基类 | `MeshMaterial` (E, nu, density) | `ShellMaterial` (thickness) |
| 变形度量 | 3D 变形梯度 F (3x3) | 2D 基本形式 a + b (2x2 曲面度量 + 曲率) |
| 能量缩放 | 体积 | 膜 ∝ h，弯曲 ∝ h³/12 |
| 单元拓扑 | 4 节点 tet / 8 节点 cubic | 6 节点三角形壳元 |
| 质量矩阵 | 体积 × density | 面网格面积 × scale（C++ 侧默认 ×100） |
| 弹性参数 | 3 (E, nu, density) | StVK: 5 / Fabric: 12 |

## 5.1 C++ 新增：`VolumetricMeshes::ShellMesh`

在 Vega 层新增 shell mesh 类型，和 `TetMesh` / `CubicMesh` 对称，统一走 `VolumetricMesh → load*Mesh() → SimulationMesh` 路径。

```cpp
// 新文件 volumetricMeshShell.h — 纯加法

namespace pgo::VolumetricMeshes {

class ShellMesh {
public:
  ShellMesh(const Mesh::TriMeshRef &surfaceMesh,
            double E_mem, double nu_mem,
            double E_bend, double nu_bend,
            double thickness);

  const Mesh::TriMeshRef &surfaceMesh() const;

  double E_membrane() const;
  double nu_membrane() const;
  double E_bending() const;
  double nu_bending() const;
  double thickness() const;

private:
  // stores: surface mesh vertices + triangles (owned copy)
  //         E_mem, nu_mem, E_bend, nu_bend, thickness
};

}  // namespace pgo::VolumetricMeshes
```

`loadShellMesh()` 使用 `E_mem, nu_mem, thickness` 构造 `SimulationMeshENuhMaterial`，`E_bend, nu_bend` 传给 Koiter-STVK elastic model。

C++ 三层构造路径统一为：

```
Phase 2: TetMesh   → loadTetMesh()   → SimulationMesh (TET)
Phase 2: CubicMesh → loadCubicMesh() → SimulationMesh (CUBIC)
Phase 4: ShellMesh → loadShellMesh() → SimulationMesh (SHELL)  ← NEW
```

### Material 独立性

Python 的 `ENuMaterial` 和 `StVKShellMaterial` **不共享基类**——它们分别对应不同层的 C++ 类型：

```
Python                 C++ Vega 层                      C++ Simulation 层
ENuMaterial       →    VolumetricMeshes::ENuMaterial  →  SimulationMeshENuMaterial
StVKShellMaterial →    VolumetricMeshes::ShellMesh    →  SimulationMeshENuhMaterial
```

`ENuMaterial` 有 `density`（体积质量），`StVKShellMaterial` 有 `thickness`（壳厚度）。互相不能替换。

## 5.2 ShellMaterial 类型

```python
# pypgo/mesh.py

class ShellMaterial:
    """Base class for Koiter shell materials.

    All shell material subtypes are plain data carriers.  Parameters are
    passed by value to C++ at simulation construction time — no virtual
    dispatch across the language boundary.
    """
    def __init__(self, name: str, thickness: float):
        self._name = name
        self._thickness = thickness

    @property
    def name(self) -> str: return self._name

    @property
    def thickness(self) -> float: return self._thickness

    @property
    def material_type(self) -> str:
        """'STVK' or 'FABRIC'."""
        raise NotImplementedError


class StVKShellMaterial(ShellMaterial):
    """Koiter-StVK shell material (5 elastic parameters).

    Membrane and bending stiffness can differ — set E_bending=None
    and nu_bending=None to use the same values as membrane.
    """
    def __init__(self, name: str, *,
                 thickness: float,
                 E_membrane: float, nu_membrane: float,
                 E_bending: float | None = None,
                 nu_bending: float | None = None):
        super().__init__(name, thickness)
        self._E_membrane = E_membrane
        self._nu_membrane = nu_membrane
        self._E_bending = E_bending if E_bending is not None else E_membrane
        self._nu_bending = nu_bending if nu_bending is not None else nu_membrane

    @property
    def material_type(self) -> str: return "STVK"

    @property
    def E_membrane(self) -> float: return self._E_membrane

    @property
    def nu_membrane(self) -> float: return self._nu_membrane

    @property
    def E_bending(self) -> float: return self._E_bending

    @property
    def nu_bending(self) -> float: return self._nu_bending

    def __repr__(self):
        return (f"StVKShellMaterial({self.name!r}, thickness={self.thickness}, "
                f"E_mem={self.E_membrane:.3g}, nu_mem={self.nu_membrane}, "
                f"E_bend={self.E_bending:.3g}, nu_bend={self.nu_bending})")


# Future:
# class FabricShellMaterial(ShellMaterial):   # 12 params + fiber directions
```

**为什么 `ShellMaterial` 不继承 `MeshMaterial`：**

- `MeshMaterial` 有 `density`（体积仿真从 density × 体积算 mass）
- Shell 的 mass 从面网格面积 × scale 算，没有 density 概念
- `ShellMaterial` 有 `thickness`（膜/弯曲刚度缩放），`MeshMaterial` 没有
- 各自独立，类型系统天然阻止 `ShellMesh(ENuMaterial(...))`

## 5.3 ShellMesh

`ShellMesh` 是对 C++ `SimulationMesh` (shell type) 的 Python wrapper，模式和 `VolumeMesh` 一致。

内部 `ShellMeshCore` 调用 `loadShellMesh(surfaceMesh, material)` 将三角形面网格转换为 6 节点壳单元，然后暴露基本属性。

```python
from pypgo.mesh import ShellMesh, StVKShellMaterial

mat = StVKShellMaterial(
    "fabric",
    thickness=0.001,
    E_membrane=1e6, nu_membrane=0.4,
)

shell = ShellMesh(tri_data, material=mat)

# 基本属性
shell.mesh_data    # → TriMeshData（lazy export 回纯三角形数据）
shell.material     # → ShellMaterial
shell.thickness    # → 0.001 (delegate to material.thickness)
```

构造时校验（Python 层）：
- `mesh_data` 必须是 `TriMeshData`
- `material` 必须是 `ShellMaterial`（`ENuMaterial` 会 `TypeError`）

C++ 映射（和 VolumeMesh 对称）：
```
ShellMesh(tri_data, mat)
  → _core.create_shell_mesh(triMeshGeo, E_mem, nu_mem, E_bend, nu_bend, thickness)
  → new VolumetricMeshes::ShellMesh(surfaceMesh, ...)
  → 存储为 ShellMeshCore（owns VolumetricMeshes::ShellMesh）

将来接 SimulationMesh 时：
  → SimulationMesh::loadShellMesh(shellMeshCore.get())
  → SimulationMesh (SHELL)   ← 留给 M3/M4 solver milestone
```

## 5.4 I/O

OBJ 只能存 geometry，thickness + material 需要额外存储。定义 `.shell.json` 格式：

```python
# pypgo/io.py

shell.save("fabric.shell.json")     # → geometry.obj + fabric.shell.json
shell = ShellMesh.load("fabric.shell.json")
```

```json
{
  "version": 1,
  "geometry": "fabric.obj",
  "thickness": 0.001,
  "material": {
    "type": "STVK",
    "name": "fabric",
    "E_membrane": 1000000.0,
    "nu_membrane": 0.4,
    "E_bending": 1000000.0,
    "nu_bending": 0.4
  }
}
```

## 5.5 绑定清单

### Private `_core` 新增

```
# ShellMesh — wraps VolumetricMeshes::ShellMesh（和 VolumeMeshCore 对称）
ShellMeshCore
  .num_vertices()
  .num_elements()
  .surface_mesh() → TriMeshDataCore
  .E_membrane() → double
  .nu_membrane() → double
  .E_bending() → double
  .nu_bending() → double
  .thickness() → double
create_shell_mesh(triMeshGeo, E_mem, nu_mem, E_bend, nu_bend, thickness)
  → ShellMeshCore (owns VolumetricMeshes::ShellMesh)
```

### Public Python 新增

```python
pypgo.mesh:  ShellMaterial, StVKShellMaterial, ShellMesh
pypgo.io:    ShellMesh.save / ShellMesh.load (.shell.json 格式)
```

## 5.6 模块总览（Phase 2 + 3 + 4）

```python
pypgo.mesh:       ENuMaterial, MeshSet, MeshRegion, VolumeMesh,
                  VolumeMeshBuilder, BarycentricEmbedding,
                  ShellMaterial, StVKShellMaterial, ShellMesh
pypgo.mesh_geo:   TriMeshData(.bbox, .take_elements, .concatenate),
                  TetMeshData(.volume, .center_of_mass, .bbox, .take_elements, .concatenate),
                  CubicMeshData(.volume, .center_of_mass, .bbox, .take_elements, .concatenate),
                  TriMeshGeo(.face_areas, .face_normals, .vertex_normals),
                  TetMeshGeo, CubicMeshGeo, MeshDataType,
                  create_box, create_sphere, create_cylinder, create_torus
pypgo.io:         read_veg, write_veg, VegFile,
                  read_obj, write_obj,
                  ShellMesh.save / ShellMesh.load
pypgo.tools.mesh: cubic_mesher, tet_mesher, has_tetwild,
                  check_surface_quality, QualityReport
pypgo._core:      ShellMaterialCore, ShellMeshCore, create_shell_mesh
```

### C++ 改动影响范围（Phase 4 新增）

| 改动 | 类型 | 影响 |
|------|------|------|
| `volumetricMeshShell.h` | 新文件 | 零影响，纯加法 |
| `mesh_bindings.cpp` 追加 `ShellMeshCore` + `create_shell_mesh` | 扩展现有文件 | 追加 nanobind 定义，不影响现有绑定 |

## 5.7 测试清单

- `StVKShellMaterial` 默认 `E_bending=E_membrane`, `nu_bending=nu_membrane`
- `StVKShellMaterial` 显式设置 bending 参数也能工作
- `ShellMesh(tri_data, material=StVKShellMaterial(...))` 构造成功
- `ShellMesh(tri_data, material=ENuMaterial(...))` 抛 `TypeError`
- `ShellMesh(tri_data, material=StVKShellMaterial(...))` 且 `tri_data` 非 `TriMeshData` 抛 `TypeError`
- `ShellMesh.save` / `ShellMesh.load` roundtrip 数据一致

## 5.8 执行顺序（Phase 4）

```
21. _core 绑定：ShellMaterialCore, ShellMeshCore, create_shell_mesh
22. Python 层：ShellMaterial, StVKShellMaterial
23. Python 层：ShellMesh（wrapper over ShellMeshCore，类型校验）
24. Python 层：ShellMesh.save / .load (.shell.json 格式)
25. 测试
```
