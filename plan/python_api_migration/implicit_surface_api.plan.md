# Implicit Surface Library Refactor + Python Binding Plan

> **状态日期：** 2026-06-01  
> **适用范围：** `src/core/implicitSurface/` C++ 架构重构 + `src/python/pypgo/bindings/implicit_bindings.cpp` + `pypgo/implicit.py` Python binding。  
> **执行约束：** C++ 重构和 Python binding 同步进行；重构不改变现有数值行为；OpenVDB 路径保留原有接口并扩展，不删除。

---

## 目标

把当前过程式 `implicitSurface` 库改造成以 `ImplicitField` 为中心的**惰性场代数**：

- 所有标量场（球、盒子、网格距离场、采样网格）都实现统一的 `ImplicitField` 接口（`eval` + `bounds`）
- CSG 布尔运算（union / intersection / difference）和 shell thickening 以惰性 composed field 表达，不立即分配中间 DenseGrid
- `GridField`（`DenseGrid` 的继承者）通过 buffer protocol 暴露为零拷贝 NumPy view
- Python 用户通过 `|`、`&`、`-` 运算符构建 CSG pipeline，通过 `.sample_to_grid()` 按需物化，再送 marching cubes 或 OpenVDB 提取

---

## 非目标

- 不改变 libigl marching cubes / distance field 数值行为
- 不重写 OpenVDB wrapper（只扩展使其接受 `ImplicitField`）
- 不实现 Python-subclassable `ImplicitField`（nanobind trampoline 推迟到 future_work；Python 侧保留继承层次，`SphereField`/`MeshUnsignedDistanceField`/`BoxField`/`GridField` 都继承 Python `ImplicitField` 基类，但无法在 Python 里自定义 `eval`）
- 不引入 SDF 符号化；本期只实现 `MeshUnsignedDistanceField`（无符号距离场）；signed 版本推迟到 future_work
- `BoxField` 本期只实现 AABB（轴对齐）；OBB 版本推迟到 future_work
- 不删除 `DenseGrid`（保留为 deprecated alias 一个版本）
- 不绑定 `gridSpec.h` 的内部 `linearIndex` 函数

---

## 当前状态（简述）

```
implicitSurface/
├── field/gridSpec.h           GridSpec: bmin/bmax/resolution 值类型
│        denseGrid.h/cpp       DenseGrid: resolution³ double 数组
├── geometry/meshDistance.h    computeMeshUnsignedDistance() → DenseGrid（批量 libigl）
│           sphereField.h      SphereField struct + thickenSphereShell / evaluateBallSDF
│           shellThickening.h  thickenMeshShell(DenseGrid&, thickness, DenseGrid&)
├── operations/booleanOps.h    applyBoolean(a, b, op, out) — 立即分配 DenseGrid
└── extraction/marchingCubesExtractor.h   extractMarchingCubes(DenseGrid, opts, mesh&)
                openVDBExtractor.h         buildOpenVDB*(SphereField|TriMeshGeo) → unique_ptr<LevelSet>
```

**主要问题：**
1. 没有统一的"隐式场"接口；各子系统相互不可组合
2. 布尔运算立即物化，组合 N 个场需要 N 次 `resolution³` 内存分配
3. `DenseGrid` 没有 point query（eval）能力；无法直接用于 OpenVDB
4. Python binding 必须绕过 output-parameter 风格手动翻转

---

## 设计决策

### D1. `ImplicitField` 用虚函数接口，`shared_ptr` 持有

选 virtual dispatch 而非 variant / CRTP，理由：nanobind 对 `std::shared_ptr<Base>` + 继承层次有原生支持，Python 侧不需要任何 workaround。`variant` 是封闭类型集合，每加一种新场都要修改 variant；`CRTP` 无运行时多态，无法绑 Python。

```cpp
class ImplicitField {
public:
    virtual double eval(const V3d& p) const = 0;
    virtual Mesh::LightBoundingBox bounds() const;   // 默认返回无界
    virtual void evalBatch(const V3d* pts, double* out, size_t n) const;
    GridField sampleToGrid(const GridSpec& spec, int numThreads = 0) const;
    virtual ~ImplicitField() = default;
};
```

所有权：`std::shared_ptr<ImplicitField>`，允许 BooleanField / OffsetField 组合持有子场而不产生 lifetime 问题。

### D2. 使用 `Mesh::LightBoundingBox` 作为 bounds 返回类型，不新建 AABB

`implicitSurface` 已依赖 `mesh` 库（`TriMeshGeo`）。`LightBoundingBox`（`src/core/mesh/boundingBox.h`）有完整的 expand / intersect / getIntersection API。`isUnbounded(bb)` 用 `!bb.verifyBox()` 表达（`bmin > bmax` → 无界），不需要引入哨兵常量。

### D3. GridField 替代 DenseGrid，加 eval + buffer protocol

`GridField` 继承 `ImplicitField`，`eval` 做三线性插值。同时通过 nanobind buffer_protocol 把 `data()` 指针作为 shape `(res, res, res)` 的 numpy float64 view 暴露，零拷贝。`DenseGrid` 保留为 deprecated alias（`using DenseGrid = GridField`），一个版本后删除。

### D4. MeshUnsignedDistanceField 持有 TriMeshGeo + 懒建 TriMeshBVTree 用于 point query

当前 `computeMeshUnsignedDistance` 用 libigl 批量填充 DenseGrid。重构后：
- 单点 `eval` 用 `TriMeshBVTree::closestTriangleQuery`（返回 `sqrt(result.dist2)`）
- 批量 `sampleToGrid` override：**特例化为调用 libigl 批量路径**（效率与现有 `computeMeshUnsignedDistance` 等价）
- BVH 懒构建（`std::once_flag`），第一次 `eval` 时触发

这避免了每次 `sampleToGrid` 都建 BVH + 逐点查询，保留现有性能特性。

### D5. BooleanField 和 OffsetField 是惰性 composed field，不立即分配内存

`BooleanField(shared_ptr<ImplicitField> a, b, BooleanOp op)` 的 `eval` 直接做 `min/max/max(a,-b)`。`OffsetField(shared_ptr<ImplicitField> inner, double offset)` 的 `eval` 做 `inner->eval(p) - offset`。只有 `.sample_to_grid()` 时才一次性物化。`applyBoolean` / `thickenMeshShell` 保留为调用惰性 op 后立即 `.sampleToGrid()` 的 convenience wrapper（deprecated）。

### D6. sampleToGrid 用 OpenMP collapse(3) 并行，Python-derived 类型不支持并行

`ImplicitField::sampleToGrid` 默认实现用 OpenMP 并行调用 `eval`。因为 `eval` 是纯 C++ virtual，这是安全的。如果未来引入 Python trampoline（subclass in Python），Python-defined `eval` 需要持有 GIL，与 OpenMP 并行冲突——届时 trampoline 实现需标记为"no-parallel"。本计划不实现 trampoline，只文档化此约束。`MeshUnsignedDistanceField::sampleToGrid` 覆盖为 libigl 批量调用，不走 OpenMP eval 路径。

### D7. Python 运算符：`|`、`&`、`-`，返回 shared_ptr<ImplicitField>

Python 层（`pypgo/implicit.py`）定义包装类 `ImplicitField`，`__or__ = union`、`__and__ = intersection`、`__sub__ = difference`。全程返回 `ImplicitField` 对象（包装 `BooleanField`），最终 `.sample_to_grid()` 物化。

### D8. OpenVDB 路径保留现有接口，新增 GridField → OpenVDB 路径

`buildOpenVDBShellFromMesh` / `buildOpenVDBSphereShell` / `buildOpenVDBBallLevelSet` 签名不变。新增：
- `buildOpenVDBFromGridField(const GridField&, const OpenVDBOptions&)` — 把 DenseGrid 转成 OpenVDB narrow-band（Python 用户使用 marching cubes 或 OpenVDB 的分支口）
- `extractOpenVDB` 接受 `OpenVDBLevelSet`，不改变

Python 侧统一用 `has_openvdb()` guard，`extract_openvdb` 接受 opaque `OpenVDBLevelSet` handle。

### D9. Python 模块为 pypgo.implicit，不放在 pypgo.tools

`pypgo.tools.mesh` 是工具函数集合。隐式场代数是一个独立的计算范式，配 `pypgo.implicit` 顶层模块，与 `pypgo.mesh`、`pypgo.fem` 平级。

### D10. BoxField 新增，作为有界剪裁的标准原语

解析盒子 SDF：`eval(p) = max(|p - center| - half_extent)` — 外面正、里面负。与球、网格距离场组合时可用于空间剪裁（`field - BoxField` = 只保留 box 内部的场）。

---

## C++ 重构

### 5.1 新增 `core/ImplicitField.h`

```cpp
// src/core/implicitSurface/core/ImplicitField.h
#pragma once

#include "boundingBox.h"    // Mesh::LightBoundingBox
#include "meshLinearAlgebra.h"  // V3d

#include <memory>

// Forward declarations
namespace pgo::ImplicitSurface {
struct GridSpec;
class GridField;
}

namespace pgo::ImplicitSurface {

class ImplicitField {
public:
    virtual double eval(const V3d& p) const = 0;

    // Advisory bounds. Default: invalid LightBoundingBox = unbounded.
    // isUnbounded(bounds()) == true iff !bounds().verifyBox()
    virtual Mesh::LightBoundingBox bounds() const;

    // Batch eval. Default: loop over eval(). Override for SIMD or BVH batching.
    virtual void evalBatch(const V3d* pts, double* out, size_t n) const;

    // Sample this field to a GridField (parallel unless numThreads==0 → hardware concurrency).
    // WARNING: not safe for Python-trampoline subclasses if numThreads != 1.
    GridField sampleToGrid(const GridSpec& spec, int numThreads = 0) const;

    virtual ~ImplicitField() = default;
};

inline bool isUnbounded(const Mesh::LightBoundingBox& bb) { return !bb.verifyBox(); }

}  // namespace pgo::ImplicitSurface
```

**实现文件** `core/ImplicitField.cpp`：
- `bounds()` 默认返回 `Mesh::LightBoundingBox{}`（`bmin=+DBL_MAX`，`bmax=-DBL_MAX`，即 `!verifyBox()`）
- `evalBatch` 默认 for 循环
- `sampleToGrid`：分配 `GridField(spec)`，`#pragma omp parallel for collapse(3) num_threads(t)` 遍历 xyz，调 `eval`，填入 flat index

---

### 5.2 新增 `fields/GridField.h/cpp`（替代 DenseGrid）

```cpp
// src/core/implicitSurface/fields/GridField.h
#pragma once

#include "core/ImplicitField.h"
#include "field/gridSpec.h"

#include <vector>

namespace pgo::ImplicitSurface {

class GridField : public ImplicitField {
public:
    explicit GridField(const GridSpec& spec);

    // ImplicitField interface
    double eval(const V3d& p) const override;         // trilinear interpolation; clamps outside bounds
    Mesh::LightBoundingBox bounds() const override;

    // Direct array access (for extraction, buffer protocol, deprecated wrappers)
    double& at(int x, int y, int z);
    double  at(int x, int y, int z) const;
    double& operator[](int linearIdx);
    double  operator[](int linearIdx) const;
    double* data();
    const double* data() const;

    int resolution() const;
    int size() const;                // resolution^3
    const GridSpec& gridSpec() const;

    void fill(double value);
    void setZero();

private:
    GridSpec spec_;
    std::vector<double> data_;
};

// Deprecated alias for one version
using DenseGrid = GridField;

}  // namespace pgo::ImplicitSurface
```

**eval 实现**（三线性插值）：把世界坐标 p 映射到 `[0, resolution-1]` 格点坐标，做 `lerp` 三次，越界时 clamp 到边界值。

**GridSpec 一致性**：`GridField::gridSpec()` 代替 `DenseGrid::gridSpec()`，字段一致。现有调用 `DenseGrid::at(x,y,z)` / `DenseGrid::data()` 直接编译通过（alias）。

---

### 5.3 新增 `fields/SphereField.h/cpp`（重构原 geometry/sphereField.h）

```cpp
// src/core/implicitSurface/fields/SphereField.h
#pragma once

#include "core/ImplicitField.h"

namespace pgo::ImplicitSurface {

class SphereField : public ImplicitField {
public:
    V3d center;
    double radius;

    SphereField(const V3d& center, double radius);

    double eval(const V3d& p) const override;   // (p - center).norm() - radius
    Mesh::LightBoundingBox bounds() const override;

    // Factory: fit sphere to mesh bounding box (preserving old behavior)
    static SphereField fromMeshBBox(const Mesh::TriMeshGeo& mesh);

    // Geometry utility (NOT ImplicitField interface — kept for mesh projection use case)
    int projectOpenBoundaryToSphere(Mesh::TriMeshGeo& mesh) const;
};

}  // namespace pgo::ImplicitSurface
```

**迁移说明**：原 `geometry/sphereField.h` 里的自由函数：
- `computeSphereFieldFromBBox` → `SphereField::fromMeshBBox`（静态工厂）
- `projectOpenBoundaryToSphere` → `SphereField::projectOpenBoundaryToSphere`（成员）
- `thickenSphereShell(sphere, t, grid, out)` → 用 `OffsetOp`：`OffsetField(sphere_impl, 0.5*t).sampleToGrid(spec)`
- `evaluateBallSDF(sphere, grid, out)` → `sphere.sampleToGrid(spec)`

保留原自由函数为 deprecated wrapper（调用新 API），一个版本后删除。

OpenVDB 调用点 `buildOpenVDBSphereShell`、`buildOpenVDBBallLevelSet` 仍接收 `const SphereField&`，因为 `SphereField` IS-A `ImplicitField`，签名向后兼容。如果这两个函数的实现需要 `center` / `radius`，直接访问 public 字段。

---

### 5.4 新增 `fields/MeshUnsignedDistanceField.h/cpp`

```cpp
// src/core/implicitSurface/fields/MeshUnsignedDistanceField.h
#pragma once

#include "core/ImplicitField.h"
#include "triMeshGeo.h"
#include "boundingVolumeTree.h"

#include <memory>
#include <mutex>

namespace pgo::ImplicitSurface {

class MeshUnsignedDistanceField : public ImplicitField {
public:
    explicit MeshUnsignedDistanceField(Mesh::TriMeshGeo mesh);

    // Point query via TriMeshBVTree::closestTriangleQuery — BVH built lazily on first call
    double eval(const V3d& p) const override;

    Mesh::LightBoundingBox bounds() const override;

    // Override: uses libigl batch path (same as old computeMeshUnsignedDistance)
    // Much faster than calling eval() per voxel for grid sampling.
    GridField sampleToGrid(const GridSpec& spec, int numThreads = 0) const override;

private:
    Mesh::TriMeshGeo mesh_;
    mutable std::unique_ptr<Mesh::TriMeshBVTree> bvh_;
    mutable std::once_flag bvhFlag_;

    void ensureBVH() const;
};

}  // namespace pgo::ImplicitSurface
```

**eval 实现**：
```cpp
double MeshUnsignedDistanceField::eval(const V3d& p) const {
    ensureBVH();
    auto result = bvh_->closestTriangleQuery(mesh_.ref(), p);
    return std::sqrt(result.dist2);
}
```

**sampleToGrid override**：
```cpp
GridField MeshUnsignedDistanceField::sampleToGrid(const GridSpec& spec, int) const {
    // Use libigl batch — equivalent to old computeMeshUnsignedDistance
    EigenSupport::VXd dist;
    libiglInterface::computeDistanceField(
        mesh_, spec.bmin, spec.bmax, spec.resolution, /*robust=*/1, /*sign=*/0, dist);
    GridField grid(spec);
    for (int i = 0; i < grid.size(); ++i)
        grid[i] = dist[i];
    return grid;
}
```

**旧接口迁移**：`computeMeshUnsignedDistance(mesh, grid, outDist)` → deprecated wrapper：
```cpp
inline void computeMeshUnsignedDistance(const Mesh::TriMeshGeo& mesh,
    const GridSpec& spec, GridField& out) {
    out = MeshUnsignedDistanceField(mesh).sampleToGrid(spec);
}
```

---

### 5.5 新增 `fields/BoxField.h`

```cpp
// src/core/implicitSurface/fields/BoxField.h
#pragma once
#include "core/ImplicitField.h"

namespace pgo::ImplicitSurface {

class BoxField : public ImplicitField {
public:
    V3d center;
    V3d halfExtent;

    BoxField(const V3d& center, const V3d& halfExtent);
    BoxField(const Mesh::LightBoundingBox& bb);  // convenience ctor

    // SDF: max(abs(p-center) - halfExtent) — negative inside, positive outside
    double eval(const V3d& p) const override;
    Mesh::LightBoundingBox bounds() const override;
};

}  // namespace pgo::ImplicitSurface
```

Header-only（4 行实现），不需要 .cpp。

---

### 5.6 重构 `ops/BooleanOp.h/cpp`（惰性 BooleanField）

**旧的** `applyBoolean(a, b, op, out)` 立即分配并填充 `DenseGrid`。  
**新的** 提供惰性 `BooleanField` + 工厂函数，保留 `applyBoolean` 为 deprecated eager wrapper。

```cpp
// src/core/implicitSurface/ops/BooleanOp.h
#pragma once

#include "core/ImplicitField.h"
#include <memory>

namespace pgo::ImplicitSurface {

enum class BooleanOp { Union, Intersection, Difference };

class BooleanField : public ImplicitField {
public:
    BooleanField(std::shared_ptr<ImplicitField> a,
                 std::shared_ptr<ImplicitField> b,
                 BooleanOp op);

    double eval(const V3d& p) const override;
    Mesh::LightBoundingBox bounds() const override;

private:
    std::shared_ptr<ImplicitField> a_, b_;
    BooleanOp op_;
};

// Factory helpers
std::shared_ptr<ImplicitField> makeUnion(std::shared_ptr<ImplicitField> a,
                                          std::shared_ptr<ImplicitField> b);
std::shared_ptr<ImplicitField> makeIntersection(std::shared_ptr<ImplicitField> a,
                                                  std::shared_ptr<ImplicitField> b);
std::shared_ptr<ImplicitField> makeDifference(std::shared_ptr<ImplicitField> a,
                                               std::shared_ptr<ImplicitField> b);

// Deprecated: eager materialise (calls makeXxx(...).sampleToGrid(out.gridSpec()))
[[deprecated("Use BooleanField and sampleToGrid instead")]]
void applyBoolean(const GridField& a, const GridField& b, BooleanOp op, GridField& out);

}  // namespace pgo::ImplicitSurface
```

**BooleanField::bounds() 推导**：
```cpp
Mesh::LightBoundingBox BooleanField::bounds() const {
    auto ba = a_->bounds(), bb = b_->bounds();
    switch (op_) {
    case BooleanOp::Union:
        if (isUnbounded(ba) || isUnbounded(bb)) return {};   // 无界
        { auto r = ba; r.expand(bb); return r; }
    case BooleanOp::Intersection:
        if (isUnbounded(ba)) return bb;
        if (isUnbounded(bb)) return ba;
        return ba.getIntersection(bb);
    case BooleanOp::Difference:
        return ba;   // A - B 的 bounds 不超过 A
    }
}
```

---

### 5.7 新增 `ops/OffsetField.h`（替代 shellThickening）

```cpp
// src/core/implicitSurface/ops/OffsetField.h
#pragma once
#include "core/ImplicitField.h"
#include <memory>

namespace pgo::ImplicitSurface {

// eval(p) = inner->eval(p) - offset
// For shell thickening: OffsetField(distance_field, 0.5 * thickness)
class OffsetField : public ImplicitField {
public:
    OffsetField(std::shared_ptr<ImplicitField> inner, double offset);
    double eval(const V3d& p) const override;
    Mesh::LightBoundingBox bounds() const override;

private:
    std::shared_ptr<ImplicitField> inner_;
    double offset_;
};

// Deprecated eager wrapper
[[deprecated("Use OffsetField and sampleToGrid instead")]]
void thickenMeshShell(const GridField& meshDist, double thickness, GridField& out);

}  // namespace pgo::ImplicitSurface
```

Header-only（`eval` 是 2 行），deprecated wrapper 调 `OffsetField(...).sampleToGrid`。

---

### 5.8 更新 Extraction 层

**MarchingCubes**：签名从 `DenseGrid&` 改为 `const GridField&`，因为 `DenseGrid = GridField`，**无需修改**（alias 透明）。

**OpenVDB**：新增一个 overload：
```cpp
// 新增：任意 GridField → OpenVDB narrow-band level-set
[[nodiscard]] std::unique_ptr<OpenVDBLevelSet>
buildOpenVDBFromGridField(const GridField& field, const OpenVDBOptions& opts);
```
现有 `buildOpenVDBShellFromMesh`、`buildOpenVDBSphereShell`、`buildOpenVDBBallLevelSet`、`combineOpenVDBLevelSets`、`extractOpenVDBLevelSet` **签名不变**。

---

### 5.9 CMakeLists.txt 改动

```cmake
# src/core/implicitSurface/CMakeLists.txt

set(IMPLICIT_SURFACE_SOURCES
  core/ImplicitField.cpp        # NEW
  fields/GridField.cpp          # NEW (replaces field/denseGrid.cpp functionally)
  fields/SphereField.cpp        # NEW (refactored from geometry/sphereField.cpp)
  fields/MeshUnsignedDistanceField.cpp  # NEW
  ops/BooleanOp.cpp             # UPDATED (adds BooleanField)
  geometry/shellThickening.cpp  # KEPT (deprecated wrappers only)
  geometry/meshDistance.cpp     # KEPT (deprecated wrapper calling libigl)
  field/denseGrid.cpp           # KEPT until DenseGrid alias removed
  extraction/marchingCubesExtractor.cpp
  extraction/openVDBExtractor.cpp
)

set(IMPLICIT_SURFACE_HEADERS
  core/ImplicitField.h
  field/gridSpec.h
  fields/GridField.h
  fields/SphereField.h
  fields/MeshUnsignedDistanceField.h
  fields/BoxField.h
  ops/BooleanOp.h
  ops/OffsetField.h
  extraction/marchingCubesExtractor.h
  extraction/openVDBExtractor.h
)
```

不需要新增外部依赖（`TriMeshBVTree` 已在 `mesh` 里，`mesh` 已是硬依赖）。

---

### 5.10 deprecated 符号时间表

| 符号 | 在哪里 | 替代 | 删除时机 |
|------|-------|------|---------|
| `DenseGrid` alias | `fields/GridField.h` | `GridField` | 下一 minor 版本 |
| `computeMeshUnsignedDistance(mesh, grid, out)` | `geometry/meshDistance.h` | `MeshUnsignedDistanceField(mesh).sampleToGrid(spec)` | 同上 |
| `applyBoolean(a, b, op, out)` | `ops/BooleanOp.h` | `BooleanField + sampleToGrid` | 同上 |
| `thickenMeshShell(dist, t, out)` | `geometry/shellThickening.h` | `OffsetField(inner, 0.5*t).sampleToGrid(spec)` | 同上 |
| `thickenSphereShell(sphere, t, spec, out)` | `fields/SphereField.h` | `OffsetField(sphere_ptr, 0.5*t).sampleToGrid(spec)` | 同上 |
| `evaluateBallSDF(sphere, spec, out)` | `fields/SphereField.h` | `sphere.sampleToGrid(spec)` | 同上 |

---

## nanobind Binding 层（`implicit_bindings.cpp`）

新增文件：`src/python/pypgo/bindings/implicit_bindings.cpp`。在 `module.cpp` 中注册 `init_implicit_bindings(m)`。

### 6.1 GridSpec binding

```cpp
nb::class_<GridSpec>(m, "PyGridSpec")
    .def(nb::init([](nb::ndarray<double,nb::shape<3>> bmin,
                     nb::ndarray<double,nb::shape<3>> bmax, int res) {
        GridSpec g;
        g.bmin = V3d(bmin(0), bmin(1), bmin(2));
        g.bmax = V3d(bmax(0), bmax(1), bmax(2));
        g.resolution = res;
        validateGridSpec(g);   // 抛 std::runtime_error → Python ValueError
        return g;
    }), nb::arg("bmin"), nb::arg("bmax"), nb::arg("resolution"))
    .def_rw("resolution", &GridSpec::resolution)
    .def_prop_rw("bmin", ...)
    .def_prop_rw("bmax", ...);
```

### 6.2 GridField binding（buffer protocol）

```cpp
nb::class_<GridField, std::shared_ptr<GridField>>(m, "PyGridField", nb::buffer_protocol())
    .def_buffer([](GridField& g) {
        int r = g.resolution();
        return nb::buffer_info(g.data(),
            {(size_t)r, (size_t)r, (size_t)r},
            {(size_t)r*r*8, (size_t)r*8, (size_t)8});
    })
    .def("grid_spec",   &GridField::gridSpec)
    .def("resolution",  &GridField::resolution)
    .def("eval", [](const GridField& g, nb::ndarray<double,nb::shape<3>> p) {
        return g.eval(V3d(p(0), p(1), p(2)));
    })
    .def("alloc_like", [](const GridField& g) {
        return std::make_shared<GridField>(g.gridSpec());
    });
```

### 6.3 ImplicitField base binding

```cpp
nb::class_<ImplicitField, std::shared_ptr<ImplicitField>>(m, "PyImplicitField")
    .def("eval", [](const ImplicitField& f, nb::ndarray<double,nb::shape<3>> p) {
        nb::gil_scoped_release _;
        return f.eval(V3d(p(0), p(1), p(2)));
    })
    .def("sample_to_grid", [](const ImplicitField& f, const GridSpec& spec, int threads) {
        nb::gil_scoped_release _;
        return std::make_shared<GridField>(f.sampleToGrid(spec, threads));
    }, nb::arg("grid_spec"), nb::arg("num_threads") = 0)
    .def("bounds", [](const ImplicitField& f) -> nb::object {
        auto bb = f.bounds();
        if (isUnbounded(bb)) return nb::none();
        // return (bmin_array, bmax_array)
        ...
    });
```

### 6.4 Concrete field bindings

```cpp
// SphereField
nb::class_<SphereField, ImplicitField, std::shared_ptr<SphereField>>(m, "PySphereField")
    .def(nb::init<V3d, double>(), nb::arg("center"), nb::arg("radius"))
    .def_rw("center", &SphereField::center)
    .def_rw("radius", &SphereField::radius)
    // 静态工厂：接收 PyTriMeshData，返回 shared_ptr<SphereField>
    .def_static("from_mesh_bbox", [](const Mesh::MeshData<3>& data) {
        return std::make_shared<SphereField>(
            SphereField::fromMeshBBox(Mesh::TriMeshGeo(data)));
    }, nb::arg("surface_data"));

// MeshUnsignedDistanceField
nb::class_<MeshUnsignedDistanceField, ImplicitField, std::shared_ptr<MeshUnsignedDistanceField>>(
    m, "PyMeshUnsignedDistanceField")
    .def(nb::init([](const Mesh::MeshData<3>& data) {
        return std::make_shared<MeshUnsignedDistanceField>(Mesh::TriMeshGeo(data));
    }), nb::arg("surface_data"));

// BoxField
nb::class_<BoxField, ImplicitField, std::shared_ptr<BoxField>>(m, "PyBoxField")
    .def(nb::init<V3d, V3d>(), nb::arg("center"), nb::arg("half_extent"))
    .def(nb::init([](nb::ndarray<double,nb::shape<3>> bmin,
                     nb::ndarray<double,nb::shape<3>> bmax) {
        Mesh::LightBoundingBox bb(V3d(bmin(0),bmin(1),bmin(2)),
                                   V3d(bmax(0),bmax(1),bmax(2)));
        return std::make_shared<BoxField>(bb);
    }), nb::arg("bmin"), nb::arg("bmax"));
```

### 6.5 Boolean + Offset bindings（自由函数）

```cpp
m.def("implicit_union", &makeUnion);
m.def("implicit_intersection", &makeIntersection);
m.def("implicit_difference", &makeDifference);
m.def("implicit_offset", [](std::shared_ptr<ImplicitField> inner, double offset) {
    return std::make_shared<OffsetField>(inner, offset);
});
```

### 6.6 Extraction bindings

```cpp
m.def("extract_marching_cubes",
    [](const GridField& field, double iso_offset) {
        MarchingCubesOptions opts; opts.isoOffset = iso_offset;
        Mesh::TriMeshGeo out;
        {
            nb::gil_scoped_release _;
            extractMarchingCubes(field, opts, out);
        }
        return out.toMeshData();
    }, nb::arg("field"), nb::arg("iso_offset") = 0.0);

m.def("has_openvdb", &has_openvdb_impl);  // returns compile-time bool

// OpenVDB opaque handle
nb::class_<OpenVDBLevelSet, std::shared_ptr<OpenVDBLevelSet>>(m, "PyOpenVDBLevelSet");

m.def("build_openvdb_shell_from_mesh", [](const Mesh::MeshData<3>& data,
        double thickness, ...) {
    nb::gil_scoped_release _;
    return buildOpenVDBShellFromMesh(Mesh::TriMeshGeo(data), thickness, opts);
});
m.def("build_openvdb_from_grid_field", [](const GridField& field, ...) {
    nb::gil_scoped_release _;
    return buildOpenVDBFromGridField(field, opts);
});
m.def("extract_openvdb", [](const OpenVDBLevelSet& ls, ...) {
    Mesh::TriMeshGeo out;
    nb::gil_scoped_release _;
    extractOpenVDBLevelSet(ls, opts, out);
    return out.toMeshData();
});
```

---

## Python 层（`pypgo/implicit.py`）

### 7.1 GridSpec

```python
# pypgo/implicit.py
import numpy as np
import pypgo._core as _core
from pypgo.mesh import TriMeshData

class GridSpec:
    def __init__(self, bmin, bmax, resolution: int):
        bmin = np.asarray(bmin, dtype=np.float64).ravel()
        bmax = np.asarray(bmax, dtype=np.float64).ravel()
        if bmin.shape != (3,) or bmax.shape != (3,):
            raise ValueError("bmin and bmax must be 3-element arrays")
        # _core 构造时执行 validateGridSpec (分辨率 < 2 / bmin >= bmax 等会抛 RuntimeError)
        self._core_obj = _core.PyGridSpec(bmin, bmax, int(resolution))

    @classmethod
    def from_mesh(cls, mesh: TriMeshData, resolution: int, padding: float = 0.1) -> "GridSpec":
        """Auto-fit bounds to mesh with fractional padding."""
        bmin, bmax = mesh.bbox
        pad = (bmax - bmin) * padding
        return cls(bmin - pad, bmax + pad, resolution)

    @property
    def resolution(self) -> int: return self._core_obj.resolution
    @property
    def bmin(self) -> np.ndarray: ...
    @property
    def bmax(self) -> np.ndarray: ...
    def __repr__(self): ...
```

### 7.2 ImplicitField 包装基类

`ImplicitField` 是所有场类型的 Python 基类，反映 C++ 的继承层次。
`GridField`、`SphereField`、`MeshUnsignedDistanceField`、`BoxField` 都继承它。
Python 用户**不能**子类化并覆盖 `eval`（nanobind trampoline 推迟到 future_work），
但可以持有、组合、传递任意 `ImplicitField` 子类实例。

```python
class ImplicitField:
    """Wraps a shared_ptr<PyImplicitField>. All CSG ops return ImplicitField."""

    def __init__(self, core_obj):
        self._core_obj = core_obj

    def eval(self, p) -> float:
        p = np.asarray(p, dtype=np.float64).ravel()
        if p.shape != (3,):
            raise ValueError("p must be a 3-element array")
        return self._core_obj.eval(p)

    def sample_to_grid(self, grid_spec: GridSpec, *, num_threads: int = 0) -> GridField:
        core = self._core_obj.sample_to_grid(grid_spec._core_obj, num_threads)
        return GridField(core)

    def bounds(self):
        """Return (bmin, bmax) arrays or None if unbounded."""
        return self._core_obj.bounds()

    # CSG operators
    def __or__(self, other: "ImplicitField") -> "ImplicitField":
        return ImplicitField(_core.implicit_union(self._core_obj, other._core_obj))

    def __and__(self, other: "ImplicitField") -> "ImplicitField":
        return ImplicitField(_core.implicit_intersection(self._core_obj, other._core_obj))

    def __sub__(self, other: "ImplicitField") -> "ImplicitField":
        return ImplicitField(_core.implicit_difference(self._core_obj, other._core_obj))

    def offset(self, value: float) -> "ImplicitField":
        """Shift the iso-level by value. Positive → shrink, negative → grow."""
        return ImplicitField(_core.implicit_offset(self._core_obj, float(value)))

    @classmethod
    def _from_core(cls, core_obj) -> "ImplicitField":
        """Internal factory used by subclass classmethods to wrap a core object."""
        obj = object.__new__(cls)
        obj._core_obj = core_obj
        return obj
```

### 7.3 GridField

`GridField` 继承 `ImplicitField`，因此它也有 `|`、`&`、`-`、`.offset()` 和 `.sample_to_grid()`。
`extract_marching_cubes` 要求参数必须是 `GridField`（已物化的场），对尚未物化的 `ImplicitField` 会抛 `TypeError`。

```python
class GridField(ImplicitField):
    """Sampled scalar field on a uniform grid. Inherits ImplicitField: eval via
    trilinear interpolation, CSG operators, offset — all work on GridField.
    Exposes .values as a zero-copy NumPy view (shape = (res, res, res))."""

    def __init__(self, core_obj):
        # core_obj is PyGridField, which IS-A PyImplicitField in C++
        super().__init__(core_obj)

    @property
    def values(self) -> np.ndarray:
        """Zero-copy NumPy view, shape=(res, res, res), dtype=float64."""
        return np.asarray(self._core_obj)

    @property
    def grid_spec(self) -> GridSpec: ...
```

### 7.4 具体场类型

```python
class SphereField(ImplicitField):
    def __init__(self, center, radius: float):
        center = np.asarray(center, dtype=np.float64).ravel()
        if center.shape != (3,):
            raise ValueError("center must be a 3-element array")
        super().__init__(_core.PySphereField(center, float(radius)))

    @classmethod
    def from_mesh_bbox(cls, mesh: TriMeshData) -> "SphereField":
        """Fit a sphere to the bounding box of the given surface mesh."""
        if not isinstance(mesh, TriMeshData):
            raise TypeError(f"mesh must be TriMeshData, got {type(mesh).__name__}")
        return cls._from_core(_core.PySphereField.from_mesh_bbox(mesh._core_obj))

    @property
    def center(self) -> np.ndarray: ...
    @property
    def radius(self) -> float: ...


class MeshUnsignedDistanceField(ImplicitField):
    """Unsigned distance field from a triangle surface mesh.

    eval() uses a BVH for per-point queries.
    sample_to_grid() uses libigl batch distance (faster for dense grids).
    Signed distance is not implemented in this version.
    """
    def __init__(self, mesh: TriMeshData):
        if not isinstance(mesh, TriMeshData):
            raise TypeError(f"mesh must be TriMeshData, got {type(mesh).__name__}")
        super().__init__(_core.PyMeshUnsignedDistanceField(mesh._core_obj))


class BoxField(ImplicitField):
    """Axis-aligned box SDF (AABB). eval < 0 inside, > 0 outside."""

    def __init__(self, center, half_extent):
        center     = np.asarray(center,      dtype=np.float64).ravel()
        half_extent = np.asarray(half_extent, dtype=np.float64).ravel()
        if center.shape != (3,) or half_extent.shape != (3,):
            raise ValueError("center and half_extent must be 3-element arrays")
        super().__init__(_core.PyBoxField(center, half_extent))

    @classmethod
    def from_bbox(cls, bmin, bmax) -> "BoxField":
        bmin = np.asarray(bmin, dtype=np.float64).ravel()
        bmax = np.asarray(bmax, dtype=np.float64).ravel()
        return cls._from_core(_core.PyBoxField(bmin=bmin, bmax=bmax))
```

### 7.5 Extraction 函数

```python
def extract_marching_cubes(field: GridField, *, iso_offset: float = 0.0) -> TriMeshData:
    if not isinstance(field, GridField):
        raise TypeError("field must be a GridField; call .sample_to_grid() first")
    return TriMeshData(_core.extract_marching_cubes(field._core_obj, float(iso_offset)))

def has_openvdb() -> bool:
    return bool(_core.has_openvdb())

class OpenVDBOptions:
    def __init__(self, voxel_size: float, half_width: float = 3.0,
                 adaptivity: float = 0.0, smooth_steps: int = 0): ...

def build_openvdb_shell_from_mesh(mesh: TriMeshData, shell_thickness: float,
                                   options: OpenVDBOptions):
    """Returns an opaque OpenVDBLevelSet handle."""
    if not has_openvdb():
        raise RuntimeError("OpenVDB is not available in this build")
    ...

def extract_openvdb(levelset, options: OpenVDBOptions) -> TriMeshData:
    if not has_openvdb():
        raise RuntimeError("OpenVDB is not available in this build")
    ...

# One-shot convenience
def thicken_mesh_surface(
    mesh: TriMeshData,
    *,
    thickness: float,
    resolution: int,
    padding: float = 0.1,
    iso_offset: float = 0.0,
) -> TriMeshData:
    """Extract a volumetrically-thickened surface mesh around the input mesh.

    Pipeline: mesh → MeshUnsignedDistanceField → OffsetField(0.5*thickness)
              → sample_to_grid → marching cubes.

    The output is a closed triangle mesh representing the shell of the input
    surface with the specified total thickness (each side ≈ 0.5*thickness).

    Args:
        mesh:       Input closed triangle surface mesh.
        thickness:  Total shell thickness in mesh units.
        resolution: Grid resolution (number of voxels per axis).
        padding:    Fractional padding added around mesh bounds for the grid
                    (default 0.1 = 10% per side). Increase if the shell is
                    clipped at the boundary.
        iso_offset: Shift the marching-cubes iso-level (default 0 = zero-crossing).
    """
    grid_spec = GridSpec.from_mesh(mesh, resolution, padding=padding)
    # MeshUnsignedDistanceField.sample_to_grid uses libigl batch path (fast)
    dist_field = MeshUnsignedDistanceField(mesh)
    shell_field = dist_field.offset(0.5 * thickness)   # ImplicitField, still lazy
    grid = shell_field.sample_to_grid(grid_spec)        # GridField, now materialised
    return extract_marching_cubes(grid, iso_offset=iso_offset)
```

---

## 公开 API 一览

### Python（`pypgo.implicit`）

```python
# Types
GridSpec(bmin, bmax, resolution)
GridSpec.from_mesh(mesh, resolution, padding=0.1)

# Base (all concrete fields inherit this)
ImplicitField                           # .eval(p), .sample_to_grid(spec), .bounds()
                                        # | & - .offset(v)

# Concrete fields (all inherit ImplicitField)
GridField                               # also: .values (numpy zero-copy view), .grid_spec
SphereField(center, radius)
SphereField.from_mesh_bbox(mesh)
MeshUnsignedDistanceField(mesh)         # eval via BVH; sample_to_grid via libigl batch
BoxField(center, half_extent)           # AABB
BoxField.from_bbox(bmin, bmax)

# Extraction (requires GridField, not bare ImplicitField)
extract_marching_cubes(field: GridField, *, iso_offset=0.0) -> TriMeshData
has_openvdb() -> bool
OpenVDBOptions(voxel_size, ...)
build_openvdb_shell_from_mesh(mesh, thickness, options)
extract_openvdb(levelset, options) -> TriMeshData

# One-shot convenience
thicken_mesh_surface(mesh, *, thickness, resolution,
                     padding=0.1, iso_offset=0.0) -> TriMeshData
```

### Private `_core`

```
PyGridSpec, PyGridField (buffer_protocol), PyImplicitField
PySphereField, PyMeshUnsignedDistanceField, PyBoxField
PyOpenVDBLevelSet (opaque)
implicit_union / implicit_intersection / implicit_difference / implicit_offset
extract_marching_cubes(field, iso_offset) -> PyTriMeshData
has_openvdb() -> bool
build_openvdb_shell_from_mesh / build_openvdb_from_grid_field / extract_openvdb
```

---

## 文件改动一览

| 文件 | 类型 | 说明 |
|------|------|------|
| `src/core/implicitSurface/core/ImplicitField.h/cpp` | 新增 | 核心抽象 |
| `src/core/implicitSurface/fields/GridField.h/cpp` | 新增 | 替代 DenseGrid，加 eval + sampleToGrid |
| `src/core/implicitSurface/fields/SphereField.h/cpp` | 新增（重构自 geometry/） | ImplicitField 实现 |
| `src/core/implicitSurface/fields/MeshUnsignedDistanceField.h/cpp` | 新增 | BVH point query + libigl batch override |
| `src/core/implicitSurface/fields/BoxField.h` | 新增 | header-only |
| `src/core/implicitSurface/ops/BooleanOp.h/cpp` | 更新 | 加 BooleanField；保留 applyBoolean deprecated |
| `src/core/implicitSurface/ops/OffsetField.h` | 新增 | header-only；thickenMeshShell deprecated wrapper |
| `src/core/implicitSurface/geometry/shellThickening.h/cpp` | 保留（deprecated wrappers） | 调用 OffsetField |
| `src/core/implicitSurface/geometry/meshDistance.h/cpp` | 保留（deprecated wrapper） | 调用 MeshUnsignedDistanceField |
| `src/core/implicitSurface/geometry/sphereField.h/cpp` | 保留（deprecated wrappers） | 调用 SphereField 新 API |
| `src/core/implicitSurface/field/denseGrid.h/cpp` | 保留（`using DenseGrid = GridField`） | 一个版本后删除 |
| `src/core/implicitSurface/extraction/openVDBExtractor.h/cpp` | 更新 | 新增 buildOpenVDBFromGridField |
| `src/core/implicitSurface/CMakeLists.txt` | 更新 | 加入新文件 |
| `src/python/pypgo/bindings/implicit_bindings.cpp` | 新增 | nanobind binding TU |
| `src/python/pypgo/bindings/module.cpp` | 更新 | 注册 init_implicit_bindings |
| `src/python/pypgo/CMakeLists.txt` | 更新 | 加入 implicit_bindings.cpp |
| `pypgo/implicit.py` | 新增 | Python public API |
| `pypgo/__init__.py` | 更新 | 加入 `from pypgo import implicit` |

---

## 测试清单

### C++ 测试（GTest）

**ImplicitField / GridField：**
- `GridField(spec).eval(p)` 在 grid 节点处返回精确 stored value
- `GridField::eval` 在节点之间做线性插值（验证 3 个 lerp 轴）
- `GridField::eval` 在边界外 clamp 而不 segfault
- `GridField::sampleToGrid` 等价于直接 `fill`（用已知解析场 SphereField 验证）
- `np.asarray(grid_field)` 共享内存（修改 view 后 `grid_field[i]` 改变）

**SphereField：**
- `eval(center) == -radius`
- `eval(center + V3d(radius, 0, 0)) ≈ 0`
- `bounds().bmin ≈ center - radius`, `bounds().bmax ≈ center + radius`
- `sampleToGrid` 后 marching cubes 输出网格顶点到球心距离 ≈ radius (相对误差 < 1/resolution)

**BoxField：**
- `eval(center) == -min(halfExtent)`（负值，内部）
- `eval(center + halfExtent * 1.5) > 0`

**MeshUnsignedDistanceField：**
- `eval(v)` 对网格顶点返回 ≈ 0
- `sampleToGrid` 数值结果与旧 `computeMeshUnsignedDistance` 一致（逐点误差 < 1e-10）
- BVH 懒构建：第一次 `eval` 触发，第二次不重建（用计数 hook 验证）

**BooleanField：**
- Union: `eval(p) == min(a.eval(p), b.eval(p))`
- Intersection: `eval(p) == max(a.eval(p), b.eval(p))`
- Difference: `eval(p) == max(a.eval(p), -b.eval(p))`
- Union bounds 是两者的包围盒并集
- Intersection bounds 是两者的包围盒交集
- Difference bounds 等于 a.bounds()

**OffsetField：**
- `eval(p) == inner.eval(p) - offset`

**Deprecated wrappers：**
- `applyBoolean` 输出与 `BooleanField.sampleToGrid` 逐点一致
- `thickenMeshShell` 输出与 `OffsetField.sampleToGrid` 逐点一致

### Python 测试（pytest）

```python
# tests/pypgo/test_implicit.py

def test_grid_spec_validation():
    with pytest.raises(RuntimeError):
        GridSpec([0,0,0], [1,1,1], resolution=1)   # < 2

def test_grid_field_numpy_zerocopy():
    g = SphereField([0,0,0], 1.0).sample_to_grid(
            GridSpec([-2,-2,-2], [2,2,2], 16))
    arr = g.values
    assert arr.shape == (16, 16, 16)
    assert arr.dtype == np.float64
    # zero-copy: modifying arr changes grid
    arr[0, 0, 0] = 999.0
    assert g.eval(g.grid_spec.bmin) == pytest.approx(999.0)

def test_sphere_field_eval():
    s = SphereField([0, 0, 0], 1.0)
    assert s.eval([0, 0, 0]) == pytest.approx(-1.0)
    assert s.eval([1, 0, 0]) == pytest.approx(0.0, abs=1e-12)
    assert s.eval([2, 0, 0]) == pytest.approx(1.0)

def test_csg_operators():
    s1 = SphereField([0,0,0], 1.0)
    s2 = SphereField([0.5,0,0], 1.0)
    union = s1 | s2
    assert union.eval([0, 0, 0]) == pytest.approx(min(s1.eval([0,0,0]), s2.eval([0,0,0])))
    diff = s1 - s2
    assert diff.eval([0, 0, 0]) == pytest.approx(max(s1.eval([0,0,0]), -s2.eval([0,0,0])))

def test_lazy_no_intermediate_alloc():
    # 连续 5 次 CSG 不会分配中间 GridField
    fields = [SphereField([i*0.3, 0, 0], 1.0) for i in range(5)]
    combined = fields[0]
    for f in fields[1:]:
        combined = combined | f
    # 只在 sample_to_grid 时分配
    grid = combined.sample_to_grid(GridSpec([-3,-3,-3], [3,3,3], 32))
    assert grid.values.shape == (32, 32, 32)

def test_thicken_mesh_surface_pipeline():
    bunny = pgo.mesh.read_obj("pypgo/examples/assets/obj/bunny.obj")
    shell = thicken_mesh_surface(bunny, thickness=0.01, resolution=64)
    assert isinstance(shell, TriMeshData)
    assert shell.num_vertices > 0

def test_mesh_distance_field_consistency():
    """sampleToGrid 结果与旧 computeMeshUnsignedDistance 一致。"""
    ...

def test_openvdb_guard():
    if not has_openvdb():
        with pytest.raises(RuntimeError, match="not available"):
            build_openvdb_shell_from_mesh(...)

def test_extract_marching_cubes_type_error():
    with pytest.raises(TypeError, match="GridField"):
        extract_marching_cubes(SphereField([0,0,0], 1.0))  # 未物化的 ImplicitField
```

---

## Build / Verify 命令

```bash
# C++ 编译（base preset）
conda run -n libpgo cmake --preset base
conda run -n libpgo cmake --build --preset base -j8

# C++ 测试
conda run -n libpgo ctest --test-dir build/base -R "ImplicitSurface|GridField|SphereField|MeshDistance|BooleanField" --output-on-failure

# Python 编译
conda run -n libpgo cmake --preset pypgo
conda run -n libpgo cmake --build --preset pypgo -j8

# Python 测试
conda run -n libpgo python -m pytest tests/pypgo/test_implicit.py -v
```

---

## Self-Review

> 初稿后的自查记录。已解决的问题用 ✅ 标注；仍需关注的用 ⚠️ 标注。

### 已修正的问题

**P1 ✅：`thicken_mesh_surface` 路径修正**

原来 `dist` 是 `GridField` 但调了 `.offset()`（`GridField` 独立时没有这个方法）。
修正：`GridField` 继承 `ImplicitField`（7.3），且 `thicken_mesh_surface` 改为先构造 `MeshUnsignedDistanceField` 再 `.offset()`，dist_field 是 `ImplicitField` 而不是 `GridField`。pipeline 正确。

**P2 ✅：`SphereField.from_mesh_bbox` Python factory 修正**

原来 `cls.__new__(cls).__init__(...)` 返回 None。
修正：7.2 引入 `ImplicitField._from_core(core_obj)` 类方法，所有子类的静态工厂统一用 `cls._from_core(...)` 包装 core object。

**P3 ✅：`GridField` Python 类型关系明确**

`class GridField(ImplicitField)`，反映 C++ 继承层次。`extract_marching_cubes` 用 `isinstance(field, GridField)` guard。

**P4 ✅：`PySphereField.from_mesh_bbox` binding 补入 6.4**

接收 `MeshData<3>`，返回 `shared_ptr<SphereField>`。

**Q4 ✅：`mesh_to_shell` 命名不准确**

改为 `thicken_mesh_surface`。"thicken" 直接描述操作语义（给输入表面加厚度），"surface" 表明输入是表面网格。`padding` 参数已透传（解决了原来的附带问题）。

**Q1 ✅：Python trampoline 推迟到 future_work，Python 侧保留继承结构**

所有具体场类型继承 Python `ImplicitField` 基类；用户无法覆盖 `eval`，但可以持有、组合、传递任意子类实例。

**Q2 ✅：只实现 unsigned distance；类名为 `MeshUnsignedDistanceField`**

**Q3 ✅：BoxField 本期 AABB only**

### 仍需关注（实现时确认）

**U1 ⚠️：`GridField::eval` 越界 clamp 行为**

计划写"clamp 格点坐标到 `[0, resolution-1]` 再三线性插值"，但没有在 C++ 头文件的 API 注释里写明。实现时需要在 `GridField.h` 的 `eval` 声明处加注释。

**U2 ⚠️：`sampleToGrid(numThreads=0)` 语义**

`0` = "不调 `omp_set_num_threads`，让 OpenMP 用自己的默认值（通常 = hardware concurrency）"。实现时记录在 `ImplicitField.h` API 注释里。

**U3 ⚠️：`BooleanField::bounds()` 双无界 Intersection 情况**

`isUnbounded(ba) && isUnbounded(bb)` → `getIntersection` 结果仍无界（`!verifyBox()`），行为正确但需在代码注释里说明，避免以后维护者觉得是 bug。

**U5 ⚠️：OpenVDB binding 细节省略（用 `...` 占位）**

`OpenVDBOptions` 的 nanobind binding 需要在实现阶段展开；建议 bind 为 Python class，与 `MarchingCubesOptions` 模式一致。
