# volumetricMesh Phase F 执行文档

## 1. 结论与目标

基于当前代码状态，Phase E 可以视为已经完成，可以正式进入 Phase F。

判断依据：

1. 几何与材料存储已经拆成：
   - [volumetric_mesh_data.h](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/internal/volumetric_mesh_data.h)
   - [material_catalog.h](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/internal/material_catalog.h)
2. I/O 已经拆到 `io/`，`volumetricMeshIO.*` 已被删掉，当前 public I/O 入口是：
   - [mesh_load.h](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/io/mesh_load.h)
   - [mesh_save.h](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/io/mesh_save.h)
   - [mesh_format_detector.h](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/io/mesh_format_detector.h)
3. Tet / Cubic 的 element kernel 与高层算法已经拆出：
   - `ops/`
   - `algorithms/`
4. 材料系统已经切到值类型：
   - [material_record.h](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/materials/material_record.h)
   - [material_access.h](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/materials/material_access.h)
5. 当前最小验收集仍通过：
   - `core.scene.volumetric_mesh`
   - `core.energy.scene_to_simulation_mesh`
   - `core.energy.material_binding`
   - `api.pgo_c.basic`
   - `cubicMesher.tool.uniform`

因此，下一阶段最值得做的事情已经不是继续薄化 [volumetricMesh.h](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/volumetricMesh.h)，而是**彻底移除 `VolumetricMesh` 这个中心基类**，把模块重构成：

1. **组合式共享存储**
2. **traits / concepts 驱动的 mesh-specific 能力**
3. **mesh-agnostic 模板算法**
4. **只在边界层保留运行时分发**

Phase F 的最终目标不是“得到一个更小的 `VolumetricMesh`”，而是：

- 允许 [volumetricMesh.h](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/volumetricMesh.h) / [volumetricMesh.cpp](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/volumetricMesh.cpp) 被完全删除
- 让 `TetMesh` / `CubicMesh` 不再依赖虚函数继承
- 让算法层形成真正的 `mesh-agnostic + mesh-specific implementation` 结构

## 2. 当前基线

### 2.1 当前 `VolumetricMesh` 还承担的职责

虽然前几个阶段已经显著收缩了基类，但 [volumetricMesh.h](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/volumetricMesh.h) 仍然同时承担：

- 公共枚举：
  - `ElementType`
  - `FileFormatType`
- 共享数据拥有：
  - `m_geometry`
  - `m_material_catalog`
- 统一查询 façade：
  - `getVerticesInElements`
  - `getElementsTouchingVertices`
  - `getContainingElement`
  - `getMass`
  - `getBoundingBox`
- 统一变换 façade：
  - `computeGravity`
  - `applyDeformation`
  - `applyLinearTransformation`
- 运行时多态入口：
  - `getElementVolume`
  - `computeElementMassMatrix`
  - `containsVertex`
  - `computeBarycentricWeights`
  - `interpolateGradient`

这意味着当前的“共享数据”、“算法入口”和“元素几何差异”仍然绑在一个类型层次里。

### 2.2 当前真正依赖 `VolumetricMesh` 基类的模块

从当前仓库实际调用面看，以下模块还把 `VolumetricMesh` 当成统一接口：

- [barycentricCoordinates.h](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/interpolationCoordinates/barycentricCoordinates.h)
- [mesh_interpolation.h](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/algorithms/mesh_interpolation.h)
- [generate_mass_matrix.h](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/algorithms/generate_mass_matrix.h)
- [generate_surface_mesh.h](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/algorithms/generate_surface_mesh.h)
- [mesh_save.h](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/io/mesh_save.h)
- [mesh_queries.h](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/internal/mesh_queries.h)
- [mesh_mass_properties.h](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/internal/mesh_mass_properties.h)
- [mesh_transforms.h](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/internal/mesh_transforms.h)

这说明 Phase F 的难点不在 `TetMesh` / `CubicMesh` 自己，而在**把这批统一入口改造成模板算法或边界分发**。

### 2.3 当前已经具备的有利条件

Phase F 不需要从零开始，因为我们已经有了很好的基础：

1. `TetMesh` / `CubicMesh` 的 element-specific 几何核已经在：
   - [tet_mesh_ops.h](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/ops/tet_mesh_ops.h)
   - [cubic_mesh_ops.h](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/ops/cubic_mesh_ops.h)
2. 共享几何/材料状态已经是组合式成员，而不是裸数组
3. I/O 已经被拆散，不再需要一个基类承担“读完文件如何装配”的中心职责
4. 材料系统已经值类型化，不再需要通过虚基类参与 mesh 继承层次

换句话说，Phase F 是“把前几阶段已经拆出来的内容重新组织成正确的类型系统”，而不是再做一轮大拆文件。

## 3. 范围与非目标

### 3.1 本阶段范围

- 把 mesh 共有枚举和共有值类型从 `VolumetricMesh` 中抽离
- 引入共享 storage 类型，替代基类的数据拥有职责
- 建立 `mesh_traits` / `mesh_concepts`
- 把 mesh-agnostic 模块改成模板化或 traits 驱动
- 把运行时分发收口到边界层
- 迁移下游并最终删除 `volumetricMesh.h/.cpp`

### 3.2 本阶段不做

- 不在 Phase F 内继续重构材料模型
- 不在 Phase F 内统一 `std::span` / `std::optional` 风格
- 不在 Phase F 内引入新的 ABI 兼容层
- 不在 Phase F 内为了“保留 `VolumetricMesh` 名字”而再造一个 CRTP 大基类

原因：

- 材料系统已经完成 Phase E，这一轮不应回滚边界
- `span` / `optional` 是下一阶段 API 现代化，和去基类化是两个维度
- 你当前目标是**删掉错误中心抽象**，不是维持兼容

## 4. 先固定的设计决策

### 4.1 不再引入新的“大基类替代品”

Phase F 明确禁止这两种做法：

1. 把 `VolumetricMesh` 改名后继续保留同样的中心职责
2. 用 CRTP 重新造一个新的“大而全模板基类”

理由很简单：

- 问题不是“虚函数慢”
- 问题是“共享数据、元素差异、算法入口、边界分发全塞进了一个层次”

因此正确方向是：

- **共享数据**：组合
- **元素差异**：traits + ops
- **通用算法**：template / concept
- **运行时未知类型**：边界层 `variant`

### 4.2 引入 `MeshStorage`

建议新增：

```text
src/core/scene/volumetricMesh/storage/
├── mesh_storage.h
└── mesh_storage.cpp
```

固定职责：

- 持有：
  - `internal::VolumetricMeshData geometry`
  - `internal::MaterialCatalog materials`
- 不提供任何 element-specific 虚函数
- 只提供基础状态访问和 invariant helper

建议形状：

```cpp
struct MeshStorage {
    internal::VolumetricMeshData geometry;
    internal::MaterialCatalog material_catalog;
};
```

注意：

- 这里不要求把 `VolumetricMeshData` / `MaterialCatalog` 再降级成原始数组
- Phase B/E 的成果应直接复用

### 4.3 引入 `mesh_traits` / `mesh_concepts`

建议新增：

```text
src/core/scene/volumetricMesh/traits/
├── mesh_traits.h
└── mesh_tags.h

src/core/scene/volumetricMesh/concepts/
└── mesh_concepts.h
```

最小公共概念建议固定为：

```cpp
template <class MeshT>
concept VolumetricMeshLike = requires(const MeshT& mesh, int el, Vec3d pos, double* weights) {
    { mesh.getNumVertices() } -> std::convertible_to<int>;
    { mesh.getNumElements() } -> std::convertible_to<int>;
    { mesh.getNumElementVertices() } -> std::convertible_to<int>;
    { mesh.getVertexIndex(el, 0) } -> std::convertible_to<int>;
    ops::compute_barycentric_weights(mesh, el, pos, weights);
    { ops::contains_vertex(mesh, el, pos) } -> std::same_as<bool>;
    { ops::element_volume(mesh, el) } -> std::convertible_to<double>;
};
```

注意：

- Phase F 的目标不是把所有算法都写成 concepts-heavy 模板秀
- 目标是让编译期约束能清楚表达“这个算法需要 mesh 提供哪些能力”

### 4.4 运行时分发只留在边界层

建议新增一个非常小的边界类型，例如：

```cpp
using AnyMeshRef = std::variant<
    std::reference_wrapper<const TetMesh>,
    std::reference_wrapper<const CubicMesh>>;
```

只允许它出现在真正需要运行时未知 mesh 类型的入口里，例如：

- `generate_surface_mesh`
- `mesh_save`
- 少量 tools 入口

禁止把它继续往下传到：

- `mesh_queries`
- `mesh_mass_properties`
- `mesh_interpolation`

也就是说：

- **边界层可以 variant**
- **算法层必须 template / trait**

### 4.5 `TetMesh` / `CubicMesh` 改成组合式类型

Phase F 结束后的理想形态应接近：

```cpp
class TetMesh final {
public:
    // public API

private:
    MeshStorage m_storage;
};

class CubicMesh final {
public:
    // public API

private:
    MeshStorage m_storage;
    double m_cube_size;
    double m_inv_cube_size;
    int m_parallelepiped_mode;
};
```

即：

- `TetMesh` / `CubicMesh` 保留公共概念名字
- 但不再 `public VolumetricMesh`
- 共有数据通过 `MeshStorage` 组合获得

## 5. 目标目录结构

建议在现有目录上继续扩展为：

```text
src/core/scene/volumetricMesh/
├── algorithms/
├── concepts/
│   └── mesh_concepts.h
├── io/
├── materials/
├── ops/
├── storage/
│   ├── mesh_storage.h
│   └── mesh_storage.cpp
├── traits/
│   ├── mesh_tags.h
│   └── mesh_traits.h
├── types/
├── tetMesh.h/.cpp
├── cubicMesh.h/.cpp
└── volumetricMesh.h/.cpp   # 最终删除
```

注意：

- `storage/` 是 Phase F 的新增核心
- 不建议再把这些塞回 `internal/`，否则“真正的架构层”会继续隐藏在 `internal` 垃圾桶里

## 6. 阶段划分

### 阶段 F1：先解除 `VolumetricMesh` 作为公共类型中心的头文件耦合

1. **把 `ElementType` / `FileFormatType` 从 `VolumetricMesh` 中抽离** `[M]` `[low-risk]`
   - 新增：
     - `types/mesh_kind.h`
     - `types/file_format.h`
   - 调整：
     - `io/mesh_format_detector.*`
     - `io/mesh_io_types.h`
     - tests 中对 `VolumetricMesh::ElementType` 的引用
   - 为什么：只要枚举还挂在基类上，I/O 和 tests 就不得不 include `volumetricMesh.h`

2. **把 `editing` 入口从 `VolumetricMesh` 命名空间中剥离成自由函数模块** `[M]` `[med-risk]`
   - 目标：
     - 不再需要 `namespace editing` 依赖基类类型
   - 依赖：步骤 1
   - 为什么：这条线仍在把 `VolumetricMesh` 当公共中心对象

3. **梳理所有仍然只为拿共有类型而 include `volumetricMesh.h` 的调用点** `[S]` `[low-risk]`
   - 明确分成三类：
     - 只需要枚举/值类型
     - 需要通用算法入口
     - 真的还依赖运行时多态
   - 依赖：步骤 1
   - 为什么：这一步是后续迁移顺序的输入

### 阶段 F2：建立组合式 storage 层

4. **新增 `storage/mesh_storage`** `[M]` `[med-risk]`
   - 持有：
     - `VolumetricMeshData`
     - `MaterialCatalog`
   - 提供：
     - 只读/可写访问
     - 最小 invariant 校验
   - 为什么：让共享状态脱离基类

5. **把 `TetMesh` / `CubicMesh` 先改成“双持有期”结构** `[L]` `[high-risk]`
   - 过渡期允许：
     - 保留旧基类
     - 同时引入 `MeshStorage`
   - 但要求：
     - 新逻辑只读写 `MeshStorage`
     - 旧基类字段只作为过渡镜像，逐步删除
   - 依赖：步骤 4
   - 为什么：一次性把所有成员迁完风险太高，Phase F 需要一个有限的中间态

6. **让 `ops::mesh_construction` 直接面向 `MeshStorage` 注入** `[M]` `[med-risk]`
   - 不再要求通过 `VolumetricMesh` 基类注入 geometry/materials
   - 依赖：步骤 4、5
   - 为什么：文件构造和 `LoadedMeshData` 注入是最天然的 storage 边界

### 阶段 F3：建立 traits / concepts，并把 element-specific 能力从虚函数切走

7. **新增 `traits/mesh_tags` 与 `traits/mesh_traits`** `[M]` `[med-risk]`
   - 目标：
     - `TetTag`
     - `CubicTag`
     - `mesh_traits<TetMesh>`
     - `mesh_traits<CubicMesh>`
   - 为什么：用 traits 明确表达 mesh-specific 的编译期信息

8. **新增 `concepts/mesh_concepts`** `[M]` `[med-risk]`
   - 至少提供：
     - `VolumetricMeshLike`
     - `TetMeshLike`
     - `CubicMeshLike`
   - 依赖：步骤 7
   - 为什么：让通用算法不再依赖“基类 + 注释”约定能力集

9. **把 element-specific 虚接口替换为 ops + traits 调度** `[L]` `[high-risk]`
   - 覆盖：
     - `containsVertex`
     - `computeBarycentricWeights`
     - `getElementVolume`
     - `computeElementMassMatrix`
     - `interpolateGradient`
   - 依赖：步骤 7、8
   - 为什么：这是去掉 `virtual` 的核心步骤

### 阶段 F4：把 mesh-agnostic 模块改成模板算法

10. **模板化 `internal/mesh_queries`** `[L]` `[high-risk]`
    - 从 `const VolumetricMesh&` 改成：
      - `template <VolumetricMeshLike MeshT>`
    - 依赖：步骤 8、9
    - 为什么：这是调用最频繁、收益最高的一组通用查询

11. **模板化 `internal/mesh_mass_properties`** `[L]` `[high-risk]`
    - 覆盖：
      - volume
      - mass
      - inertia
      - bounding box
    - 依赖：步骤 8、9
    - 为什么：它当前几乎是典型的 “通用流程 + element-specific hooks”

12. **模板化 `internal/mesh_transforms` 与 `algorithms/mesh_interpolation`** `[L]` `[high-risk]`
    - 覆盖：
      - gravity
      - deformation
      - linear transform
      - interpolation weights generation
      - containing elements
    - 依赖：步骤 8、9
    - 为什么：这两组现在仍然把 `VolumetricMesh` 当作统一能力对象

13. **模板化 `algorithms/generate_mass_matrix`** `[L]` `[high-risk]`
    - 依赖：步骤 8、9、11
    - 为什么：它是 `VolumetricMesh*` 外部依赖面里最重要的一条

### 阶段 F5：把运行时分发收口到边界层

14. **新增边界层 `AnyMeshRef` / `AnyMutableMeshRef`** `[M]` `[med-risk]`
    - 只服务于运行时未知 mesh 类型入口
    - 不进入 `internal/*`
    - 依赖：步骤 10~13
    - 为什么：要删基类，但边界层仍需要一个运行时分发载体

15. **改造 `generate_surface_mesh` / `mesh_save` 等边界模块使用 `variant` 分发** `[L]` `[med-risk]`
    - 依赖：步骤 14
    - 为什么：这些模块确实需要在运行时根据 mesh 类型做分支，但不该再为此保留一个基类

16. **评估 `barycentricCoordinates` 的迁移形状** `[M]` `[med-risk]`
    - 二选一：
      - 模板化构造入口
      - 用 `AnyMeshRef` 作为运行时输入
    - 依赖：步骤 10、12、14
    - 为什么：这是模块外部仍显式依赖 `VolumetricMesh*` 的关键点

### 阶段 F6：删除基类并迁移下游

17. **让 `TetMesh` / `CubicMesh` 正式移除 `public VolumetricMesh`** `[L]` `[high-risk]`
    - 依赖：步骤 5、9、10~16
    - 为什么：这是整个阶段的核心里程碑

18. **批量迁移下游调用点** `[L]` `[high-risk]`
    - 顺序建议：
      1. `sceneToSimulationMesh`
      2. `interpolationCoordinates`
      3. `io/*`
      4. `algorithms/*`
      5. tools
      6. C API
      7. `runSimCore`
    - 依赖：步骤 17
    - 为什么：这条线决定了基类能否真正被删除

19. **删除 [volumetricMesh.h](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/volumetricMesh.h) / [volumetricMesh.cpp](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/volumetricMesh.cpp)** `[L]` `[high-risk]`
    - 完成条件：
      - 仓库内生产代码不再 include 它
      - 所有原虚接口已有替代
      - 运行时边界已收口到 `variant`
    - 依赖：步骤 18
    - 为什么：这是 Phase F 的收官，不应拖成“只剩一个薄壳但还不敢删”

## 7. 测试计划

Phase F 需要在现有回归基础上新增一组“去基类化护栏”。

### 7.1 需要新增的测试类别

1. **traits / concepts 编译期测试**
   - `mesh_traits` 的 `static_assert`
   - `VolumetricMeshLike<TetMesh>`
   - `VolumetricMeshLike<CubicMesh>`

2. **storage 注入与共享状态测试**
   - `MeshStorage` 默认构造与 move/copy 行为
   - `LoadedMeshData -> MeshStorage` 注入不破坏 invariant

3. **模板化通用算法等价测试**
   - `mesh_queries`
   - `mesh_mass_properties`
   - `mesh_transforms`
   - `generate_mass_matrix`

4. **边界层运行时分发测试**
   - `AnyMeshRef` 到 `generate_surface_mesh`
   - `AnyMeshRef` 到 `mesh_save`

5. **下游回归**
   - `sceneToSimulationMesh`
   - `barycentricCoordinates`
   - C API
   - tools

### 7.2 建议新增的最小测试文件

```text
tests/core/scene/volumetricMesh/traits/
├── mesh_traits_test.cpp
└── mesh_concepts_test.cpp

tests/core/scene/volumetricMesh/storage/
└── mesh_storage_test.cpp

tests/core/scene/volumetricMesh/dispatch/
└── any_mesh_ref_test.cpp
```

### 7.3 持续验收命令

```bash
cmake --build /Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/build/core-debug --target \
  core_scene_volumetric_mesh_test \
  core_energy_scene_to_simulation_mesh_test \
  core_energy_material_binding_test \
  api_pgo_c_test \
  cubicMesher_tool_test

ctest --test-dir /Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/build/core-debug \
  -R "core.scene.volumetric_mesh|core.energy.scene_to_simulation_mesh|core.energy.material_binding|api.pgo_c.basic|cubicMesher.tool.uniform" \
  --output-on-failure
```

Phase F 期间，只有当上面这组测试持续通过时，才允许删除 [volumetricMesh.h](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/volumetricMesh.h)。

## 8. 风险与顺序建议

### 8.1 最高风险点

1. **模板化 mesh-agnostic 算法时，行为语义被意外改变**
   - 特别是：
     - `getContainingElement`
     - `getClosestElement`
     - barycentric / contains 的当前约定

2. **过早删除基类，导致运行时边界没有替代**
   - `generate_surface_mesh`
   - `mesh_save`
   - `barycentricCoordinates`

3. **共享 storage 过渡期的双写/镜像错误**
   - 如果 Phase F2 走“双持有期”，就必须严格限制旧字段只读

### 8.2 建议执行顺序

如果真的开始做 Phase F，我建议按下面顺序推进：

1. F1：抽枚举与共有类型
2. F2：落 `MeshStorage`
3. F3：落 traits / concepts
4. F4：模板化 `mesh_queries` / `mesh_mass_properties` / `mesh_transforms`
5. F4：模板化 `mesh_interpolation` / `generate_mass_matrix`
6. F5：收口边界分发
7. F6：改 `TetMesh` / `CubicMesh` 继承结构
8. F6：迁下游并删基类

不要先尝试直接删除 [volumetricMesh.h](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/volumetricMesh.h)，否则你会被一整批“其实只是需要统一运行时入口”的模块反向卡住。

## 9. 总结

Phase F 的核心不是“把 `VolumetricMesh` 再清理一下”，而是**终止这个基类继续作为错误中心抽象存在**。

正确的落地形态应当是：

- `TetMesh` / `CubicMesh`：组合式公共类型
- `MeshStorage`：共享数据
- `ops/`：mesh-specific element kernel
- `traits/` + `concepts/`：编译期能力边界
- `algorithms/`：mesh-agnostic 模板实现
- `variant` 边界层：少量运行时未知类型入口

做到这一步之后，模块整体才会真正从“历史继承树 + 转发 façade”切换成你要的那种：

**mesh agnostic + mesh specific implementation**
