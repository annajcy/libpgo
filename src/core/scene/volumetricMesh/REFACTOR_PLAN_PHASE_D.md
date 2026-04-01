# volumetricMesh Phase D 执行文档

## 1. 结论与目标

基于当前代码状态，Phase C 可以视为已经完成，可以正式进入 Phase D。

判断依据：

1. [volumetricMeshIO.h](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/volumetricMeshIO.h) 已收缩为 public façade，不再暴露 `detail::LoadedMeshData`
2. I/O 已拆入 `io/` 子目录，包含：
   - [mesh_io_types.h](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/io/mesh_io_types.h)
   - [mesh_format_detector.cpp](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/io/mesh_format_detector.cpp)
   - [mesh_ascii_reader.cpp](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/io/mesh_ascii_reader.cpp)
   - [mesh_binary_reader.cpp](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/io/mesh_binary_reader.cpp)
   - [mesh_ascii_writer.cpp](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/io/mesh_ascii_writer.cpp)
   - [mesh_binary_writer.cpp](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/io/mesh_binary_writer.cpp)
   - [material_serde.cpp](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/io/material_serde.cpp)
   - [veg_parser.cpp](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/io/detail/veg_parser.cpp)
   - [tetgen_reader.cpp](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/io/detail/tetgen_reader.cpp)
3. `core.scene.volumetric_mesh`、`core.energy.scene_to_simulation_mesh`、`core.energy.material_binding`、`api.pgo_c.basic`、`cubicMesher.tool.uniform` 目前都通过
4. [volumetricMeshIO.cpp](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/volumetricMeshIO.cpp) 已收缩到 `150` 行，满足 Phase C 目标

Phase D 的目标不是再动 I/O，而是把 `TetMesh` / `CubicMesh` 里的元素几何核和高层算法继续拆开，让这两个类从“实现容器”进一步收敛为“窄 façade + element-specific dispatch”。

这一阶段只处理四件事：

1. 为 tet / cubic 提炼独立的 element ops 层
2. 把 `CubicMesh` 里的高层算法迁出类本体
3. 统一 Tet/Cubic 的 `assignFromData` / 构造注入路径
4. 为 `GenerateSurfaceMesh` 引入统一的元素面枚举抽象

## 2. 当前基线

### 2.1 仍然偏重的源文件

当前主要热点已经从 I/O 转移到具体 mesh 类型本体：

- [cubicMesh.cpp](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/cubicMesh.cpp): `763` 行
- [tetMesh.cpp](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/tetMesh.cpp): `353` 行
- [generateSurfaceMesh.cpp](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/generateSurfaceMesh.cpp): `186` 行

### 2.2 当前还留在类里的职责

#### `TetMesh`

仍直接承载：

- tet 体积 / determinant
- barycentric 计算
- gradient 计算
- edge 枚举
- closest-element 距离逻辑
- `assignFromData`

#### `CubicMesh`

仍直接承载：

- `computeAlphaBetaGamma`
- barycentric 计算
- gradient 计算
- inertia / mass matrix / edge 枚举
- `interpolateData`
- `normalCorrection`
- `subdivide`
- `assignFromData`

#### `GenerateSurfaceMesh`

仍直接自己维护：

- tet face 枚举与朝向规则
- cubic face 枚举与四边面拆三角规则
- 通过 `getNumElementVertices()` + `getElementType()` 做手写分支

### 2.3 Phase D 之后要解决的问题

如果不做 Phase D，后面 Phase E 的 API 现代化会有两个问题：

1. 你会在大而杂的 `TetMesh` / `CubicMesh` 类里直接改签名，改动面会非常散
2. `GenerateSurfaceMesh`、`volumetricMeshInterpolation`、`sceneToSimulationMesh` 这类下游没有稳定的几何核接口可依赖

所以 Phase D 的本质是：**先把 element-specific logic 抽成内部能力层，再谈 public API 切换。**

## 3. 范围与非目标

### 3.1 本阶段范围

- 新增 tet / cubic 的 `ops/` 内部几何核
- 把 `CubicMesh` 中的高层算法外移到 `algorithms/`
- 为 `TetMesh` / `CubicMesh` 建立共享的数据注入 helper
- 给 `GenerateSurfaceMesh` 提供统一的 face enumeration 抽象
- 补对应测试与构建源列表

### 3.2 本阶段不做

- 不在 Phase D 内开始 `raw pointer -> std::span` 的 public API 改签名
- 不在 Phase D 内把 `getContainingElement()` 改成 `std::optional`
- 不在 Phase D 内改 `io::save()` 的错误表达
- 不在 Phase D 内切 `sceneToSimulationMesh` 到新 façade
- 不在 Phase D 内收掉现有 `downcast*Material` 调用

原因：

- 这些都是下一阶段的横切 API 变更
- Phase D 的价值是先把实现层切干净，让下一阶段不需要在 `tetMesh.cpp` / `cubicMesh.cpp` 里到处找逻辑

## 4. 先固定的设计决策

### 4.1 引入 `ops/` 与 `algorithms/` 子目录

建议在模块下新增：

```text
src/core/scene/volumetricMesh/
├── ops/
│   ├── tet_mesh_ops.h/.cpp
│   ├── cubic_mesh_ops.h/.cpp
│   ├── element_face_ops.h/.cpp
│   └── mesh_construction.h/.cpp
├── algorithms/
│   ├── cubic_mesh_interpolation.h/.cpp
│   ├── cubic_mesh_normal_correction.h/.cpp
│   └── cubic_mesh_refinement.h/.cpp
├── tetMesh.cpp
├── cubicMesh.cpp
└── generateSurfaceMesh.cpp
```

原则：

- `ops/` 放元素几何核、枚举、轻量级数据注入 helper
- `algorithms/` 放高层流程和较大实现
- 不把这些继续塞进 `internal/`，否则目录会混成“所有东西都叫 internal”

### 4.2 `TetMesh` / `CubicMesh` 先保留 public 方法名

Phase D 只做实现迁出，不做 public 名称切换：

- `TetMesh::computeBarycentricWeights(...)` 仍保留
- `CubicMesh::subdivide()` 仍保留
- `CubicMesh::interpolateData(...)` 仍保留

但这些方法应改为转发到 `ops::*` 或 `algorithms::*`。

### 4.3 `GenerateSurfaceMesh` 继续保留现有 public 入口

Phase D 的目标不是重写 surface API，而是把“每种 element 的面如何枚举、如何定向”抽出来。

也就是：

- public 还是 `GenerateSurfaceMesh::computeMesh(...)`
- 内部改成依赖 `element_face_ops`

### 4.4 `assignFromData` 统一到共享 helper

当前 [tetMesh.cpp](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/tetMesh.cpp) 和 [cubicMesh.cpp](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/cubicMesh.cpp) 仍然各自做：

- element type 检查
- geometry 注入
- material catalog 注入
- cubeSize 等派生字段初始化

建议 Phase D 统一成：

- `ops::mesh_construction::assign_common_loaded_data(...)`
- Tet/Cubic 只保留自己的 element-type 特定检查和派生字段更新

## 5. 阶段划分

### 阶段 D1：先抽 tet / cubic element ops

1. **新增 `ops/tet_mesh_ops`** `[M]` `[med-risk]`
   - 承接：
     - determinant / signed volume / volume
     - barycentric
     - gradient
     - edge 枚举
     - closest-element distance helper
   - 依赖：无
   - 为什么：tet 的核心几何核是稳定的，最适合先抽出来

2. **新增 `ops/cubic_mesh_ops`** `[M]` `[med-risk]`
   - 承接：
     - `computeAlphaBetaGamma`
     - barycentric
     - gradient
     - inertia / mass matrix helper
     - edge 枚举
   - 依赖：无
   - 为什么：这是 `CubicMesh` 里最混杂但又最稳定的一层

3. **让 `TetMesh` / `CubicMesh` 的 element-level 方法改成 façade 转发** `[M]` `[low-risk]`
   - 目标：
     - 类保留 public 虚函数实现
     - 具体实现下沉到 `ops::*`
   - 依赖：步骤 1、2
   - 为什么：为 Phase E 的 API 切换准备稳定的底层实现位置

### 阶段 D2：把 `CubicMesh` 的高层算法迁出

4. **抽出 `algorithms/cubic_mesh_interpolation`** `[L]` `[med-risk]`
   - 搬走：
     - `CubicMesh::interpolateData`
   - 新入口建议：
     - `algorithms::interpolate_cubic_mesh_data(...)`
   - 依赖：步骤 2
   - 为什么：这不是 element kernel，而是高层批处理算法

5. **抽出 `algorithms/cubic_mesh_normal_correction`** `[L]` `[med-risk]`
   - 搬走：
     - `CubicMesh::normalCorrection`
   - 依赖：步骤 2
   - 为什么：这个函数长、状态多、矩阵写入密集，不该继续停留在 mesh 本体里

6. **抽出 `algorithms/cubic_mesh_refinement`** `[M]` `[med-risk]`
   - 搬走：
     - `CubicMesh::subdivide`
   - 依赖：步骤 2
   - 为什么：细分本质上是 refinement algorithm，不是 mesh identity 的一部分

7. **让 `CubicMesh` 保留薄 façade 方法** `[M]` `[low-risk]`
   - `subdivide()` / `interpolateData()` / `normalCorrection()` 暂时继续存在
   - 但改为转发到 `algorithms::*`
   - 依赖：步骤 4、5、6
   - 为什么：不在 Phase D 同时改 public API 形状

### 阶段 D3：统一构造与 data 注入路径

8. **新增 `ops/mesh_construction`，统一 `assignFromData` 公共部分** `[M]` `[low-risk]`
   - 承接：
     - `LoadedMeshData` 的 geometry / material_catalog 注入
     - invariant 校验
   - 依赖：无
   - 为什么：Tet/Cubic 现在仍有平行重复的 data 注入代码

9. **让 `TetMesh::assignFromData` / `CubicMesh::assignFromData` 只保留 type-specific 部分** `[M]` `[low-risk]`
   - Tet：只保留 tet-specific 元素类型检查
   - Cubic：只保留 cubic-specific 检查 + `cubeSize` / `invCubeSize` 更新
   - 依赖：步骤 8
   - 为什么：减少后续维护成本，避免两个入口逐渐偏离

10. **顺手评估文件构造路径是否还需要保留“二段注入”** `[S]` `[low-risk]`
    - 这一步不要求删 API
    - 只要求明确：
      - 哪些构造函数以后可以直接依赖 shared helper
      - 哪些还需要保留类内字段初始化
    - 依赖：步骤 9
    - 为什么：为下一阶段的 API 现代化扫清隐藏分支

### 阶段 D4：抽出统一的面枚举抽象

11. **新增 `ops/element_face_ops`** `[M]` `[med-risk]`
    - 提供：
      - tet face list + orientation rule
      - cubic face list + orientation rule
      - triangulation helper for quad faces
    - 依赖：步骤 1、2
    - 为什么：`GenerateSurfaceMesh` 当前把 element face knowledge 写死在一个函数里

12. **让 `GenerateSurfaceMesh::computeMesh()` 依赖 face ops，而不是手写 tet/cubic 分支细节** `[M]` `[med-risk]`
    - 依赖：步骤 11
    - 为什么：把“面枚举知识”与“表面去重流程”拆开，后者才是 `GenerateSurfaceMesh` 的主职责

13. **把 face 枚举抽象复用到后续 surface / export / visualization 场景** `[S]` `[low-risk]`
    - 这一步先做最小接入评估，不要求本阶段全部改完
    - 依赖：步骤 11、12
    - 为什么：避免 `generateSurfaceMesh` 刚抽出来，其他文件又继续手写一份元素面逻辑

## 6. 测试计划

Phase D 建议补的测试仍按源码目录拆分，不回到单一大文件。

建议新增：

```text
tests/core/scene/volumetricMesh/
├── ops/
│   ├── tet_mesh_ops_test.cpp
│   ├── cubic_mesh_ops_test.cpp
│   ├── element_face_ops_test.cpp
│   └── mesh_construction_test.cpp
├── algorithms/
│   ├── cubic_mesh_interpolation_test.cpp
│   ├── cubic_mesh_normal_correction_test.cpp
│   └── cubic_mesh_refinement_test.cpp
├── tet_mesh_test.cpp
├── cubic_mesh_test.cpp
└── generate_surface_mesh_test.cpp
```

### 6.1 Tet ops 护栏

- determinant / volume 与现有 `TetMesh` 语义一致
- barycentric 权重和为 1
- `containsVertex` 与 barycentric 判定一致
- gradient 对单 tet 常量场 / 线性场行为不漂移
- edge 枚举数量和拓扑不变

### 6.2 Cubic ops 护栏

- `computeAlphaBetaGamma` 在轴对齐 cube 上返回稳定结果
- barycentric 权重和为 1
- edge 枚举仍是 12 条边
- mass matrix / inertia tensor 数值与旧实现一致

### 6.3 Cubic 高层算法护栏

- `interpolateData` 在内部点、外部点、零阈值路径上与旧行为一致
- `normalCorrection` 的零填充与非零填充路径不变
- `subdivide()` 后元素数、`cubeSize`、materials、sets 与当前测试一致

### 6.4 Surface 护栏

- 单 tet 仍生成 4 个三角面
- 单 cube 仍生成 6 个 quad / 12 个 triangle
- `allElementFaces=true` 与当前行为保持一致
- tet orientation 正负分支都覆盖

## 7. 构建与验证

每个子阶段至少跑下面这组：

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

如果新增 `ops/` 和 `algorithms/` 测试文件，建议仍并入 `core_scene_volumetric_mesh_test`，避免 target 继续碎裂。

## 8. 建议提交拆分

不要把 Phase D 混成一个提交。建议至少拆成下面 6 个提交：

1. `D1-tet-ops`: 抽 `tet_mesh_ops`
2. `D1-cubic-ops`: 抽 `cubic_mesh_ops`
3. `D2-cubic-algorithms`: 抽 cubic interpolation / normal correction / refinement
4. `D3-mesh-construction`: 统一 `assignFromData`
5. `D4-face-ops`: 抽 `element_face_ops` 并接入 `GenerateSurfaceMesh`
6. `D-tests`: 补 ops / algorithms / surface 护栏与 CMake 列表

如果 `CubicMesh::subdivide()` 的迁出影响较大，可以单独再拆一个提交，不要和 interpolation / normal correction 混在一起。

## 9. Phase D 退出条件

Phase D 完成时，至少要满足下面这些条件：

1. `TetMesh` 的核心元素几何实现已下沉到 `ops/tet_mesh_ops`
2. `CubicMesh` 的核心元素几何实现已下沉到 `ops/cubic_mesh_ops`
3. `CubicMesh::interpolateData` / `normalCorrection` / `subdivide` 已迁到 `algorithms/`
4. `TetMesh::assignFromData` / `CubicMesh::assignFromData` 的公共注入逻辑已统一
5. `GenerateSurfaceMesh` 已依赖统一的 face enumeration 抽象
6. 最小验证集继续通过

建议再加两个量化标准：

- [cubicMesh.cpp](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/cubicMesh.cpp) 压到 `400` 行以内
- [tetMesh.cpp](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/tetMesh.cpp) 压到 `250` 行以内

如果 Phase D 结束后，`CubicMesh` 仍同时持有：

- element-level math
- interpolation pipeline
- normal correction pipeline
- refinement pipeline

那就说明这轮还没有真正完成类清理。

## 10. 下一阶段入口

Phase D 完成后，再进入 Phase E：

- `raw pointer -> std::span`
- `getContainingElement() -> find_containing_element()`
- `io::save()` 错误表达升级
- `material_cast<T>()` / 统一材料访问入口

也就是说，Phase D 的价值是把“实现重构”做完，让 Phase E 能专注于 public API 的破坏式切换，而不是一边重写类内部结构、一边改下游调用点。
