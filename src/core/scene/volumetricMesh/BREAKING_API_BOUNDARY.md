# volumetricMesh Breaking API Boundary

## 1. Scope of Breaking Changes

本文件定义的是 Phase B 及之后允许触发的“破坏式切换边界”，不是现在就去改这些接口。

目标是明确三件事：

- 哪些 public 入口准备收缩、改签名或迁移
- 哪些仓库内调用点必须同一批一起改
- 哪些切换必须分批进行，不能混在一次模糊提交里

Phase A 的要求只是把边界说清楚，并确认仓库内核心调用面没有漏掉。

## 2. Public Headers to Replace

第一批需要重点盯防的头文件是：

- `volumetricMesh.h`
- `tetMesh.h`
- `cubicMesh.h`
- `volumetricMeshIO.h`
- `volumetricMeshExport.h`
- `generateSurfaceMesh.h`
- `generateMassMatrix.h`

原因不是这些文件都会在同一轮被重写，而是它们共同承载了当前最宽的 public surface：
构造、查询、导出、I/O、质量矩阵、表面网格生成都在这里暴露。

## 3. Signatures to Delete or Rewrite

后续破坏式切换优先关注下面四类接口形态：

### 3.1 返回哨兵值的查询接口

典型代表：

- `VolumetricMesh::getContainingElement(Vec3d)` 以 `-1` 表示失败

这类接口后续如果改成 `std::optional` 或显式结果对象，会影响所有依赖“负值即失败”分支的调用方。

### 3.2 原始输出缓冲区接口

典型代表：

- `computeBarycentricWeights(..., double* weights)`
- `computeElementMassMatrix(..., double* massMatrix)`
- `getElementEdges(..., int* edgeBuffer)`
- `computeGravity(double* gravityForce, ...)`
- `interpolateGradient(..., double* grad)`

这类接口后续如果统一迁到 `std::span`，会形成一次横切式签名变更。

### 3.3 协议感知构造路径

典型代表：

- `TetMesh(const std::filesystem::path&, FileFormatType, int)`
- `TetMesh(std::span<const std::byte>)`
- `TetMesh(const std::filesystem::path&, int specialFileType, int verbose)`
- `CubicMesh(const std::filesystem::path&, FileFormatType, int)`
- `CubicMesh(std::span<const std::byte>)`

后续如果统一为 `io::*` 工厂 / loader 路径，调用方会从“直接构造对象”迁到“显式加载 API”。

### 3.4 宽基类算法接口

`VolumetricMesh` 目前同时暴露几何访问、材料访问、查询、插值、导出相关职责。
后续一旦拆分职责，所有直接依赖基类宽接口的下游都要同步迁移。

## 4. Repository Call Sites That Must Move Together

下面清单已经按仓库现状做过一次 `rg` 核对。它不是“所有可能受影响文件”的全集，
但覆盖了第一次破坏式切换不能漏掉的核心调用面。

### 4.1 API 层

- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/api/runSimCore.cpp`
  直接依赖 `TetMesh`、`GenerateMassMatrix`、`SceneToSimulationMesh::fromTetMesh`
- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/api/c/pgo_c.cpp`
  直接依赖 `TetMesh`、`volumetricMeshExport`、`volumetricMeshIO`
- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/api/c/pgo_c.h`
  暴露 C API surface，若底层对象生命周期或加载路径变化，这里必须同步改
- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/api/python/pypgo/pypgo.cpp`
- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/api/python/pypgo/pypgo.h`
  这里不直接 include `tetMesh.h`，但它们依赖 `pgo_c.h` 暴露出来的 tet mesh C API，因此属于同批迁移边界

### 4.2 tools

- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/tools/cubicMesher/cubicMesher.cpp`
- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/tools/tetMesher/tetMesher.cpp`
- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/tools/tetMesher/mshFileToVegFile.cpp`
- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/tools/tetMesher/generateTetMeshSurfaceMesh.cpp`

这些工具直接依赖 `TetMesh` / `CubicMesh` / `volumetricMeshIO` / `GenerateSurfaceMesh`，
是第一次头文件或加载路径切换时必须一起过一遍的入口。

### 4.3 energy bridge

- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/energy/solidDeformationModel/sceneToSimulationMesh.cpp`
- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/energy/solidDeformationModel/sceneToSimulationMesh.h`
- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/energy/solidDeformationModel/deformationModelFDTest.cpp`

这是 `volumetricMesh -> simulation mesh` 的直接桥接层。任何 `TetMesh` 构造、material 访问、
element 查询语义变化，都会首先在这里暴露。

### 4.4 utils

- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/utils/animationIO/animationLoader.cpp`
- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/utils/animationIO/animationLoader.h`

这里直接持有 `std::shared_ptr<VolumetricMeshes::TetMesh>`，并调用 `GenerateSurfaceMesh::computeMesh`，
属于典型的“不是 API 层，但会被接口切换波及”的调用面。

### 4.5 其他直接依赖点

- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/energy/solidDeformationModel/tetVolumeConstraintFunctions.cpp`
  直接 include `tetMesh.h`

这个文件不一定要和第一轮全部接口收缩一起改，但如果 `TetMesh` 头文件入口或基础查询语义变动，不能漏查它。

### 4.6 tests

- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/tests/core/scene/geometry_test.cpp`
- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/tests/core/scene/volumetricMesh/tet_mesh_test.cpp`
- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/tests/core/scene/volumetricMesh/cubic_mesh_test.cpp`
- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/tests/core/scene/volumetricMesh/volumetric_mesh_edit_test.cpp`
- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/tests/core/scene/volumetricMesh/generate_surface_mesh_test.cpp`
- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/tests/core/scene/volumetricMesh/volumetric_mesh_clone_test.cpp`
- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/tests/core/energy/simulationMesh_material_binding_test.cpp`
- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/tests/core/energy/scene_to_simulation_mesh_test.cpp`
- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/tests/api/c/pgo_c_test.cpp`
- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/tests/tools/cubicMesher_test.cpp`

注意：Phase A 早期计划里原本写的是 `tests/core/scene/core_scene_test.cpp`，但当前仓库已经拆成模块化测试布局。
后续切接口时，应以这里列出的真实测试文件为准，而不是继续追旧路径。

## 5. Cutover Batches

### 批次 1：基础查询签名切换

范围：

- `raw pointer -> std::span`
- `-1 -> std::optional` 或显式结果对象

这一批必须同看：

- `volumetricMesh.h`
- `tetMesh.h`
- `cubicMesh.h`
- `sceneToSimulationMesh.cpp`
- `runSimCore.cpp`
- `animationLoader.cpp`
- 直接依赖相关查询的测试

### 批次 2：构造路径切换

范围：

- 删除特殊协议构造函数
- 文件 / binary 加载统一走 `io::*`

这一批必须同看：

- `tetMesh.h` / `cubicMesh.h`
- `volumetricMeshIO.h`
- `runSimCore.cpp`
- `pgo_c.cpp`
- `pypgo.cpp`
- `tetMesher.cpp`
- `mshFileToVegFile.cpp`
- `deformationModelFDTest.cpp`

### 批次 3：头文件与命名入口切换

范围：

- 旧 camelCase 文件迁移到新的稳定入口
- 可能引入更窄的 facade header

这一批必须同看：

- 所有直接 include `tetMesh.h` / `cubicMesh.h` / `generateSurfaceMesh.h` / `generateMassMatrix.h` 的仓库内文件
- CMake target 是否仍链接到正确库
- tests 是否覆盖了新入口

## 6. Phase B Entry Conditions

在进入真正的结构拆分前，至少要满足以下条件：

- Phase A 的测试护栏已通过
- `INVARIANTS.md` 已存在并可指导 review
- 本文档列出的 API / tools / energy bridge / utils / tests 调用面已人工确认
- 团队已经明确第一次破坏式切换属于哪一个批次，而不是多批混改

如果连“这次改的是查询签名、构造路径，还是头文件入口”都说不清，就还不应该开始 Phase B。
