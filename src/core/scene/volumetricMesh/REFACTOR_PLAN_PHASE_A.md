# volumetricMesh Phase A 执行文档

## 1. 目标

Phase A 的目的不是开始大规模重构，而是先把后续重构的**护栏**立起来。具体只做三件事：

1. 建立足够覆盖 `volumetricMesh` 关键行为的回归测试骨架
2. 把模块当前依赖的关键不变量明确写成文档
3. 把“破坏式切接口时必须一起改的文件”盘清楚，形成切换边界

这一步结束后，才进入真正的结构拆分。

## 2. 产出物

Phase A 完成时，应该有以下产出：

- `tests/core/scene/core_scene_test.cpp` 被扩充，覆盖 `TetMesh` / `CubicMesh` 的更多行为
- `tests/core/energy/` 下增加或扩充 `SceneToSimulationMesh` 相关回归
- 必要时新增一个专门针对 mesh 编辑/细分的测试文件
- 在 `volumetricMesh` 目录下新增 `INVARIANTS.md`
- 在 `volumetricMesh` 目录下新增 `BREAKING_API_BOUNDARY.md`
- 本文档中的“切换文件清单”被验证过，没有漏掉核心调用面

## 3. 执行顺序

建议按下面顺序执行，不要颠倒：

1. 补测试
2. 跑测试并修补测试基础设施
3. 写不变量文档
4. 写切换边界文档
5. 复核 Phase B 之前的“允许改动范围”

---

## 4. 步骤 1：补充模块级回归测试骨架

### 4.1 现有测试基础

仓库里已经有可复用的测试入口，不需要从零搭：

- `tests/core/scene/core_scene_test.cpp`
- `tests/core/scene/CMakeLists.txt`
- `tests/core/energy/simulationMesh_material_binding_test.cpp`
- `tests/core/energy/CMakeLists.txt`
- `tests/api/c/pgo_c_test.cpp`

其中已经存在的覆盖包括：

- Tet/Cubic 的 ASCII/Binary round-trip
- Binary memory constructor
- `VolumetricMesh::clone()` 的深拷贝材料行为
- `SceneToSimulationMesh::fromTetMesh()` 的基础烟测

这意味着 Phase A 不需要重建测试体系，而是**沿着已有 test target 扩容**。

### 4.2 测试补充策略

#### 4.2.1 优先扩 `core_scene_test.cpp`

优先把下面四类行为补到 `tests/core/scene/core_scene_test.cpp`：

1. **barycentric 权重**
   - Tet：内部点权重和为 1，外部点至少有一项越界
   - Cubic：内部点 trilinear 权重和为 1

2. **`subset_in_place`**
   - 子集裁剪后：
     - `numElements` 正确
     - `elements.size()` 正确
     - `elementMaterial.size()` 正确
     - `sets[0]` 仍覆盖全部剩余元素
     - 如启用 `remove_isolated_vertices`，`numVertices` 下降且索引合法

3. **`CubicMesh::subdivide()`**
   - `numElements` 变为原来的 8 倍
   - `cubeSize` 变为原来的一半
   - `sets` 中元素索引被正确扩展
   - `elementMaterial.size()` 与 `numElements` 保持一致

4. **`GenerateSurfaceMesh::computeMesh()`**
   - Tet：单 tetrahedron 生成 4 个三角面
   - Cubic：单 cube 生成 6 个 quad，`triangulate=true` 时生成 12 个三角面

#### 4.2.2 在 `core/energy` 层补一个更直接的 `SceneToSimulationMesh` 回归

虽然 `tests/core/energy/simulationMesh_material_binding_test.cpp` 已经覆盖了 `SceneToSimulationMesh::fromTetMesh()` 的基础路径，但还不够像“重构护栏”。建议新增下面两类断言：

1. **几何保持**
   - 顶点数量、单元数量与输入 TetMesh 一致
   - element type 为 `SimulationMeshType::TET`

2. **材料绑定保持**
   - ENu material 的关键参数映射正确
   - 不支持的材料类型在切换前就明确期望行为

建议实现方式二选一：

- 方案 A：继续扩 `simulationMesh_material_binding_test.cpp`
- 方案 B：新增 `tests/core/energy/scene_to_simulation_mesh_test.cpp`

建议选 **方案 B**，因为它更利于后续单独扩容。

#### 4.2.3 暂不新建专门的 C API Phase A 测试文件

Phase A 只需要确认 C API 仍被现有测试触达即可，不建议现在新建一个庞大的 C API regression 文件。当前只要做到：

- 记录 `tests/api/c/pgo_c_test.cpp` 是 Phase C / Phase D 切接口时必须同步关注的护栏
- 如果后续 API 切换开始影响 `pgo_c.cpp`，再补更细粒度测试

### 4.3 建议新增/修改的文件

#### 必改

- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/tests/core/scene/core_scene_test.cpp`

#### 二选一

- 扩充 `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/tests/core/energy/simulationMesh_material_binding_test.cpp`
- 或新增 `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/tests/core/energy/scene_to_simulation_mesh_test.cpp`

#### 如果新增 energy 测试文件，则同时修改

- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/tests/core/energy/CMakeLists.txt`

### 4.4 推荐测试用例清单

| 用例名 | 放置位置 | 核心断言 |
|---|---|---|
| `TetMeshBarycentricWeightsForInteriorPoint` | `core_scene_test.cpp` | 权重和为 1；每项在 `[0,1]` |
| `TetMeshContainingElementRejectsExteriorPoint` | `core_scene_test.cpp` | 外部点返回“不包含”语义 |
| `CubicMeshBarycentricWeightsForInteriorPoint` | `core_scene_test.cpp` | 8 个权重和为 1 |
| `SubsetInPlacePreservesMeshInvariants` | `core_scene_test.cpp` | 元素数、顶点数、set/material 映射一致 |
| `SubsetInPlaceRemovesIsolatedVertices` | `core_scene_test.cpp` | 顶点重映射正确 |
| `CubicMeshSubdivideExpandsElementsAndSets` | `core_scene_test.cpp` | 元素数量乘 8，集合被同步扩展 |
| `GenerateSurfaceMeshForTetProducesFourFaces` | `core_scene_test.cpp` | 4 个三角面 |
| `GenerateSurfaceMeshForCubeProducesSixQuads` | `core_scene_test.cpp` | 6 个四边面 |
| `GenerateSurfaceMeshForCubeTriangulatedProducesTwelveTriangles` | `core_scene_test.cpp` | 12 个三角面 |
| `SceneToSimulationMeshPreservesTetTopology` | `scene_to_simulation_mesh_test.cpp` 或现有 energy test | 顶点数/单元数/类型正确 |
| `SceneToSimulationMeshPreservesENuMaterialParams` | 同上 | E、nu、density 映射正确 |

### 4.5 测试实现注意事项

1. 不要依赖随机数据
2. 测试输入尽量用单 tet、单 cube、双 element 小 mesh
3. 尽量避免文件系统以外的外部依赖
4. 文件 round-trip 继续复用当前 `uniqueTempPath()` 和 `readBinaryFile()` 风格
5. 对 `subset_in_place` 这类会改内部索引的操作，断言要偏“结构正确性”，不要写死重编号后的每一个索引顺序，除非顺序本身是协议

### 4.6 建议执行命令

在仓库根目录执行：

```bash
cmake -S /Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo -B /Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/build -DPGO_ENABLE_TESTS=ON
cmake --build /Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/build --target core_scene_test core_energy_material_binding_test api_pgo_c_test
ctest --test-dir /Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/build -R "core.scene.geometry|core.energy.material_binding|api.pgo_c.basic" --output-on-failure
```

如果新建 `scene_to_simulation_mesh_test.cpp`，把对应 test target 一并加进 build 和 ctest 过滤。

### 4.7 步骤 1 完成标准

满足以下条件，步骤 1 才算完成：

- `core_scene_test` 覆盖了本阶段要求的 Tet/Cubic 行为
- `SceneToSimulationMesh::fromTetMesh` 有独立可读的回归测试
- 所有新增/修改测试在本地稳定通过
- 没有为了“让测试方便写”去提前重构生产代码接口

---

## 5. 步骤 2：明确 `VolumetricMesh` 的不变量文档

### 5.1 目标文件

新增：

- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/INVARIANTS.md`

不要把这部分只写在 `REFACTOR_PLAN.md` 里。Phase A 的要求是把它变成一个以后改代码时必须先看的工程文档。

### 5.2 文档必须包含的内容

#### 5.2.1 几何数据不变量

- `numElementVertices` 在一个 mesh 实例内固定
- `elements.size() == numElements * numElementVertices`
- 所有 `elements[i]` 都满足 `0 <= elements[i] < numVertices`
- `vertices.size() == numVertices`

#### 5.2.2 材料/区域不变量

- `elementMaterial.size() == numElements`
- `numMaterials == materials.size()`
- `numSets == sets.size()`
- `numRegions == regions.size()`
- `sets[0].getName() == allElements`
- `sets[0]` 覆盖 `[0, numElements)` 的所有元素

#### 5.2.3 同步规则

把下面四条写清楚：

1. `regions` 是“集合到材料”的声明式表示
2. `elementMaterial` 是面向运行时查询的缓存/展开表示
3. 任何修改 `sets` / `regions` / `numElements` 的操作后，都必须重新建立 `elementMaterial`
4. `subset_in_place`、`subdivide`、`addMaterial` 都属于会破坏同步关系的高风险操作

#### 5.2.4 当前已知脆弱点

明确列出以下函数是“后续重构优先关注点”：

- `assignMaterialsToElements`
- `propagateRegionsToElements`
- `addMaterial`
- `editing::subset_in_place`
- `editing::remove_isolated_vertices`
- `CubicMesh::subdivide`

### 5.3 推荐文档结构

建议 `INVARIANTS.md` 用下面结构：

```text
# volumetricMesh Invariants
## 1. Geometry Invariants
## 2. Material / Set / Region Invariants
## 3. Synchronization Rules
## 4. Mutation Hotspots
## 5. What Must Be Rechecked After Refactor
```

### 5.4 步骤 2 完成标准

- `INVARIANTS.md` 已存在
- 文档覆盖 geometry / material / synchronization 三类不变量
- 至少列出 5 个“高风险 mutation hotspot”
- 文档内容能直接指导后续 PR review，不是空泛描述

---

## 6. 步骤 3：定义破坏式切换边界，并列出需要同步修改的仓库内调用点

### 6.1 目标文件

新增：

- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/BREAKING_API_BOUNDARY.md`

这个文档要回答的不是“未来可以怎么设计”，而是：

- 哪些旧接口准备删
- 删掉后仓库里哪些文件必须一起改
- 哪些改动必须在同一轮提交里完成

### 6.2 需要明确列出的“直接替换对象”

文档里至少要点名下面几类：

#### 6.2.1 头文件层

- `volumetricMesh.h`
- `tetMesh.h`
- `cubicMesh.h`
- `volumetricMeshIO.h`
- `volumetricMeshExport.h`
- `generateSurfaceMesh.h`
- `generateMassMatrix.h`

#### 6.2.2 计划删除或改签名的接口类型

- 返回 `-1` 表示失败的查询接口
- 原始 `double*` / `int*` 输出缓冲区接口
- 协议感知构造函数
- 宽而杂的基类成员算法接口

#### 6.2.3 必须同批改动的下游

**API 层**

- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/api/runSimCore.cpp`
- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/api/c/pgo_c.cpp`
- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/api/c/pgo_c.h`
- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/api/python/pypgo/pypgo.cpp`
- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/api/python/pypgo/pypgo.h`

**tools**

- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/tools/cubicMesher/cubicMesher.cpp`
- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/tools/tetMesher/tetMesher.cpp`
- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/tools/tetMesher/mshFileToVegFile.cpp`
- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/tools/tetMesher/generateTetMeshSurfaceMesh.cpp`

**energy bridge**

- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/energy/solidDeformationModel/sceneToSimulationMesh.cpp`
- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/energy/solidDeformationModel/sceneToSimulationMesh.h`
- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/energy/solidDeformationModel/deformationModelFDTest.cpp`

**utils**

- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/utils/animationIO/animationLoader.cpp`
- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/utils/animationIO/animationLoader.h`

**tests**

- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/tests/core/scene/core_scene_test.cpp`
- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/tests/core/energy/simulationMesh_material_binding_test.cpp`
- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/tests/api/c/pgo_c_test.cpp`
- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/tests/tools/cubicMesher_test.cpp`

### 6.3 文档里应该有的“切换批次”

建议把未来破坏式切换按批次写出来：

#### 批次 1：基础查询签名切换

- `raw pointer -> std::span`
- `-1 -> std::optional`

#### 批次 2：构造路径切换

- 删除特殊协议构造函数
- 文件加载统一走 `io::*`

#### 批次 3：头文件/命名切换

- camelCase 文件名迁到 snake_case 新入口

### 6.4 推荐文档结构

建议 `BREAKING_API_BOUNDARY.md` 用下面结构：

```text
# volumetricMesh Breaking API Boundary
## 1. Scope of Breaking Changes
## 2. Public Headers to Replace
## 3. Signatures to Delete or Rewrite
## 4. Repository Call Sites That Must Move Together
## 5. Cutover Batches
## 6. Phase B Entry Conditions
```

### 6.5 步骤 3 完成标准

- `BREAKING_API_BOUNDARY.md` 已存在
- 文档明确列出至少一轮“必须同批切换”的文件清单
- `runSimCore`、C API、tools、`sceneToSimulationMesh` 都已被列入
- 切换批次按“查询签名 / 构造路径 / 头文件命名”至少分成 3 组

---

## 7. Phase A 退出条件

满足以下条件后，才能进入 Phase B：

- 测试护栏已补齐并通过
- `INVARIANTS.md` 已写完
- `BREAKING_API_BOUNDARY.md` 已写完
- 你已经能回答：
  - 哪些函数一改就会影响 `sceneToSimulationMesh`
  - 哪些行为目前已经被测试覆盖
  - 哪些文件需要在第一次破坏式切换时一起提交

如果这三件事里任意一件没做完，不要进入结构拆分阶段。

## 8. 推荐提交拆分

虽然你允许破坏式 API 变更，但 Phase A 仍建议拆成 2 到 3 个提交：

1. `test(volumetricMesh): expand regression coverage`
2. `docs(volumetricMesh): document invariants`
3. `docs(volumetricMesh): define breaking api boundary`

这样做的好处是，后续真正动结构时，回溯会清楚很多。

## 9. 本阶段不做的事

Phase A 明确不做下面这些：

- 不拆 `volumetricMesh.cpp`
- 不拆 `volumetricMeshIO.cpp`
- 不改 public API 签名
- 不改文件命名
- 不引入 `std::variant`
- 不调整 `sceneToSimulationMesh` 实现

这些都属于 Phase B 及之后的工作。
