# volumetricMesh Phase B 执行文档

## 1. 目标

Phase B 的目标不是继续补护栏，而是开始真正拆 `VolumetricMesh` 的内部结构，并把后续 Phase C 之后的 I/O、API 切换建立在更干净的内部模型上。

这一阶段只解决四件事：

1. 把几何存储从 `VolumetricMesh` 本体中抽走
2. 把材料 / set / region / `elementMaterial` 的同步职责抽走
3. 去掉 `editing` 对内部字段的 `friend` 式穿透
4. 把非虚的通用算法从基类成员实现迁到明确的内部算法层

## 2. 进入条件

进入 Phase B 前，默认已经满足以下条件：

- [INVARIANTS.md](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/INVARIANTS.md) 已完成
- [BREAKING_API_BOUNDARY.md](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/BREAKING_API_BOUNDARY.md) 已完成
- `tests/core/scene/volumetricMesh/` 下的回归测试已经按模块拆开
- `tests/core/energy/scene_to_simulation_mesh_test.cpp` 已存在，可作为 bridge 护栏

建议在每个子阶段开始前，先跑一次最小护栏：

```bash
cmake --build /Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/build --target \
  core_scene_volumetric_mesh_test \
  core_energy_scene_to_simulation_mesh_test \
  core_energy_material_binding_test \
  api_pgo_c_test \
  cubicMesher_tool_test

ctest --test-dir /Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/build \
  -R "core.scene.volumetric_mesh|core.energy.scene_to_simulation_mesh|core.energy.material_binding|api.pgo_c.basic|cubicMesher.tool.uniform" \
  --output-on-failure
```

## 3. 范围与非目标

### 3.1 本阶段范围

- `VolumetricMesh`、`TetMesh`、`CubicMesh` 的内部数据布局改为组合而不是直接摊平在基类里
- `assignMaterialsToElements`、`propagateRegionsToElements`、`addMaterial` 等逻辑迁移到独立材料目录对象
- `subset_in_place`、`remove_isolated_vertices`、`subdivide` 改为通过受控 mutation 接口工作
- 质量、重力、邻接、几何查询、形变等非虚逻辑迁到内部算法文件
- 构造路径开始从 “文件 / LoadedMeshData 直接灌字段” 改成 “先构建内部对象，再交给 facade”

### 3.2 本阶段不做

- 不在 Phase B 内拆 `volumetricMeshIO.cpp` 的 reader / writer / detector 翻译单元
- 不在 Phase B 内做最终的 public header 重命名和目录重排
- 不在 Phase B 内做 `raw pointer -> std::span`、`-1 -> std::optional` 这类横切式 API 清理
- 不在 Phase B 内把材料系统直接改成 `std::variant` 或值类型

说明：

- 你允许破坏式切换，但这不等于把所有类型、目录、签名改动混在同一轮里
- Phase B 的重点是先把内部结构切干净，让后面的破坏式 API 替换不是“在泥地上换轮子”

## 4. 先固定的设计决策

在开始写代码前，Phase B 先固定下面四个决策，避免边做边漂移：

### 4.1 `VolumetricMesh` 仍保留为 public facade

- `VolumetricMesh`、`TetMesh`、`CubicMesh` 暂时继续作为对外入口
- 但它们不再直接拥有几何数组和材料数组
- 这一步的目标是 “public 名字不一定立刻变，内部所有权先切开”

### 4.2 先保留现有 `Material / Set / Region` 类型定义

- Phase B 不急着把 `VolumetricMesh::Material`、`Set`、`Region` 立即挪成全新公共类型
- 第一刀先抽“所有权和同步逻辑”，不是先抽“命名空间和类型名字”
- 否则会把结构改造和 API 命名迁移耦合在一起

### 4.3 新增文件优先放到模块内部子目录

建议在 [CMakeLists.txt](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/CMakeLists.txt) 中新增内部文件，并采用下面布局：

```text
src/core/scene/volumetricMesh/
├── internal/
│   ├── volumetric_mesh_data.h/.cpp
│   ├── material_catalog.h/.cpp
│   ├── region_assignment.h/.cpp
│   ├── mesh_mutation.h/.cpp
│   ├── mesh_subset_editor.h/.cpp
│   ├── mesh_queries.h/.cpp
│   ├── mesh_mass_properties.h/.cpp
│   └── mesh_transforms.h/.cpp
├── volumetricMesh.h/.cpp
├── tetMesh.h/.cpp
├── cubicMesh.h/.cpp
├── volumetricMeshEdit.h/.cpp
└── ...
```

原因：

- 这能把新结构和旧文件并存起来，降低单次提交的噪声
- 也能让测试继续按 `src` 目录结构拆，而不是重新塞回一个大文件

### 4.4 测试继续按源码结构拆

这一点直接沿用你前面的偏好，不再回到单文件测试模式。

建议测试布局同步扩成：

```text
tests/core/scene/volumetricMesh/
├── tet_mesh_test.cpp
├── cubic_mesh_test.cpp
├── volumetric_mesh_edit_test.cpp
├── generate_surface_mesh_test.cpp
├── volumetric_mesh_clone_test.cpp
├── material_catalog_test.cpp
├── mesh_queries_test.cpp
└── internal/
    ├── volumetric_mesh_data_test.cpp
    └── mesh_subset_editor_test.cpp
```

原则：

- 对外行为回归继续放在现有 facade 级测试里
- 新内部对象如果需要直接测，单独放 `internal/`
- 不把 Phase B 的结构测试堆回某个“大而全”的 scene test

## 5. 阶段划分

Phase B 建议拆成四个子阶段执行，每个子阶段都要保持主干可编译、核心护栏可运行。

### 阶段 B1：抽出 `VolumetricMeshData`

#### 5.1.1 目标

把下面这些字段从 `VolumetricMesh` 直接拥有改成由几何数据对象持有：

- `numVertices`
- `vertices`
- `numElementVertices`
- `numElements`
- `elements`

#### 5.1.2 建议新增文件

- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/internal/volumetric_mesh_data.h`
- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/internal/volumetric_mesh_data.cpp`

#### 5.1.3 `VolumetricMeshData` 建议职责

建议让它只负责几何真相源，不碰材料和 I/O 协议：

- 持有 `vertices` 和扁平 `elements`
- 记录 `num_element_vertices`
- 从存储自动推导 `num_vertices()`、`num_elements()`
- 提供 `vertex(i)`、`vertex_indices(element)`、`set_vertex(i, ...)`
- 提供 `renumber_vertices(...)`
- 提供几何级 invariant 校验

建议接口形态尽量直接：

```cpp
class VolumetricMeshData {
public:
    VolumetricMeshData(int num_element_vertices,
                       std::vector<Vec3d> vertices,
                       std::vector<int> elements);

    int num_vertices() const;
    int num_elements() const;
    int num_element_vertices() const;

    std::span<Vec3d> vertices();
    std::span<const Vec3d> vertices() const;
    std::span<const int> vertex_indices(int element) const;

    void renumber_vertices(std::span<const int> permutation);
    void validate_basic_invariants() const;
};
```

#### 5.1.4 需要改动的现有文件

- [volumetricMesh.h](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/volumetricMesh.h)
- [volumetricMesh.cpp](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/volumetricMesh.cpp)
- [tetMesh.h](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/tetMesh.h)
- [tetMesh.cpp](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/tetMesh.cpp)
- [cubicMesh.h](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/cubicMesh.h)
- [cubicMesh.cpp](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/cubicMesh.cpp)
- [volumetricMeshIO.h](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/volumetricMeshIO.h)

#### 5.1.5 实施顺序

1. 新增 `VolumetricMeshData`
2. 在 `VolumetricMesh` 中引入 `std::unique_ptr<VolumetricMeshData>` 或直接内嵌值成员
3. 把所有只读 getter 改成转发到 data object
4. 调整 `TetMesh::assignFromData`、`CubicMesh::assignFromData`
5. 调整 clone / copy constructor，确认深拷贝语义不变
6. 跑 tet / cubic round-trip 与 clone 测试

#### 5.1.6 风险点

- `numVertices`、`numElements` 目前是独立缓存，改成推导值后容易漏掉旧代码中的写路径
- `getVertex(int element, int vertex)`、`getVertexIndices(int element)` 这类热点访问要避免语义漂移
- `CubicMesh::subdivide()` 和 `editing::subset_in_place()` 暂时仍会改几何，B1 结束时可以先允许它们通过 facade 间接写 data object

#### 5.1.7 完成标准

- `VolumetricMesh` 不再直接拥有几何原始数组
- tet / cubic 构造、拷贝、binary memory constructor 仍通过现有测试
- `SceneToSimulationMesh::fromTetMesh()` 不需要改业务语义就能继续工作

### 阶段 B2：抽出 `MaterialCatalog` 与 `RegionAssignment`

#### 5.2.1 目标

把下面这些职责从 `VolumetricMesh` 本体中搬走：

- `materials`
- `sets`
- `regions`
- `elementMaterial`
- `assignMaterialsToElements`
- `propagateRegionsToElements`
- `addMaterial`
- `setSingleMaterial`

#### 5.2.2 建议新增文件

- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/internal/material_catalog.h`
- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/internal/material_catalog.cpp`
- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/internal/region_assignment.h`
- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/internal/region_assignment.cpp`

#### 5.2.3 设计建议

`MaterialCatalog` 负责所有权与对外查询：

- 拥有 `materials`
- 拥有 `sets`
- 拥有 `regions`
- 拥有 `element_materials`
- 提供 `material(i)`、`element_material(el)`、`set(i)`、`region(i)`
- 提供 `set_single_material(...)`
- 提供 `add_material(...)`
- 提供 `subset_elements(...)`
- 提供 `validate_against_num_elements(num_elements)`

`RegionAssignment` 负责“如何从 `sets + regions` 展开出 `element_materials`”：

- 接受 `num_elements`
- 接受只读 `sets` / `regions`
- 生成 `std::vector<int> element_materials`
- 统一处理默认材料、未覆盖元素、非法索引等校验

这里的关键不是类名，而是先把“展开规则”从 `VolumetricMesh` 里拿出来，形成单独审查点。

#### 5.2.4 需要改动的现有文件

- [volumetricMesh.h](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/volumetricMesh.h)
- [volumetricMesh.cpp](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/volumetricMesh.cpp)
- [tetMesh.cpp](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/tetMesh.cpp)
- [cubicMesh.cpp](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/cubicMesh.cpp)
- [volumetricMeshIO.cpp](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/volumetricMeshIO.cpp)

#### 5.2.5 实施顺序

1. 先让 `MaterialCatalog` 复用现有 `VolumetricMesh::Material / Set / Region` 类型
2. 把只读 getter 转发到 catalog
3. 把 `setSingleMaterial`、`addMaterial`、构造路径中的 material 初始化切到 catalog
4. 把 `assignMaterialsToElements` 和 `propagateRegionsToElements` 删除或下沉到 `RegionAssignment`
5. 确认 `sets[0] == allElements` 仍由 catalog 统一维护
6. 跑 clone、subset、subdivide、`scene_to_simulation_mesh` 护栏

#### 5.2.6 风险点

- 这是 Phase B 最容易引入“几何对了、材料错位”的子阶段
- `volumetricMeshIO.cpp` 会直接受影响，因为加载路径要构建 catalog 所需状态
- `SceneToSimulationMesh::fromTetMesh()` 当前逐 element 取 material，任何 `elementMaterial` 漂移都会立刻暴露

#### 5.2.7 完成标准

- `VolumetricMesh` 不再直接维护 `elementMaterial`
- `sets` / `regions` / `elementMaterial` 的同步规则在代码中只有一个权威实现
- `makeTwoTetMeshWithDistinctMaterials()` 和 `makeTwoCubeMeshWithDistinctMaterials()` 相关测试继续通过

### 阶段 B3：去掉 `friend` 编辑路径，建立受控 mutation 边界

#### 5.3.1 目标

把 [volumetricMeshEdit.cpp](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/volumetricMeshEdit.cpp) 从“直接摸内部字段”的实现，改成“只操作 data object 与 material catalog”的实现。

#### 5.3.2 建议新增文件

- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/internal/mesh_mutation.h`
- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/internal/mesh_mutation.cpp`
- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/internal/mesh_subset_editor.h`
- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/internal/mesh_subset_editor.cpp`

#### 5.3.3 设计建议

优先方案：

- `VolumetricMesh` 提供一个非常窄的内部 mutation access
- access 只暴露 `VolumetricMeshData&` 与 `MaterialCatalog&`
- `editing::subset_in_place` 与 `editing::remove_isolated_vertices` 通过这个 access 工作

不建议继续保留：

- `friend namespace editing`
- 对 `numElements`、`elements`、`elementMaterial`、`sets` 的散乱写入

#### 5.3.4 本阶段还要顺手收掉的逻辑

`CubicMesh::subdivide()` 也属于 mutation hotspot，建议在 B3 一起改成复用同一套 mutation 基础设施，而不是继续单独手写：

- 旧 element 到新 element 的扩展映射
- set 扩展
- material 扩展
- 顶点追加与重编号

如果 B3 不把 `subdivide()` 一并纳入，后面还是会留下第二条旁路。

#### 5.3.5 需要改动的现有文件

- [volumetricMeshEdit.h](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/volumetricMeshEdit.h)
- [volumetricMeshEdit.cpp](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/volumetricMeshEdit.cpp)
- [cubicMesh.cpp](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/cubicMesh.cpp)
- [volumetricMesh.h](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/volumetricMesh.h)

#### 5.3.6 测试安排

继续扩这些文件，而不是回收成单一大文件：

- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/tests/core/scene/volumetricMesh/volumetric_mesh_edit_test.cpp`
- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/tests/core/scene/volumetricMesh/cubic_mesh_test.cpp`
- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/tests/core/scene/volumetricMesh/internal/mesh_subset_editor_test.cpp`

建议新增断言：

- `subset_in_place` 后 `sets[0]` 仍覆盖剩余全部元素
- `subset_in_place` 后 `elementMaterial.size() == numElements`
- `subdivide()` 后两个原始 cube 的 material 分区仍成块扩展
- `remove_isolated_vertices` 不会破坏 element index 连续性

#### 5.3.7 完成标准

- `editing` 不再依赖 `friend` 穿透字段
- `CubicMesh::subdivide()` 不再单独维护第二套材料同步逻辑
- `volumetric_mesh_edit_test.cpp` 和 `cubic_mesh_test.cpp` 通过

### 阶段 B4：把非虚通用算法迁出基类成员实现

#### 5.4.1 目标

让 `VolumetricMesh` 逐步退化为 “窄 façade + 虚接口”，而不是“数据对象 + 材料对象 + 算法大杂烩”。

#### 5.4.2 建议新增文件

- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/internal/mesh_queries.h`
- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/internal/mesh_queries.cpp`
- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/internal/mesh_mass_properties.h`
- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/internal/mesh_mass_properties.cpp`
- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/internal/mesh_transforms.h`
- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/internal/mesh_transforms.cpp`

#### 5.4.3 第一批建议迁出的函数

- `getVolume`
- `getMass`
- `getInertiaParameters`
- `getMeshGeometricParameters`
- `getBoundingBox`
- `getVerticesInElements`
- `getElementsTouchingVertices`
- `getElementsWithOnlyVertices`
- `getVertexNeighborhood`
- `getClosestVertex`
- `getContainingElement`
- `computeGravity`
- `applyDeformation`
- `applyLinearTransformation`
- `renumberVertices`

说明：

- `containsVertex`、`getElementVolume`、`computeBarycentricWeights` 这类与元素类型强相关的虚函数先不动
- B4 的目标是把“通用实现”从类本体挪出去，不是立刻改最终 public API 名称

#### 5.4.4 需要改动的现有文件

- [volumetricMesh.cpp](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/volumetricMesh.cpp)
- [volumetricMesh.h](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/volumetricMesh.h)
- [tetMesh.cpp](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/tetMesh.cpp)
- [cubicMesh.cpp](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/cubicMesh.cpp)

#### 5.4.5 测试安排

建议新增：

- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/tests/core/scene/volumetricMesh/mesh_queries_test.cpp`
- `/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/tests/core/scene/volumetricMesh/internal/volumetric_mesh_data_test.cpp`

现有测试继续作为行为护栏：

- `tet_mesh_test.cpp`
- `cubic_mesh_test.cpp`
- `generate_surface_mesh_test.cpp`
- `scene_to_simulation_mesh_test.cpp`

#### 5.4.6 完成标准

- `volumetricMesh.cpp` 中大部分非虚算法实现已经外移
- `VolumetricMesh` 成员函数只剩 façade 转发或虚接口桥接
- Phase C 开始拆 I/O 时，不再需要继续先碰几何 / 材料存储布局

## 6. Phase B 建议提交拆分

不要把 B1 到 B4 混成一个提交。建议至少拆成下面 5 个提交：

1. `B1-data-object`: 新增 `VolumetricMeshData`，接入 `VolumetricMesh`
2. `B2-material-catalog`: 新增 `MaterialCatalog` / `RegionAssignment`
3. `B3-editing-boundary`: 改 `subset_in_place` / `remove_isolated_vertices` / `subdivide`
4. `B4-algorithms`: 迁出 `mesh_queries` / `mesh_mass_properties` / `mesh_transforms`
5. `B4-tests`: 补所有新增测试和 CMake 源列表

如果某一步无法独立保持可编译，就说明拆分方式还不够好，应该继续缩小提交范围，而不是把更多逻辑揉进去。

## 7. 需要同步改的构建与测试文件

除了源码本体，Phase B 几乎一定会碰到下面这些文件：

- [CMakeLists.txt](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/CMakeLists.txt)
- [CMakeLists.txt](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/tests/core/scene/CMakeLists.txt)

如果你给 `tests/core/scene/volumetricMesh/` 增加了 `internal/` 子目录，对应的 `CORE_SCENE_VOLUMETRIC_MESH_SOURCES` 也要同步拆开，不要继续用一个越来越长的列表硬塞。

建议做法：

- 保留 `tet_mesh_test.cpp`、`cubic_mesh_test.cpp`、`volumetric_mesh_edit_test.cpp` 这些行为测试
- 新结构级测试追加单独 source 条目
- 如果数量继续增长，可以在 `tests/core/scene/volumetricMesh/CMakeLists.txt` 再下沉一层局部列表文件

## 8. Phase B 退出条件

Phase B 结束时，至少要满足下面这些条件，才应该进入 Phase C：

1. `VolumetricMesh` 不再直接持有几何数据和材料同步缓存
2. `editing` 不再通过 `friend` 直接修改内部字段
3. `CubicMesh::subdivide()` 复用统一 mutation 路径
4. `VolumetricMesh` 的通用算法已迁出到内部算法层
5. 现有关键行为测试全部通过：
   - `core.scene.volumetric_mesh`
   - `core.energy.scene_to_simulation_mesh`
   - `core.energy.material_binding`
   - `api.pgo_c.basic`
   - `cubicMesher.tool.uniform`

如果 Phase B 做完后，`volumetricMesh.cpp` 仍然同时拥有：

- 几何数组
- 材料数组
- 编辑逻辑
- 大量通用算法

那就说明这轮还没有真正完成内部结构拆分，只是把代码挪了一点位置。

## 9. 下一阶段入口

Phase B 完成后，再进入 Phase C：

- 拆 [volumetricMeshIO.cpp](/Users/jinceyang/Desktop/codebase/libpgo_refactor/libpgo/src/core/scene/volumetricMesh/volumetricMeshIO.cpp)
- 让 reader / writer / detector / material serde 分离
- 再准备后续的 API 签名清理与下游统一切换

也就是说，Phase B 的价值不是“让类图更好看”，而是把 Phase C 之后真正危险的切接口动作，建立在已经稳定的内部边界上。
