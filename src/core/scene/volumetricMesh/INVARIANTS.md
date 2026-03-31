# volumetricMesh Invariants

## 1. Geometry Invariants

- `numElementVertices` 在单个 mesh 实例生命周期内固定；`TetMesh` 为 `4`，`CubicMesh` 为 `8`
- `elements.size() == numElements * numElementVertices`
- 任意 `elements[i]` 都必须满足 `0 <= elements[i] < numVertices`
- `vertices.size() == numVertices`
- `getVertexIndex(element, local_vertex)` 的访问前提是：
  `0 <= element < numElements` 且 `0 <= local_vertex < numElementVertices`
- `getVertexIndices(element)` 返回的 span 必须正好覆盖该 element 的全部局部顶点，不允许跨 element

这些约束一旦被破坏，后续所有依赖顶点索引连续布局的逻辑都会一起失效，包括：
`computeBarycentricWeights`、`getContainingElement`、`GenerateSurfaceMesh::computeMesh`、
`GenerateMassMatrix::computeMassMatrix` 和导出路径。

## 2. Material / Set / Region Invariants

- `elementMaterial.size() == numElements`
- `numMaterials == materials.size()`
- `numSets == sets.size()`
- `numRegions == regions.size()`
- `sets[0].getName() == allElements`
- `sets[0]` 必须覆盖 `[0, numElements)` 的全部 element，且不允许漏元素
- `getElementMaterial(el)` 的语义依赖 `elementMaterial[el]` 是合法的 material index
- `regions[ri]` 的 `materialIndex` 与 `setIndex` 都必须引用仍然存在的 material / set

当前实现里，`regions` 与 `sets` 共同描述“哪个元素集合使用哪个材料”，而
`elementMaterial` 是展开后的逐 element 缓存。Review 时不能只看其中一层是否正确。

## 3. Synchronization Rules

### 3.1 声明式表示与运行时缓存

1. `regions` 是“集合到材料”的声明式表示
2. `sets` 定义 element 的成员关系
3. `elementMaterial` 是面向运行时查询的展开缓存
4. 运行时大多数 material 查询最终都走 `elementMaterial`

### 3.2 什么时候必须重建 `elementMaterial`

只要修改了以下任一维度，就必须重新确认 `elementMaterial` 是否与 `sets` / `regions` / `numElements`
仍然同步：

- `sets`
- `regions`
- `numElements`
- material index 的布局

当前代码中，显式承担这件事的核心函数是 `propagateRegionsToElements()`，而
`assignMaterialsToElements()` 在构造与加载路径上负责先建立默认状态、再补齐未覆盖元素。

### 3.3 当前实现里的特殊情况

- 单材料构造路径直接把 `elementMaterial` 设为全 `0`，不经过 `propagateRegionsToElements()`
- 多材料构造路径依赖 `propagateRegionsToElements()` 完成首次展开
- `addMaterial()` 会同时改 `materials`、`sets`、`regions` 和 `elementMaterial`
- `editing::subset_in_place()` 会缩短 `numElements`、裁剪 `elements` / `elementMaterial` / `sets`
- `editing::remove_isolated_vertices()` 会重排顶点并改写 `elements`
- `CubicMesh::subdivide()` 会重建 `vertices` / `elements` / `sets`，然后重新传播 region 到 element

结论：任何未来重构如果把 `sets`、`regions`、`elementMaterial` 的职责拆开，必须先定义新的单一真相来源，
否则很容易引入“几何正确但材料错位”这类隐蔽回归。

## 4. Mutation Hotspots

以下位置是后续重构时的高风险 mutation hotspot：

- `VolumetricMesh::assignMaterialsToElements`
- `VolumetricMesh::propagateRegionsToElements`
- `VolumetricMesh::addMaterial`
- `editing::subset_in_place`
- `editing::remove_isolated_vertices`
- `CubicMesh::subdivide`
- `VolumetricMesh` 多材料构造函数
- `volumetricMeshIO.cpp` 中的加载 / 保存路径

这些位置的共同特点是：同时写入多份互相关联的数据结构，或者依赖“连续扁平数组 + 约定索引”的隐式协议。

## 5. What Must Be Rechecked After Refactor

后续进入 Phase B 及之后的结构调整时，至少要重新检查下面这些行为：

- `TetMesh` / `CubicMesh` 的 ASCII / Binary round-trip 仍保持几何一致
- binary memory constructor 仍与文件序列化协议一致
- `computeBarycentricWeights` 与 `getContainingElement` 的语义未漂移
- `editing::subset_in_place` 后：
  `numElements`、`numVertices`、element index、`sets[0]`、material 查询仍一致
- `CubicMesh::subdivide()` 后：
  element 数、`cubeSize`、set 扩展、material 绑定仍一致
- `GenerateSurfaceMesh::computeMesh()` 对 tet / cubic / triangulated cubic 的面数仍正确
- `SceneToSimulationMesh::fromTetMesh()` 仍保持 tet 拓扑和 ENu material 参数映射

当前 Phase A 的测试护栏已经覆盖上述关键行为；后续任何破坏式接口切换，都应该先确认这些测试仍然可以表达相同语义。
