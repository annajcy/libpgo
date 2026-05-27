# Cubic Mesh FEM 集成计划

## 概述

首期目标是在 libpgo 中打通 cubic（六面体/体素）网格 FEM 的 `runSim + 基本 FEM 装配` 主路径，分三大层：

1. **网格生成工具** (`cubicMesher`) -- 从均匀网格或三角面片体素化生成 `.veg` cubic 网格
2. **网格读取管线** -- `loadCubicMesh()` + `DeformationModelManager` 支持 CUBIC 类型 + `runSim` 自动检测
3. **变形模型** (`CubicMeshDeformationModel`) -- 8 节点三线性六面体单元的逐元 FEM

参考实现：`/Users/jinceyang/Desktop/codebase/libpgo_main/`

### 首期范围

- `cubicMesher`：生成 `.veg` cubic mesh 及配套表面网格资产
- `loadCubicMesh()`：将 cubic volumetric mesh 转为 `SimulationMesh`
- `runSim`：识别并加载 cubic volume mesh，走通主仿真入口
- `DeformationModelManager` + `DeformationModelAssembler`：支持 cubic element 的 FEM 初始化与装配
- `CubicMeshDeformationModel`：提供 cubic 单元的能量、梯度、Hessian 及 mixed derivative
- `triangleMeshExternalContactHandler` / `triangleMeshSelfContactHandler`：去掉表面 embedding 展开时残留的 tet-only 假设，使 cubic 示例里的 contact 路径可用
- `examples/cubic/<case>/`：提供可运行的端到端示例

### 首期非目标

- 将 repo 中所有 tet-specific 工具和约束一起泛化到 cubic
- 一次性重写整套 contact 系统；首期只修复 cubic 主样例会经过的表面 embedding 展开逻辑，不扩展额外 contact feature
- 修改 `prescribedPrincipleStressConstraintFunctions`、`tetVolumeConstraintFunctions` 等已有 tet 专用模块
- 宣称“整个 libpgo 所有 solidDeformationModel 相关功能均已支持 cubic”

本计划中的“集成完成”仅指首期范围内的主路径完成；tet-only 辅助模块可在后续阶段按需扩展。

---

## 阶段 1：Cubic Mesher 工具

### 目标
添加一个最小版 `cubicMesher` 工具，只支持 `triangle mesh -> cubic mesh (.veg)`，并可选导出对应的表面 `.obj`。不引入新的第三方依赖，为后续阶段提供 cubic mesh 资产。

### 新建文件

#### `src/tools/cubicMesher/CMakeLists.txt`

```cmake
set(AVAILABLE_LIBS)
foreach(lib ${PGO_GLOBAL_LIBRARY_TARGETS})
  if(TARGET ${lib})
    list(APPEND AVAILABLE_LIBS ${lib})
  endif()
endforeach()

add_libpgo_tools(cubicMesher
  "cubicMesher.cpp;cubicMesherIO.cpp;triangleMeshVoxelizer.cpp"
  "${AVAILABLE_LIBS}")
target_link_libraries(cubicMesher PRIVATE argparse::argparse)
```

#### `src/tools/cubicMesher/cubicMesher.cpp`

主入口，不再设计子命令；默认行为就是将输入三角面片体素化为 cubic mesh。

- 参数：`--input-mesh PATH`, `--resolution N`, `--output-mesh PATH`, `--output-surface PATH`（可选）, `--E`, `--nu`, `--density`

实现不追求做成通用 meshing 工具，而是优先满足本计划首期所需的 cubic 资产生成。
首期固定不处理 `padding-voxels`，等主路径稳定后如确有需要再扩展。

其中 `resolution` 的语义约定为：**输入 mesh AABB 的最短边上划分的 cubic 单元数**。
若三轴包围盒边长分别为 `sx, sy, sz`，则：

- `smin = min(sx, sy, sz)`
- 单元边长 `h = smin / resolution`
- 三轴单元数分别为：
  - `nx = ceil(sx / h)`
  - `ny = ceil(sy / h)`
  - `nz = ceil(sz / h)`

因此最终规则 grid 不必是 `N x N x N`，而是 `nx x ny x nz`；但所有单元的边长都相同，仍然是 cubic 单元。

#### `src/tools/cubicMesher/triangleMeshVoxelizer.h/cpp`

- `TriangleMeshVoxelizerOptions` 结构体
- `createTriangleMeshCubicMesh()` 函数
- 流水线：
  1. 加载输入三角面片
     - 首期采用严格输入检查：输入 surface mesh 默认要求无自交、manifold、且闭合
     - 可复用 repo 现有能力做校验：
       - `TriMeshBVTree::selfIntersectionExact(...)`：检测几何自交
       - `areTrianglesManifold(...)`：检测是否为 manifold
       - `getExteriorEdges(...)`：检测是否存在边界边
     - 若存在 `self-intersection`、`non-manifold` 或 `boundary edge`，则直接 `throw` 退出，而不是仅给 warning 后继续体素化
  2. 计算 AABB，并按最短边分辨率规则构造规则 cubic grid；首期不额外添加 padding 体素层
     - 记 AABB 边长为 `sx, sy, sz`
     - 由 `resolution` 计算单元边长 `h = min(sx, sy, sz) / resolution`
     - 用 `nx = ceil(sx / h)`、`ny = ceil(sy / h)`、`nz = ceil(sz / h)` 确定三轴单元数
     - 若 `nx * h`、`ny * h`、`nz * h` 略大于原始 AABB，对多出的长度在两侧均匀分配，使最终 world-space grid 居中包住原 mesh
  3. 对规则体素网格进行最小占据判定，优先使用 repo 现有能力：
     - 默认判定规则为：`occupied = centerInside || triangleAABBOverlap`
     - `centerInside`：体素中心的 inside/outside 判断复用 `PointInsideOutsideQuery`
     - `triangleAABBOverlap`：不对每个 voxel 暴力枚举所有 triangle，而是先用 BV tree 查询与该 voxel AABB 可能相交的 candidate triangles，再对 candidate 调用 `whetherTriangleIntersectBoundingBox(...)`
     - 实现顺序上优先做 `centerInside`；仅当中心点不在内部时，才走 BV tree + triangle/AABB overlap 的边界补偿路径
  4. 遍历 `nx x ny x nz` 个 voxel，对每个 voxel 计算 world-space AABB 与中心点，收集占据的 voxel index
  5. 通过 `CubicMesh::createFromUniformGrid()` 创建 cubic mesh，并将其顶点映射回上一步定义的 world-space grid
     - 输入给 `createFromUniformGrid()` 的是占据 voxel 的整数三元组索引 `(i, j, k)`
     - `createFromUniformGrid()` 内部会为每个 occupied voxel 枚举 8 个角点格点坐标，并对所有角点去重，形成全局 `vertices`
     - 再通过 `map<tripleIndex, int>` 将每个角点格点坐标映射为一个全局顶点编号，填入 `elements`
     - `elements` 在语义上按每 8 个整数一组理解：`elements[8 * e + 0 ... 7]` 即第 `e` 个 cubic 单元连接的 8 个全局顶点编号，也就是该 hex 单元的 connectivity
     - 这样相邻 voxel 会自动共享公共角点，避免重复顶点；随后再把 canonical grid 上的顶点坐标映射回前面定义的 world-space `gridMin + h * (ix, iy, iz)`
     - 这里的 `(ix, iy, iz)` 指的就是顶点对应的整数格点坐标；world-space remap 采用最简单的轴对齐规则：`worldPos = gridMin + h * (ix, iy, iz)`
     - 之所以需要这一步，是因为 `createFromUniformGrid()` 默认先在 canonical 坐标系里生成顶点（例如 `-0.5 + i / resolution`），而首期 `cubicMesher` 需要的输出是前面 voxelizer 定义好的真实 world-space 规则网格

不引入新的第三方库；尽量复用现有 `TriMeshGeo`、`PointInsideOutsideQuery`、`geometryQuery` 等基础设施。

#### `src/tools/cubicMesher/cubicMesherIO.h/cpp`

- `saveCubicMesh()`：保存为 `.veg`
- `writeSurfaceMesh()`：调用 `GenerateSurfaceMesh` 提取表面并保存为 `.obj`
  - 对 `CubicMesh` 调用 `GenerateSurfaceMesh::computeMesh(..., triangulate=true)`，提取外表面三角片
  - `GenerateSurfaceMesh` 会通过统计每个 quad 面是否被两个 hex 共享来剔除内部面，只保留真正的外表面
  - 将输出的表面顶点和三角形面片组装为 `Mesh::TriMeshGeo`，再调用 `TriMeshGeo::save()` 写出 `.obj`

### 修改文件

#### `src/tools/CMakeLists.txt`

添加：
```cmake
add_subdirectory(cubicMesher)
```

### 验证

- 对 box.obj 进行体素化，验证生成的单元数合理
- 提取表面网格并验证拓扑正确性
- 生成后续阶段所需的 `examples/cubic/box/` 资产：
  ```bash
  cubicMesher --input-mesh examples/cubic/box/box.obj \
      --resolution 4 \
      --output-mesh examples/cubic/box/box.veg \
      --E 10000000 --nu 0.45 --density 1000
  ```

---

## 阶段 2：Phase 2 + 3 联动切分

### 目标
将“cubic mesh 读取管线”和“cubic FEM 核心模型”按可执行的小批次联动推进，而不是把 phase 2 误解为一个可独立收口的纯 loader 阶段。

repo truth 决定了这两层必须联动：

- `runSim` 目前仍是 tet-only 主路径
- `DeformationModelManager` 还没有 `SimulationMeshType::CUBIC` 分支
- `DeformationModelAssembler` 仍残留固定局部维度假设，不能直接接收 cubic 的 24 DOF/element
- 一旦 cubic mesh 真正进入 `DeformationModelManager -> DeformationModelAssembler -> DeformationModelEnergy` 主链路，就必须有可用的 `CubicMeshDeformationModel`

因此新的实施方式不是 “先做完 phase 2，再开始 phase 3”，而是拆成下面三个连续可验证的小批次：

1. **2A: 加载与通用层打底**
2. **2B: cubic FEM 核心模型**
3. **2C: 启用 runSim 端到端 cubic 主路径**

每个小批次都必须留下可验证产物，并且前一批次不要提前暴露一个必然在下一批次前崩掉的半成品用户路径。

### 2A. 加载与通用层打底

#### 目标
先补齐不依赖 cubic FEM 数值实现本身的基础设施，让 repo 对 cubic mesh 的数据通路完整、对 assembler 的局部维度假设消除，但先不对 `runSim` 暴露一个会走到空缺 FEM 的 cubic 仿真入口。

#### 2A.1 SimulationMesh 加载器

修改 `src/core/solidDeformationModel/simulationMesh.h/cpp`：

- 在已有 `loadTetMesh()` 声明附近添加 `loadCubicMesh(const VolumetricMeshes::CubicMesh *)`
- 在实现里仿照 `loadTetMesh()`：
  - 导出 cubic 顶点坐标
  - 导出每个单元的 8 个顶点索引
  - 从 `ENuMaterial` 构造 `SimulationMeshENuMaterial`
  - 创建 `SimulationMeshType::CUBIC`

明确要求：

- `numElementVertices=8`
- 材料复制策略与 `loadTetMesh()` 一致
- 不在这里做任何 cubic 专属 FEM 逻辑

实现过程补充：

- `SimulationMesh` 构造函数接受的是两块连续数组风格的数据，而不是直接吃 `CubicMesh` 对象：
  - `vertexPositions`：长度为 `3 * numVertices` 的扁平 `double` 数组，布局为
    `x0, y0, z0, x1, y1, z1, ...`
  - `elementVertexIndices`：长度为 `numElements * numElementVertices` 的扁平 `int` 数组；对 cubic 来说就是每个单元连续 8 个全局顶点编号
- 因此 `loadCubicMesh()` 的职责只是把 `VolumetricMeshes::CubicMesh` 中按对象访问的数据，转换成 `SimulationMesh(...)` 所需的连续输入缓存
- 顶点导出流程：
  - 遍历 `cubicMesh->getNumVertices()`
  - 对每个顶点调用 `cubicMesh->getVertex(vi)` 读取 `Vec3d`
  - 依次把 `x/y/z` 压入 `std::vector<double> vtx`
  - 最终 `vtx.data()` 作为 `SimulationMesh` 的 `vertexPositions`
- 单元连通性导出流程：
  - 遍历 `cubicMesh->getNumElements()`
  - 对每个单元按局部顶点序 `j = 0..7` 调用 `cubicMesh->getVertexIndex(ei, j)`
  - 依次压入 `std::vector<int> elementVertices`
  - 最终布局按每 8 个整数为一组，对应一个 cubic 单元的 connectivity
  - `numElementVertices` 在这里固定传 `8`，不在 loader 中重排 cubic 顶点顺序
- 材料导出流程与 `loadTetMesh()` 保持完全一致：
  - 对每个 element 读取 `cubicMesh->getElementMaterial(ei)`
  - 用 `downcastENuMaterial(...)` 取出 `ENuMaterial`
  - 新建 `SimulationMeshENuMaterial(mat->getE(), mat->getNu())`
  - 将该材料指针放入 `materials`
  - 同时令 `elementMaterialIndices.push_back(ei)`
- 上述策略意味着这里仍然采用“每个单元对应一份 `SimulationMeshENuMaterial` 拷贝”的方案，而不是共享 `CubicMesh` 的 material table；这样可以与当前 `loadTetMesh()` 的语义、生命周期和后续 `SimulationMeshImpl` 内部 clone 逻辑保持一致
- 材料生命周期说明：
  - `loadCubicMesh()` 内部临时维护的是 `std::vector<SimulationMeshMaterial *> materials`
  - 这些指针只是作为 `SimulationMesh(...)` 构造参数传入，并不会被 `SimulationMeshImpl` 直接原样接管
  - `SimulationMeshImpl` 构造函数内部会遍历传入的 `mats`，对每个材料再执行一次 `clone()`，存入自己的 `materials` 成员
  - 因此真正长期归 `SimulationMesh` 持有的是 `SimulationMeshImpl::materials` 里的 clone 副本，而不是 loader 中 `new` 出来的那批临时对象
  - 所以在 `SimulationMesh` 构造完成后，loader 必须像现有 `loadTetMesh()` 一样立刻 `delete` 掉自己临时创建的 `SimulationMeshENuMaterial *`
  - 若不释放这批临时对象，会因为 `SimulationMeshImpl` 已经持有 clone 副本而产生额外内存泄漏
- 这一层只负责几何和材料数据搬运，不负责：
  - hex shape function
  - 积分点 / Jacobian
  - cubic stiffness / force / Hessian
  - 任何 voxel/cubic 特有数值逻辑

#### 2A.2 DeformationModelAssembler 动态局部维度

修改 `src/core/solidDeformationModel/deformationModelAssembler.h/cpp`，把 assembly 层从“默认最大 24 DOF 的半固定实现”改成真正按
`simulationMesh->getNumElementVertices() * 3` 工作的动态实现。

这一步是联动切分里的硬前置，因为当前代码里至少存在以下真实问题：

- `computeGradient()` 里局部位置缓冲已经是 `V24d`
- 但局部梯度缓冲还是 `V18d`
- 这对 cubic 的 `24` 自由度块会直接出错

具体修改边界：

- `localp`、`localGradx`、局部 Hessian、`d2E_dxda`、`d2E_dxdb` 等临时量统一改为 `ES::VXd` / `ES::MXd`
- `IndexMatrix` 改为动态矩阵；按 `neleVtx * 3` 和参数维度初始化
- 去掉固定的 `24 * 24` 临时数组假设
- `plasticParam` / `elasticParam` 改为按 `numPlasticParams` / `numElasticParams` 分配的动态缓冲区
- 确保 tet、shell 现有路径不回退

原则：

- assembly 层统一动态维度
- `TetMeshDeformationModel`、`KoiterDeformationModel`、后续 `CubicMeshDeformationModel` 仍可保留各自固定维度内部数学实现

现状问题：

- `DeformationModelAssembler` 构造阶段已经会读取 `simulationMesh->getNumElementVertices()`，并且模板稀疏矩阵的 triplet 填充循环也大多按 `neleVtx` 工作，说明这一层已经部分具备按单元顶点数泛化的意图
- 但局部缓冲区和 inverse-index 缓存仍残留固定维度假设：
  - `computeEnergy()`、`computeGradient()`、`computeHessian()`、`compute_df_da()`、`compute_df_db()` 中的局部位置缓冲仍写成 `ES::V24d`
  - `computeGradient()` 中的局部梯度缓冲仍写成 `ES::V18d`，与 cubic 所需的 `24` 维局部自由度不兼容
  - `computeHessian()`、`compute_df_da()`、`compute_df_db()` 里的局部二阶导缓存仍是 `double localKData[24 * 24]`
  - 头文件中的 `IndexMatrix` 仍固定为 `24 x 24`，而 `elementKInverseIndices`、`element_dfda_InverseIndices`、`element_dfdb_InverseIndices` 都依赖该固定类型
  - `plasticParam[20]`、`elasticParam[20]` 也是假定参数数目不会超过 20 的静态上限
- 这导致当前 assembler 处于“循环维度看似动态，但实际缓存仍按 tet/shell 常见尺寸写死”的状态；一旦 cubic 单元真正进入 gradient / Hessian / parameter Jacobian 路径，就会出现维度不匹配甚至越界写入风险

改造目标：

- 将 assembler 这一层的局部工作维度统一定义为：
  - `localDOFs = simulationMesh->getNumElementVertices() * 3`
  - `plasticCols = numPlasticParams`
  - `elasticCols = numElasticParams`
- assembler 只负责：
  - 从全局位移 `x` 中抽取单元局部自由度块
  - 调用对应 `DeformationModel` 计算局部能量/梯度/Hessian/参数导数
  - 将局部结果按 inverse-index 映射散装回全局向量/稀疏矩阵
- assembler 不再假设局部块固定是 `12`、`18` 或 `24`，而是完全由当前 mesh 类型驱动
- 与此同时，具体 FEM 模型内部仍允许保留固定维度实现：
  - tet 模型内部继续按 12 DOF 数学实现
  - shell / koiter 继续按自身固定局部自由度实现
  - 后续 cubic 模型内部可按 24 DOF 固定实现
- 也就是说，此阶段做的是“assembly 层动态化”，不是强迫所有 `DeformationModel` 内核一起改成动态矩阵风格

具体改哪些成员 / 函数：

- 头文件 `deformationModelAssembler.h`
  - 删除固定 `24 x 24` 的 `IndexMatrix` 作为主类型依赖
  - 统一使用动态 `DynamicIndexMatrix`
  - 将
    - `elementKInverseIndices`
    - `element_dfda_InverseIndices`
    - `element_dfdb_InverseIndices`
    三组成员改为存储动态索引矩阵
- 构造函数 `DeformationModelAssembler::DeformationModelAssembler(...)`
  - 保留现有基于 `neleVtx` 的 triplet 模板生成逻辑
  - 但在构造 `elementKInverseIndices` 时，按 `(3 * neleVtx) x (3 * neleVtx)` 创建动态索引矩阵
  - 在构造 `element_dfda_InverseIndices` 时，按 `(3 * neleVtx) x numPlasticParams` 创建动态索引矩阵
  - 在构造 `element_dfdb_InverseIndices` 时，按 `(3 * neleVtx) x numElasticParams` 创建动态索引矩阵
- `getPlasticParameters()` / `getElasticParameters()`
  - 保持“从全局参数表中提取当前单元参数块”的语义不变
  - 调用侧不再传入固定长度栈数组，而改为传入按 `numPlasticParams` / `numElasticParams` 分配的动态缓冲
- `computeEnergy()`
  - 将 `localp` 从 `ES::V24d` 改为长度为 `localDOFs` 的 `ES::VXd`
  - 将 `plasticParam` / `elasticParam` 从固定长度数组改为动态缓冲
- `computeGradient()`
  - 将 `localp` 从 `ES::V24d` 改为 `ES::VXd`
  - 将 `localGradx` 从错误的 `ES::V18d` 改为长度为 `localDOFs` 的 `ES::VXd`
  - sanity check 循环继续按 `neleVtx * 3` 遍历即可
- `computeHessian()`
  - 将局部二阶导缓存从 `double localKData[24 * 24]` 改为按 `localDOFs * localDOFs` 分配的动态缓冲
  - 再映射为动态 `ES::MXd(localDOFs, localDOFs)`
- `compute_df_da()`
  - 将局部位置缓冲与参数缓冲都改为动态
  - 将局部导数缓存改为按 `localDOFs * numPlasticParams` 分配
  - 再映射为动态 `ES::MXd(localDOFs, numPlasticParams)`
- `compute_df_db()`
  - 将局部位置缓冲与参数缓冲都改为动态
  - 将局部导数缓存改为按 `localDOFs * numElasticParams` 分配
  - 再映射为动态 `ES::MXd(localDOFs, numElasticParams)`
- 验证要求：
  - tet 现有装配路径行为不回退
  - shell 现有装配路径行为不回退
  - assembler 能够正确接受 `SimulationMeshType::CUBIC` 的 `8` 顶点单元输入，并至少在维度层面安全通过 energy / gradient / Hessian / `df/da` / `df/db` 路径

#### 2A.3 runSim 的 volumetric mesh 基础改造

修改 `src/tools/runSim/runSim.cpp`，但在 2A 只完成“tet/cubic volumetric mesh 输入层梳理”和不依赖 cubic FEM 的共用路径整理：

- 保留已有 `"tet-mesh"` 键
- 新增 `"cubic-mesh"` 键，用它与 `"tet-mesh"` 显式区分两类 volumetric mesh 输入
- 配置层要求两者至多出现一个；若同时出现或都缺失，则直接报错
- 对已提供的 mesh 文件先调用 `VolumetricMesh::getElementType(filename)` 做类型校验：
  - 若配置使用 `"tet-mesh"`，则文件类型必须是 `VolumetricMesh::TET`
  - 若配置使用 `"cubic-mesh"`，则文件类型必须是 `VolumetricMesh::CUBIC`
  - 若键名声明的类型与文件真实类型不一致，则直接报错并退出，而不是静默接受
- 在完成上述校验后，再按配置键构造对应的 `TetMesh` 或 `CubicMesh`
- 构造完成后在代码内部统一落到 `VolumetricMeshes::VolumetricMesh *` 视角，供后续共用路径使用
- 将质量矩阵入口改为统一使用 `VolumetricMeshes::VolumetricMesh *`
- 将重心插值入口改为统一使用 `VolumetricMeshes::VolumetricMesh *`

这里依赖的 repo truth：

- `GenerateMassMatrix::computeMassMatrix(const VolumetricMesh *, ...)` 已支持 cubic
- `BarycentricCoordinates` 构造函数本来就接受 `VolumetricMesh *`

2A 结束时允许的状态：

- `runSim` 代码层已经具备读取 cubic volume mesh 的基础结构
- 但真正把 cubic mesh 送进 `DeformationModelManager::init()` 的最终启用可以等 2B/2C 一起收口

#### 2A 验证

- 新增或临时运行一个 small helper test：对 `examples/cubic/box/box.veg` 调用 `loadCubicMesh()`，验证顶点数、单元数、element vertex count 正确
- 编译并跑 tet / shell 相关已有测试或至少关键 target，确认 assembler 动态化不回退
- 验证 `runSim` 对 tet 旧配置仍兼容：`"tet-mesh"` 老键仍能工作
- 验证 `runSim` 对 cubic 新配置键能正确识别：`"cubic-mesh"` 路径可以被读取并进入共用 mass / interpolation 初始化流程
- 验证类型校验逻辑：例如将 tet 文件错误地写到 `"cubic-mesh"` 或将 cubic 文件错误地写到 `"tet-mesh"` 时，`runSim` 会在入口阶段直接报错

### 2B. CubicMeshDeformationModel 核心 FEM

#### 目标
补齐 cubic 单元真正的能量、梯度、Hessian 和 mixed derivative，使 `DeformationModelManager` 能对 cubic 元素创建真正可用的 deformation model。

#### 新建文件

`src/core/solidDeformationModel/cubicMeshDeformationModel.h`

- 类 `CubicMeshDeformationModel : public DeformationModel`
- 构造函数：`CubicMeshDeformationModel(const double restPositions[24], ElasticModel*, PlasticModel*)`
- 当前基类所需纯虚接口全部实现：
  - `allocateCacheData()`, `freeCacheData()`, `prepareData()`
  - `computeEnergy()`, `compute_dE_dx()`, `compute_d2E_dx2()`
  - `compute_d2E_dxda()`, `compute_d2E_dxdb()`
  - `vonMisesStress()`, `maxStrain()`
  - `enableSPD()`
- `getNumVertices() = 8`, `getNumDOFs() = 24`, `getNumMaterialLocations() = 8`

`src/core/solidDeformationModel/cubicMeshDeformationModel.cpp`

实现策略：

- 直接参考 `libpgo_main` 的 cubic deformation model
- 保持代码自包含，只依赖当前 repo 已有的 `DeformationModel` / `ElasticModel3DDeformationGradient` / `PlasticModel3DDeformationGradient` / `EigenSupport`

#### 数学设计与参考实现对齐为

- 单元类型固定为 8 节点三线性 hex，局部顶点顺序严格与 `CubicMesh` 现有 8 点顺序一致；这里不做额外重排，也不引入“按标准 hex 顺序重新编号”的适配层
- 参考单元参数域采用 `[0, 1]^3`，局部坐标记为 `(alpha, beta, gamma)`，而不是 `[-1, 1]^3`
- 使用 `2 x 2 x 2` Gauss 积分，共 `8` 个积分点；每轴积分点坐标固定为：
  - `0.5 - 0.5 / sqrt(3)`
  - `0.5 + 0.5 / sqrt(3)`
  每个积分点的权重固定为 `1 / 8`
- 在每个积分点上先按参考实现计算三线性形函数对参数坐标的导数矩阵 `dN_dabc`，其布局与参考实现一致，按 `3 x 8` 存储：
  - 第 0 行是 `dN / d alpha`
  - 第 1 行是 `dN / d beta`
  - 第 2 行是 `dN / d gamma`
- 记静止构型顶点矩阵为 `X in R^(3x8)`，当前构型顶点矩阵为 `x in R^(3x8)`，则在每个积分点上：
  - `Dm = X * dN_dabc^T`
  - `Fref = x * dN_dabc^T * DmInv`
  这里 `Fref` 表示尚未施加塑性分解前、由几何映射直接得到的 deformation gradient，命名与参考实现保持一致
- 塑性模型仍沿用当前 repo 的 `PlasticModel3DDeformationGradient` 路线：
  - `Fp = A(a)`
  - `FpInv = A(a)^(-1)`
  - `detFp = det(A(a))`
  - `Fe = Fref * FpInv`
  也就是说 elastic model 实际看到的是 `Fe`，不是 `Fref`
- 每个积分点的能量项写成：
  - `psi_q = psi(Fe_q, materialParam)`
  - `weightDetJ_q = (1 / 8) * abs(det(Dm_q))`
  - `E_q = psi_q * weightDetJ_q * detFp`
  单元总能量是 `sum_q E_q`
- 梯度和 Hessian 的主公式也与参考实现一致：
  - 一阶导使用 `P = d psi / d Fe`
  - `dFdx` 维度固定为 `9 x 24`
  - 单点梯度项通过 `P * Bm` 写回，其中
    `Bm = detFp * FpInv^T * restBm`
  - 单点 Hessian 项通过
    `dFdx^T * dPdF * dFdx`
    累加得到
- `restBm` 和 `rest_dFdx` 都是在构造函数中基于静止构型预计算并长期缓存的“参考积分点数据”：
  - `restBm = weightDetJ * DmInv^T * dN_dabc`
  - `rest_dFdx` 是未乘 `FpInv` 之前、仅依赖静止构型 Jacobian 的 `9 x 24` deformation-gradient Jacobian
  - `prepareData()` 时再基于当前 `FpInv` 生成当前积分点的 `dFdx` 与 `Bm`
- cache 维度策略明确约束为“参数动态、几何固定”：
  - 保留几何相关固定尺寸：
    - 单元自由度固定为 `24`
    - 积分点数量固定为 `8`
    - 每个积分点的 `dFdx` 固定为 `9 x 24`
    - 每个积分点的 `Bm` 固定为 `3 x 8`
    - 每个积分点的 `Fref`、`Fe`、`U`、`V`、`S` 继续用固定尺寸矩阵/向量缓存
  - 去掉参数相关的硬编码上限，不再使用 `kMaxNumPlasticParams` / `kMaxNumElasticParams` 这类静态容量假设
  - plastic / elastic 参数相关缓存改为按运行时参数个数动态分配，包括：
    - `plasticParam`
    - `ddetA_da`
    - `d2detA_da2`
    - `dAInv_dai`
    - `d2AInv_dai_daj`
    - `materialParam`
  - `CacheData` 的动态参数缓冲在 `allocateCacheData()` 阶段一次性按
    `plasticModel->getNumParameters()` 和 `elasticModel->getNumParameters()` 分配完成，避免在每次 `prepareData()` 中重复分配
  - 这样可以保持 hex 几何内核的固定小矩阵实现风格，同时消除参数维度被硬编码上限截断的风险
- 对 plastic 参数 `a` 的 mixed derivative 路线与参考实现一致：
  - 先基于 `dAInv/da`、`d2AInv/da2`、`ddetA/da`、`d2detA/da2` 计算 `Fe` 对参数的变化
  - 再把 `dP/dF` 与 `dFe/da` 链接起来得到 `dP/da`
  - 最终将体积项 `weightDetJ * detFp` 的导数与 `psi(Fe)` 的导数一起累加到 `d2E_dxda`
- 对 elastic/material 参数 `b` 的 mixed derivative 路线也保持参考实现一致：
  - `b` 只进入 elastic model，不进入几何映射和 `Fp`
  - 在每个积分点上调用 `compute_dP_dparam(...)`
  - 再通过 `dFdx^T * dP/db` 形成 `d2E_dxdb`
- `prepareData()` 的缓存组织也按参考实现收口：
  - 单元级缓存：`Fp`、`FpInv`、`detFp`、按运行时维度分配的 plastic 参数导数、按运行时维度分配的 material 参数
  - 积分点级缓存：`Fref`、`Fe`、`U/V/S`、`dFdx`、`Bm`
  这样 `computeEnergy()` / `compute_dE_dx()` / `compute_d2E_dx2()` / `compute_d2E_dxda()` / `compute_d2E_dxdb()` 都只做“读取缓存并逐积分点累加”，不重复几何预处理

#### 同批修改

`src/core/solidDeformationModel/CMakeLists.txt`

- 增加 `cubicMeshDeformationModel.h`
- 增加 `cubicMeshDeformationModel.cpp`

#### 2B 验证

- 为 cubic 单元补一个专用有限差分测试入口，测试对象直接面向 `CubicMeshDeformationModel`，思路参考 `deformationModelFDTest.cpp`，但不要强行复用 tet-only helper 或假定单积分点逻辑
- 测试输入要覆盖一组最小、可控的 8 节点 hex rest shape：
  - 优先使用轴对齐 unit cube 或 edge length 已知的长方体 rest pose
  - 顶点顺序严格按 `CubicMesh` 当前 8 点顺序给出，避免“形函数顺序正确但测试顶点顺序错误”导致假失败
- `prepareData()` / 几何缓存 sanity 检查：
  - 对 rest pose 调用 `prepareData()` 后，不应出现 `NaN` / `Inf`
  - `getNumVertices() == 8`
  - `getNumDOFs() == 24`
  - `getNumMaterialLocations() == 8`
  - 8 个积分点的 `Fe` 在 rest pose 且 `Fp = I` 时应接近单位阵
  - 8 个积分点的 `weightDetJ` 应全部为正，并且总和应接近该 hex 的参考体积
- 能量检查：
  - 在 rest pose 且塑性参数对应 `Fp = I` 时，`computeEnergy()` 应有限且非负
  - 对一个小扰动位移状态 `x = X + eps * dx`，`computeEnergy()` 应保持有限
  - 若所选 elastic model 在 rest pose 的理论最小值为 0，则额外检查 rest energy 接近 0
- `x` 方向有限差分检查：
  - 用中心差分对 `computeEnergy(x)` 关于 `24` 个自由度逐项求导，并与 `compute_dE_dx()` 对比
  - 用中心差分对 `compute_dE_dx()` 关于 `x` 求导，并与 `compute_d2E_dx2()` 对比
  - Hessian 还应满足基本对称性检查：`||H - H^T||` 足够小
- `a`（plastic 参数）相关有限差分检查：
  - 先选用当前 runSim / DMM 主路径计划使用的 `PlasticModel3DDeformationGradient` 具体实例
  - 用中心差分对 `compute_dE_dx()` 关于 plastic 参数 `a` 求导，并与 `compute_d2E_dxda()` 对比
  - 若测试中额外暴露 `compute_dE_da()` 或 `compute_d2E_da2()` helper，则可再补：
    - `E(a)` 的一阶 FD 对 `dE/da`
    - `dE/da` 的二阶 FD 对 `d2E/da2`
  - 但 phase 2B 的硬要求仍以 `compute_d2E_dxda()` 的正确性为主
- `b`（elastic/material 参数）相关有限差分检查：
  - 选用一个带非零材料参数数目的 elastic model 配置，避免 `numElasticParams == 0` 时测试退化为空路径
  - 用中心差分对 `compute_dE_dx()` 关于 material 参数 `b` 求导，并与 `compute_d2E_dxdb()` 对比
  - 若测试中额外暴露 `compute_dE_db()` / `compute_d2E_db2()` helper，可继续补做单独 FD；但 phase 2B 的主验收仍以 `compute_d2E_dxdb()` 为准
- 应力与应变结果的基本正确性检查：
  - `vonMisesStress()` 与 `maxStrain()` 返回的积分点数量应为 `8`
  - rest pose 下结果应有限
  - 对小扰动后结果仍应有限，且不出现异常负体积导致的数值崩溃
- 数值稳定性与误差口径：
  - FD 使用中心差分
  - `x`、`a`、`b` 三类变量的扰动步长应分别调参，不强求共用一个 `eps`
  - 误差同时检查绝对误差与相对误差，避免在接近 0 的条目上出现误判
  - 对所有检查项打印“最大分量误差”和“最大相对误差”，便于后续定位是几何项、塑性项还是材料参数项出错

### 2C. 启用 cubic 主链路

#### 目标
在 2A 和 2B 都就位后，真正把 cubic mesh 接入 `runSim -> DMM -> Assembler -> Energy` 主链路，形成首个可运行的 cubic 仿真入口。

#### 2C.1 DeformationModelManager CUBIC 分支

修改 `src/core/solidDeformationModel/deformationModelManager.cpp`：

- 添加 `#include "cubicMeshDeformationModel.h"`
- 在 mesh element type 分支里加入 `SimulationMeshType::CUBIC`
- 对每个 cubic element 抽取 `8 * 3 = 24` 个静止位置并构造 `CubicMeshDeformationModel`

明确要求：

- `DeformationModelManager` 继续沿用当前项目初始化方式：
  - `dmm->init(plasticModelType, elasticMaterialType)`
  - `dmm->setEnforceSPD(1)`

#### 2C.2 runSim 正式启用 cubic volume mesh

在 2A 的通用 volume mesh 结构基础上，真正启用 cubic 分支：

- 当 `elemType == VolumetricMeshes::VolumetricMesh::CUBIC` 时：
  - 构造 `CubicMesh`
  - 缩放顶点
  - 调用 `loadCubicMesh()`
  - 走与 tet 相同的后续 simulation path

这里不再需要任何 tet 占位逻辑。

现状问题：

- `runSim` 目前已经完成 cubic/tet 配置解析、文件类型校验、体网格缩放、表面插值矩阵生成和质量矩阵生成
- 但在 FEM 初始化前仍然显式拦截 `VolumetricMesh::CUBIC` 并提前退出，只把 cubic 当成“预处理可用、主链路未启用”的状态
- 后续 FEM 初始化代码仍假设 `volumetricMesh` 一定是 `TetMesh`，直接 `dynamic_cast<const TetMesh *>` 并调用 `loadTetMesh()`

改造目标：

- 删除 cubic 的提前退出 gate，让 `runSim` 能真正把 cubic volume mesh 送入 `SimulationMesh -> DeformationModelManager -> DeformationModelAssembler` 主链路
- `runSim` 自身不引入独立的 cubic solver 分支，只在“从 `VolumetricMesh` 构造 `SimulationMesh`”这一步按类型分流
- tet 和 cubic 在得到各自 `SimulationMesh` 之后，统一复用现有的 DMM 初始化、SPD 设置、assembler 构造、plasticity 初始化和后续 simulation path

具体改哪些位置：

- 修改 `src/tools/runSim/runSim.cpp`
- 删除当前 `if (volumetricMesh->getElementType() == VolumetricMeshes::VolumetricMesh::CUBIC) return 1;` 的占位退出逻辑
- 将紧随其后的 tet-only 初始化代码改成按 `volumetricMesh->getElementType()` 分支构造 `SimulationMesh`
- 当 element type 为 `TET` 时：
  - `dynamic_cast<const VolumetricMeshes::TetMesh *>`
  - 调用 `loadTetMesh()`
- 当 element type 为 `CUBIC` 时：
  - `dynamic_cast<const VolumetricMeshes::CubicMesh *>`
  - 调用 `loadCubicMesh()`
- 分支结束后统一得到 `std::shared_ptr<SolidDeformationModel::SimulationMesh> simMesh`
- 后续代码继续统一执行：
  - `dmm->setMesh(simMesh.get(), nullptr, nullptr)`
  - `dmm->init(plasticModelType, elasticMaterialType)`
  - `dmm->setEnforceSPD(1)`
  - `DeformationModelAssembler(dmm, elementWeights.data())`
- 检查 plasticity 初始化代码不再依赖 tet 特有类型假设：
  - 继续通过 `dmm->getDeformationModel(ei)->getPlasticModel()` 查询 plastic model
  - 若仍使用 `PlasticModel3DDeformationGradient` 和 `6` 个 plastic params，则复用当前初始化逻辑
  - 若后续模型参数维度调整，则这里应改为按 plastic model 查询结果驱动，而不是继续写死 element type

收口标准：

- `runSim` 不再把 cubic 当作“预处理后立即退出”的特殊输入
- `examples/cubic/box/box.json` 可以走到 `SimulationMesh` 构造、DMM 初始化和 assembler 构造
- tet 路径保持不回退；cubic 路径与 tet 共享同一条 simulation pipeline，只在 `loadTetMesh()` / `loadCubicMesh()` 处做类型分流

#### 2C.3 示例配置

阶段 4 的 `examples/cubic/box/box.json` 成为本阶段收口时的最小端到端配置。

配置键要求：

- tet 配置继续使用 `"tet-mesh"`
- cubic 配置显式使用 `"cubic-mesh"`
- `surface-mesh` 保持与当前示例一致，指向 `box.obj`

#### 2C.4 修正 contact 中残留的 tet embedding 假设

已定位的问题：

- `runSim` 已经通过 `BarycentricCoordinates` 为表面顶点生成体网格 embedding，并把 `embeddingVertexIndices` / `embeddingWeights` 传给 external/self contact handler
- 该 embedding 的 arity 由体网格类型决定：tet 为 `4`，cubic 为 `8`
- 当前 `src/core/contact/triangleMeshExternalContactHandler.cpp` 和 `src/core/contact/triangleMeshSelfContactHandler.cpp` 在构造 sample-to-volume interpolation matrix 时，仍写死：
  - `for (int j = 0; j < 4; j++)`
  - `vertexEmbeddingIndices->at(vid * 4 + j)`
  - `vertexEmbeddingWeights->at(vid * 4 + j)`
- 这会导致 cubic case 只读取前 `4` 个 cube 顶点权重，后 `4` 个权重被静默丢弃；结果不是 FEM 核心算错，而是 contact 力和接触 Hessian 被回传到错误的 volumetric DOF

改造目标：

- 让 contact handler 按实际 embedding arity 展开表面顶点，而不是默认每个表面点都嵌入到 tet 的 `4` 个节点
- 保持现有 contact energy 和 point-triangle 几何逻辑不变，只修正“sample 如何映射回体网格自由度”这一步

具体改法：

- 修改 `src/core/contact/triangleMeshExternalContactHandler.cpp`
- 修改 `src/core/contact/triangleMeshSelfContactHandler.cpp`
- 在进入 embedding 展开循环前，统一推导：
  - `embeddingArity = vertexEmbeddingIndices->size() / vertices.size()`
- 增加输入一致性检查：
  - `vertexEmbeddingIndices->size() == vertexEmbeddingWeights->size()`
  - `vertices.size() > 0`
  - `vertexEmbeddingIndices->size() % vertices.size() == 0`
  - `embeddingArity > 0`
- 将所有 `vid * 4 + j` / `j < 4` 的表面 embedding 展开逻辑替换为：
  - `vid * embeddingArity + j`
  - `j < embeddingArity`
- 明确不修改那些“几何上本来就代表 1 个点 + 1 个三角形 = 4 个 sample”的常量 `4`，避免把 point-triangle pair coupling 里的合法写法误改掉

验证策略：

- 继续保留现有 FEM-only 验证，确认 cubic deformation model 与 assembler 路径没有回退
- 在 `tests/src/tools/runSim_gtest.cpp` 增补 cubic + contact 的回归检查，至少覆盖：
  - cubic embedding arity 为 `8`
  - contact handler 生成的 sample-to-volume 映射不再只使用前 `4` 个顶点
  - 启用 contact 的 `examples/cubic/box/box.json` 不会因为 embedding 截断而产生明显错误
- 如现有 gtest 不便直接检查 handler 内部插值矩阵，则增加一个最小 contact regression test；核心要求是不再让 cubic contact 走 tet-only 展开路径

#### 2C 验证

- `runSim examples/cubic/box/box.json`
- 验证 cubic box 能完成最小静态或动态仿真，不在初始化阶段崩溃
- 检查：
  - mesh 读取成功
  - 质量矩阵生成成功
  - interpolation matrix 生成成功
  - contact handler 对 surface embedding 的展开使用实际 arity，而不是默认 `4`
  - `DeformationModelManager` 成功创建 cubic element FEM
  - `DeformationModelAssembler` 可装配 cubic 的局部 `24x24` 块

### 交付边界

联动切分后的 phase 2 完成标准不是“代码里出现了 `loadCubicMesh()`”，而是：

- cubic volume mesh 能进入主仿真代码路径
- cubic element 有真正的 deformation model
- assembler 对 tet / shell / cubic 三类局部维度统一成立
- contact handler 不再把 cubic surface embedding 按 tet 的 `4` 个权重截断
- `examples/cubic/box/box.json` 能作为首个端到端 cubic 示例跑通

phase 4 仍保留为“补示例、扩展验证、做更多 case”的阶段，但首个 cubic 端到端样例必须在 2C 收口时就已经可跑。

---

## 阶段 4：示例与端到端测试

### 目标
创建完整的 cubic mesh 仿真示例，验证端到端正确性。

### 新建文件

#### `examples/cubic/box/box.json`

```json
{
    "cubic-mesh": "box.veg",
    "surface-mesh": "box.obj",
    "elastic-material": "stable-neo",
    "g": [0, -9.81, 0],
    "init-vel": [0, 0, 0],
    "init-disp": [0, 0, 0],
    "scale": 1.0,
    "timestep": 0.001,
    "num-timestep": 2000,
    "dump-interval": 10,
    "solver-eps": 1e-6,
    "solver-max-iter": 500,
    "damping-params": [0, 0],
    "contact-stiffness": 1000,
    "contact-sample": 6,
    "contact-friction-coeff": 0,
    "contact-vel-eps": 1e-5,
    "sim-type": "dynamic",
    "output": "ret-cubic-box"
}
```

#### 网格资产

当前示例资产保持与 repo 现状一致：

- 使用阶段 1 的 `cubicMesher` 生成 `box.veg`
- `surface-mesh` 直接复用现有的 `box.obj`
- 不额外要求 `fixed.txt` 或固定点约束文件

### 验证

- 直接运行当前示例配置：`runSim examples/cubic/box/box.json`
- 验证 dynamic 配置下可稳定进入时间步推进并输出结果帧
- 保留 contact 参数开启状态，确认 cubic case 的接触响应不再复现“只吃前 4 个 embedding 权重”的错误
- 与 tet mesh box 示例做定性对比，确认结果合理

---

## 依赖关系与实施顺序

```
阶段 1 (cubicMesher 工具)
  |
  |  生成网格资产
  v
阶段 2A (loadCubicMesh + runSim volumetric mesh 基础改造 + Assembler 动态维度)
  |
  v
阶段 2B (CubicMeshDeformationModel 核心 FEM)
  |
  v
阶段 2C (启用 DMM CUBIC 分支 + runSim cubic 主链路)
  |
  v
阶段 4 (示例 + 端到端测试)
```

**推荐执行顺序**：1 -> 2A -> 2B -> 2C -> 4

---

## 文件变更总览

### 新建文件（8 个源文件）
| 文件 | 说明 |
|------|------|
| `src/tools/cubicMesher/CMakeLists.txt` | cubicMesher 构建配置 |
| `src/tools/cubicMesher/cubicMesher.cpp` | 主入口 |
| `src/tools/cubicMesher/triangleMeshVoxelizer.h` | 体素化声明 |
| `src/tools/cubicMesher/triangleMeshVoxelizer.cpp` | 体素化实现 |
| `src/tools/cubicMesher/cubicMesherIO.h` | IO 工具声明 |
| `src/tools/cubicMesher/cubicMesherIO.cpp` | IO 工具实现 |
| `src/core/solidDeformationModel/cubicMeshDeformationModel.h` | 六面体单元 FEM 头文件 |
| `src/core/solidDeformationModel/cubicMeshDeformationModel.cpp` | 六面体单元 FEM 实现（~600 行，从参考移植） |

### 修改文件（11 个）
| 文件 | 变更内容 |
|------|----------|
| `src/tools/CMakeLists.txt` | 添加 cubicMesher 子目录 |
| `src/core/solidDeformationModel/CMakeLists.txt` | 添加 cubicMeshDeformationModel.h/cpp |
| `src/core/solidDeformationModel/simulationMesh.h` | 添加 `loadCubicMesh()` 声明 |
| `src/core/solidDeformationModel/simulationMesh.cpp` | 实现 `loadCubicMesh()` |
| `src/core/solidDeformationModel/deformationModelManager.cpp` | 添加 CUBIC 分支 |
| `src/core/solidDeformationModel/deformationModelAssembler.h` | 将局部索引/缓冲区改为动态维度 |
| `src/core/solidDeformationModel/deformationModelAssembler.cpp` | 统一 tet/shell/cubic 的 assembly 维度处理 |
| `src/tools/runSim/runSim.cpp` | 自动检测网格类型，支持 cubic mesh |
| `src/core/contact/triangleMeshExternalContactHandler.cpp` | 将表面 embedding 展开从 tet 固定 `4` 改为按实际 arity 推导 |
| `src/core/contact/triangleMeshSelfContactHandler.cpp` | 同步修正 self-contact 的 cubic embedding 展开逻辑 |
| `tests/src/tools/runSim_gtest.cpp` | 增补 cubic + contact 的回归检查，覆盖 embedding arity 与 contact 路径 |
