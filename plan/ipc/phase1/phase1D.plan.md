# Phase 1D Plan: `runIPCSim` tet/cubic unified self-contact IPC 接线

Source plan: `/Users/jinceyang/Desktop/codebase/libpgo/plan/ipc/ipc_friction.plan.md`  
Depends on: `/Users/jinceyang/Desktop/codebase/libpgo/plan/ipc/phase1BC.plan.md`  
Repo-truth reference: `/Users/jinceyang/Desktop/codebase/libpgo/plan/ipc/phase1BC.impl.md`

## Summary

`Phase 1D` 的目标不是再发明一层新的 IPC 数学，而是把已经落地的：

- `SurfaceIPCCore`
- `EmbeddedSurfaceIPCPotentialEnergy`
- `runIPCSim` shell mode

真正推进到 volume path，让 `runIPCSim` 在 **tet / cubic** 两条线上也走同一套 embedded-surface self-contact IPC 主链路。

这一步完成后，仓库里的 unified IPC self-contact 入口应当满足：

- `shell`：`W = I`
- `tet`：`W = BarycentricCoordinates::generateInterpolationMatrix()`
- `cubic`：同样通过 `W = BarycentricCoordinates::generateInterpolationMatrix()`

且三者都通过同一个 `EmbeddedSurfaceIPCPotentialEnergy` 接到 time integrator，而不再为 volume mesh 继续走 legacy `TriangleMeshSelfContactHandler` 主路径。

## Why This Is A Separate Phase

`phase1BC` 已经证明了两件事：

- unified IPC adapter 的 simulation-space pull-back 数学是成立的；
- `runIPCSim` shell-only 主链路已经能独立跑通。

`Phase 1D` 单独拆出来，是因为 volume path 的复杂度主要来自：

- `tet / cubic` mesh 读取与类型判定
- surface embedding `W` 的构造与验证
- volume mass matrix / elastic energy / rest position 的初始化
- 输出表面和 restart 状态的 volume/surface 坐标映射

也就是说，这一批的核心风险是 **runtime wiring**，不是 barrier 数学本身。

## Scope Lock

### In Scope

- 把 `runIPCSim` 从 shell-only 扩展为 `shell / tet / cubic` 三模态 unified IPC self-contact 入口。
- 在 tet/cubic 模式下复用现有：
  - `RunSim::parseVolumeMeshInputConfig(...)`
  - `RunSim::resolveRunSimPaths(...)`
  - `RunSim::loadValidatedVolumeMesh(...)`
  - `RunSim::initializeVolumetricSimulation(...)`
  - `InterpolationCoordinates::BarycentricCoordinates::generateInterpolationMatrix()`
- 在 tet/cubic 模式下创建 `EmbeddedSurfaceIPCPotentialEnergy(surfaceV, surfaceF, W, ipcParams)`，并通过 `addGeneralImplicitForceModel(...)` 接入 `ImplicitBackwardEulerTimeIntegrator`。
- 让输出表面 `retXXXX.obj` 固定来自：
  - `psurf = surfaceRestPositions + W * u` for tet/cubic
  - `psurf = surfaceRestPositions + u` for shell
- 为 tet/cubic 增加 `runIPCSim` CLI smoke test。
- 为 cubic 增加一个明确面向 unified IPC path 的 consistency gate，证明 `runIPCSim` 使用的 contact embedding 与当前 repo 既有 cubic surface embedding 语义一致。

### Out Of Scope

- 不实现 `Phase 1.5` 的 inversion-free / material-feasible max step。
- 不实现 external contact。
- 不实现 friction。
- 不改写 `runSim` 的 legacy self-contact 主逻辑。
- 不把 `runIPCSim` 切到 `TRBDF2`。
- 不在本批引入 broad phase / spatial hash。
- 不把 shell path 再次大改成另一套结构；shell 只做必要的分支整理。
- 默认不把 `base-config` 一并做进来。

## Completion Standard

`Phase 1D` 合入的标准固定为：

- `runIPCSim` 不再报 “shell-only”，而是能根据 config 正确进入 shell / tet / cubic 三个分支。
- tet/cubic 路径下的 self-contact 不再通过 legacy `TriangleMeshSelfContactHandler`，而是统一通过 `EmbeddedSurfaceIPCPotentialEnergy`。
- `runIPCSim` 的 tet/cubic 最小 smoke case 能完成 `0~1` 步运行并写出 `deformXXXX.u` 与 `retXXXX.obj`。
- cubic 路径有一个明确测试证明：`runIPCSim` 实际使用的 `W` 与 repo 当前 barycentric embedding 语义一致，而不是“理论上应该一致”。
- `runShellSim`、`runSim`、现有 `runSim_gtest` 回归不被破坏。
- `Phase 1D` 的交付同时覆盖：
  - source changes
  - build/test target wiring
  - 可直接运行的 tet/cubic IPC examples

只要以上条件成立，我会把它视为 **Phase 1 的 volume self-contact 接线闭环**；多步稳定性和不翻单元仍留给 `Phase 1.5`。

## Repo Truth This Plan Builds On

当前 repo 已有足够多的现成积木，不需要重做：

- `EmbeddedSurfaceIPCPotentialEnergy` 已支持一般稀疏 `surfaceFromSimulationDispMap`，不是只支持 `W = I`。
- `tests/src/core/contact/embeddedSurfaceIPCPotentialEnergy_gtest.cpp` 已验证：
  - `W = I` 时与 `CIPCPotentialEnergy` 对齐
  - 一般稀疏 `W` 的 `W^T g`、`W^T H W` pull-back 数学
- `tests/src/tools/runSim_gtest.cpp` 已验证：
  - tet/cubic 的 barycentric embedding matrix 构造
  - legacy self/external contact handler 使用的 sample embedding 与 barycentric interpolation 一致
  - `initializeVolumetricSimulation(...)` 的 cubic 主路径可初始化
- `runIPCSim.cpp` 已有 shell 版 restart / dump / attachment / logging 主循环。

因此 `Phase 1D` 的重点不是新建底层能力，而是把这些现成能力接成一条可维护的 volume IPC 主线。

## Key Design Decisions

### 1. `Phase 1D` 只扩 `runIPCSim`，不同时重构 `runSim`

这一批只解决 unified IPC 主入口的 volume path，不顺手把 `runSim` 改写成也共享这条路径。

原因是：

- `runSim` 当前仍承担 legacy self/external/friction 行为；
- `Phase 1D` 的验收目标是新入口能工作，而不是完成 legacy contact 迁移；
- 如果在这一批同时动 `runSim`，scope 会从 “接线” 变成 “迁移”。

结论固定为：

- `runIPCSim` 扩展到 tet/cubic；
- `runSim` 继续保留 legacy path，只把它当作 repo-truth 参考与测试基线。

### 2. volume path 继续只支持 `ImplicitBackwardEuler`

`runIPCSim` 当前使用 `ImplicitBackwardEulerTimeIntegrator`，这与 `phase1BC` 一致。

本批不引入 `TRBDF2` 的原因是：

- `Phase 1D` 还没有 friction，也没有 stage-aware state；
- volume IPC 首要风险是 contact pull-back 与 runtime wiring；
- `TRBDF2` 的真正复杂度会在 friction/stage context 阶段才显现。

因此本批固定为：

- shell / tet / cubic 统一先跑 `ImplicitBackwardEuler`
- `TRBDF2` 保持 out of scope

### 3. tet/cubic 的 `W` 直接复用 barycentric embedding，不新增 cubic-special 分支

`Phase 1D` 明确坚持 source plan 的主线：

- `tet` 通过 barycentric interpolation 构造 `W`
- `cubic` 也通过完全同一条 barycentric interpolation 路径构造 `W`
- `EmbeddedSurfaceIPCPotentialEnergy` 看不到 mesh type，只接收 `W`

这一步里不允许：

- 为 cubic 单独实现 contact primitive
- 为 cubic 单独实现 surface sample 映射类
- 在 adapter 里加 `if (cubic) ... else ...` 的 pull-back 分支

如果 cubic 有特殊性，只允许体现在：

- volumetric mesh 类型判定
- `initializeVolumetricSimulation(...)` 返回的 FEM/runtime 结构
- barycentric embedding 的 arity 为 `8`

### 4. embedding 验证边界：adapter 只做 shape check，builder 做轻量 sanity，强语义由测试 gate 保证

`EmbeddedSurfaceIPCPotentialEnergy` 在 `Phase 1D` 中继续保持 generic adapter 定位：

- 它消费的输入是已经构造好的 `surfaceFromSimulationDispMap = W`
- 它负责把 `u_sim` 映到 `u_surf = W u_sim`
- 它不负责重新证明 `W` 的 barycentric 来源、tet/cubic arity 语义或与体网格拓扑的一致性

因此 runtime 检查边界固定为三层：

1. adapter constructor / call-site 只保留基础 shape/size 检查：
   - `W.rows() == 3 * numSurfaceVertices`
   - `W.cols() > 0`
   - `u_sim.size() == W.cols()`
2. `buildVolumeIpcSimulation(...)` 补 volume wiring 相关的廉价 sanity check：
   - `W.rows() == 3 * surfaceMesh.numVertices()`
   - `W.cols() == restPosition.size()`
   - `W.nonZeros() > 0`
   - `W.cols() == 3 * volumetricMesh->getNumVertices()`
3. 强语义检查继续放在测试层，而不上升到 adapter runtime validator：
   - tet/cubic 的 `W` 必须与 repo-truth barycentric embedding 逐元素一致
   - 这项一致性由 `phase1D` 的测试 gate 保证

这样做的原因固定为：

- adapter 的职责是消费 `W`，不是复刻 `BarycentricCoordinates` 的构造逻辑
- runtime wiring 需要被 assert，但不值得把 barycentric 合法性恢复逻辑塞进 contact adapter
- repo 现有 legacy handler 也采用类似边界：runtime 检查 embedding 数组结构，强语义由测试对齐 barycentric 基线

### 5. shell 与 volume 共享一个 time-stepping 外壳，但 setup 分成两个 helper 分支

当前 `runIPCSim.cpp` 的 shell-only 版本把 parsing、setup、主循环写在一个文件里。

到 `Phase 1D`，我打算把它整理成：

- `buildShellIpcSimulation(...)`
- `buildVolumeIpcSimulation(...)`

两个匿名命名空间 helper，返回一个统一的运行时描述结构，例如：

```cpp
struct IpcSimulationContext
{
  EigenSupport::SpMatD M;
  EigenSupport::VXd simulationRestPosition;
  EigenSupport::VXd surfaceRestPositions;
  EigenSupport::SpMatD surfaceFromSimulationDispMap;
  std::shared_ptr<SolidDeformationModel::DeformationModelEnergy> elasticEnergy;
  std::vector<std::shared_ptr<ConstraintPotentialEnergies::MultipleVertexPulling>> pullingEnergies;
  std::vector<EigenSupport::VXd> pullingTargets;
  std::vector<EigenSupport::VXd> pullingTargetRests;
  pgo::Mesh::TriMeshGeo surfaceMesh;
};
```

关键点不是结构名字，而是边界：

- shell 分支负责生成 `W = I`
- volume 分支负责生成一般稀疏 `W`
- 主时间推进循环只消费统一后的 `context`
- `simulationRestPosition` 明确表示 simulation-space rest positions
- `surfaceRestPositions` 明确表示 surface-space rest positions

这样既避免 `runIPCSim.cpp` 继续长成第二个 `runSim.cpp`，也不需要在本批额外抽出新的共享库文件。

### 6. `M` 的构造语义按 mesh family 保持 repo-truth，不为“共用外壳”强行统一

`Phase 1D` 虽然会让 shell 和 volume 共用一套 time-stepping 外壳，但 `IpcSimulationContext::M` 的物理来源不被视为同一种东西。

这一点固定按当前 repo-truth 处理：

- shell：
  - 继续使用 `libiglInterface::computeMassMatrix(surfaceMesh, M, 1, 1)`
  - 继续保留当前 `M *= 100` 的 shell-only 硬编码后处理
- volume：
  - 继续复用 `runSim.cpp` 的 volumetric mass matrix 构造语义
  - 使用 `VolumetricMeshes::GenerateMassMatrix::computeMassMatrix(...)`
  - 不额外乘 shell 的 `*100` 常数

这里的设计目标不是“把 shell 和 volume 质量模型统一”，而是：

- 让 `Phase 1D` 聚焦 runtime wiring
- 不在接线阶段顺手引入新的 density / mass-parameterization 议题
- 保持 shell 数值行为与现有 `runIPCSim` 一致
- 保持 tet/cubic 数值行为与现有 `runSim` volume path 一致

因此 `IpcSimulationContext::M` 在代码形状上是统一字段，但在构造语义上必须按 mesh family 分开理解。实现期不允许：

- 把 shell 的 `*100` 魔数传播到 tet/cubic
- 借 `Phase 1D` 顺手改写 shell 的质量建模语义
- 为了“外壳统一”而新增一套折中的 mass 公式

### 7. volume material contract 复用现有 `runSim` 语义，不在本批新设计参数层

`Phase 1D` 的 tet/cubic path 不是一个新的 volume FEM/material 入口；它只是把 unified IPC self-contact 接到现有 volume simulation 语义上。

因此 volume material 决策固定为：

- `elastic-material` 继续沿用同一个 config key，但其允许值按 mesh family 分开解释
- `runIPCSim` 的 tet/cubic path 直接复用当前 `runSim` volume path 的 material contract
- `elastic-material` 只支持当前 volume path 已支持的：
  - `stable-neo`
  - `stvk-vol`
- `koiter-stvk` 继续视为 shell-only，不允许用于 tet/cubic
- `buildVolumeIpcSimulation(...)` 直接调用
  `initializeVolumetricSimulation(volumetricMesh, elasticMat, plasticMat)`
  所代表的既有 repo-truth
- `Phase 1D` 不新增 volume `(E, nu)` config、thickness/bending 参数或新的 material parameterization 语义
- `Phase 1D` 也不把 volume plastic material 暴露为新 config；继续使用
  `initializeVolumetricSimulation(...)` 当前默认的
  `DeformationModelPlasticMaterial::VOLUMETRIC_DOF6`

这样做的目标固定为：

- 让 `Slice B` 保持 implementation-ready
- 把 `Phase 1D` 的工作面收敛在 IPC runtime wiring
- 避免在接线阶段顺手引入一套新的 volume material 配置设计
- 保持 tet/cubic 数值行为尽量贴近当前 `runSim.cpp` 的 repo-truth

对应地，volume path 的错误处理也要写清楚：

- 若 tet/cubic config 写了 `elastic-material = koiter-stvk`，则显式报错
- 若写了当前 volume path 不支持的其它 material 名，也显式报错
- 不允许对 shell-only material 做 silent fallback 或隐式映射
- 推荐错误信息直接指出 mode-specific contract，例如：
  - `runIPCSim phase1D tet/cubic modes only support elastic-material = stable-neo or stvk-vol; koiter-stvk is shell-only.`

### 8. `scale` 语义对 shell 与 volume 分开处理；volume 复用现有 `runSim` 语义

`Phase 1D` 必须明确区分：

- shell-only `runIPCSim` 首版中的 `scale ~= 1` 保守限制
- volume path 在现有 `runSim` 中已经成立的 non-unit scale 语义

因此这一批固定采用：

- volume path：
  - 复用当前 `runSim.cpp` 的 `scale` 语义
  - 允许 `scale != 1`
  - 对 volumetric mesh 与 surface mesh 应用同一个 `scale`
  - `surfaceRestPositions`、barycentric `W`、mass matrix 与输出表面都建立在缩放后的几何上
- shell path：
  - 是否继续保留当前 `scale ~= 1` guard，作为独立决策处理
  - `Phase 1D` 默认不借 volume 接线顺手改写 shell scale 语义

实现边界也固定为：

- 不允许在 `runIPCSim` 的统一入口、mesh-type dispatch 之前放置全局 unit-scale assert
- 若 shell 继续保留 `scale ~= 1` 限制，则该 guard 只能留在 shell 分支
- volume helper 不允许继承 shell 的 `PGO_ALOG(std::abs(scale - 1.0) < 1e-6)` 语义

这样做的目标固定为：

- 让 tet/cubic 保持与当前 `runSim` volume preprocessing 一致
- 不让 shell-only 的首版限制污染 volume path
- 避免 `Phase 1D` 额外扩大成 “volume IPC 接线 + shell scale 语义重构”

### 9. `init-vel` / `init-disp` 固定解释为 simulation-space 初值，而不是 surface-only 条件

在 unified IPC path 中，`EmbeddedSurfaceIPCPotentialEnergy` 消费的是 simulation displacement `u_sim`，surface state 只是通过 `W u_sim` 派生出来的。

因此 `Phase 1D` 中：

- `init-vel`
- `init-disp`

都固定解释为 simulation-space generalized coordinate 的初始条件，而不是 surface-space 条件。

对应约束固定为：

- shell / tet / cubic 都按同一语义处理
- `initialVel` 继续沿用当前 `runIPCSim` 的处理方式，广播到所有 simulation vertices
- 对 tet/cubic，这明确意味着 `initialVel` 作用于所有 volume vertices，而不只是 surface vertices
- `init-disp` 在 `Phase 1D` 中继续保持 zero-only guard
- 本批不开放非零初始位移，也不新设计 volume/surface 间的初始位移映射语义

这样做的目标固定为：

- 保持 time integrator 状态语义与 `u_sim` 一致
- 避免在 `Phase 1D` 顺手扩出一整套新的初值解释规则
- 让 volume IPC 接线与 shell unified path 保持同一套 generalized-coordinate 语义

实现期不允许：

- 把 `initialVel` 解释成 surface-only velocity
- 对 tet/cubic 只给表面顶点赋初速度
- 在 `Phase 1D` 中 silently 放开非零 `init-disp`

### 10. `fixed-vertices` 始终解释为 simulation vertex indices；tet/cubic 不得误用 surface indices

`fixed-vertices` 在 unified IPC path 中始终用于构造 simulation-space 的 attachment / pulling constraints，因此它的索引域固定是：

- simulation mesh vertex indices

而不是：

- contact surface mesh vertex indices

这条约束在三类 mesh family 下的具体含义固定为：

- shell：
  - 由于 `sim vertex == surface vertex`，`fixed-vertices` 在数值上与 surface index 重合
  - 但语义上它仍然是 simulation vertex index
- tet/cubic：
  - `fixed-vertices` 必须是 volume mesh vertex indices
  - 不允许把 `surface-mesh` 的顶点编号直接当成 tet/cubic 的 `fixed-vertices`

这样写死的原因固定为：

- `fixed-vertices` 最终作用在 simulation-space `restPosition` / `u_sim` 上
- volume path 同时存在 simulation vertices 与 surface vertices 两套索引空间
- 若不显式区分，最容易出现“拿 shell/surface 的 fixed.txt 去喂 tet/cubic config”的误用

与此同时，`Phase 1D` 对 `fixed-vertices` 的**约束实现语义**也固定保持 repo-truth：

- 继续通过 `ConstraintPotentialEnergies::MultipleVertexPulling` 接入 soft attachment / pulling energy
- 继续使用现有 `movement` 与 `coeff` 配置语义
- 不在本批切到 `TimeIntegrator::setFixedVertices(...)` 的 hard fixing / fixed-DOF elimination 路径

这样做的原因固定为：

- 当前 `runSim` / `runIPCSim` 的 `fixed-vertices` driver 语义就是 soft pulling
- `Phase 1D` 的目标是 IPC runtime wiring，不是边界条件语义迁移
- 若改成 hard fixing，则会同时改变 `movement` / `coeff` 的解释和 case 数值行为

因此 `Phase 1D` 的配置与样例约束固定为：

- plan 文本必须明确这条索引域规则
- tet/cubic example config 必须在注释或配套说明中写明：
  `fixed-vertices` 文件中的编号是 volume mesh vertices，不是 surface-mesh vertices
- shell example 也应注明它之所以看起来像 surface index，只是因为 shell 下两套顶点空间退化重合

实现期不允许：

- 在 tet/cubic path 中把 `fixed-vertices` 解释成 surface vertex indices
- 在文档或 example 中使用容易让人误以为是 surface index 的模糊措辞
- 在 `Phase 1D` 中把 `fixed-vertices` 的实现从 `MultipleVertexPulling` 静默迁移为 hard fixing / DOF elimination

### 11. output / restart 语义固定以 simulation displacement `u` 为单一状态

`EmbeddedSurfaceIPCPotentialEnergy` 仍然以 simulation displacement `u` 为优化变量，因此：

- restart 文件 `deformXXXX.u` 继续保存 `u / uvel / uacc`
- tet/cubic 路径不保存单独的 `u_surf`
- output surface mesh 通过 `W * u` 现算

这点和 shell 的差异只在：

- shell：`W = I`
- volume：`W` 为 barycentric interpolation

不允许在 volume path 里新增一套“surface state 持久化格式”。

### 12. `collisionHandler` 本体采用 simulation-lifetime；每帧只重注册，不重建

`Phase 1D` 中的 unified IPC collision handler 指的是：

- `std::shared_ptr<Contact::CIPC::EmbeddedSurfaceIPCPotentialEnergy> collisionHandler`

它的生命周期固定覆盖整个 simulation，而不是 per-frame 临时对象。

因此 driver 语义固定为：

- 在 shell/tet/cubic setup 完成后构造一次 `collisionHandler`
- 每帧若继续沿用当前 driver 结构调用
  `clearGeneralImplicitForceModel()`
  ，则只重新执行
  `addGeneralImplicitForceModel(collisionHandler, ...)`
- 不允许每帧重新 `new EmbeddedSurfaceIPCPotentialEnergy(...)`

这样做的原因固定为：

- `EmbeddedSurfaceIPCPotentialEnergy` 持有的 surface mesh、`surfaceRestPositions`、`W` 与 IPC 参数都属于静态配置
- 当前几何状态由每次 `func/gradient/hessianDirect/computeMaxStepSize` 调用时传入的 `u_sim` 决定
- 因此 frame-local 的是 integrator 对 general force model 的注册关系，而不是 handler 本体
- 这一语义也与当前 shell unified IPC 路径保持一致

实现期不允许：

- 因为每帧 `clearGeneralImplicitForceModel()` 就把 `collisionHandler` 也设计成 per-frame 对象
- 在 tet/cubic path 中重复做静态 mesh / embedding / rest-state 初始化

### 13. `ipc-heuristic` 保持 shell-only；volume path 显式拒绝 `ipc-heuristic=true`

这是我在本批里主动做的一个保守收窄，也是为了避免 config 契约出现含糊 precedence 规则。

当前 shell `ipc-heuristic` 使用了明显带 shell 假设的参数来源；直接把它原样搬到 tet/cubic 并不稳妥。为避免在 `Phase 1D` 把 attention 从“接线正确”分散到“heuristic 是否合理”，本计划先采用：

- shell：保持现有 heuristic 行为
- tet/cubic：首版要求显式提供 `ipc-dhat` 与 `ipc-kappa`
- tet/cubic：显式拒绝 `ipc-heuristic=true`

对应的 volume-path 错误处理固定为：

- 若 tet/cubic config 中出现 `ipc-heuristic=true`，直接报错
- 即使同时提供了显式 `ipc-dhat` / `ipc-kappa`，也仍然报错
- 不做 silent fallback
- 不做 “heuristic 与 explicit 参数谁优先” 的隐式 precedence 规则

这样做的目标固定为：

- 保持 shell heuristic 与 volume wiring 两个议题解耦
- 避免把 shell thickness 假设偷偷传播到 tet/cubic
- 让用户从错误信息中立即知道 volume path 需要显式 `ipc-dhat` / `ipc-kappa`

## Planned Code Changes

### Slice A. Generalize `runIPCSim` mesh-type dispatch

目标文件：

- `src/tools/runSim/runIPCSim.cpp`

具体动作：

- 删除当前的 shell-only guard：
  - `if (jconfig.exist("tet-mesh") || jconfig.exist("cubic-mesh")) throw ...`
- 改为统一判定：
  - `tet-mesh` 存在 -> volume tet
  - `cubic-mesh` 存在 -> volume cubic
  - 两者都不存在且 `surface-mesh` 存在 -> shell
  - 两者同时存在 -> config error
- 保留 phase1BC 已有的 out-of-scope 拒绝：
  - `external-objects`
  - friction 相关功能接线

本 slice 的结果应该是：`runIPCSim` 的入口语义终于与总计划 2.3 节对齐，而不是继续写死 shell-only。

### Slice B. Add volume setup path backed by existing helpers

目标文件：

- `src/tools/runSim/runIPCSim.cpp`

必要时可少量补 helper include：

- `runSimVolumeMeshIO.h`
- `runSimFEMSetup.h`
- `barycentricCoordinates.h`
- `generateMassMatrix.h`

具体动作：

- 读取并 scale volumetric mesh
- 读取并 scale surface mesh
- flatten `surfaceRestPositions`
- 构造 barycentric coordinates 与 `W`
- 读取 `init-vel` / `init-disp`，并保持 unified initial-state 语义：
  - `initialVel` 广播到所有 simulation vertices
  - `init-disp` 继续走 zero-only guard
- 读取 `fixed-vertices`，并保持 unified index-domain 语义：
  - shell 下读取的是 shell simulation vertex indices
  - tet/cubic 下读取的是 volume mesh vertex indices
  - 不把 surface-mesh vertex indices 当作 tet/cubic attachment indices
- 处理 IPC 参数时保持 mode-specific heuristic 语义：
  - shell 分支可继续支持 `ipc-heuristic=true`
  - tet/cubic 分支若看到 `ipc-heuristic=true` 则显式报错
  - tet/cubic 分支要求显式 `ipc-dhat` 与 `ipc-kappa`
- 读取并校验 volume `elastic-material`：
  - 只接受 `stable-neo`
  - 只接受 `stvk-vol`
  - 显式拒绝 `koiter-stvk`
  - 报错信息必须直接告诉用户 tet/cubic 当前允许的 material 列表
- 通过 `initializeVolumetricSimulation(...)` 获取：
  - `simMesh`
  - `assembler`
  - `elasticEnergy`
  - `simulationRestPosition`
- 用 volumetric mass matrix 初始化 `ImplicitBackwardEulerTimeIntegrator`
- 在 helper 末尾补 wiring sanity check：
  - `W.rows() == 3 * surfaceMesh.numVertices()`
  - `W.cols() == simulationRestPosition.size()`
  - `W.cols() == 3 * volumetricMesh->getNumVertices()`
  - `W.nonZeros() > 0`

mass matrix 语义在本 slice 中也必须显式保持：

- shell helper 延续当前 `surface mass matrix + *100`
- volume helper 延续当前 `GenerateMassMatrix::computeMassMatrix(...)`
- volume path 不引入额外乘法常数

material 语义在本 slice 中同样必须显式保持：

- tet/cubic helper 直接复用 `runSim` volume path 的 material contract
- 不在本批新读 volume `(E, nu)` 或其它 material 参数字段
- `initializeVolumetricSimulation(...)` 的默认 `VOLUMETRIC_DOF6` plastic path 保持不变
- shell/volume 虽共用 `elastic-material` 这个 config key，但实现时必须按 mode-specific 支持列表分别校验，不允许混用

scale 语义在本 slice 中也必须显式保持：

- volume helper 允许 `scale != 1`
- volumetric mesh 与 surface mesh 必须使用同一个 `scale`
- volume path 不允许继承 shell 的 unit-scale guard

initial-state 语义在本 slice 中也必须显式保持：

- `initialVel` 作用域是 simulation vertices，不是 surface vertices
- tet/cubic path 不允许只给表面顶点赋初速度
- `init-disp` 继续保持 zero-only guard

fixed-vertex 语义在本 slice 中也必须显式保持：

- `fixed-vertices` 的索引域始终是 simulation vertices
- tet/cubic helper 不允许把 surface vertex indices 直接喂给 pulling constraints
- example config / 配套说明必须显式标注 tet/cubic 的 fixed index 来自 volume mesh

IPC 参数语义在本 slice 中也必须显式保持：

- shell helper 与 volume helper 对 `ipc-heuristic` 的支持范围分开处理
- volume helper 不允许接受 `ipc-heuristic=true`
- volume helper 不允许在 heuristic 与 explicit 参数之间做 silent precedence

这一块的核心要求是：

- shell 和 volume path 在 contact 接口上统一
- 但 volume 的 FEM/mass/rest-position 初始化不重复造轮子

### Slice C. Reuse `EmbeddedSurfaceIPCPotentialEnergy` unchanged for tet/cubic

目标文件：

- `src/core/contact/embeddedSurfaceIPCPotentialEnergy.h`
- `src/core/contact/embeddedSurfaceIPCPotentialEnergy.cpp`
- `tests/src/core/contact/embeddedSurfaceIPCPotentialEnergy_gtest.cpp`

预期策略：

- 正常情况下不改 adapter public API
- 只在测试层补 volume-oriented coverage
- 不把 tet/cubic barycentric 语义验证塞进 adapter runtime，只保留现有基础维度检查

要验证的点：

- adapter 对非方阵 `W` 的 DOF 规模仍然正确
- tet/cubic 场景下 `getNumDOFs() == 3 * numVolumeVertices`
- `computeMaxStepSize(u, du)` 仍只做 contact-side pull-forward / pull-back，不偷偷掺入 material feasibility

换句话说，本 slice 的目标是证明 adapter 已经“够用”，而不是再继续扩它的接口。

### Slice D. Unify output surface generation and restart semantics

目标文件：

- `src/tools/runSim/runIPCSim.cpp`

具体动作：

- 主循环中统一维护 `u / uvel / uacc`
- shell path 输出使用 `surfaceRestPositions + u`
- volume path 输出使用 `surfaceRestPositions + W * u`
- restart 仍从 `deformXXXX.u` 恢复 simulation-space state

额外约束：

- 不新增 `surface-deformXXXX.*` 一类的并行输出格式
- 不在 volume path 保存 surface-only restart state
- `frameGap`、`num-timestep = 0/1` 的边界语义保持与 phase1BC 一致

### Slice E. Add volume IPC example configs

目标文件：

- `examples/ipc/tet/box-hang/box-ipc.json`
- `examples/ipc/cubic/box-hang/box-ipc.json`

或者若你更希望贴近旧目录，也可以放在：

- `examples/box/box-ipc.json`
- `examples/cubic/box/box-ipc.json`

我当前更倾向于前者，因为它更清楚地表达“这是 unified IPC 入口示例，不是 legacy runSim config”。

`Phase 1D` 默认采用独立 IPC example 目录：

- `examples/ipc/tet`
- `examples/ipc/cubic`

而不是把新 config 混放回 legacy case 目录。

配置策略先保持保守：

- 不依赖 `base-config`
- 明确给出 `ipc-dhat`
- 明确给出 `ipc-kappa`
- `external-objects` 先不放进去
- 必须显式说明 `fixed-vertices` 的索引域：
  - shell example: simulation vertex indices（shell 下数值上等于 surface vertex index）
  - tet/cubic example: volume mesh vertex indices, not surface-mesh vertex indices
- tet example 默认将所需 asset 复制到 `examples/ipc/tet` 目录下，使该目录自洽包含：
  - `box.veg`
  - `box.obj`
  - 对应的 fixed-vertex asset
- cubic example 默认将所需 asset 复制到 `examples/ipc/cubic` 目录下，使该目录自洽包含：
  - `box.veg`
  - `box.obj`
  - 对应的 fixed-vertex asset
- `Phase 1D` 不采用跨目录相对引用 legacy case asset 的做法；新的 IPC example 目录应当可以独立移动/运行而不依赖外部 mesh 路径

### Slice F. Add tests that lock the volume unified path

目标文件：

- `tests/src/tools/runIPCSim_gtest.cpp`
- `tests/src/tools/runSim_gtest.cpp`
- 视需要补少量 `tests/src/core/contact/embeddedSurfaceIPCPotentialEnergy_gtest.cpp`

我计划新增的测试分成三层：

1. `runIPCSim` tet CLI smoke
   - `num-timestep=0` case:
     - 断言进程成功
     - 断言输出目录存在
     - 断言不会生成 `deform0000.u`
     - 断言不会生成 `ret0000.obj`
   - `num-timestep=1` case:
     - fixture 固定使用 `dump-interval=1`
     - 断言进程成功
     - 断言会生成 `deform0000.u`
     - 断言会生成 `ret0000.obj`

2. `runIPCSim` cubic CLI smoke
   - `num-timestep=0` case:
     - 断言进程成功
     - 断言输出目录存在
     - 断言不会生成 `deform0000.u`
     - 断言不会生成 `ret0000.obj`
   - `num-timestep=1` case:
     - fixture 固定使用 `dump-interval=1`
     - 断言进程成功
     - 断言会生成 `deform0000.u`
     - 断言会生成 `ret0000.obj`
   - 重点确认 cubic path 不会因为 `W` 的 8 点 embedding 或 FEM 初始化失败

3. multi-step output persistence check
   - 用 `num-timestep=2` 或更小的最小多帧 case
   - 断言 `deform0000.u` 与 `deform0001.u` 都被写出
   - 断言 `retXXXX.obj` 是否写出严格受 `dump-interval` 控制，而不是“每步必写”

4. cubic consistency gate
   - gate 输入固定为真实 CLI example：
    `examples/ipc/cubic/box-hang/box-ipc.json`
   - 断言对象固定为 `buildVolumeIpcSimulation(...)` 实际返回的
     `context.surfaceFromSimulationDispMap`
  - baseline 固定从同一个 `examples/ipc/cubic/box-hang/box-ipc.json` 解析得到的
     `cubic-mesh` / `surface-mesh` 显式构造：
     `BarycentricCoordinates bc(...); auto expected = bc.generateInterpolationMatrix();`
   - 它必须与该 `expected` 逐元素一致
   - 这项测试卡的是 “`runIPCSim` 从 CLI config 到 runtime context 的 wiring 不出错”，不是复用 legacy `cubicConfigPath()` 或重复证明同一条 helper 自己调用自己的数学
   - 测试必须直接复用生产实现返回的同一 `IpcSimulationContext`；
     不允许为了测试再单独重建一条只生成 embedding matrix 的平行 setup 路径

5. tet consistency gate
   - 与 cubic 同理
   - gate 输入固定为真实 CLI example：
    `examples/ipc/tet/box-hang/box-ipc.json`
   - baseline 同样从该 example config 解析出的 `tet-mesh` / `surface-mesh`
     显式构造 `BarycentricCoordinates` 后得到
   - 断言 `buildVolumeIpcSimulation(...)` 返回的
     `context.surfaceFromSimulationDispMap`
     与该 `expected` 逐元素一致
   - 同样要求测试复用生产 builder 返回的 runtime context，而不是单独拼一份 test-only `W`

其中第 4/5 点是 `Phase 1D` 的质量门，而不是“可选增强”。

### Slice G. Build/Test/Example Checklist

这部分不是可选收尾工作，而是 `Phase 1D` 完成定义的一部分。

必须逐项核对：

- `src/tools/runSim/CMakeLists.txt`
  - 若 `runIPCSim` 为了 volume path 新增 helper 源文件或拆分实现，必须显式更新 target
- `tests/src/tools/CMakeLists.txt`
  - `runIPCSim_gtest` 若新增 test helper / 源文件，必须显式更新 target
- `tests/src/core/contact/CMakeLists.txt`
  - 若 `embeddedSurfaceIPCPotentialEnergy_gtest` 扩展了依赖，也必须同步更新 target
- example configs
  - 至少新增一份 tet IPC config
  - 至少新增一份 cubic IPC config
  - 明确哪些 volumetric/surface mesh asset 与 fixed-vertex asset 需要复制到新的 IPC example 目录
- validation commands
  - 必须在 plan closeout 中保留对应的 build target 列表
  - 必须在 plan closeout 中保留对应的 `ctest` 过滤表达式
  - 必须在 plan closeout 中保留 tet/cubic example 的 CLI smoke 命令

实现期不允许：

- 只改源文件而不更新对应 target
- 只靠测试内联临时 config，而不提供 repo 内可直接运行的 tet/cubic IPC example
- 把 CMake/example 更新当作“最后顺手收尾”的可选工作

## Validation Plan

我预计本批的验证命令会收敛到：

```bash
cmake --build --preset base_no_mkl_debug --target \
  embeddedSurfaceIPCPotentialEnergy_gtest \
  runIPCSim_gtest \
  runSim_gtest

ctest --test-dir build/base_no_mkl_debug --output-on-failure -R \
  "EmbeddedSurfaceIPCPotentialEnergy|RunIPCSim|RunSimVolumeMeshIO"

build/base_no_mkl_debug/bin/runIPCSim examples/ipc/tet/box-hang/box-ipc.json --log
build/base_no_mkl_debug/bin/runIPCSim examples/ipc/cubic/box-hang/box-ipc.json --log
```

如果实现中为了减少重复提取了新的 helper，我会把对应单测一起加入 build target，但不会把验证范围扩到 external/friction 相关测试。

## Risks And Mitigations

### Risk 1. `runIPCSim.cpp` 迅速复制成第二个 `runSim.cpp`

Mitigation:

- 只引入 shell setup / volume setup 两个 helper 分支
- 不在本批把 legacy external/self/friction 逻辑搬进来
- 主循环只保留 unified IPC 所需状态

### Risk 2. cubic 路径“能跑”但实际没有锁住 embedding 语义

Mitigation:

- 把 cubic consistency test 作为合入门槛
- 用新的 IPC example config 做 gate 输入，并从同一 config 显式构造 barycentric baseline，而不是挂在 legacy `cubicConfigPath()` 上

### Risk 3. `ipc-heuristic` 在 volume path 上引入额外调参争议

Mitigation:

- 先把 tet/cubic 的 heuristic 排除在本批之外
- volume path 首版要求显式 `ipc-dhat / ipc-kappa`

### Risk 4. 用户误以为 `Phase 1D` 已包含 inversion-free

Mitigation:

- 文档与错误信息中都继续明确：
  - 当前 max step 仍只有 contact-feasible half
  - material-feasible half 属于 `Phase 1.5`

## Intentional Narrowing From Source Plan

相对总计划 `ipc_friction.plan.md`，这份 `Phase 1D` 文档做了三点有意收窄：

- 把工作面聚焦到 `runIPCSim` volume self-contact 接线，不顺带迁移 `runSim`
- 默认不把 `base-config` 做进这一批
- 默认不把 tet/cubic 的 `ipc-heuristic` 做进这一批

我这样收窄的原因很简单：`Phase 1D` 的关键风险在 wiring correctness，不在 config sugar。
