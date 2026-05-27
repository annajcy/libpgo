# Phase 1BC Plan: `EmbeddedSurfaceIPCPotentialEnergy` + `runIPCSim` shell 闭环

Source plan: `/Users/jinceyang/Desktop/codebase/libpgo/plan/ipc/ipc_friction.plan.md`  
Depends on: `/Users/jinceyang/Desktop/codebase/libpgo/plan/ipc/phase1A.plan.md`  
Repo-truth reference: `/Users/jinceyang/Desktop/codebase/libpgo/plan/ipc/phase1A.impl.md`

## Summary

这一批不按原始拆分把 `1B` 和 `1C` 分开落，而是合并成一个可验证交付：

- `1B` 的 simulation-space adapter:
  `EmbeddedSurfaceIPCPotentialEnergy`
- `1C` 的 shell unified entry:
  `runIPCSim` shell mode

合并后的目标很明确：

- 不只把 `SurfaceIPCCore` 再包一层；
- 而是交付一条完整可运行的 shell Embedded Surface IPC 主链路；
- 让 repo 第一次可以在不走 legacy `runShellSim + CIPCPotentialEnergy` 主路径的前提下，直接用 unified IPC 入口跑 shell self-contact。

这一批完成后，仓库里会同时存在两条 shell self-contact 路径：

1. `runShellSim + CIPCPotentialEnergy`
   继续作为 legacy/reference 路径保留；
2. `runIPCSim + EmbeddedSurfaceIPCPotentialEnergy`
   作为 unified IPC 主路径的第一条真实交付。

## Why Merge 1B And 1C

只做 adapter 而不做 shell 入口，会留下两个问题：

- 数学 pull-back 虽然写出来了，但没有真实 driver 去验证它在积分器里的接线；
- Phase 2/3 之后真正要复用的对象不是“一个单独的 adapter 类”，而是一条从 config 到 time integrator 都可跑通的 unified IPC pipeline。

因此本批固定采用：

- 文档名固定为 `phase1BC.plan.md`
- 但范围明确是 `1B + 1C merged-for-shell`

这样做的收益是：

- 可以先在最简单的 `W = I` 场景下把 simulation-space adapter、profiling、CLI/config、time integrator 接线一起验证；
- 后续 tet/cubic 扩展时，新增的是 embedding `W` 和 volume mesh preprocessing，而不是再去补第一条 driver。

## Scope Lock

### In Scope

- 新增 `EmbeddedSurfaceIPCPotentialEnergy`
- 新增 `runIPCSim.cpp` 和对应 build target
- `runIPCSim` 第一版只要求 shell mode 跑通
- 在 shell mode 下通过 `W = I` 接入 `SurfaceIPCCore`
- 把新 adapter 接到现有 implicit time integrator 的 general implicit model 路径
- 为新 adapter 落地独立单测
- 为 `runIPCSim` shell mode 落地 CLI/config/smoke test
- 复用 Phase 1A 已完成的 profiling infrastructure，并把 adapter 层 section 固定下来

### Out Of Scope

- tet/cubic unified IPC runtime path
- inversion-free max step
- external contact
- friction
- 把 legacy floor energy 迁进 `EmbeddedSurfaceIPCPotentialEnergy` 或 `runIPCSim`
- broad phase / spatial hash
- 改写 `runShellSim` 默认行为
- 让 `runIPCSim` 直接取代现有用户入口

## Completion Standard

这一批的完成标准不是“所有 mesh type 的 unified IPC 都能工作”，而是更窄的一组 shell 闭环标准：

- `EmbeddedSurfaceIPCPotentialEnergy` 已成为一个独立的 simulation-space `PotentialEnergy`
- shell 模式下 `W = I` 时，它和 `CIPCPotentialEnergy` 的数值结果对齐
- `runIPCSim` 能读取 shell IPC config 并完成至少单步或零步 smoke run
- `runIPCSim` shell 路径的 contact energy、gradient、Hessian、`computeMaxStepSize()` 全部接通
- adapter profiling section 已接到真实入口，而不是占位 no-op
- `runShellSim` 和现有 tests 不回归

只要上述条件成立，这一批就可以合入；tet/cubic、material max step、external、friction 全部留到后续批次。

## Repo Truth This Plan Builds On

当前 repo 已有以下前提，可直接复用：

- `SurfaceIPCCore` 已经落地，是唯一的 surface-space IPC 数学实现
- `CIPCPotentialEnergy` 已经是 thin wrapper，不再拥有第二套 contact kernel
- profiling 基础设施已存在，且 `contact.surface.*` 与 `contact.wrapper.*` section 已固定
- `runShellSim` 已经证明 shell 路径的 `PotentialEnergy + time integrator` 主链路可用
- `tests/src/tools/runSim_gtest.cpp` 已经为 tet/cubic embedding preprocessing 建好了回归基线
- `examples/shell/shell.json` 已可作为 shell CLI smoke 的真实输入基础

这意味着本批不需要再重新设计 surface IPC 数学，也不需要重新证明 shell FEM 主流程可行；重点是把 unified IPC adapter 与入口拼起来。

## Key Design Decisions

### 1. `EmbeddedSurfaceIPCPotentialEnergy` 固定只接受 simulation displacement

虽然源计划草稿里写过 `bool is_input_disp`，但本批明确不在新 adapter 上保留这个 legacy 模式开关。

原因是：

- unified IPC 主路径在当前 repo 里优化的广义坐标本来就是位移 `u_sim`
- shell/tet/cubic 三类 solid path 的 time integrator 都围绕位移未知量工作
- `isInputDisp` 只属于 legacy `CIPCPotentialEnergy` 的兼容职责，不应该被带进新主路径

因此新类的语义固定为：

- 输入 `simulationDisplacements` 总是 simulation displacement
- 输入 `trialSimulationDisplacements` 总是 line search / CCD 使用的 simulation-space trial increment
- adapter 内部自行构造 absolute simulation/surface positions

即：

```text
surfaceDisplacements = surfaceFromSimulationDispMap * simulationDisplacements
surfacePositions = surfaceRestPositions + surfaceDisplacements
trialSurfaceDisplacements = surfaceFromSimulationDispMap * trialSimulationDisplacements
```

为避免后续实现里把 `position / displacement / simulation / surface` 混写，本批命名规范固定为：

- `surfaceRestPositions`
- `simulationDisplacements` / `surfaceDisplacements`
- `trialSimulationDisplacements` / `trialSurfaceDisplacements`
- `surfaceFromSimulationDispMap`
- `surfaceGradient` / `simulationGradient`
- `surfaceHessian` / `simulationHessian`

文档后续默认使用这套全称命名；`u_sim / du_sim / g_sim / H_sim / W` 只作为公式别名，不再作为首选 API 命名。

### 2. 新 adapter 不复制 `SurfaceIPCCore` 的几何缓存职责

`EmbeddedSurfaceIPCPotentialEnergy` 只负责三件事：

- 维护 simulation-space 到 surface-space 的映射数据
- 调用 `SurfaceIPCCore`
- 把 `g_surf / H_surf` pull back 到 simulation space

它不负责：

- 自己 build PT/EE pairs
- 自己维护第二套 barrier/CCD 逻辑
- 自己决定 active pair identity

single source of truth 仍然固定为 `SurfaceIPCCore`。

### 3. 首版不预埋 `simulationRestPositions_` 这类 future-only state

按本批当前 scope，contact 数学只需要：

- `surfaceRestPositions`
- `surfaceFromSimulationDispMap`
- `simulationDisplacements`
- `trialSimulationDisplacements`

即可构造：

```text
surfacePositions = surfaceRestPositions + surfaceFromSimulationDispMap * simulationDisplacements
trialSurfaceDisplacements = surfaceFromSimulationDispMap * trialSimulationDisplacements
```

因此本批明确不在 `EmbeddedSurfaceIPCPotentialEnergy` 中预留 `simulationRestPositions_` 这类当前不参与任何首版计算的 future-only state。

约束固定为：

- 首版类状态只保留当前 contact 计算真正需要的成员
- 不为了 Phase 1.5 / Phase 2 / Phase 3 的未来需求提前保存 simulation-space absolute rest positions
- 若后续 phase 确实需要该状态，再连同其数学用途、校验规则、与 `surfaceRestPositions_` 的关系一起单独引入

这样做的目的固定为：

- 避免在首版类里保留“看起来重要但当前完全不参与计算”的 dead state
- 降低 API 和对象状态复杂度
- 让 phase1BC 的实现与验收只围绕 shell unified IPC 闭环本身

### 4. shell path 固定走 `W = I`

shell 是 unified IPC 的第一条落地路径，但不允许保留 shell-only 数学分支。

首版规则固定为：

- shell 的 surface rest geometry 直接作为 `surfaceRestVertices`
- shell 的 simulation unknown 与 surface displacement 一一对应，因此 `surfaceFromSimulationDispMap` 为单位稀疏矩阵
- `EmbeddedSurfaceIPCPotentialEnergy` 与未来 tet/cubic 路径使用完全同一个类

这样 shell 路径的意义不是“单独实现一个 shell 版本”，而是先用最简单的 embedding 验证 unified 架构本身。

### 5. 新 adapter 继续采用 `hessianDirect()` 主入口

为了和当前 `CIPCPotentialEnergy`、time integrator helper 的使用方式保持一致，本批固定采用：

- `func()`
- `gradient()`
- `computeMaxStepSize()`
- `hessianDirect()`

作为真实计算入口。

`hessian()` / `createHessian()` 在首版中沿用和 `CIPCPotentialEnergy` 一致的策略：

- 不作为直接调用入口
- 若被误用则抛异常

这样可以避免在首版里为“可变 Hessian sparsity 模板”额外做一层不必要抽象。

此外，首版明确不要求 `mutable SurfaceIPCCore surfaceIPCCore_;` 这类设计。
Phase 1A 中 `mutable core` 的存在，是为了 legacy wrapper 在 `const` 入口里同步 public 参数；而新 adapter 没有这类 wrapper-parameter sync 需求。
因此本批固定采用：

- `surfaceIPCCore_` 为普通成员，不声明为 `mutable`
- 若后续 phase 需要引入跨调用 cache 或 stage-local mutable state，必须在对应 phase 文档中单独声明其原因与边界

### 6. profiling section 在 adapter 层新增一组稳定名字

为与 Phase 1A 中“后续若引入 simulation-space adapter，则优先使用独立前缀，例如 `contact.adapter.*`”的约定保持连续，本批固定采用 `contact.adapter.*` 前缀，而不再使用 `contact.embedded.*`。

本批固定在 `EmbeddedSurfaceIPCPotentialEnergy` 上新增 wrapper-level profiling section，命名为：

- `contact.adapter.func`
- `contact.adapter.gradient`
- `contact.adapter.hessian_direct`
- `contact.adapter.max_step`
- `contact.adapter.map_to_surface`
- `contact.adapter.pullback_gradient`
- `contact.adapter.pullback_hessian`

约束固定为：

- `SurfaceIPCCore` 继续只记录 surface kernel 内部 section
- `func / gradient / hessian_direct / max_step` 为 adapter 顶层 section
- `map_to_surface / pullback_gradient / pullback_hessian` 为内部嵌套 section
- `computeMaxStepSize()` 中 `surfaceFromSimulationDispMap * trialSimulationDisplacements` 这一步计入 `contact.adapter.map_to_surface`
- 不把 adapter 层统计混进 `contact.wrapper.*`

`contact.wrapper.*` 继续保留给 legacy `CIPCPotentialEnergy`。
测试层面的断言规则也固定为：

- profiling 单测只断言对应 `contact.adapter.*` section 的存在
- 不断言某个 section 的独占计时、嵌套占比或精确调用次数

这样可以让 profiling 测试在保留语义约束的同时，尽量避免因实现细节调整而变脆。

### 7. `runIPCSim` 首版只做 IPC path，不引入 legacy contact 开关

`runIPCSim` 的职责固定为：

- 读取 IPC config
- 构造 shell simulation
- 创建 `EmbeddedSurfaceIPCPotentialEnergy`
- 把它挂到 time integrator
- 驱动一个统一 IPC-only 的仿真主循环

首版不支持：

- `"contact-model"` 之类 legacy/IPC 切换开关
- floor penalty compatibility mode
- tet/cubic mode

这样可以让 shell first version 保持边界清晰，避免一开始就把 `runSim` 的 legacy 分支带进新入口。

### 8. legacy floor energy 首版不迁移

本批明确不把 legacy `CIPCPotentialEnergy` 上的 floor energy 带进 unified IPC 主路径。

边界固定为：

- `CIPCPotentialEnergy` 继续保留 floor post-pass，作为 legacy/reference 行为的一部分
- `EmbeddedSurfaceIPCPotentialEnergy` 首版不实现 floor penalty
- `runIPCSim` 首版不提供 floor compatibility 开关

原因固定为：

- floor energy 不属于 embedded-surface IPC core，而是 legacy wrapper 上的额外 special-case
- 本批的主要目标是先验证最干净的 shell unified IPC 闭环，而不是覆盖所有 legacy 兼容项
- 若后续确实需要“地面”语义，优先应在 external obstacle 阶段作为统一 external contact 来建模，而不是继续在新 adapter 上复制一套 floor 特例

因此，本批所有 shell validation 都应优先选择“不依赖 floor penalty 才能成立”的 case。

### 9. `runIPCSim` shell config 首版固定为独立单文件 `shell-ipc.json`

相对总计划里推荐的 `*-ipc.json + base-config` 组织方式，本批主动收窄为更简单的首版策略：

- 新增独立的 unified IPC config：`examples/shell/shell-ipc.json`
- `runIPCSim` 首版只读取这一份 IPC config 文件
- 本批不实现 `base-config` merge helper
- `examples/shell/shell.json` 继续保留为 legacy/reference config，但不参与 `runIPCSim` 首版解析

同时，本批把 IPC 参数策略一并写死：

- `ipc-dhat` 与 `ipc-kappa` 必须显式写在 `shell-ipc.json` 中
- 若 `shell-ipc.json` 缺失 `ipc-dhat` 或 `ipc-kappa`，`runIPCSim` 直接报配置错误并退出

这样做的目的固定为：

- 先把 shell unified IPC 闭环压到最小配置面
- 避免在 `runIPCSim` 首版里引入隐式 heuristic/default
- 避免把实现时间消耗在 `base-config` 继承语义上
- 让 `runIPCSim` 保持边界清晰的 IPC-only 入口

### 10. `eps_ee / slackness` 首版固定走默认值，并打印最终 IPC 参数

本批不扩展 shell IPC config 的参数面；首版只暴露：

- `ipc-dhat`
- `ipc-kappa`

其余 `SurfaceIPCCore::Parameters` 字段固定采用默认值：

- `eps_ee = 0.0`
- `slackness = 1.0`

对应约束固定为：

- `shell-ipc.json` 首版不接受 `ipc-eps-ee` 或 `ipc-slackness` 之类新字段
- `EmbeddedSurfaceIPCPotentialEnergy` 构造 `SurfaceIPCCore::Parameters` 时，显式写入这四个最终值，而不是依赖隐式未初始化状态
- `EmbeddedSurfaceIPCPotentialEnergy` 的 API 可以保留 `ipcParams = {}` 这一默认参数，便于单测和最小构造；但 `runIPCSim` 路径禁止依赖该默认，必须先显式构造并填满四个最终参数值后再传入
- `runIPCSim` 启动时必须打印最终采用的
  - `ipc-dhat`
  - `ipc-kappa`
  - `eps_ee`
  - `slackness`

这样可以同时满足：

- 首版配置面保持最小
- 数值来源清晰可复现
- 后续如果要开放更多 IPC 参数，可以作为单独的小批次演进，而不会与 shell 主链路首版耦合

## Concrete Deliverables

### A. New contact adapter

新增：

- `src/core/contact/embeddedSurfaceIPCPotentialEnergy.h`
- `src/core/contact/embeddedSurfaceIPCPotentialEnergy.cpp`

首版类职责固定为：

- 持有 `SurfaceIPCCore surfaceIPCCore_`
- 持有 `surfaceRestPositions_`
- 持有 `surfaceFromSimulationDispMap_`
- 持有 `std::vector<int> simulationDOFs_`

该类固定采用单阶段初始化，不保留 `setSurfaceMesh(...)` 这类二阶段 setup 接口。
surface rest geometry 的 single source of truth 固定为：

- `surfaceRestVertices`
- `surfaceTriangles`

`surfaceRestPositions_` 由 `surfaceRestVertices` 在构造函数内部 flatten 得到，不再作为第二份外部输入传入。
`simulationDOFs_` 也在构造函数中一次性初始化为：

- `[0, 1, ..., numSimulationDOFs - 1]`
- 其中 `numSimulationDOFs = surfaceFromSimulationDispMap.cols()`

首版不在 adapter 内做 fixed-DOF exclusion、constraint masking 或 config-driven DOF 裁剪；这些继续由 time integrator / constraints 路径负责。

核心接口形状固定为：

```cpp
class EmbeddedSurfaceIPCPotentialEnergy : public NonlinearOptimization::PotentialEnergy
{
public:
  EmbeddedSurfaceIPCPotentialEnergy(
    const EigenSupport::MXd &surfaceRestVertices,
    const EigenSupport::MXi &surfaceTriangles,
    const EigenSupport::SpMatD &surfaceFromSimulationDispMap,
    const SurfaceIPCCore::Parameters &ipcParams = {});

  double func(EigenSupport::ConstRefVecXd simulationDisplacements) const override;
  void gradient(
    EigenSupport::ConstRefVecXd simulationDisplacements,
    EigenSupport::RefVecXd simulationGradient) const override;
  void hessian(EigenSupport::ConstRefVecXd x, EigenSupport::SpMatD &hess) const override;
  void createHessian(EigenSupport::SpMatD &hess) const override;
  void hessianDirect(
    EigenSupport::ConstRefVecXd simulationDisplacements,
    EigenSupport::SpMatD &simulationHessian) const override;
  double computeMaxStepSize(
    EigenSupport::ConstRefVecXd simulationDisplacements,
    EigenSupport::ConstRefVecXd trialSimulationDisplacements) const override;

  void getDOFs(std::vector<int> &dofs) const override;
  int getNumDOFs() const override;
  int isHessianTopologyFixed() const override { return 0; }
};
```

构造函数内部必须完成以下强校验，并在失败时抛 `std::invalid_argument`：

- `surfaceRestVertices.cols() == 3`
- `surfaceRestVertices.rows() > 0`
- `surfaceFromSimulationDispMap.rows() == 3 * surfaceRestVertices.rows()`
- `surfaceFromSimulationDispMap.cols() > 0`
- `surfaceTriangles.cols() == 3`
- `surfaceTriangles` 顶点索引合法

这里的 `surfaceRestVertices` 约定固定为 `N x 3`，与当前 tri-mesh / libigl 相关代码路径的几何矩阵约定保持一致。

对象一旦构造成功，即视为 fully initialized，可直接用于 `func / gradient / hessianDirect / computeMaxStepSize`。

实现要求固定为：

- `func()`:
  `surfacePositions -> surfaceIPCCore_.computeEnergy(surfacePositions)`
- `gradient()`:
  `simulationGradient = surfaceFromSimulationDispMap^T * surfaceGradient`
- `hessianDirect()`:
  `simulationHessian = surfaceFromSimulationDispMap^T * surfaceHessian * surfaceFromSimulationDispMap`
- `computeMaxStepSize()`:
  `contactMaxStep = surfaceIPCCore_.computeMaxStepSize(surfacePositions, trialSurfaceDisplacements)`

### B. Shell helper construction

首版不新增单独的 shell-specialized contact class。

`runIPCSim` 中 shell mode 的构造规则固定为：

- 从 `surface-mesh` 读取 shell mesh
- 直接复用当前 `runShellSim.cpp` 已存在的 shell FEM 组装主链，而不是为 `runIPCSim` 另起一套 shell-only 组装方式；当前 repo-truth 下，这条主链明确包括：
  - `SimulationMesh`
  - `DeformationModelManager`
  - `DeformationModelAssembler`
  - `DeformationModelEnergy`
- `libiglInterface` 在该路径上的职责仍主要是质量矩阵相关处理，不作为单独的 shell elastic energy 主装配入口
- 用 shell surface rest positions作为 `surfaceRestVertices`
- 构造单位 embedding `W = I`
- 创建 `EmbeddedSurfaceIPCPotentialEnergy`
- 通过 `addGeneralImplicitForceModel(...)` 挂到 time integrator

### C. New CLI entry

新增：

- `src/tools/runSim/runIPCSim.cpp`

并在：

- `src/tools/runSim/CMakeLists.txt`

里增加对应 target。

首版 `runIPCSim` 只要求支持：

- 位置参数：`config`
- 可选参数：`--log`

shell 运行所需字段至少包括：

- `surface-mesh`
- `fixed-vertices`
- `g`
- `init-vel`
- `init-disp`
- `scale`
- `timestep`
- `num-timestep`
- `damping-params`
- `sim-type`
- `solver-eps`
- `solver-max-iter`
- `elastic-material`
- `dump-interval`
- `output`
- `ipc-dhat`
- `ipc-kappa`

第一版先固定：

- 只接受 `sim-type == "dynamic"`
- config 中继续保留 `elastic-material` 字段，因为它是当前 shell 路径的真实输入校验项，而不是占位门面字段
- `elastic-material` 首版只接受 shell 路径当前已支持的 `koiter-stvk`；若给出其他值，`runIPCSim` 直接报配置错误并退出
- `ipc-dhat` / `ipc-kappa` 必须出现在 `shell-ipc.json` 中，缺失时 fatal
- `eps_ee = 0.0`、`slackness = 1.0` 固定使用 `SurfaceIPCCore::Parameters` 默认值，不从 config 读取
- 如果配置里出现 `tet-mesh` 或 `cubic-mesh`，明确报“Phase 1B not implemented yet”

### D. Example config

本批同时新增一个 shell IPC config，固定放在：

- `examples/ipc/shell/shell-ipc.json`

首版固定策略：

- `shell-ipc.json` 是 `runIPCSim` 首版唯一读取的 config 文件
- `shell-ipc.json` 是独立单文件，不依赖 `base-config`
- 所有 IPC 专有字段只写在 `shell-ipc.json` 中，不回写 `shell.json`

这份 config 的作用不是长期配置模板，而是：

- 给 `runIPCSim` shell smoke test 提供稳定输入
- 给后续 external/friction 阶段提供统一入口样例

## Tests

### 1. `EmbeddedSurfaceIPCPotentialEnergy` unit test

新增：

- `tests/src/core/contact/embeddedSurfaceIPCPotentialEnergy_gtest.cpp`

建议复用：

- `tests/src/core/contact/testCIPCHelpers.h`

首批测试固定覆盖：

- `W = I` 时，`func / gradient / hessianDirect / computeMaxStepSize`
  与 `CIPCPotentialEnergy(isInputDisp=true)` 对齐
- 非平凡稀疏 `W` 下，验证：
  - `surfacePositions = surfaceRestPositions + surfaceFromSimulationDispMap * simulationDisplacements`
  - `simulationGradient = surfaceFromSimulationDispMap^T * surfaceGradient`
  - `simulationHessian = surfaceFromSimulationDispMap^T * surfaceHessian * surfaceFromSimulationDispMap`
- profiling 开启时，能看到 `contact.adapter.*` section

其中 `W = I` 的对齐测试不是 finite-difference 校验，而是同一 `SurfaceIPCCore` 数学在两层包装下的等价性检查，因此容差应显著严于 Phase 1A 的 FD 容差。首版推荐固定为：

- `func()`:
  绝对误差或相对误差不超过 `1e-10`
- `gradient()`:
  相对误差不超过 `1e-9`
- `hessianDirect()`:
  相对误差不超过 `1e-8`
- `computeMaxStepSize()`:
  绝对误差或相对误差不超过 `1e-10`

若实现中发现稀疏装配顺序导致的 roundoff 仍超过上述量级，必须在测试或实现说明里明确原因；不得无说明地退回到 Phase 1A 那类 `1e-4 / 1e-3` 级别的宽松容差。

profiling 相关测试固定只检查 `contact.adapter.*` section 是否出现，不检查独占计时或嵌套比例。

其中第二类测试不要求真实 tet/cubic mesh；
可以直接构造一个小尺寸、行数为 `3 * nsurf`、列数为 `3 * nsim` 的手工稀疏矩阵来验证 pull-back 数学。

### 2. `runIPCSim` shell CLI test

固定新增：

- `tests/src/tools/runIPCSim_gtest.cpp`

本批不保留“并入 `tests/src/tools/runSim_gtest.cpp`”这一备选方案。
原因固定为：

- `runIPCSim` 测的是新入口，不应继续堆在 legacy `runSim` 名字下
- 独立 test file 与独立 target 能让 validation 命令保持明确，不需要在文档里再写条件分支
- 后续若 `runIPCSim` 扩展到 tet/cubic/external/friction，也更适合在独立 test 文件中继续演进

首批 shell CLI 测试覆盖：

- `--log` 能在 config 同目录生成 `.log`
- `num-timestep = 0` 时可成功完成 preprocessing + solver setup
- `num-timestep = 1` 时可成功跑完整个 timestep
- 当 `shell-ipc.json` 缺失 `ipc-dhat` 或 `ipc-kappa` 时，程序以配置错误失败退出
- 日志中打印最终采用的 `ipc-dhat / ipc-kappa / eps_ee / slackness`
- 对“缺失 `ipc-dhat` / `ipc-kappa`”这类失败路径，测试固定在 test body 中 on-the-fly 写临时 JSON；本批不向 repo 新增 checked-in 的坏配置样例

这里的 CLI smoke test 范围固定为：

- 主要验证新入口、config 解析、shell FEM setup、time integrator 接线与最小步进
- 不保证测试用例一定进入 active contact pair
- 不把“真实进入 self-contact 的端到端场景”作为本批 CLI smoke 的必选验收项

若需要验证真正进入 active pair 的 shell case，本批优先通过手动 smoke 或后续更聚焦的 contact integration case 完成，而不是强塞进首版 CLI smoke。

### 3. Non-regression

继续保持通过：

- `tests/src/core/contact/cipcPotentialEnergy_gtest.cpp`
- `tests/src/core/contact/cipcProfiling_gtest.cpp`
- `tests/src/core/contact/surfaceIPCCore_gtest.cpp`
- `tests/src/tools/runSim_gtest.cpp`

## Implementation Order

建议按下面顺序落：

1. 新增 `EmbeddedSurfaceIPCPotentialEnergy`
2. 先写 `W = I` 对齐测试
3. 再写手工稀疏 `W` pull-back 测试
4. 接入 adapter profiling
5. 新增 `runIPCSim.cpp` shell-only 骨架
6. 跑通 `examples/ipc/shell/shell-ipc.json`
7. 最后补 `runIPCSim` CLI/logging/smoke test

这个顺序的目的，是先把最难出数值错位的问题锁死，再做 CLI。
其中真正的关键路径固定为：

- 第 2 步 `W = I` 对齐测试
- 第 3 步手工稀疏 `W` 的 pull-back 测试

这两步必须先稳定通过，再继续推进 `runIPCSim` 的 shell 接线；否则后续 CLI/smoke 即使跑通，也不足以说明 adapter 数学接线正确。

## Validation

本批预期验证命令固定为：

```bash
cmake --build --preset base_no_mkl_debug --target \
  embeddedSurfaceIPCPotentialEnergy_gtest \
  cipcPotentialEnergy_gtest \
  cipcProfiling_gtest \
  surfaceIPCCore_gtest \
  runIPCSim_gtest

ctest --test-dir build/base_no_mkl_debug --output-on-failure -R \
  "EmbeddedSurfaceIPCPotentialEnergy|CIPCPotentialEnergy|CIPCProfiling|SurfaceIPCCore|RunIPCSim"
```

另外需要至少一次手动 smoke：

```bash
build/base_no_mkl_debug/bin/runIPCSim examples/ipc/shell/shell-ipc.json --log
```

验收观察点固定为：

- 程序成功退出
- log 正常生成
- shell 输出目录创建成功
- 日志中能看到 IPC 参数与 solver 初始化信息

## Intentional Narrowing Relative To Source Plan

相对总计划，这份 phase 文档做了三点主动收窄：

1. 不在本批实现 tet/cubic
2. 不在本批实现 `base-config` 配置继承
3. 不在本批给新 adapter 保留 `is_input_disp`

这三点收窄都是为了先交付一条最短、最清晰、可验证的 unified shell IPC 路径。

## Exit Condition

这份计划完成后，仓库应当进入这样的状态：

- `SurfaceIPCCore`
  已是唯一 surface IPC kernel
- `CIPCPotentialEnergy`
  是 legacy/reference wrapper
- `EmbeddedSurfaceIPCPotentialEnergy`
  是 unified simulation-space adapter
- `runIPCSim`
  已能跑 shell self-contact IPC

届时下一批就可以集中处理：

- tet/cubic embedding/runtime path
- inversion-free material max step

而不需要再回头补第一条 unified shell pipeline。
