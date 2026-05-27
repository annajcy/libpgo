# Phase 1.5 Plan: inversion-free / material-feasible max step

Source plan: `/Users/jinceyang/Desktop/codebase/libpgo/plan/ipc/ipc_friction.plan.md`  
Depends on:
- `/Users/jinceyang/Desktop/codebase/libpgo/plan/ipc/phase1/phase1A.plan.md`
- `/Users/jinceyang/Desktop/codebase/libpgo/plan/ipc/phase1/phase1BC.plan.md`
- `/Users/jinceyang/Desktop/codebase/libpgo/plan/ipc/phase1/phase1D.plan.md`

## Summary

`Phase 1.5` 的目标很窄也很关键：把当前只受 contact-feasible 限制的 line search，补成同时受 material-feasible 限制。

这一步不引入新的 contact kernel，不改 external/friction，也不改 integrator 的求解结构；它只做一件事：

- 让 `DeformationModelEnergy::computeMaxStepSize()` 不再恒等于 `1.0`
- 让 `tet / cubic` 都能在试探步上避免体单元翻转或 integration point Jacobian 失正
- 让已有的 integrator 聚合逻辑自然得到
  $$
  \alpha_{\max} = \min(\alpha_{\text{contact}}, \alpha_{\text{material}})
  $$
- 同时给 `DeformationModelEnergy` / `runSim` / `runIPCSim` 接一个 config 级
  `enable-material-max-step` bool 开关，默认 `true`；若显式设为 `false`，
  则材料 inversion-free max-step clamp 整体旁路，`computeMaxStepSize()` 在材料侧直接返回 `1.0`

本批完成后，`runIPCSim` 的 volume self-contact 主链路会第一次具备 “不穿透之外，也尽量不翻体单元” 的基本材料可行性保护。

## Scope Lock

### In Scope

- 在 `src/core/solidDeformationModel/deformationModelEnergy.h/.cpp` 中实现真正的 `computeMaxStepSize(x, dx)`
- 按 mesh type 支持两类材料可行性判定：
  - `SimulationMeshType::TET`
  - `SimulationMeshType::CUBIC`
- 复用已有 integrator 聚合逻辑：
  - `ImplicitBackwardEulerEnergy::computeMaxStepSize()`
  - `TRBDF2TimeIntegratorEnergy::computeMaxStepSize()`
- 新增面向材料可行性的单元测试 target
- 做最小必要的 `runIPCSim` 多步验证

### Out Of Scope

- 不改 `SurfaceIPCCore` 的 `computeMaxStepSize()`
- 不把 inversion-free 判定塞进 contact 类或 adapter 类
- 不把 `SimulationMeshType::TRIANGLE` / `SHELL` 纳入本 phase 的 inversion-free 约束
- 不实现 external contact
- 不实现 friction
- 不重写 `runIPCSim` 主循环
- 不改 `runSim` 的 legacy contact 主路径
- 不做 broad phase / spatial hash
- 不做一般化的解析 root formula
- 不负责保证 trial state 上 `DeformationModelEnergy::func / gradient / hessian` 一定 finite；本 phase 只保证 volume non-inversion，不保证所有模型在试探点上的数值有限性
- 不负责保证 shell trial state 的 finite energy / finite gradient；这属于后续 `shell anti-collapse / finiteness safeguard` 议题

## Completion Standard

`Phase 1.5` 合入的标准固定为：

- `DeformationModelEnergy::computeMaxStepSize()` 对 `tet/cubic` 不再恒返回 `1.0`
- `ImplicitBackwardEuler` 与 `TRBDF2` 无需额外 special-case 即可自动吃到材料 max step
- `tet/cubic` 的最小材料可行性单测全部通过
- `runSim` / `runIPCSim` 通过 config 字段 `loglevel` 暴露 max-step 观测：
  - 默认 `trace`
  - `trace`：每次 material/contact clamp 都打印，并保留 solve summary
  - `debug` / `info`：保留 solve summary
  - `warn`：只保留异常 warning
- `enable-material-max-step=false` 时：
  - `DeformationModelEnergy` 不再做材料缩步
  - `materialClampCount_` 不递增
  - `runSim` / `runIPCSim` 都能把该配置传到 runtime
- `runIPCSim` 的最小 volume 多步样例在结束时满足可机器判定的几何条件：
  - dump 出的最小 `det(D_s)` 或最小 integration-point `det(F)` 仍为正；
  - 运行日志或统计中至少出现过一次 material max step 严格小于 `1.0`，证明材料 clamp 确实触发过
- 现有 contact max step 与新的 material max step 能同时生效

只要以上条件成立，就视为 `Phase 1.5` 完成；更高层的长期稳定性和 external/friction 耦合行为留给后续阶段。

## Repo Truth This Plan Builds On

当前 repo 已有以下前提，`Phase 1.5` 直接复用：

- `SurfaceIPCCore` 已实现 contact-feasible `computeMaxStepSize()`
- `EmbeddedSurfaceIPCPotentialEnergy` 已把 contact side 的 `computeMaxStepSize()` 接到 unified IPC 主路径
- `ImplicitBackwardEulerEnergy::computeMaxStepSize()` 与 `TRBDF2TimeIntegratorEnergy::computeMaxStepSize()` 已经会对所有 implicit model 取最小值
- `DeformationModelEnergy::computeMaxStepSize()` 当前 repo truth 已不再恒等于 `1.0`；
  这个 plan 文件里的相关表述应理解为本 phase 的设计意图与验收边界，而不是未实现状态
- `DeformationModelEnergy` 当前持有 `forceModelAssembler`，而 `forceModelAssembler` 已公开提供：
  - `getDeformationModelManager()`
  - `getDeformationModelManager()->getMesh()`
  - `getDeformationModelManager()->getDeformationModel(ele)`
- IPC contact 与 integrator 聚合当前也已支持只读观测：
  - `SurfaceIPCCore::{getContactClampCount,getMinContactFeasibleAlphaThisSolve,resetContactMaxStepStats}`
  - `ImplicitBackwardEulerEnergy::{getMinFeasibleAlphaThisSolve,getMinLineSearchAlphaThisSolve,getMinEffectiveAlphaThisSolve}`
  - `TRBDF2TimeIntegratorEnergy::{getMinFeasibleAlphaThisSolve,getMinLineSearchAlphaThisSolve,getMinEffectiveAlphaThisSolve}`
- `TetMeshDeformationModel` 已有 public helper `computeDs(...)`
- `CubicMeshDeformationModel` 已有 public helper `computeF(local_x, materialLocationID, F)`
- `SimulationMesh` 已能提供 mesh type、element vertex indices 与 rest positions
- shell triangle 在 3D 中没有与 `tet/cubic` 同等明确的 orientation-preserving 符号量；单 triangle normal 的符号不适合作为本 phase 的严格 inversion 判据

因此这次不是发明新基础设施，而是把已有材料几何事实真正接进 line search。

## Key Design Decisions

### 1. 接入点固定在 `DeformationModelEnergy`

这一批的入口 single source of truth 仍固定在：

- `src/core/solidDeformationModel/deformationModelEnergy.h`
- `src/core/solidDeformationModel/deformationModelEnergy.cpp`

但实现 ownership 不再把所有 helper 都堆进 `deformationModelEnergy.cpp`，而是固定分成 5 层：

- `src/core/pgoLogging/pgoLogging.h/.cpp`
  - 统一 logger `WARN`
- `src/core/basicAlgorithms/polynomialRootUtils.h/.cpp`
  - `CubicPolynomial`
  - critical-point isolation
  - bracketed `bisection`
- `src/core/solidDeformationModel/materialMaxStepPolynomialUtils.h/.cpp`
  - `det(A + alpha B) - eps` 的列多线性展开
  - 共享的 material-feasible alpha 搜索 helper
- `src/core/solidDeformationModel/{tet,cubic,koiter}MeshDeformationModel.*`
  - 每种 element 的 local feasibility
- `src/core/solidDeformationModel/deformationModelAssembler.*`
  - local gather 与全局 `min`

原因是：

- 材料可行性属于 deformation model 的责任，不属于 contact kernel
- integrator 侧已经有统一聚合，不值得再发明第二套 max-step 协调层
- `Phase 2/3` 以后 external 与 friction 也应继续复用这个聚合结果，而不是反向侵入材料能量

### 2. 仍然只暴露一个 public API

public API 仍保持为：

```cpp
double computeMaxStepSize(EigenSupport::ConstRefVecXd x,
  EigenSupport::ConstRefVecXd dx) const override;
```

这一批不新增新的 public helper，例如：

- `computeMaterialMaxStepSize(...)`
- `computeElementMaxStepSize(...)`
- `computeInversionFreeAlpha(...)`

需要的 helper 不再一律塞进 `DeformationModelEnergy.cpp` 的匿名命名空间；当前 repo truth 固定为：

- `DeformationModelEnergy`
  只保留 absolute-position 语义、`PotentialEnergy` 入口和 `materialClampCount_`
- `DeformationModelAssembler`
  负责 element gather 与全局 `min`
- concrete deformation model
  负责单 element `computeLocalMaxStepSize(...)`
- logging / polynomial / determinant helper
  分别放到上面的专用模块里

例外：

- 允许新增只读的 test-facing getter，例如
  `int64_t getMaterialClampCount() const`
- 这类 getter 仅用于观测材料 clamp 是否触发，不构成长期算法 API，也不改变 `computeMaxStepSize(...)` 作为主入口的设计
- 当前 repo truth 里，材料侧额外暴露的是
  `getMinMaterialFeasibleAlphaThisSolve() const` 与
  `resetMaterialMaxStepStats() const`，而不是 `getLastMaterialAlpha() const`
- 允许同时新增一个很小的 runtime toggle：
  - `setEnableMaterialMaxStep(bool)`
  - `isMaterialMaxStepEnabled() const`
  - 其职责仅限于让 config 可以显式旁路材料 max-step clamp，不改变主算法 API 形状

### 3. 坐标语义严格沿用 repo truth

`DeformationModelEnergy` 内部对当前状态的解释固定为：

- 若 `restPosition.size() > 0`，则 `x` 是 displacement，absolute positions 为
  $$
  x_{\text{abs}} = x_{\text{rest}} + x
  $$
- 否则 `x` 直接就是 absolute positions
- 试探更新统一为
  $$
  x_{\text{trial}}(\alpha) = x_{\text{abs}} + \alpha\,dx
  $$
- 当前 / trial absolute positions 的 authoritative source 固定是：
  - `restPosition.size() > 0` 时使用 `restPosition + x` 与 `restPosition + x + alpha * dx`
  - `restPosition.size() == 0` 时使用 `x` 与 `x + alpha * dx`
- `SimulationMesh::getVertex(...)` 在本 phase 中只允许用于读取 rest-frame 几何事实，例如：
  - shell 的 rest triangle 几何
  - tet 的 rest `D_m`
  - 各类基于 rest frame 的阈值量
- `SimulationMesh::getVertex(...)` 不允许参与 current/trial absolute position 的重建；实现时不得把它当作 `x_rest` 的替代来源。

不在本批引入新的 generalized coordinate 语义。

### 4. root-find 固定采用 root isolation + bisection refine，并收敛为 named constants

本批对 `tet/cubic` 统一采用：

1. 先构造
   \[
   g(\alpha)=\phi(\alpha)-\epsilon
   \]
   的三次多项式
2. 通过 `g'(\alpha)` 的实根，把 `[0,1]` 切成若干单调小区间
3. 从左到右扫描这些区间，定位第一个与退化边界相交的 root bracket
4. 对已 bracket 的根区间，首版用保守 `bisection` 做 refine
5. 返回值乘 `0.99`
6. 结果下限 clamp 到 `1e-12`

三次多项式系数的构造方式也在本 phase 明确钉死，不允许实现者自由选择：

- 对任一 `tet` element 或 `cubic` integration point，都先把待检查的 `3 x 3` 矩阵写成
  \[
  M(\alpha)=A+\alpha B
  \]
  其中 `A` 来自当前 absolute positions 对应的 `D_s(0)` / `F_ref,q(0)`，`B` 来自沿 `dx` 的线性增量。
- 系数固定按 `det` 的列多线性展开直接计算，不允许用 sampling + Vandermonde 拟合：
  - 若
    \[
    A=[a_0,\ a_1,\ a_2],\quad B=[b_0,\ b_1,\ b_2]
    \]
  - 则
    \[
    g(\alpha)=\det(A+\alpha B)-\epsilon
    = c_0 + c_1 \alpha + c_2 \alpha^2 + c_3 \alpha^3
    \]
  - 其中
    \[
    c_0=\det(a_0,a_1,a_2)-\epsilon
    \]
    \[
    c_1=\det(b_0,a_1,a_2)+\det(a_0,b_1,a_2)+\det(a_0,a_1,b_2)
    \]
    \[
    c_2=\det(b_0,b_1,a_2)+\det(b_0,a_1,b_2)+\det(a_0,b_1,b_2)
    \]
    \[
    c_3=\det(b_0,b_1,b_2)
    \]
- `tet` 与 `cubic` 的差别只在于 `A/B` 的来源不同：
  - `tet` 由 `D_s(0)` 与对应的 `delta D_s`
  - `cubic` 由 integration-point 上 `F_ref,q(0)` 与对应的 `delta F_ref,q`

退化分支处理规则在语义上固定如下，避免不同实现者各猜一套：

- 若 `g(0) <= 0`，按“当前状态已非法”处理：
  - 返回恢复性 clamp
  - 触发普通 `WARN`
  - 不再继续做 root isolation
- 若 `deg(g) == 0`：
  - 常数项始终大于 `0` 时直接返回 `1.0`
  - 否则走非法初值恢复路径
- 若 `deg(g) == 1`：
  - 直接检查唯一线性根是否落在 `(0, 1]`
  - 若不存在这样的根且 `g(0) > 0`，直接返回 `1.0`
- 若 `deg(g) == 2` 或 `3`：
  - 先求 `g'` 在 `(0,1)` 中的实根
  - 若无实根，则把整段 `[0,1]` 视为单调区间
  - 若有实根，则与端点 `0, 1` 一起排序、去重后形成单调子区间
- 对每个单调子区间 `[l, r]`：
  - 若 `g(l) > 0` 且 `g(r) > 0`，整段可行，继续往右扫描
  - 若 `g(r) == 0`，则 `r` 本身就是 candidate boundary root
  - 若 `g(l) > 0` 且 `g(r) < 0`，则该段存在最早失效边界，对此段做 `bisection` refine
- 若扫描完整个 `[0,1]` 都没有找到 boundary root，且 `g(0) > 0`，直接返回 `1.0`

这里的 `deg(g) = 0/1/2/3` 主要是策略说明，不要求实现里真的做显式的浮点阶数分派：

- 不要求再引入额外的 `|c_3| < tol`、`|c_2| < tol` 之类 degree 判定阈值
- 实现可以统一走“三次通用管线”：
  - 求 `g'` 的至多两个实根
  - 若最高次系数恰为 `0`，对应求根过程自然退化为二次或一次
  - 再按得到的单调子区间做扫描与 `bisection` refine
- 文档里的 `deg(g)` 分支只是帮助说明语义，不要求代码里真的写一套独立的 `deg-switch`

若发现 `phi(0) <= eps`，则这被视为“当前状态已非法”，不是正常的 material-tight clamp。此时实现仍可返回恢复性的极小正数以避免上层立即退化，但必须：

- 触发普通 `WARN`
- warning 至少带：
  - mesh type
  - element id
  - 当前 `phi(0)` 与阈值 `eps`
- 不允许静默吞掉这类状态

这里明确不采用“整段 `[0,1]` 统一二分”的黑盒近似，也不在首版引入解析 Cardano 求根。原因是：

- `g(\alpha)` 虽然是三次，但 `[0,1]` 上不保证整体单调；
- 先做 root isolation 再 refine，能避免错误依赖“整段前缀可行域”的隐含假设；
- 对已 bracket 的根区间，`bisection` 比 `Brent` 更简洁，也足够稳健，适合作为首版实现。

这条规则对 `tet/cubic` 一致，不引入 mesh-specific 的 closed-form root solver。

实现期固定把下面几个量写成 named constants，而不是裸字面量：

- `kMaxStepInteriorSafety = 0.99`
- `kMaxStepMinClamp = 1e-12`
- `kTetRelativeDetEps = 1e-8`
- `kCubicRelativeDetEps = 1e-8`

非法初值 warning 与 material clamp 统计也要绑定到具体实现机制，而不是停留在描述层：

- 非法初值 warning 固定复用
  `src/core/pgoLogging/pgoLogging.h/.cpp`
  中的统一 logger
- 调用面统一使用普通 `WARN`
- 不做 warn-once 去重
- warning payload 至少包含：
  - mesh type
  - element id
  - `phi(0)`
  - `eps`
- 为了让 “至少出现一次 `alpha_material < 1.0`” 变成真正机器可判定的信号，实现应维护一个可测试的材料缩步计数通道：
  - 首选形状是 `DeformationModelEnergy` 内部的 `materialClampCount_`
  - 该计数建议声明为 `mutable std::atomic<int64_t>`
  - 每次返回严格小于 `1.0` 的 material max step 时自增
  - Stage 3 的工具级验收优先读这个计数或其结构化 summary，而不是 grep 任意日志

其中：

- `0.99` 与 contact side 的 safety/slackness 语义是**刻意对齐**的：
  - 都表示 “找到边界后退一点，保持 strict interior”
  - 不是偶然取了同量级
- 但 `Phase 1.5` 仍允许它们在代码中作为独立 constant 命名，避免和 contact 参数 storage 强绑死

### 5. `cubic` 必须复用现有 public helper，不允许重抄内核

`cubic` 的材料判据虽然最复杂，但实现边界要守住：

- 不在 `DeformationModelEnergy` 里重写高斯点形函数导数
- 不复制 `restDmInv`
- 不复制 `CubicMeshDeformationModel::prepareData()` 的内部数学

固定通过：

```cpp
dynamic_cast<const CubicMeshDeformationModel *>
```

拿到 element model，再复用：

```cpp
computeF(local_x, materialLocationID, F)
```

以 `det(F)` 判定 integration point 正性。

`local_x` 的内存布局也在本 phase 钉死，避免实现者再去翻 `cubicMeshDeformationModel.cpp`：

- `computeF(local_x, materialLocationID, F)` 要求的 `local_x` 固定是 node-major
- 具体布局为：
  - `x0, y0, z0, x1, y1, z1, ..., x7, y7, z7`
- 不允许传 coordinate-major 的 `x0..x7, y0..y7, z0..z7`

## Stage Breakdown

## Stage 0: Shared Helpers And First Reviewable TET Slice

### Goal

先把 `Phase 1.5` 需要的共同脚手架补齐，并在同一批里至少接通最小 `tet` 路径，避免留下一个无法独立 review 的纯 skeleton 提交：

- absolute position 组装
- element local positions gather
- cubic polynomial root isolation
- bracketed `bisection` refine
- safety clamp

这一 stage 的 tet slice 边界也明确钉死：

- 只要求单 `tet` element 路径能在 `computeMaxStepSize()` 里返回真实材料 max step
- 不要求在这一 stage 完成多 element 聚合、完整测试矩阵或工具级 smoke
- 目标是让 Stage 0 的提交本身已经不是 skeleton，但也不伪装成 “tet 全量已完工”

### Files

- `src/core/pgoLogging/pgoLogging.h`
- `src/core/pgoLogging/pgoLogging.cpp`
- `src/core/basicAlgorithms/polynomialRootUtils.h`
- `src/core/basicAlgorithms/polynomialRootUtils.cpp`
- `src/core/solidDeformationModel/materialMaxStepPolynomialUtils.h`
- `src/core/solidDeformationModel/materialMaxStepPolynomialUtils.cpp`
- `src/core/solidDeformationModel/deformationModel.h`
- `src/core/solidDeformationModel/deformationModelAssembler.h`
- `src/core/solidDeformationModel/deformationModelAssembler.cpp`
- `src/core/solidDeformationModel/deformationModelEnergy.h`
- `src/core/solidDeformationModel/deformationModelEnergy.cpp`
- `src/core/solidDeformationModel/tetMeshDeformationModel.h`
- `src/core/solidDeformationModel/tetMeshDeformationModel.cpp`
- `src/core/solidDeformationModel/cubicMeshDeformationModel.h`
- `src/core/solidDeformationModel/cubicMeshDeformationModel.cpp`
- `src/core/solidDeformationModel/koiterDeformationModel.h`
- `src/core/solidDeformationModel/koiterDeformationModel.cpp`

### Work Items

- 在 `pgoLogging` 中复用统一 logger 输出非法初值 warning
- 在 `basicAlgorithms` 中新增纯数学 helper：
  - `CubicPolynomial`
  - `findCriticalPointsInUnitInterval(...)`
  - `findFirstBoundaryRootByBisection(...)`
- 在 `solidDeformationModel/materialMaxStepPolynomialUtils.*` 中新增：
  - `buildDeterminantCubicFromAffineMatrixPath(...)`
  - `findConservativeFeasibleAlpha(...)`
  - safety clamp / recovery clamp constants
- 同一 stage 内至少把一个最小 `tet` case 接通到 `computeMaxStepSize()`，使 Stage 0 的提交本身就是可运行、可 review、可测试的
- 在 Stage 0 明确接入 repo-truth access chain：
  - `forceModelAssembler->getDeformationModelManager()`
  - `forceModelAssembler->getDeformationModelManager()->getMesh()`
  - `forceModelAssembler->getDeformationModelManager()->getDeformationModel(ele)`
- 明确 helper 的 ownership 边界：
  - `DeformationModelEnergy`
    只负责 absolute-position assemble 与 clamp count
  - `DeformationModelAssembler`
    负责 gather local `x / dx` 与 per-element `min`
  - concrete `DeformationModel`
    负责单 element `computeLocalMaxStepSize(...)`
- 在 `DeformationModel` 基类上新增：
  - `LocalMaxStepResult`
  - `computeLocalMaxStepSize(const double *x_local, const double *dx_local) const`
- 在 `DeformationModelAssembler::computeMaxStepSize()` 中接入统一流程：
  - gather local `x / dx`
  - 调 concrete deformation model 的 `computeLocalMaxStepSize(...)`
  - 遍历所有 element 取最小材料可行步长
  - 对非法初值统一调普通 `WARN`
- 在 `DeformationModelEnergy::computeMaxStepSize()` 中只保留统一入口：
  - early exit: `dx.size() == 0` 或 `dx.norm() == 0`
  - `restPosition + x` / `x` 的 absolute-position 语义
  - 对 assembler 的转发
- 当发现 `phi(0) <= eps` 时：
  - 返回恢复性 clamp 值
  - 同时发出普通 `WARN`
  - 不把这种情况伪装成普通的小步长收缩
- 在 `DeformationModelEnergy` 内部预留材料缩步统计：
  - 每次返回 `< 1.0` 的 material max step 时递增 `materialClampCount_`
- 先保守支持未知 mesh type：
  - 返回 `1.0`
  - 不在本批扩展新 mesh family
- `SimulationMeshType::TRIANGLE` / `SHELL` 在本 phase 明确保留为 `1.0`
- `SimulationMeshType::EDGE_QUAD` 在本 phase 也显式保持 `1.0`

### Exit Criteria

- `computeMaxStepSize()` 的代码形状不再是单行 `return 1.0`
- logging / math / model / assembler / energy 的 ownership 已分离
- Stage 0 本身已经包含一个最小 `tet` 可执行 slice，而不是纯骨架
- `TRIANGLE` / `SHELL` 路径在本 phase 明确保持不变

## Stage 1: TET Material Max Step

### Goal

先把最简单、最稳定的 `tet` 分支做通。

### Files

- `src/core/solidDeformationModel/tetMeshDeformationModel.h`
- `src/core/solidDeformationModel/tetMeshDeformationModel.cpp`
- `src/core/solidDeformationModel/deformationModelAssembler.cpp`
- `src/core/solidDeformationModel/deformationModelEnergy.cpp`
- `tests/src/core/solidDeformationModel/deformationModelEnergyMaxStep_gtest.cpp`
- `tests/src/core/solidDeformationModel/CMakeLists.txt`

### Feasibility Definition

对每个 tet element，定义：

$$
\phi(\alpha)=\det(D_s(\alpha))
$$

其中 `D_s(alpha)` 的构造语义与 `TetMeshDeformationModel::prepareData()` 保持一致。

阈值固定为：

$$
\epsilon_{\det} = 10^{-8}\,|\det(D_m)|
$$

这里显式取绝对值；本 phase 不要求 rest tet 预先保证统一正定向，避免 left-handed rest tet 让阈值变成负数。

### Work Items

- 从 `SimulationMesh` gather 一个 tet 的 `4 x 3` trial positions
- 直接调用静态 helper `TetMeshDeformationModel::computeDs(...)`，不通过 element instance 或 `dynamic_cast`
- 为单个 tet 实现 `phi(alpha) > eps_det` 判据
- 把单 tet 判据接入统一的 cubic polynomial root isolation helper
- 对定位到的最早根区间，用 `bisection` 做 refine
- 对全部 tet element 取最小值
- 把 tet 的 reference test fixture 直接写死到测试里，避免实现者各自挑数：
  - rest tet:
    - `v0 = (0, 0, 0)`
    - `v1 = (1, 0, 0)`
    - `v2 = (0, 1, 0)`
    - `v3 = (0, 0, 1)`
  - canonical inversion-inducing `dx`:
    - 仅移动 `v3`
    - `dx_3 = (0, 0, -2)`
  - `deg == 0` fixture：
    - 对 4 个顶点统一施加 `dx = (1, 2, 3)`
    - 这是纯刚体平移，对应 `B == 0`
    - 预期返回 `1.0`

### Tests

- 新增单 tet 例子：
  - 使用上面的 canonical tet + `dx`
  - 断言 `computeMaxStepSize() < 1`
  - 断言按返回步长更新后 `det(D_s) > 0`
- 新增多 tet 聚合例子：
  - 构造至少两个 tet element，其中只有一个 element 会更早触发失正
  - 断言 `DeformationModelEnergy::computeMaxStepSize()` 返回该最早 element 的临界步长，而不是其它 element 的步长
- 新增可行步例子：
  - `alpha = 1` 应保持可行
  - 返回值应接近 `1`

### Exit Criteria

- `tet` 分支在单测中能稳定缩步
- `shell` 路径在本 phase 未被错误纳入材料判据

## Stage 2: CUBIC Material Max Step

### Goal

在不复制 cubic 内部 FEM 数学的前提下，把 `cubic` 分支接上。

### Files

- `src/core/solidDeformationModel/cubicMeshDeformationModel.h`
- `src/core/solidDeformationModel/cubicMeshDeformationModel.cpp`
- `src/core/solidDeformationModel/deformationModelAssembler.cpp`
- `src/core/solidDeformationModel/deformationModelEnergy.cpp`
- `tests/src/core/solidDeformationModel/deformationModelEnergyMaxStep_gtest.cpp`

### Feasibility Definition

对每个 cubic element 的每个 integration point `q`：

$$
\phi_q(\alpha) = \det(F_{\text{ref},q}(\alpha))
$$

其中 `F_ref,q(alpha)` 的语义与 `CubicMeshDeformationModel::prepareData()` 保持一致。

单个 element 的可行步长取所有 integration point 的最小值。

### Work Items

- 在 assembler 中 gather 单个 cubic element 的 `8 x 3` local trial positions
- 由 `CubicMeshDeformationModel::computeLocalMaxStepSize(...)` 直接处理单 element 路径，不再由 energy/assembler 做 `dynamic_cast`
- 对 `materialLocationID = 0..7`：
  - 调 `computeF(local_x, q, F)`
  - 以 `det(F)` 判定正性
- 为 cubic element 接入统一的 cubic polynomial root isolation helper
- 对定位到的最早根区间，用 `bisection` 做 refine
- 对所有 cubic element 取全局最小值
- 把 cubic 的 canonical test fixture 也写死为一个 unit cube，`local_x` 使用 node-major：
  - rest hex vertices:
    - `v0 = (0, 0, 0)`
    - `v1 = (1, 0, 0)`
    - `v2 = (1, 1, 0)`
    - `v3 = (0, 1, 0)`
    - `v4 = (0, 0, 1)`
    - `v5 = (1, 0, 1)`
    - `v6 = (1, 1, 1)`
    - `v7 = (0, 1, 1)`
  - canonical inversion-inducing `dx`:
    - bottom face `v0..v3` 固定
    - top face `v4..v7` 统一施加 `(0, 0, -2)`

### Tests

- 单 hex 测试：
  - 使用上面的 canonical unit-cube hex + top-face `dz = -2`
  - 断言 `computeMaxStepSize() < 1`
  - 断言按该步长更新后所有 integration point `det(F_ref,q) > 0`
- 新增多 element 聚合例子：
  - 构造至少两个 cubic element，其中只有一个 element / integration point 更早触发失正
  - 断言 `DeformationModelEnergy::computeMaxStepSize()` 返回该最早失正位置对应的步长
- 再加一个明显可行的 `dx`，返回值应接近 `1`

### Exit Criteria

- `tet/cubic` 两类单测全部通过
- `cubic` 路径没有引入重复数学实现

## Stage 3: Integration Wiring Validation

### Goal

确认新的材料 max step 能真正被 unified IPC 主路径吃到，而不只是单测里正确。

### Files

- `tests/src/core/basicAlgorithms/polynomialRootUtils_gtest.cpp`
- `tests/src/tools/runSim_gtest.cpp` 或新的 `runIPCSim` 相关测试文件
- `tests/src/core/solidDeformationModel/deformationModelEnergyMaxStep_gtest.cpp`
- 必要时更新 phase1.5 验证说明文档

### Work Items

- 运行最小多步样例：
  - `examples/ipc/tet/box-hang/box-ipc.json`
  - `examples/ipc/cubic/box-hang/box-ipc.json`
- 验证在存在接触或大步长试探时：
  - contact max step 仍然生效
  - material max step 也能共同缩步
- 把工具级验收收敛成可机器判定的输出：
  - tet 路径记录最终最小 `det(D_s)`
  - cubic 路径记录最终最小 integration-point `det(F)`
  - 通过 `materialClampCount_` 或等价结构化 summary 记录至少一次 `alpha_material < 1.0`
- 新增最小 integrator 聚合 regression：
  - 构造一个 `DeformationModelEnergy` 与一个返回固定较大/较小 max-step 的 trivial implicit energy
  - 分别在 `ImplicitBackwardEulerEnergy::computeMaxStepSize()` 与 `TRBDF2TimeIntegratorEnergy::computeMaxStepSize()` 上断言最终结果等于 `min(material, other)`
- 若 `runIPCSim` 现有 test target 还不完整，则这一 stage 先以手工 smoke + 现有回归测试完成，不把 CLI test 缺失当作算法 blocker

### Exit Criteria

- 单测之外，最小 unified IPC 路径也能观察到材料缩步效果
- 无新增回归

## Stage 4: Documentation And Follow-Up Boundaries

### Goal

在代码落地后，把 `shell` defer 边界和实现结果一起沉淀进文档，避免后续 `Phase 2`、`Phase 3` 再猜一次 repo truth。

### Files

- `/Users/jinceyang/Desktop/codebase/libpgo/plan/ipc/phase1.5/phase1.5.plan.md`
- `/Users/jinceyang/Desktop/codebase/libpgo/plan/ipc/phase1.5/phase1.5.impl.md`

### Work Items

- 在实现记录中明确：
  - `SimulationMeshType::TRIANGLE` / `SHELL` 在 `Phase 1.5` 中保持 `computeMaxStepSize() == 1.0`
  - 这不是 bug，而是 scope choice
  - 如后续需要 shell 数值保护，应单列 follow-up：
    - `shell anti-collapse`
    - `shell finite-energy safeguard`
- 更新实现后的 repo truth：
  - 哪些 mesh type 已落地
  - ownership 已分成 logging / math / model / assembler / energy 五层
  - 测试命令和结果
- 若实现中对 source plan 有收窄或偏移，在 `phase1.5.impl.md` 里显式记录

### Exit Criteria

- 计划文档与实现记录一致
- 后续 phase 无需重新猜测 `Phase 1.5` 的 repo truth

## Testing Plan

### New Test Target

新增：

- `tests/src/core/solidDeformationModel/deformationModelEnergyMaxStep_gtest.cpp`

并在：

- `tests/src/core/solidDeformationModel/CMakeLists.txt`

中加入对应 target。

### Coverage Requirements

至少覆盖以下断言：

- tet 单单元 determinant 变号前缩步
- 多 tet element 时，全局结果等于最早失正 element 的步长
- cubic 单 hex integration point 失正前缩步
- 多 cubic element / integration point 时，全局结果等于最早失正位置的步长
- tet / cubic 的最早正实根由 root isolation + `bisection` refine 给出，而不是整段 `[0,1]` 黑盒二分
- `dx = 0` 或显然可行时返回 `1.0`
- 初始状态已非法时返回受 clamp 的极小正数而不是 `0`
- 初始状态已非法时每次都会触发 `WARN`，而不是静默返回极小步长
- 三次系数来自 `det(A + alpha B)` 的直接展开，而不是 sampling + Vandermonde 拟合
- `deg(g) = 0/1/2/3` 的语义分支都被覆盖
- `deg == 0` 的 canonical fixture 由纯刚体平移给出，返回值应为 `1.0`
- `SimulationMeshType::TRIANGLE` / `SHELL` 在本 phase 中仍返回 `1.0`
- `SimulationMeshType::EDGE_QUAD` 在本 phase 中显式保持 `1.0`
- `tet/cubic` 两类 mesh 在统一 helper 路径下都使用相同的 safety factor 和下限策略
- `materialClampCount_` 或等价统计信号能被测试读取，并能证明至少一次 material clamp 发生
- `loglevel=trace/debug/info/warn` 的 summary / per-clamp / abnormal-warning 行为有工具级回归测试锁定
- `ImplicitBackwardEulerEnergy::computeMaxStepSize()` 与 `TRBDF2TimeIntegratorEnergy::computeMaxStepSize()` 会正确返回 `min(material, other)`

## Validation Commands

### Unit Tests

```bash
cmake --build --preset base_no_mkl_debug --target deformationModelEnergyMaxStep_gtest
ctest --test-dir build/base_no_mkl_debug --output-on-failure -R "DeformationModelEnergyMaxStep"
```

### Regression Sweep

```bash
ctest --test-dir build/base_no_mkl_debug --output-on-failure -R "DeformationModelEnergyMaxStep|SurfaceIPCCore|CIPCPotentialEnergy|EmbeddedSurfaceIPCPotentialEnergy|RunSim"
```

### IPC Smoke Validation

若 `runIPCSim` 的对应 gtest 尚未成形，则补手工 smoke：

```bash
build/base_no_mkl_debug/bin/runIPCSim <ipc-config>
```

具体 smoke config 固定为：

- `examples/ipc/tet/box-hang/box-ipc.json`
- `examples/ipc/cubic/box-hang/box-ipc.json`

目标不是长时间动画稳定，而是确认 volume 多步推进时材料 max step 确实已接入，并能通过最终最小 `det(D_s)` / `det(F)` 与 `materialClampCount_ > 0` 这样的机器可判定信号被观察到。

## Risks And Guardrails

### Risk 1: `cubic` 实现滑向重复数学

Guardrail:

- 只允许复用 `CubicMeshDeformationModel` public helper
- 不允许在 `DeformationModelEnergy` 里复制 shape derivative / `restDmInv`

### Risk 2: 误把 shell 纳入 volume-style inversion-free 语义

Guardrail:

- `SimulationMeshType::TRIANGLE` / `SHELL` 在本 phase 明确保留为 `1.0`
- 不为 shell 发明伪装成 volume inversion-free 的 normal-sign 判据
- 若后续 shell 确实需要保护，单列 `shell anti-collapse / finiteness safeguard` 议题

### Risk 2.5: 未显式锁住 `EDGE_QUAD` 的保守行为

Guardrail:

- `SimulationMeshType::EDGE_QUAD` 在本 phase 显式保持 `1.0`
- 增加 negative test，防止未来误并入 tri/shell 或 volume 分支

### Risk 3: 初值非法时导致上层 solver 卡死

Guardrail:

- 对 `phi(0) <= eps` 的情况返回恢复性的极小正数
- 同时触发普通 `WARN`
- warning 至少带 `mesh type + element id + phi(0) + eps`
- 不返回严格 `0`
- 保持与现有 `CIPC::computeMaxStepSize()` 返回风格一致，但不允许静默吞掉非法初值

### Risk 3.25: `alpha_material < 1.0` 只能靠 grep 日志观测

Guardrail:

- `DeformationModelEnergy` 内部维护 `materialClampCount_` 或等价结构化统计
- Stage 3 的 smoke / regression 优先校验统计信号，而不是 grep 任意 stdout
- 计数只在真正返回 `< 1.0` 的 material max step 时递增

### Risk 3.5: 把 non-inversion 误当成 finiteness 保证

Guardrail:

- 文档与测试都明确：`Phase 1.5` 只保证 volume non-inversion，不保证 trial state 上 energy/gradient/hessian 一定 finite
- `NaN/Inf` 仍由上层 line search 的 finiteness check 兜底
- 这一点当前在 source plan 中仍按 repo-level 假设处理，未在本计划内强绑具体类名
- 若实现期核实发现上层 finiteness check 缺失，应在 `phase1.5.impl.md` 里显式记录，并单列 follow-up

### Risk 4: 把算法问题和 CLI 接线问题混在一起

Guardrail:

- 优先完成 `deformationModelEnergyMaxStep_gtest`
- `runIPCSim` 只做后置 smoke
- 不把 CLI test 缺失当作 Phase 1.5 算法 blocker

## Planned Deliverables

- `DeformationModelEnergy::computeMaxStepSize()` 的真正实现
- `tet/cubic` 两类材料可行性 helper
- `deformationModelEnergyMaxStep_gtest`
- `tests/src/core/solidDeformationModel/CMakeLists.txt` 更新
- `phase1.5.impl.md` 实现记录骨架
