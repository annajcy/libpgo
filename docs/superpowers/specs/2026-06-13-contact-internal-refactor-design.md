# Contact 内部重构设计

实施状态：phase 1 implementation plan 已拆分为 `docs/superpowers/plans/2026-06-13-contact-internal-refactor-implementation.md`。实现完成后，本 spec 中的 mapped wrapper、`StepDependentEnergy`、`EvaluationStateAwareEnergy` contact 路径应全部不存在于生产代码中。

日期：2026-06-13

## 背景

`libpgo` 当前 contact 模块已经同时包含 sampled penalty 和 IPC，但两者的结构来源不同：

- sampled penalty 继承自旧版 `src/core/contact` 的 handler/kernel 流程，核心是 sampled
  detection、external/self handler、child energy 构造与求值。
- IPC 引入了 active set、CCD max-step、line-search superset 等更完整的 optimizer 生命周期。
  当前 `libpgo` IPC 尚未实现 friction，第一版 IPC 重构只覆盖 normal barrier contact。

之前讨论过引入 `ContactTable` 来表达 participant 之间的 pairwise contact 语义。但 `self contact`
和 `external contact` 本身表达不了 `ContactTable` 想承载的完整语义，例如 material pair、局部 label、
多 body pair mask、method-specific 参数解释等。如果第一版同时引入 `ContactTable`，容易把“内部结构重构”
和“用户语义系统设计”绑在一起，导致范围过大。

因此第一版 spec 调整为：**不引入 `ContactTable` 体系，只做 contact 内部重构**。

## 第一版目标

1. 保持 sampled penalty 的算法行为尽量贴近旧版。
2. 明确 sampled penalty 与 IPC 的内部边界。
3. 拆清 pair detection、active set/cache、assembly、solver lifecycle 的职责。
4. 修正 sampled penalty 当前 line-search active set 冻结语义的问题。
5. 为后续 `ContactTable` 或更完整 contact scene 语义预留结构，但第一版不实现。
6. 尽量减少用户可见 API 变化，除非现有 API 明确导致错误行为。

## 第一版非目标

1. 不引入 `ContactTable`、`ContactElement`、`ContactParticipant` 或 per-pair `ContactModel`。
2. 不删除 `enable_self_contact` / `enable_external_contact`。
3. 不支持 material pair matrix、per-vertex label、per-face label 或 pairwise 参数表。
4. 不把 sampled penalty 改造成 IPC 风格的 barrier/non-penetration method。
5. 不要求 sampled penalty 和 IPC 共享同一个 pair representation。
6. 不统一 sampled penalty 与 IPC 的 Python API 语义；sampled penalty 的 normal/frictional 入口会在第一版收敛。
7. 不改变 `NonlinearOptimization::PotentialEnergy` 的外部契约。
8. 不覆盖旧版 self handler 暴露但旧仓库应用层未实际使用的扩展能力：CCD self contact、
   `setKeepPrevious()`、`setExcludedVertices()`、`setExcludedTriangles()`。这些能力第一版视为非目标；
   如果迁移中发现真实用户依赖，可作为 optional compatibility 单独补回。

## 设计原则

### 语义系统后置

第一版保留现有 sampled penalty 参数语义：

```python
SampledPenaltyParameters(
    stiffness=...,
    samples=...,
    enable_self_contact=True,
    enable_external_contact=True,
)
```

这些参数仍然是 coarse switch，不尝试表达复杂 pairwise 规则。它们的局限性留到第二版
`ContactTable` / scene-level 设计中解决。

### sampled penalty 行为贴近旧版

旧版 sampled penalty 的行为可以概括为：

```text
每次评估时：
  external handler / self handler 执行检测
  根据检测结果构造 PointPenetrationEnergy 或 PointTrianglePairCouplingEnergyWithCollision
  子 energy 直接计算 value / gradient / hessian
```

第一版 sampled penalty 应保持这个数值模型：

- penalty contact，不提供 IPC 式 non-penetration guarantee。
- 不参与 `computeMaxStepLimit()`。
- 不实现 `LineSearchAwareEnergy`。
- 不依赖 solver 预缓存 active set。
- line search trial evaluation 在 trial point 重新检测 active set。
- 保留 old handler/kernel 的检测和 child-energy 构造模式。
- self contact 第一版只保留旧仓库实际使用的 default DCD 路径：
  `execute(usurf.data()) -> handleContactDCD(0, 100) -> buildContactEnergy()`。

### IPC 保留现有 solver 协议

IPC 继续负责：

- `computeMaxStepLimit(x, dx)`：用 CCD 计算可行步长。
- `beginLineSearch(x, dx)`：构建 `[x, x + dx]` 上的 conservative active-set superset。
- trial evaluation：在 frozen superset 上按 trial positions 重新判断实际 contribution。

第一版重构不改变 IPC 的外部行为，但内部不保留 `SurfaceIPCCore` 作为 façade 兼容壳。
`IPCContactEnergy` 应直接组合新的 topology、pair generator、assembler 和 cache。

## 当前问题

### sampled penalty 的 line search 语义不正确

当前 sampled penalty 的 line-search active set 逻辑等价于：

```cpp
activeSetCache_.beginLineSearch(surfacePositions, buildActiveSet);
```

其中 `surfaceStep` 被忽略，因此冻结的是 `x` 处 exact active set，而不是
`[x, x + dx]` 的 conservative superset。

这会漏掉 line search trial 中新出现的 contact pair：

```text
x 处无接触
alpha = 0.7 时产生接触
但 frozen active set 不包含该 pair
trial energy 错误偏小
```

第一版修正方式：**sampled penalty 不实现 `LineSearchAwareEnergy`**。这样 line search 每个 trial
evaluation 都会走普通 evaluation 路径，在 trial point 重新检测 contact。代价是更慢，但语义正确，
也更贴近旧版 sampled penalty。

### sampled penalty 的内部 cache 边界不清

当前 sampled penalty 同时有 detector、active set cache、optional friction state 和 child energy，但它们的
lifecycle 与 IPC 的 active-set lifecycle 混在了 contact energy 层。为了完全贴近旧版，第一版 sampled
penalty 不保留 active-set cache；只把边界显式化：

- builder：负责检测并构造 sampled penalty 的 evaluation bundle。
- evaluation bundle：保存 external/self child energies 和必要 buffers。
- evaluator：消费 evaluation bundle 计算 value/gradient/hessian。
- public energy：实现 `PotentialEnergy`，管理 mapping 与 evaluation lifecycle。

### IPC core 职责过重

`SurfaceIPCCore` 当前同时承担 topology、active set generation、max-step、line-search superset、
energy assembly 等职责。第一版直接拆除这个 god object 边界，让调用方依赖新的具体组件。

## 第一版架构

### 共同 mapping 与继承边界

第一版 public contact energy 都以 **simulation displacement** 作为 `PotentialEnergy` 输入。surface-space
positions 是每个 contact energy 内部通过 DOF mapping 计算出来的 evaluation view，不再通过一层
mapping base class 暴露给 solver 或 Python。

现有 `EmbeddedDofMap` 已经承担了大部分 surface view 职责。第一版应将它升级或重命名为
`SurfaceDofMap`，作为普通成员组合进具体 contact energy：

```cpp
class SurfaceDofMap
{
public:
  VXd surfaceDisplacements(ConstRefVecXd simulationDisplacements) const;
  VXd surfacePositions(ConstRefVecXd simulationDisplacements) const;
  VXd pullbackGradient(ConstRefVecXd surfaceGradient) const;
  void pullbackHessian(const SpMatD &surfaceHessian, SpMatD &simulationHessian) const;

  int numSimulationDofs() const;
  int numSurfaceDofs() const;
  const std::vector<int> &simulationDofs() const;
};
```

因此第一版 contact 新架构不再引入或继续依赖这些抽象层：

```text
MappedContactEnergy
MappedSurfacePotentialEnergy
MappedEvaluationContactEnergy
MappedStepAwareContactEnergy
```

最终继承结构保持最小：

```cpp
class StatefulContactEnergy :
  public NonlinearOptimization::PotentialEnergy,
  public NonlinearOptimization::StepAwareEnergy
{
public:
  virtual ContactModelKind contactModelKind() const = 0;
  virtual bool isStepDependent() const { return false; }
  void beginStep(const NonlinearOptimization::StepState &) override {}
};
```

规则：

- `StepAwareEnergy` 是 solver lifecycle observer，默认 no-op；它不表示 energy 一定依赖上一时间步。
- `isStepDependent()` 是 step dependency 的唯一显式查询入口。
- 第一版删除 `StepDependentEnergy` marker class，不再通过 marker 继承判断 dependency。
- contact 新架构不使用 `EvaluationStateAwareEnergy`；IPC 的 exact active-set cache 与 line-search cache
  由 `IPCContactEnergy` 内部管理。
- `LineSearchAwareEnergy` 只由 IPC 实现；floor 和 sampled penalty 不实现。
- concrete contact energy 只能通过一条主链继承 `PotentialEnergy`，避免菱形继承。

第一版 concrete contact energy 结构：

```text
FloorContactEnergy
  : StatefulContactEnergy
  owns SurfaceDofMap

SampledPenaltyContactEnergy
  : StatefulContactEnergy
  owns SurfaceDofMap

IPCContactEnergy
  : StatefulContactEnergy
  : LineSearchAwareEnergy
  owns SurfaceDofMap
```

### floor

floor contact 当前继承 `MappedSurfacePotentialEnergy`。第一版改成和其他 contact energy 一样直接组合
`SurfaceDofMap`：

```cpp
class FloorContactEnergy : public StatefulContactEnergy
{
  SurfaceDofMap dofMap_;
  FloorPenaltyParameters params_;
};
```

evaluation 流程：

```text
func/gradient/hessian(u):
  x_surf = dofMap_.surfacePositions(u)
  compute floor penalty in surface space
  pull back surface gradient / Hessian through dofMap_
```

这只是结构调整，不改变 floor penalty 的数值模型。

### sampled penalty

建议保留 method-specific evaluation bundle，而不是强行转成 IPC 的 `PT/EE/PE/PP` pair set。

```cpp
struct SampledPenaltyEvaluationBundle
{
  std::unique_ptr<PointPenetrationEnergy> externalEnergy;
  std::unique_ptr<PointTrianglePairCouplingEnergyWithCollision> selfEnergy;
  // existing buffers / maps / diagnostics
};
```

建议拆成：

```text
src/core/contact/sampled_penalty/sampledPenaltyContactBuilder.*
src/core/contact/sampled_penalty/sampledPenaltyEvaluationBundle.*
src/core/contact/sampled_penalty/sampledPenaltyContactEvaluator.*
src/core/contact/sampled_penalty/sampledPenaltyContactEnergy.*
```

职责：

- `SampledPenaltyContactBuilder`
  - 持有或调用 existing external/self handlers。
  - 根据 `enable_self_contact` / `enable_external_contact` 决定是否执行对应 detection。
  - self detection 走 default DCD sampled self contact；不调用旧 handler 的 CCD overload。
  - 不暴露 `keepPrevious` 或 self exclusion 参数。
  - 构造 external/self child energies。
  - 输出 `SampledPenaltyEvaluationBundle`。
- `SampledPenaltyContactEvaluator`
  - 对 evaluation bundle 中的 child energies 求 value/gradient/hessian。
  - 不做 broad phase，不更新 evaluation bundle。
- `SampledPenaltyContactEnergy`
  - 实现 public simulation-space `StatefulContactEnergy`，输入是 simulation displacement。
  - 内部组合 `SurfaceDofMap`，把 simulation displacement 映射为 absolute surface positions。
  - 第一版不实现 `LineSearchAwareEnergy`。
  - 第一版不实现 contact-specific `computeMaxStepLimit()`。
  - 不再区分 normal 与 frictional 两个 sampled penalty 算法类；friction 是同一个 energy 的可选
    step-dependent mode。

建议结构：

```cpp
class SampledPenaltyContactEnergy : public StatefulContactEnergy
{
  SurfaceDofMap dofMap_;
  SampledPenaltyContactBuilder builder_;
  SampledPenaltyContactEvaluator evaluator_;
  std::optional<SampledPenaltyFrictionState> friction_;
};
```

public C++ factory 和 Python `SampledPenaltyEnergy` 返回的就是这个 simulation-space energy。Python 或
solver 不直接接触 surface-position child energy，也不再需要 mapping base class。

#### sampled penalty 位置语义

`SampledPenaltyContactBuilder` 的 public 输入语义固定为 **absolute surface positions**。原因是
`SampledPenaltyContactEnergy` 内部的 `SurfaceDofMap` 已经把 simulation displacement 映射成了
surface absolute positions：

```text
surface_positions = surface_rest_positions + W * simulation_displacements
```

因此 builder 不允许把 absolute positions 直接传给旧 handler 的 `double *` overload。如果继续复用旧
handler，第一版的 canonical 路径固定为：

```text
build_from_positions(surface_positions)
  call handler absolute-position overload
```

如果某个旧 handler 暂时只有 displacement overload，允许在 builder 内部提供一个很窄的兼容 adapter：

```text
build_from_positions(surface_positions)
  execute_displacement_handler_from_positions(surface_positions)
    displacement = surface_positions - surface_rest_positions
    call handler displacement overload
```

这个 displacement conversion 只能存在于 builder/handler adapter 边界内，不能泄漏到 energy、
evaluator、solver lifecycle 或 Python binding 层。

推荐第一版提供显式的 `build_from_positions(...)` / `execute_at_positions(...)` 语义，避免调用点误把
absolute positions 当 displacement。friction 的 `previous_x` 也按 absolute surface positions 存储和传递。

#### sampled penalty evaluation 构造次数

单项 evaluation 逻辑：

```text
func/gradient/hessian(u):
  x_surf = dofMap_.surfacePositions(u)
  bundle = builder.build_from_positions(x_surf)
  evaluator.compute(bundle, x_surf, ...)
```

fused evaluation 必须在一次 public call 内只构造一次 evaluation bundle：

```text
func_grad(u):
  x_surf = dofMap_.surfacePositions(u)
  bundle = builder.build_from_positions(x_surf)
  evaluator.func_grad(bundle, x_surf, ...)
  pull back gradient through dofMap_

gradient_hessian(u):
  x_surf = dofMap_.surfacePositions(u)
  bundle = builder.build_from_positions(x_surf)
  evaluator.gradient_hessian(bundle, x_surf, ...)
  pull back gradient / Hessian through dofMap_

func_grad_hessian(u):
  x_surf = dofMap_.surfacePositions(u)
  bundle = builder.build_from_positions(x_surf)
  evaluator.func_grad_hessian(bundle, x_surf, ...)
  pull back gradient / Hessian through dofMap_
```

也就是说，第一版不跨 call 缓存 bundle，但也不能依赖 `PotentialEnergy` 默认 fused implementation
去重复调用 `gradient` / `hessian` / `func`，否则一次 fused evaluation 会检测并构造三遍 child energy，
这会偏离旧版 fused evaluation 的行为和性能预期。

line search 中由于没有 frozen line-search state，每个 trial point 都会重新构造 evaluation bundle。

### sampled penalty friction mode

第一版只保留一个 `SampledPenaltyContactEnergy` 实现。它接收 optional friction 参数：

```cpp
struct SampledPenaltyContactEnergyOptions
{
  ParametersSpec params;
  std::optional<FrictionParametersSpec> friction;
};
```

语义：

- `friction == std::nullopt`：normal sampled penalty，child energies 的 `frictionCoeff` 为 0。
- `friction.has_value()`：`SampledPenaltyContactEnergy::beginStep()` 要求 previous simulation
  displacement 与正 timestep，内部映射并保存 previous surface positions；builder 构造 child energies
  时配置 friction coeff、previous-position function、velocity epsilon。

这更贴近旧版 kernel 设计：`PointPenetrationEnergy` 与
`PointTrianglePairCouplingEnergyWithCollision` 内部本来就包含 friction 分支，是否启用由
`frictionCoeff > 0` 和 step state 决定，而不是由两套 contact energy 算法决定。

step-dependent 语义按实例配置决定，而不是按 marker 继承决定：

- `friction == std::nullopt`：`beginStep()` 是 no-op，`isStepDependent()` 返回 false。
- `friction.has_value()`：`beginStep(previous_x, timestep)` 映射 previous simulation displacement，
  存储 previous surface positions，`isStepDependent()` 返回 true。

这样 C++ 仍然只有一个 sampled penalty 算法实现；虽然所有 contact energy 都继承 `StepAwareEnergy`，
non-friction sampled penalty 仍不会被误判为 step-dependent。要求 friction evaluation 前显式提供
`beginStep()` 状态，是相对旧版 raw kernel 的有意收紧：旧版 kernel 只看 `frictionCoeff > 0`，新架构
要求 previous positions / timestep 的生命周期更明确。

这属于 deliberate behavior tightening：数值模型仍然是旧版 sampled penalty friction，区别只是旧版通过
`setComputeLastPosFunction(...)` / `setToLastPosFunction(...)`、`setTimestep(...)`、`setVelEps(...)`
等 setter 隐式配置 step state；新 API 要求在启用 friction 后先调用
`begin_step(previous_x, timestep)`。如果缺少 previous positions 或 timestep，friction evaluation 应
明确报错或拒绝执行，而不是静默使用未初始化或 stale state。

### lifecycle 约束

`StepAwareEnergy` 和 `isStepDependent()` 的语义必须分开：

- `StepAwareEnergy::beginStep(...)`：solver 在每个 dynamic step 开始时广播 step state。
- `StatefulContactEnergy::isStepDependent()`：告诉 Python、diagnostics 或调度层 evaluation 是否真的依赖
  previous-step state。

所有 concrete contact energy 都可以接收 `beginStep(...)`，但大多数实现保持 no-op。当前第一版只有启用
friction 的 `SampledPenaltyContactEnergy` 返回 `isStepDependent() == true`。IPC normal barrier 和
floor penalty 返回 false；IPC 将来加入 friction 时再改变该语义。

### IPC

建议把 IPC 内部拆成：

```text
src/core/contact/ipc/ipcTopology.*
src/core/contact/ipc/ipcPairGenerator.*
src/core/contact/ipc/ipcContactAssembler.*
src/core/contact/ipc/ipcContactEnergy.*
```

职责：

- `IPCTopology`
  - surface topology、primitive adjacency、过滤规则、edge/triangle connectivity。
- `IPCPairGenerator`
  - 当前点 active set generation。
  - line-search swept superset generation。
  - max-step CCD query。
  - 第一版保留现有 `SelfPairSet` / `ExternalPairSet` 表达，不强行合并成 `PT/EE/PE/PP`
    统一 simplex pair set。
  - 对外提供一个统一入口，对内拆成 `SelfIPCPairGenerator` 与 `ExternalIPCPairGenerator`。
- `IPCContactAssembler`
  - 对给定 IPC active set 计算 value/gradient/hessian。
  - 管理或接收 normal barrier 参数。
  - 对外提供一个统一入口，对内保留 self/external 子 assembler。
- `IPCContactEnergy`
  - 实现 public simulation-space `StatefulContactEnergy`，并额外实现 `LineSearchAwareEnergy`。
  - 组合 DOF map、topology、generator、assembler、cache。
  - `beginStep()` 只处理 moving obstacle time 或清理必要 IPC cache；当前 normal barrier IPC
    `isStepDependent()` 返回 false。

第一版不保留 `SurfaceIPCCore` façade。允许在迁移过程中短暂保留底层 helper 函数或文件，但
`IPCContactEnergy` 的成员结构应直接是：

```cpp
SurfaceDofMap dofMap_;
IPCTopology topology_;
IPCPairGenerator pairGenerator_;
IPCContactAssembler assembler_;
IPCActiveSetCache activeSetCache_;
```

也就是说，旧 `SurfaceIPCCore` 的 public API 不作为新架构的一层继续存在。

### IPC 内部子模块

第一版不合并 self/external 的算法实现。推荐结构：

```text
IPCPairGenerator
  SelfIPCPairGenerator
  ExternalIPCPairGenerator

IPCContactAssembler
  SurfaceIPCSelfBarrierAssembler
  SurfaceIPCExternalBarrierAssembler
```

`IPCPairGenerator` 对调用方返回一个 `SurfaceIPCActiveSet`，但内部 self 与 external 仍分别负责：

- self：dynamic surface 内部 PT/EE broad phase、topology adjacency filter、CCD max-step。
- external：dynamic surface 与 obstacle surface 的 PT/TP/EE broad phase、obstacle pose/query、CCD max-step。

`IPCContactAssembler` 对调用方提供统一的 value/gradient/hessian 接口，但内部继续复用或迁移现有
`computeSelf*` 与 `computeExternal*` assembly 逻辑。

### IPC active set 表达

第一版保留：

```cpp
struct SurfaceIPCActiveSet
{
  EigenSupport::VXd positions;
  SelfPairSet selfPairs;
  ExternalPairSet externalPairs;
};
```

其中 `positions` 保留，用于 cache validation 和 diagnostics。assembler 仍应显式接收当前 evaluation
positions；不能把 active set 内部的 `positions` 当成唯一位置来源。line-search superset cache 中的
`positions` 表示 superset 的起点或构建参考点，而不是所有 trial alpha 的实际位置。

### IPC 与 libuipc 内部体系的对齐

第一版只对齐 libuipc 的内部 pipeline 分层，不照搬它的 `SimSystem`、GPU backend 或
`contact_tabular` 用户语义。

对应关系：

```text
libuipc GlobalVertexManager
  -> libpgo SurfaceDofMap + surface positions evaluation view

libuipc GlobalTrajectoryFilter + SimplexTrajectoryFilter
  -> libpgo IPCPairGenerator

libuipc filter_toi(alpha)
  -> libpgo IPCPairGenerator::compute_max_step(x, dx)

libuipc detect(alpha) / filter_active()
  -> libpgo build_active_set(x) 和 build_line_search_superset(x, dx)

libuipc SimplexNormalContact / IPCSimplexNormalContact
  -> libpgo IPCContactAssembler 的职责边界

libuipc SimplexFrictionalContact / IPCSimplexFrictionalContact
  -> phase 1 不对齐；作为未来 IPC friction 扩展参考

libuipc GlobalContactManager
  -> libpgo IPCContactEnergy 的 lifecycle/cache adapter；phase 1 不引入 contact table 语义
```

因此第一版 IPC 的核心拆分是：

```text
IPCPairGenerator
  owns active pair generation, swept line-search superset, CCD max-step

IPCContactAssembler
  owns normal barrier value, gradient, hessian assembly for a fixed active set

IPCContactEnergy
  owns solver lifecycle, cache selection, surface-to-simulation DOF mapping
```

这里的边界与 libuipc 的 `TrajectoryFilter -> ContactReporter -> engine lifecycle` 同构，但落在
`libpgo` 现有 `PotentialEnergy` 和 solver protocol 之内。

第一版不把 `SurfaceIPCActiveSet` 改成 libuipc 风格的统一 simplex pair set。原因是当前 `libpgo`
IPC 只支持 self PT/EE 与 external PT/TP/EE；如果为了形式统一而把 external TP 临时翻译成 PT、
再让 PE/PP 为空，会让 pair 表达看起来更统一，但实际语义不更清楚。第一版应优先保证数值行为等价和
职责边界清晰。统一 geometry space、PE/PP pair、contact element id 和 pairwise contact model
留到第二版语义系统设计时一起处理。

## solver lifecycle 约定

### 普通 evaluation

contact 新架构不依赖 solver 预先准备 evaluation state。普通 evaluation 由 concrete contact energy
自己完成 surface mapping 与 method-specific cache 选择。

IPC active-set cache 选择顺序必须固定为：

```text
func/gradient/hessian(u):
  x_surf = dofMap_.surfacePositions(u)
  activeSet = activeSetForEvaluation(x_surf)

activeSetForEvaluation(x_surf):
if line-search active set exists:
    use line-search superset with current surface positions
else if exact active set cache matches x_surf:
    use cached exact active set
else:
    build exact active set at x_surf
```

`beginLineSearch()` 不应覆盖 exact active set cache；`endLineSearch()` 只清空 line-search superset。
这样普通 evaluation cache 与 line-search cache 的生命周期保持独立。

sampled penalty 第一版不使用跨 call active-set cache：

```text
func/gradient/hessian(u)
  x_surf = dofMap_.surfacePositions(u)
  builder.build_from_positions(x_surf)
  evaluator.compute(...)
```

### IPC line search

```text
computeMaxStepLimit(x, dx)
  map simulation displacement trajectory to surface trajectory
  compute feasible alpha by CCD

beginLineSearch(x, scaled_dx)
  map simulation displacement segment to surface segment
  build conservative active-set superset over [x_surf, x_surf + scaled_dx_surf]

func(x + alpha * scaled_dx)
  map trial simulation displacement to trial surface positions
  use frozen superset
  recompute actual distance/activation at trial point

endLineSearch()
  clear frozen superset
```

### sampled penalty line search

```text
beginLineSearch(...)
  not implemented

func(x + alpha * dx)
  rebuild sampled penalty evaluation bundle at trial point
```

这个设计牺牲一些速度，但避免错误冻结 active set。

## Python/API 影响

第一版不引入 `ContactTable` / participants，但 sampled penalty Python API 做一个明确收敛：
normal 与 frictional sampled penalty 不再是两个 energy facade。统一入口为：

```python
energy = SampledPenaltyEnergy(
    surface=surface,
    surface_triangles=triangles,
    params=SampledPenaltyParameters(
        stiffness=1e4,
        samples=4,
        enable_self_contact=True,
        enable_external_contact=True,
    ),
    friction=None,
)
```

启用 friction 时：

```python
energy = SampledPenaltyEnergy(
    surface=surface,
    surface_triangles=triangles,
    params=SampledPenaltyParameters(
        stiffness=1e4,
        samples=4,
        enable_self_contact=True,
        enable_external_contact=True,
    ),
    friction=FrictionParameters(
        friction_coeff=0.4,
        velocity_eps=1e-3,
    ),
)
```

删除或废弃：

```python
FrictionalSampledPenaltyEnergy(...)
```

语义：

- `friction is None`：normal sampled penalty；底层 C++ energy 的 `beginStep()` 为 no-op，Python facade
  如保留 `begin_step()` 也为 no-op，`is_step_dependent()` 返回 false。
- `friction is not None`：sampled penalty 启用 optional friction mode，`begin_step(previous_x, timestep)`
  必须在 evaluation 前提供 previous simulation displacement 和正 timestep；energy 内部映射为 previous
  surface positions。
- `SampledPenaltyParameters` 继续保留 `enable_self_contact` / `enable_external_contact`，第一版不迁移到
  pairwise contact semantics。
- `FrictionParameters` 保留为 value object，但不再对应一个单独 energy class。

C++ binding 也应收敛为一个 factory：

```text
_create_sampled_penalty_contact_energy(..., friction: Optional[FrictionParameters])
```

不再暴露：

```text
_create_frictional_sampled_penalty_contact_energy(...)
PyFrictionalSampledPenaltyContactEnergy
```

IPC 入口保持现状。若内部重构需要新增高级参数，应优先放在 method-specific params 中，不引入
participant/table 概念。

## 与第二版 ContactTable 的关系

第一版不设计 `ContactTable` API，但内部边界应避免阻碍第二版：

- sampled penalty builder 不应直接依赖 Python 参数对象。
- evaluator 不应知道 self/external 开关。
- IPC pair generation 和 assembly 应逐步分离。
- diagnostics 中可以开始记录 pair source，例如 `self` / `external` / `ipc_pt` / `ipc_ee`。

sampled penalty 的 optional friction state 第一版不拆成独立 manager。它可以保留在
`SampledPenaltyContactEnergy` 内部；本轮只要求它不依赖 evaluation-bundle cache，不继承错误的
line-search freeze 语义。

第一版 diagnostics 只要求 C++ 内部可见，不作为 Python public API 暴露。建议至少记录：

- sampled penalty self/external active count。
- IPC self PT/EE count。
- IPC external PT/TP/EE count。
- IPC line-search superset count。
- IPC max-step alpha。

第二版如果引入 `ContactTable`，可以替换 detector 的 coarse enable source：

```text
第一版：
  enable_self_contact / enable_external_contact

第二版：
  pairwise contact policy / material pair policy / participant labels
```

但第二版不应要求 sampled penalty 改变其基本 penalty behavior。

## 测试计划

1. `SurfaceDofMap` 的 `surfaceDisplacements()` / `surfacePositions()` 与现有 mapping 结果一致。
2. `SurfaceDofMap::pullbackGradient()` 与 `pullbackHessian()` 的结果与显式 `W.transpose()` /
   `W.transpose() * H * W` 一致。
3. floor contact 改为组合 `SurfaceDofMap` 后，value、gradient、Hessian 与重构前一致。
4. sampled penalty 在非 line-search evaluation 中与旧版/当前行为数值一致。
5. sampled penalty line search trial 点会重新构造 evaluation bundle。
6. sampled penalty 不参与 `computeMaxStepLimit()`，返回默认 alpha。
7. `enable_self_contact=False` 时 self handler 不执行或不贡献能量。
8. `enable_external_contact=False` 时 external handler 不执行或不贡献能量。
9. sampled penalty optional friction mode 在 step lifecycle 中保持现有行为。
10. sampled penalty Python API 统一为 `SampledPenaltyEnergy(..., friction=None | FrictionParameters(...))`，
   `friction is None` 时 `is_step_dependent()` 为 false，`friction is not None` 时必须先提供
   `begin_step(previous_x, timestep)`。
11. 删除 `StepDependentEnergy` marker 后，C++/Python 测试不再用 `dynamic_cast<StepDependentEnergy *>`
   判断 step dependency，而是检查 `StatefulContactEnergy::isStepDependent()` /
   Python `is_step_dependent`。
12. contact 新架构中的 floor、sampled penalty、IPC 不再通过 `EvaluationStateAwareEnergy` 暴露
    contact evaluation cache；IPC cache 只通过 `IPCContactEnergy` 内部路径验证。
13. 不再暴露 `FrictionalSampledPenaltyEnergy`、`_create_frictional_sampled_penalty_contact_energy`、
   `PyFrictionalSampledPenaltyContactEnergy`；examples 和 pypgo tests 同步迁移到统一入口。
14. IPC max-step 行为与重构前一致。
15. IPC line-search superset 行为与重构前一致。
16. IPC 普通 active set 与 line-search active set cache 分离。
17. sampled penalty 与 IPC 可以同时作为 energies 加入 solver，生命周期互不干扰。
18. repeated `func/gradient/hessian` 调用会各自重新执行 sampled penalty builder，不复用跨 call
    evaluation bundle。
19. IPC `IPCPairGenerator` 的 self/external 子 generator 分别可测试，合并后的 `SurfaceIPCActiveSet`
    pair count 与重构前一致。
20. IPC `IPCContactAssembler` 的 self/external 子 assembler 分别可测试，总能量、梯度、Hessian 与重构前一致。
21. diagnostics 至少在 C++ 测试或 profiling 中可访问，不要求 Python API。

## 推荐实施顺序

1. `[S][low-risk]` 升级或重命名 `EmbeddedDofMap` 为 `SurfaceDofMap`，补齐 gradient/Hessian pullback
   与 DOF query，并增加独立测试。
2. `[S][low-risk]` 删除 `StepDependentEnergy` marker，新增 `StatefulContactEnergy::isStepDependent()`，
   让 `StatefulContactEnergy` 继承 `PotentialEnergy + StepAwareEnergy` 且默认 `beginStep()` no-op。
3. `[S][low-risk]` refactor floor contact：移除对 mapped base class 的继承，直接组合 `SurfaceDofMap`，
   保持数值行为不变。
4. `[S][low-risk]` 为 sampled penalty 增加测试，锁定旧版/当前 value、gradient、hessian 行为。
5. `[S][med-risk]` 移除 sampled penalty 的 `LineSearchAwareEnergy` 暴露，确保 line search trial 重新检测。
6. `[M][med-risk]` 把 sampled penalty evaluation bundle 构造与 value/gradient/hessian evaluation
   拆成 builder/evaluator。
7. `[S][med-risk]` 移除 sampled penalty evaluation-bundle cache 路径，`func/gradient/hessian`
   每次调用现场构造。
8. `[M][med-risk]` refactor sampled penalty：`SampledPenaltyContactEnergy` 直接组合 `SurfaceDofMap`，
   public factory 返回 simulation-space energy，不再返回 mapping adapter。
9. `[M][med-risk]` 收敛 sampled penalty C++ factory 与 Python API：把 normal/frictional 合并为
   `createSampledPenaltyEnergy(..., std::optional<FrictionContactSpec>)` /
   `SampledPenaltyEnergy(..., friction=None | FrictionParameters(...))`，删除单独 frictional factory、
   binding handle、Python facade，并同步更新 tests/examples。
10. `[M][med-risk]` 新增 `IPCPairGenerator`，迁移 active set generation、line-search superset 和 max-step 入口。
11. `[M][med-risk]` 新增 `IPCContactAssembler`，迁移 IPC value/gradient/hessian assembly 入口。
12. `[M][high-risk]` 修改 `IPCContactEnergy` 直接组合 `SurfaceDofMap`、`IPCTopology`、`IPCPairGenerator`、
    `IPCContactAssembler` 和 `IPCActiveSetCache`，移除对 `SurfaceIPCCore` 的依赖。
13. `[S][med-risk]` 删除或停用 contact 新架构中的 mapped energy base class 与 factory 路径，确保
    floor、sampled penalty、IPC 都直接实现 simulation-space public energy。
14. `[S][low-risk]` 增加 C++ 内部 contact diagnostics，区分 sampled self/external 与 IPC primitive pair counts。
15. `[M][med-risk]` 在第二版 spec 中重新讨论 `ContactTable` / participant / pairwise material semantics。

## 开放问题

1. 第二版 `ContactTable` 是否应该从 sampled penalty 的真实需求倒推，而不是先设计完整 scene 语义？
