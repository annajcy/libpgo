# Contact API Refactor Plan

> **状态日期：** 2026-06-04
> **适用范围：** C++ `contact/` construction boundary + Python `pypgo.contact` binding.
> **执行约束：** 不重写 IPC barrier、CCD、sampled penalty 数值 kernel、active-set 数值逻辑或 floor/contact 能量公式。本计划只重构 contact energy 的长期构造边界、ownership、obstacle lifecycle、stateful contact energy contract、Python API、以及与 `EnergySet` / solver service 的对接方式。
> **并行约束：** 本计划后续实现中，凡遇到可表达为简单 `parallel_for` / range loop / 三维逐点循环的 TBB 或 OpenMP 并行需求，统一使用 `src/core/parallelism` 的 facade API（当前命名空间为 `pgo::parallel`，如 `pgo::parallel::parallelFor*`），contact 业务模块不得新增直接 `#include <tbb/...>`、`tbb::parallel_for` 或 `#pragma omp parallel for`。如果 contact kernel 确实需要 `parallel_reduce`、TLS、concurrent containers、锁、custom partitioner 等复杂 TBB/OpenMP 模式，先为 `core/parallelism` 增加窄抽象，或在本 plan 中明确记录为 scoped exception。
>
> ## 依赖 plan 的实施状态（2026-06-04）
>
> 本 plan 依赖 4 个姊妹 plan。time integrator plan 已在 contact plan 之前实施完成，
> 其间落地了部分 contact plan 所需的基础设施。下表标注哪些已经就绪：
>
> | 交叉决策 / 依赖 | 来源 | 状态 |
> |---|---|:-:|
> | `StepAwareEnergy` + `StepState` under `NonlinearOptimization`（§18.1）| time integrator T1 | ✅ |
> | `EnergyStateKind` + `PotentialEnergy::stateKind()`（§18.3）| energy plan E1 | ✅ |
> | `PotentialEnergy` / `LineSearchAwareEnergy` 角色分离（§18.2）| 既有 | ✅ |
> | `EnergySet`（原 `PotentialEnergies`）| energy plan | ✅ |
> | `minimize(problem, x0, NewtonOptions)` + `FixedVariables` | solver plan | ✅ |
> | `SolverControl` | solver plan | ✅ |
> | `QuadraticPotentialEnergy::setLinearTerm/setAValues`（D2）| time integrator | ✅ |
> | `acceptsDynamicSolveStatus` | 既有 | ✅ |
> | `EvaluationStateAwareEnergy` evaluation-state prepare hook（§18.7）| contact C2 | ✅ |
> | `StepDependentEnergy` marker（§18.8）| contact C2 | ✅ |
> | `EmbeddedDofMap`（§18.4）| contact C2 | ✅ |
> | `StatefulContactEnergy`（§4）| contact C2 | ✅ |
> | Obstacle hierarchy（§7）| contact C3 | ✅ |
> | `IPCContactEnergy`（§5）| contact C5 | ✅ |
> | `SampledPenaltyContactEnergy` normal-only model（§6）| contact C6 | ✅ |
> | `FrictionalSampledPenaltyContactEnergy` step-dependent model（§6）| contact C6 | ✅ |
> | `runIPCSim` contact setup migration（§18.5 / C8 scoped）| contact C8 | ✅ |
>
> **过渡期设计：** `runIPCSim` loop 当前（time integrator plan 之后）直接使用
> `ImplicitEulerStepper`，每帧重建 stepper。`ImplicitEulerStepper::step()` 内已有
> `dynamic_cast<StepAwareEnergy*>` 的 per-term dispatch 循环，对尚未成为
> `StepAwareEnergy` 的 contact energy 是空转。contact plan 产出
> `StatefulContactEnergy` 后，该循环自动生效，无需改 stepper 代码。
>
> **实施记录（2026-06-04）：**
>
> - C3/C4 已按 typed obstacle hierarchy 落地：`ObstacleSurface` 是抽象只读基类，`StaticObstacleSurface` / `MovingObstacleSurface` / `LinearMovingObstacleSurface` / `TrajectoryObstacleSurface` 承担具体构造与时间更新；`SurfaceIPCCore` 只拥有 `std::unique_ptr<ObstacleSurface>`，内部拆成 static/moving typed buckets，通过 `ObstacleSurfaceView` 把 broad phase、external assembler、external max-step 与 ownership 解耦。
> - C5 已直接 rename/migrate 到 `IPCContactEnergy`，不保留新的 wrapper 路径。
> - C6 已拆成 normal `SampledPenaltyContactEnergy` 与 frictional `FrictionalSampledPenaltyContactEnergy`；后者实现 `StepDependentEnergy`。
> - Sampled penalty 底层数值 kernel 已从旧 `legacy_penalty` 目录迁到 `src/core/contact/sampled_penalty/kernels`；C API、runSim shared tests、CMake 和 long-lived sampled penalty energy 均使用新 include 路径。
> - C8 第一版按 scoped migration 完成：移除 `--legacy`，`ContactBackendKind::LegacyPenalty` 改为 `SampledPenalty`，`obstacleSetup` 直接生产 typed obstacle ownership；dynamic loop 使用 long-lived stateful contact energy。完整 `SimulationProblem` / `SimulationRuntime` 大拆分可作为后续 runIPCSim 架构清理，不再阻塞本 contact API refactor。
> - C9 Python facade 已补齐 `ContactVertexEmbedding`、`FloorParameters`、`ObstacleSpec.linear_velocity(reference_time=...)`、`FloorEnergy.set_height(...)` 与 public `__all__` surface；public API 不暴露 raw C++ contact internals。

> **生命周期决策更新（2026-06-05）：**
>
> 手动 `refreshActiveSet(...)` / `clearActiveSet()` 公开协议已废弃并从 C++/Python public API 移除。`StatefulContactEnergy` 现在只表示 common contact boundary；需要 active-set cache 的 IPC 与 sampled penalty 通过内部 `ActiveSetContactEnergy` 接收 solver-owned `prepareEvaluationState(x)` / line-search hooks。直接 `func` / `gradient` / `hessian` 与 Python `value` / `gradient` / `hessian` 会按状态自动构建或复用 active set。本文后续历史段落中要求 Python/manual caller 显式 `refresh_active_set(x)` 的说法均已被本决策取代。

## 目标

建立一个长期可复用的 C++ contact construction facade，并把 IPC 与 sampled penalty 都提升为一等 `StatefulContactEnergy`：

```cpp
namespace pgo::NonlinearOptimization
{
struct StepState;
class StepAwareEnergy;
enum class EnergyStateKind;
class EvaluationStateAwareEnergy;
class StepDependentEnergy;
}  // namespace pgo::NonlinearOptimization

namespace pgo::Contact
{

struct ContactSurfaceSpec;
struct ContactVertexEmbedding;
struct FloorSpec;
using ContactStepState = NonlinearOptimization::StepState;
struct StaticObstacleSpec;
struct LinearMovingObstacleSpec;
using ObstacleSpec = std::variant<StaticObstacleSpec, LinearMovingObstacleSpec>;
class EmbeddedDofMap;
class ContactSurfaceAdapter;

enum class ContactModelKind
{
  Floor,
  IPC,
  SampledPenalty,
};

class StatefulContactEnergy;

std::shared_ptr<EmbeddedSurfaceFloorPotentialEnergy> createFloorEnergy(
  const ContactSurfaceSpec &surface,
  const FloorSpec &floor);

namespace IPC
{
struct ParametersSpec;
class IPCContactEnergy;

std::shared_ptr<IPCContactEnergy> createIPCEnergy(
  const ContactSurfaceSpec &surface,
  const EigenSupport::MXi &surfaceTriangles,
  const ParametersSpec &params,
  std::vector<ObstacleSpec> obstacles);
}  // namespace IPC

namespace SampledPenalty
{
struct ParametersSpec;
struct FrictionParametersSpec;
class SampledPenaltyContactEnergy;
class FrictionalSampledPenaltyContactEnergy;

std::shared_ptr<SampledPenaltyContactEnergy> createSampledPenaltyEnergy(
  const ContactSurfaceSpec &surface,
  const ParametersSpec &params,
  std::vector<ObstacleSpec> obstacles = {});

std::shared_ptr<FrictionalSampledPenaltyContactEnergy> createFrictionalSampledPenaltyEnergy(
  const ContactSurfaceSpec &surface,
  const ParametersSpec &normalParams,
  const FrictionParametersSpec &frictionParams,
  std::vector<ObstacleSpec> obstacles = {});
}  // namespace SampledPenalty

}  // namespace pgo::Contact
```

Python、未来 `RunSimConfig`、以及 C++ examples 都通过 facade 构造 contact energies，不直接拼 `FloorPenaltyParameters`、`SurfaceIPCCore::Parameters`、sampled penalty handlers、obstacle sampler 或 IPC core lifecycle。

Python 第一版：

```python
import pypgo as pgo

surface_embedding = pgo.contact.ContactVertexEmbedding(
    indices=embedding_indices,
    weights=embedding_weights,
    arity=embedding_arity,
)

surface = pgo.contact.ContactSurface(
    rest_vertices=surface_rest,
    surface_from_simulation_disp_map=A,
    vertex_embedding=surface_embedding,  # required by SampledPenaltyEnergy
)

floor = pgo.contact.FloorEnergy(
    surface,
    axis="y",
    side="keep_above",
    height=0.0,
    kappa=1e6,
)

ipc = pgo.contact.IPCEnergy(
    surface,
    surface_triangles=triangles,
    params=pgo.contact.IPCParameters(dhat=1e-3),
    obstacles=[
        pgo.contact.ObstacleSpec.static(obs_rest, obs_tris),
        pgo.contact.ObstacleSpec.linear_velocity(
            moving_rest,
            moving_tris,
            velocity=[0.0, -1.0, 0.0],
        ),
    ],
)

ipc.begin_step(time=t, timestep=dt, previous_x=u_prev)
ipc.refresh_active_set(u0)

sampled_penalty = pgo.contact.SampledPenaltyEnergy(
    surface,
    params=pgo.contact.SampledPenaltyParameters(
        stiffness=1.0,
        samples=2,
    ),
    obstacles=[pgo.contact.ObstacleSpec.static(obs_rest, obs_tris)],
)
sampled_penalty.begin_step(time=t, timestep=dt, previous_x=u_prev)
sampled_penalty.refresh_active_set(u0)

frictional_penalty = pgo.contact.FrictionalSampledPenaltyEnergy(
    surface,
    params=pgo.contact.SampledPenaltyParameters(
        stiffness=1.0,
        samples=2,
    ),
    friction=pgo.contact.FrictionParameters(
        friction_coeff=1.0,
        velocity_eps=1.0,
    ),
    obstacles=[pgo.contact.ObstacleSpec.static(obs_rest, obs_tris)],
)
frictional_penalty.begin_step(time=t, timestep=dt, previous_x=u_prev)
frictional_penalty.refresh_active_set(u0)

total = pgo.energy.EnergySet([(elastic, 1.0), (floor, 1.0), (ipc, 1.0)])
result = pgo.solver.solve_newton(total, x0=u0, fixed_dofs=fixed)
```

上面的代码同时展示 IPC 和 sampled penalty 的 construction surface；实际 simulation/solve 通常选择其中一个 contact model 放入 `EnergySet`。

Python 用户不看到：

- `EmbeddedDofMap` / `ContactSurfaceAdapter` / mapping/cache lifecycle；
- `SurfaceIPCCore`；
- sampled penalty handler classes；
- `ObstacleSurface` / `StaticObstacleSurface` / `MovingObstacleSurface` raw classes；
- obstacle sampler / pose cache construction；
- `FloorPenaltyParameters`、`SurfaceIPCCore::Parameters` raw structs；
- `PointPenetrationEnergy` / `PointTrianglePairCouplingEnergyWithCollision` active term ownership；
- `markObstacleStatic(objectId)`；
- `setObstacleTime(...)` / `set_obstacle_time(...)` compatibility API；
- `hessian` / `hessianDirect` / `createHessian` allocation details；
- any raw pointer / borrowed-reference construction rule。

## 当前问题

相关文件：

- `src/core/contact/mappedSurfacePotentialEnergy.h/.cpp`
- `src/core/contact/embeddedSurfaceFloorPotentialEnergy.h/.cpp`
- `src/core/contact/ipc/embeddedSurfaceIPCPotentialEnergy.h/.cpp`
- `src/core/contact/ipc/core/surfaceIPCCore.h/.cpp`
- `src/core/contact/ipc/external/obstacleSurface.h/.cpp`
- `src/core/contact/ipc/broadPhase/surfaceIPCExternalBroadPhase.cpp`
- `src/core/contact/ipc/core/surfaceIPCExternalBarrierAssembler.*`
- `src/core/contact/ipc/core/surfaceIPCMaxStep.*`
- `src/core/contact/sampled_penalty/kernels/*`（本计划重命名为 sampled penalty contact model；不是 legacy backend）
- `src/tools/sim/runIPCSim/contact/legacyPenaltyContact.*`
- `src/tools/sim/runIPCSim/contact/contactBackend.h`
- `src/tools/sim/runIPCSim/setup/obstacleSetup.*`
- `src/tools/sim/runIPCSim/setup/shellSetup.cpp`
- `src/tools/sim/runIPCSim/setup/volumeSetup.cpp`
- `src/python/pypgo/bindings/contact_bindings.cpp`（新增）

### 1. 没有长期 construction facade

当前 Python plan 如果直接绑定 `EmbeddedSurfaceFloorPotentialEnergy` / `EmbeddedSurfaceIPCPotentialEnergy` ctor，会让 Python binding、`RunSimConfig`、examples 各自重复翻译：

- NumPy / sparse map 到 surface adapter；
- floor axis / side / kappa / height；
- IPC parameters；
- obstacle mesh、static/moving 类型、pose cache 初始化、time update。

construction-level policy 应该集中在一个小 facade，而不是散在每个 caller。

### 2. `legacy_penalty` 实际是 sampled penalty contact model

当前 `contact/sampled_penalty/kernels/*` 不是应该藏起来的旧路径，而是一套和 IPC 平行的 contact model：

- 它使用 triangle sampling、sample embedding、DCD/CCD contact detection 生成 active contact pairs；
- external contact 的 active term 是 `PointPenetrationEnergy`；
- self contact 的 active term 是 `PointTrianglePairCouplingEnergyWithCollision`；
- 当前 `runIPCSim` 每步 `execute(...) -> buildContactEnergy() -> addGeneralImplicitForceModel(...)`，但这只是 runtime wiring，不应该决定长期 API。

本计划把它重命名为 sampled penalty contact model，并把它重构成长生命周期 normal-only `SampledPenaltyContactEnergy` 与 frictional `FrictionalSampledPenaltyContactEnergy`。二者和 IPC 一样进入 `EnergySet`，区别只在 active set 如何刷新、是否需要 step history、以及 eval kernel 如何配置 active terms。

### 3. State convention 必须固定为 simulation displacement

`MappedSurfacePotentialEnergy::func(simulationDisplacements)` 的 state 是 simulation displacement，不是 simulation absolute position，也不是 surface displacement。内部用 `surfaceFromSimulationDispMap_` 把 simulation 位移映射到 surface 位移，再叠加 rest position 得到 surface 绝对位置。

Python `ContactSurface` 和所有 contact energies 都声明：

```text
state_kind == "displacement"
```

这样可以直接组合进 deformation energy 的 `EnergySet`，不会产生 mixed-state ambiguity。

这个信息不应该只是 Python wrapper 上的字符串常量。`EnergyStateKind` / `PotentialEnergy::stateKind() const` 由 `energy_api_refactor.plan.md` 的 E1 引入；本计划只消费并 override 这个 C++ enum。Contact energy、deformation energy、vertex pulling/attachment 这类位移态能量返回 `EnergyStateKind::Displacement`；generic linear/quadratic energy 返回 `EnergyStateKind::Generic`。

### 4. Hessian topology 非固定

`MappedSurfacePotentialEnergy::isHessianTopologyFixed() == 0`。当前 `hessian(x, H)` / `createHessian(H)` 会抛，真正路径是 `hessianDirect(x, H)`。Energy plan E0/E1 会统一 naming 和 `evaluateHessian` helper；Contact binding 必须走 helper，不直接调用 topology-fixed path。

Contact plan 不单独 rename energy API，但需要在 tests 中覆盖 `FloorEnergy.hessian(u)` / `IPCEnergy.hessian(u)` 走动态 Hessian 路径。

### 5. 当前 obstacle lifecycle 有半初始化状态

当前 `ObstacleSurface` 构造后 `current_` 是 zero，真正 pose cache 只有 `update(t)` 后才建立。static obstacle 通过 `markObstacleStatic(objectId)` 间接 `update(0.0)`，moving obstacle 需要 caller 记得 `setObstacleTime(t)`。

这个设计把几件事混在一起了：

- static obstacle 不应该有 “sample at t = 0” 语义；
- moving obstacle 的时间推进不应该污染 static obstacle；
- `markObstacleStatic(objectId)` 暴露了 slot-index lifecycle；
- construction 后对象不是 ready-to-evaluate；
- `setObstacleTime` 名字太宽，实际只应该推进 moving obstacles。

本计划把 obstacle 重构成 static/moving 类型体系，construction 后 immediately ready。

### 6. Contact energy 有 mutable cache，不是线程共享纯函数

`IPCContactEnergy`、`SampledPenaltyContactEnergy` 和 `FrictionalSampledPenaltyContactEnergy` 都有 mutable active-set cache 和 line-search active-set state。Python 文档必须说明：

- 可以把同一个 energy object 放进一个 solve/evaluation pipeline；
- 不承诺同一个 contact energy instance 可被多个线程并发 evaluation / solve；
- 如需并发，应构造独立 energy instances。

这和 solver plan 的 `LineSearchAwareEnergy` freeze 规则相互依赖：line search policy 只负责 alpha，contact energy 自己管理 line-search active set。

### 7. Contact energy 不是 hard constraints

Floor / IPC 是 `PotentialEnergy`，不是 `ConstraintFunctions`。不要把 bbox/floor/contact 混入 `pypgo.constraints`。如果未来需要 hard geometric constraints，另走 constraints plan。

## 非目标

- 不重写 IPC barrier、CCD、broad phase、active-set construction、friction-related internals。
- 不重写 sampled penalty contact 数值 kernel。`PointPenetrationEnergy`、`PointTrianglePairCouplingEnergyWithCollision`、triangle sampling 和 collision detection 先作为 implementation kernel 保留。
- 不保留 `legacy` 命名作为长期 public API；`legacy_penalty` 迁移为 sampled penalty contact model。
- 不暴露 `MappedSurfacePotentialEnergy` subclassing 到 Python。
- 不暴露 `StatefulContactEnergy` subclassing 到 Python。
- 不暴露 `SurfaceIPCCore` raw API。
- 不暴露 sampled penalty handler raw API。
- 不暴露 obstacle concrete classes、sampler、pose cache 到 Python。
- 不暴露 `PointPenetrationEnergy` / `PointTrianglePairCouplingEnergyWithCollision` active term objects 到 Python。
- 不保留 `markObstacleStatic(objectId)` public lifecycle。
- 不保留 `setObstacleTime(...)` / `set_obstacle_time(...)` compatibility method；统一迁移到 `setMovingObstacleTime(...)` / `set_moving_obstacle_time(...)`。
- 不保留 `runIPCSim --legacy` 作为长期 CLI；直接移除该 alias，迁移到 `--contact-model sampled-penalty`。
- 不为 moving floor / surface pressure 设计 first-class Python class；列入 future work。
- 不让 contact energy 进入 `pypgo.constraints`。
- 不承诺 `IPCEnergy` instance 的 concurrent evaluation thread-safety。
- 不改变既有 `runIPCSim` JSON 字段含义；本 plan 明确列出的 CLI 变更（移除 `--legacy`）和 sampled penalty static external-only/frictional split 是有意行为变更，必须同步测试和文档。

## 关键设计决策

### 1. C++ 长期边界是 common `contactEnergyFactory`

新增：

- `src/core/contact/contactEnergyFactory.h`
- `src/core/contact/contactEnergyFactory.cpp`
- `src/core/contact/statefulContactEnergy.h`
- `src/core/contact/embeddedDofMap.h`
- `src/core/contact/contactSurfaceAdapter.h`

核心 public API：

```cpp
namespace pgo::NonlinearOptimization
{
// EnergyStateKind and PotentialEnergy::stateKind() are defined by
// energy_api_refactor.plan.md E1. Contact consumes that shared enum.

struct StepState
{
  double time = 0.0;
  double timestep = 0.0;
  const EigenSupport::VXd *previousX = nullptr;
};

class StepAwareEnergy
{
public:
  virtual ~StepAwareEnergy() = default;
  virtual void beginStep(const StepState &state) = 0;
};

class EvaluationStateAwareEnergy
{
public:
  virtual ~EvaluationStateAwareEnergy() = default;
  virtual void prepareEvaluationState(EigenSupport::ConstRefVecXd x) const = 0;
};

class StepDependentEnergy
{
public:
  virtual ~StepDependentEnergy() = default;
};
}  // namespace pgo::NonlinearOptimization

namespace pgo::Contact
{

struct ContactSurfaceSpec
{
  EigenSupport::MXd restVertices;
  EigenSupport::SpMatD surfaceFromSimulationDispMap;
  std::optional<ContactVertexEmbedding> vertexEmbedding = std::nullopt;
};

struct ContactVertexEmbedding
{
  std::vector<int> vertexEmbeddingIndices;
  std::vector<double> vertexEmbeddingWeights;
  int embeddingArity = 0;
};

struct FloorSpec
{
  FloorAxis axis = FloorAxis::INVALID;
  FloorSide side = FloorSide::KEEP_ABOVE;
  double height = std::numeric_limits<double>::quiet_NaN();
  double kappa = std::numeric_limits<double>::quiet_NaN();
};

using ContactStepState = NonlinearOptimization::StepState;

struct StaticObstacleSpec
{
  EigenSupport::MXd restVertices;
  EigenSupport::MXi triangles;
};

struct LinearMovingObstacleSpec
{
  EigenSupport::MXd restVertices;
  EigenSupport::MXi triangles;
  EigenSupport::V3d velocity = EigenSupport::V3d::Zero();
  double referenceTime = 0.0;
};

using ObstacleSpec = std::variant<StaticObstacleSpec, LinearMovingObstacleSpec>;

enum class ContactModelKind
{
  Floor,
  IPC,
  SampledPenalty,
};

class StatefulContactEnergy:
  public NonlinearOptimization::PotentialEnergy,
  public NonlinearOptimization::LineSearchAwareEnergy,
  public NonlinearOptimization::StepAwareEnergy,
  public NonlinearOptimization::EvaluationStateAwareEnergy
{
public:
  virtual ContactModelKind contactModelKind() const = 0;
  NonlinearOptimization::EnergyStateKind stateKind() const override
  {
    return NonlinearOptimization::EnergyStateKind::Displacement;
  }
  virtual void beginStep(const ContactStepState &state) = 0;
  virtual void refreshActiveSet(EigenSupport::ConstRefVecXd x) const = 0;
  virtual void clearActiveSet() const = 0;
  void prepareEvaluationState(EigenSupport::ConstRefVecXd x) const override
  {
    refreshActiveSet(x);
  }
};

std::shared_ptr<EmbeddedSurfaceFloorPotentialEnergy> createFloorEnergy(
  const ContactSurfaceSpec &surface,
  const FloorSpec &floor);

}  // namespace pgo::Contact
```

IPC-specific API:

```cpp
namespace pgo::Contact::IPC
{

struct ParametersSpec
{
  double dhat = 1e-1;
  std::optional<double> dhatExternal = std::nullopt;
  double kappa = 0.1;
  double epsEE = 0.0;
  double slackness = 1.0;
  double ccdThickness = 0.0;
};

class IPCContactEnergy;

std::shared_ptr<IPCContactEnergy> createIPCEnergy(
  const ContactSurfaceSpec &surface,
  const EigenSupport::MXi &surfaceTriangles,
  const ParametersSpec &params = {},
  std::vector<ObstacleSpec> obstacles = {});

SurfaceIPCCore::Parameters toSurfaceIPCParameters(const ParametersSpec &spec);

}  // namespace pgo::Contact::IPC
```

Sampled penalty-specific API:

```cpp
namespace pgo::Contact::SampledPenalty
{

struct ParametersSpec
{
  double stiffness = 1.0;
  int samples = 1;
  bool enableSelfContact = true;
  bool enableExternalContact = true;
};

struct FrictionParametersSpec
{
  double frictionCoeff = 1.0;
  double velocityEps = 1.0;
};

class SampledPenaltyContactEnergy;
class FrictionalSampledPenaltyContactEnergy;

std::shared_ptr<SampledPenaltyContactEnergy> createSampledPenaltyEnergy(
  const ContactSurfaceSpec &surface,
  const ParametersSpec &params,
  std::vector<ObstacleSpec> obstacles = {});

std::shared_ptr<FrictionalSampledPenaltyContactEnergy> createFrictionalSampledPenaltyEnergy(
  const ContactSurfaceSpec &surface,
  const ParametersSpec &normalParams,
  const FrictionParametersSpec &frictionParams,
  std::vector<ObstacleSpec> obstacles = {});

}  // namespace pgo::Contact::SampledPenalty
```

Facade responsibilities:

- validate shapes and finite scalar params before constructing kernel objects；
- translate `FloorSpec` to `FloorPenaltyParameters`；
- translate `IPC::ParametersSpec` to `SurfaceIPCCore::Parameters`；
- translate `SampledPenalty::ParametersSpec` to normal sampled penalty handler/energy parameters；
- translate `SampledPenalty::FrictionParametersSpec` only for `FrictionalSampledPenaltyContactEnergy`；
- translate obstacle specs to concrete `StaticObstacleSurface` / `LinearMovingObstacleSurface` objects；
- construct IPC and sampled penalty energies in a ready-to-own state. Active sets are still refreshed by `beginStep(...)` / `refreshActiveSet(...)`。

`EmbeddedSurfaceFloorPotentialEnergy` remains a numerical implementation class. `EmbeddedSurfaceIPCPotentialEnergy` is renamed/migrated into `IPCContactEnergy` as the long-term C++ IPC contact energy; no new public construction path should depend on the old class name after C5. Sampled penalty classes under the current `legacy_penalty` folder become implementation kernels behind `SampledPenaltyContactEnergy` and `FrictionalSampledPenaltyContactEnergy`. Python binding should construct through the factory, not direct ctor calls.

### 2. `ContactSurfaceSpec` is the shared surface adapter input

Floor, IPC, and sampled penalty require a shared contact surface description:

```text
surface rest vertices
surface-from-simulation displacement map
optional per-surface-vertex embedding into simulation DOFs
```

Python exposes this as:

```python
@dataclass(frozen=True)
class ContactSurface:
    rest_vertices: np.ndarray
    surface_from_simulation_disp_map: pgo.sparse.SparseMatrix
    vertex_embedding: ContactVertexEmbedding | None = None
```

`ContactSurface` is not an energy. It is a reusable construction value object. Each energy construction copies data into C++ owned storage; deleting `ContactSurface` or its input arrays after construction does not affect the energy.

Factory validation:

- `restVertices.cols() == 3`
- `restVertices.rows() > 0`
- `surfaceFromSimulationDispMap.rows() == 3 * restVertices.rows()`
- `surfaceFromSimulationDispMap.cols() > 0`
- if `vertexEmbedding` exists, index and weight arrays must have the same size；
- if `vertexEmbedding` exists, `embeddingArity == vertexEmbeddingIndices.size() / restVertices.rows()` and is positive；
- sampled penalty construction requires `vertexEmbedding` until its handlers are refactored to consume sparse maps directly。

Implementation boundary:

- `EmbeddedDofMap` is the reusable C++ mapping primitive. It owns rest positions plus a sparse displacement map, maps simulation displacement to embedded displacement/position, and pulls embedded gradients/Hessians/max-step queries back to simulation DOFs.
- `ContactSurfaceAdapter` is the contact-specific wrapper around `EmbeddedDofMap` plus optional `ContactVertexEmbedding` metadata needed by sampled penalty sampling code.
- New contact code should not keep adding mapping helpers to `MappedSurfacePotentialEnergy`; that class becomes a compatibility/numerical implementation layer until floor/IPC are migrated to composition.

Sketch:

```cpp
class EmbeddedDofMap
{
public:
  EmbeddedDofMap(EigenSupport::MXd restPositions, EigenSupport::SpMatD displacementMap);

  int numSourceDofs() const;
  int numEmbeddedDofs() const;

  EigenSupport::VXd embeddedDisplacements(EigenSupport::ConstRefVecXd sourceX) const;
  EigenSupport::VXd embeddedPositions(EigenSupport::ConstRefVecXd sourceX) const;
  void pullbackGradient(EigenSupport::ConstRefVecXd embeddedGradient, EigenSupport::RefVecXd sourceGradient) const;
  void pullbackHessian(const EigenSupport::SpMatD &embeddedHessian, EigenSupport::SpMatD &sourceHessian) const;
};
```

### 3. IPC parameters mirror source defaults, with Python-friendly `dhat_external`

Python dataclass:

```python
@dataclass(frozen=True)
class IPCParameters:
    dhat: float = 1e-1
    dhat_external: float | None = None
    kappa: float = 0.1
    eps_ee: float = 0.0
    slackness: float = 1.0
    ccd_thickness: float = 0.0
```

Translation rule:

```text
dhat_external is None  =>  C++ dhat_external = dhat
```

Validation:

- all numeric fields must be finite；
- `dhat > 0`
- `dhat_external is None or dhat_external > 0`
- `kappa >= 0`
- `slackness > 0`
- `ccd_thickness >= 0`

### 4. `StatefulContactEnergy` is the common long-lived contact boundary

IPC and sampled penalty both become long-lived potential energies with explicit step and active-set lifecycle. The step lifecycle itself is common optimization/simulation infrastructure; contact only adds the active-set contract:

```cpp
namespace pgo::NonlinearOptimization
{
struct StepState
{
  double time = 0.0;
  double timestep = 0.0;
  const EigenSupport::VXd *previousX = nullptr;
};

class StepAwareEnergy
{
public:
  virtual ~StepAwareEnergy() = default;
  virtual void beginStep(const StepState &state) = 0;
};
}  // namespace pgo::NonlinearOptimization

namespace pgo::Contact
{
using ContactStepState = NonlinearOptimization::StepState;

class StatefulContactEnergy:
  public NonlinearOptimization::PotentialEnergy,
  public NonlinearOptimization::LineSearchAwareEnergy,
  public NonlinearOptimization::StepAwareEnergy,
  public NonlinearOptimization::EvaluationStateAwareEnergy
{
public:
  virtual ContactModelKind contactModelKind() const = 0;
  NonlinearOptimization::EnergyStateKind stateKind() const override
  {
    return NonlinearOptimization::EnergyStateKind::Displacement;
  }
  virtual void beginStep(const ContactStepState &state) = 0;
  virtual void refreshActiveSet(EigenSupport::ConstRefVecXd x) const = 0;
  virtual void clearActiveSet() const = 0;
  void prepareEvaluationState(EigenSupport::ConstRefVecXd x) const override
  {
    refreshActiveSet(x);
  }
};
}  // namespace pgo::Contact
```

Lifecycle contract:

- `StepState.time` is the **step start time**, and `StepState.previousX` is the displacement state at that same time. Contact energies must not reinterpret `StepState.time` as the target/evaluation time.
- Kinematic moving obstacle contact evaluation defaults to the implicit target time `state.time + state.timestep`. For IPC this preserves current `runIPCSim` behavior, where dynamic obstacle poses are sampled at the step end time. `setMovingObstacleTime(t)` remains the explicit override/helper for static or manual evaluation loops.
- `beginStep(state)` sets frame/step state such as start time, timestep, previous displacement, and moving obstacle poses; it clears stale exact and line-search active sets but does not perform contact detection.
- `refreshActiveSet(x)` is the canonical detection/rebuild entry point for the normal evaluation point `x`. It is called directly by Python/manual users, and indirectly by solver/optimizer code through `prepareEvaluationState(x)`.
- `refreshActiveSet(x)`, `clearActiveSet()`, line-search hooks, and `prepareEvaluationState(x)` are `const` methods whose implementations mutate only internal caches. This matches the existing `PotentialEnergy` evaluation model and lets `EnergySet` store `shared_ptr<const PotentialEnergy>` while still forwarding evaluation-state callbacks.
- `func` / `gradient` / safe one-shot `hessian` consume the currently active contact model; they must not run contact detection themselves. Outside line search, evaluating without a matching exact active set is an API error with a clear message telling callers to use `refreshActiveSet(x)` or `prepareEvaluationState(x)` first.
- The solver evaluation-state prepare hook dispatches `EvaluationStateAwareEnergy::prepareEvaluationState(xEval)` immediately before every normal solver-owned evaluation point, including the Newton current point before `func_grad_hessian(x_i, ...)`, non-frozen line-search trial points, subiteration-one post-step gradient evaluation, and optimizer final-objective evaluation.
- When a bounded line-search policy uses `beginLineSearch(x, dxClamped)`, contact trial evaluations consume the frozen/superset line-search active set and deliberately skip `prepareEvaluationState(xTrial)` until `endLineSearch()` clears that line-search state.
- `computeMaxStepLimit(x, dxRaw)` is called before line search and returns a feasible alpha for the raw Newton direction. The solver applies the global feasible alpha first, then calls `beginLineSearch(x, dxClamped)`.
- `beginLineSearch(x, dxClamped)` receives the already-clamped direction and may build caches/supersets valid for `x + alpha * dxClamped`, `alpha in [0, 1]` unless the line-search policy advertises a wider probe range. It must not compute or modify feasible alpha.
- During line search, contact pairs are frozen or conservatively supersetted through `beginLineSearch(...)` / `endLineSearch()` from `LineSearchAwareEnergy`.
- `clearActiveSet()` releases active pairs, temporary energies, buffers, and line-search state.

This is deliberately not a Python subclassing API. Python receives concrete energy objects with methods such as `begin_step(...)`, `refresh_active_set(x)`, and `clear_active_set()`.

### 5. `IPCContactEnergy` implements `StatefulContactEnergy`

`IPCContactEnergy` is the public long-lived IPC energy. C5 migrates/renames the current `EmbeddedSurfaceIPCPotentialEnergy` implementation into `IPCContactEnergy` directly. Do not introduce a long-lived wrapper/adapter class as the final design. A short-lived local alias is acceptable only inside the same migration patch while callers are being moved; the C5 done state has no public/new construction path depending on `EmbeddedSurfaceIPCPotentialEnergy`.

Internal state:

```cpp
ContactSurfaceAdapter surfaceAdapter_;
IPC::SurfaceIPCCore core_;
mutable SurfaceIPCActiveSet activeSet_;
mutable EigenSupport::VXd activeSetSurfacePositions_;
mutable bool hasActiveSet_ = false;

mutable SurfaceIPCActiveSet lineSearchActiveSet_;
mutable bool hasLineSearchActiveSet_ = false;
```

Implementation:

- `beginStep(state)` clears exact and line-search active sets, stores the step context, and delegates moving obstacle poses to `core_.setMovingObstacleTime(state.time + state.timestep)`.
- `refreshActiveSet(x)` maps simulation displacement to surface positions, then calls `core_.buildActiveSet(surfacePositions)`.
- `prepareEvaluationState(x)` uses the `StatefulContactEnergy` default and calls `refreshActiveSet(x)` for the solver-owned normal evaluation point.
- `func` / `gradient` / safe one-shot `hessian` evaluate with the prepared exact active set after Energy E0 rename. If the active set is absent or stale outside line search, they throw; direct Python evaluation must call `refresh_active_set(x)` first.
- `beginLineSearch(x, dxClamped)` maps both vectors to surface space and builds `core_.buildLineSearchActiveSetSuperset(surfaceX, surfaceDxClamped)`.
- `endLineSearch()` clears the line-search active set and does not promote the line-search superset into the exact active-set cache. The next normal evaluation point is prepared explicitly.
- `computeMaxStepLimit(x, dx)` keeps using IPC CCD/max-step through `SurfaceIPCCore`.

`IPCEnergy.set_moving_obstacle_time(t)` remains a model-specific helper and directly calls the moving-obstacle update path at exactly `t`. It is not a compatibility alias for `setObstacleTime` and does not apply `+ timestep`.

### 6. Sampled penalty is split into normal and frictional energies

The current `legacy_penalty` implementation becomes sampled penalty contact. It is split into two long-term C++ classes:

- `SampledPenaltyContactEnergy`: normal penalty contact only. It is a `StatefulContactEnergy`. Dynamic solves may use both external and self contact; physical static solve builders may only use a frozen external-contact subset by default.
- `FrictionalSampledPenaltyContactEnergy`: normal penalty contact plus dynamic friction terms. It derives from `SampledPenaltyContactEnergy` and `NonlinearOptimization::StepDependentEnergy`; physical static solve builders reject it by default. It still reports `ContactModelKind::SampledPenalty`; callers distinguish frictional behavior through the concrete type or `StepDependentEnergy` marker.

```cpp
class SampledPenaltyContactEnergy : public StatefulContactEnergy
{
public:
  ContactModelKind contactModelKind() const override { return ContactModelKind::SampledPenalty; }
  void beginStep(const ContactStepState &state) override;
  void refreshActiveSet(EigenSupport::ConstRefVecXd x) override;
  void clearActiveSet() override;

protected:
  virtual void configureExternalActiveEnergy(PointPenetrationEnergy &energy);
  virtual void configureSelfActiveEnergy(PointTrianglePairCouplingEnergyWithCollision &energy);
};

class FrictionalSampledPenaltyContactEnergy final:
  public SampledPenaltyContactEnergy,
  public NonlinearOptimization::StepDependentEnergy
{
public:
  void beginStep(const ContactStepState &state) override;

protected:
  void configureExternalActiveEnergy(PointPenetrationEnergy &energy) override;
  void configureSelfActiveEnergy(PointTrianglePairCouplingEnergyWithCollision &energy) override;
};
```

Internal state:

```cpp
struct SampledPenaltyActiveSet
{
  std::shared_ptr<PointPenetrationEnergy> externalEnergy;
  std::unique_ptr<PointPenetrationEnergyBuffer> externalBuffer;

  std::shared_ptr<PointTrianglePairCouplingEnergyWithCollision> selfEnergy;
  std::unique_ptr<PointTrianglePairCouplingEnergyWithCollisionBuffer> selfBuffer;

  bool empty() const;
  void clear();
};

ContactSurfaceAdapter surfaceAdapter_;
SampledPenalty::ParametersSpec params_;
std::optional<SampledPenalty::FrictionParametersSpec> frictionParams_;  // only in frictional subclass

TriangleMeshExternalContactHandler externalHandler_;
TriangleMeshSelfContactHandler selfHandler_;

SampledPenaltyActiveSet activeSet_;
mutable SampledPenaltyActiveSet lineSearchActiveSet_;
mutable bool hasLineSearchActiveSet_ = false;

EigenSupport::VXd previousX_;  // only required by frictional subclass
double timestep_ = 0.0;        // only required by frictional subclass
bool hasStepState_ = false;    // only required by frictional subclass
```

Implementation:

- `SampledPenaltyContactEnergy::beginStep(state)` updates moving obstacles to `state.time + state.timestep` when obstacles are moving, then clears `activeSet_` and `lineSearchActiveSet_`. It does not require `previousX` and does not set friction fields on active terms.
- `FrictionalSampledPenaltyContactEnergy::beginStep(state)` requires `state.previousX != nullptr` and `state.timestep > 0`, stores `previousX`, `timestep`, and the start time, then delegates the common clear/update work to the base implementation.
- `refreshActiveSet(x)` maps `x` to surface displacement, runs external/self contact detection through the handlers, and rebuilds `activeSet_`.
- active external term delegates to `PointPenetrationEnergy`;
- active self term delegates to `PointTrianglePairCouplingEnergyWithCollision`;
- `SampledPenaltyContactEnergy::configureExternalActiveEnergy(...)` and `configureSelfActiveEnergy(...)` set buffers, stiffness, position functions, and normal-only state. The normal self-contact path must still call any existing kernel setup required for normal evaluation, including `computeClosestPosition(...)` if the migrated kernel still uses it to initialize `hessianBlocks`, `gradientBlocks`, normals, or contact status.
- `FrictionalSampledPenaltyContactEnergy` overrides those hooks to additionally set last-position functions, friction coefficient, timestep, and velocity epsilon.
- `func` returns the sum of active external/self terms, or zero when there are no active contacts;
- `gradient` and safe one-shot `hessian` sum active term contributions into the simulation-space result;
- `beginLineSearch(...)` copies or rebuilds one `SampledPenaltyActiveSet` for the base point and freezes it; trial evaluations never rerun contact detection.
- If a sampled penalty active set is absent or stale outside line search, direct evaluation throws; direct Python/manual callers must call `refreshActiveSet(x)` / `refresh_active_set(x)` first. Solver/runIPCSim paths use `prepareEvaluationState(...)` for normal evaluation points.

`SampledPenaltyActiveSet` is intentionally internal. It is the RAII owner for active penalty energies and buffers, so the public contact energy does not accumulate scattered `shared_ptr` + raw buffer fields as the sampled penalty model grows.

This turns the current runtime flow:

```text
execute(...) -> buildContactEnergy() -> addGeneralImplicitForceModel(...)
```

into:

```text
beginStep(...) -> refreshActiveSet(x) -> evaluate long-lived SampledPenaltyContactEnergy / FrictionalSampledPenaltyContactEnergy
```

Static-solve rule:

- `SampledPenaltyContactEnergy` is normal-only and may be used in physical static solves only through an explicit frozen-active-set static adapter.
- The default physical static policy is external-only: build one active set from the static solve start state, freeze it for the whole solve, and do not expose `EvaluationStateAwareEnergy` or `LineSearchAwareEnergy` from the static adapter. This prevents static Newton iterations from silently becoming a dynamic active-set update loop.
- Sampled self-contact remains dynamic-solve-only in v1. Self-contact active sets are too state/path dependent for the default physical static solve contract; supporting static self-contact later requires an explicit continuation/fixed-active-set policy and dedicated convergence tests.
- `FrictionalSampledPenaltyContactEnergy` is `StepDependentEnergy` and is rejected by physical static solve builders by default.
- Plain mathematical optimizer entry points such as `solve_newton` do not perform this rejection; the gate belongs to `runIPCSim` static mode, `SimulationProblem` validation, and future physical static-solve builders.

### 7. Common obstacle surface uses explicit static/moving hierarchy

Replace the current single `ObstacleSurface + TrajectorySampler + staticObstacles_ bool vector` model with a real type split:

```cpp
class ObstacleSurface
{
public:
  virtual ~ObstacleSurface() = default;

  virtual std::unique_ptr<ObstacleSurface> cloneSurface() const = 0;

  virtual int32_t objectId() const = 0;
  virtual void setObjectId(int32_t id) = 0;

  virtual const EigenSupport::VXd &currentPositions() const = 0;
  virtual const EigenSupport::MXi &triangles() const = 0;
  virtual const EigenSupport::MXi &uniqueEdges() const = 0;
  virtual const EigenSupport::MXi &contactEdges() const = 0;
  virtual const ObstaclePoseCache &cache() const = 0;
};

class StaticObstacleSurface final : public ObstacleSurface
{
  // No time API.
public:
  std::unique_ptr<StaticObstacleSurface> cloneStatic() const;
  std::unique_ptr<ObstacleSurface> cloneSurface() const override;
};

class MovingObstacleSurface : public ObstacleSurface
{
public:
  virtual void setTime(double t) = 0;
  virtual std::unique_ptr<MovingObstacleSurface> cloneMoving() const = 0;
};

class LinearMovingObstacleSurface final : public MovingObstacleSurface
{
public:
  LinearMovingObstacleSurface(
    EigenSupport::MXd restVertices,
    EigenSupport::MXi triangles,
    EigenSupport::V3d velocity,
    double referenceTime = 0.0);

  void setTime(double t) override;
  std::unique_ptr<MovingObstacleSurface> cloneMoving() const override;
  std::unique_ptr<ObstacleSurface> cloneSurface() const override;
};
```

Key semantics:

- `StaticObstacleSurface` builds `currentPositions`, `contactEdges`, and `ObstaclePoseCache` directly from its geometry in the constructor.
- `StaticObstacleSurface` never samples at `t = 0`; it is simply time-independent.
- `MovingObstacleSurface` owns the time API. The abstract base `ObstacleSurface` has no `setTime`.
- `LinearMovingObstacleSurface` initializes itself to `t = 0.0` in its constructor, so construction returns a ready object.
- There is no separate `ContactTimeSpec` or construction-time initial time parameter. Runtime changes go through `setMovingObstacleTime(t)`.
- `referenceTime` belongs to the linear trajectory formula, not to contact construction lifecycle.

### 8. Contact models consume common obstacle views

The static/moving obstacle hierarchy is common contact infrastructure, not IPC-only infrastructure. The implementation should move the current `ipc/external/obstacleSurface.*` and `obstaclePoseCache.*` concepts into a common contact external/obstacle module, then let IPC and sampled penalty consume that module.

`SurfaceIPCCore` should not make broad phase / assembler code depend on concrete obstacle ownership. Internally it may own static and moving obstacles separately, or it may receive a common `ContactObstacleSet` from the factory:

```cpp
struct ObstacleSlot
{
  enum class Kind { Static, Moving };
  Kind kind;
  std::size_t index;
  int32_t objectId;
};

class SurfaceIPCCore
{
  std::vector<std::unique_ptr<StaticObstacleSurface>> staticObstacles_;
  std::vector<std::unique_ptr<MovingObstacleSurface>> movingObstacles_;
  std::vector<ObstacleSlot> obstacleOrder_;
};
```

Algorithms consume an ephemeral read-only merged view:

```cpp
struct ObstacleSurfaceView
{
  int32_t objectId;
  const EigenSupport::VXd *currentPositions;
  const EigenSupport::MXi *triangles;
  const EigenSupport::MXi *uniqueEdges;
  const EigenSupport::MXi *contactEdges;
  const ObstaclePoseCache *cache;
};

std::vector<ObstacleSurfaceView> SurfaceIPCCore::obstacleViews() const;
```

Design rules:

- `objectId` is assigned once from the user/factory input order.
- `obstacleViews()` returns views in the same input order, even though storage is split by type.
- External pairs and diagnostics use `objectId`, not static/moving storage index.
- `ObstacleSurfaceView` is a local evaluation snapshot. Do not store it across `setMovingObstacleTime`, cache invalidation, or obstacle mutation.
- Broad phase, external barrier assembler, and external max-step code should take `std::span<const ObstacleSurfaceView>` or `const std::vector<ObstacleSurfaceView>&`, not owning obstacle containers.

This removes `staticObstacles_` bool flags and the need for `markObstacleStatic`.

### 9. Time API is explicit: moving obstacles only

Rename the lifecycle API:

```cpp
class SurfaceIPCCore
{
public:
  void setMovingObstacleTime(double t);
};

class IPCContactEnergy
{
public:
  void setMovingObstacleTime(double t);
};
```

Python exposes exactly:

```python
ipc.set_moving_obstacle_time(t)
```

Rules:

- no `setObstacleTime(...)` compatibility wrapper；
- no Python `set_obstacle_time(...)` alias；
- calling `set_moving_obstacle_time(t)` on an IPC energy with only static obstacles is a no-op except normal cache invalidation policy；
- `IPCContactEnergy::setMovingObstacleTime` clears energy/line-search active-set caches, then asks `SurfaceIPCCore` to update only moving obstacles。

### 10. Python exposes static and linear moving obstacle specs

Python keeps the construction surface compact and value-like:

```python
@dataclass(frozen=True)
class ObstacleSpec:
    kind: Literal["static", "linear_velocity"]
    rest_vertices: np.ndarray
    triangles: np.ndarray
    velocity: np.ndarray | None = None
    reference_time: float = 0.0

    @staticmethod
    def static(rest_vertices, triangles) -> "ObstacleSpec": ...

    @staticmethod
    def linear_velocity(
        rest_vertices,
        triangles,
        velocity,
        reference_time: float = 0.0,
    ) -> "ObstacleSpec": ...
```

Validation:

- `rest_vertices` is `(n, 3) float64`
- `triangles` is `(m, 3) int64`
- triangle indices are in `[0, n)`
- `velocity` is finite shape `(3,)` for `linear_velocity`
- `reference_time` is finite

The Python spec is only a construction value. It does not expose obstacle object identity, pose cache, sampler, or time mutation.

### 11. Obstacle copy semantics use `clone()`

With virtual obstacle surfaces, copying must be explicit:

- `ObstacleSurface::cloneSurface()` exists for generic read-only cloning when needed.
- `StaticObstacleSurface::cloneStatic()` deep-copies static obstacles into typed static storage.
- `MovingObstacleSurface::cloneMoving()` deep-copies moving obstacles into typed moving storage.
- `SurfaceIPCCore` copy constructor / assignment deep-clone all obstacles and preserve `obstacleOrder_`.
- `IPCContactEnergy` copy behavior remains safe if the existing class is copied by value in tests or downstream code.

Do not silently make `SurfaceIPCCore` move-only unless a repo-wide audit proves no caller depends on copyability. The safer migration is deep clone.

### 12. Python `FloorEnergy` is a wrapper over factory output

Python API:

```python
@dataclass(frozen=True)
class FloorParameters:
    axis: Literal["x", "y", "z"]
    side: Literal["keep_above", "keep_below"]
    height: float
    kappa: float

floor = pgo.contact.FloorEnergy(
    surface,
    axis="y",
    side="keep_above",
    height=0.0,
    kappa=1e6,
)
```

`FloorEnergy` may also accept `parameters=FloorParameters(...)`; passing both `parameters` and split fields is an error.

Readable properties:

- `floor.axis`
- `floor.side`
- `floor.height`
- `floor.kappa`
- `floor.state_kind == "displacement"`

Implementation decision: Python wrapper stores a `FloorParameters` snapshot for readable metadata. `set_height(h)` calls C++ `setFloorHeight(h)` and updates the wrapper snapshot. We do not need extra C++ getters for `axis/side/kappa` in M3.

### 13. `IPCEnergy` wrapper owns construction metadata but not raw internals

Readable properties:

- `ipc.params`
- `ipc.num_obstacles`
- `ipc.num_moving_obstacles`
- `ipc.state_kind == "displacement"`

Not exposed:

- obstacle list mutable view；
- `mark_obstacle_static`；
- active-set cache；
- line-search cache；
- raw `SurfaceIPCCore`。

Python `IPCEnergy.begin_step(...)` and `IPCEnergy.refresh_active_set(x)` forward to the C++ `StatefulContactEnergy` lifecycle. `IPCEnergy.set_moving_obstacle_time(t)` remains available as a model-specific helper for static optimization loops that only need moving obstacle time control.

### 14. Sampled penalty wrappers split normal and frictional contact

Python API:

```python
@dataclass(frozen=True)
class SampledPenaltyParameters:
    stiffness: float = 1.0
    samples: int = 1
    enable_self_contact: bool = True
    enable_external_contact: bool = True

@dataclass(frozen=True)
class FrictionParameters:
    friction_coeff: float = 1.0
    velocity_eps: float = 1.0

penalty = pgo.contact.SampledPenaltyEnergy(
    surface,
    params=pgo.contact.SampledPenaltyParameters(samples=2),
    obstacles=[pgo.contact.ObstacleSpec.static(obs_rest, obs_tris)],
)
penalty.begin_step(time=t, timestep=dt, previous_x=u_prev)
penalty.refresh_active_set(u)

frictional = pgo.contact.FrictionalSampledPenaltyEnergy(
    surface,
    params=pgo.contact.SampledPenaltyParameters(samples=2),
    friction=pgo.contact.FrictionParameters(
        friction_coeff=1.0,
        velocity_eps=1.0,
    ),
    obstacles=[pgo.contact.ObstacleSpec.static(obs_rest, obs_tris)],
)
frictional.begin_step(time=t, timestep=dt, previous_x=u_prev)
frictional.refresh_active_set(u)
```

Readable properties:

- `penalty.params`
- `frictional.params`
- `frictional.friction`
- `penalty.num_obstacles`
- `penalty.has_active_set`
- `penalty.state_kind == "displacement"`
- `frictional.state_kind == "displacement"`

Static-solve visibility:

- `SampledPenaltyEnergy` is normal-only. Physical static-solve builders expose only the frozen external-contact subset by default; sampled self-contact remains dynamic-solve-only in v1.
- `FrictionalSampledPenaltyEnergy` is step-dependent; physical static-solve builders reject it by default.
- Python `solve_newton` does not reject `FrictionalSampledPenaltyEnergy`, because it is a generic optimizer rather than a physical static-solve builder.

Not exposed:

- `TriangleMeshExternalContactHandler`
- `TriangleMeshSelfContactHandler`
- `PointPenetrationEnergy`
- `PointTrianglePairCouplingEnergyWithCollision`
- active buffers or active term ownership

### 15. Hessian/evaluation goes through energy helpers

Python `FloorEnergy.value/gradient/hessian`, `IPCEnergy.value/gradient/hessian`, `SampledPenaltyEnergy.value/gradient/hessian`, and `FrictionalSampledPenaltyEnergy.value/gradient/hessian` use the same `PotentialEnergy` binding surface as other energies. Hessian must call the Energy plan helper:

```cpp
pgo::NonlinearOptimization::evaluateHessian(*energy, x)
```

This is required because contact energies are not Hessian-topology-fixed.

### 16. Contact energy construction is self-owning

Construction copies all inputs into C++ owned storage:

- `ContactSurfaceSpec.restVertices`
- `ContactSurfaceSpec.surfaceFromSimulationDispMap`
- `ContactSurfaceSpec.vertexEmbedding` indices/weights
- `surfaceTriangles`
- obstacle rest vertices
- obstacle triangles
- linear velocity / reference time
- sampled penalty parameter values
- friction parameter values for `FrictionalSampledPenaltyContactEnergy`
- sampled penalty active term buffers are owned by the long-lived sampled penalty energy instance (`SampledPenaltyContactEnergy` or `FrictionalSampledPenaltyContactEnergy`)

Binding must not use ad-hoc keep-alive containers such as `std::shared_ptr<void>` just to keep Python arrays alive. Lifetime tests must delete Python inputs after construction and still evaluate successfully.

### 17. `runIPCSim` migrates to stateful contact models

Because `ObstacleSurface` becomes abstract and `legacy_penalty` becomes first-class sampled penalty contact, existing `runIPCSim` setup code must be migrated in the same implementation phase.

Migration rule:

- Existing JSON field meanings stay unchanged, except for the explicitly documented sampled penalty static external-only/frictional behavior. CLI `--legacy` removal is an intentional non-JSON behavior change.
- `obstacleSetup.*` should return facade obstacle specs or an obstacle set accepted by `createIPCEnergy`.
- `shellSetup.cpp` / `volumeSetup.cpp` should not call `markObstacleStatic`.
- Any existing dynamic obstacle setup should express linear motion through `LinearMovingObstacleSpec` / `LinearMovingObstacleSurface`.
- `legacyPenaltyContact.*` should be renamed/migrated to sampled penalty contact naming.
- `ContactBackendKind::LegacyPenalty` should become `ContactBackendKind::SampledPenalty`.
- `--legacy` is removed, not retained as a deprecated alias. CLI, tests, README, and examples must migrate to `--contact-model sampled-penalty`.
- dynamic loop and static solve should call `beginStep(...)` on persistent energies; Newton solver / optimizer should call `prepareEvaluationState(x)` before normal evaluation points rather than rebuilding temporary penalty force models each step or relying on hidden detection inside `func`.
- physical static solve accepts floor, IPC, and frozen external-only normal sampled penalty contact.
- physical static solve rejects `NonlinearOptimization::StepDependentEnergy` by default. This rejects `FrictionalSampledPenaltyContactEnergy` unless a future explicit static-friction policy provides a reference state and positive pseudo-timestep.
- plain optimizer APIs do not reject `StepDependentEnergy`; the rejection belongs to `runIPCSim` static mode, `SimulationProblem` validation, and future physical static solve builders.

### 18. Cross-cutting architecture decisions

这些决策不只属于 contact，但 contact plan 必须按它们落地，避免把长期边界又写回局部 special case：

1. ✅ Step lifecycle is common infrastructure.  **（已落地）**

`StepAwareEnergy` lives under `NonlinearOptimization`, not under `Contact`. `StatefulContactEnergy` derives from it and adds only contact-specific `refreshActiveSet(x)` / `clearActiveSet()` semantics. Future time integrator energies or history-dependent material energies can reuse `StepAwareEnergy` without pretending to be contact energies.

→ 文件：`src/core/nonlinearOptimization/stepAwareEnergy.h`（time integrator plan 落地）。

2. ✅ Energy roles stay separated.  **（已落地）**

`PotentialEnergy` is value/gradient/Hessian. `LineSearchAwareEnergy` is freeze/unfreeze for line-search evaluation. A future `MaxStepAwareEnergy` should own feasible-step / CCD-style capability instead of leaving every caller to branch on `PotentialEnergy::computeMaxStepLimit(...)`. This contact plan does not require the full `MaxStepAwareEnergy` extraction to finish first, but new IPC/contact code should be written as if max-step is an optional capability, not a hard assumption on every energy.

→ 既有代码，无变化。

3. ✅ `state_kind` comes from C++.  **（已落地）**

Use the shared `NonlinearOptimization::EnergyStateKind { Generic, Displacement }` and `PotentialEnergy::stateKind() const` from the Energy plan. Python `energy.state_kind` maps this enum. `StatefulContactEnergy` returns `Displacement` by default, and contact Python wrappers must not hard-code a divergent string.

→ 文件：`src/core/nonlinearOptimization/potentialEnergy.h`（energy plan 落地）。

4. Surface mapping uses `EmbeddedDofMap`.

`ContactSurfaceAdapter` is a contact facade over the reusable `EmbeddedDofMap`. The generic map owns rest embedded positions and the sparse displacement map, and provides mapping/pullback helpers. This keeps future deformation, contact, attachments, and embedded energies from each growing their own simulation-to-embedded DOF code.

5. `runIPCSim` should split problem description from runtime state.

Long-term setup should parse JSON into a mostly immutable `SimulationProblem`:

```text
mesh/formulation/material/contact specs/boundary specs/output options
```

The mutable execution state should live in `SimulationRuntime`:

```text
current x/v/time, solver service, EnergySet, StatefulContactEnergy instances, output writers
```

`SimulationProblem` constructs contact through `contactEnergyFactory`; `SimulationRuntime` owns per-step `beginStep(...)`, solver invocation, and output. Newton solver / optimizer own `prepareEvaluationState(x)` immediately before normal evaluation points, while line search owns `beginLineSearch(...)` / `endLineSearch()`. This keeps config parsing, contact construction, per-frame mutation, and per-evaluation active-set refresh from staying tangled in setup files.

6. Sampled penalty active terms are one internal object.

`SampledPenaltyContactEnergy` owns a `SampledPenaltyActiveSet` for the prepared exact evaluation state and, when needed, one frozen line-search active set. Active external/self penalty energies and their buffers are not individual long-term members on the public energy class.

7. Evaluation-state prepare is common optimization infrastructure.

`EvaluationStateAwareEnergy` lives under `NonlinearOptimization`, not under `Contact`. Newton solver calls `prepareEvaluationState(xEval)` immediately before normal evaluations that it owns: current-iterate `func_grad_hessian`, non-frozen line-search trial evaluations, subiteration-one post-step gradient evaluation, and optimizer final-objective evaluation. `EnergySet` implements this interface by mapping the global evaluation state to each term's local DOFs and forwarding to child energies that implement the interface. `StatefulContactEnergy::prepareEvaluationState(x)` defaults to `refreshActiveSet(x)`. This makes exact active-set refresh explicit and keeps energy evaluation methods from depending on hidden lazy rebuilds.

8. `StepDependentEnergy` marks energies that require per-step history.

`StepDependentEnergy` lives under `NonlinearOptimization` and is a marker for energies whose value/gradient/Hessian depend on step history such as `previousX`, positive `timestep`, or velocity-like state. It is intentionally narrower than "time dependent": IPC normal contact with moving obstacles can be evaluated in static mode at an explicit obstacle time and is not `StepDependentEnergy`. Physical static solve builders reject `StepDependentEnergy` by default; generic mathematical optimizers do not.

## 目标 C++ API

New public construction header:

```cpp
// src/core/contact/contactEnergyFactory.h
namespace pgo::NonlinearOptimization
{
struct StepState;
class StepAwareEnergy;
enum class EnergyStateKind;
class EvaluationStateAwareEnergy;
class StepDependentEnergy;
}  // namespace pgo::NonlinearOptimization

namespace pgo::Contact
{

struct ContactSurfaceSpec;
struct ContactVertexEmbedding;
struct FloorSpec;
using ContactStepState = NonlinearOptimization::StepState;
struct StaticObstacleSpec;
struct LinearMovingObstacleSpec;
using ObstacleSpec = std::variant<StaticObstacleSpec, LinearMovingObstacleSpec>;
class EmbeddedDofMap;
class ContactSurfaceAdapter;
enum class ContactModelKind;
class StatefulContactEnergy;

std::shared_ptr<EmbeddedSurfaceFloorPotentialEnergy> createFloorEnergy(
  const ContactSurfaceSpec &surface,
  const FloorSpec &floor);

namespace IPC
{
struct ParametersSpec;
class IPCContactEnergy;

std::shared_ptr<IPCContactEnergy> createIPCEnergy(
  const ContactSurfaceSpec &surface,
  const EigenSupport::MXi &surfaceTriangles,
  const ParametersSpec &params = {},
  std::vector<ObstacleSpec> obstacles = {});
}  // namespace IPC

namespace SampledPenalty
{
struct ParametersSpec;
struct FrictionParametersSpec;
class SampledPenaltyContactEnergy;
class FrictionalSampledPenaltyContactEnergy;

std::shared_ptr<SampledPenaltyContactEnergy> createSampledPenaltyEnergy(
  const ContactSurfaceSpec &surface,
  const ParametersSpec &params = {},
  std::vector<ObstacleSpec> obstacles = {});

std::shared_ptr<FrictionalSampledPenaltyContactEnergy> createFrictionalSampledPenaltyEnergy(
  const ContactSurfaceSpec &surface,
  const ParametersSpec &normalParams = {},
  const FrictionParametersSpec &frictionParams = {},
  std::vector<ObstacleSpec> obstacles = {});
}  // namespace SampledPenalty

}  // namespace pgo::Contact
```

Existing numerical APIs remain available for internal code, but Python and new C++ construction examples use the factory. Contact lifecycle APIs are made explicit and not duplicated:

```text
PotentialEnergy::stateKind -> C++ source of truth for Python state_kind
setObstacleTime       -> removed / replaced by setMovingObstacleTime
set_obstacle_time     -> never exposed
markObstacleStatic    -> removed from construction path
legacy penalty        -> renamed to sampled penalty contact
--legacy              -> removed; use --contact-model sampled-penalty
StepDependentEnergy   -> rejected by physical static solve builders by default
```

## Python API 定稿草案

```python
import pypgo as pgo
import numpy as np

surface_embedding = pgo.contact.ContactVertexEmbedding(
    indices=embedding_indices,
    weights=embedding_weights,
    arity=embedding_arity,
)

surface = pgo.contact.ContactSurface(
    rest_vertices=np.asarray(surface_rest, dtype=np.float64),
    surface_from_simulation_disp_map=A,
    vertex_embedding=surface_embedding,
)

floor = pgo.contact.FloorEnergy(
    surface,
    axis="y",
    side="keep_above",
    height=0.0,
    kappa=1e6,
)

ipc = pgo.contact.IPCEnergy(
    surface,
    surface_triangles=np.asarray(triangles, dtype=np.int64),
    params=pgo.contact.IPCParameters(dhat=1e-3),
    obstacles=[
        pgo.contact.ObstacleSpec.static(obs_rest, obs_tris),
        pgo.contact.ObstacleSpec.linear_velocity(
            moving_rest,
            moving_tris,
            velocity=np.array([0.0, -1.0, 0.0]),
        ),
    ],
)

ipc.begin_step(time=0.25, timestep=dt, previous_x=u_prev)
ipc.refresh_active_set(u0)

penalty = pgo.contact.SampledPenaltyEnergy(
    surface,
    params=pgo.contact.SampledPenaltyParameters(
        stiffness=1.0,
        samples=2,
    ),
    obstacles=[pgo.contact.ObstacleSpec.static(obs_rest, obs_tris)],
)
penalty.begin_step(time=0.25, timestep=dt, previous_x=u_prev)
penalty.refresh_active_set(u0)

frictional_penalty = pgo.contact.FrictionalSampledPenaltyEnergy(
    surface,
    params=pgo.contact.SampledPenaltyParameters(stiffness=1.0, samples=2),
    friction=pgo.contact.FrictionParameters(
        friction_coeff=1.0,
        velocity_eps=1.0,
    ),
    obstacles=[pgo.contact.ObstacleSpec.static(obs_rest, obs_tris)],
)
frictional_penalty.begin_step(time=0.25, timestep=dt, previous_x=u_prev)
frictional_penalty.refresh_active_set(u0)

assert floor.state_kind == "displacement"
assert ipc.state_kind == "displacement"
assert ipc.params.dhat == 1e-3

total = pgo.energy.EnergySet([(floor, 1.0), (ipc, 1.0)])
u = total.zero_state()
g = total.gradient(u)
H = total.hessian(u)
```

`pypgo.contact` public surface:

```text
pypgo.contact
  ContactSurface
  ContactVertexEmbedding
  FloorEnergy
  FloorParameters
  IPCEnergy
  IPCParameters
  SampledPenaltyEnergy
  SampledPenaltyParameters
  FrictionalSampledPenaltyEnergy
  FrictionParameters
  ObstacleSpec
```

Not public:

```text
MappedSurfacePotentialEnergy
EmbeddedSurfaceFloorPotentialEnergy
EmbeddedSurfaceIPCPotentialEnergy
EmbeddedDofMap
ContactSurfaceAdapter
IPCContactEnergy
SampledPenaltyContactEnergy
FrictionalSampledPenaltyContactEnergy
StepDependentEnergy
FloorPenaltyParameters
SurfaceIPCCore
StatefulContactEnergy
ObstacleSurface
StaticObstacleSurface
MovingObstacleSurface
LinearMovingObstacleSurface
ObstacleSurfaceView
TriangleMeshExternalContactHandler
TriangleMeshSelfContactHandler
PointPenetrationEnergy
PointTrianglePairCouplingEnergyWithCollision
markObstacleStatic
setObstacleTime
set_obstacle_time
```

## File Map

### 新增

- `src/core/contact/contactEnergyFactory.h`
- `src/core/contact/contactEnergyFactory.cpp`
- `src/core/contact/statefulContactEnergy.h`
- `src/core/contact/embeddedDofMap.h`
- `src/core/contact/embeddedDofMap.cpp`
- `src/core/contact/contactSurfaceAdapter.h`
- `src/core/contact/contactSurfaceAdapter.cpp`
- `src/core/contact/external/obstacleSurface.h`
- `src/core/contact/external/obstacleSurface.cpp`
- `src/core/contact/external/obstaclePoseCache.h`
- `src/core/contact/external/obstaclePoseCache.cpp`
- `src/core/contact/ipc/ipcContactEnergy.h`
- `src/core/contact/ipc/ipcContactEnergy.cpp`
- `src/core/contact/sampled_penalty/sampledPenaltyContactEnergy.h`
- `src/core/contact/sampled_penalty/sampledPenaltyContactEnergy.cpp`
- `src/core/contact/sampled_penalty/frictionalSampledPenaltyContactEnergy.h`
- `src/core/contact/sampled_penalty/frictionalSampledPenaltyContactEnergy.cpp`
- `src/core/contact/sampled_penalty/sampledPenaltyActiveSet.h`
- `src/core/contact/sampled_penalty/sampledPenaltyActiveSet.cpp`
- `src/core/nonlinearOptimization/evaluationStateAwareEnergy.h`
- `src/core/nonlinearOptimization/stepDependentEnergy.h`
- `src/python/pypgo/bindings/contact_bindings.cpp`
- `pypgo/contact.py`
- `tests/src/core/contact/contact_energy_factory_gtest.cpp`
- `tests/src/core/contact/stateful_contact_energy_gtest.cpp`
- `tests/src/core/contact/sampled_penalty_contact_energy_gtest.cpp`
- `tests/pypgo/test_contact.py`

### 修改

- `src/core/contact/CMakeLists.txt`：编入 `contactEnergyFactory.*`。
- `src/core/nonlinearOptimization/CMakeLists.txt`：编入 `evaluationStateAwareEnergy.h` 和 `stepDependentEnergy.h`；`StepAwareEnergy` 已由 time integrator plan 落地，`EnergyStateKind` 来自 Energy plan，不在 contact plan 里重复添加。
- `src/core/contact/mappedSurfacePotentialEnergy.h/.cpp`：迁移为 common `EmbeddedDofMap` + `ContactSurfaceAdapter` composition，或让旧类临时委托 adapter，避免 common contact 层继续挂在 `Contact::IPC` namespace 下。
- `src/core/contact/ipc/external/obstacleSurface.h/.cpp`：迁移到 common `src/core/contact/external/`，并改为 abstract base + static/moving concrete hierarchy。
- `src/core/contact/ipc/external/obstaclePoseCache.h/.cpp`：迁移到 common `src/core/contact/external/`。
- `src/core/contact/ipc/core/surfaceIPCCore.h/.cpp`：split obstacle storage、stable `objectId`、`ObstacleSurfaceView`、`setMovingObstacleTime`。
- `src/core/contact/ipc/broadPhase/surfaceIPCExternalBroadPhase.cpp`：external obstacle path 改吃 view。
- `src/core/contact/ipc/broadPhase/surfaceIPCBroadPhase.h`：external obstacle signatures 改吃 view。
- `src/core/contact/ipc/core/surfaceIPCExternalBarrierAssembler.*`：external obstacle signatures 改吃 view。
- `src/core/contact/ipc/core/surfaceIPCMaxStep.*`：external obstacle signatures 改吃 view。
- `src/core/contact/ipc/embeddedSurfaceIPCPotentialEnergy.h/.cpp`：迁移/重命名为 `ipcContactEnergy.h/.cpp`；constructor 接新 obstacle ownership；`setObstacleTime` 改为 `setMovingObstacleTime`；移除 `markObstacleStatic` construction path。C5 完成后不保留新的 public construction path 使用旧类名。
- `src/core/contact/sampled_penalty/kernels/*`：迁移/重命名为 `src/core/contact/sampled_penalty/*`，保留数值 kernel 行为。
- `src/tools/sim/runIPCSim/contact/contactBackend.h`：`LegacyPenalty` 改为 `SampledPenalty`，并对接 `StatefulContactEnergy`。
- `src/tools/sim/runIPCSim/contact/legacyPenaltyContact.*`：迁移/重命名为 sampled penalty contact backend，停止每步临时 rebuild force model。
- `src/tools/sim/runIPCSim/app/session.*`：逐步拆成 `SimulationProblem` + `SimulationRuntime`，把 JSON/setup 产物和每步 mutable runtime 分开。
- `src/tools/sim/runIPCSim/setup/obstacleSetup.*`：返回新 obstacle specs / obstacle set。
- `src/tools/sim/runIPCSim/setup/shellSetup.cpp`：移除 `markObstacleStatic` 调用，接入新 factory/spec。
- `src/tools/sim/runIPCSim/setup/volumeSetup.cpp`：移除 `markObstacleStatic` 调用，接入新 factory/spec。
- `src/tools/sim/runIPCSim/cli/cli.cpp`：移除 `--legacy`，使用 `--contact-model sampled-penalty`。
- `src/tools/sim/runIPCSim/app/app.h/.cpp`：`RunIPCSimOptions` 使用 `ContactBackendKind::SampledPenalty`。
- `tests/src/core/contact/CMakeLists.txt`：新增 `contact_energy_factory_gtest`。
- `src/python/pypgo/CMakeLists.txt`：编入 `contact_bindings.cpp`，确保 `pypgo_core` link `contact` / `nonlinearOptimization`。
- `src/python/pypgo/bindings/module.cpp`：注册 contact bindings。
- `pypgo/__init__.py`：导出 `pypgo.contact`。
- `plan/python_api_migration/api_coverage.md`：Contact 一节加入 factory/spec/wrapper coverage。
- `plan/python_api_migration/numpy_data_contract.md`：明确 contact array/sparse input contract。

### 不动

- IPC 数值公式：barrier、CCD、active-set construction、friction-related internals。
- sampled penalty 数值公式、triangle sampling、DCD/CCD detection kernel。
- `runIPCSim` JSON schema and behavior。

## Task 拆分

### Task C1: Common contact specs + factory audit

- Add `contactEnergyFactory.h/.cpp` with:
  - `ContactSurfaceSpec`
  - `ContactVertexEmbedding`
  - `FloorSpec`
  - common validation helpers
  - `createFloorEnergy`
- Add IPC-specific specs:
  - `IPC::ParametersSpec`
- Add common obstacle specs:
  - `StaticObstacleSpec`
  - `LinearMovingObstacleSpec`
  - `ObstacleSpec = std::variant<...>`
- Add sampled penalty-specific specs:
  - `SampledPenalty::ParametersSpec`
  - `SampledPenalty::FrictionParametersSpec`
  - translation helpers
- Factory boundary:
  - C1 只实现 `createFloorEnergy` 和 shared validation/translation helpers；
  - `createIPCEnergy` 在 C5 与 `IPCContactEnergy` 一起实现；
  - `createSampledPenaltyEnergy` 和 `createFrictionalSampledPenaltyEnergy` 在 C6 与 sampled penalty energies 一起实现。
- Encode current `SurfaceIPCCore::Parameters` field mapping exactly:
  - `dhat`
  - `dhatExternal`
  - `kappa`
  - `epsEE`
  - `slackness`
  - `ccdThickness`
- Implement `dhatExternal == nullopt => dhat_external = dhat`.
- Validate shape/scalar errors in factory and throw `std::invalid_argument`.
- Add `contact_energy_factory_gtest` coverage for parameter translation and invalid shapes, including that friction parameters are not part of normal-only sampled penalty construction.

### Task C2: Step-aware base + embedded DOF map + stateful contact base

- ✅ Use Energy plan E1's `EnergyStateKind` / `PotentialEnergy::stateKind() const`; do not define a second enum in contact.
- ✅ Add `stepAwareEnergy.h` with `StepState` and `StepAwareEnergy`.
  → 文件：`src/core/nonlinearOptimization/stepAwareEnergy.h`（time integrator plan 已落地）。
- Add `evaluationStateAwareEnergy.h` under `NonlinearOptimization` with `EvaluationStateAwareEnergy::prepareEvaluationState(x) const`.
- Add `stepDependentEnergy.h` under `NonlinearOptimization` as a marker for energies that require step history such as `previousX` and positive `timestep`.
- Make `EnergySet` implement `EvaluationStateAwareEnergy` by forwarding evaluation states to children after mapping global DOFs to local DOFs.
- Update Newton solver to call `prepareEvaluationState(xEval)` immediately before each normal solver-owned evaluation point. Bounded line-search trial evaluations with an active `LineSearchAwareEnergy` scope skip this hook and consume the line-search superset instead; non-frozen line-search policies prepare each trial point explicitly.
- Preserve the existing max-step / line-search order: `computeMaxStepLimit(x, dxRaw)` first, clamp `dxRaw`, then `beginLineSearch(x, dxClamped)`.
- Add `statefulContactEnergy.h` with:
  - `ContactModelKind`
  - `ContactStepState = NonlinearOptimization::StepState`
  - `StatefulContactEnergy`
- Make `StatefulContactEnergy` derive from `StepAwareEnergy` and `EvaluationStateAwareEnergy`, return `EnergyStateKind::Displacement`, and implement `prepareEvaluationState(x) const` as `refreshActiveSet(x) const`.
- Keep `refreshActiveSet(x)`, `clearActiveSet()`, `beginLineSearch(...)`, `endLineSearch()`, and `prepareEvaluationState(x)` const, with active-set and line-search caches stored as mutable implementation state.
- Add common `EmbeddedDofMap` by extracting/migrating mapping and pullback logic from `MappedSurfacePotentialEnergy`.
- Add `ContactSurfaceAdapter` as the contact wrapper around `EmbeddedDofMap` plus optional `ContactVertexEmbedding`.
- Keep state convention as simulation displacement.
- Add tests:
  - `PotentialEnergy::stateKind()` defaults to `Generic`;
  - `StatefulContactEnergy::stateKind()` is `Displacement`;
  - `EmbeddedDofMap` maps simulation displacement to embedded displacement and positions;
  - `EmbeddedDofMap` validates row/column dimensions;
  - `EmbeddedDofMap` pullback gradient/Hessian matches the old mapped-surface behavior on a small fixture;
  - stateful contact interface is usable through `PotentialEnergy`, `LineSearchAwareEnergy`, `StepAwareEnergy`, and `EvaluationStateAwareEnergy` pointers;
  - `EnergySet::prepareEvaluationState` forwards only to evaluation-state-aware child energies and uses local DOF mapping;
  - Newton evaluation-state prepare hook invokes `prepareEvaluationState` before current-point evaluation, before non-frozen trial evaluation, before subiteration-one post-step gradient evaluation, and before optimizer final objective.

### Task C3: Obstacle hierarchy + ready construction

Status: implemented.

- Obstacle surface and pose cache infrastructure remains in `contact/ipc/external` for this API cutover, but the ownership/evaluation boundary is now common-style: construction produces typed obstacle objects and all IPC external algorithms consume `ObstacleSurfaceView`.
- Refactor `ObstacleSurface` to an abstract read-only base.
- Add `StaticObstacleSurface final`.
- Add non-final `MovingObstacleSurface` with virtual `setTime(double)`.
- Add `LinearMovingObstacleSurface final`.
- Add `TrajectoryObstacleSurface final` for custom sampler tests and legacy sampler-style dynamic obstacle fixtures; new factory paths still prefer `StaticObstacleSurface` / `LinearMovingObstacleSurface`.
- Static constructor builds pose cache directly from geometry.
- Moving constructor initializes itself at `t = 0.0`.
- Add `cloneSurface()` to every concrete obstacle class.
- Add typed `cloneStatic()` / `cloneMoving()` helpers so split storage can be copied without downcasting.
- Remove sampler-based static construction from the new path.
- Add C++ tests:
  - static obstacle is ready immediately after construction;
  - linear moving obstacle is ready immediately after construction;
  - moving obstacle changes pose after `setTime(t)`;
  - typed clone preserves current pose and cache validity.
  - sampler-style coverage constructs `TrajectoryObstacleSurface`, never raw `ObstacleSurface`.

### Task C4: SurfaceIPCCore obstacle ownership + views

Status: implemented.

- Replace `std::vector<ObstacleSurface> obstacles_` and `std::vector<bool> staticObstacles_` with split static/moving ownership.
- Add `ObstacleSlot` to preserve input order and stable `objectId`.
- Add `ObstacleSurfaceView` and `obstacleViews()`.
- Update external broad phase, external barrier assembler, and external max-step code to consume views.
- Rename `setObstacleTime` to `setMovingObstacleTime`.
- Remove `markObstacleStatic` from the construction path.
- Add tests:
  - mixed static/moving obstacles preserve input-order `objectId`;
  - `obstacleViews()` order matches input order;
  - static-only `setMovingObstacleTime(t)` is a no-op for geometry;
  - moving-only and mixed obstacle updates only change moving obstacle poses.
  - external broad phase / assembler / max-step tests pass `ObstacleSurfaceView` vectors, not owning obstacle containers.

### Task C5: IPCContactEnergy stateful implementation

- Add `IPCContactEnergy` as the public IPC contact energy class.
- Migrate/rename `EmbeddedSurfaceIPCPotentialEnergy` behavior into `IPCContactEnergy`; do not leave a long-lived wrapper final design.
- Implement:
  - `contactModelKind() == ContactModelKind::IPC`
  - `beginStep(...)` uses `state.time + state.timestep` for moving obstacle poses;
  - `refreshActiveSet(x)`
  - `clearActiveSet()`
  - `prepareEvaluationState(x)` through `StatefulContactEnergy` default;
  - `beginLineSearch(...)`
  - `endLineSearch()`
  - `setMovingObstacleTime(t)`
  - `createIPCEnergy(...)` facade construction through `contactEnergyFactory`
- Ensure `computeMaxStepLimit(x, dxRaw)` remains outside `beginLineSearch`; `beginLineSearch(x, dxClamped)` builds a frozen/superset active set for the clamped segment and does not rerun detection for trial points.
- Outside line search, direct evaluation without a matching exact active set is an error. Python/manual callers must explicitly call `refreshActiveSet(x)` / `refresh_active_set(x)` first.
- Add tests:
  - construction with no obstacles, static obstacle, and linear moving obstacle;
  - `beginStep` updates moving obstacle time to `state.time + state.timestep`;
  - `setMovingObstacleTime(t)` updates moving obstacle time to exactly `t`;
  - `refreshActiveSet` builds a reusable active set;
  - `prepareEvaluationState(x)` refreshes the exact active set before a normal evaluation point;
  - `beginLineSearch` freezes/supersets pairs;
  - direct evaluation without explicit refresh throws, while solver-owned evaluation succeeds through `prepareEvaluationState`;
  - `evaluateHessian` works for non-fixed Hessian topology.

### Task C6: SampledPenaltyContactEnergy and FrictionalSampledPenaltyContactEnergy stateful implementation

- Rename/migrate current `legacy_penalty` code to sampled penalty naming.
- Add `SampledPenaltyContactEnergy` as a long-lived normal-only `StatefulContactEnergy`.
- Add `FrictionalSampledPenaltyContactEnergy` deriving from `SampledPenaltyContactEnergy` and `NonlinearOptimization::StepDependentEnergy`.
- Reuse current kernels:
  - `TriangleMeshExternalContactHandler`
  - `TriangleMeshSelfContactHandler`
  - `PointPenetrationEnergy`
  - `PointTrianglePairCouplingEnergyWithCollision`
- Implement:
  - `contactModelKind() == ContactModelKind::SampledPenalty`
  - normal `SampledPenaltyContactEnergy::beginStep(...)` updates moving obstacle poses at `state.time + state.timestep` and releases old active buffers, but does not require `previousX` or friction state;
  - frictional `beginStep(...)` requires `state.previousX != nullptr` and `state.timestep > 0`, stores `previousX`, `timestep`, and time, then delegates common work;
  - `refreshActiveSet(x)` runs detection and builds active external/self terms;
  - `func` / `gradient` / safe one-shot `hessian` sum active external/self terms after Energy E0 rename;
  - `beginLineSearch(...)` freezes active terms and does not rerun detection;
  - `clearActiveSet()` releases active terms and buffers.
  - `createSampledPenaltyEnergy(...)` facade construction through `contactEnergyFactory`.
  - `createFrictionalSampledPenaltyEnergy(...)` facade construction through `contactEnergyFactory`.
- Add protected active-term configuration hooks so normal energy sets buffers/stiffness/position functions and any normal self-contact kernel setup such as `computeClosestPosition(...)`; frictional energy additionally sets last-position functions, friction coefficient, timestep, and velocity epsilon.
- Add internal `SampledPenaltyActiveSet` as the single owner for active external/self penalty energies and buffers.
- Add tests:
  - no active contacts returns zero energy/gradient/Hessian;
  - active external contact evaluates finite energy/gradient/Hessian;
  - active self contact evaluates finite energy/gradient/Hessian;
  - normal sampled penalty does not set friction inputs and can be used without `previousX`;
  - frictional sampled penalty rejects missing `previousX` and non-positive `timestep`;
  - frictional sampled penalty updates previous-state/friction inputs;
  - `FrictionalSampledPenaltyContactEnergy` is discoverable through `StepDependentEnergy`;
  - `prepareEvaluationState(x)` refreshes active terms;
  - direct evaluation without explicit refresh throws, while solver-owned evaluation succeeds through `prepareEvaluationState`;
  - line-search evaluation does not rebuild active terms.

### Task C7: C++ floor/contact factory energy tests

- Floor tests:
  - construct via `createFloorEnergy`;
  - check `getNumDOFs() == map.cols()`;
  - zero displacement evaluates finite;
  - `setFloorHeight` changes energy as expected;
  - input matrix/map temporaries may be destroyed after construction.
- IPC tests:
  - construct via `createIPCEnergy` with no obstacles;
  - construct with one static obstacle;
  - construct with one linear moving obstacle;
  - call `setMovingObstacleTime(t)`;
  - evaluate IPC energy, gradient, Hessian via helper;
  - all outputs finite;
  - `evaluateHessian` works for non-fixed Hessian topology.
- Sampled penalty tests:
  - construct via `createSampledPenaltyEnergy`;
  - construct via `createFrictionalSampledPenaltyEnergy`;
  - validate `ContactVertexEmbedding` requirement;
  - call `beginStep` and `refreshActiveSet`;
  - evaluate value, gradient, Hessian via helper;
  - all outputs finite;
  - normal sampled penalty is not `StepDependentEnergy`;
  - frictional sampled penalty is `StepDependentEnergy`.

### Task C8: runIPCSim contact setup migration

Status: implemented for contact setup/runtime lifecycle. Full `SimulationProblem` / `SimulationRuntime` class extraction is deferred as a runIPCSim architecture cleanup and is not required for this contact API cutover.

- Split runIPCSim setup/runtime responsibilities:
  - `SimulationProblem`: parsed mesh/formulation/material/contact/boundary/output specs;
  - `SimulationRuntime`: current state, solver service, `EnergySet`, stateful contact energies, output writers.
- Update `obstacleSetup.*` to produce new obstacle specs / obstacle set.
- Update `shellSetup.cpp` and `volumeSetup.cpp` to stop calling `markObstacleStatic`.
- Update `legacyPenaltyContact.*` to sampled penalty naming and long-lived energy usage.
- Replace `ContactBackendKind::LegacyPenalty` with `ContactBackendKind::SampledPenalty`.
- Remove CLI `--legacy`; do not keep a deprecated alias.
- Add `--contact-model sampled-penalty` as the sampled penalty selection path.
- Preserve existing JSON behavior for static external obstacle cases.
- Preserve existing dynamic obstacle behavior if current examples/configs rely on linear motion.
- Preserve existing sampled penalty JSON fields such as `contact-stiffness`, `contact-samples`, `contact-friction-coeff`, and `contact-vel-eps`.
- Dynamic sampled penalty configs with `contact-friction-coeff > 0` construct `FrictionalSampledPenaltyContactEnergy`; configs with `contact-friction-coeff == 0` construct normal `SampledPenaltyContactEnergy`.
- Physical static solve validates contact terms and rejects `StepDependentEnergy` by default. Static sampled penalty therefore supports frozen external-only normal contact (`contact-friction-coeff == 0`) and rejects frictional sampled penalty unless a future static-friction policy is added. Static sampled self-contact is not enabled in v1.
- Per-step loops call `beginStep(...)`; Newton solver / optimizer call `prepareEvaluationState(x)` before normal evaluation points. JSON/setup parsing code must not own active-set refresh.
- Add or update a focused setup-level test if the repo has one; otherwise add a short manual validation command to the implementation PR notes.

### Task C9: Python value objects and bindings

Status: implemented.

- Add `pypgo/contact.py`:
  - `ContactSurface`
  - `ContactVertexEmbedding`
  - `FloorParameters`
  - `IPCParameters`
  - `SampledPenaltyParameters`
  - `FrictionParameters`
  - `ObstacleSpec.static(...)`
  - `ObstacleSpec.linear_velocity(...)`
  - `FloorEnergy`
  - `IPCEnergy`
  - `SampledPenaltyEnergy`
  - `FrictionalSampledPenaltyEnergy`
- Add `contact_bindings.cpp`:
  - convert `ContactSurface` to `ContactSurfaceSpec`;
  - convert optional `ContactVertexEmbedding`;
  - convert `FloorParameters` / split kwargs to `FloorSpec`;
  - convert `IPCParameters` to `IPC::ParametersSpec`;
  - convert `SampledPenaltyParameters` to `SampledPenalty::ParametersSpec`;
  - convert `FrictionParameters` to `SampledPenalty::FrictionParametersSpec`;
  - convert static / linear moving `ObstacleSpec` to C++ `ObstacleSpec`;
  - call C++ factory functions;
  - wrap returned `shared_ptr<PotentialEnergy>` using Energy plan handle protocol;
  - expose `state_kind` through the shared Energy binding's C++ `stateKind()` mapping;
  - expose `begin_step(...)`, `refresh_active_set(x)`, and `clear_active_set()` for contact energies;
  - expose `IPCEnergy.set_moving_obstacle_time(t)`;
  - expose `FrictionalSampledPenaltyEnergy` as a concrete energy whose `begin_step(...)` requires `previous_x` and positive `timestep`;
  - do not expose `set_obstacle_time`.
- Python errors:
  - invalid array shapes / dtype -> `ValueError`;
  - invalid axis / side -> `ValueError`;
  - non-finite / invalid scalar params -> `ValueError`.

### Task C10: Python contact tests

- Add `tests/pypgo/test_contact.py`:
  - `ContactSurface` validates shape and map dimensions;
  - `ContactVertexEmbedding` validates shape/arity and is required for sampled penalty;
  - `FloorEnergy` construction from split kwargs and from `FloorParameters`;
  - `FloorEnergy.set_height` updates value and `height`;
  - `IPCEnergy` construction with default params;
  - `IPCParameters(dhat=...)` maps `dhat_external is None` to same C++ value by behavioral or binding-level parity test;
  - static obstacle construction works without explicit time initialization;
  - linear moving obstacle construction works and `set_moving_obstacle_time(t)` changes evaluation behavior in a deterministic fixture;
  - `IPCEnergy.begin_step` and `refresh_active_set` work;
  - `SampledPenaltyEnergy` construction works with embedding;
  - `SampledPenaltyEnergy.begin_step` and `refresh_active_set` work;
  - `FrictionalSampledPenaltyEnergy` construction works with embedding and `FrictionParameters`;
  - `FrictionalSampledPenaltyEnergy.begin_step` rejects missing `previous_x` and non-positive `timestep`;
  - no `set_obstacle_time` attribute exists;
  - `value` / `gradient` / `hessian` work for floor, IPC, normal sampled penalty, and frictional sampled penalty;
  - contact energies combine in `EnergySet`;
  - deleting input NumPy arrays / `ContactSurface` after construction does not break evaluation;
  - `state_kind == "displacement"`;
  - `pypgo.contact` public names do not include raw C++ internals.

### Task C11: Docs and downstream alignment

- Update `api_coverage.md` contact section.
- Update `numpy_data_contract.md`:
  - `ContactSurface.rest_vertices`: `(n, 3) float64`
  - `ContactVertexEmbedding.indices`: flat int array
  - `ContactVertexEmbedding.weights`: flat float array
  - `surface_triangles`: `(m, 3) int64`
  - obstacle `rest_vertices`: `(n, 3) float64`
  - obstacle `triangles`: `(m, 3) int64`
  - obstacle `velocity`: `(3,) float64`
  - `surface_from_simulation_disp_map`: `pypgo.sparse.SparseMatrix` first; SciPy adapter if sparse plan provides one.
- Add future notes:
  - moving floor;
  - surface pressure;
  - sim config construction uses `contactEnergyFactory`.
  - solver/energy plan follow-up should extract `MaxStepAwareEnergy` from the current `PotentialEnergy::computeMaxStepLimit(...)` optional behavior.

## Self-review / plan audit（2026-06-04）

本轮 review 结论：核心设计决策已经定稿，当前 plan 不再保留需要实现者临场拍板的 contact lifecycle 分叉。实现时若发现 repo truth 与本节冲突，以本节和 §4/§17/Task C2-C8 的 contract 为准，并把偏差写入实现 PR notes。

已明确的决策：

- `StepState.time` 是 step start time；moving obstacle contact 默认用 `state.time + state.timestep` 作为 evaluation/target time。
- Newton evaluation-state prepare hook 是 v1 必需 contract；solver 在 normal evaluation 前显式调用 `prepareEvaluationState(xEval)`，不走“下一次 eval lazy rebuild”作为语义。
- `computeMaxStepLimit(x, dxRaw)` 在 line search 之前执行；`beginLineSearch(x, dxClamped)` 只接收已经 clamp 后的方向并冻结/构造 pair superset。
- lazy refresh 机制取消；direct evaluation 缺少 matching active set 时直接抛错，Python/manual 路径必须显式 `refresh_active_set(x)`，solver/runIPCSim 路径依赖 `prepareEvaluationState(xEval)`。
- `IPCContactEnergy` 直接 rename/migrate 现有 `EmbeddedSurfaceIPCPotentialEnergy` 行为；不采用长期 wrapper final design。
- `StepDependentEnergy` 是 per-step/history dependency marker；physical static solve builders 默认拒绝它，generic optimizer 不拒绝。
- sampled penalty 拆成 normal-only `SampledPenaltyContactEnergy` 和 frictional `FrictionalSampledPenaltyContactEnergy`；normal energy 本体可长期持有 external/self active set，physical static 默认只允许 frozen external-only adapter；frictional 是 `StepDependentEnergy`。
- `runIPCSim --legacy` 直接移除；sampled penalty 选择路径是 `--contact-model sampled-penalty`，内部 `ContactBackendKind::LegacyPenalty` 迁移为 `SampledPenalty`。

自检发现并已在 plan 中修正：

- `EvaluationStateAwareEnergy::prepareEvaluationState(x)` 必须是 `const`，因为 `EnergySet` 和 solver 以 `shared_ptr<const PotentialEnergy>` / const evaluation contract 组合能量；`refreshActiveSet`、`clearActiveSet`、line-search hooks 同样按 const cache mutation 设计。
- `ContactModelKind` 目标 API 必须包含 `Floor`，否则 factory 产出的 floor contact energy 无法通过同一 contact-kind contract 表达。

剩余实现风险（不需要新增设计决策，但需要测试覆盖）：

- sampled penalty 的 surface displacement、simulation displacement、vertex embedding 三者在 shell/volume 路径里历史语义容易混淆；C6/C8 必须用 shell identity map 和 volume embedding fixtures 各测一条。
- obstacle hierarchy 迁移会同时触碰 broad phase、assembler、max-step 和 setup code；C3/C4/C5 必须保持 `objectId` input-order 稳定，并用 mixed static/moving fixtures 验证。
- `runIPCSim` setup/runtime split 是较大整理；如果实现 PR 为了控制风险分两步落地，第一步也必须留下等价命名边界，并保证 contact lifecycle 不再散落在 JSON/setup parsing code 中。
- active-set lifecycle 测试不要只测“能 eval”；还要断言 direct eval 缺 explicit refresh 会抛错，solver/run-loop path 通过 `prepareEvaluationState` 成功，line-search trial 不触发 exact detection。

## 验收标准

- `contact_energy_factory_gtest` 全部通过。
- `stateful_contact_energy_gtest` 全部通过。
- `sampled_penalty_contact_energy_gtest` 全部通过。
- `python -m pytest tests/pypgo/test_contact.py` 全部通过。
- `pypgo/examples/contact_api_demo.ipynb` 由 `pypgo/examples/scripts/generate_contact_api_demo.py` 生成，并能 top-to-bottom 执行。
- contact API demo 使用仓库内具体 mesh asset，生成 `pypgo/examples/outputs/contact_api_demo.abc`。
- Python construction never directly instantiates `EmbeddedSurfaceFloorPotentialEnergy` / `EmbeddedSurfaceIPCPotentialEnergy` / sampled penalty handlers; it calls `contactEnergyFactory`.
- `IPCParameters(dhat=x, dhat_external=None)` maps to C++ `dhat_external == x`.
- `SampledPenaltyParameters` maps only to normal sampled penalty stiffness/sample/self/external-contact values.
- `FrictionParameters` maps to sampled penalty friction/velocity-eps values.
- `IPCContactEnergy`, `SampledPenaltyContactEnergy`, and `FrictionalSampledPenaltyContactEnergy` derive from `StatefulContactEnergy`.
- `FrictionalSampledPenaltyContactEnergy` derives from `NonlinearOptimization::StepDependentEnergy`; normal `SampledPenaltyContactEnergy` does not.
- `StatefulContactEnergy` derives from `StepAwareEnergy` and `EvaluationStateAwareEnergy`, implements `prepareEvaluationState(x)` via `refreshActiveSet(x)`, and reports `EnergyStateKind::Displacement`.
- `EnergySet` forwards evaluation-state prepare calls to child energies with correct local DOF mapping.
- Newton solver dispatches evaluation-state prepare immediately before normal evaluation points, and skips it while a bounded line-search superset is active.
- `computeMaxStepLimit(x, dxRaw)` runs before line search; `beginLineSearch(x, dxClamped)` receives the already-clamped direction.
- `EmbeddedDofMap` is the shared mapping/pullback implementation used by contact surface construction.
- `beginStep(...)`, `refreshActiveSet(x)`, and `clearActiveSet()` are implemented for IPC, normal sampled penalty, and frictional sampled penalty.
- `beginStep(...)` treats `StepState.time` as step start time; moving obstacle contact evaluation defaults to `state.time + state.timestep`.
- Solver/run-loop integration refreshes stateful contact active sets explicitly through `EvaluationStateAwareEnergy::prepareEvaluationState(xEval)`.
- Direct contact evaluation without explicit refresh/prepare throws instead of lazy-refreshing.
- Static obstacle construction does not require sampler/update/time initialization.
- Linear moving obstacle construction is ready at `t = 0.0`.
- `IPCEnergy.set_moving_obstacle_time(t)` updates moving obstacles only and uses exactly `t`, not `t + timestep`.
- Physical static solve builders reject `StepDependentEnergy` by default; generic optimizer APIs do not.
- Static sampled penalty supports frozen external-only normal contact and rejects frictional sampled penalty by default. Static sampled self-contact is dynamic-only in v1 unless a later explicit static self-contact policy is added.
- No Python `set_obstacle_time` compatibility method exists.
- No new C++ `setObstacleTime` compatibility wrapper remains in the migrated path.
- No public construction path calls `markObstacleStatic`.
- No public API or CLI uses `legacy` naming for sampled penalty contact.
- `runIPCSim --legacy` is removed, not kept as a deprecated alias.
- `runIPCSim --contact-model sampled-penalty` selects sampled penalty contact.
- Sampled penalty does not rebuild active terms during line search.
- Sampled penalty active external/self energies and buffers are owned through `SampledPenaltyActiveSet`, not scattered public-class members.
- Mixed static/moving obstacles preserve input-order `objectId`.
- Broad phase / external barrier / max-step code consumes `ObstacleSurfaceView`, not owning obstacle containers.
- `FloorEnergy` / `IPCEnergy` / `SampledPenaltyEnergy` / `FrictionalSampledPenaltyEnergy` Hessian calls use dynamic Hessian helper and do not call topology-fixed allocation path.
- `pypgo.contact` public surface contains only:
  - `ContactSurface`
  - `ContactVertexEmbedding`
  - `FloorEnergy`
  - `FloorParameters`
  - `IPCEnergy`
  - `IPCParameters`
  - `SampledPenaltyEnergy`
  - `SampledPenaltyParameters`
  - `FrictionalSampledPenaltyEnergy`
  - `FrictionParameters`
  - `ObstacleSpec`
- `pypgo.contact` public surface does not expose:
  - `MappedSurfacePotentialEnergy`
  - `EmbeddedSurfaceFloorPotentialEnergy`
  - `EmbeddedSurfaceIPCPotentialEnergy`
  - `EmbeddedDofMap`
  - `ContactSurfaceAdapter`
  - `IPCContactEnergy`
  - `SampledPenaltyContactEnergy`
  - `FrictionalSampledPenaltyContactEnergy`
  - `StatefulContactEnergy`
  - `StepDependentEnergy`
  - `FloorPenaltyParameters`
  - `SurfaceIPCCore`
  - `ObstacleSurface`
  - `StaticObstacleSurface`
  - `MovingObstacleSurface`
  - `LinearMovingObstacleSurface`
  - `ObstacleSurfaceView`
  - `TriangleMeshExternalContactHandler`
  - `TriangleMeshSelfContactHandler`
  - `PointPenetrationEnergy`
  - `PointTrianglePairCouplingEnergyWithCollision`
  - `markObstacleStatic`
  - `setObstacleTime`
  - `set_obstacle_time`
- `contact_bindings.cpp` does not use ad-hoc keep-alive containers for input arrays.
- runIPCSim setup/runtime migration has an explicit `SimulationProblem` / `SimulationRuntime` split or an equivalent named boundary documented in the implementation PR.
- Existing `runIPCSim` JSON field meanings are preserved after migration, except for the explicitly documented sampled penalty static external-only/frictional behavior. CLI `--legacy` removal is intentional and documented.

## Dependencies & Execution Order

### 外部依赖

- **Energy plan E0/E1**: contact hessian naming, dynamic Hessian helper, and shared `EnergyStateKind`.
- **Energy plan E4/E6**: Python `PotentialEnergy` handle and `EnergySet`.
- **Solver/Energy follow-up**: `MaxStepAwareEnergy` extraction is recommended but not a blocking prerequisite for this contact plan.
- **Sparse plan / M2**: `pypgo.sparse.SparseMatrix` or accepted sparse adapter for `surface_from_simulation_disp_map`.
- **Solver plan** is not required for contact construction tests. Solver E2E can be added after `solve_newton` exists.

### 内部任务依赖

```text
C1 (factory/spec)
  ├─ C2 (step-aware base + embedded map + stateful base)
  │    ├─ C3 (obstacle hierarchy)
  │    │    └─ C4 (core ownership + views)
  │    │         ├─ C5 (IPCContactEnergy)
  │    │         └─ C6 (SampledPenaltyContactEnergy + FrictionalSampledPenaltyContactEnergy)
  │    │              └─ C7 (C++ factory energy tests)
  │    │                   ├─ C8 (runIPCSim migration)
  │    │                   └─ C9 (Python value objects + bindings; also depends on Energy E4/E6)
  │    │                        └─ C10 (Python tests)
  └─ C11 (docs; last)
```

### 推荐顺序

C1 -> C2 -> C3 -> C4 -> C5 -> C6 -> C7 -> C8 -> C9 -> C10 -> C11.

C5 / C6 可在 C4 后并行，但 C7/C9 必须等两者都完成。C9 还必须等 Energy E4/E6，因为 Python contact energy 要包装成统一 `pypgo.energy.PotentialEnergy` 并能进入 `EnergySet`。

### 全局执行顺序（跨 plan）

原计划顺序（本 plan 第 4，Time Integrator 第 5）：

```
1. Solver plan          ✅ 已完成
2. Constraints plan     ✅ 已完成
3. Implicit Surface plan ✅ 已完成
4. Contact plan         ← 本 plan（当前）
5. Time Integrator plan ✅ 已完成（在 contact 之前实施）
```

**实际执行顺序的变更：** Time Integrator plan 在 contact plan 之前实施完成。
原计划要求 C2 产出 `StepAwareEnergy` 作为 Time Integrator T7 的硬依赖。实际执行中，
`StepAwareEnergy` / `StepState` 作为共享基础设施直接落地在 `NonlinearOptimization`
（符合 §18.1 的设计），不属于 Contact namespace。Time Integrator 的 `ImplicitEulerStepper::step()`
内已有 `dynamic_cast<StepAwareEnergy*>` 的 per-term dispatch 循环，contact plan 产出
`StatefulContactEnergy` 后自动生效，无需额外 adapter。

**结果：** contact plan 现在可以依赖已就绪的 `StepAwareEnergy` + `StepState` +
`DynamicStepper` 基础设施，从 C1/C2 的 `StatefulContactEnergy` 和 obstacle hierarchy 开始实施。

## 输出（供下游使用）

- C++:
  - `NonlinearOptimization::StepState`
  - `NonlinearOptimization::StepAwareEnergy`
  - `NonlinearOptimization::EvaluationStateAwareEnergy`
  - `NonlinearOptimization::StepDependentEnergy`
  - `ContactSurfaceSpec`
  - `ContactVertexEmbedding`
  - `ContactStepState`
  - `ContactModelKind`
  - `StatefulContactEnergy`
  - `EmbeddedDofMap`
  - `ContactSurfaceAdapter`
  - `FloorSpec`
  - `IPC::ParametersSpec`
  - `StaticObstacleSpec`
  - `LinearMovingObstacleSpec`
  - `ObstacleSpec`
  - `IPCContactEnergy`
  - `SampledPenalty::ParametersSpec`
  - `SampledPenalty::FrictionParametersSpec`
  - `SampledPenaltyActiveSet` (internal)
  - `SampledPenaltyContactEnergy`
  - `FrictionalSampledPenaltyContactEnergy`
  - `ObstacleSurface`
  - `StaticObstacleSurface`
  - `MovingObstacleSurface`
  - `LinearMovingObstacleSurface`
  - `ObstacleSurfaceView`
  - `createFloorEnergy(...)`
  - `createIPCEnergy(...)`
  - `createSampledPenaltyEnergy(...)`
  - `createFrictionalSampledPenaltyEnergy(...)`
  - `setMovingObstacleTime(...)`
  - `SimulationProblem` / `SimulationRuntime` boundary for `runIPCSim`
- Python:
  - `pypgo.contact.ContactSurface`
  - `pypgo.contact.ContactVertexEmbedding`
  - `pypgo.contact.FloorParameters`
  - `pypgo.contact.FloorEnergy`
  - `pypgo.contact.IPCParameters`
  - `pypgo.contact.SampledPenaltyParameters`
  - `pypgo.contact.FrictionParameters`
  - `pypgo.contact.ObstacleSpec.static(...)`
  - `pypgo.contact.ObstacleSpec.linear_velocity(...)`
  - `pypgo.contact.IPCEnergy`
  - `pypgo.contact.SampledPenaltyEnergy`
  - `pypgo.contact.FrictionalSampledPenaltyEnergy`
  - `pypgo.contact.IPCEnergy.begin_step(...)`
  - `pypgo.contact.SampledPenaltyEnergy.begin_step(...)`
  - `pypgo.contact.FrictionalSampledPenaltyEnergy.begin_step(...)`
  - `pypgo.contact.IPCEnergy.refresh_active_set(...)`
  - `pypgo.contact.SampledPenaltyEnergy.refresh_active_set(...)`
  - `pypgo.contact.FrictionalSampledPenaltyEnergy.refresh_active_set(...)`
  - `pypgo.contact.IPCEnergy.set_moving_obstacle_time(...)`
- Future:
  - M4/M6 `RunSimConfig` should construct floor / IPC / sampled penalty / obstacles through `contactEnergyFactory`, not through duplicated `runIPCSim/setup/*` construction logic.
