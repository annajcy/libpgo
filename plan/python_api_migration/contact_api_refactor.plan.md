# Contact API Refactor Plan

> **状态日期：** 2026-05-29
> **适用范围：** C++ `contact/` construction boundary + Python `pypgo.contact` binding.
> **执行约束：** 不重写 IPC barrier、CCD、active-set 数值逻辑或 floor/contact 能量公式。本计划只重构 contact energy 的长期构造边界、ownership、obstacle lifecycle、Python API、以及与 `EnergySet` / solver service 的对接方式。

## 目标

建立一个长期可复用的 C++ contact construction facade：

```cpp
namespace pgo::Contact::IPC
{

struct ContactSurfaceSpec;
struct FloorSpec;
struct IPCParametersSpec;
struct StaticObstacleSpec;
struct LinearMovingObstacleSpec;
using ObstacleSpec = std::variant<StaticObstacleSpec, LinearMovingObstacleSpec>;

std::shared_ptr<EmbeddedSurfaceFloorPotentialEnergy> createFloorEnergy(
  const ContactSurfaceSpec &surface,
  const FloorSpec &floor);

std::shared_ptr<EmbeddedSurfaceIPCPotentialEnergy> createIPCEnergy(
  const ContactSurfaceSpec &surface,
  const EigenSupport::MXi &surfaceTriangles,
  const IPCParametersSpec &params,
  std::vector<ObstacleSpec> obstacles);

}  // namespace pgo::Contact::IPC
```

Python、未来 `RunSimConfig`、以及 C++ examples 都通过 facade 构造 contact energies，不直接拼 `FloorPenaltyParameters`、`SurfaceIPCCore::Parameters`、obstacle sampler 或 IPC core lifecycle。

Python 第一版：

```python
import pypgo as pgo

surface = pgo.contact.ContactSurface(
    rest_vertices=surface_rest,
    surface_from_simulation_disp_map=A,
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

ipc.set_moving_obstacle_time(t)

total = pgo.energy.EnergySet([(elastic, 1.0), (floor, 1.0), (ipc, 1.0)])
result = pgo.solver.solve_newton(total, x0=u0, fixed_dofs=fixed)
```

Python 用户不看到：

- `MappedSurfacePotentialEnergy` 内部 adapter/cache/lifecycle；
- `SurfaceIPCCore`；
- `ObstacleSurface` / `StaticObstacleSurface` / `MovingObstacleSurface` raw classes；
- obstacle sampler / pose cache construction；
- `FloorPenaltyParameters`、`SurfaceIPCCore::Parameters` raw structs；
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

### 2. State convention 必须固定为 simulation displacement

`MappedSurfacePotentialEnergy::func(simulationDisplacements)` 的 state 是 simulation displacement，不是 simulation absolute position，也不是 surface displacement。内部用 `surfaceFromSimulationDispMap_` 把 simulation 位移映射到 surface 位移，再叠加 rest position 得到 surface 绝对位置。

Python `ContactSurface` 和所有 contact energies 都声明：

```text
state_kind == "displacement"
```

这样可以直接组合进 deformation energy 的 `EnergySet`，不会产生 mixed-state ambiguity。

### 3. Hessian topology 非固定

`MappedSurfacePotentialEnergy::isHessianTopologyFixed() == 0`。当前 `hessian(x, H)` / `createHessian(H)` 会抛，真正路径是 `hessianDirect(x, H)`。Energy plan E0/E1 会统一 naming 和 `evaluateHessian` helper；Contact binding 必须走 helper，不直接调用 topology-fixed path。

Contact plan 不单独 rename energy API，但需要在 tests 中覆盖 `FloorEnergy.hessian(u)` / `IPCEnergy.hessian(u)` 走动态 Hessian 路径。

### 4. 当前 obstacle lifecycle 有半初始化状态

当前 `ObstacleSurface` 构造后 `current_` 是 zero，真正 pose cache 只有 `update(t)` 后才建立。static obstacle 通过 `markObstacleStatic(objectId)` 间接 `update(0.0)`，moving obstacle 需要 caller 记得 `setObstacleTime(t)`。

这个设计把几件事混在一起了：

- static obstacle 不应该有 “sample at t = 0” 语义；
- moving obstacle 的时间推进不应该污染 static obstacle；
- `markObstacleStatic(objectId)` 暴露了 slot-index lifecycle；
- construction 后对象不是 ready-to-evaluate；
- `setObstacleTime` 名字太宽，实际只应该推进 moving obstacles。

本计划把 obstacle 重构成 static/moving 类型体系，construction 后 immediately ready。

### 5. Contact energy 有 mutable cache，不是线程共享纯函数

`EmbeddedSurfaceIPCPotentialEnergy` 内部有 mutable energy active-set cache 和 line-search active-set state。Python 文档必须说明：

- 可以把同一个 energy object 放进一个 solve/evaluation pipeline；
- 不承诺同一个 `IPCEnergy` instance 可被多个线程并发 evaluation / solve；
- 如需并发，应构造独立 energy instances。

这和 solver plan 的 `LineSearchAwareEnergy` freeze 规则相互依赖：line search policy 只负责 alpha，contact energy 自己管理 line-search active set。

### 6. Contact energy 不是 hard constraints

Floor / IPC 是 `PotentialEnergy`，不是 `ConstraintFunctions`。不要把 bbox/floor/contact 混入 `pypgo.constraints`。如果未来需要 hard geometric constraints，另走 constraints plan。

## 非目标

- 不重写 IPC barrier、CCD、broad phase、active-set construction、friction-related internals。
- 不绑定 `contact/legacy_penalty/*`。Legacy penalty backend 由 future sim context/config path 处理，不进入 `pypgo.contact` direct construction path。
- 不暴露 `MappedSurfacePotentialEnergy` subclassing 到 Python。
- 不暴露 `SurfaceIPCCore` raw API。
- 不暴露 obstacle concrete classes、sampler、pose cache 到 Python。
- 不保留 `markObstacleStatic(objectId)` public lifecycle。
- 不保留 `setObstacleTime(...)` / `set_obstacle_time(...)` compatibility method；统一迁移到 `setMovingObstacleTime(...)` / `set_moving_obstacle_time(...)`。
- 不为 moving floor / surface pressure 设计 first-class Python class；列入 future work。
- 不让 contact energy 进入 `pypgo.constraints`。
- 不承诺 `IPCEnergy` instance 的 concurrent evaluation thread-safety。
- 不改变 `runIPCSim` JSON 行为；只迁移它的 obstacle construction 调用路径。

## 关键设计决策

### 1. C++ 长期边界是 `contactEnergyFactory`

新增：

- `src/core/contact/contactEnergyFactory.h`
- `src/core/contact/contactEnergyFactory.cpp`

核心 public API：

```cpp
namespace pgo::Contact::IPC
{

struct ContactSurfaceSpec
{
  EigenSupport::MXd restVertices;
  EigenSupport::SpMatD surfaceFromSimulationDispMap;
};

struct FloorSpec
{
  FloorAxis axis = FloorAxis::INVALID;
  FloorSide side = FloorSide::KEEP_ABOVE;
  double height = std::numeric_limits<double>::quiet_NaN();
  double kappa = std::numeric_limits<double>::quiet_NaN();
};

struct IPCParametersSpec
{
  double dhat = 1e-1;
  std::optional<double> dhatExternal = std::nullopt;
  double kappa = 0.1;
  double epsEE = 0.0;
  double slackness = 1.0;
  double ccdThickness = 0.0;
};

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

std::shared_ptr<EmbeddedSurfaceFloorPotentialEnergy> createFloorEnergy(
  const ContactSurfaceSpec &surface,
  const FloorSpec &floor);

std::shared_ptr<EmbeddedSurfaceIPCPotentialEnergy> createIPCEnergy(
  const ContactSurfaceSpec &surface,
  const EigenSupport::MXi &surfaceTriangles,
  const IPCParametersSpec &params = {},
  std::vector<ObstacleSpec> obstacles = {});

SurfaceIPCCore::Parameters toSurfaceIPCParameters(const IPCParametersSpec &spec);
FloorPenaltyParameters toFloorPenaltyParameters(const FloorSpec &spec);

}  // namespace pgo::Contact::IPC
```

Facade responsibilities:

- validate shapes and finite scalar params before constructing kernel objects；
- translate `FloorSpec` to `FloorPenaltyParameters`；
- translate `IPCParametersSpec` to `SurfaceIPCCore::Parameters`；
- translate obstacle specs to concrete `StaticObstacleSurface` / `LinearMovingObstacleSurface` objects；
- construct IPC energy in a ready-to-evaluate state。

`EmbeddedSurfaceFloorPotentialEnergy` / `EmbeddedSurfaceIPCPotentialEnergy` stay as numerical implementation classes. Python binding should construct through the factory, not direct ctor calls.

### 2. `ContactSurfaceSpec` is the shared surface adapter input

Both floor and IPC require the same pair:

```text
surface rest vertices
surface-from-simulation displacement map
```

Python exposes this as:

```python
@dataclass(frozen=True)
class ContactSurface:
    rest_vertices: np.ndarray
    surface_from_simulation_disp_map: pgo.sparse.SparseMatrix
```

`ContactSurface` is not an energy. It is a reusable construction value object. Each energy construction copies data into C++ owned storage; deleting `ContactSurface` or its input arrays after construction does not affect the energy.

Factory validation:

- `restVertices.cols() == 3`
- `restVertices.rows() > 0`
- `surfaceFromSimulationDispMap.rows() == 3 * restVertices.rows()`
- `surfaceFromSimulationDispMap.cols() > 0`

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

### 4. Obstacle core uses explicit static/moving hierarchy

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

### 5. `SurfaceIPCCore` stores obstacles by type but exposes read-only views

`SurfaceIPCCore` should not make broad phase / assembler code depend on concrete obstacle ownership. Internally it owns static and moving obstacles separately:

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

### 6. Time API is explicit: moving obstacles only

Rename the lifecycle API:

```cpp
class SurfaceIPCCore
{
public:
  void setMovingObstacleTime(double t);
};

class EmbeddedSurfaceIPCPotentialEnergy
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
- `EmbeddedSurfaceIPCPotentialEnergy::setMovingObstacleTime` clears energy/line-search active-set caches, then asks `SurfaceIPCCore` to update only moving obstacles。

### 7. Python exposes static and linear moving obstacle specs

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

### 8. Obstacle copy semantics use `clone()`

With virtual obstacle surfaces, copying must be explicit:

- `ObstacleSurface::cloneSurface()` exists for generic read-only cloning when needed.
- `StaticObstacleSurface::cloneStatic()` deep-copies static obstacles into typed static storage.
- `MovingObstacleSurface::cloneMoving()` deep-copies moving obstacles into typed moving storage.
- `SurfaceIPCCore` copy constructor / assignment deep-clone all obstacles and preserve `obstacleOrder_`.
- `EmbeddedSurfaceIPCPotentialEnergy` copy behavior remains safe if the existing class is copied by value in tests or downstream code.

Do not silently make `SurfaceIPCCore` move-only unless a repo-wide audit proves no caller depends on copyability. The safer migration is deep clone.

### 9. Python `FloorEnergy` is a wrapper over factory output

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

### 10. `IPCEnergy` wrapper owns construction metadata but not raw internals

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

### 11. Hessian/evaluation goes through energy helpers

Python `FloorEnergy.value/gradient/hessian` and `IPCEnergy.value/gradient/hessian` use the same `PotentialEnergy` binding surface as other energies. Hessian must call the Energy plan helper:

```cpp
pgo::NonlinearOptimization::evaluateHessian(*energy, x)
```

This is required because contact energies are not Hessian-topology-fixed.

### 12. Contact energy construction is self-owning

Construction copies all inputs into C++ owned storage:

- `ContactSurfaceSpec.restVertices`
- `ContactSurfaceSpec.surfaceFromSimulationDispMap`
- `surfaceTriangles`
- obstacle rest vertices
- obstacle triangles
- linear velocity / reference time

Binding must not use ad-hoc keep-alive containers such as `std::shared_ptr<void>` just to keep Python arrays alive. Lifetime tests must delete Python inputs after construction and still evaluate successfully.

### 13. `runIPCSim` migrates to the new obstacle construction path

Because `ObstacleSurface` becomes abstract, existing `runIPCSim` setup code that returns `std::vector<ObstacleSurface>` must be migrated in the same implementation phase.

Migration rule:

- JSON behavior stays unchanged.
- `obstacleSetup.*` should return facade obstacle specs or an obstacle set accepted by `createIPCEnergy`.
- `shellSetup.cpp` / `volumeSetup.cpp` should not call `markObstacleStatic`.
- Any existing dynamic obstacle setup should express linear motion through `LinearMovingObstacleSpec` / `LinearMovingObstacleSurface`.

## 目标 C++ API

New public construction header:

```cpp
// src/core/contact/contactEnergyFactory.h
namespace pgo::Contact::IPC
{

struct ContactSurfaceSpec;
struct FloorSpec;
struct IPCParametersSpec;
struct StaticObstacleSpec;
struct LinearMovingObstacleSpec;
using ObstacleSpec = std::variant<StaticObstacleSpec, LinearMovingObstacleSpec>;

std::shared_ptr<EmbeddedSurfaceFloorPotentialEnergy> createFloorEnergy(
  const ContactSurfaceSpec &surface,
  const FloorSpec &floor);

std::shared_ptr<EmbeddedSurfaceIPCPotentialEnergy> createIPCEnergy(
  const ContactSurfaceSpec &surface,
  const EigenSupport::MXi &surfaceTriangles,
  const IPCParametersSpec &params = {},
  std::vector<ObstacleSpec> obstacles = {});

}  // namespace pgo::Contact::IPC
```

Existing numerical APIs remain available for internal code, but Python and new C++ construction examples use the factory. Obstacle lifecycle APIs are renamed, not duplicated:

```text
setObstacleTime       -> removed / replaced by setMovingObstacleTime
set_obstacle_time     -> never exposed
markObstacleStatic    -> removed from construction path
```

## Python API 定稿草案

```python
import pypgo as pgo
import numpy as np

surface = pgo.contact.ContactSurface(
    rest_vertices=np.asarray(surface_rest, dtype=np.float64),
    surface_from_simulation_disp_map=A,
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

ipc.set_moving_obstacle_time(0.25)

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
  FloorEnergy
  FloorParameters
  IPCEnergy
  IPCParameters
  ObstacleSpec
```

Not public:

```text
MappedSurfacePotentialEnergy
EmbeddedSurfaceFloorPotentialEnergy
EmbeddedSurfaceIPCPotentialEnergy
FloorPenaltyParameters
SurfaceIPCCore
ObstacleSurface
StaticObstacleSurface
MovingObstacleSurface
LinearMovingObstacleSurface
ObstacleSurfaceView
markObstacleStatic
setObstacleTime
set_obstacle_time
```

## File Map

### 新增

- `src/core/contact/contactEnergyFactory.h`
- `src/core/contact/contactEnergyFactory.cpp`
- `src/python/pypgo/bindings/contact_bindings.cpp`
- `pypgo/contact.py`
- `tests/src/core/contact/contact_energy_factory_gtest.cpp`
- `tests/pypgo/test_contact.py`

### 修改

- `src/core/contact/CMakeLists.txt`：编入 `contactEnergyFactory.*`。
- `src/core/contact/ipc/external/obstacleSurface.h/.cpp`：改为 abstract base + static/moving concrete hierarchy。
- `src/core/contact/ipc/core/surfaceIPCCore.h/.cpp`：split obstacle storage、stable `objectId`、`ObstacleSurfaceView`、`setMovingObstacleTime`。
- `src/core/contact/ipc/broadPhase/surfaceIPCExternalBroadPhase.cpp`：external obstacle path 改吃 view。
- `src/core/contact/ipc/broadPhase/surfaceIPCBroadPhase.h`：external obstacle signatures 改吃 view。
- `src/core/contact/ipc/core/surfaceIPCExternalBarrierAssembler.*`：external obstacle signatures 改吃 view。
- `src/core/contact/ipc/core/surfaceIPCMaxStep.*`：external obstacle signatures 改吃 view。
- `src/core/contact/ipc/embeddedSurfaceIPCPotentialEnergy.h/.cpp`：constructor 接新 obstacle ownership；`setObstacleTime` 改为 `setMovingObstacleTime`；移除 `markObstacleStatic` construction path。
- `src/tools/sim/runIPCSim/setup/obstacleSetup.*`：返回新 obstacle specs / obstacle set。
- `src/tools/sim/runIPCSim/setup/shellSetup.cpp`：移除 `markObstacleStatic` 调用，接入新 factory/spec。
- `src/tools/sim/runIPCSim/setup/volumeSetup.cpp`：移除 `markObstacleStatic` 调用，接入新 factory/spec。
- `tests/src/core/contact/CMakeLists.txt`：新增 `contact_energy_factory_gtest`。
- `src/python/pypgo/CMakeLists.txt`：编入 `contact_bindings.cpp`，确保 `pypgo_core` link `contact` / `nonlinearOptimization`。
- `src/python/pypgo/bindings/module.cpp`：注册 contact bindings。
- `pypgo/__init__.py`：导出 `pypgo.contact`。
- `plan/python_api_migration/api_coverage.md`：Contact 一节加入 factory/spec/wrapper coverage。
- `plan/python_api_migration/numpy_data_contract.md`：明确 contact array/sparse input contract。

### 不动

- IPC 数值公式：barrier、CCD、active-set construction、friction-related internals。
- `contact/legacy_penalty/*`。
- `runIPCSim` JSON schema and behavior。

## Task 拆分

### Task C1: Contact factory + spec audit

- Add `contactEnergyFactory.h/.cpp` with:
  - `ContactSurfaceSpec`
  - `FloorSpec`
  - `IPCParametersSpec`
  - `StaticObstacleSpec`
  - `LinearMovingObstacleSpec`
  - `ObstacleSpec = std::variant<...>`
  - translation helpers
  - `createFloorEnergy`
  - `createIPCEnergy`
- Encode current `SurfaceIPCCore::Parameters` field mapping exactly:
  - `dhat`
  - `dhatExternal`
  - `kappa`
  - `epsEE`
  - `slackness`
  - `ccdThickness`
- Implement `dhatExternal == nullopt => dhat_external = dhat`.
- Validate shape/scalar errors in factory and throw `std::invalid_argument`.
- Add `contact_energy_factory_gtest` coverage for parameter translation and invalid shapes.

### Task C2: Obstacle hierarchy + ready construction

- Refactor `ObstacleSurface` to an abstract read-only base.
- Add `StaticObstacleSurface final`.
- Add non-final `MovingObstacleSurface` with virtual `setTime(double)`.
- Add `LinearMovingObstacleSurface final`.
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

### Task C3: SurfaceIPCCore obstacle ownership + views

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

### Task C4: C++ floor/IPC factory energy tests

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

### Task C5: runIPCSim contact setup migration

- Update `obstacleSetup.*` to produce new obstacle specs / obstacle set.
- Update `shellSetup.cpp` and `volumeSetup.cpp` to stop calling `markObstacleStatic`.
- Preserve existing JSON behavior for static external obstacle cases.
- Preserve existing dynamic obstacle behavior if current examples/configs rely on linear motion.
- Add or update a focused setup-level test if the repo has one; otherwise add a short manual validation command to the implementation PR notes.

### Task C6: Python value objects and bindings

- Add `pypgo/contact.py`:
  - `ContactSurface`
  - `FloorParameters`
  - `IPCParameters`
  - `ObstacleSpec.static(...)`
  - `ObstacleSpec.linear_velocity(...)`
  - `FloorEnergy`
  - `IPCEnergy`
- Add `contact_bindings.cpp`:
  - convert `ContactSurface` to `ContactSurfaceSpec`;
  - convert `FloorParameters` / split kwargs to `FloorSpec`;
  - convert `IPCParameters` to `IPCParametersSpec`;
  - convert static / linear moving `ObstacleSpec` to C++ `ObstacleSpec`;
  - call C++ factory functions;
  - wrap returned `shared_ptr<PotentialEnergy>` using Energy plan handle protocol;
  - expose `IPCEnergy.set_moving_obstacle_time(t)`;
  - do not expose `set_obstacle_time`.
- Python errors:
  - invalid array shapes / dtype -> `ValueError`;
  - invalid axis / side -> `ValueError`;
  - non-finite / invalid scalar params -> `ValueError`.

### Task C7: Python contact tests

- Add `tests/pypgo/test_contact.py`:
  - `ContactSurface` validates shape and map dimensions;
  - `FloorEnergy` construction from split kwargs and from `FloorParameters`;
  - `FloorEnergy.set_height` updates value and `height`;
  - `IPCEnergy` construction with default params;
  - `IPCParameters(dhat=...)` maps `dhat_external is None` to same C++ value by behavioral or binding-level parity test;
  - static obstacle construction works without explicit time initialization;
  - linear moving obstacle construction works and `set_moving_obstacle_time(t)` changes evaluation behavior in a deterministic fixture;
  - no `set_obstacle_time` attribute exists;
  - `value` / `gradient` / `hessian` work for floor and IPC;
  - contact energies combine in `EnergySet`;
  - deleting input NumPy arrays / `ContactSurface` after construction does not break evaluation;
  - `state_kind == "displacement"`;
  - `pypgo.contact` public names do not include raw C++ internals.

### Task C8: Docs and downstream alignment

- Update `api_coverage.md` contact section.
- Update `numpy_data_contract.md`:
  - `ContactSurface.rest_vertices`: `(n, 3) float64`
  - `surface_triangles`: `(m, 3) int64`
  - obstacle `rest_vertices`: `(n, 3) float64`
  - obstacle `triangles`: `(m, 3) int64`
  - obstacle `velocity`: `(3,) float64`
  - `surface_from_simulation_disp_map`: `pypgo.sparse.SparseMatrix` first; SciPy adapter if sparse plan provides one.
- Add future notes:
  - moving floor;
  - surface pressure;
  - sim config construction uses `contactEnergyFactory`.

## 验收标准

- `contact_energy_factory_gtest` 全部通过。
- `python -m pytest tests/pypgo/test_contact.py` 全部通过。
- Python construction never directly instantiates `EmbeddedSurfaceFloorPotentialEnergy` / `EmbeddedSurfaceIPCPotentialEnergy`; it calls `contactEnergyFactory`.
- `IPCParameters(dhat=x, dhat_external=None)` maps to C++ `dhat_external == x`.
- Static obstacle construction does not require sampler/update/time initialization.
- Linear moving obstacle construction is ready at `t = 0.0`.
- `IPCEnergy.set_moving_obstacle_time(t)` updates moving obstacles only.
- No Python `set_obstacle_time` compatibility method exists.
- No new C++ `setObstacleTime` compatibility wrapper remains in the migrated path.
- No public construction path calls `markObstacleStatic`.
- Mixed static/moving obstacles preserve input-order `objectId`.
- Broad phase / external barrier / max-step code consumes `ObstacleSurfaceView`, not owning obstacle containers.
- `FloorEnergy` / `IPCEnergy` Hessian calls use dynamic Hessian helper and do not call topology-fixed allocation path.
- `pypgo.contact` public surface contains only:
  - `ContactSurface`
  - `FloorEnergy`
  - `FloorParameters`
  - `IPCEnergy`
  - `IPCParameters`
  - `ObstacleSpec`
- `pypgo.contact` public surface does not expose:
  - `MappedSurfacePotentialEnergy`
  - `EmbeddedSurfaceFloorPotentialEnergy`
  - `EmbeddedSurfaceIPCPotentialEnergy`
  - `FloorPenaltyParameters`
  - `SurfaceIPCCore`
  - `ObstacleSurface`
  - `StaticObstacleSurface`
  - `MovingObstacleSurface`
  - `LinearMovingObstacleSurface`
  - `ObstacleSurfaceView`
  - `markObstacleStatic`
  - `setObstacleTime`
  - `set_obstacle_time`
- `contact_bindings.cpp` does not use ad-hoc keep-alive containers for input arrays.
- `runIPCSim` JSON behavior is unchanged after migration.

## Dependencies & Execution Order

### 外部依赖

- **Energy plan E0/E1**: contact hessian naming and dynamic Hessian helper.
- **Energy plan E4/E6**: Python `PotentialEnergy` handle and `EnergySet`.
- **Sparse plan / M2**: `pypgo.sparse.SparseMatrix` or accepted sparse adapter for `surface_from_simulation_disp_map`.
- **Solver plan** is not required for contact construction tests. Solver E2E can be added after `solve_newton` exists.

### 内部任务依赖

```text
C1 (factory/spec)
  ├─ C2 (obstacle hierarchy)
  │    └─ C3 (core ownership + views)
  │         ├─ C4 (C++ factory energy tests)
  │         └─ C5 (runIPCSim migration)
  └─ C6 (Python value objects + bindings)
       └─ C7 (Python tests)
            └─ C8 (docs)
```

### 推荐顺序

C1 -> C2 -> C3 -> C4 -> C5 -> C6 -> C7 -> C8.

## 输出（供下游使用）

- C++:
  - `ContactSurfaceSpec`
  - `FloorSpec`
  - `IPCParametersSpec`
  - `StaticObstacleSpec`
  - `LinearMovingObstacleSpec`
  - `ObstacleSpec`
  - `ObstacleSurface`
  - `StaticObstacleSurface`
  - `MovingObstacleSurface`
  - `LinearMovingObstacleSurface`
  - `ObstacleSurfaceView`
  - `createFloorEnergy(...)`
  - `createIPCEnergy(...)`
  - `setMovingObstacleTime(...)`
- Python:
  - `pypgo.contact.ContactSurface`
  - `pypgo.contact.FloorParameters`
  - `pypgo.contact.FloorEnergy`
  - `pypgo.contact.IPCParameters`
  - `pypgo.contact.ObstacleSpec.static(...)`
  - `pypgo.contact.ObstacleSpec.linear_velocity(...)`
  - `pypgo.contact.IPCEnergy`
  - `pypgo.contact.IPCEnergy.set_moving_obstacle_time(...)`
- Future:
  - M4/M6 `RunSimConfig` should construct floor / IPC / obstacles through `contactEnergyFactory`, not through duplicated `runIPCSim/setup/*` construction logic.
