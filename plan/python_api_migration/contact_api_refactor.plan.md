# Contact API Refactor Plan

> **状态日期：** 2026-05-28
> **适用范围：** C++ `contact/` (IPC + floor + mapped surface) 边界重构 + Python `pypgo.contact` binding。
> **执行约束：** 不重写 IPC 数值内核（barrier、CCD、active set 缓存）；本计划只重构 contact energy 的构造接口、ownership、与 `EnergySet` / Newton solver 的对接边界。

## 目标

让 Python 用户能从 NumPy / `pypgo` 既有 mesh / sparse 对象出发，直接构造可与 `EnergySet` 组合的 contact energy：

```python
import pypgo as pgo

floor = pgo.contact.FloorEnergy(
    surface_rest_vertices=surface_rest,            # (n, 3) float64
    surface_from_simulation_disp_map=A,            # pypgo.sparse.SparseMatrix
    axis="y",
    side="keep_above",
    height=0.0,
    kappa=1e6,
)

ipc = pgo.contact.IPCEnergy(
    surface_rest_vertices=surface_rest,
    surface_triangles=triangles,                   # (m, 3) int64
    surface_from_simulation_disp_map=A,
    params=pgo.contact.IPCParameters(dhat=1e-3),
    obstacles=[],
)

total = pgo.energy.EnergySet([
    (elastic, 1.0),
    (floor,   1.0),
    (ipc,     1.0),
])
```

不让 Python 看到：

- `hessian` vs `hessianDirect`（IPC `isHessianTopologyFixed() == 0`，由 `evaluateHessian` 统一）；
- `MappedSurfacePotentialEnergy` 内部 cached active set / line search 状态；
- `EmbeddedSurfaceIPCPotentialEnergy::setObstacleTime` / `markObstacleStatic` 之外的 obstacle 内部结构；
- 任何 raw pointer / 借用 `const SpMatD &` 引用的构造约定。

## 当前问题

相关文件：

- `src/core/contact/mappedSurfacePotentialEnergy.h/.cpp`
- `src/core/contact/embeddedSurfaceFloorPotentialEnergy.h/.cpp`
- `src/core/contact/ipc/embeddedSurfaceIPCPotentialEnergy.h/.cpp`
- `src/core/contact/ipc/core/surfaceIPCCore.{h,cpp}`
- `src/core/contact/ipc/external/obstacleSurface.{h,cpp}`
- `src/core/contact/legacy_penalty/*`（不在本计划主线绑定范围，见非目标）
- `src/python/pypgo/bindings/contact_bindings.cpp`（新增）

具体别扭点：

### 1. Hessian topology 非固定

`MappedSurfacePotentialEnergy::isHessianTopologyFixed()` 返回 `0`。直接绑 `hessian()` 会让 Python 用户看见 `H` 拓扑不对的错误。必须由 `evaluation.h::evaluateHessian` 统一走 `hessianDirect`，这一点已经在 `energy_api_refactor.plan.md::Task E1` 覆盖，本计划只需要保证 contact energy 始终通过该 helper 被求值。

### 2. State convention：simulation displacement

`MappedSurfacePotentialEnergy::func(simulationDisplacements)` 的 state 是 simulation displacement（不是 simulation absolute position，也不是 surface displacement）。内部用 `surfaceFromSimulationDispMap_` 把 simulation 位移映射到 surface 位移、再叠加 rest position 得到 surface 绝对位置。Python 用户应该看到统一的 displacement state（与 `deformation_energy` 一致）。

### 3. 构造接受 `const SpMatD &` 引用

`MappedSurfacePotentialEnergy` 的 ctor 接受 `const EigenSupport::SpMatD &surfaceFromSimulationDispMap` 并在内部存为成员（按值复制）；这没有 lifetime 问题。但 IPC obstacles 通过 `std::vector<ObstacleSurface>` 按值传，`ObstacleSurface` 内部如果借用外部数据，binding 必须自检。需要在 Task C2 内显式审计 `ObstacleSurface` 的 ownership。

### 4. Floor / IPC 参数表达

- `FloorPenaltyParameters` 用 `FloorAxis` / `FloorSide` enum + `floorHeight` / `floorKappa` double 表达，floorAxis 缺省值是 `INVALID` 且 `floorHeight` / `floorKappa` 是 `NaN` 哨兵。Python 用户不该构造 NaN 哨兵；binding 必须强制传齐。
- `SurfaceIPCCore::Parameters` 是 IPC 数值参数（`dhat`、`epsv`、`mu`、`maxNewtonIters` 等，具体字段视 core header），需要以 Python dataclass 形态稳定下来。

### 5. Obstacle 时间 / 静态标记

`EmbeddedSurfaceIPCPotentialEnergy::setObstacleTime(double t)` 与 `markObstacleStatic(int32_t)` 修改 mutable 状态。Python 用户需要稳定接口控制这两点：dynamic loop 每帧推进时间；静态 obstacle 应在构造或 setup 时一次性标记。

### 6. Ownership：依赖 surface rest + map 数据

IPC / floor 都依赖：

- surface rest vertices `(n, 3)`；
- surface from simulation displacement sparse map；
- IPC 额外依赖 triangles `(m, 3)`、`SurfaceIPCCore::Parameters`、`std::vector<ObstacleSurface>`。

这些当前在 ctor 内按值拷贝，已经具备 self-owning 语义；本计划要做的是在 Python wrapper 上确认这一点，**禁止**在 binding 层用 `keepAlive_` 等机制保活外部 Python 对象。

### 7. 没有统一的 contact factory

`runIPCSim` 通过 `setup/floorSetup.cpp` 和 `setup/obstacleSetup.cpp` 把 JSON config 翻译成 floor / obstacle 列表，再喂给 contact energy。Python 第一版只暴露程序构造接口；JSON 路径走 M4 `pypgo.sim.RunSimConfig` 的 floor / obstacle 字段，本计划只覆盖到 energy 构造层。

## 非目标

- 不重写 IPC 数值内核 / CCD / barrier / active set / friction。
- 不在 M3 重做 obstacle 的完整 lifecycle（dynamic obstacle motion 等留给 M6）。本计划只暴露 `set_obstacle_time` 与 `mark_obstacle_static`。
- 不绑定 `contact/legacy_penalty/*`。Volume legacy penalty backend 由 `pypgo.sim.from_config(..., backend="legacy_penalty")` (M4/M6) 走 sim context，不进入 `pypgo.contact` 的直接构造路径。
- 不为 surface pressure / moving floor 设计 first-class Python class；moving floor 列入 M6 表面。
- 不在本计划暴露 `MappedSurfacePotentialEnergy` 子类化能力到 Python。
- 不重做 `ObstacleSurface` 数据格式；只确认其 by-value ownership 满足 binding 要求。

## 设计决策

### 1. Contact energy 自包含 ownership

每个 contact energy class 自身就是完整的 owner：

- `EmbeddedSurfaceFloorPotentialEnergy` 持 `surfaceRestPositions_`、`surfaceFromSimulationDispMap_`、`FloorPenaltyParameters`。
- `EmbeddedSurfaceIPCPotentialEnergy` 持上述 + triangles、`SurfaceIPCCore`（含 params）、`std::vector<ObstacleSurface>`。

Python wrapper **不**额外保活 Python 端的 rest vertices / map / triangles：binding 时一律 copy 进 C++ owned 存储。这一规则与 `energy_api_refactor.plan.md::Task E2` 的 owning generic energies 一致。

### 2. Hessian 通过 `evaluateHessian` 统一

Python `IPCEnergy.hessian(u)` / `FloorEnergy.hessian(u)` 内部走 `pgo::NonlinearOptimization::evaluateHessian(*self.handle, u)`；不会触发 `isHessianTopologyFixed() == 0` 的拓扑错误路径。

### 3. Floor 参数：Python dataclass + 显式 enum

提供 Python dataclass：

```python
@dataclass(frozen=True)
class FloorParameters:
    axis: Literal["x", "y", "z"]
    side: Literal["keep_above", "keep_below"]
    height: float
    kappa: float
```

binding 时翻译为 `FloorPenaltyParameters`。string → enum 在 binding helper 内完成；不允许传 `None` / NaN。

`FloorEnergy` ctor 既可接 `FloorParameters` 实例，也可拆开传 `axis`/`side`/`height`/`kappa` 关键字，效果等价。

### 4. IPC 参数：稳定 Python dataclass

引入：

```python
@dataclass(frozen=True)
class IPCParameters:
    dhat: float
    # 后续字段镜像 SurfaceIPCCore::Parameters；具体清单在 Task C1 完成审计后定稿。
```

binding 时 field-by-field 写入 `SurfaceIPCCore::Parameters`，不暴露 raw C++ struct。新增 / 删除字段都需要同步更新 dataclass、binding 翻译、parity test。

### 5. Obstacle：第一版只支持静态列表 + 时间推进

```python
@dataclass(frozen=True)
class ObstacleSpec:
    rest_vertices: np.ndarray   # (n, 3) float64
    triangles: np.ndarray       # (m, 3) int64
    static_: bool = False        # 对应 markObstacleStatic
    # motion / time-dependent transform 留给 M6
```

`IPCEnergy` 构造接受 `obstacles: list[ObstacleSpec]`，binding 时翻译为 `std::vector<ObstacleSurface>` 并按 `static_` 调用 `markObstacleStatic(objectId)`。

提供运行时 setter：

```python
ipc.set_obstacle_time(t: float) -> None
```

不暴露 `obstacleSurfaces` 列表的可变 view。

### 6. State convention 在 Python 上声明 `"displacement"`

所有 contact energy `state_kind == "displacement"`，与 `deformation_energy` 一致；可以直接组合进 `EnergySet([elastic, floor, ipc])` 而不会被 `EnergySet` 判为 mixed。

### 7. Contact factory 不绑定

不暴露 `runIPCSim` 的 `floorSetup` / `obstacleSetup`。这两个函数在 M4 通过 `RunSimConfig.floors` / `RunSimConfig.obstacles` 重新出现（解析 JSON 后构造 `FloorEnergy` / `ObstacleSpec` 列表）。本计划只保证程序构造路径稳定。

## 目标 C++ API

不动现有 contact header；只在 binding 层做适配。所有 C++ 接口现状如下，本计划要求 binding 全部走这些既有方法：

```cpp
// src/core/contact/embeddedSurfaceFloorPotentialEnergy.h
EmbeddedSurfaceFloorPotentialEnergy(
  const EigenSupport::MXd &surfaceRestVertices,
  const EigenSupport::SpMatD &surfaceFromSimulationDispMap,
  const FloorPenaltyParameters &params);

// src/core/contact/ipc/embeddedSurfaceIPCPotentialEnergy.h
EmbeddedSurfaceIPCPotentialEnergy(
  const EigenSupport::MXd &surfaceRestVertices,
  const EigenSupport::MXi &surfaceTriangles,
  const EigenSupport::SpMatD &surfaceFromSimulationDispMap,
  const SurfaceIPCCore::Parameters &ipcParams = {},
  std::vector<ObstacleSurface> obstacleSurfaces = {});

void setObstacleTime(double t);
void markObstacleStatic(int32_t objectId);
```

如果 Task C2 审计发现 `ObstacleSurface` 内部持有 raw pointer 或借用 mesh，C++ 侧需要补一个 owning 构造重载（task 拆分内说明）。

## Python API 定稿草案

```python
import pypgo as pgo
import numpy as np

surface_rest = np.asarray(rest_vertices, dtype=np.float64)
triangles    = np.asarray(face_indices, dtype=np.int64)
A            = pgo.sparse.SparseMatrix.from_scipy(A_scipy)  # 或 M2 builder

floor = pgo.contact.FloorEnergy(
    surface_rest_vertices=surface_rest,
    surface_from_simulation_disp_map=A,
    axis="y",
    side="keep_above",
    height=0.0,
    kappa=1e6,
)

ipc_params = pgo.contact.IPCParameters(dhat=1e-3)
ipc = pgo.contact.IPCEnergy(
    surface_rest_vertices=surface_rest,
    surface_triangles=triangles,
    surface_from_simulation_disp_map=A,
    params=ipc_params,
    obstacles=[
        pgo.contact.ObstacleSpec(
            rest_vertices=obs_rest,
            triangles=obs_tris,
            static_=True,
        ),
    ],
)

ipc.set_obstacle_time(0.0)

assert floor.state_kind == "displacement"
assert ipc.state_kind   == "displacement"

total = pgo.energy.EnergySet([(floor, 1.0), (ipc, 1.0)])
u = total.zero_state()
g = total.gradient(u)
H = total.hessian(u)
```

约束：

- 所有数组在 ctor 内 copy 进 C++ owned 存储；Python `del surface_rest` 不影响 energy。
- `axis` 接受 `"x" | "y" | "z"`；`side` 接受 `"keep_above" | "keep_below"`；非法值抛 `ValueError`。
- `IPCParameters` 字段顺序与 dataclass 严格匹配 `SurfaceIPCCore::Parameters`。
- `FloorEnergy.height`、`FloorEnergy.kappa` 可读；不暴露 `set_floor_height` 之外的 mutator（floor motion 留给 M6）。
- `set_obstacle_time(t)` 与 dynamic loop 解耦：静态 solve 调一次 `t=0` 即可。
- 不暴露 `mark_obstacle_static`；该状态在构造时通过 `ObstacleSpec.static_` 决定。

`pypgo.contact` M3 表面：

```text
pypgo.contact
  FloorEnergy
  FloorParameters
  IPCEnergy
  IPCParameters
  ObstacleSpec
```

## File Map

### 新增

- `src/python/pypgo/bindings/contact_bindings.cpp`
- `pypgo/contact.py`：Python wrapper、dataclass、`state_kind`。
- `tests/pypgo/test_contact.py`：构造、求值、weight 组合、obstacle 时间推进、`del input arrays`。
- `tests/src/core/contact/contact_ownership_gtest.cpp`：confirm `EmbeddedSurfaceFloor/IPCPotentialEnergy` 不依赖输入数组 lifetime（characterization；本计划只新增 test，不改实现）。

### 修改

- `src/python/pypgo/CMakeLists.txt`：编入新 binding TU。
- `src/python/pypgo/bindings/module.cpp`：注册 `pypgo.contact` 子模块。
- `plan/python_api_migration/api_coverage.md`：Contact 一节加入 dataclass 字段约束、`state_kind`。
- `pypgo/__init__.py`：导出 `pypgo.contact`。

### 不动

- `embeddedSurfaceFloorPotentialEnergy.{h,cpp}`、`embeddedSurfaceIPCPotentialEnergy.{h,cpp}`、`mappedSurfacePotentialEnergy.{h,cpp}` 实现保持不变（除非 Task C2 审计发现真实 ownership 缺陷）。
- `contact/legacy_penalty/*`、`runIPCSim/setup/floorSetup.cpp`、`runIPCSim/setup/obstacleSetup.cpp` 不在本计划修改范围。

## Task 拆分

### Task C1: `IPCParameters` field audit

- 读 `src/core/contact/ipc/core/surfaceIPCCore.h`，列出 `SurfaceIPCCore::Parameters` 当前字段、类型、默认值。
- 在本 plan 的“IPC 参数定稿表”补一节，写明 Python `IPCParameters` 字段对照。
- 决定哪些字段 M3 公开、哪些 M3 隐藏使用默认值（例如内部 solver tolerance）。
- 不写代码；只产出文档增量。

### Task C2: Obstacle ownership audit

- 阅读 `ObstacleSurface` 定义和构造路径，确认它持有的几何/状态数据是否完全 by-value。
- 如果借用外部 mesh / pointer：在本 plan 补一节描述并新增 owning 构造重载（C++ 改动属于本 task，必须 by-value test 验证）。
- 如果完全 by-value：在本 plan 标记为已验证，binding 直接走现有 ctor。

### Task C3: 在 C++ 加 contact ownership test

- 新增 `tests/src/core/contact/contact_ownership_gtest.cpp`：
  - 构造 `EmbeddedSurfaceFloorPotentialEnergy` / `EmbeddedSurfaceIPCPotentialEnergy`；
  - 立即释放传入的 `MXd surfaceRestVertices` / `SpMatD map` / `MXi triangles` 临时对象；
  - 在零位移 + 小扰动位移下分别求 `func` / `gradient` / `evaluateHessian`；
  - 期望全部成功、数值有限。

### Task C4: Python wrapper — `FloorEnergy`

- 在 `contact_bindings.cpp` 绑 `EmbeddedSurfaceFloorPotentialEnergy`：
  - ctor 接 NumPy `surface_rest_vertices`、`pypgo.sparse.SparseMatrix` `surface_from_simulation_disp_map`、`FloorParameters` 或散字段。
  - 绑 `setFloorHeight(h)`，暴露为 `floor.set_height(h)`。
  - 暴露 `floor.height`、`floor.kappa`、`floor.axis`、`floor.side` 只读属性。
- 在 `pypgo/contact.py` 写 `FloorParameters` dataclass 与 `state_kind = "displacement"`。
- 测试：构造 → 求值 → `set_height` 影响 `value` → `del input arrays` 不影响 → 与 `EnergySet` 组合一致。

### Task C5: Python wrapper — `IPCEnergy`

- 绑 `EmbeddedSurfaceIPCPotentialEnergy`：
  - ctor 接 NumPy 数组 + `IPCParameters` + `list[ObstacleSpec]`。
  - 暴露 `set_obstacle_time(t)`。
  - 不暴露 `markObstacleStatic` 直接；改由 `ObstacleSpec.static_` 决定。
- 在 `pypgo/contact.py` 写 `IPCParameters`、`ObstacleSpec` dataclass + `state_kind = "displacement"`。
- 测试：
  - 无 obstacle 时构造 + 求值；
  - 单 static obstacle 构造 + `set_obstacle_time(0.0)` + 求值；
  - 与 `FloorEnergy` 组合进 `EnergySet`，`hessian(u)` 返回完整 sparse；
  - 输入 NumPy 数组释放后仍可求值。

### Task C6: 文档与覆盖矩阵更新

- 更新 `plan/python_api_migration/api_coverage.md` 中 Contact 一节，列出 `FloorEnergy`、`IPCEnergy`、`FloorParameters`、`IPCParameters`、`ObstacleSpec` 与 parity test 路径。
- 更新 `numpy_data_contract.md`：明确 `surface_rest_vertices (n,3) float64`、`surface_triangles (m,3) int64`、`surface_from_simulation_disp_map` 接受 `pypgo.sparse.SparseMatrix` 或 SciPy CSR。
- 不为 moving floor / dynamic obstacle / legacy penalty 写公开 API；列入 M4/M6。

## 验收标准

- `tests/src/core/contact/contact_ownership_gtest.cpp` 全部通过。
- `python -m pytest tests/pypgo/test_contact.py` 全部通过，覆盖：
  - Floor 构造、`set_height`、`value`/`gradient`/`hessian`、与 `EnergySet` 组合；
  - IPC 构造（无/有 obstacle）、`set_obstacle_time`、`value`/`gradient`/`hessian`、与 `EnergySet` 组合；
  - 输入 NumPy 数组释放后能量仍可求值；
  - `state_kind == "displacement"`；
  - 错误 `axis` / `side` 字符串抛 `ValueError`。
- `pypgo.contact` 没有出现 `markObstacleStatic`、`FloorPenaltyParameters`、`SurfaceIPCCore` 等 C++ 类型名。
- `contact_bindings.cpp` 内不使用 `std::shared_ptr<void>` 或 ad-hoc keep-alive 容器；所有 ownership 由 `EmbeddedSurfaceFloor/IPCPotentialEnergy` 自身承担。
- `runIPCSim` 既有 JSON 配置行为不受影响（本计划不动 setup 路径）。
