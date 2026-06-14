# `pypgo.contact` — 接触能量模块架构

> 包目录：`pypgo/contact/`（5 个文件）。上级架构见 [../overview.md](../overview.md)。

## 模块职责

把接触表达为**势能**：每个接触模型都是一个 [`PotentialEnergy`](../energy/base.md)，与弹性能量、软固定一起加进 [`EnergySet`](../energy/sets.md)，交给 Newton 求解器（[../solver/overview.md](../solver/overview.md)）统一最小化。模块提供三类接触模型：

| 模型 | 性质 | 数学核心 |
|---|---|---|
| **IPC**（Incremental Potential Contact） | 屏障型，**保证不穿透** | 对数屏障 $b(d^2,\hat d^2)$ + ACCD 可行步长 |
| **采样罚接触**（sampled penalty） | 罚型，允许微穿透 | 单边二次罚 $\tfrac12 c\,\langle (p-p_0)\!\cdot\!n\rangle_-^2$（可加摩擦） |
| **地板罚**（floor） | 罚型，解析半空间 | 单边二次罚 $\tfrac12\kappa\,\langle s(x_a-h)\rangle_-^2$ |

Python 层全部是**只读门面**：参数校验 + `_core._create_*` 工厂调用 + 句柄持有；公式与装配全部在 C++（`src/core/contact/`）。

## 理论流水线

接触能量定义在**接触表面**（三角网格）上，但求解变量是**仿真 DOF**（体网格位移 $\mathbf u$）。流水线分四阶段：

```
仿真位移 u ──S──> 表面位置 x_s ──broad phase──> 活动集（PT/EE 对）──> E, ∇E, ∇²E ──Sᵀ──> 仿真梯度/Hessian
                                  └── CCD ──> α_max（可行步长，回馈 Newton 线搜索）
```

**阶段 1 — 表面映射**（`src/core/contact/surfaceDofMap.cpp`）。接触表面位置由稀疏插值矩阵 $S$（`surface_from_simulation`）从仿真位移线性生成：

$$\mathbf x_s = \bar{\mathbf x}_s + S\,\mathbf u,\qquad
E(\mathbf u) = E_s(\mathbf x_s),\qquad
\nabla_{\mathbf u}E = S^\top \nabla_{\!s} E_s,\qquad
\nabla^2_{\mathbf u}E = S^\top H_s\, S$$

其中 $\bar{\mathbf x}_s$ 是表面静止位置。`SurfaceDofMap` 是共享的映射帮助类，负责由仿真位移生成表面位置，并把表面梯度 / Hessian pullback 到仿真 DOF。$S=I$ 时表面顶点即仿真顶点（`ContactSurface.identity`）；嵌入仿真时 $S$ 是重心插值矩阵（`ContactSurface.embedded`）。见 [surface.md](surface.md)。

**阶段 2 — 活动集构建**（broad phase，空间哈希）。以当前表面位置枚举距离小于 $\hat d$（IPC）或发生穿透（罚接触）的几何对；具体接触能量各自组合需要的映射、pair generator、assembler 或 evaluator。`StatefulContactEnergy` 只提供共同的 step-aware 接触边界；IPC 额外实现线搜索生命周期来在线搜索期间冻结 swept superset。

**阶段 3 — 能量装配**。以 IPC 为例（as-implemented，`src/core/contact/ipc/geometry/ipcBarrier.cpp:20-27`），屏障作用在**平方距离** $s=d^2$ 上：

$$b(s,\hat s) = -\Big(\frac{s}{\hat s}-1\Big)^2 \ln\frac{s}{\hat s},\qquad \hat s = \hat d^2,\quad 0<s<\hat s$$

总能量是活动对的加权和（权重 = 静止面积/边长乘积）：

$$E_{\text{IPC}}(\mathbf x_s) = \kappa\Big[\sum_{(p,t)\in\text{PT}} w_{pt}\, b\big(d^2_{pt},\hat d^2\big)
 + \sum_{(a,b)\in\text{EE}} w_{ab}\, m_{ab}\, b\big(d^2_{ab},\hat d^2\big)\Big]$$

逐项公式与 PP/PE/PT/EE 距离分类见 [energies.md](energies.md)。

**阶段 4 — 可行步长（CCD）**。IPC 能量定义域要求一切距离 $d>0$；Newton 试探步 $\mathbf x+\alpha\,\Delta\mathbf x$ 必须保持在定义域内。`IPCEnergy` 实现 [`PotentialEnergy.max_step`](../energy/base.md)：用 ACCD（additive CCD）算出首次接触时间并乘以松弛系数（`src/core/contact/ipc/core/surfaceIPCMaxStep.cpp`），求解器在线搜索第 6 步（见 [../solver/overview.md](../solver/overview.md)）以此为步长上界——这是 **Newton 全程不穿透**的机制。

## 状态化接触：`begin_step`

接触能量是**长寿命**对象（一次构造、跨步复用），但有两类逐步状态：

1. **摩擦**需要上一步位移 $\mathbf x^{t}$ 来定义切向滑移；
2. **运动障碍物**需要当前时刻 $t$ 来摆放位姿。

C++ 侧通过 `StepAwareEnergy::beginStep(StepState{time, timestep, previousX})` 注入（`src/core/nonlinearOptimization/stepAwareEnergy.h`），Python 侧由 [`StatefulContactMixin.begin_step`](base.md) 暴露。动力学循环里 **C++ stepper 在每步开始自动派发**（`src/core/simulation/common/dynamicStepperUtils.cpp:30-41`），无需 Python 驱动；静力学求解则需手动调用一次（见 [../sim/overview.md](../sim/overview.md)）。

## 模块 ↔ 数学 ↔ 阶段 主表

| 文件 | 数学对象 | 公式 | 文档 |
|---|---|---|---|
| `surface.py` | 表面映射 | $\mathbf x_s = \bar{\mathbf x}_s + S\mathbf u$ | [surface.md](surface.md) |
| `params.py` | 参数值对象 | $\hat d,\kappa,\varepsilon_{ee}$；$c,\mu,\varepsilon_v$；$h,\kappa_f$ | [params.md](params.md) |
| `base.py` | 步状态注入 | $\text{StepState}=(t,h,\mathbf x^t)$ | [base.md](base.md) |
| `energies.py` | 三类接触能量 | $b(d^2,\hat d^2)$、单边罚、可选摩擦势 $f_0$ | [energies.md](energies.md) |
| `__init__.py` | 公开面 | — | [\_\_init\_\_.md](__init__.md) |

## C++ 引擎对应

| Python | C++ 类 | 位置 |
|---|---|---|
| `ContactSurface` | `Contact::ContactSurfaceSpec`（peer `PyContactSurface`） | `src/python/pypgo/contact/core.cpp` |
| `FloorEnergy` | `Contact::Floor::FloorContactEnergy` | `src/core/contact/floor/floorContactEnergy.cpp` |
| `IPCEnergy` | `Contact::IPC::IPCContactEnergy` → `SurfaceDofMap` + `IPCPairGenerator` + `IPCContactAssembler` + `IPCActiveSetCache` | `src/core/contact/ipc/ipcContactEnergy.cpp`、`ipc/ipcPairGenerator.cpp`、`ipc/ipcContactAssembler.cpp` |
| `SampledPenaltyEnergy` | `Contact::SampledPenalty::SampledPenaltyContactEnergy`（可选摩擦） | `src/core/contact/sampled_penalty/sampledPenaltyContactEnergy.cpp` |
| `ObstacleSpec` | `IPC::StaticObstacleSurface` / `LinearMovingObstacleSurface` | `src/core/contact/ipc/external/obstacleSurface.cpp` |
| 表面↔仿真链式法则 | `Contact::SurfaceDofMap`（由具体接触能量组合使用） | `src/core/contact/surfaceDofMap.cpp` |
| `begin_step` 协议 | `NonlinearOptimization::StepAwareEnergy` | `src/core/nonlinearOptimization/stepAwareEnergy.h` |

绑定层：`src/python/pypgo/contact/bindings.cpp`（注册 `_core._create_*` 工厂与 `PyStatefulContactEnergy` 层级）+ `core.cpp`（包装实现）；C++ 工厂派发在 `src/core/contact/contactEnergyFactory.cpp`。

## 贯穿示例

球落地（IPC 自接触 + 静态障碍物 + 地板，配动力学循环）：

```python
import numpy as np
import pypgo

# 1) 接触表面：体网格的嵌入表面（S = 重心插值矩阵）
emb = pypgo.mesh.SurfaceEmbedding.from_volume(vol_mesh)   # 见 ../mesh/
surface = pypgo.contact.ContactSurface.from_surface_embedding(emb)

# 2) IPC 能量：d̂=5mm 屏障 + 静止地面网格障碍物
ipc = pypgo.contact.IPCEnergy(
    surface, emb.rest_surface.triangles,
    params=pypgo.contact.IPCParameters(dhat=5e-3, kappa=1e4),
    obstacles=[pypgo.contact.ObstacleSpec.static(ground_v, ground_f)],
)

# 3) 解析地板罚（z >= 0）
floor = pypgo.contact.FloorEnergy(surface, axis="z", side="keep_above",
                                  height=0.0, stiffness=1e6)

# 4) 总能量 = 弹性 + 接触，进动力学循环（begin_step 由 C++ stepper 自动派发）
total = pypgo.energy.EnergySet([(deform, 1.0), (ipc, 1.0), (floor, 1.0)])
sim = pypgo.sim.DynamicSimulation(mass=M, state=state0, timestep=1e-3,
                                  energy=total, integrator="implicit_euler")
frames = sim.run(200, external_force=gravity)
```

更多真实调用见 `pypgo/tools/sim/_scene.py`（`_build_contact_energies`）、`examples/contact_api_demo.ipynb`、`tests/pypgo/test_contact.py`。

## 模型选择速查

| 需求 | 模型 |
|---|---|
| 绝对不穿透（自接触/薄壳/大步长） | `IPCEnergy` |
| 软接触、要摩擦、能接受微穿透 | `SampledPenaltyEnergy(..., friction=FrictionParameters(...))` |
| 简单外部碰撞、性能优先 | `SampledPenaltyEnergy` |
| 解析地面/天花板 | `FloorEnergy` |
