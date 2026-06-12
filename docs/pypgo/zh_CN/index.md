# pypgo 中文参考文档

pypgo 是 libpgo C++ 仿真引擎的 Python 门面包，覆盖：FEM 弹性体仿真（本构模型、单元 formulation、能量装配）、IPC / 罚函数接触、Newton 非线性优化、隐式时间积分、网格处理与动画 I/O。

本文档采用**理论优先**（theory-first）的组织方式：先给出每个模块对应的数学对象与公式，再挂上 Python API 与 C++ 实现位置。目录结构与 `pypgo/` 包的源码结构一一对应——每个 `.py` 文件有一篇同名文档，每个子包文件夹有一篇 `overview.md` 讲该模块的系统架构。

## 三层架构

```
┌────────────────────────────────────────────────────┐
│  pypgo/                Python 门面（本文档的对象）     │
│  纯 Python：参数校验、数据封装、API 设计              │
├────────────────────────────────────────────────────┤
│  src/python/pypgo/     nanobind 绑定层 (_core.abi3.so)│
│  每个子模块一组 bindings.cpp / core.cpp              │
├────────────────────────────────────────────────────┤
│  src/core/             C++ 数学引擎                  │
│  solidDeformationModel / contact / nonlinear-        │
│  Optimization / simulation / mesh / ...              │
└────────────────────────────────────────────────────┘
```

Python 层的类大多是**只读句柄**（handle peer 模式）：构造时把参数交给 `_core` 工厂函数，得到 C++ 对象存进 `_handle`，之后的求值（`value` / `gradient` / `hessian`）全部经虚函数派发到 C++。因此**真正的公式在 `src/core/` 里**；本文档中每个公式都标注其 C++ 实现文件。

## 全包理论流水线

一次典型的弹性体仿真按以下数学阶段展开（括号内为负责的 pypgo 子包）：

$$
\underbrace{\text{几何/网格}}_{\texttt{mesh}}
\;\to\;
\underbrace{\Psi(\mathbf F),\ \text{formulation}}_{\texttt{fem}}
\;\to\;
\underbrace{E(\mathbf u)=\textstyle\sum_i w_i E_i(\mathbf u)}_{\texttt{energy}}
\;\to\;
\underbrace{\min_{\mathbf u} E \ \text{s.t. bounds}}_{\texttt{solver}}
\;\to\;
\underbrace{\text{隐式时间积分}}_{\texttt{sim}}
\;\to\;
\underbrace{\text{动画/应力输出}}_{\texttt{animation}}
$$

| 阶段 | 数学对象 | 子包 | 文档 |
|---|---|---|---|
| 几何与网格 | 三角/四面体/立方体网格、SDF、嵌入映射 | `pypgo.mesh`, `pypgo.implicit` | [mesh/overview](mesh/overview.md), [implicit/overview](implicit/overview.md) |
| 运动学与本构 | 形变梯度 $\mathbf F$、能量密度 $\Psi(\mathbf F)$、应力 $\mathbf P=\partial\Psi/\partial\mathbf F$ | `pypgo.fem` | [fem/overview](fem/overview.md) |
| 离散化与装配 | $E(\mathbf u)=\sum_e\sum_q w_q\,\Psi(\mathbf F_q)$，formulation 提供形函数与求积 | `pypgo.fem` | [fem/energy](fem/energy.md), [fem/formulations](fem/formulations.md) |
| 通用能量与组合 | $E=\sum_i w_iE_i$、二次型、软固定、约束罚 | `pypgo.energy`, `pypgo.constraints` | [energy/overview](energy/overview.md), [constraints/overview](constraints/overview.md) |
| 接触 | IPC 屏障 $b(d)$、罚接触、Coulomb 摩擦 | `pypgo.contact` | [contact/overview](contact/overview.md) |
| 优化求解 | 阻尼 Newton $(\nabla^2E+\lambda I)\Delta x=-\nabla E$ + 线搜索 | `pypgo.solver` | [solver/overview](solver/overview.md) |
| 时间积分 | 增量势能 $\min_x \tfrac1{2h^2}\|x-\tilde x\|_M^2+E(x)$ | `pypgo.sim` | [sim/overview](sim/overview.md) |
| 后处理 | Alembic 动画、von Mises 应力场 | `pypgo.animation` | [animation/overview](animation/overview.md) |
| 命令行工具 | 以上全流程的 CLI 封装 | `pypgo.tools` | [tools/overview](tools/overview.md) |
| 基础设施 | 稀疏矩阵、并行控制、输入校验 | `pypgo.sparse` 等 | [sparse](sparse.md), [parallel](parallel.md), [_utils](_utils.md) |

## 总公式 ↔ 模块索引

跨包的核心数学量速查表（详细推导见各模块文档）：

| 数学量 | 公式 | Python API | C++ 实现 | 文档 |
|---|---|---|---|---|
| 势能求值接口 | $E(\mathbf x)$, $\nabla E$, $\nabla^2 E$ | `PotentialEnergy.value/gradient/hessian` | `nonlinearOptimization/potentialEnergy.h` 虚接口 | [energy/base](energy/base.md) |
| 线性能量 | $b^\top x$ | `energy.LinearEnergy` | `genericPotentialEnergies/linearPotentialEnergy.h` | [energy/algebraic](energy/algebraic.md) |
| 二次能量 | $\tfrac12 x^\top A x + b^\top x$ | `energy.QuadraticEnergy` | `genericPotentialEnergies/quadraticPotentialEnergy.cpp` | [energy/algebraic](energy/algebraic.md) |
| 顶点软固定 | $\tfrac{c}{2}\sum_i\|u_i+\bar x_i-t_i\|^2$ | `energy.VertexAttachment` | `constraintPotentialEnergies/multiVertexPullingSoftConstraints.cpp` | [energy/attachment](energy/attachment.md) |
| 嵌入点软固定 | $c\,\|W_s\,u\|^2$ | `energy.EmbeddedVertexAttachment` | 纯 Python 组装 → `QuadraticEnergy` | [energy/attachment](energy/attachment.md) |
| 约束罚 | $\tfrac{w}{2}\|C(x)\|^2$ | `energy.ConstraintPenalty` | `nonlinearOptimization/constraints/potentialEnergyFromConstraintFunctions.cpp` | [energy/penalty](energy/penalty.md) |
| 能量加权和 | $\sum_i w_i E_i$ | `energy.EnergySet` | `nonlinearOptimization` `EnergySet` | [energy/sets](energy/sets.md) |
| 硬约束函数 | $C(x)\in\mathbb R^m$, $\partial C/\partial x$ | `constraints.ConstraintFunction` | `nonlinearOptimization/constraints/` | [constraints/base](constraints/base.md) |
| 阻尼 Newton | $(\nabla^2E+\lambda I)\Delta x=-\nabla E$ | `solver.NewtonOptimizer` | `nonlinearOptimization/solver/newton/NewtonSolver.cpp` | [solver/optimizer](solver/optimizer.md) |
| Armijo 线搜索 | $f(x+\alpha p)\le f(x)+c\,\alpha\,g^\top p$ | `solver.Backtrack` | `nonlinearOptimization/solver/newton/lineSearch.cpp` | [solver/line_search](solver/line_search.md) |
| 弹性能量密度 | $\Psi(\mathbf F)$（StVK / StableNeo / …） | `fem.StVK` 等 | `solidDeformationModel/material/elastic/` | [fem/elastic](fem/elastic.md) |
| FEM 装配 | $E(\mathbf u)=\sum_e\sum_q w_q \Psi(\mathbf F_q)$ | `fem.deformation_energy` | `solidDeformationModel/deformation/deformationModelAssembler.cpp` | [fem/energy](fem/energy.md) |
| IPC 屏障 | $b(d)$，$d<\hat d$ 时发散 | `contact.IPCEnergy` | `contact/ipc/core/surfaceIPCBarrierKernels.cpp` | [contact/energies](contact/energies.md) |
| 隐式时间积分 | Backward Euler / TRBDF2 增量势能 | `sim.DynamicSimulation` | `simulation/` | [sim/overview](sim/overview.md) |

## 一个端到端示例

四面体网格上的静力平衡（重力 + 底面固定），展示各子包如何衔接：

```python
import numpy as np
import pypgo

# 1. mesh: 读 VEG 体网格（含材料）
veg = pypgo.mesh.read_veg("examples/assets/veg/tet/box.veg")
volume = pypgo.mesh.VolumeMesh.from_veg_file(veg)
sim_mesh = pypgo.fem.SimulationMesh.create_volumetric(volume)

# 2. fem: formulation × 本构 → 形变能量；重力经 body_force → LinearEnergy
fm = pypgo.fem.TetLinear()
deform = pypgo.fem.deformation_energy(
    sim_mesh,
    formulation=fm,
    elastic=pypgo.fem.StableNeo(),
    elastic_field=pypgo.fem.ElementwiseField(),
    plastic=pypgo.fem.VolumetricPlasticity(dofs=0),
    plastic_field=pypgo.fem.ElementwiseField(),
)
mass_field = pypgo.fem.volume_density(volume)
f_g = fm.body_force(sim_mesh, [0.0, -9.8, 0.0], mass_field)
gravity = pypgo.energy.LinearEnergy(-f_g)   # E = -f_gᵀu

# 3. energy: 底面顶点软固定，与形变能量加权求和
bottom = np.flatnonzero(veg.mesh_data.vertices[:, 1] < 1e-6)
pin = pypgo.energy.VertexAttachment(
    sim_mesh=sim_mesh, vertex_indices=bottom,
    target_positions=np.zeros(bottom.size * 3), coeff=1e6)
total = pypgo.energy.EnergySet([(deform, 1.0), (gravity, 1.0), (pin, 1.0)])

# 4. solver: Newton 求最小能量位移
problem = pypgo.solver.OptimizationProblem(objective=total)
opt = pypgo.solver.NewtonOptimizer(max_iterations=50, gradient_tolerance=1e-6)
result = opt.solve(problem, total.zero_state())
print(result.status, result.iterations)  # u* = result.x
```

## 目录

### 根模块
- [overview.md](overview.md) — pypgo 顶层架构（三层结构、handle peer 模式、惰性导入）
- [\_\_init\_\_.md](__init__.md) · [_utils.md](_utils.md) · [sparse.md](sparse.md) · [parallel.md](parallel.md)

### 子包
- [energy/](energy/overview.md) — 通用势能：代数能量、软固定、约束罚、能量组合
- [constraints/](constraints/overview.md) — 硬约束函数 $C(x)$ 与界
- [solver/](solver/overview.md) — Newton 优化器、线搜索、稀疏线性求解
- [fem/](fem/overview.md) — FEM formulation、本构模型、形变能量装配、可微层
- [contact/](contact/overview.md) — IPC / 罚函数 / 摩擦 / 地面接触
- [sim/](sim/overview.md) — 动力学状态、时间积分器、仿真主循环
- [mesh/](mesh/overview.md) — 网格数据、几何查询、体网格与材料、网格处理
- [implicit/](implicit/overview.md) — 隐式场（SDF）、采样网格、等值面提取
- [animation/](animation/overview.md) — 位移序列 I/O、Alembic、应力统计与 VDB
- [tools/](tools/overview.md) — 命令行工具（网格 / 动画 / 仿真）

### 相关资源
- 用法示例：`examples/*.ipynb`（17 个 demo notebook）、`examples/sim_configs/`（CLI 场景配置）
- API 实际调用参考：`tests/pypgo/`
