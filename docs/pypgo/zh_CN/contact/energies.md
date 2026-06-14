# `pypgo/contact/energies.py` — 三类接触能量实现

> 源文件：`pypgo/contact/energies.py`（222 行）。模块架构见 [overview.md](overview.md)。
>
> Python 层每个类只是参数校验 + 一个 `_core._create_*_contact_energy` 工厂调用 + 句柄持有；**全部公式在 C++**（`src/core/contact/`，工厂派发 `contactEnergyFactory.cpp`，绑定 `src/python/pypgo/contact/`）。本篇逐类给出每个接触模型的 as-implemented 能量形式（已逐一与 C++ 源核对）。

## 共同数学框架

各接触能量都继承 [`PotentialEnergy`](../energy/base.md)：能量定义在表面位置 $\mathbf x_s=\bar{\mathbf x}_s+S\mathbf u$ 上，对仿真 DOF 的梯度/Hessian 经 $S^\top(\cdot)\,S$ pullback（见 [surface.md](surface.md)）。构造共性：

```python
Energy(surface, surface_triangles, *, params=...)   # surface 必须是 ContactSurface
```

`surface_triangles` 是表面三角形索引 `(m,3)`（`FloorEnergy` 例外——逐顶点解析罚，不需要三角形）。除 `FloorEnergy` 外都混入 [`StatefulContactMixin`](base.md)（`begin_step` / `is_step_dependent`）。C++ 侧的共同边界是 step-aware 的 `StatefulContactEnergy`；具体能量各自组合映射、活动对生成和装配：IPC 用 `SurfaceDofMap`、`IPCPairGenerator`、`IPCContactAssembler` 与 `IPCActiveSetCache`，采样罚接触用 `SurfaceDofMap`、`SampledPenaltyContactBuilder`、`SampledPenaltyEvaluationBundle` 与 `SampledPenaltyContactEvaluator`。

三类模型的核心标量函数：

| 模型 | 核心函数 | 定义域 |
|---|---|---|
| 罚（floor / sampled penalty） | 单边二次罚 $\tfrac12\kappa\,d^2$，仅穿透侧激活 | 全空间（允许微穿透） |
| IPC 屏障 | $b(s,\hat s)=-(s/\hat s-1)^2\ln(s/\hat s)$，$s=d^2$ | $d>0$（不穿透，$d\to0$ 发散） |
| 摩擦 | 平滑化滑移势 $f_0(d)$（$C^1$，分段三次/线性） | 全空间 |

---

## class `FloorEnergy`

```python
FloorEnergy(surface, *, parameters=None, axis="z", side="keep_above",
            height=0.0, stiffness=1.0)
```

解析半空间（地板/天花板）的单边二次罚。参数二选一：整体给 `parameters`（[`FloorParameters`](params.md)），或给拆散字段（两者混用抛 `ValueError`，内部用 `_UNSET` 哨兵区分"没传"与"传 None"）。工厂 `_core._create_floor_contact_energy(surface, axis, side, height, stiffness)`。

**as-implemented**（`src/core/contact/floor/floorContactEnergy.cpp:77-122`）。记轴分量 $a$、平面高度 $h$、方向符号 $s$（`keep_above` 取 $s=+1$，罚 $x_a<h$；`keep_below` 取 $s=-1$，罚 $x_a>h$），逐表面顶点：

$$d_v = s\,\big(x_{v,a}-h\big),\qquad
E(\mathbf x_s)=\sum_{v\,:\,d_v<0}\tfrac12\,\kappa_f\,d_v^2$$

梯度只在违约顶点的 $a$ 分量非零（$\partial E/\partial x_{v,a}=\kappa_f\,d_v\,s$，90-102 行）；Hessian 是对角矩阵，违约顶点的 $(a,a)$ 元为 $\kappa_f$（104-122 行）——**单边激活由能量分段实现，没有活动集机制**，每次求值直接扫全部顶点。

**自检**（已数值验证）：合法侧（$d_v\ge0$）能量恒为 0、梯度为 0；穿透 $0.2$ 时 $E=\tfrac12\kappa_f(0.2)^2$ ✓。

注意 `FloorEnergy` **不混入** `StatefulContactMixin`：能量是位置的纯函数，没有步状态；静力学/动力学都无需 `begin_step`。

### 方法 `set_height(height)`

运行期改平面高度（C++ `setFloorHeight`，`floorContactEnergy.cpp:65-70`），同时重建 Python 侧的 `parameters` 快照保持一致。用于"地板缓慢抬升"之类的脚本化场景。

### 属性

`surface` / `parameters` / `axis` / `side` / `height` / `stiffness` 都是构造（或 `set_height` 后）的只读快照。

---

## class `SampledPenaltyEnergy`

```python
SampledPenaltyEnergy(surface, surface_triangles, *, params=None, friction=None)
```

采样罚接触；`friction=None` 时只有法向罚，传入 [`FrictionParameters`](params.md) 时启用半隐式 Coulomb 摩擦。工厂 `_core._create_sampled_penalty_contact_energy(surface, triangles, stiffness, samples, enable_self, enable_external, friction_coeff=None, velocity_eps=None)`，C++ 统一类 `Contact::SampledPenalty::SampledPenaltyContactEnergy`（`sampledPenaltyContactEnergy.cpp`）。法向罚参数见 [`SampledPenaltyParameters`](params.md)。

**采样**：每个表面三角形按 `samples` 细分布点（重心坐标组合），采样点 $i$ 的位置由表面顶点插值 $\mathbf p_i=\sum_j w_{ij}\,\mathbf x_j$；其面积权 $c_i$ 是该点分摊的三角形面积、按全网格最大值归一（`triangleMeshExternalContactHandler.cpp:121-133, 321-333`），故 $c_i\in(0,1]$。

**外部接触**（`kernels/pointPenetrationEnergy.cpp:185-252`）。活动集构建时对每个采样点查询各外部网格的最近点 $\mathbf p_{0,i}$ 与该处伪法线 $\mathbf n_i$，仅保留"法线相对 + 在内侧"的样本并取穿透最深的物体（`triangleMeshExternalContactHandler.cpp:444-457`）。能量是法向投影的单边二次罚：

$$E_{\text{ext}}(\mathbf x_s)=c\sum_{i\in\mathcal A}\tfrac12\,c_i\,\big[(\mathbf p_i-\mathbf p_{0,i})\cdot\mathbf n_i\big]^2,
\qquad \mathcal A=\{\,i:(\mathbf p_i-\mathbf p_{0,i})\cdot\mathbf n_i\le 0\,\}$$

其中 $c$ 是 `stiffness`（C++ `coeffAll`，`pointPenetrationEnergy.cpp:246` / `sampledPenaltyContactEnergy.cpp:124`）。穿透判据 `isInside` 即 $(\mathbf p-\mathbf p_0)\cdot\mathbf n\le0$（同文件 519-522 行）。Hessian 取 Gauss–Newton 形式 $c\,c_i\,w_{ij}w_{ik}\,\mathbf n\mathbf n^\top$（415-517 行）——忽略 $\mathbf p_0,\mathbf n$ 随位置的变化（活动集内冻结），天然 PSD。

**自接触**（`kernels/pointTrianglePairCouplingEnergyWithCollision.cpp:447-535`）。自碰撞检测产出采样点-三角形对；每对的法向穿透罚在活动集构建时被预组装成 12 维局部坐标（点 + 三角形三顶点）的二次型：

$$E_{\text{self}}=c\sum_{\text{pairs}}\Big(\tfrac12\,\mathbf x_\ell^\top H_p\,\mathbf x_\ell+\mathbf g_p^\top\mathbf x_\ell\Big)$$

（系数块 $H_p,\mathbf g_p$ 由 `computeClosestPosition` 以当前最近点/法线冻结生成，求值时仍复查接触状态。）`enable_self_contact` / `enable_external_contact` 分别开关两个处理器；`SampledPenaltyContactBuilder` 按当前表面位置构建 `SampledPenaltyEvaluationBundle`，`SampledPenaltyContactEvaluator` 再完成能量、梯度、Hessian 及 fused 输出（`sampledPenaltyContactEnergy.cpp:90-218`、`sampledPenaltyContactBuilder.cpp`、`sampledPenaltyContactEvaluator.cpp`）。

罚模型**没有 `max_step`/CCD**——穿透由罚力事后推回，大步长下可能穿深；要硬保证用 [`IPCEnergy`](#class-ipcenergy)。

`begin_step` 在无摩擦时是**安全 no-op**；`is_step_dependent == False`。启用摩擦后，`begin_step` 必须收到 `previous_x` 与正 `timestep`，并且 `is_step_dependent == True`。

### 可选摩擦

额外参数 [`FrictionParameters`](params.md)（$\mu$ = `friction_coeff`、$\varepsilon_v$ = `velocity_eps`）启用 sampled penalty 摩擦：

```python
penalty = pypgo.contact.SampledPenaltyEnergy(
    surface, tris,
    params=pypgo.contact.SampledPenaltyParameters(stiffness=1e5, samples=3),
    friction=pypgo.contact.FrictionParameters(friction_coeff=0.4, velocity_eps=1e-2),
)
```

摩擦势 as-implemented（`pointPenetrationEnergy.cpp:83-104`）。每个活动样本以**步内位移差**近似切向滑移：

$$\mathbf r=\big(I-\mathbf n\mathbf n^\top\big)\big(\mathbf p-\mathbf p^{\,t}\big),\qquad
d=\|\mathbf r\|,\qquad k=\varepsilon_v\,h$$

其中 $\mathbf p^t$ 由 `begin_step` 注入的上一步位移 $\mathbf x^t$ 插值（`sampledPenaltyFrictionState.cpp:42-66`）。平滑滑移势（IPC 的 $f_0$ 同构，把静摩擦尖点 $|d|$ 在 $d<k$ 内换成三次多项式）：

$$f_0(d)=\begin{cases}-\dfrac{d^3}{3k^2}+\dfrac{d^2}{k}+\dfrac{k}{3} & d<k\\[4pt] d & d\ge k\end{cases}$$

每样本摩擦能量与法向力大小成比：

$$E_{\text{fric}}=\mu\,f_{n,i}\,f_0(d_i),\qquad
f_{n,i}=c\,c_i\,\big|(\mathbf p_i-\mathbf p_{0,i})\cdot\mathbf n_i\big|$$

半隐式含义：$f_n$ 与 $\mathbf n$ 随当前迭代更新，但梯度/Hessian 把 $f_n$ 视为常数（只微分 $f_0$）。

### 方法 `begin_step(*, time, timestep, previous_x=None)`

无摩擦时可省略 `previous_x`；有摩擦时 Python 先校验 `previous_x` 非空、`timestep > 0`，C++ 再存储 $\mathbf x^t,h$ 并重置活动集。

---

## class `IPCEnergy`

```python
IPCEnergy(surface, surface_triangles, *, params=None, obstacles=None)
```

IPC（Incremental Potential Contact）屏障接触：自接触 + 可选外部障碍物，配 ACCD 可行步长，**Newton 全程保证不穿透**。工厂 `_core._create_ipc_contact_energy(surface, triangles, dhat, dhat_external, kappa, eps_ee, slackness, ccd_thickness, obstacles)`，C++ `IPC::IPCContactEnergy` 组合 `SurfaceDofMap`（表面映射）、`IPCPairGenerator`（活动对 / 可行步长 / 障碍物位姿）、`IPCContactAssembler`（能量、梯度、Hessian 装配）和 `IPCActiveSetCache`（直接求值缓存与线搜索 superset）。参数语义见 [`IPCParameters`](params.md)，障碍物见 [`ObstacleSpec`](params.md)。

### 屏障函数

**as-implemented**（`ipc/geometry/ipcBarrier.cpp:20-49`，注释言明 matching Codim-IPC）：定义在**平方距离** $s=d^2$、$\hat s=\hat d^2$ 上，

$$b(s,\hat s)=-\Big(\frac{s}{\hat s}-1\Big)^2\ln\frac{s}{\hat s},\qquad 0<s<\hat s\ \text{（否则 }b=0\text{）}$$

$$\frac{\partial b}{\partial s}=-\frac1{\hat s}(r-1)\Big(2\ln r+\frac{r-1}{r}\Big),\qquad
\frac{\partial^2 b}{\partial s^2}=-\frac1{\hat s^2}\Big(2\ln r+\frac{4(r-1)}{r}-\frac{(r-1)^2}{r^2}\Big),\qquad r=\frac{s}{\hat s}$$

$d\to0^+$ 时 $b\to+\infty$（不穿透屏障），$d\to\hat d^-$ 时 $b,\ b',\ b''\to0$（$C^2$ 平滑关断）。

### 活动对与总能量

broad phase（空间哈希，AABB 膨胀 $\hat d$）枚举两类几何对（`ipc/broadPhase/surfaceIPCSelfBroadPhase.cpp`）：

- **PT 对**：表面顶点 × 非邻接三角形，权重 $w_{pt}=A_p\,A_t$（顶点 Voronoi 面积 × 三角形面积，静止网格上算，`surfaceIPCTopology.cpp:48-60`、`surfaceIPCPairs.h:22`）；
- **EE 对**：非邻接边 × 边，权重 $w_{ab}=\ell_a\,\ell_b$（静止边长积，`surfaceIPCTopology.cpp:63-67`、`surfaceIPCPairs.h:28`）。

每对贡献（`ipc/core/surfaceIPCBarrierKernels.cpp:66-67, 137-139`）：

$$E_{\text{IPC}}(\mathbf x_s)=\kappa\Big[\sum_{(p,t)}w_{pt}\,b\big(d^2_{pt},\hat d^2\big)
+\sum_{(a,b)}w_{ab}\,m_{ab}\,b\big(d^2_{ab},\hat d^2\big)\Big]$$

梯度/Hessian 经 $d^2$ 对 12 维局部坐标的链式（同文件 70-77、141-161 行）：

$$\nabla E=w\kappa\,\frac{\partial b}{\partial s}\,\nabla d^2,\qquad
\nabla^2E=w\kappa\Big(\frac{\partial^2b}{\partial s^2}\,\nabla d^2\,(\nabla d^2)^\top+\frac{\partial b}{\partial s}\,\nabla^2 d^2\Big)$$

每对的局部 Hessian 做特征值截断的 **PSD 投影**（`projectToPSD`，78 / 161 行），保证总 Hessian 半正定。

### 距离分类（PP/PE/PT/EE）

$d^2$ 不是单一公式——先按最近特征分类再派发（`ipc/geometry/ipcDistancePrimitives.cpp`）：

- `classifyPT`（23-118 行）：点-三角形对按最近特征退化为 **PP**（点-顶点）、**PE**（点-边，三条边各一种）或真 **PT**（内部投影），7 种情形；
- `classifyEE`（120-196 行，注释标明 based on Codim-IPC `DISTANCE_TYPE.h`）：边-边对退化为 4 种 PP、4 种 PE 或真 EE，9 种情形；近平行（叉积近零）时强制退化为 PE。

各退化情形的 $d^2$ 及解析导数（6/9/12 维）来自 Codim-IPC 自动生成代码（`geometry/generated/CIPC_autogen.h`），再嵌入回 12 维局部坐标（`embedPP`/`embedPE`，516-552 行）。

### EE mollifier

近平行边对的 EE 距离不连续可导，IPC 用 mollifier 平滑关断 EE 屏障、让 PT 屏障接管（`ipcDistancePrimitives.cpp:435-447`）：

$$x=\|\mathbf e_a\times\mathbf e_b\|^2,\qquad
m_{ab}=\begin{cases}\dfrac{x}{\varepsilon_\times}\Big(2-\dfrac{x}{\varepsilon_\times}\Big) & x<\varepsilon_\times\\[2pt] 1 & x\ge\varepsilon_\times\end{cases}$$

（$C^1$：$m(\varepsilon_\times)=1$、$m'(\varepsilon_\times)=0$。）`eps_ee = 0` 时整段跳过（kernel 中 `epsEe > 0` 才计算，`surfaceIPCBarrierKernels.cpp:126`），阈值量纲注意事项见 [params.md](params.md)。

### 障碍物

`obstacles` 是 [`ObstacleSpec`](params.md) 列表（静态 / 匀速平移）。障碍物-表面对用 $\hat d_{\text{ext}}$（`dhat_external`），kernel 复用 PT/EE 实现但只取**动态侧**的梯度/Hessian 块（障碍物顶点不是 DOF；`surfaceIPCBarrierKernels.cpp:170-247` 的 `pointStaticTriangle` / `staticPointTriangle` / `edgeStaticEdge`）。

### `max_step`（ACCD 可行步长）

实现 [`PotentialEnergy.max_step(x, dx)`](../energy/base.md)，求解器在线搜索第 6 步以其为步长上界（[../solver/overview.md](../solver/overview.md)）。流程（`ipcContactEnergy.cpp:215-231` → `IPCPairGenerator::computeMaxStepLimit` → `surfaceIPCMaxStep.cpp`）：

1. 把 $(\mathbf u,\Delta\mathbf u)$ 映成表面 $(\mathbf x_s, S\Delta\mathbf u)$；
2. 扫掠 AABB（每侧膨胀 `ccd_thickness`）+ 空间哈希枚举候选 PT/EE 对（自接触 + 障碍物三方向：表面点-障碍三角形、障碍点-表面三角形、边-边）；
3. 每对跑 **ACCD**（additive CCD，保守推进，`ipcCCD.cpp:26-82`，$\eta=0.1$）：以"接触"定义 $d\le\xi$（$\xi$ = `ccd_thickness`，104-106 行）迭代下界步进求首次接触时间 $\text{toi}$；
4. 发现 $\text{toi}<\alpha$ 即收紧 $\alpha\leftarrow\text{toi}\cdot s$（$s$ = `slackness`，`surfaceIPCMaxStep.cpp:206-208, 264-265`）。

注意线搜索期间**障碍物冻结在采样位姿**（`surfaceIPCMaxStep.cpp:395-401` 的 `obsDisp = 0`），步内运动不参与扫掠。

### `begin_step` 与 `set_moving_obstacle_time(time)`

`begin_step(time, timestep, ...)`（混入自 [base.md](base.md)）把运动障碍物推进到**步末时刻**：C++ `beginStep` 调 `setMovingObstacleTime(time + timestep)`（`ipcContactEnergy.cpp:234-236`）——隐式积分求步末状态，障碍物取步末位姿与之自洽。动力学循环由 C++ stepper 自动派发；静力学或自定义循环可手动调 `set_moving_obstacle_time(t)`（直转发 `_handle`，同时清空 `IPCActiveSetCache` 并更新 `IPCPairGenerator` 的障碍物位姿，`ipcContactEnergy.cpp:262-266`）。`is_step_dependent == False`：给定障碍物位姿后能量是位置的纯函数。

---

## 用法示例

```python
import numpy as np
import pypgo

surface = pypgo.contact.ContactSurface.from_surface_embedding(emb)
tris = emb.rest_surface.triangles

# IPC：自接触 + 匀速下压的板障碍物
ipc = pypgo.contact.IPCEnergy(
    surface, tris,
    params=pypgo.contact.IPCParameters(dhat=5e-3, kappa=1e4, slackness=0.9),
    obstacles=[pypgo.contact.ObstacleSpec.linear_velocity(
        plate_v, plate_f, velocity=[0, 0, -0.1])],
)

# 摩擦罚：动力学循环里 begin_step 由 C++ stepper 自动派发
fric = pypgo.contact.SampledPenaltyEnergy(
    surface, tris,
    params=pypgo.contact.SampledPenaltyParameters(stiffness=1e5, samples=3),
    friction=pypgo.contact.FrictionParameters(friction_coeff=0.4, velocity_eps=1e-2),
)

# 解析地板，运行期抬升
floor = pypgo.contact.FloorEnergy(surface, axis="z", height=0.0, stiffness=1e6)
floor.set_height(0.05)

total = pypgo.energy.EnergySet([(deform, 1.0), (ipc, 1.0), (floor, 1.0)])
```

真实调用见 `pypgo/tools/sim/_scene.py`（`_build_contact_energies`）、`examples/contact_api_demo.ipynb`、`tests/pypgo/test_contact.py`。

## 模型选择速查

| 需求 | 类 |
|---|---|
| 绝对不穿透（自接触/大步长） | `IPCEnergy` |
| 软接触 + Coulomb 摩擦 | `SampledPenaltyEnergy(..., friction=FrictionParameters(...))` |
| 外部碰撞、性能优先 | `SampledPenaltyEnergy` |
| 解析地面/天花板 | `FloorEnergy` |

## 交叉链接

- 参数值对象（每个字段的 C++ 落点）：[params.md](params.md)
- 表面映射 $S$ 与链式法则：[surface.md](surface.md)
- `begin_step` 协议与派发时机：[base.md](base.md)
- `max_step` 在 Newton 线搜索中的位置：[../energy/base.md](../energy/base.md)、[../solver/overview.md](../solver/overview.md)
- 动力学循环（自动派发 `begin_step`）：[../sim/overview.md](../sim/overview.md)
