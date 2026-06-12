# `pypgo/contact/params.py` — 接触参数值对象

> 源文件：`pypgo/contact/params.py`（140 行）。模块架构见 [overview.md](overview.md)。
>
> 全部是冻结 dataclass，`__post_init__` 做范围校验与类型规范化（经 `pypgo._utils` 的 `finite_scalar` / `positive_scalar` / `nonnegative_scalar`，见 [../_utils.md](../_utils.md)）。每个参数下面标注它在 C++ 公式中的确切位置（对应公式推导见 [energies.md](energies.md)）。

## class `SampledPenaltyParameters`

```python
SampledPenaltyParameters(stiffness=1.0, samples=1,
                         enable_self_contact=True, enable_external_contact=True)
```

| 字段 | 数学角色 | 约束 |
|---|---|---|
| `stiffness` | 罚刚度总系数 $c$：进入 `coeffAll`，整个罚能量乘 $c$（`pointPenetrationEnergy.cpp:246`、`sampledPenaltyContactEnergy.cpp:124,142`）。$c=0$ 时活动集直接为空（`sampledPenaltyContactDetector.cpp:92-93`） | $\ge 0$ |
| `samples` | 每三角形采样点数：罚约束施加在三角形上的采样点（重心组合）而非仅顶点，提高接触分辨率 | 正整数 |
| `enable_self_contact` | 是否构建自接触处理器（`TriangleMeshSelfContactHandler`） | bool 化 |
| `enable_external_contact` | 是否构建外部网格接触处理器（`TriangleMeshExternalContactHandler`） | bool 化 |

## class `FrictionParameters`

```python
FrictionParameters(friction_coeff=1.0, velocity_eps=1.0)
```

| 字段 | 数学角色 | 约束 |
|---|---|---|
| `friction_coeff` | Coulomb 摩擦系数 $\mu$：摩擦能量 $=\mu\,f_n\,f_0(\|\mathbf r\|)$ 的乘子 | $\ge 0$ |
| `velocity_eps` | 速度平滑阈 $\varepsilon_v$：摩擦势 $f_0$ 在滑移量 $d<k=\varepsilon_v h$ 内用三次多项式平滑静摩擦尖点（$h$ 为时间步长；见 [energies.md](energies.md) 摩擦小节） | $> 0$ |

$\varepsilon_v$ 是**速度**量纲：滑移速度低于 $\varepsilon_v$ 的区域被视为"近静止"并平滑处理（canonical IPC 摩擦的 $\epsilon_v$ 同义）。

## class `IPCParameters`

```python
IPCParameters(dhat=1e-1, dhat_external=None, kappa=0.1,
              eps_ee=0.0, slackness=1.0, ccd_thickness=0.0)
```

| 字段 | 数学角色 | 约束 |
|---|---|---|
| `dhat` | 自接触屏障激活距离 $\hat d$：屏障作用于 $0<d^2<\hat d^2$（`ipcBarrier.cpp:20-27`），broad phase 的 AABB 膨胀半径也是它（`surfaceIPCSelfBroadPhase.cpp:46`） | $> 0$ |
| `dhat_external` | 障碍物接触的 $\hat d_{\text{ext}}$；`None` 时取 `dhat` | $> 0$ |
| `kappa` | 屏障刚度 $\kappa$：每对贡献 $w\,\kappa\,b(d^2,\hat d^2)$（`surfaceIPCBarrierKernels.cpp:66-67`） | $> 0$ |
| `eps_ee` | 边-边 mollifier 阈值 $\varepsilon_\times$：**0 = 关闭 mollifier**（kernel 中 `epsEe > 0` 才计算，`surfaceIPCBarrierKernels.cpp:126`）。注意 as-implemented 阈值直接比较 $\|\mathbf e_a\times\mathbf e_b\|^2 \ge \varepsilon_\times$（绝对量纲），canonical IPC 取 $\varepsilon_\times = 10^{-3}\,\|\bar{\mathbf e}_a\|^2\|\bar{\mathbf e}_b\|^2$（静止边长相对量）——本实现把换算责任留给调用方 | $\ge 0$ |
| `slackness` | CCD 松弛系数 $s$：发现碰撞时间 $\text{toi}<\alpha$ 时取 $\alpha\leftarrow \text{toi}\cdot s$（`surfaceIPCMaxStep.cpp:206-208`）。**注意默认 1.0 表示不留余量**；canonical IPC 常用 $s=0.8\sim0.9$ 在 toi 内侧留安全距离 | $> 0$ |
| `ccd_thickness` | 最小分离厚度 $\xi$：CCD 把"接触"定义为 $d\le\xi$ 而非 $d\le 0$（`ipcCCD.cpp:104-106`），broad-phase AABB 同步膨胀 $\xi$（`surfaceIPCMaxStep.cpp:112-114`）。给薄壳/退化网格留几何厚度 | $\ge 0$ |

派发到 `_core._create_ipc_contact_energy`，C++ 落点 `SurfaceIPCCore::Parameters`（`surfaceIPCCore.cpp:74-82`）。

## class `FloorParameters`

```python
FloorParameters(axis="z", side="keep_above", height=0.0, stiffness=1.0)
```

| 字段 | 数学角色 | 约束 |
|---|---|---|
| `axis` | 罚作用坐标轴 $a\in\{x,y,z\}$ | 三选一 |
| `side` | 半空间方向：`keep_above` 罚 $x_a<h$（地板，$s=+1$），`keep_below` 罚 $x_a>h$（天花板，$s=-1$）（`floorContactEnergy.cpp:34-47`） | 二选一 |
| `height` | 平面高度 $h$ | 有限 |
| `stiffness` | 罚刚度 $\kappa_f$：$E=\sum_v \tfrac12\kappa_f\,d_v^2$ | $\ge 0$ |

## class `ObstacleSpec`（IPC 外部障碍物）

字段：`kind`（`"static"` / `"linear_velocity"`）、`rest_vertices (n,3)`、`triangles (m,3)`、`velocity (3,)|None`、`reference_time`。`__post_init__` 深拷贝并校验（线速度障碍必须给 `velocity`）。属性 `t0` 是 `reference_time` 的别名。

### 静态方法 `static(rest_vertices, triangles)`

静止障碍物：位姿恒为 $\mathbf x_{\text{obs}}(t)=\bar{\mathbf x}_{\text{obs}}$。C++ 落点 `IPC::StaticObstacleSurface`。

### 静态方法 `linear_velocity(rest_vertices, triangles, velocity, *, reference_time=0.0, t0=None)`

匀速平移障碍物：

$$\mathbf x_{\text{obs}}(t) = \bar{\mathbf x}_{\text{obs}} + (t - t_0)\,\mathbf v$$

C++ 落点 `IPC::LinearMovingObstacleSurface`（`obstacleSurface.cpp`），时刻 $t$ 由 [`begin_step`](base.md) → `setMovingObstacleTime(t+h)` 或手动 `IPCEnergy.set_moving_obstacle_time` 推进。`t0` 关键字是 `reference_time` 的别名（后写优先）。

**CCD 注意**：线搜索期间障碍物**冻结在采样位姿**，步内运动不参与扫掠（`surfaceIPCMaxStep.cpp:395-401` 的 `obsDisp = 0`）——快速运动障碍物应配小时间步。

## 用法示例

```python
import pypgo.contact as contact

params = contact.IPCParameters(dhat=5e-3, kappa=1e4, slackness=0.9, ccd_thickness=1e-4)
obs = contact.ObstacleSpec.linear_velocity(plate_v, plate_f, velocity=[0, 0, -0.1], t0=0.0)

fric = contact.FrictionParameters(friction_coeff=0.4, velocity_eps=1e-2)
pen = contact.SampledPenaltyParameters(stiffness=1e5, samples=3)
```

## 交叉链接

- 参数消费方与公式推导：[energies.md](energies.md)
- 校验工具：[../_utils.md](../_utils.md)
