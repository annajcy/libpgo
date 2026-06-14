# `pypgo/tools/sim/_config.py` — 场景配置 schema 与加载（私有）

> 源文件：`pypgo/tools/sim/_config.py`（527 行）。模块架构见 [overview.md](overview.md)。
>
> 全套 sim CLI 的**单一事实来源**：JSON schema、默认值、合法值与一致性校验都在这里。纯 Python dataclass，不触碰任何库对象——装配在 [_scene.md](_scene.md)。

## 优先级与路径解析

```
内置默认值  <  JSON 配置（--config）  <  CLI flags
```

- `load_config` 先读 JSON 为 dict，再把 CLI 覆盖按**点路径**（如 `"dynamic.timestep"`）写进同一 dict（`_set_dotted`），最后统一构造 dataclass——所以 CLI 覆盖与 JSON 字段享受**同一套校验**；
- JSON 内的路径字段（`mesh.volume/surface`、选择器 `file`、障碍物 `mesh`、`output.directory`）按 **JSON 文件所在目录**解析为绝对路径（`_resolve_json_paths`）；CLI 给的路径由 [_cli.md](_cli.md) 先 `resolve()`（按 CWD）再传入，原样使用。

## class `ConfigError(ValueError)`

一切非法/不一致配置的统一异常；CLI 层接住后走 `parser.error`（退出码 2）。

## 顶层 schema（`SimConfig`，冻结 dataclass）

```json
{
    "type": "tet",                  // 可选；与命令的 mesh_type 不符即报错
    "mesh": { ... },
    "material": { ... },
    "constraints": { ... },
    "loads": { "gravity": [0, 0, -9.8] },
    "contact": [ { ... }, ... ],
    "initial_state": { ... },
    "solver": { ... },
    "dynamic": { ... },
    "output": { ... }
}
```

| 字段 | 类型 | 文内小节 |
|---|---|---|
| `mesh_type` / `mode` | 由命令注入（非 JSON 字段） | — |
| `mesh` | `MeshConfig` | 下文 |
| `material` | `VolumeMaterialConfig` 或 `ShellMaterialConfig`（按 mesh_type） | 下文 |
| `constraints` | `ConstraintsConfig` | 下文 |
| `loads` | `LoadsConfig` | 下文 |
| `contact` | `tuple[ContactConfig, ...]` | 下文 |
| `initial_state` | `InitialStateConfig` | 下文 |
| `solver` | `SolverConfig` | 下文 |
| `dynamic` | `DynamicConfig` | 下文 |
| `output` | `OutputConfig` | 下文 |

合法值常量：`MESH_TYPES=("tet","cubic","shell")`、`VOLUME_FORMULATIONS=("auto","tet-linear","cubic-linear","cubic-tricubic-hermite")`、`VOLUME_ELASTIC_MODELS=("stable_neo","stvk","stvk_volume","linear_elastic","mooney_rivlin")`、`CONTACT_MODELS=("ipc","floor","sampled_penalty")`、`INTEGRATORS=("implicit_euler","trbdf2")`。

---

## 顶点选择器（`VertexSelector` / `RegionSelector`）

`constraints.fixed`、各 attachment 的 `vertices` 共用。**三选一**（多给或不给抛 `ConfigError`）：

```json
{ "file": "fixed.txt" }                       // 空白分隔的整数索引文件
{ "indices": [0, 5, 9] }                      // 内联索引
{ "region": { "axis": "z", "side": "max", "tolerance": 1e-6 } }
```

### class `RegionSelector`

| 字段 | 默认 | 含义 |
|---|---|---|
| `axis` | 必填 | `"x"/"y"/"z"` |
| `side` | 必填 | `"min"/"max"` |
| `tolerance` | `1e-6` | 选中坐标满足 $|c_i-c_{\text{bound}}|\le\text{tol}$ 的顶点（$c_{\text{bound}}$=该轴最小/最大值） |

构造期校验 axis/side 合法、tolerance 非负。实际解析（读文件/选区域、去重排序、范围检查）在 [_scene.md](_scene.md) 的 `resolve_vertex_selector`。

---

## 各 Section dataclass（字段 / 默认 / 含义）

### `MeshConfig` — `"mesh"`

| 字段 | 类型/默认 | 含义 |
|---|---|---|
| `volume` | Path / `None` | `.veg` 体网格；**tet/cubic 必填**，shell 禁止 |
| `surface` | Path / `None` | OBJ 曲面；**所有类型必填**（体：嵌入/接触面；shell：仿真网格本身） |
| `formulation` | `"auto"` | 体网格的离散化；`auto`→tet 取 `tet-linear`、cubic 取 `cubic-linear`；交叉组合（如 tet 配 `cubic-linear`）报错 |

### `VolumeMaterialConfig` — `"material"`（tet/cubic）

| 字段 | 默认 | 含义 |
|---|---|---|
| `model` | `"stable_neo"` | 弹性本构（5 选 1，映射见 [../../fem/elastic.md](../../fem/elastic.md)）；$E,\nu$ 本身来自 `.veg` 材料区域 |
| `density` | `None` | 覆盖密度（kg/m³）；`None` = 用 `.veg` 各区域密度 |
| `enable_material_max_step` | `True` | 透传 [`DeformationOptions`](../../fem/energy.md)（StVK 等模型的步长保护） |

### `ShellMaterialConfig` — `"material"`（shell）

| 字段 | 默认 | 含义 |
|---|---|---|
| `thickness` | `1e-3` | 壳厚 $h$（m） |
| `E_membrane` / `nu_membrane` | `1e6` / `0.4` | 膜 $E,\nu$（构造期烘焙，见 [../../fem/elastic.md](../../fem/elastic.md) `KoiterStVKShellMaterial`） |
| `mass` | `{density: 1000}` | `ShellMassConfig`：`areal_density`（kg/m²）与 `density`（kg/m³，乘厚度）**恰好二选一** |
| `enable_material_max_step` | `True` | 同上 |

### `ConstraintsConfig` — `"constraints"`

| 字段 | 默认 | 含义 |
|---|---|---|
| `fixed` | `None` | 选择器；选中**仿真网格顶点**的全部 DOF 做硬固定（消元，Hermite 下含导数 DOF——每顶点 24 个全 clamp） |
| `attachments` | `()` | `AttachmentConfig` 列表：仿真顶点软固定 |
| `surface_attachments` | `()` | `SurfaceAttachmentConfig` 列表：**嵌入曲面顶点**软固定（formulation 无关，Hermite 也可用） |

`AttachmentConfig`：`vertices`（选择器，必填）、`coeff`（默认 `1e5`）、`movement`（(3,) 速度向量或 `None`；非零 movement **要求 dynamic 模式**，static 下报错）。
`SurfaceAttachmentConfig`：`vertices`（在**曲面网格**顶点集上选）、`coeff`（默认 `1e5`）。能量形式 $c\,\|(W\mathbf u)_i\|^2$ 见 [../../energy/attachment.md](../../energy/attachment.md)。

### `LoadsConfig` — `"loads"`

| 字段 | 默认 | 含义 |
|---|---|---|
| `gravity` | `(0,0,0)` | 重力加速度 $\mathbf g$（m/s²）；进 formulation 的 `body_force` |

### `ContactConfig` — `"contact"`（数组，每项一个接触能量）

每项必填 `model`（3 选 1）；其余字段按模型取用：

| 字段 | 默认 | 用于 | 含义 |
|---|---|---|---|
| `dhat` | `1e-3` | ipc | 屏障激活距离 $\hat d$（自接触） |
| `dhat_external` | `None` | ipc | 对障碍物的单独 $\hat d$（`None`=同 `dhat`） |
| `kappa` | `1000.0` | ipc | 屏障刚度 $\kappa$ |
| `obstacles` | `()` | **仅 ipc**（其他模型给了报错） | `{mesh: OBJ路径, velocity: [vx,vy,vz]?}` 列表；有速度→移动障碍 |
| `axis` / `side` / `height` | `"z"` / `"keep_above"` / `0.0` | floor | 半空间约束：沿 `axis` 保持在 `height` 之上/之下 |
| `stiffness` | `1.0` | floor、sampled | 罚刚度 |
| `samples` | `1` | sampled | 每三角形采样数 |
| `enable_self_contact` / `enable_external_contact` | `True` / `True` | sampled | 开关 |
| `friction_coeff` / `velocity_eps` | `0.0` / `1e-4` | sampled | 摩擦系数 $\mu$ / 速度正则 $\epsilon_v$；省略摩擦字段表示无摩擦 |

模式约束：只有 `sampled_penalty` 可带摩擦字段；非 sampled 模型出现 `friction_coeff` 或 `velocity_eps` 会报错。`sampled_penalty` 仅在 `friction_coeff > 0` 时启用摩擦，且**要求 dynamic**（摩擦需要速度）；省略摩擦字段时静态 sampled penalty 合法。各能量数学见 [../../contact/overview.md](../../contact/overview.md)。

### `InitialStateConfig` — `"initial_state"`

| 字段 | 默认 | 含义 |
|---|---|---|
| `displacement` | `(0,0,0)` | 均匀初始位移（每顶点同一平移；Hermite 下只写前 3 个平移 DOF） |
| `velocity` | `(0,0,0)` | 均匀初始速度（dynamic 用） |

### `SolverConfig` — `"solver"`

| 字段 | 默认 | 含义（透传 [`NewtonOptimizer`](../../solver/optimizer.md)） |
|---|---|---|
| `max_iterations` | `50` | Newton 迭代上限 |
| `gradient_tolerance` | `1e-6` | 梯度收敛阈值 |

### `DynamicConfig` — `"dynamic"`（static 模式下给出会打 warning 并忽略）

| 字段 | 默认 | 含义 |
|---|---|---|
| `timestep` | `None` | 步长 $\Delta t$；**dynamic 必填且 > 0** |
| `num_steps` | `1` | 步数（≥ 0） |
| `integrator` | `"implicit_euler"` | `implicit_euler` / `trbdf2`（见 [../../sim/overview.md](../../sim/overview.md)） |
| `damping` | `(0,0)` | Rayleigh `[质量阻尼, 刚度阻尼]`，必须恰好 2 元 |

### `OutputConfig` — `"output"`

| 字段 | 默认 | 含义（产物布局见 [_outputs.md](_outputs.md)） |
|---|---|---|
| `directory` | `None` | 输出目录；**必填**（JSON 或 `--output-dir`） |
| `write_surfaces` | `False` | 逐帧形变表面 OBJ |
| `write_states` | `False` | 逐帧位移 `.u` |
| `write_stress` | `False` | 逐帧逐元素 von Mises JSON |
| `write_abc` | `False` | Alembic 动画（**仅 dynamic**；static 下 warning 忽略；要求 Alembic build） |
| `dump_interval` | `1` | 每隔几帧 dump 一次（≥ 1，static 不用） |

---

## func `load_config()`

```python
load_config(*, mesh_type: str, mode: str, json_path=None, overrides: dict | None = None) -> SimConfig
```

唯一入口。流程：读 JSON（可缺省）→ 解析 JSON 内路径 → 套 CLI 点路径覆盖 → 校验 `type` 一致 → 逐 Section 构造 + 交叉校验（上文各表内规则）→ 返回冻结 `SimConfig`。任何问题抛 `ConfigError`（带字段定位的消息）。

## 用法示例

```python
from pypgo.tools.sim._config import load_config

cfg = load_config(
    mesh_type="tet", mode="dynamic",
    json_path="examples/sim_configs/tet_dynamic_bunny_ipc.json",
    overrides={"dynamic.num_steps": 10, "output.directory": "/tmp/run"},
)
cfg.dynamic.timestep, cfg.contact[0].model    # 0.01, 'ipc'（示例值）
```

## 交叉链接

- CLI flag → 点路径的映射：[_cli.md](_cli.md)
- 配置如何变成能量/约束对象：[_scene.md](_scene.md)
- 示例配置全集：[examples/sim_configs/README.md](../../../../../examples/sim_configs/README.md)
