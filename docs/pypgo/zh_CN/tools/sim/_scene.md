# `pypgo/tools/sim/_scene.py` — 场景装配（私有）

> 源文件：`pypgo/tools/sim/_scene.py`（372 行）。模块架构见 [overview.md](overview.md)。
>
> 把冻结的 [`SimConfig`](_config.md) 变成**与网格类型无关**的 `SceneBundle`——runner（[_runners.md](_runners.md)）拿到 bundle 后不再对 mesh_type 分支。本文件是 sim CLI 里唯一接触库对象（fem/energy/contact）的装配层。

## 装配总流程

```
SimConfig
  │ 体（tet/cubic）：read_veg → VolumeMesh ─┐    shell：read_obj ──┐
  ▼                                         ▼                      ▼
formulation 选择（TetLinear / CubicLinear / CubicTricubicHermite / KoiterShell）
  ▼
SimulationMesh.create_volumetric / create_shell
  ▼
fem.deformation_energy（elastic 按 material.model，ElementwiseField，塑性 dofs=0）
  ▼
质量与重力：mass = fm.mass_matrix(sim_mesh, mass_field)
            f_g  = fm.body_force(sim_mesh, g, mass_field)   （g≠0 时）
  ▼
约束：fixed → DOF 索引（硬固定，交给 runner 消元）
      attachments → energy.VertexAttachment（仿真顶点软固定，3 DOF/顶点限定）
      surface_attachments → energy.EmbeddedVertexAttachment（经曲面嵌入 W，formulation 无关）
  ▼
接触：ContactSurface.embedded(surface, W) / .identity（shell）
      → FloorEnergy / IPCEnergy / SampledPenaltyEnergy / FrictionalSampledPenaltyEnergy
  ▼
SceneBundle（能量列表 + 质量/重力 + 表面映射）
```

## func `resolve_vertex_selector()`

```python
resolve_vertex_selector(selector: VertexSelector, vertices: np.ndarray) -> np.ndarray
```

[选择器](_config.md)三分支落地：`file`（读空白分隔整数，非整 token 报错）/ `indices`（直取）/ `region`（坐标满足 $|c_i-c_{\text{bound}}|\le\text{tol}$ 的顶点）。统一 `np.unique` 去重排序；空集或越界抛 `ConfigError`。`vertices` 是相对哪个网格由调用处决定——`fixed`/`attachments` 对**仿真网格**顶点，`surface_attachments` 对**曲面网格**顶点。

## class `MovingAttachment`（dataclass）

带 `movement` 的 attachment 的运行期载体：`energy`（`VertexAttachment` 实例）、`velocity`（(3,) 位移/时间）、`num_vertices`。dynamic 循环每步把目标设为 `velocity * t_next`（见 [_runners.md](_runners.md)）——即匀速拖拽软约束。

## class `SceneBundle`（dataclass）

runner 需要的一切：

| 字段 | 含义 |
|---|---|
| `sim_mesh` / `formulation` / `deformation` | 仿真网格、formulation、[`DeformationEnergy`](../../fem/energy.md) |
| `attachment_energies` / `contact_energies` | 软约束与接触能量列表 |
| `stateful_contacts` | 需要 `begin_step(time, timestep, previous_x)` 的接触（IPC、sampled penalty 两族） |
| `moving_attachments` | `MovingAttachment` 列表 |
| `mass` | 质量矩阵（SparseMatrix） |
| `gravity_force` | $\mathbf f_g$（num_dofs 向量；$\mathbf g=0$ 时为零向量） |
| `fixed_dofs` | 硬固定 DOF 索引（`None`=无）；由顶点索引 × `dofs_per_vertex` 展开（Hermite 下一顶点 24 个 DOF 全 clamp） |
| `num_dofs` / `dofs_per_vertex` | 总 DOF 数 / 每顶点 DOF（tet/cubic-linear/shell=3，tricubic Hermite=24） |
| `surface_rest` / `surface_triangles` | 静止曲面顶点 (m,3) 与三角形 (k,3) |
| `surface_map` | 仿真 DOF → 曲面位移 DOF 的稀疏嵌入 $W$；`None`=恒等（shell） |

### `surface_positions(u)`

$$\mathbf x_{\text{surf}}=\bar{\mathbf x}_{\text{surf}}+\mathrm{reshape}(W\mathbf u)\qquad(W=\text{None 时直接 reshape }\mathbf u)$$

输出 (m,3)。所有表面输出（OBJ/abc）经它，与 formulation 无关。

### `initial_vector(values)`

把均匀 3 向量铺成 num_dofs 初值：3 DOF/顶点时 `np.tile`；Hermite（24 DOF）只写每顶点**前 3 个平移 DOF**，导数 DOF 置零。

### `weighted_energies(*, include_gravity_potential)`

组装 `[(E, 1.0), ...]`：deformation + attachments + contacts；`include_gravity_potential=True` 且 $\|\mathbf f_g\|>0$ 时追加 **`LinearEnergy(-f_g)`**——重力势能 $-\mathbf f_g^\top\mathbf u$（[../../energy/algebraic.md](../../energy/algebraic.md)）。static 走势能项，dynamic 不含（重力作外力进 stepper），见 [_runners.md](_runners.md)。

---

## 体场景装配（`_build_volume_scene`，as-implemented 要点）

1. **formulation 选择**（`_volume_formulation`）：先按元素宽度核对网格类型（tet=4 点、cubic=8 点，不符报错）；`auto` → tet 取 `TetLinear`、cubic 取 `CubicLinear`；显式 `cubic-tricubic-hermite` → `CubicTricubicHermite`（[../../fem/formulations.md](../../fem/formulations.md)）；
2. **弹性**：`material.model` 经 `_VOLUME_ELASTIC` 映射到 [fem 本构类](../../fem/elastic.md)；塑性固定 `VolumetricPlasticity(dofs=0)`（无塑性）；
3. **质量场**：`material.density` 给了用均匀 `VolumeDensity`，否则 `volume_density(volume)` 取 `.veg` 各区域密度（[../../fem/mass.md](../../fem/mass.md)）；
4. **曲面嵌入**：`W = fm.surface_embedding_matrix(volume, surface.vertices)`；接触面 `ContactSurface.embedded(surface.vertices, W)`（[../../contact/surface.md](../../contact/surface.md)）——接触作用在嵌入曲面上、经 $W^\top$ 拉回仿真 DOF；
5. **attachments 守卫**：`dofs_per_vertex != 3`（即 Hermite）时普通 `attachments` 直接报错——`VertexAttachment` 按 3 DOF/顶点假设组装；Hermite 场景请用 `surface_attachments`；
6. **surface_attachments**：`_surface_attachment_energy` —— 在曲面顶点集上解析选择器，构造 [`EmbeddedVertexAttachment(embedding=W, vertex_indices, coeff, num_dofs)`](../../energy/attachment.md)：$E=c\sum_i\|(W\mathbf u)_i\|^2$，把**物理点**（而非 DOF）钉在静止位置，对任何 formulation 语义一致。

## 壳场景装配（`_build_shell_scene` 差异）

- 网格即曲面：`SimulationMesh.create_shell(surface, KoiterStVKShellMaterial(...))` + `KoiterShell()` formulation + `KoiterStVK()` 本构、`ShellPlasticity(dofs=0)`；
- 质量场：`areal_density` → `ShellArealDensity`；否则 `ShellDensityThickness(density, thickness)`（[../../fem/mass.md](../../fem/mass.md)）；
- 接触 `ContactSurface.identity(rest)`、`surface_map=None`（仿真顶点就是曲面顶点）；
- `surface_attachments` 走 `embedding=None` 的 `EmbeddedVertexAttachment`（恒等嵌入）。

## func `build_scene(cfg)`

按 `cfg.mesh_type` 派发到上述两个装配函数，返回 `SceneBundle`。

## 交叉链接

- 配置定义：[_config.md](_config.md)；执行：[_runners.md](_runners.md)
- 能量与嵌入软固定：[../../energy/attachment.md](../../energy/attachment.md)
- formulation/质量/体力：[../../fem/formulations.md](../../fem/formulations.md)、[../../fem/mass.md](../../fem/mass.md)
- 接触能量族：[../../contact/overview.md](../../contact/overview.md)
