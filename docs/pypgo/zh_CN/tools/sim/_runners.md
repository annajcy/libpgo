# `pypgo/tools/sim/_runners.py` — static / dynamic 执行循环（私有）

> 源文件：`pypgo/tools/sim/_runners.py`（178 行）。模块架构见 [overview.md](overview.md)。
>
> 两个 runner 只依赖 [`SceneBundle`](_scene.md) 的契约，对 mesh_type **零分支**。求解器统一为 [`NewtonOptimizer(max_iterations, gradient_tolerance)`](../../solver/optimizer.md)（来自 `solver` 配置段）。

## func `run_static(bundle, cfg) -> dict`

解静力平衡：

$$\mathbf u^*=\arg\min_{\mathbf u}\ \underbrace{E_{\text{def}}+\sum E_{\text{att}}+\sum E_{\text{contact}}}_{\text{weighted\_energies}}\;\underbrace{-\,\mathbf f_g^\top\mathbf u}_{\text{LinearEnergy}(-\mathbf f_g)}$$

as-implemented 步骤：

1. `x0 = bundle.initial_vector(initial_state.displacement)`；
2. 目标 = `EnergySet(weighted_energies(include_gravity_potential=True))`（重力作为势能项进目标）；
3. 硬固定：`problem.fix_variables(fixed_dofs, x0[fixed_dofs], num_dofs)`——消元而非罚（[../../solver/problem.md](../../solver/problem.md)）；
4. **接触状态初始化**：static 没有 stepper，对每个 `stateful_contacts` 手动 `begin_step(time=0, timestep=1.0, previous_x=x0)`（1.0 是无意义的伪步长，statics 无时间积分）；
5. `NewtonOptimizer().solve(problem, x0)`。

summary 字段：`mode/mesh_type/num_dofs/converged/status/iterations/final_gradient_max_norm/max_abs_u`，以及 `max_fixed_abs_u`（被固定 DOF 的最大位移——应为 0，作消元正确性自检）。

static 输出（按 `output` flags，写入 `output.directory`）：

| flag | 文件 |
|---|---|
| `write_surfaces` | `final_surface.obj`（`surface_positions(x*)`） |
| `write_states` | `states/deform_final.u`（$(n,1)$ 列，[.u 格式](../../animation/abc.md)） |
| `write_stress` | `stress/von_mises_final.json`：`{frame:0, time:0.0, stress_type:"von_mises", location:"element", values:[...]}`，values = [`deformation.element_von_mises(u*)`](../../fem/energy.md) |
| `write_abc` | 忽略 + warning（static 无动画） |
| 总是 | `summary.json` |

## func `run_dynamic(bundle, cfg) -> dict`

逐步隐式时间积分，物理在 [`DynamicSimulation`](../../sim/simulation.md)（C++ stepper）：

1. 初态 `DynamicState(displacement=x0, velocity=v0, acceleration=0)`（均匀向量经 `initial_vector`）；
2. 能量 = `EnergySet(weighted_energies(include_gravity_potential=False))`——**重力不进势能**，改为每步 `sim.step(external_force=bundle.gravity_force)` 作外力；
3. `DynamicSimulation(mass, state, timestep, energy, integrator, damping, fixed_dofs)`（integrator/damping 来自 `dynamic` 段）；
4. 接触 `begin_step` 由 **C++ stepper 每步自动派发**（dispatchBeginStep，含移动障碍的时间推进）——与 static 不同，Python 侧不驱动；
5. 主循环 `num_steps` 次：
   - 移动 attachment：`energy.set_targets(tile(velocity * t_next))`（匀速拖拽目标，见 [_scene.md](_scene.md) `MovingAttachment`）；
   - `frame = sim.step(external_force, optimizer)`；
   - `frame.frame_index % dump_interval == 0` 时 dump（**被拒帧也写**——位移是最后被接受的状态，便于诊断发散）；
   - `frame.accepted == False` 时中断循环。

dynamic 输出（dump 帧共享一个 `dump_interval`）：

| flag | 文件 |
|---|---|
| `write_surfaces` | `surface/surface{frame:04d}.obj` |
| `write_states` | `states/deform{frame:04d}.u` |
| `write_stress` | `stress/von_mises{frame:04d}.json`（同 static schema，`frame`/`time` 取当前帧） |
| `write_abc` | 循环结束后一次性 `AbcWriter.dump` 写 `animation.abc`：rest=曲面静止位置、每个 dump 帧一条曲面位移样本，`fps=1/(dump_interval·Δt)`（[fps 当前实际不生效](../../animation/abc.md)）；要求 Alembic build，否则启动即 `ConfigError` |
| 总是 | `summary.json` |

summary 字段：`mode/mesh_type/num_dofs/num_frames/final_time/final_timestep_id` + `frames`（逐帧 `frame_index/accepted/status/iterations`）。

`write_states`+`write_stress` 的布局正好是 [`dump_stress_vdb`](../../animation/stress_vdb.md) / [`compute_stress_field_stats`](../../animation/stress_stats.md) 的自动探测约定。

## 交叉链接

- 上游装配：[_scene.md](_scene.md)；文件写出助手：[_outputs.md](_outputs.md)
- 时间积分（implicit_euler/trbdf2、帧接受/拒绝语义）：[../../sim/overview.md](../../sim/overview.md)
- 求解器：[../../solver/optimizer.md](../../solver/optimizer.md)
- 输出的下游消费：[../animation/overview.md](../animation/overview.md)
