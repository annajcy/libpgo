# `pypgo/fem/torch.py` — PyTorch 可微平衡层

> 源文件：`pypgo/fem/torch.py`（273 行，纯 Python；需要 PyTorch，CPU + float64）。模块架构见 [overview.md](overview.md)。
>
> 把"材料参数 → 静力平衡态"这个映射包装成 `torch.autograd.Function`：前向解一次平衡，反向用**隐函数定理 + 伴随法**回传参数梯度。用于材料反演 / 形状-材料联合优化等可微仿真任务。

## 共同数学框架

设参数 $\theta$（弹性场 $\mathbf b$ 或塑性场 $\mathbf a$），平衡位移由一阶条件隐式定义（在自由 DOF 上）：

$$\mathbf u^*(\theta)=\arg\min_{\mathbf u} E(\mathbf u,\theta)
\quad\Longleftrightarrow\quad
\nabla_{\mathbf u}E\big(\mathbf u^*(\theta),\theta\big)=\mathbf 0$$

观测量是表面顶点位置 $\mathbf y=\mathbf X_s+\mathbf u^*_s$。对一阶条件求全导数（隐函数定理）：

$$\frac{d\mathbf u^*}{d\theta}
=-\Big(\underbrace{\nabla^2_{\mathbf u}E}_{H}\Big)^{-1}
\underbrace{\frac{\partial^2E}{\partial\mathbf u\,\partial\theta}}_{J}$$

损失 $L(\mathbf y)$ 的参数梯度用**伴随法**避免显式求 $d\mathbf u^*/d\theta$（as-implemented，`torch.py:54-85` 的 `backward`）：

$$H_{\!f\!f}\,\boldsymbol\lambda_f=\Big(\frac{\partial L}{\partial\mathbf u}\Big)_f,
\qquad
\frac{\partial L}{\partial\theta}=-J^\top\boldsymbol\lambda$$

下标 $f$ 表示限制在自由 DOF（固定 DOF 的行列被剔除）。$J$ 即 [`DeformationEnergy.elastic_jacobian` / `plastic_jacobian`](energy.md)；带参数相关外载荷 $\mathbf f_g(\theta)$ 时内层梯度是 $\nabla_uE-\mathbf f_g$，故 $J$ 要减去 $\partial\mathbf f_g/\partial\theta$（`torch.py:260-266`）。

**实现注意**：backward 中 Hessian 被 `to_dense()` 后用 `np.linalg.solve` 求解（`torch.py:76-82`）——稠密路径，适合中小网格；参数 Jacobian 同样稠密化。

---

## class `_StaticEquilibriumFunction`（私有，`torch.autograd.Function`）

前向（`torch.py:17-51`）：校验张量（CPU、float64、1-D、长度 = 参数 DOF 数）→ 写入参数场 → 组目标（叠加外载荷）→ `OptimizationProblem` + `fix_variables` → 内层 Newton 求解（**热启动**：上次解作为初值）→ 返回表面顶点 `(m, 3)` 张量。上下文保存参数与位移供 backward。

反向（`torch.py:53-85`）：把 `grad_surface (m,3)` 散射回全 DOF 梯度（Hermite 时只填每顶点前 3 个分量——观测的是位置，不是导数 DOF）→ 伴随求解 → $-J^\top\lambda$。

用户不直接用此类；经下面两个 `nn.Module` 的 `forward` 调用。

---

## class `_BaseStaticEquilibriumLayer`（`torch.nn.Module` 基类）

两个公开层的共享实现。

### 构造

```python
_BaseStaticEquilibriumLayer(*,
    energy,                  # DeformationEnergy（参数场的宿主）
    fixed_dofs, fixed_values,# 硬固定的 DOF 与取值（唯一、范围内、等长）
    surface_vertices,        # (m, 3) 观测表面点的静止位置
    surface_vertex_ids,      # (m,) 顶点 id（须 < energy.num_vertices）
    inner_optimizer=None,    # pypgo.solver.Optimizer，None → NewtonOptimizer()
    objective_energy=None,   # 内层目标；None → energy 本身。
                             #   传 EnergySet 可叠加固定外力/软固定等项（num_dofs 须一致）
    external_load=None)      # 参数相关外载荷（force()/parameter_jacobian() 协议），
                             #   仅 ElasticStaticEquilibriumLayer 支持
```

构造时缓存：自由 DOF 掩码、每顶点 DOF 步长（`num_dofs // num_vertices`，Hermite=24）、参数形状（与 `energy.num_elastic/plastic_dofs` 一致性校验）、热启动向量。

### `forward(parameter_values) → Tensor (m, 3)`

`parameter_values`：1-D float64 CPU 张量，长度 = `num_parameter_dofs`。返回平衡后的表面顶点位置（可参与后续可微计算图）。

### `reset_warm_start(displacement=None)`

重置内层求解热启动（`None` → 零状态）。参数大幅跳变导致内层不收敛时使用。

### 属性 `last_equilibrium_displacement` / `last_surface_vertices` / `last_inner_result`

最近一次前向的完整位移、表面位置、[`SolverResult`](../solver/result.md)（检查内层是否收敛！）。前向未跑过时抛 `RuntimeError`。

### `_build_objective()`（内部）

`external_load` 非空时，每次前向把当前载荷叠加为
`EnergySet([(objective_energy, 1), (LinearEnergy(-load), 1)])`——载荷随参数变，必须每次重建（`torch.py:177-188`）。

---

## class `ElasticStaticEquilibriumLayer`

```python
ElasticStaticEquilibriumLayer(*, energy, fixed_dofs, fixed_values,
                              surface_vertices, surface_vertex_ids,
                              inner_optimizer=None, objective_energy=None,
                              external_load=None)
```

参数 = **弹性场** $\mathbf b$（`num_parameter_dofs == energy.num_elastic_dofs`）。体本构通道数为 0（[elastic.md](elastic.md)），所以实际用例是 **Koiter 壳**（5 通道：刚度与厚度）。支持 `external_load`（如 [`SelfWeightGravity`](mass.md)：厚度变 → 质量变 → 自重变，混合导数自动修正）。

### `_parameter_jacobian(u)`（内部）

$J=$ `energy.elastic_jacobian(u)`；有外载荷时 $J\mathrel{-}=\partial\mathbf f_g/\partial\mathbf b$（`torch.py:260-266`）。

---

## class `PlasticStaticEquilibriumLayer`

同上，参数 = **塑性场** $\mathbf a$（`num_parameter_dofs == energy.num_plastic_dofs`；配 [`VolumetricPlasticity(dofs=3/6)`](plastic.md)）。$J=$ `energy.plastic_jacobian(u)`；不支持 `external_load`。

---

## 用法示例

```python
import torch
import pypgo

layer = pypgo.fem.PlasticStaticEquilibriumLayer(
    energy=deform,                       # dofs=6 的塑性场
    fixed_dofs=fixed, fixed_values=np.zeros(len(fixed)),
    surface_vertices=surf_rest,          # (m, 3)
    surface_vertex_ids=surf_ids,
    inner_optimizer=pypgo.solver.NewtonOptimizer(max_iterations=80),
)

a = torch.tensor(a0, dtype=torch.float64, requires_grad=True)
opt = torch.optim.Adam([a], lr=1e-2)
for it in range(100):
    y = layer(a)                          # 前向：解平衡
    loss = ((y - y_target) ** 2).sum()
    opt.zero_grad(); loss.backward()      # 反向：伴随法
    opt.step()
    assert layer.last_inner_result.converged   # 内层必须收敛，否则梯度无意义
```

## 数学自检

- 伴随梯度与有限差分一致：$\frac{L(\theta+h e_i)-L(\theta-h e_i)}{2h}\approx(\partial L/\partial\theta)_i$（内层须收紧 `gradient_tolerance`，且 `enforce_spd=False` 以保证 Hessian 精确，见 [energy.md](energy.md)）
- 固定 DOF 不进入伴随系统：$\lambda$ 在固定 DOF 上为零（`torch.py:78-82` 只解自由块）✓

## 交叉链接

- 导数来源：[energy.md](energy.md)（`elastic_jacobian`/`plastic_jacobian`、`hessian`）
- 内层求解：[../solver/optimizer.md](../solver/optimizer.md) · [../solver/problem.md](../solver/problem.md)（`fix_variables`）
- 参数相关载荷：[mass.md](mass.md)（`SelfWeightGravity`）
