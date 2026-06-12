# `pypgo/sim/stepper.py` — 动力学时间积分器

> 源文件：`pypgo/sim/stepper.py`（43 行）。模块架构见 [overview.md](overview.md)。
>
> Python 层只是**积分器选择标签**：`PyDynamicStepper` peer 仅携带 kind（BackwardEuler / TRBDF2）与 $\gamma$（`src/python/pypgo/simulation/dynamic/core.h:18-48`），真正的 stepper 在 `DynamicSimulation` 构造时由 `makeDynamicStepper(kind, problem, gamma)` 实例化（`src/core/simulation/dynamicStepService.cpp:14-30`）。本篇给出两个积分器的 as-implemented 阶段方程（已与 C++ 源核对，关键恒等式经数值验证）。

## 共同数学框架

每个阶段都是一次"二次惯性项 + 非线性势能"的最小化（见 [overview.md](overview.md)）：

$$\min_{\mathbf u}\ \tfrac12\,\mathbf u^\top A_s\,\mathbf u+\mathbf l_s^\top\mathbf u+E(\mathbf u)
\quad\Longleftrightarrow\quad
A_s\,\mathbf u+\mathbf l_s+\nabla E(\mathbf u)=\mathbf 0$$

积分器的全部内容就是 $(A_s,\mathbf l_s)$ 的构造与解出后的 $(\mathbf u,\mathbf v,\mathbf a)$ 更新。记 $h$ 为步长、$M$ 质量阵、$D$ 为步首组装的 Rayleigh 阻尼（[simulation.md](simulation.md)）、上标 $n$ 为步首状态。

两个 stepper 的 `step` 流程一致（`backwardEulerStepper.cpp:30-53`、`trbdf2Stepper.cpp:42-103`）：派发 `begin_step` → 组装 $D$ → 逐阶段 `prepareStageResidual` + Newton → 接受判定（任一阶段失败即整步拒绝、状态不前进）→ 状态更新。

---

## class `DynamicStepper`

基类门面：校验并持有 `_core.PyDynamicStepper` peer。不直接实例化——用下面两个具体类，或传给 `DynamicSimulation(integrator=...)`（也接受字符串 `"implicit_euler"` / `"trbdf2"`，见 [simulation.md](simulation.md)）。

---

## class `BackwardEulerDynamicStepper`

```python
BackwardEulerDynamicStepper()
```

隐式（向后）Euler。一阶精度、L-稳定、数值阻尼强——接触/大步长场景的默认选择。

**离散化**：$\mathbf v^{n+1}=\dfrac{\mathbf u^{n+1}-\mathbf u^n}{h}$，$\mathbf a^{n+1}=\dfrac{\mathbf v^{n+1}-\mathbf v^n}{h}$，代入 $M\mathbf a^{n+1}+D\mathbf v^{n+1}+\nabla E(\mathbf u^{n+1})=\mathbf f_{\text{ext}}$。

**as-implemented**（`src/core/simulation/backwardEuler/backwardEulerStageBuilder.cpp:14-26`）：

$$A=\frac{1}{h^2}M+\frac{1}{h}D,\qquad
\mathbf l=-\Big(\mathbf f_{\text{ext}}+\frac{1}{h}M\,\mathbf v^n+A\,\mathbf u^n\Big),\qquad
\mathbf u_{\text{init}}=\mathbf u^n$$

驻点方程展开即

$$\frac{1}{h^2}M\big(\mathbf u-\underbrace{(\mathbf u^n+h\mathbf v^n)}_{\tilde{\mathbf x}\ \text{惯性预测}}\big)
+\frac{1}{h}D\big(\mathbf u-\mathbf u^n\big)+\nabla E(\mathbf u)=\mathbf f_{\text{ext}}$$

——incremental potential $\frac{1}{2h^2}\|\mathbf u-\tilde{\mathbf x}\|_M^2+\frac{1}{2h}\|\mathbf u-\mathbf u^n\|_D^2+E(\mathbf u)-\mathbf f_{\text{ext}}^\top\mathbf u$ 的最小化。

**状态更新**（同文件 29-41 行）：

$$\mathbf u^{n+1}=\mathbf u^\ast,\qquad
\mathbf v^{n+1}=\frac{\mathbf u^{n+1}-\mathbf u^n}{h},\qquad
\mathbf a^{n+1}=\frac{\mathbf v^{n+1}-\mathbf v^n}{h},\qquad
t^{n+1}=t^n+h$$

注意 $\mathbf a$ 仅作记录（BE 的阶段方程不读它）；TRBDF2 则真正消费它。

---

## class `TRBDF2DynamicStepper`

```python
TRBDF2DynamicStepper(gamma=0.5)
```

TR-BDF2 复合单步法：把 $[t^n,t^n+h]$ 拆成**梯形法则**（TR）子步 $[t^n,t^n+\gamma h]$ 与 **BDF2** 子步 $[t^n+\gamma h,t^n+h]$，二阶精度、比 TR 多 L-稳定性（经典取 $\gamma=2-\sqrt2$ 时两阶段 Jacobian 相同；本实现默认 $\gamma=0.5$）。每步两次 Newton 求解。$\gamma\in(0,1]$（Python/C++ 双重校验）；$\gamma\ge1-10^{-9}$ 时退化为**单阶段**纯 TR（`trbdf2Stepper.cpp:23, 72-84`）。

记 $\alpha=\dfrac{2}{\gamma h}$，系数（**as-implemented**，`src/core/simulation/trbdf2/trbdf2StageBuilder.cpp:11-35`）：

$$\beta_0=\frac{2-\gamma}{\gamma(1-\gamma)^2h^2},\quad \beta_1=-\beta_0,\quad
\beta_2=\frac{1-\gamma}{\gamma h},\quad \beta_3=-\frac{1}{\gamma(1-\gamma)h},$$

$$\beta_4=\frac{(2-\gamma)^2}{(1-\gamma)^2h^2},\quad
\beta_5=\frac{1}{\gamma(1-\gamma)h},\quad \beta_6=-\beta_5,\quad
\beta_7=\frac{2-\gamma}{(1-\gamma)h}$$

### 阶段 1 — TR over $\gamma h$（`computeStage1`，37-59 行）

$$A_1=\alpha^2M+\alpha D,\qquad
\mathbf l_1=-\big(2\alpha M\mathbf v^n+M\mathbf a^n+D\mathbf v^n+\mathbf f_{\text{ext}}\big)-A_1\mathbf u^n,\qquad
\mathbf u_{\text{init}}=\mathbf u^n$$

解出 $\mathbf u_\gamma$ 后中间状态（`updateAfterStage1`，61-74 行）：

$$\mathbf v_\gamma=\alpha(\mathbf u_\gamma-\mathbf u^n)-\mathbf v^n,\qquad
\mathbf a_\gamma=\alpha^2(\mathbf u_\gamma-\mathbf u^n)-2\alpha\mathbf v^n-\mathbf a^n$$

**恒等式核对**（已数值验证）：$\mathbf v_\gamma$ 的定义等价于梯形法则 $\dfrac{\mathbf u_\gamma-\mathbf u^n}{\gamma h}=\dfrac{\mathbf v^n+\mathbf v_\gamma}{2}$；驻点方程 $A_1\mathbf u_\gamma+\mathbf l_1+\nabla E=\mathbf 0$ 展开正是

$$M\mathbf a_\gamma+D\mathbf v_\gamma+\nabla E(\mathbf u_\gamma)=\mathbf f_{\text{ext}}$$

### 阶段 2 — BDF2 over 节点 $(t^n,\ t^n+\gamma h,\ t^n+h)$（`computeStage2`，76-102 行）

$$A_2=\beta_4M+\beta_7D,\qquad
\mathbf l_2=M\big(\beta_0\mathbf u^n+\beta_1\mathbf u_\gamma+\beta_2\mathbf v^n+\beta_3\mathbf v_\gamma\big)
+D\big(\beta_5\mathbf u^n+\beta_6\mathbf u_\gamma\big)-\mathbf f_{\text{ext}}-A_2\mathbf u^n$$

初值取 $\mathbf u_\gamma$。解出 $\mathbf u^{n+1}$ 后（`updateAfterStage2`，104-121 行）：

$$\mathbf v^{n+1}=\beta_5\mathbf u^n+\beta_6\mathbf u_\gamma+\beta_7(\mathbf u^{n+1}-\mathbf u^n),\qquad
\mathbf a^{n+1}=\beta_0\mathbf u^n+\beta_1\mathbf u_\gamma+\beta_2\mathbf v^n+\beta_3\mathbf v_\gamma+\beta_4(\mathbf u^{n+1}-\mathbf u^n)$$

**恒等式核对**（已数值验证）：$\mathbf v^{n+1}$ 恰是过 $(t^n,\mathbf u^n)$、$(t^n+\gamma h,\mathbf u_\gamma)$、$(t^n+h,\mathbf u^{n+1})$ 的二次插值在 $t^n+h$ 处的导数——非均匀节点 BDF2；$\mathbf a^{n+1}$ 同构地是 $(\mathbf v^n,\mathbf v_\gamma,\mathbf v^{n+1})$ 的 BDF2 导数。$\gamma=\tfrac12$ 时退化为熟悉的均匀形式 $\mathbf v^{n+1}=\dfrac{\mathbf u^n-4\mathbf u_\gamma+3\mathbf u^{n+1}}{h}$。驻点方程展开为

$$M\mathbf a^{n+1}+D\mathbf v^{n+1}+\nabla E(\mathbf u^{n+1})=\mathbf f_{\text{ext}}$$

**失败语义**：阶段 1 不被接受 → 直接整步拒绝（不跑阶段 2，`stage_results` 只有 1 条）；阶段 2 失败同样拒绝并回到步首状态（`trbdf2Stepper.cpp:63-68, 94-98`）。

### 属性 `gamma`

只读，直读 C++ peer（`PyTRBDF2DynamicStepper::gamma`）。

---

## 用法示例

```python
import pypgo

be = pypgo.sim.BackwardEulerDynamicStepper()
tr = pypgo.sim.TRBDF2DynamicStepper(gamma=2 - 2**0.5)   # 经典 L-稳定取值
tr.gamma                                                 # 0.5857...

sim = pypgo.sim.DynamicSimulation(mass=M, state=state0, timestep=1e-3,
                                  energy=total, integrator=tr)
```

## 交叉链接

- 消费方与每步流程：[simulation.md](simulation.md)、[overview.md](overview.md)
- 阶段二次型如何与 $E$ 组合、固定 DOF 处理：`src/core/simulation/common/stageResidual.cpp`、`dynamicStepperUtils.cpp`（见 [overview.md](overview.md) 阶段 3）
- 状态/结果类型：[state.md](state.md)
