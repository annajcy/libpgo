# 形状表示与时间积分（libpgo 学习笔记）

本笔记系统整理形状表示、连续介质动力学离散化、主流时间积分方案的数学推导，
并与 libpgo 中 `ImplicitBackwardEulerTimeIntegrator` / `TRBDF2TimeIntegrator` /
`TimeIntegrator` / `NewtonSolver` 的源码严格对应。

目录：

1. [形状表示](#1-形状表示)
2. [状态变量与牛顿第二定律](#2-状态变量与牛顿第二定律)
3. [质量矩阵](#3-质量矩阵)
4. [时间积分方案对比](#4-时间积分方案对比)
5. [隐式 Backward Euler：推导与优化形式](#5-隐式-backward-euler推导与优化形式)
6. [BDF2 与 TR-BDF2](#6-bdf2-与-tr-bdf2)
7. [阻尼模型（D 与 μI 的区别）](#7-阻尼模型d-与-μi-的区别)
8. [Newton 求解器：线搜索 / 可行步长 / 阻尼](#8-newton-求解器线搜索--可行步长--阻尼)
9. [libpgo 架构分层](#9-libpgo-架构分层)
10. [三种 Implicit Force Model 类型](#10-三种-implicit-force-model-类型)
11. [数据结构：稀疏矩阵与映射](#11-数据结构稀疏矩阵与映射)
12. [装配路径：兔子示例](#12-装配路径兔子示例)

---

## 1. 形状表示

固体/软体仿真里常见的形状表示大致分成四类：

| 表示 | 代表 | 优点 | 缺点 |
|------|------|------|------|
| 隐式方程 | 球面 $x^2+y^2+z^2=r^2$、SDF | 拓扑查询、距离查询简单 | 难以精确表示尖锐边角、难以直接做 FEM |
| 显式网格 | 三角形 / 四面体网格 | 直接对接 FEM、可视化成熟 | 拓扑固定、碰撞/自接触复杂 |
| 粒子 | SPH、MPM 中的 material point | 易于处理大变形、断裂 | 数值误差大，需要重建显式表面 |
| 神经隐式 | NeRF、SDF-net | 可微、端到端学习 | 评估昂贵、约束处理弱 |

libpgo 走的是典型的显式网格 + 基于有限元的能量路线：模型是四面体网格，
运动方程在顶点 DOF 上写成半离散 ODE，空间离散来自 FEM，时间上交给隐式积分器。

## 2. 状态变量与牛顿第二定律

设物体有 $N$ 个顶点，自由度数 $n_3 = 3N$。关键状态是三个向量：

$$
x^n \in \mathbb{R}^{n_3}, \quad v^n \in \mathbb{R}^{n_3}, \quad a^n \in \mathbb{R}^{n_3}
$$

分别记当前时刻的位置、速度、加速度。牛顿第二定律的连续形式：

$$
\dot x = v, \qquad M\dot v = f_{\text{ext}} - f_{\text{int}}(x) - D v
$$

其中：

- $M$ 为 $n_3 \times n_3$ 的质量矩阵；
- $f_{\text{int}}(x) = \nabla E_{\text{int}}(x)$ 为弹性内力（注意符号：$f = -\nabla E$ 意味着内力等于势能梯度的负值，下面所有公式里 $E_{\text{int}}$ 的梯度都是作为恢复力加到右端的）；
- $D$ 为粘性阻尼矩阵（Rayleigh 阻尼）。

libpgo 中状态存放在 `TimeIntegrator` 基类 (`timeIntegrator.h:155-157`)：

```cpp
EigenSupport::VXd q, qvel, qacc;        // x^n, v^n, a^n
EigenSupport::VXd q1, qvel1, qacc1;     // x^{n+1}, v^{n+1}, a^{n+1}
```

一步推进后 `proceedTimestep()` 会把 `q1 → q`、`qvel1 → qvel`、`qacc1 → qacc`。

## 3. 质量矩阵

### 3.1 Lumped vs Consistent

两种质量矩阵对应两种不同的物理近似，区别不只是"对角 vs 稀疏"。

**Consistent mass**
从动能的严格变分推导得到。动能泛函

$$
T = \tfrac12 \int_\Omega \rho\, \dot{x}^\top \dot{x} \, dV
$$

用形函数 $x = Nq$ 代入，得到

$$
T = \tfrac12 \dot q^\top M \dot q, \qquad M = \int_\Omega \rho\, N^\top N \, dV
$$

于是 $M$ 是 Galerkin 投影下的"真实"离散质量算子，对称正定，
特征谱与原连续算子一致。对线性四面体，它按顶点-顶点耦合，$12\times12$
分块里每一个 $3\times3$ 块都非零（见下方公式）。

**Lumped mass**
牺牲精度换取数值便利。两种常见做法：

- **Row-sum lumping**：把 consistent 矩阵每一行求和放到对角，即
  $M_{ii}^{\text{lump}} = \sum_j M_{ij}^{\text{cons}}$。物理直觉是
  把"质量"全部集中到所在顶点。
- **HRZ (diagonal scaling)**：取 consistent 矩阵的对角，再按总质量重标定；
  适合高阶单元 row-sum 会出现负值的场合。

对角矩阵的好处是显式求逆只需一次除法 —— 这对显式时间积分（FE、
Symplectic Euler）至关重要，因为每步都要 $v_{n+1} = v_n + h M^{-1} f$。
代价是引入数值频率畸变：lumped 版本的最大频率比 consistent 低，
高频模式被"抹平"。对隐式积分器（BE、TR-BDF2），反正每步都要解
$(A + \nabla^2 E)\delta u = r$ 这样的大型线性系统，对角化带来的收益
微乎其微，因此 libpgo **始终使用 consistent mass**（`MasK` 与 `K`
同拓扑）。

**线性四面体的 consistent mass（12×12 分块形式）**

$$
M_e = \frac{\rho V}{20}
\begin{bmatrix} 2I_3 & I_3 & I_3 & I_3 \\ I_3 & 2I_3 & I_3 & I_3 \\ I_3 & I_3 & 2I_3 & I_3 \\ I_3 & I_3 & I_3 & 2I_3 \end{bmatrix}
$$

其中 $V$ 为四面体体积，$\rho$ 为密度，$I_3$ 是 $3\times 3$ 单位阵。
对角块为 $\tfrac{\rho V}{10} I_3$、非对角块为 $\tfrac{\rho V}{20} I_3$，
四顶点总质量 $\mathrm{tr}(M_e) = 3 \cdot 4 \cdot \tfrac{2\rho V}{20} = \tfrac{6\rho V}{5}$？
—— 这里做个理智检查：总质量其实是 $\sum_{i,j} M_{ij}^{(kk)} = \rho V$
（任取一个笛卡尔分量 $k$，$\sum_{i,j} M_{ij}$ 应等于单元总质量 $\rho V$），
直接展开 $\tfrac{\rho V}{20}(4\cdot 2 + 12\cdot 1) = \tfrac{\rho V}{20}\cdot 20 = \rho V$，
对应。

**何时选 lumped**
(a) 显式积分；
(b) 接触冲击问题里想避免"质量耦合导致的虚假远场响应"；
(c) 粒子法 MPM/PIC，节点本质上就是独立质点。

### 3.2 FEM 推导脉络

libpgo 的 `MasK` 不是手写出来的，而是走标准 FEM 流水线。完整链条如下。

**Step 1：动能变分写成矩阵形式**

连续动能

$$
T(\dot x) = \tfrac12 \int_\Omega \rho \, \dot x^\top \dot x \, dV.
$$

在单元 $\Omega_e$ 里用形函数插值位置/速度。对四节点线性四面体，
$N(\xi) = [N_1, N_2, N_3, N_4]$ 是体心坐标（barycentric），满足
$\sum_i N_i = 1$、$N_i \ge 0$ 在单元内。把向量形式写开：

$$
x(\xi) = \begin{bmatrix} N_1 I_3 & N_2 I_3 & N_3 I_3 & N_4 I_3 \end{bmatrix}
\begin{bmatrix} x_1 \\ x_2 \\ x_3 \\ x_4 \end{bmatrix}
= \mathbf{N}(\xi)\, q_e,
\qquad \mathbf{N} \in \mathbb{R}^{3\times 12}
$$

于是

$$
T_e = \tfrac12 \dot q_e^\top \left[ \int_{\Omega_e} \rho\, \mathbf N^\top \mathbf N \, dV \right] \dot q_e
    = \tfrac12 \dot q_e^\top M_e \dot q_e.
$$

$M_e$ 的 $(i,j)$ 分块（$3\times 3$）为 $\rho \int_{\Omega_e} N_i N_j\, dV \cdot I_3$。

**Step 2：参数域积分 + 雅可比**

把体积积分从物理坐标 $(X_1, X_2, X_3)$ 换到参数坐标 $(\xi_1, \xi_2, \xi_3)$：

$$
\int_{\Omega_e} (\cdot)\, dV = \int_{\hat\Omega} (\cdot)\, |\det J|\, d\xi, \qquad
J = \frac{\partial X}{\partial \xi}.
$$

对线性四面体，映射
$X(\xi) = X_1 + (X_2-X_1)\xi_1 + (X_3-X_1)\xi_2 + (X_4-X_1)\xi_3$
是仿射的，$J = [X_2-X_1, X_3-X_1, X_4-X_1]$ 为常数 $3\times3$ 矩阵，

$$
|\det J| = 6V
$$

（因为参考单元 $\hat\Omega$ 是顶点 $\{0, e_1, e_2, e_3\}$ 的四面体，
体积 $\tfrac{1}{6}$；物理四面体体积 $V = \tfrac{1}{6}|\det J|$）。

**Step 3：重心坐标积分公式**

线性四面体上有闭式积分公式：

$$
\int_{\Omega_e} N_i^{a} N_j^{b} N_k^{c} N_l^{d}\, dV
= \frac{a!\, b!\, c!\, d!}{(a+b+c+d+3)!}\, 6V.
$$

特别地：

- $\int N_i\, dV = \tfrac{V}{4}$，
- $\int N_i N_j\, dV = \tfrac{V}{20}$ （$i \ne j$），
- $\int N_i^2\, dV = \tfrac{V}{10}$。

把它们组装成 $4\times4$ 矩阵 $\tilde M = \rho \int N^\top N\, dV$：

$$
\tilde M = \frac{\rho V}{20}
\begin{bmatrix} 2 & 1 & 1 & 1 \\ 1 & 2 & 1 & 1 \\ 1 & 1 & 2 & 1 \\ 1 & 1 & 1 & 2 \end{bmatrix}
$$

再 Kronecker 上 $I_3$（每顶点 3 个 DOF 独立），即得前一节那个
$12\times12$ 的 $M_e$。对高阶单元（二次四面体、六面体）通常用
Gauss 积分数值求积。

**Step 4：全局装配**

定义顶点 DOF 排布 $q \in \mathbb{R}^{3N}$，单元 DOF $q_e = P_e\, q$，
其中 $P_e \in \{0,1\}^{12\times 3N}$ 是布尔选取矩阵。则

$$
T = \sum_e T_e = \tfrac12 \dot q^\top \underbrace{\left(\sum_e P_e^\top M_e P_e\right)}_{M}\, \dot q.
$$

这个累加的稀疏结构是：**$(i,j)$ 块非零 $\iff$ 顶点 $i,j$ 共享至少一个单元**。
libpgo 对这一步做了两件事优化：

1. 在初始化时遍历一次所有单元，确定稀疏模式（`K` 的 `outer/inner` 指针）；
   `MasK` 和 `K` 共用同一个模式，于是 $M$ 与 Hessian $K$ 的槽位一一对齐，
   后续做 $\tfrac{1}{h^2}M + \tfrac{1}{h}D + K$ 时直接做 `valuePtr` 级向量加法。
2. 为每个单元预建 `SpMatI`——"单元 $(i,j)$ 块的槽位号 $\to$ 全局 `valuePtr` 下标"。
   组装时走 `ES::addSmallToBig(scale, M_e, MasK, 1.0, mapping, stride)`，
   全程零哈希、零符号重排，只做算术操作。

**Step 5：从 FEM 到运动方程**

对动能 $T(\dot q) = \tfrac12 \dot q^\top M \dot q$、势能 $E_{\text{int}}(q)$ 应用
Lagrange 方程

$$
\frac{d}{dt}\frac{\partial L}{\partial \dot q} - \frac{\partial L}{\partial q} = Q_{\text{ext}},
\qquad L = T - E_{\text{int}},
$$

得到半离散 ODE

$$
M\ddot q + \nabla E_{\text{int}}(q) = f_{\text{ext}}.
$$

加上瑞利阻尼 $D\dot q$ 后就是第 2 节的那条方程，也是 BE/TR-BDF2 离散的起点。

**为什么需要 $P_e$ 和映射 `SpMatI`？**
纯数学推导里 $P_e$ 是布尔矩阵，$P_e^\top (\cdot) P_e$ 实际就是
"把单元级矩阵塞到全局矩阵的对应槽位"。直接按数学定义做乘法会创建
$3N\times12$ 的稀疏矩阵、然后再做两次稀疏-稀疏乘法，代价极大。
libpgo 的 `SpMatI` 是把这三次矩阵乘法退化成一次向量 scatter：
只保留 "local nnz $k$ $\to$ global nnz $\ell$" 的索引数组，
运行时 `big.valuePtr()[ℓ] += scale * small.valuePtr()[k]`。

## 4. 时间积分方案对比

将连续 ODE $\dot y = f(y)$ 离散到时步 $h$。常见方案：

| 方案 | 更新公式 | 精度 | 稳定性 |
|------|---------|------|--------|
| 显式 Euler (FE) | $y_{n+1} = y_n + h f(y_n)$ | 1 阶 | 条件稳定 |
| Symplectic Euler | $v_{n+1} = v_n + h a(x_n)$，$x_{n+1}=x_n + h v_{n+1}$ | 1 阶 | 对 Hamilton 系统辛、能量保守 |
| Backward Euler (BE) | $y_{n+1} = y_n + h f(y_{n+1})$ | 1 阶 | A-稳定、耗散 |
| BDF2 | $\frac{3y_{n+1}-4y_n+y_{n-1}}{2h} = f(y_{n+1})$ | 2 阶 | A-稳定，不自启动 |
| TR | $y_{n+1} = y_n + \tfrac{h}{2}(f(y_n)+f(y_{n+1}))$ | 2 阶 | A-稳定但不强 L-稳定 |
| TR-BDF2 | 先 TR 到中间点 $y_{\gamma}$，再 BDF2 到 $y_{n+1}$ | 2 阶 | A-稳定 + L-稳定 |

工程上对大刚度、大时步软体仿真，多用 BE（最稳）或 TR-BDF2（阶数 + L-稳定兼得）。

## 5. 隐式 Backward Euler：推导与优化形式

### 5.1 从牛顿方程到代数方程

把位置、速度都用隐式时间差分展开：

$$
v^{n+1} = \frac{x^{n+1}-x^n}{h}, \qquad
a^{n+1} = \frac{v^{n+1}-v^n}{h} = \frac{x^{n+1}-x^n-h v^n}{h^2}
$$

代入 $M a^{n+1} = f_{\text{ext}} - f_{\text{int}}(x^{n+1}) - D v^{n+1}$，用 $u := x^{n+1}$：

$$
\frac{M}{h^2}(u - x^n - h v^n) + \frac{D}{h}(u - x^n) + f_{\text{int}}(u) = f_{\text{ext}}
$$

整理：

$$
\underbrace{\left(\frac{1}{h^2}M + \frac{1}{h}D\right)}_{A} u + f_{\text{int}}(u)
= \underbrace{f_{\text{ext}} + \frac{1}{h}M v^n + A x^n}_{b}
$$

### 5.2 转成非线性优化问题

由于 $f_{\text{int}} = \nabla E_{\text{int}}$，上式是以下变分问题的一阶条件：

$$
\min_u \; \Phi(u) = \tfrac12 u^\top A u + E_{\text{int}}(u) - b^\top u
$$

注意：**优化变量是 $u$（全位置），不是位移 $\Delta u$。** 这与纯弹性静力学文献中以位移为变量的写法不同，libpgo 刻意统一为位置变量，所以公式中会看到 $b^\top u$ 而不是 $b^\top \Delta u$。

### 5.3 代码对应：`ImplicitBackwardEulerTimeIntegrator`

`src/core/simulation/implicitBackwardEulerTimeIntegrator.cpp`：

- **`updateA()` (line 168-185)**
  ```cpp
  // A += (1/h)^2 M
  ES::addSmallToBig(1.0/(h*h), MasK, A, 1.0, Kmapping, 1);
  // A += (1/h) D
  (Mp<VXd>(A.valuePtr(), A.nonZeros())) +=
      Mp<const VXd>(D.valuePtr(), D.nonZeros()) * (1.0/h);
  ```
  精确对应 $A = \tfrac{1}{h^2}M + \tfrac{1}{h}D$。

- **`updateb()` (line 187-202)**
  ```cpp
  ES::mv(MasK, qvel, b); b *= 1.0/h;   // (1/h) M v^n
  b += f_ext;                          // + f_ext
  ES::mv(A, q, b, 1.0, 1.0);           // + A x^n
  ```
  精确对应 $b = f_{\text{ext}} + \tfrac{1}{h}M v^n + A x^n$。

- **`doTimestep()` (line 46-109)**：按 `InitialGuessMode` 选择 $u^{(0)} = x^n$ 或 $u^{(0)} = x^n + h v^n$，然后调 `solver->solve` 求 $u$。收敛后回填：
  ```cpp
  q1  = z;                    // u
  qvel1 = (z - q) / h;        // v^{n+1}
  qacc1 = (qvel1 - qvel) / h; // a^{n+1}
  ```

- **`updateD()` (line 127-166)**：把 Rayleigh 阻尼组装进 $D$，形式为
  $D = d_M \cdot M + d_K \cdot K$（每个子模型各自按 `massDampingParamsAll[i]` /
  `dampingParamsAll[i]` 叠加）。注意 `dampingParamsAll[i] > 0` 时会先调一次
  `hessian(q, curK)` 以 $K(x^n)$ 作为刚度项。

### 5.4 代码对应：`ImplicitBackwardEulerEnergy`

`src/core/simulation/implicitBackwardEulerTimeIntegratorHelper.cpp`：

- **`func()` (line 18-36)**
  ```cpp
  double energy = ES::vTMv(intg->A, x, intg->temp0, 0) * 0.5; // 0.5 u^T A u
  for (...) energy += implicitModelsAll[i]->func(x);          // + E_int(u)
  energy -= x.dot(intg->b);                                   // - b^T u
  ```

- **`gradient()` (line 38-55)**
  ```cpp
  ES::mv(intg->A, x, grad, 0);                          // Au
  for (...) { implicitModelsAll[i]->gradient(x, fint); grad += fint; } // + ∇E_int
  grad -= intg->b;                                      // - b
  ```

- **`hessian()` (line 57-76)**：遍历所有子模型，把单元 $K_i = \nabla^2 E_i$ 按 `mapping` 累加到全局 `hess`，最后加上 $A$。于是 $\nabla^2 \Phi = A + \sum_i \nabla^2 E_i$。

> **符号陷阱**：BE 助手里是 `- b^T u`，但 TR-BDF2 助手里是 `+ b^T u`。这不是笔误，
> 而是 `updateb1()`/`updateb2()` 在组装 `b1`/`b2` 时已经带上了整体负号
> （见 TR-BDF2 小节），于是 Lagrangian 里的符号相应翻转。

## 6. BDF2 与 TR-BDF2

### 6.1 BDF2 的不自启动性

BDF2 公式 $\tfrac{3y_{n+1}-4y_n+y_{n-1}}{2h}=f(y_{n+1})$ 需要两步历史 $y_n, y_{n-1}$。
首步没有 $y_{-1}$，必须用别的 1 阶方法（例如 BE）先推一步。这就是它"不自启动"
的含义。

### 6.2 TR-BDF2 的两阶段结构

设 $\gamma \in (0, 1)$ 为中间步比例。令 $\alpha := 2/(\gamma h)$。

**阶段 1（TR，推进到 $t_n + \gamma h$）** 用梯形法：

$$
v^{\gamma} = \frac{2}{\gamma h}(u - x^n) - v^n, \qquad a^{\gamma} = \frac{2}{\gamma h}(v^{\gamma} - v^n) - a^n
$$

代入运动方程并以 $u$ 为未知量，得到

$$
\underbrace{(\alpha^2 M + \alpha D)}_{A_1}\, u + f_{\text{int}}(u) = \underbrace{-(2\alpha M v^n + M a^n + D v^n + f_{\text{ext}}) - A_1 x^n}_{-b_1 - A_1 x^n \; \Rightarrow \; \text{已包含 } -A_1 x^n}
$$

解出 $u \Rightarrow x^{\gamma}$，同时更新 $v^{\gamma}, a^{\gamma}$。

**阶段 2（BDF2，推进到 $t_{n+1}$）** 在 $\{x^n, x^{\gamma}, x^{n+1}\}$ 上写二阶后向差分，
得到 $v^{n+1}, a^{n+1}$ 关于 $u := x^{n+1}$ 的线性表达式，代入得

$$
A_2 \, u + f_{\text{int}}(u) = \text{...}
$$

其中 $A_2 = \beta_4 M + \beta_7 D$，系数由 $\gamma, h$ 解析算出（见 `updateCoeffs`）。

### 6.3 代码对应：`TRBDF2TimeIntegrator`

`src/core/simulation/TRBDF2TimeIntegrator.cpp`：

- **`updateCoeffs()` (line 69-85)**：计算 $\alpha, \beta_0 \dots \beta_7$。
  ```cpp
  alpha   = 2.0 / (y * timestep);
  beta[0] = (2.0 - y) / (y * (1-y)*(1-y) * h*h);
  beta[1] = -beta[0];
  beta[2] = (1.0 - y) / (y * h);
  beta[3] = -1.0 / (y * (1-y) * h);
  beta[4] = (2-y)*(2-y) / ((1-y)*(1-y) * h*h);
  beta[5] = 1.0 / (y * (1-y) * h);
  beta[6] = -beta[5];
  beta[7] = (2-y) / ((1-y) * h);
  ```

- **`updateA1()` (line 126-141)**：$A_1 = \alpha^2 M + \alpha D$，矩阵级拷贝。
- **`updateb1()` (line 160-185)**：
  ```cpp
  ES::mv(MasK, qvel, b1); b1 *= 2.0 * alpha;   // 2α M v^n
  ES::mv(MasK, qacc, b1, 1.0, 1.0);            // + M a^n
  ES::mv(D,    qvel, b1, 1.0, 1.0);            // + D v^n
  b1 += f_ext;                                 // + f_ext
  b1 *= -1;                                    // 负号
  ES::mv(A1, q, b1, -1.0, 1.0);                // - A1 x^n
  ```
  注意最后的全局翻号 + `- A1 q`，这解释了 TR-BDF2 助手里 `energy += x.dot(b)`
  为什么是 `+`：把负号吸收到了 `b1/b2` 里。

- **`doTimestep()` (line 240-351)**：先解阶段 1 得到 $z_1$，回填
  $(q_y, v_y, a_y)$；若 $\gamma<1$ 再解阶段 2 得到 $z_2$，回填
  $(q_1, v_1, a_1)$；若 $\gamma=1$ 退化为 TR，直接用阶段 1 结果。

- **`updateA2()` (line 143-158)**：$A_2 = \beta_4 M + \beta_7 D$。

- **`updateb2()` (line 187-211)**：按 $M, D$ 两部分组装，末尾同样 `- A2 q`。

### 6.4 代码对应：`TRBDF2TimeIntegratorEnergy`

`src/core/simulation/TRBDF2TimeIntegratorHelper.cpp`：

- **`func()` (line 19-41)**：`0.5 * u^T A u + Σ E_i(u) + b^T u`
- **`gradient()` (line 43-61)**：`A u + Σ ∇E_i(u) + b`
- **`hessian()` (line 63-81)**：`A + Σ ∇^2 E_i(u)`

两个阶段共享同一个 `TRBDF2TimeIntegratorEnergy` 类模板：阶段 1 绑 `(A1, b1)`，
阶段 2 绑 `(A2, b2)`（见 `TRBDF2TimeIntegrator` 构造函数 line 42-43）。

## 7. 阻尼模型（D 与 μI 的区别）

两个看起来都像"加到对角"的量，其实作用层次完全不同：

| 符号 | 物理/数值 | 形式 | 作用 |
|------|----------|------|------|
| $D$ | 物理阻尼（Rayleigh） | $D = d_M M + d_K K$ | 改写动力学：$M\dot v + D v + \nabla E = f_{\text{ext}}$ |
| $\mu I$ | 数值正则化 | 仅在 Newton 迭代中 $H \leftarrow H + \mu I$ | 让病态或非 SPD 的 Hessian 数值上可解（Levenberg–Marquardt / trust-region 思路） |

libpgo 的默认 `SolverParam.addDamping = 0`（`NewtonSolver.h:45`），也就是默认
不加 $\mu I$。打开时规则（`NewtonSolver.cpp:221-225`）：

```cpp
if (solverParam.addDamping) {
  for (int i = 0; i < A11.rows(); i++)
    A11.coeffRef(i, i) += lambdaScale * lambda0;
}
```

其中 `lambda0 = ||grad||_∞`，`lambdaScale` 在梯度不降时保持、下降时乘 0.9、过小时置零
（line 187-208）。这是把 LM 的信赖域参数自适应实现成了对角正则。

## 8. Newton 求解器：线搜索 / 可行步长 / 阻尼

`src/core/nonlinearOptimization/NewtonSolver.cpp` 的 `solve()` 展开如下（概念等价）：

```text
for iter in 0..numIter:
    g   = ∇Φ(x)
    if iter > 0 and ||g||_∞ < eps: break
    H   = ∇²Φ(x)
    H   ← remove fixed rows/cols           # 见 A11, A11Mapping
    if addDamping: H += λ·I                 # 见 line 221-225
    solve  H δx = -g                        # Pardiso / MKL / Eigen
    s_max = energy->computeMaxStepSize(x, δx)  # 可行性约束（如无穿模）
    δx   *= s_max
    α = line_search(x, δx)                  # LSM_SIMPLE: halving until E decreases
    x   += α · δx
```

几个关键细节：

1. **可行步长**：在进入能量线搜索之前，`computeMaxStepSize` 保证 $x + s_{\max}\delta x$
   仍然可行（比如 IPC 的碰撞距离约束）。这不是 Wolfe 条件，只是防止数值跳出物理可行域。
2. **线搜索默认是 `LSM_SIMPLE`**（`NewtonSolver.cpp:351-367`）：从 $\alpha=1$ 出发做折半，
   直到 $E(x+\alpha\delta x) < E(x)$，最多 100 次。这比 Wolfe 回溯弱，但便宜、对
   非光滑能量（接触、自相交）更鲁棒。
3. **矩阵类型 `REAL_SYM_INDEFINITE`**（`NewtonSolver.cpp:90, 94`）：Pardiso 不假设
   Hessian 正定。因为带接触/摩擦的弹性问题里 $\nabla^2 E$ 常常是不定的，
   Pardiso 会走 $LDL^\top$ 并处理负特征值，无需用户手动 SPD 投影。
4. **SPD 投影**：libpgo 没有实现"逐元素做特征值截断"那种严格 SPD 投影；所谓的
   "SPD 投影"是上面 `+ λI` 的对角正则，而且默认关闭。

## 9. libpgo 架构分层

```text
TimeIntegrator (base)
   ├── ImplicitBackwardEulerTimeIntegrator
   │     └── ImplicitBackwardEulerEnergy   (扮演 PotentialEnergy 的壳)
   └── TRBDF2TimeIntegrator
         └── TRBDF2TimeIntegratorEnergy    (两份：绑 A1/b1 与 A2/b2)

                 │ (is-a PotentialEnergy)
                 ▼
         TimeIntegratorSolver
            └── 选一种后端：NewtonSolver / IpoptOptimizer / KnitroOptimizer
```

- `TimeIntegrator` 基类（`timeIntegrator.h`）持有：
  - 共享稀疏结构 `MasK, K, K1, Kmapping`；
  - 汇总的 Hessian 模式 `hessianAll`；
  - 三类 implicit 模型容器（`implicitModelsAll*` 是最终聚合）；
  - 状态 `q/qvel/qacc → q1/qvel1/qacc1`；
  - 约束 `constraints`、范围 `uRangeLow/Hi` 等。
- 具体积分器负责 **组装 $A, b$**（与该积分方案的时间离散式严格匹配），并实现
  `doTimestep()` 的主流程；它通过组合一个"能量壳"把 $(A, b, E_{\text{int}})$
  包装成 `PotentialEnergy` 交给求解器。
- 求解器端只认 `PotentialEnergy` 接口 (`func / gradient / hessian / ...`)，
  不知道时间积分的存在 —— 这是解耦的关键。

## 10. 三种 Implicit Force Model 类型

`TimeIntegrator::ImplicitModelType`（`timeIntegrator.h:144-149`）：

| 枚举 | 来源 | 必须实现的接口 | 加入方式 |
|------|------|--------------|---------|
| `IMT_ELASTIC` | 主弹性能量（构造时传入） | `PotentialEnergy` | 构造函数参数 `elasticPotential` |
| `IMT_SAME_TOPOLOGY` | 附加网格对齐能量 | `PotentialEnergyAligningMeshConnectivity` | `addImplicitForceModel()` |
| `IMT_GENERAL` | 任意能量（形状/拓扑可变） | `PotentialEnergy` | `addGeneralImplicitForceModel()` |

**为什么分三种？** 装配效率：

- `IMT_ELASTIC` 和 `IMT_SAME_TOPOLOGY` 的 Hessian 拓扑与主网格一致或子集，
  可以预建 `SpMatI` 映射，每步只做数值 `addSmallToBig`，零哈希。
- `IMT_GENERAL` 覆盖 CIPC 式接触能量：接触图每步都在变，
  `isHessianTopologyFixed() == false`，这时走 `hessianDirect` 路径，
  每次重新 `makeCompressed` + symbolic factorization。

在 BE/TRBDF2 助手里，遍历 `implicitModelsAll` 时不区分类型，
但对 `isHessianTopologyFixed()` 返回 false 的模型会改走 `hessianDirect`
（见 `hessianDirect` 实现 `implicitBackwardEulerTimeIntegratorHelper.cpp:124-157`）。

## 11. 数据结构：稀疏矩阵与映射

### 11.1 Eigen 稀疏矩阵回顾

`EigenSupport::SpMatD` = `Eigen::SparseMatrix<double, Eigen::RowMajor>`，
CSR 存储：

- `outerIndexPtr()` 长度 $n+1$：每行起点；
- `innerIndexPtr()` 长度 `nnz`：列索引；
- `valuePtr()` 长度 `nnz`：数值。

"**模式**"= outer + inner，"**数值**"= `valuePtr`。libpgo 保持一个事实：
只要模式不变，就只 memset `valuePtr` 再填值，不重建索引。

### 11.2 `SpMatI`：槽位地址簿

`SpMatI = SparseMatrix<int>`，在本项目里它**不承载矩阵数值**，而是"小矩阵到大矩阵的槽位地址簿"：

- `mapping.valuePtr()[k] = ℓ` 的含义是：**小矩阵第 $k$ 个 NNZ 应该加到大矩阵第 $\ell$ 个 NNZ**。
- 预建一次后，热路径就是线性遍历 `valuePtr` 做 `big.valuePtr()[ℓ] += scale * small.valuePtr()[k]`，O(nnz_small)。

这就是 `ES::addSmallToBig(scale, small, big, alpha, mapping, stride)` 的语义。
在 `updateA`, `updateD`, `hessian` 里所有的矩阵叠加都走这条路径。

### 11.3 具体数据成员

`TimeIntegrator` 基类 (`timeIntegrator.h:107-142`)：

```cpp
SpMatD  MasK;       // 全局质量矩阵（按 K 的 pattern 扩充）
SpMatD  K, K1;      // 主弹性模型的 Hessian 模板
SpMatI  Kmapping;   // K -> hessianAll 的槽位映射
SpMatD  hessianAll; // 所有固定拓扑模型累加后的最大 pattern（能量壳创建 Hessian 用它）

// 每个 implicit model 一份：
vector<SpMatD*>  implicitModelsAll_K;       // 该模型自己的 local Hessian
vector<SpMatD*>  implicitModelsAll_K1;      // 备份 / 阻尼计算用
vector<SpMatD*>  implicitModelsAll_M;       // 与该模型相关的质量模板
vector<SpMatI*>  implicitModelsAll_Kmaping; // local K -> hessianAll 的槽位
vector<VXd*>     implicitModelsAll_fint;    // 该模型贡献的内力缓存
```

### 11.4 约束装配同构

约束雅可比也走同样的"local → global by mapping"。例如
`ConstraintFunctionsAssembler::init()` 中：

```cpp
entriesj.emplace_back((int)i, dofs[it.col()], 1.0);
```

这行不是"把 1.0 写进矩阵"，而是"登记 (行 $i$, 全局列 dofs[j]) 位置有一个 NNZ"，
`1.0` 是模式占位符；真正的值在求解热路径里由
`constraints1D[i]->jacobian(x, buf->constraints1D_J[i])` +
`ES::addSmallToBig(..., jacobian1DMappings[i])` 填入。

## 12. 装配路径：兔子示例

假设一只四面体网格兔子，主材料 StVK，再加一条肌肉（同拓扑的附加能量），
再加 CIPC 自接触：

1. 构造 `ImplicitBackwardEulerTimeIntegrator(mass, stvk_energy, dM, dK, h, niter, eps)`。
   - 基类保存 `elasticEnergy = stvk_energy`，建 `K = stvk_energy->createHessian()`、`MasK` 与 `K` 同模式、`Kmapping = K→hessianAll`（初始 `hessianAll = K`）。
2. `addImplicitForceModel(muscle)`：追加到 `additionalForceModels`，模型类型 `IMT_SAME_TOPOLOGY`。
3. `addGeneralImplicitForceModel(cipc_contact)`：追加到 `generalAdditionalForceModels`，类型 `IMT_GENERAL`，`isHessianTopologyFixed() == false`。
4. 首步 `doTimestep()` 内部的 `assembleImplicitModels()` 把三类合并到 `implicitModelsAll`，
   若 `generalForceModelChanged` 则重新扩展 `hessianAll` 的模式并重建各 `*_Kmaping`。
5. 每次迭代里：
   - `updateD()` 遍历 `implicitModelsAll`，只对 `isHessianTopologyFixed()` 的（StVK、muscle）叠加 $D$，CIPC 跳过。
   - `updateA()` 组装 $\tfrac{1}{h^2}M + \tfrac{1}{h}D$。
   - `updateb()` 组装 $f_{\text{ext}} + \tfrac{1}{h}Mv + Aq$。
   - Newton 求 $u$：每步的 `hessian()` 把 StVK、muscle 通过 mapping 累加、CIPC 走 `hessianDirect`。
6. Newton 收敛后回填 $q_1, v_1, a_1$，`proceedTimestep()` 推进到下一帧。

这是 libpgo 把"多种不同性质的势能统一为 `PotentialEnergy`，统一送进同一个
隐式积分 + Newton 求解"的完整闭环。

---

## 附：关键文件速查

| 主题 | 文件 | 行号 |
|------|------|------|
| BE 状态变量、主循环 | `src/core/simulation/implicitBackwardEulerTimeIntegrator.cpp` | `doTimestep` 46–109 |
| BE $A$ 组装 | 同上 | `updateA` 168–185 |
| BE $b$ 组装 | 同上 | `updateb` 187–202 |
| BE 能量/梯度/Hessian | `src/core/simulation/implicitBackwardEulerTimeIntegratorHelper.cpp` | 18–76 |
| TR-BDF2 系数 | `src/core/simulation/TRBDF2TimeIntegrator.cpp` | `updateCoeffs` 69–85 |
| TR-BDF2 阶段 1 | 同上 | `updateA1/b1` 126–185 |
| TR-BDF2 阶段 2 | 同上 | `updateA2/b2` 143–211 |
| TR-BDF2 主循环 | 同上 | `doTimestep` 240–351 |
| TR-BDF2 能量壳 | `src/core/simulation/TRBDF2TimeIntegratorHelper.cpp` | 19–81 |
| Newton 主循环 | `src/core/nonlinearOptimization/NewtonSolver.cpp` | `solve` 107–469 |
| 线搜索 LSM_SIMPLE | 同上 | 351–367 |
| 对角阻尼（LM 风格） | 同上 | 221–225 |
| 基类数据成员 | `src/core/simulation/timeIntegrator.h` | 107–173 |
