# C-IPC Walkthrough：理论 + libpgo 仓库实现

本文档是 libpgo 当前 C-IPC 实现（`src/core/contact/CIPC.{h,cpp}` 等）的**逐层解读**，
把 Li et al. 的 *Codim-IPC: Stable and Efficient Contact for Elastodynamics* (SIGGRAPH 2021)
以及其所依赖的 *Incremental Potential Contact* (SIGGRAPH 2020) 与仓库代码对齐。

目标读者：已经读过 IPC 基础，现在要把仓库里当前针对三角面片的 C-IPC 扩展到 **tet 体网格** 和
**cubic（六面体）体网格**，并且**首期不实现 friction**。

阅读顺序建议：

1. §1 先把术语和符号钉死；
2. §2–§5 是理论骨架，可直接作为后续代码的 "应该做什么" 的对照；
3. §6–§12 对应源码里每个模块 / 每个函数，带 `path:line` 引用；
4. §13 给出一次 Newton 迭代完整控制流；
5. §14 说明当前仓库 C-IPC 在哪里被用（以及哪里**还没有被用**）；
6. §15 列出把 C-IPC 扩展到 tet / cubic 的改造蓝图。

---

## 1. 术语与符号

- $x \in \mathbb{R}^{3n}$：所有接触顶点的位置（在本实现里是表面三角网的顶点）。
- $h$：时间步长。
- $\hat x^n = x^n + h v^n + h^2 M^{-1} f_{\text{ext}}$：显式 Euler 预测位置（惯性预测点）。
- $d_k(x)$：第 $k$ 个碰撞对的距离；$s_k = d_k^2$。
- $\hat d$：barrier 激活距离（代码中叫 `dhat`）。
- $\hat s = \hat d^2$：barrier 在平方距离上的截止。
- $\kappa$：barrier 刚度（代码中叫 `kappa`）。
- PT / EE：point-triangle / edge-edge 接触对。
- $A_k$：第 $k$ 对的几何权重（顶点 lumped 面积 × 三角形面积，或两条边的长度乘积）。

为方便与论文交叉对照，本文档在每个核心公式旁都给出论文相应小节号。

---

## 2. Incremental Potential（IP）的基本框架

每个时间步解一次（表面看起来是无约束的）最小化：

$$
E(x) = \underbrace{\tfrac{1}{2}\|x - \hat x^n\|_M^2}_{\text{inertia}} + \underbrace{h^2 \Psi(x)}_{\text{elasticity}} + \underbrace{B(x; \kappa, \hat d)}_{\text{contact barrier}} + \underbrace{D(x)}_{\text{friction (skipped)}}
$$

$x^{n+1} = \arg\min_x E(x)$。

- $\Psi$：超弹性势能（本仓库里由 tet 或 cubic FEM 装配得到，见 `deformationModelAssembler` / `solidDeformationModelEnergy`）；
- $B$：IPC 对数-barrier，只在 $d_k < \hat d$ 的对上有贡献；
- $D$：friction 项，此次扩展显式**不做**。

关键点：$B$ 是严格正的、在 $d_k \to 0^+$ 时发散到 $+\infty$ 的光滑项。只要数值上保证迭代过程中**始终满足** $d_k > 0$，接触就会被自动处理成 "投影回可行域" 的效果；这正是后面 §9 CCD + §10 filter line search 的作用。

---

## 3. 距离原语：7 个 PT 子情形 + 9 个 EE 子情形

C-IPC 跟 IPC 一样，只把接触建模成两类原子查询：

**Point-Triangle（7 种）**

| Type                              | 几何意义                      |
|-----------------------------------|-------------------------------|
| `PP_PT0 / PP_PT1 / PP_PT2`        | 点到三角形某个顶点最近（3 种）|
| `PE_PT0T1 / PE_PT1T2 / PE_PT2T0`  | 点到三角形某条边最近（3 种）  |
| `PT`                              | 点到三角形内部最近（1 种）    |

**Edge-Edge（9 种）**

| Type                                             | 几何意义                        |
|--------------------------------------------------|---------------------------------|
| `PP_Ea0Eb0 / PP_Ea0Eb1 / PP_Ea1Eb0 / PP_Ea1Eb1`  | 端点-端点（4 种）               |
| `PE_Ea0_Eb / PE_Ea1_Eb / PE_Eb0_Ea / PE_Eb1_Ea`  | 一条边端点到另一条边内部（4 种）|
| `EE`                                             | 两条边内部-内部（1 种）         |

这些类型在 `src/core/contact/CIPC.h:31-54` 的 `PTDistType` / `EEDistType` 两个 enum 里明确列出；
分类函数 `classifyPT` / `classifyEE` 会根据当前构型在这 7/9 种情形中挑出 "当前活跃" 的那一种，
然后调用对应子例程。

**为什么统一用 $d^2$ 而不是 $d$**：

- $d^2$ 在可行域上处处 $C^\infty$（而 $\sqrt{\cdot}$ 在 $d\to 0$ 会吃掉一阶导数）；
- 所有梯度 / Hessian 都是有理/多项式表达式，可以由符号微分一次生成，就是仓库里的
  `CIPC_autogen.h` / `CIPC_autogen_ll.h`；
- barrier 本身就是定义在平方距离 $s = d^2$ 上的。

在代码里，每个活跃子情形都带着一个标准的 12-DOF 局部排布：

- PT：$[p; t_0; t_1; t_2]$；
- EE：$[e_a^0; e_a^1; e_b^0; e_b^1]$。

后面所有 gradient / Hessian / CCD 缓冲区都按这 12 维来写（见 `V12d` / `M12d`）。

---

## 4. 对数-barrier 与面积权重

### 4.1 barrier 函数

原始 IPC 使用的 barrier（代码在 `CIPC.cpp` 的 `namespace barrier`，声明在 `CIPC.h:143-153`）：

$$
b(s, \hat s) = \begin{cases}
-\left(\dfrac{s}{\hat s} - 1\right)^2 \ln\!\left(\dfrac{s}{\hat s}\right) & 0 < s < \hat s, \\
0 & s \ge \hat s.
\end{cases}
$$

性质：

- 在 $s = \hat s$ 处 $C^2$（$b$、$b'$、$b''$ 都为 0）；
- 在 $s \to 0^+$ 发散到 $+\infty$；
- 只对平方距离 $s$ 求导，后面再通过链式法则把 $\nabla_x d^2$、$\nabla_x^2 d^2$ 乘进去。

代码里 `b(s, shat)` / `dbds(s, shat)` / `d2bds2(s, shat)` 三个函数各自返回 $b$、$\partial b/\partial s$、$\partial^2 b/\partial s^2$。$b$ 的第二导数在 `CIPC.cpp:876` 实现为

```cpp
-(2.0 * std::log(r) + 4.0 * rm1 * invr - rm1 * rm1 * invr * invr) / shat2
```

其中 `r = s / shat`、`rm1 = r - 1`、`invr = 1 / r`、`shat2 = shat * shat`。

### 4.2 厚度 / stackable offset（C-IPC 相对于 IPC 的主扩展）

C-IPC 把每个 codim 元素 $i$ 分成两部分厚度：

- **刚性 offset $\xi_i$**：不可压缩的厚度壳 / 绳芯；
- **弹性激活 $\hat d$**：barrier 开始起作用的距离。

对对偶 $k = (i, j)$ 定义 $\xi_k = (\xi_i + \xi_j)/2$，并把 barrier 改写为

$$
b^\xi(s, \hat s) = b\!\left( s - \xi_k^2,\; 2\xi_k \hat d + \hat d^2 \right).
$$

当 $\xi_k = 0$ 就退化到经典 IPC。对应代码路径**是**：`additiveCCD` 里的 `thickness`（`minDistance`）参数承担 $\xi_k$ 的角色（见 `CIPC.cpp:898-948`）；而当前 `CIPCPotentialEnergy::computeEnergy/Gradient/Hessian` 仍是 $\xi_k = 0$ 的纯 IPC 形式。**对 tet/cubic 扩展而言，是否启用 offset 是首期可选的**：如果只做体网格之间的非贯穿，$\xi_k = 0$ 就够了；只有在需要模拟薄壳/绳堆叠时才要开启。

### 4.3 面积 / 长度权重 $A_k$

为了做到 mesh 无关，每个 pair 的 barrier 项会额外乘一个几何权重：

$$
B(x) = \kappa \sum_{k \in \mathcal C(x)} A_k\, b\!\bigl(d_k^2(x),\, \hat d^2\bigr).
$$

仓库的 `buildAreaWeights`（`CIPC.cpp:1151-1180`）在 `setMesh` 时预计算：

- `triArea_[fi]` = `0.5 * |(v1-v0) × (v2-v0)|`；
- `vertexArea_[vi]` = 顶点所在三角形面积和除以 3（典型 lumped 面积）；
- `edgeLength_[ei]` = 静止构型下边长。

然后在 `findCollisionPairs`（见 §7）里填到 `PTPair.weight = vertexArea_[p] * triArea_[f]`
和 `EEPair.weight = edgeLength_[ea] * edgeLength_[eb]`。`weight` 进入每对的 $A_k$。

---

## 5. EE 的 mollifier：解决近平行退化

### 5.1 为什么需要

通用 EE 距离依赖一个 $2\times 2$ 线性系统的解，其系数矩阵在两条边平行时奇异；随着边靠近平行，
"活跃子情形" 会从 `EE` 翻转到 `PE` 再到 `PP`，而 $\nabla^2 d^2_{\text{EE}}$ 在这些边界上**只是 $C^0$ 的**，
Newton 会失去二阶光滑性。

### 5.2 Mollifier 形式

$$
m(c, \bar c) = \begin{cases}
1, & c \ge \bar c, \\
\left(2 - \dfrac{c}{\bar c}\right)\dfrac{c}{\bar c}, & 0 \le c < \bar c,
\end{cases}
$$

其中 $c(x) = \|e_a \times e_b\|^2$，$\bar c = \varepsilon_x = 10^{-3}\,\|\bar e_a\|^2 \|\bar e_b\|^2$。
代码里 `eps_ee`（`CIPC.h:200`）就是 $\bar c$，通常取 0 表示 "自动按静止边长平方乘积 × 1e-3 推算"。

EE 项变为：

$$
B_{EE}^k(x) = m(c(x), \bar c)\, b(d_k^2(x), \hat d^2).
$$

当 $c \to 0$（近平行）$m \to 0$，EE 项被连续抹掉；与此同时另一条 "PP / PE" 分支的 barrier 也正处于活跃范围，接手 barrier 职责。净效果是整个 $B$ 在 EE→PE→PP 边界上保持 $C^2$。

声明见 `CIPC.h:111-119`，实现在 `CIPC.cpp` 的 `namespace distance`。

### 5.3 Hessian 组合

对每一对 EE，其 12×12 Hessian 要走乘积法则：

$$
H_k^{EE} = b \cdot \nabla^2 m + \nabla m\, \nabla b^T + \nabla b\, \nabla m^T + m \cdot \bigl( b''\,\nabla s\, \nabla s^T + b' \nabla^2 s \bigr).
$$

PT 没有 mollifier，直接用第二个括号里的两项：

$$
H_k^{PT} = \kappa A_k \bigl( b''\,\nabla s\, \nabla s^T + b' \nabla^2 s \bigr).
$$

组装细节见 `computeHessian` / `computeAll`（`CIPC.cpp:1850-2162`）。

---

## 6. Friction（本次显式略过）

IPC / C-IPC 的 friction 是 Coulomb 摩擦，把非光滑的 $\mathrm{sign}(v_T)$ 替成一个
$C^1$ 光滑的 $f_1(\|v_T\|; \varepsilon_v)$，并用 "半隐式" 把 $D(x)$ 写成只关于 $x$ 的函数。
在 tet/cubic 首期扩展里**不实现**，所以：

- 新代码里**不要**提供 friction 系数；
- `CIPCPotentialEnergy` 的接口保持目前的形式，不引入 `frictionCoeff`；
- 需要时再从 `pointTrianglePairCouplingEnergyWithCollision` 里移植，那里已有 velocity 相关逻辑。

---

## 7. Additive CCD（ACCD）：实现在哪、做什么

传统 IPC 的 CCD 可以用多项式根求解；在薄壳 + offset 场景下 root-finding 的数值鲁棒性不够。
C-IPC 的 §5.4 提出 **additive CCD**：不解多项式，用距离函数的**保守推进**反复走小步，直到距离跌到 gap 以下或者 $t > t_{\max}$。

仓库实现完整坐落在 `CIPC.cpp:881-1069` 的 `namespace ccd`，核心是 `additiveCCD`（line 898-948）：

```cpp
static constexpr double ACCD_CONSERVATIVE_RESCALING = 0.1;  // η
double d_sq = distanceSquared(x);
double d    = std::sqrt(d_sq);
if (d <= minDistance) { toi = 0.0; return true; }

double d_func = d_sq - minDistance * minDistance;
const double gap = eta * d_func / (d + minDistance);

toi = 0.0;
for (long i = 0; i < ACCD_MAX_ITER; ++i) {
  double toi_lower = (1.0 - eta) * d_func / ((d + minDistance) * maxDispMag);
  x += toi_lower * dx;
  d_sq = distanceSquared(x);
  d    = std::sqrt(d_sq);
  d_func = d_sq - minDistance * minDistance;
  if (d_func <= 0.0) break;
  if (toi > 0.0 && d_func / (d + minDistance) < gap) break;  // 距离已经逼近 gap
  toi += toi_lower;
  if (toi > tmax) return false;
}
return true;
```

要点：

1. `minDistance` 就是 §4.2 里的 $\xi_k$（对纯 IPC 传 0）；
2. 每次推进量 $t_l = (1-\eta)(d^2-\xi^2)/((d+\xi)\,l_p)$，其中 $l_p$ 是这 12 维位移向量的最大范数（`maxDispMag`，见 `CIPC.cpp:983,1030`）；
3. 迭代前先 `subtractMean` 把 12-DOF 位移中的共同平移去掉（translation invariance，`CIPC.cpp:951-961`），这是 C-IPC 论文 Algorithm 1 中那一步 $\bar p = \tfrac14 \sum p_i$；
4. 返回的 `toi` 是保守的碰撞时间下界，**永远不等于真实 TOI**，但足以保证 $x + \text{toi}\cdot dx$ 仍然严格可行。

特殊处理（`edgeEdgeCCD` 内部）：当 EE 子距离因近平行陷入数值病态（$d^2 - \xi^2 \le 0$ 但并没有真的碰上），用四端点之间的最小 PP 距离退化一下，防止卡死。

对应的 PT / EE 高层接口：

- `ccd::pointTriangleCCD(p, t0, t1, t2, dp, dt0, dt1, dt2, thickness, tMax)` — `CIPC.cpp:963`
- `ccd::edgeEdgeCCD(ea0, ea1, eb0, eb1, dea0, dea1, deb0, deb1, thickness, tMax)` — `CIPC.cpp:1012`

---

## 8. PSD 投影：保证 Newton 方向恒为下降方向

原始 $\nabla^2 E$ 里，$b'\nabla^2 d^2$ 的 $\nabla^2 d^2$ 可以是不定的。
在每对 12×12 块**本地**做 PSD 投影，然后再 scatter 到全局稀疏矩阵，是保持 Newton 正定的标准做法。

实现在 `M12d projectToPSD(const M12d &H)`（`CIPC.cpp:1074`）：

```cpp
Eigen::SelfAdjointEigenSolver<M12d> es(H);
const auto &evals = es.eigenvalues();
if (evals(0) >= 0.0) return H;       // 早停：已是 PSD
Eigen::DiagonalMatrix<double, 12> D(evals);
for (int i = 0; i < 12; ++i)
  if (D.diagonal()(i) < 0.0) D.diagonal()(i) = 0.0;
// reconstruct U * D * U^T
```

要点：

- 只需做 12×12 自伴特征分解，代价很小；
- `evals(0)` 是排序后的最小特征值，用来做早退出；
- 组装时 elastic 元素块（tet 是 12×12，cubic 是 24×24）也要各自做 PSD 投影，但那发生在 `deformationModel*` 侧，不在本模块。

---

## 9. Kappa（barrier 刚度）的自适应调节

IPC/C-IPC 里 $\kappa$ 理论上是自适应的：

- 初值按 "让 barrier 的力与惯性/弹性力在同量级" 挑；
- 如果最小距离 $d_{\min}$ 连续两次 Newton 迭代落在目标阈值 $d_{\min}^{\text{tgt}} \ll \hat d$ 之下，就 $\kappa \leftarrow 2\kappa$；
- 上限 $\kappa_{\max}$ 用来防止 Newton Hessian 被打爆。

**仓库当前实现**：`CIPCPotentialEnergy::kappa` 是**静态配置**（`CIPC.h:199`），没有在内部根据 $d_{\min}$ 动态抬升。`runShellSim.cpp:218` 直接传 `E * h` 作为 kappa。

对 tet/cubic 首期这不是必须做的事；静态 kappa 在示例场景里够用，但一旦要做复杂堆叠就需要补 §9 的自适应逻辑。建议把它作为 `CIPCPotentialEnergy` 的 future work 而不是在 tet/cubic 扩展里同时做。

---

## 10. Filter line search（barrier-aware 回溯）

Newton 给出搜索方向 $p = -H^{-1} g$。为了让 $x + \alpha p$ **整条轨迹** $\tau \in [0, \alpha]$ 都保持可行（$d_k > \xi_k$），必须：

1. 先用 CCD 算出最大允许步 $\alpha_{\max}$；
2. 再在 $[0, \alpha_{\max}]$ 内做 Armijo 回溯，让 $E(x + \alpha p) \le E(x) + c_1\, \alpha\, g^T p$。

步 1 由 `CIPCPotentialEnergy::computeMaxStepSize`（`CIPC.cpp:1439`）完成：它跑一次 broad-phase 收集候选，然后对每个候选对调用 `ccd::pointTriangleCCD` / `ccd::edgeEdgeCCD`，把 `toi` 最小值（乘以 `slackness`）作为允许步。

- `slackness` 默认为 1（`CIPC.h:208`），但为安全会保留下限 `1e-12`。

步 2 在仓库里由更上层的 Newton solver 完成（`src/core/nonlinearOptimization/`）。关键接口契约：
`computeMaxStepSize` 必须在 `hessianDirect` 之后、`func` 之前被调用；`func` 每次重新计算 barrier 时都**自己先调 `findCollisionPairs`**（见 `CIPC.cpp:2168`），所以 Newton 层不用维护碰撞对缓存。

---

## 11. 模块地图（文件级）

```
src/core/contact/
├── CIPC.h / CIPC.cpp             # 主入口：距离/barrier/ccd/PSD/Pair 管理/能量/梯度/Hessian
├── CIPC_autogen.h                # 距离一阶/二阶导符号生成（Point-Line / Point-Plane 等）
├── CIPC_autogen_ll.h             # 同上 low-level 版本（MATLAB symbolic 导出）
├── CCDKernel.h / CCDKernel.cpp   # 早期 CCD 后端（Vega 安全 CCD、三方 CCD 适配）——与 CIPC ACCD 并存
├── contactEnergyUtilities.{h,cpp}# 共享小工具：PosFunction 类型等
├── pointPenetrationEnergy.*      # 旧 penalty：外部物体接触
├── pointTrianglePairCouplingEnergyWithCollision.*  # 旧 penalty：自接触配对
├── triangleMeshSelfContactDetection.*  # 自接触 BVH 宽相（与 CIPC 空间哈希并存）
├── triangleMeshSelfContactHandler.*    # 自接触高层包装（penalty 路线）
└── triangleMeshExternalContactHandler.* # 外部接触高层包装（penalty 路线）
```

**重要区分**：

- **新** C-IPC 路线：`CIPCPotentialEnergy` 直接消费表面三角网顶点；自带 barrier 能量 + ACCD + PSD 投影。
- **旧** penalty 路线：`pointPenetrationEnergy` / `pointTrianglePairCouplingEnergyWithCollision` 加 `triangleMesh*Handler`，基于二次惩罚，不是 log-barrier，也没有 ACCD。

当前仓库里两条路线**并存**，在不同的 driver 里被选用（见 §14）。

---

## 12. CIPC.cpp 模块内按文件顺序导览

以下按 `CIPC.cpp` 中的代码顺序列出关键入口，标 `path:line`；未列出的行号是实现细节。

| 起始行 | 符号 | 作用 |
|--------|------|------|
| `CIPC.cpp:~45` | `distance::classifyPT` | 把 $(p, t_0, t_1, t_2)$ 分到 7 种子情形之一 |
| `CIPC.cpp:~150` | `distance::classifyEE` | 把 4 个端点分到 9 种 EE 子情形之一 |
| `CIPC.cpp:~400-~820` | `distance::{ppSqDist, peSqDist, ptSqDist, eeSqDist, eeMollifier}` 及其 grad/hess | 每种子距离的 $d^2$、$\nabla d^2$、$\nabla^2 d^2$；mollifier 同样三联 |
| `CIPC.cpp:~820` | `distance::computePTSqDist*` / `computeEESqDist*` | 分类派发统一接口，输出按 12-DOF 排布 |
| `CIPC.cpp:870-879` | `barrier::b / dbds / d2bds2` | 对数-barrier 关于 $s$ 的 0/1/2 阶导 |
| `CIPC.cpp:898` | `ccd::additiveCCD` | ACCD 核心循环 |
| `CIPC.cpp:951` | `ccd::subtractMean` | 位移去平均（translation invariance） |
| `CIPC.cpp:963` | `ccd::pointTriangleCCD` | PT CCD wrapper |
| `CIPC.cpp:1012` | `ccd::edgeEdgeCCD` | EE CCD wrapper（含近平行退化处理） |
| `CIPC.cpp:1074` | `projectToPSD` | 12×12 自伴特征分解 + 特征值截断 |
| `CIPC.cpp:1100` | `CIPCPotentialEnergy::setMesh` | 接收 `MXd V, MXi F`，存拓扑和 rest pos |
| `CIPC.cpp:1119` | `buildEdges` | 从三角表面提取去重边集合 |
| `CIPC.cpp:1136` | `buildAdjacency` | 顶点-三角、边-顶点相邻对（用于宽相时自剔除） |
| `CIPC.cpp:1151` | `buildAreaWeights` | `triArea_` / `vertexArea_` / `edgeLength_` 预计算 |
| `CIPC.cpp:1189-1283` | `AABB` + `SpatialHash` | 匿名 namespace 里的空间哈希结构（Teschner 素数） |
| `CIPC.cpp:1291` | `findCollisionPairs` | 宽相：插三角/查顶点 → PT 对；插边/查边 → EE 对 |
| `CIPC.cpp:1439` | `computeMaxStepSize` | ACCD 全局最小步 |
| `CIPC.cpp:1685` | `computeEnergy` | $\sum \kappa A_k b$ + 可选地板 penalty |
| `CIPC.cpp:1752` | `computeGradient` | 每对 12 维局部梯度 scatter |
| `CIPC.cpp:1850` | `computeHessian` | 每对 12×12 本地 Hessian → PSD 投影 → scatter |
| `CIPC.cpp:1981` | `computeAll` | 一次宽相 + 能量/梯度/Hessian 一次算完 |
| `CIPC.cpp:2165` | `func` | Potential 接口包装 `computeEnergy` |
| `CIPC.cpp:2172` | `gradient` | Potential 接口包装 `computeGradient` |
| `CIPC.cpp:2179-2186` | `hessian` / `createHessian` | 故意抛异常（稀疏拓扑随宽相变化，不走固定拓扑接口） |
| `CIPC.cpp:2189` | `hessianDirect` | Potential 的可变拓扑 Hessian 接口 |

几个关键结构：

### 12.1 空间哈希（broad phase）

`findCollisionPairs`（`CIPC.cpp:1291-1434`）的策略：

1. TBB 并行构造每个顶点 / 三角形 / 边的 AABB，并整体膨胀 `dhat`；
2. 以 `triBox` 平均对角线做 `cellSize`，建两张哈希表：
   - `triHash` 插入所有三角 AABB，随后**并行**按每个顶点 AABB 查询；得到 PT 候选；
   - `edgeHash` 插入所有边 AABB，随后并行按每条边 AABB 查询；得到 EE 候选；
3. 候选再做 narrow phase：AABB 重叠 + `computePTSqDist` / `computeEESqDist` 小于 `dhat2`；
4. 每对保存 `[a, b, c, d]` 四个 DOF 索引 + `weight`。

并发去重用 TBB `enumerable_thread_specific` 局部向量累积，每个 worker 线程自带 `visited_stamp` 数组。**注意：本实现不主动剔除 "顶点-自身三角"、"相邻边共享顶点" 这种几何上合法的邻接对**；EE 的共享顶点由 mollifier 自然衰减吸收，PT 的 "p 就是三角形顶点之一" 则在遍历里由 `if (vi == tri[0] || ...)` 显式跳过（`CIPC.cpp:1369`）。

### 12.2 PT / EE pair 描述

```cpp
struct PTPair { int p, t0, t1, t2; double weight; };
struct EEPair { int ea0, ea1, eb0, eb1; double weight; };
```

全局 DOF 索引统一是 `3*i + 0/1/2`，因此每对局部 12 维与全局稀疏向量/矩阵之间是 **4×3** 索引映射。`computeHessian` 用两层 4×4 循环枚举节点对，3×3 块写入 `hess(3*idx_i + r, 3*idx_j + c)`。

### 12.3 地板 penalty（可选）

构造函数里的 `useFloor_ / floorHeight_ / floorKappa_` 用来打开一层非 IPC 的**软地板**：$0.5\,\kappa_{\text{floor}}(z - h_f)^2$，只在 $z < h_f$ 时有效。**这不是 barrier**，不保证非贯穿，只是一个方便 debug 的下限；tet/cubic 扩展不需要依赖它，建议在体网格主路径默认关掉。

---

## 13. 一次 Newton 迭代的控制流

把上面各节连起来，一次 time step 的核心循环如下（伪代码）：

```
# Warm start
x ← x_hat = x^n + h v^n + h^2 M^{-1} f_ext
repeat (Newton iteration):
    # 宽相 + 当前对 E, ∇E, ∇²E
    findCollisionPairs(x)              # CIPC.cpp:1291
    E  ← computeEnergy(x)              # CIPC.cpp:1685
    g  ← computeGradient(x)            # CIPC.cpp:1752
    H  ← computeHessian(x)             # CIPC.cpp:1850  (含 projectToPSD)
    # 叠加 inertia + elasticity 部分（tet/cubic FEM 装配）
    # 解 Newton 系统
    p ← solve( M/h^2 + H_elastic + H_contact ,  -g )
    # ACCD 限步
    α_max ← computeMaxStepSize(x, p)   # CIPC.cpp:1439
    α ← α_max
    # Armijo 回溯（Newton solver 主循环内）
    while E(x + α p) > E(x) + c1 · α · g·p:
        α ← α / 2
        if α < α_min: break
    x ← x + α p
    if ||p||_∞ / h < ε_N: break
end repeat
v^{n+1} ← (x - x^n) / h;   x^{n+1} ← x
```

在仓库里：

- `findCollisionPairs` 会在 `func / gradient / hessianDirect / computeMaxStepSize` 各自入口被自动调用一次；
- `hessian()` / `createHessian()` 都抛异常（见 `CIPC.cpp:2179-2186`）——**要走的是 `hessianDirect`**。这也意味着把 `CIPCPotentialEnergy` 塞给 Newton solver 的时候必须选 "支持可变拓扑" 的 Hessian 入口，参考 `isHessianTopologyFixed() == 0` 的其他 energy 被接入的方式。

---

## 14. 当前 C-IPC 在仓库里被用在哪（以及没用在哪）

`grep` 全仓（排除 `build/`）`CIPCPotentialEnergy` 的调用点只有一个：

- `src/tools/runSim/runShellSim.cpp:217-222`：薄壳 / cloth 动力学场景专用 driver。

```cpp
std::shared_ptr<Contact::CIPC::CIPCPotentialEnergy> collisionHandler =
  std::make_shared<Contact::CIPC::CIPCPotentialEnergy>(
      surfaceBox.sides().norm() * 1e-3,   // dhat
      E * h,                              // kappa
      true                                // isInputDisp
  );
collisionHandler->setMesh(V, F);  // V, F = 表面三角面片
```

其余（`src/tools/runSim/runSim.cpp`、`runObstacleSim.cpp`、`runSimFromInitState*.cpp` 等）**都没有用 C-IPC**，而是用：

- `TriangleMeshExternalContactHandler` + `PointPenetrationEnergy`（外部 kinematic 物体的二次惩罚），以及
- `TriangleMeshSelfContactHandler` + `PointTrianglePairCouplingEnergyWithCollision`（自接触的二次惩罚）。

`tests/` 下也没有 `CIPC` 字样的回归测试。

**对 tet/cubic 扩展而言**，这意味着：

1. 现有的体仿真主链路（`runSim.cpp` + `DeformationModelManager` + `DeformationModelAssembler` + 时间积分器）是用**旧 penalty 接触**跑起来的；要把 C-IPC 切进去需要改 driver 层，而不是简单复用现有 handler；
2. `CIPCPotentialEnergy` 已经把表面三角网的 barrier / CCD / PSD 全搞定，**关键在 "把体网格暴露给它的表面顶点 DOF 和全局 DOF 之间桥接"**。体网格顶点的"面 DOF"不是独立变量，是全局 DOF 的一部分，只要 `CIPCPotentialEnergy` 直接消费全局 DOF 序列，就不必引入额外的 embedding 矩阵（tet/cubic 里边界三角面片的顶点就是体网格的顶点，索引天然一致）。

---

## 15. 把 C-IPC 扩展到 tet / cubic 的改造蓝图

下面是针对 "首期**不做 friction**、以 runSim 主路径为目标" 的最小扩展方案。
这里只给设计，不写代码。

### 15.1 Scope

- **要做的**：
  1. 让 `CIPCPotentialEnergy` 能与 `runSim.cpp` 的 tet / cubic 体网格主路径配合；
  2. 体网格对体网格的自接触、以及体网格对外部三角面片物体的接触都走 C-IPC；
  3. 保证 cubic 的表面 embedding arity 正确（`plan/cubic_mesh.plan.md` §2C.4 已经点明旧 handler 把 cubic 的 8 节点 embedding 截成 4）。
- **首期不做**：
  1. friction；
  2. C-IPC 的厚度 offset $\xi$（除非后续验证需要）；
  3. 地板 / 盒子等非 barrier 辅助项；
  4. 自适应 kappa（可以先用静态值，后续按 §9 补）。

### 15.2 体网格 → CIPCPotentialEnergy 的接入点

两个自然的设计选择：

**方案 A（推荐）：直接消费体网格的全局 DOF，表面三角网只是拓扑入口。**

- `CIPCPotentialEnergy::setMesh(V, F)` 目前已经只关心拓扑和 rest pos；
- 让传入的 `V` 就是体网格的**所有顶点** `V_all`（即 `numVerts_ = volumetricMesh->numVertices()`），
  `F` 只列体网格**边界三角面**（tet 面体的外表面、cubic 的外表面 quads 先三角化）；
- 这样 `CIPCPotentialEnergy` 输入的 `x` 向量就是体网格的全局 `3*numVertices_`，与 `DeformationModelEnergy` 的 DOF 空间完全一致；
- barrier 力只会非零地作用在出现在 `triangles_` 里的顶点上；非边界顶点的 DOF 在 gradient/Hessian 中天然为 0；
- **内存**：`restPosition` 会比纯表面版本大，但 O($3 \cdot |V_{\text{vol}}|$) 仍然可接受；并且避免了 embedding 矩阵。

**方案 B（不推荐）：引入 embedding 矩阵 $S$ 把表面 DOF 投到体网格 DOF。**

- 和旧 `TriangleMeshSelfContactHandler` 的做法一致；
- 能重用当前 `CIPCPotentialEnergy` 代码的零改动（只要在 Newton 外把 $g_{\text{surf}} \leftarrow S^T g_{\text{vol}}$ 那样桥接）；
- 但多了一层映射，又要在 cubic 下注意 arity，反而比 A 麻烦；
- 只在体网格和接触表面**不同构**（如用 embedded collision mesh）时才值得。

**首期选方案 A**。`runSim.cpp` 在创建 `CIPCPotentialEnergy` 时：

1. 从 `volumetricMesh` 拿 `V_all`（直接复用 tet / cubic 顶点坐标）；
2. 用现有 `GenerateSurfaceMesh::computeMesh(volMesh, triangulate=true)` 提表面三角形；
3. 把 `(V_all, F_surface)` 送进 `setMesh`；
4. 创建 energy 时 `isInputDisp` 需要和现在 FEM energy 的约定一致；
5. 把它加到时间积分器的 "通用隐式力模型" 槽里（和 runShellSim 里 `addGeneralImplicitForceModel` 的接法一致）。

### 15.3 外部 kinematic 物体

当前 `runSim.cpp` 用 `TriangleMeshExternalContactHandler` + `PointPenetrationEnergy`。要换 C-IPC：

- 方案 1（最小改动）：把外部物体的顶点复制进 `CIPCPotentialEnergy` 的 `V`，但把它们对应的 DOF 锁死（Newton 不解这些 DOF）。
- 方案 2（偏多功能）：把 CIPC 改成支持 "两组 DOF，其中 B 组是只读的 obstacle 顶点"。需要在 `CIPCPotentialEnergy` 里增加一个 `kinematicVertexRange` 概念，跳过它们在 gradient/Hessian 的装配。

**首期选方案 1**，因为只要把 obstacle 顶点加到 `V` 再在 Newton 外把它们的 dof 固定（solver 已经支持固定 dof 列表），就能直接复用现有 `CIPCPotentialEnergy`，不必改其内部逻辑。

### 15.4 Tet 专属注意事项

- tet 主路径的 DOF arity 是 `numElementVertices * 3 = 12`，与 C-IPC 的 PT/EE 本地 12 维**无关**（PT/EE 的 12 维是 "3 个 triangle 顶点 + 1 个 point"、或 "4 个 edge 端点"，不是一个 tet）；因此不会发生维度冲突；
- tet 的 inverted element 会让 `computeEnergy` 出现负体积；C-IPC 对此没有直接帮助，依旧靠 FEM 侧的 `enforceSPD` 保证下降方向。C-IPC 只负责不贯穿表面。

### 15.5 Cubic 专属注意事项

- cubic 六面体的外表面是 quad，必须**三角化**再喂 `CIPCPotentialEnergy`；`GenerateSurfaceMesh::computeMesh(..., triangulate=true)` 已经这么做；
- `plan/cubic_mesh.plan.md` §2C.4 已经点明旧 `triangleMeshExternalContactHandler` / `triangleMeshSelfContactHandler` 在展开表面顶点 embedding 时写死了 `vid * 4 + j`，对 cubic 会截断。**切到方案 A 之后，这一问题自动消失**——C-IPC 根本不走 embedding 路径，直接消费全局 DOF。这是选方案 A 的另一个理由。
- cubic 的相邻面在三角化后相邻三角形会共享一条边，`findCollisionPairs` 会把 "同一个 hex 相邻面的两条 EE" 当 EE 候选；这时 mollifier 会把它们衰减到 0。正确性没问题，但会带来少量额外宽相开销；若性能成为瓶颈，可以在 `buildAdjacency` 里扩展 "同一单元内部边-边" 的剔除表。

### 15.6 测试建议（复用 `tests/src/tools/runSim_gtest.cpp` 脚手架）

- **smoke**：加载 box tet + box cubic，分别开 CIPC，跑 5 个时间步不崩；
- **自接触回归**：一条 "U" 型体网格自扣，给初速，看 `d_min` 不跌破 1e-6（单位：world-space mesh 对角线 * ε）；
- **外部接触**：一个 tet 立方体对着一个 kinematic 平板自由落体，CCD 触发的 `α_max` 合理（远小于 1）；
- **cubic embedding arity**：创建一个 `nx=1, ny=1, nz=1` 的单元 + 表面 12 三角形，验证所有表面 DOF 都能被接触力到达（顶点位移梯度非零），对应 `plan/cubic_mesh.plan.md` §2C.4 的问题修复。

### 15.7 改动文件一览（预估）

| 文件 | 变更 |
|------|------|
| `src/core/contact/CIPC.h` | （可选）`setMesh` 再加一个签名接受 `volumetricMesh` 直接生成 `(V, F_surface)`；保留现有 `(V, F)` 签名 |
| `src/core/contact/CIPC.cpp` | 如上，薄封装；内部实现无需改 |
| `src/tools/runSim/runSim.cpp` | 在体网格主路径里新增一条 "useCIPC" 分支，构造 `CIPCPotentialEnergy`，挂到时间积分器，替换旧 handler |
| `tests/src/tools/runSim_gtest.cpp` | 增补 tet / cubic + CIPC smoke & 自接触 & cubic arity 回归 |

注意**不**需要动的：

- `CIPC_autogen*.h`、`CCDKernel.*` 保持不动；
- 旧的 `triangleMeshSelfContactHandler` / `triangleMeshExternalContactHandler` 保持不动（可以并存，让老示例继续工作）；
- `DeformationModelAssembler` / `DeformationModelEnergy` 不用动，CIPC 是独立的 potential。

### 15.8 后续（非首期）

- 加入 §9 自适应 $\kappa$；
- 引入 §4.2 的厚度 offset 做堆叠模拟；
- 引入 friction（§6）——这一步涉及 velocity，需要和时间积分器交互；
- 用真正的 CCD（root-finding）替代 ACCD 的场景几乎不存在——保持 ACCD 即可。

---

## 附录 A：与论文章节的速查

| 本文档 | CIPC 论文 | IPC 论文 |
|--------|----------|---------|
| §2 IP | §3, Eq. (2) | §3, Eq. (1)(2) |
| §3 距离原语 | 继承自 IPC | §3 |
| §4.1 barrier | 继承自 IPC | §3, Eq. (5) |
| §4.2 厚度 offset | §5, §5.1 | 无 |
| §5 EE mollifier | 继承自 IPC | §4 |
| §6 friction | §6 | §4 |
| §7 ACCD | §5.4, Algorithm 1, Eq. (10) | 无（IPC 用 root-finding） |
| §8 PSD 投影 | 继承自 IPC | §4, Appendix |
| §9 自适应 kappa | 继承自 IPC | Appendix |
| §10 filter line search | 继承自 IPC | §4 |

## 附录 B：运行 runShellSim 验证 C-IPC 当前行为

为了在动 tet/cubic 之前先确认 C-IPC 实现本身跑得起来，建议：

1. 找 `examples/` 下与 `runShellSim` 对应的 cloth/shell 配置（如无则暂时手写一个最小 2D patch JSON）；
2. 在 `CIPCPotentialEnergy::computeAll` 里临时加一行日志，打印 `ptPairs_.size() + eePairs_.size()` 和 `min d_k`；
3. 跑几个时间步，确认碰撞对数量非零、最小距离不塌到 0；
4. 该 log 后续删除，它只是 sanity check。

这一步是 15.2 方案 A 切入前的最后稳定点；一旦确认 C-IPC 本身没 bug，再着手改 `runSim.cpp`。
