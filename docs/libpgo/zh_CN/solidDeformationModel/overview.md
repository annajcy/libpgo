# `solidDeformationModel` — 连续介质 FEM 形变模型引擎

> C++ 目录：`src/core/solidDeformationModel/`（约 50+ 文件）。Python 门面见 `pypgo/fem/`（对应文档 [pypgo/fem/overview.md](../../pypgo/zh_CN/fem/overview.md)）。

## 模块职责

把连续介质力学的理论链路——连续体假设 → 运动学（变形梯度 F）→ 本构（应变能 Ψ(F)）→ 离散能量——实现为可求值、可求导、可组装的 C++ 计算引擎。本模块是 libpgo 所有固体仿真（静态平衡、动态时间积分、材料优化、可微仿真）的数值核心。

## 理论流水线

从连续介质到离散 FEM 能量的完整链路，以及每一步的 C++ 实现位置：

$$
\underbrace{\mathbf x = \bar{\mathbf X} + \mathbf u}_{\text{变形映射 }\varphi}
\;\xrightarrow{\text{形函数 }N_a(\boldsymbol\xi)}\;
\underbrace{\mathbf F = \frac{\partial\mathbf x}{\partial\mathbf X}
= \mathbf x\cdot\frac{\partial\mathbf N}{\partial\boldsymbol\xi}^{\!\!\top}\!\!\cdot\mathbf D_m^{-1}}_{\text{变形梯度, } \texttt{VolumetricElementMapping::computeFref}}
\;\xrightarrow{\text{SVD }\mathbf F_e=\mathbf U\boldsymbol\Sigma\mathbf V^\top}\;
\underbrace{\Psi(\mathbf F_e,\mathbf U,\mathbf V,\boldsymbol\Sigma)}_{\text{本构, } \texttt{ElasticModel3DDeformationGradient}}
\;\xrightarrow{\text{求积 }\sum w_q|\det\mathbf D_m|}\;
\underbrace{E(\mathbf u)=\sum_e\sum_q w_q\,\Psi(\mathbf F_e(\mathbf u;\xi_q))\,|\det\mathbf D_m(\xi_q)|}_{\text{离散总能量, } \texttt{computeEnergy}}
$$

其中 $\bar{\mathbf X}$ 是参考位形（rest position），$\mathbf u$ 是位移，$\mathbf F_e = \mathbf F\mathbf A^{-1}$ 为乘法塑性分解后的弹性变形梯度。

## 架构总览

```
                        ┌──────────────────────────────────────┐
                        │        DeformationModelManager       │
                        │  工厂：网格 × Formulation × 本构      │
                        │  → 为每个单元创建 DeformationModel    │
                        └──────────────┬───────────────────────┘
                                       │ 管理
              ┌────────────────────────┼────────────────────────┐
              │                        │                        │
     ┌────────▼────────┐    ┌──────────▼──────────┐    ┌───────▼───────┐
     │ DeformationModel│    │ DeformationModel    │    │  ... (nele)   │
     │   (element 0)   │    │   (element 1)       │    │               │
     └────────┬────────┘    └──────────┬──────────┘    └───────┬───────┘
              │                        │                        │
     ┌────────▼────────────────────────▼────────────────────────▼───────┐
     │                   DeformationModelAssembler                       │
     │  全局组装：逐单元调 computeEnergy / compute_dE_dx / compute_d2E_dx2 │
     │  + 并行 (TBB) + 稀疏 Hessian 填充 + max-step 校验                  │
     └──────────────────────────────────────────────────────────────────┘
```

每个 `DeformationModel`（实现为 `VolumetricDeformationModel` 或 shell 模型）内部持有三条独立子组件：

```
VolumetricDeformationModel
├── VolumetricElementMapping    ← 几何映射：从节点位置算 F（持有 ShapeFunction + Quadrature）
├── ElasticModel3DDeformationGradient ← 本构：Ψ(F), P=∂Ψ/∂F, ∂P/∂F
└── PlasticModel3DDeformationGradient ← 塑性：A(a), A⁻¹, det A, 导数
```

---

## 1. 连续介质假设 → ShapeFunction + Quadrature

### 1.1 理论

连续介质假设把材料视为连续函数场——位移 $\mathbf u(\mathbf X)$、速度 $\mathbf V(\mathbf X,t)$、应力 $\boldsymbol\sigma(\mathbf X,t)$ 定义在物体的每一个材料点 $\mathbf X$ 上。FEM 离散化把连续场用有限个基函数展开，把连续积分用有限个采样点求和代替：

$$
\mathbf u(\boldsymbol\xi) \approx \sum_{a=1}^{n} N_a(\boldsymbol\xi)\,\mathbf u_a,
\qquad
\int_\Omega f(\mathbf X)\,d\mathbf X \approx \sum_{q} f(\boldsymbol\xi_q)\;w_q\;|\det\mathbf D_m(\boldsymbol\xi_q)|
$$

其中 $\boldsymbol\xi = (\xi,\eta,\zeta)$ 是参考单元参数坐标，$N_a$ 是形状函数，$(\boldsymbol\xi_q, w_q)$ 是求积点和权重，$\mathbf D_m = \frac{\partial\bar{\mathbf X}}{\partial\boldsymbol\xi}$ 是参考雅可比。

### 1.2 ShapeFunction — 连续场的离散化

**接口**：`formulations/shapeFunction/shapeFunction.h:10-21`

```cpp
class ShapeFunction {
  virtual int numNodes() const = 0;     // 基函数个数
  virtual int localDofs() const = 0;    // 局部自由度总数
  virtual void N(double xi, double eta, double zeta, double N_out[]) const = 0;
  virtual void dN_dxi(double xi, double eta, double zeta, double dN_out[]) const = 0;
  virtual void nodeCoords(int node, double xi[3]) const = 0;
};
```

`N()` 返回所有基函数在 $(\xi,\eta,\zeta)$ 处的值，`dN_dxi()` 返回 $\partial N_a/\partial\boldsymbol\xi$（按列优先 3×n 布局）。

**具体实现**：

| 类 | 单元 | 标量基函数数 | 局部 DOF | 说明 |
|:--|:--|:--|:--|:--|
| `TetLinearShapeFunction` | 四面体 | 4 | 12 | P1 线性，$\partial N/\partial\boldsymbol\xi$ 为常数 → F 在单元内为常数 |
| `CubicLinearShapeFunction` | 六面体 | 8 | 24 | 三线性，F 在单元内线性变化 |
| `CubicTricubicHermiteShapeFunction` | 六面体 | 64 | 192 | 三三次 Hermite：8 角点 × 8 模态（值 + 7 个导数模态），F 高阶变化 |

以 `TetLinearShapeFunction`（`tetLinearShapeFunction.h:13-17`）为例：

$$
\begin{aligned}
N_0 &= 1 - \xi - \eta - \zeta \\
N_1 &= \xi \\
N_2 &= \eta \\
N_3 &= \zeta
\end{aligned}
\qquad
\frac{\partial\mathbf N}{\partial\boldsymbol\xi} =
\begin{bmatrix}
-1 &  1 &  0 &  0 \\
-1 &  0 &  1 &  0 \\
-1 &  0 &  0 &  1
\end{bmatrix}
$$

导数矩阵为常数（`tetLinearShapeFunction.cpp:22-45`），因此一个四面体内 F 处处相同——这就是 constant strain tetrahedron。

三三次 Hermite（`cubicTricubicHermiteShapeFunction.h:10-34`）每个角点不仅插值位移值，还插值一阶和二阶混合导数：

```
模态 0: VALUE              (∂⁰/∂ξ⁰∂η⁰∂ζ⁰)
模态 1: D_XI               (∂¹/∂ξ¹∂η⁰∂ζ⁰)
模态 2: D_ETA              (∂⁰/∂ξ⁰∂η¹∂ζ⁰)
模态 3: D_ZETA             (∂⁰/∂ξ⁰∂η⁰∂ζ¹)
模态 4: D_XI_ETA           (∂¹/∂ξ¹∂η¹∂ζ⁰)
模态 5: D_XI_ZETA          (∂¹/∂ξ¹∂η⁰∂ζ¹)
模态 6: D_ETA_ZETA         (∂⁰/∂ξ⁰∂η¹∂ζ¹)
模态 7: D_XI_ETA_ZETA      (∂¹/∂ξ¹∂η¹∂ζ¹)
```

共 8×8=64 个标量基函数，每节点 24 DOF（3 空间分量 × 8 模态）。

### 1.3 Quadrature — 材料点采样

**接口**：`formulations/quadrature/quadrature.h:10-18`

```cpp
class Quadrature {
  virtual int numPoints() const = 0;
  virtual void point(int i, double xi[3]) const = 0;
  virtual double weight(int i) const = 0;
};
```

每个求积点就是一个 **material point**（材料点），代表连续体上该位置的采样，携带独立的 $\mathbf F$、$\Psi$、应力等物理量。

| 类 | 点数 | 位置 | 权重 | 配套形状函数 | 依据 |
|:--|:--|:--|:--|:--|:--|
| `TetLinearDefaultQuadrature` | 1 | 重心 (¼,¼,¼) | 1/6 | `TetLinearShapeFunction` | F 为常数，1 点精确 |
| `TetDegree2Quadrature` | 4 | — | — | 四面体二次 | 4 点精确积分 |
| `GaussLegendreHexQuadrature2` | 8 | $0.5\pm\frac{0.5}{\sqrt3}$ 排列 | 1/8 | `CubicLinearShapeFunction` | 2×2×2 对三线性 F 精确 |
| `GaussLegendreHexQuadrature4` | 64 | 4×4×4 Gauss | — | `CubicTricubicHermiteShapeFunction` | Hermite F 更高阶，需 4³=64 点防欠积分 |

**关键设计决策**：求积阶数与插值阶数必须匹配。注释明确说明（`gaussLegendreHexQuadrature.h:28-30`）：

> "Hermite 的变形梯度比三线性更高阶，2×2×2 会欠积分非线性材料"

---

## 2. 运动学 — 变形映射与变形梯度 F

### 2.1 VolumetricElementMapping — 几何映射器

`deformation/volume/volumetricElementMapping.h:18-74` 是 ShapeFunction 和 Quadrature 的组装点，负责：

**构造函数预计算**（`volumetricElementMapping.cpp:21-82`）——所有只依赖参考几何的量只算一次：

对每个求积点 $q$：

$$
\begin{aligned}
\mathbf D_m &= \bar{\mathbf X}\cdot\left(\frac{\partial\mathbf N}{\partial\boldsymbol\xi}\right)^{\!\!\top} \quad\text{(参考雅可比, 3×3)} \\
\mathbf D_m^{-1} &= \mathbf D_m^{-1} \\
\frac{\partial\mathbf N}{\partial\mathbf X} &= \mathbf D_m^{-\top}\cdot\frac{\partial\mathbf N}{\partial\boldsymbol\xi} \quad\text{(物理空间形状导数)} \\
w_q^{\text{phys}} &= |\det\mathbf D_m| \cdot w_q \quad\text{(物理体积微元)}
\end{aligned}
$$

**运行时 F 计算**（`volumetricElementMapping.cpp:84-96`）：

```cpp
void computeFref(const double *xLocal, int q, double F[9]) const {
    // xLocal = 当前节点位置（变形后）
    // 组装系数矩阵 (3 × numNodes)
    M3xN coefficients(3, numNodes_);
    for (int node = 0; node < numNodes_; node++)
        coefficients.col(node) = V3d(xLocal[node*3+0], ...);
    // F = x · (dN/dξ)ᵀ · Dm⁻¹
    FMap = coefficients * dN_dxi_[q].transpose() * restDmInv_[q];
}
```

这就是理论公式 $\mathbf F = \frac{\partial\mathbf x}{\partial\mathbf X}$ 的数值实现。注意这里的 `xLocal` 是当前配置下的节点位置 $\mathbf x = \bar{\mathbf X} + \mathbf u$，不是位移。

**预计算的 dF/dx**（`volumetricElementMapping.cpp:72-80`）：

$$
\left.\frac{\partial\mathbf F}{\partial\mathbf x}\right|_{\text{rest}}
$$

在参考位形预计算（9 × localDofs 矩阵），运行时通过 `computedFrefdx()` 直接取出。这是梯度和 Hessian 组装的基础——链式法则 $\partial\Psi/\partial\mathbf x = (\partial\Psi/\partial\mathbf F):(\partial\mathbf F/\partial\mathbf x)$。

### 2.2 塑性分解

`VolumetricDeformationModel::prepareData()`（`volumetricDeformationModel.cpp:81-136`）在每个求积点执行乘法塑性分解：

$$
\mathbf F_e = \mathbf F_{\text{ref}} \cdot \mathbf F_p^{-1}
$$

其中 $\mathbf F_p = \mathbf A(\mathbf a)$ 由塑性参数 $\mathbf a$ 通过塑性模型参数化。随后对 $\mathbf F_e$ 做 SVD：

$$
\mathbf F_e = \mathbf U \boldsymbol\Sigma \mathbf V^\top
$$

并将 $\mathbf U, \mathbf V, \boldsymbol\Sigma$ 存入缓存（`volumetricDeformationModelCacheData.h:36-39`），传递给本构模型的 `compute_psi`。

---

## 3. 本构模型 — 应变能 Ψ(F)

### 3.1 抽象接口

`material/elastic/elasticModel3DDeformationGradient.h` 定义纯虚函数：

```cpp
virtual double compute_psi(const double *param, const double F[9],
    const double U[9], const double V[9], const double S[3]) const = 0;
virtual void compute_P(const double *param, const double F[9],
    const double U[9], const double V[9], const double S[3], double P[9]) const = 0;
virtual void compute_dPdF(const double *param, const double F[9],
    const double U[9], const double V[9], const double S[3], double dPdF[81]) const = 0;
```

三个方法对应三个层次的导数：
- `compute_psi` → 能量密度 $\Psi(\mathbf F)$
- `compute_P` → 第一 Piola-Kirchhoff 应力 $\mathbf P = \partial\Psi/\partial\mathbf F$
- `compute_dPdF` → 材料模量 $\partial\mathbf P/\partial\mathbf F = \partial^2\Psi/\partial\mathbf F^2$

输入同时包含 $\mathbf F$ 和其 SVD 分解 $(\mathbf U,\mathbf V,\boldsymbol\Sigma)$，避免每个模型各自重复 SVD。

### 3.2 具体模型

| 模型 | 文件 | 能量密度形式 | 特点 |
|:--|:--|:--|:--|
| `StableNeoHookean` | `material/elastic/elasticModelStableNeoHookeanMaterial.h` | $\frac{\mu}{2}(I_C-3) + \frac{\lambda}{2}(J-1-\alpha)^2$ | SVD 投影 Hessian 保证正定性（Disney-Pixar 方法） |
| `StVK` | `material/elastic/elasticModel3DSTVKMaterial.h` | 基于不变量 $I_1,I_2,I_3$ | 几何非线性，大旋转下可能不保持正定 |
| `StVKVolume` | — | 仅体积部分 | 体积能量独立 |
| `MooneyRivlin` | `material/elastic/elasticModel3DMooneyRivlin.h` | 基于缩减不变量 $\bar I_1,\bar I_2,J$ | 多项式系数 $C_{pq}$ + 体积模量 $D$ |
| `LinearElastic` | `material/elastic/elasticModelLinearMaterial.h` | 小应变线性 | 各向同性线性弹性 |
| `KoiterStVK` | — | 壳 StVK | 配合 `KoiterShell` 使用 |

### 3.3 能量积分

`VolumetricDeformationModel::computeEnergy()`（`volumetricDeformationModel.cpp:143-157`）：

```cpp
double energy = 0.0;
for (int q = 0; q < numQuadPts_; q++) {
    energy += elasticModel_->compute_psi(mp, Fe[q], U[q], V[q], S[q])
            * elementMapping_.weightDetJ(q)   // |det Dm| × w_q
            * cd->detFp[q];                    // 塑性体积修正 det Fp
}
return energy;
```

对应理论积分：

$$
E(\mathbf u) = \sum_e \sum_q \Psi\big(\mathbf F_e(\mathbf u;\xi_q)\big) \cdot |\det\mathbf D_m(\xi_q)| \cdot w_q \cdot \det\mathbf F_p
$$

---

## 4. 组装层 — DeformationModelAssembler

`deformation/deformationModelAssembler.cpp`（约 1100 行）负责逐元素并行组装全局能量、梯度和 Hessian：

- **`computeEnergy`**：TBB 并行对所有元素求和
- **`compute_dE_dx`**：调每个元素的 `compute_dE_dx`（内力），经 `DofLayout::scatterAddGradient` 散射到全局梯度
- **`compute_d2E_dx2`**：调 `compute_d2E_dx2`（切线刚度矩阵），经 `DofLayout::addHessianSparsity` 填入稀疏模板
- **参数导数**：`compute_dE_db` / `compute_d2E_dxdb` 等，供材料优化和可微仿真使用

DofLayout（`formulations/dof/dofLayout.h:20-47`）抽象了 DOF 聚集 / 散射策略，使得 assembler 不感知单元类型（线性四面体还是 Hermite）。

---

## 5. 支持的单元-求积组合

| Formulation | 单元 | 形状函数 | 每顶点 DOF | 求积 | 计数 |
|:--|:--|:--|:--|:--|:--|
| `TetLinear` | 4 节点四面体 | P1 线性 | 3 | 1 点（重心） | `TetLinearShapeFunction` + `TetLinearDefaultQuadrature` |
| `CubicLinear` | 8 节点六面体 | 三线性 | 3 | 2×2×2 Gauss | `CubicLinearShapeFunction` + `GaussLegendreHexQuadrature2` |
| `CubicTricubicHermite` | 六面体 | 三三次 Hermite | 24 | 4×4×4 Gauss | `CubicTricubicHermiteShapeFunction` + `GaussLegendreHexQuadrature4` |

### 单元选择对精度/性能的影响

- **TetLinear**：每个四面体 F 为常数（constant strain），低阶近似但计算最快。需要足够密的网格来解析应变梯度。
- **CubicLinear**：F 在六面体内线性变化，比四面体能更好地解析弯曲。
- **CubicTricubicHermite**：F 在六面体内高阶变化，24 DOF/顶点可精确捕捉弯曲和扭转。对同样精度需要更少单元，但每个单元的计算量更大（64 积分点 × SVD × 本构求值）。

---

## 6. Python 入口

Python 用户通过 `pypgo.fem` 包使用本模块（详见 [pypgo/fem/overview.md](../../pypgo/zh_CN/fem/overview.md)）：

```python
import pypgo

# 1. 选 formulation（决定形状函数 + 求积规则）
fm = pypgo.fem.TetLinear()        # 或 CubicLinear / CubicTricubicHermite

# 2. 构造能量
energy = pypgo.fem.elastic_material_energy(
    sim_mesh,                      # SimulationMesh（rest positions + 拓扑）
    formulation=fm,                # 决定 F 怎么从 DOF 算
    elastic_model="stable_neo",    # 决定 Ψ(F) 的形式
    density=1e6,                   # 材料参数
)

# 3. 求值
E = energy.value(u)                # → C++ computeEnergy
g = energy.gradient(u)             # → C++ compute_dE_dx
H = energy.hessian(u)              # → C++ compute_d2E_dx2
```

---

## 7. 关键文件索引

| 文件 | 职责 | 行数（约） |
|:--|:--|:--|
| `formulations/shapeFunction/shapeFunction.h` | ShapeFunction 抽象接口 | 25 |
| `formulations/shapeFunction/tetLinearShapeFunction.h/.cpp` | P1 四面体形函数 | 43 / 96 |
| `formulations/shapeFunction/cubicTricubicHermiteShapeFunction.h/.cpp` | 三三次 Hermite 形函数 | 47 / ~300 |
| `formulations/quadrature/quadrature.h` | Quadrature 抽象接口 | 22 |
| `formulations/quadrature/tetLinearDefaultQuadrature.h/.cpp` | 四面体 1 点求积 | 25 / 22 |
| `formulations/quadrature/gaussLegendreHexQuadrature.h/.cpp` | 六面体 Gauss 求积 (2³ / 4³) | 38 / — |
| `formulations/formulation/formulation.h` | Formulation 抽象基类（创建 DeformationModel 的工厂） | 40 |
| `formulations/dof/dofLayout.h` | DOF 聚集/散射策略接口 | 50 |
| `formulations/dof/vertex3DofLayout.h/.cpp` | 每顶点 3 DOF 布局（TetLinear / CubicLinear） | — |
| `formulations/dof/cubicTricubicHermiteDofLayout.h/.cpp` | 每顶点 24 DOF 布局 | — |
| `deformation/volume/volumetricElementMapping.h/.cpp` | 几何映射：参考预计算 + 运行时 F | 77 / 106 |
| `deformation/volume/volumetricDeformationModel.h/.cpp` | 逐元素能量/梯度/Hessian + SVD + 塑性分解 | — / ~700 |
| `deformation/volume/volumetricDeformationModelCacheData.h` | 每求积点缓存：Fe, U, V, S, dFdx, Bm | 66 |
| `material/elastic/elasticModel3DDeformationGradient.h` | 本构抽象基类：compute_psi / compute_P / compute_dPdF | — |
| `material/elastic/elasticModelStableNeoHookeanMaterial.h/.cpp` | 稳定 Neo-Hookean 模型（最常用） | — |
| `material/elastic/elasticModelFactory.h/.cpp` | 本构工厂（名字 → 模型实例） | — |
| `material/plastic/plasticModel3DDeformationGradient.h` | 塑性抽象基类：computeA / computeAInv / compute_detA | — |
| `deformation/deformationModelAssembler.h/.cpp` | 全局组装器（~1100 行）：并行能量/梯度/Hessian | — / ~1100 |
| `deformation/deformationModelManager.h/.cpp` | 管理器：创建/持有所有 DeformationModel | — / ~300 |
| `deformation/deformationModel.h` | DeformationModel 抽象基类 | 99 |
