# Energy API Refactor Plan

> **状态日期：** 2026-05-28
> **适用范围：** C++ `nonlinearOptimization` energy 边界重构 + Python `pypgo.energy` binding。
> **执行约束：** 不重写数值内核（`func` / `gradient` / `hessian` 的内部计算保持不变）；本计划只重构 energy 的 ownership、组合、生命周期、求值边界，让 Python binding 不需要在外面打补丁。

## 目标

把 `PotentialEnergy` 周围的“可绑定边界”收干净，让 `pypgo.energy` 的 Python API 能写成：

```python
import pypgo as pgo

elastic = pgo.energy.deformation_energy(...)

# External force f contributes -f^T u to potential energy.
# Python 不提供 sign-flip 便利包装；用户显式 LinearEnergy(f) + 负权。
force = pgo.energy.LinearEnergy(f)

total = pgo.energy.EnergySet([
    (elastic, 1.0),
    (force,  -1.0),
])

u = total.zero_state()
e = total.value(u)
g = total.gradient(u)
H = total.hessian(u)
```

不让 Python 用户看到：

- `PotentialEnergies::addPotentialEnergy()` + `init()` 的两阶段构造；
- Hessian 拓扑固定/非固定的两条路径区分（重命名前 `hessian` / `hessianDirect`、重命名后 `hessianInPlace` / `hessian`，详见 §4）；
- 任何 `shared_ptr<void>` / `keepAlive_` 形式的“binding 层补洞” ownership；
- `getDOFs(out)` 这类 C 风格 out-param 接口；
- `LinearPotentialEnergy` / `QuadraticPotentialEnergy` 持有的外部 `const VXd &` / `const SpMatD &` 引用。

## 当前问题

相关文件：

- `src/core/nonlinearOptimization/potentialEnergy.h`
- `src/core/nonlinearOptimization/potentialEnergies.h`
- `src/core/nonlinearOptimization/potentialEnergies.cpp`
- `src/core/genericPotentialEnergies/linearPotentialEnergy.h/.cpp`
- `src/core/genericPotentialEnergies/quadraticPotentialEnergy.h/.cpp`
- `src/core/constraintPotentialEnergies/multiVertexPullingSoftConstraints.h/.cpp`（见 §8 / Task E9）
- `src/core/nonlinearOptimization/potentialEnergyFromConstraintFunctions.h`
- `src/core/contact/mappedSurfacePotentialEnergy.h`（行为依赖：`isHessianTopologyFixed() == 0`）
- `src/python/pypgo/bindings/energy_bindings.cpp`（M3 binding 入口）

具体别扭点：

### 1. 两阶段构造 `PotentialEnergies`

```cpp
auto energyAll = std::make_shared<PotentialEnergies>(n3);
energyAll->addPotentialEnergy(e0, 1.0);
energyAll->addPotentialEnergy(e1, 1.0);
energyAll->init();
```

Python 用户忘记 `init()`，hessian mapping / buffer 不会建好，运行时才报。

### 2. `hessian` vs `hessianDirect`

`PotentialEnergy::hessian(x, H)` 假设 `H` 已经按 `createHessian(H)` 建好拓扑。对 IPC 这类 `isHessianTopologyFixed() == 0` 的 energy，调用方必须改走 `hessianDirect(x, H)`。Python 用户没理由知道这个分支。

### 3. 通用 energy 持外部引用

`LinearPotentialEnergy` 持 `const EigenSupport::VXd &b`；`QuadraticPotentialEnergy` 持 `const EigenSupport::SpMatD &A` 和 `const EigenSupport::VXd *b`。Python 传进来的 NumPy / 临时 Eigen 矩阵 lifetime 不一定比 energy 长，绑定时极易出错。

### 4. `getDOFs(std::vector<int> &out)` 是 out-param

不适合直接绑定成 Python `.dofs` property。

### 5. 没有“ownership root”

每种具体 energy 的 binding 都得自己想办法保活外部依赖（simulation mesh、contact scene、material data）。一旦在 binding 层用 `std::vector<std::shared_ptr<void>> keepAlive_` 顶住，就把生命周期从类型系统里抹掉了——读不懂、改不动、不能 review。

### 6. `MappedSurfacePotentialEnergy` 的 state convention 没在类型上表达

它要的是 simulation displacement，但 base class `PotentialEnergy::func(x)` 没说明 `x` 是位移还是绝对位置。Python 用户必须靠文档区分。

### 7. `MultipleVertexPulling` 持 raw pointer 和 reference 依赖

`ConstraintPotentialEnergies::MultipleVertexPulling` 继承自 `PotentialEnergyAligningMeshConnectivity` → `PotentialEnergy`，是每个 IPC static/dynamic example 的必需品（"fixed-vertices" loading）。当前 ctor：

```cpp
MultipleVertexPulling(const SpMatD &Koff, const double *restPositionsAll,
  int numPts, const int *vertexIndices, const double *tgt,
  const double *bcCoeff, int isDisp);
```

持 `const SpMatD &` 引用（Koff sparsity 模板）+ 4 个 raw pointer，ownership 全部甩给 caller。Python binding 根本无法安全暴露这套 API。与 Linear/Quadratic 同样的改造原则应该适用：改成 owning by value。

## 非目标

- 不重写 `func` / `gradient` / `hessian` 的数值实现；只动 ownership、组合、求值 facade。
- 不引入 Python 子类化 `PotentialEnergy` 的 trampoline。Python-defined energy 列入 `future_work.md`，本计划不涉及。
- 不重构 `EnergyOptimizer::minimize` / IPOPT / Knitro / constraints 入口；solver 部分见 `solver_api_refactor.plan.md`。
- 不为旧 `PotentialEnergies` 公开 API 做向后兼容；M3 是首次公开 binding，没有调用方需要保护，可以直接破坏式迁移到 `EnergySet`。
- 不为 `SmoothRSEnergy` (MKL-gated) 提前规划 Python 表面；该 energy 在 M9 之后再决定。
- 不在本计划解决 contact energy 的 ownership 细节，那部分留给 `contact_api_refactor.plan.md`；本计划只保证 contact energy 能以 `shared_ptr<const PotentialEnergy>` 形态被组合进 `EnergySet`。

## 设计决策

### 1. `PotentialEnergies` → `EnergySet`（彻底替代，不留过渡 alias）

把 `PotentialEnergies` 重命名为 `EnergySet`，让它**自己**就是组合 energy；不再分两阶段构造、不再暴露 `addPotentialEnergy` / `init`。**不保留 `PotentialEnergies` 作为内部 alias 或包装层**——两个名字描述同一件事会成为长期负债。

内部调用点很少（构造 2 处、引用参数 3 处，详见 Task E3a），一次性迁移到 `EnergySet`。

```cpp
namespace pgo::NonlinearOptimization
{
class EnergySet : public PotentialEnergy, public LineSearchAwareEnergy
{
public:
  struct Term
  {
    std::shared_ptr<const PotentialEnergy> energy;
    double weight = 1.0;
  };

  EnergySet(int numDofs, std::vector<Term> terms);
  // override PotentialEnergy interface; init/mapping done in ctor.

  int numTerms() const;
  const Term &term(int i) const;
  void setWeight(int i, double w);

  // 不暴露 add / init。

  // setWeight 只影响 accumulate 阶段的 weight 乘数，不改变 sparsity 模板或 mapping。
  // 因此是 O(1) 操作，Newton solver 运行时切换也安全。
};

using EnergySet_p = std::shared_ptr<EnergySet>;
using EnergySet_const_p = std::shared_ptr<const EnergySet>;
}  // namespace pgo::NonlinearOptimization
```

构造内部完成的工作：

- 保存 `terms_`（owning `shared_ptr<const PotentialEnergy>`）。
- 用现有 `PotentialEnergies::init()` 的逻辑建 `hessianAll` 模板、`hessianMatrixMappings`、`allDOFs`、`isQuadraticEnergy`、`hasHessianVectorProduct`。
- `isHessianTopologyFixed()` 由 children 决定：任一 child 不固定就不固定。
- 新 `hessian`（base 默认 = `hessianAlloc + hessianInPlace`）内部根据 children 拓扑性质走当前 mapping / `hessianAlloc` + accumulate 路径。

`runIPCSim` / 内部代码原本组合 energy 的位置改用 `EnergySet`（详见 Task E3a 的 caller 迁移清单）。

### 2. ownership 不靠 `keepAlive_`

具体 energy 自己 own 自己的依赖。`EnergySet` 只持有 `shared_ptr<const PotentialEnergy>` 列表，这是它真实的组合关系，不是“隐式保活”。

举例：

```cpp
class DeformationEnergy : public PotentialEnergy
{
  std::shared_ptr<const SimulationMeshCore> simulationMesh_;
  DeformationModelBundle bundle_;
  // ...
};
```

`DeformationEnergy` 把它需要的 mesh / bundle 都作为**具名成员**保留。这是 lifetime 的真实语义，不是杂物抽屉。

Contact energy 同理，见 `contact_api_refactor.plan.md`。

### 3. 通用 energy：原类改 owning（不引入 Owned 变体）

把现有 `PredefinedPotentialEnergies::LinearPotentialEnergy` / `QuadraticPotentialEnergy` 持有 `const VXd &b` / `const SpMatD &A` / `const double *W` 的 reference 成员改成 owning by value：

```cpp
namespace pgo::PredefinedPotentialEnergies
{
class LinearPotentialEnergy : public PotentialEnergy
{
public:
  explicit LinearPotentialEnergy(EigenSupport::VXd b);
private:
  EigenSupport::VXd b_;
  std::vector<int> allDOFs_;
};

class QuadraticPotentialEnergy : public PotentialEnergy
{
public:
  explicit QuadraticPotentialEnergy(EigenSupport::SpMatD A);
  QuadraticPotentialEnergy(EigenSupport::SpMatD A, EigenSupport::VXd b);
  // inParentheses / W 的几个 ctor 保持算式语义不变，但 A / b / W 一律 by value owning
  // W 类型从 const double * 改成 EigenSupport::VXd
private:
  EigenSupport::SpMatD A_;
  std::optional<EigenSupport::VXd> b_;
  std::optional<EigenSupport::VXd> W_;
  // ... 现有 ATA_, bTA_, cache_ 等内部 precompute 保留
};
}
```

ctor 接 by value，配合 move semantics：

- 调用方持有局部 `VXd staticForce = ...` → `make_shared<LinearPotentialEnergy>(std::move(staticForce))` 零拷贝；
- 调用方持有 class member `sys` → `make_shared<QuadraticPotentialEnergy>(sys)` 一次 copy，与现有 `createHessian(H) { hess = A; }` 同量级，可忽略；
- Python binding 从 NumPy → Eigen 转换本来就要拷一次，owning 是免费。

**不引入 `OwnedLinear/QuadraticPotentialEnergy` 包装类。** 两个名字描述本该是默认行为的东西是长期负债，跟删 `PotentialEnergies` 同样逻辑。

**不预留 `shared_ptr<const SpMatD>` 共享入口。** "多个 energy 共享同一 A" 在当前 codebase 不存在；可设想的少量场景（time integrator 跨帧、参数扫）都有更好的解（保留 energy + setter、单 energy + 重组 A）。真出现这种需求时再加 ctor 重载即可，符合 YAGNI。

不动 `QuadraticPotentialEnergy` 现有 6 个 ctor 的算式语义和命名（`inParentheses` flag）；命名清理（`quadratic_form` / `least_squares` 工厂）作为可分离的后续 task，不在本 plan 范围。

### 4. Hessian API rename：`hessianDirect → hessian → hessianInPlace`、`createHessian → hessianAlloc`

现状命名误导：

| 现有名 | 实际语义 | 谁用 |
|---|---|---|
| `createHessian(H)` | 分配 + 建 sparsity 模板 | Newton 热路径初始化 |
| `hessian(x, H)` | **假设 H 已就位**，原地填值（fixed-topology only） | Newton 热路径每 iter |
| `hessianDirect(x, H)` | 一步到位求 Hessian（base 默认 `createHessian + hessian`；non-fixed override 自重建） | one-shot caller / non-fixed-topology |

`hessian` 听起来像默认入口但其实是 hot-path 优化版；`hessianDirect` 听起来像"加强版"但其实才是安全的 fallback；`createHessian` 名字看不出它只是分配模板。

按真实语义重命名：

| 新名 | 旧名 | 语义 |
|---|---|---|
| `hessian(x, H)` | `hessianDirect(x, H)` | 安全默认入口；base 默认 `hessianAlloc + hessianInPlace`，subclass 可 override |
| `hessianInPlace(x, H)` | `hessian(x, H)` | 假设 H sparsity 已就位的 hot-path；每个 subclass 实现 |
| `hessianAlloc(H)` | `createHessian(H)` | 分配 H 并建 sparsity 模板；每个 subclass 实现 |

Newton 热路径调用形态：

```cpp
SpMatD H;
energy->hessianAlloc(H);             // 一次
for (iter ...) {
  energy->hessianInPlace(x, H);      // 每次：纯填值
  // ...
}
```

One-shot / Python / non-fixed-topology 调用形态：

```cpp
SpMatD H;
energy->hessian(x, H);               // 一步到位
```

`evaluation.h::evaluateHessian` 内部就是调 `energy.hessian(x, H)`，不再需要 `hessianDirect` 名字。

**Scope：** 这是个跨 40+ 文件的 mechanical rename（base class、所有 PotentialEnergy subclass override、NewtonSolver、TimeIntegrator、ConstraintFunctions、Python binding）。必须在 Task E0 中作为单一 coordinated 改动一次完成；放在 E1 之前，让后续所有 task 都基于新名字写。

### 5. 求值 facade：`evaluation.h`

提供一组无状态 helper，包掉 out-param、NumPy 转换需要的 by-value 返回：

```cpp
namespace pgo::NonlinearOptimization
{
double evaluateValue(const PotentialEnergy &energy, EigenSupport::ConstRefVecXd x);

EigenSupport::VXd evaluateGradient(const PotentialEnergy &energy, EigenSupport::ConstRefVecXd x);

EigenSupport::SpMatD evaluateHessian(const PotentialEnergy &energy, EigenSupport::ConstRefVecXd x);

MaxStepResult evaluateMaxStep(
  const PotentialEnergy &energy,
  EigenSupport::ConstRefVecXd x,
  EigenSupport::ConstRefVecXd dx);

std::vector<int> dofsOf(const PotentialEnergy &energy);
}
```

每个 `evaluate*` 内部先调 `validateStateSize(energy, x)` 做 precondition check；该函数是 `evaluation.cpp` 内部 helper，不出现在公开 header。

关键认识：rename 之后 **`hessian(x, H)` 本身就是 dispatching entry point**——base 默认实现 = `hessianAlloc + hessianInPlace`（适用 fixed-topology），non-fixed-topology subclass override 它做 per-x 重建。所以 helper 不需要 `isHessianTopologyFixed()` if 分支：

```cpp
EigenSupport::SpMatD evaluateHessian(
  const PotentialEnergy &energy, EigenSupport::ConstRefVecXd x)
{
  EigenSupport::SpMatD H;
  energy.hessian(x, H);
  return H;
}
```

`hessianInPlace(x, H)` 单独存在是给 **Newton 热路径**——caller 自己 `hessianAlloc(H)` 一次、复用 `H` buffer 和 symbolic factorization，每次 iter 只调 `hessianInPlace` 填值。这条 hot path 不走 evaluation helper。

Python binding 一律走这层 helper；不直接调 `PotentialEnergy::hessianInPlace` 或 `hessianAlloc`。

### 6. Python-facing handle 是 `shared_ptr<const PotentialEnergy>`

Python binding 不为每个 energy 类型自造一个 “Core” wrapper。只规定一条规则：**所有公开 Python energy class 内部都持一个 `std::shared_ptr<const PotentialEnergy>`**，由具体 energy 自己 own 其依赖。

```cpp
// pseudo-binding header
class PyEnergyBase {
public:
  std::shared_ptr<const PotentialEnergy> handle;
};

class PyDeformationEnergy : public PyEnergyBase {
public:
  // ctor 把 DeformationEnergy(simulationMeshCore, bundle, spec) 包成 handle
};
```

`EnergySet` Python class 持 `std::shared_ptr<EnergySet>`，构造时从 `[(child_handle, w)]` 取 `child_handle->handle`。`EnergySet` 通过 `terms_` 已经 own 所有 child energy `shared_ptr`，所以不需要 Python wrapper 额外 keep-alive。

### 7. State convention 在 Python wrapper 层声明

C++ base 类不强制区分 displacement / generic vector。Python wrapper 暴露一个 metadata 属性：

```python
energy.state_kind  # "displacement" or "generic"
energy.zero_state()
```

- `deformation_energy` / contact / `external_force`：`state_kind == "displacement"`。
- `linear` / `quadratic`：`state_kind == "generic"`。

`zero_state()` 返回 `np.zeros(num_dofs, dtype=np.float64)`。

C++ 这层不维护 enum；只在 Python wrapper class 上写常量。

### 8. `MultipleVertexPulling` 改 owning + Koff sparsity 模板从 `SimulationMesh` 取

把现有 `MultipleVertexPulling` 改成 owning by value（与 Linear/Quadratic 同样的原则）：

```cpp
namespace pgo::ConstraintPotentialEnergies
{
class MultipleVertexPulling : public PotentialEnergyAligningMeshConnectivity
{
public:
  MultipleVertexPulling(
    EigenSupport::SpMatD Koff,
    EigenSupport::VXd restPositionsAll,
    std::vector<int> vertexIndices,
    EigenSupport::VXd targetPositions,
    double coeff = 1.0,
    bool isDisplacement = true);

  void setTargetPositions(EigenSupport::VXd tgt);
  void setCoeff(double coeff);
  // setMasks 在 M3 scope 内不暴露（当前所有调用方都传 nullptr）

private:
  EigenSupport::SpMatD Koff_;
  EigenSupport::VXd restpAll_;
  EigenSupport::VXd tgtp_;
  std::vector<int> vertexIndices_;
  double coeffAll_ = 1.0;
  int isDisp_;
};
}
```

**Koff 模板来源**：`MultipleVertexPulling` 的 ctor 需要 `Koff` 作为 Hessian sparsity 模板（由基类 `PotentialEnergyAligningMeshConnectivity` 使用，对组合 vertex 做联通对齐）。Python 构造时，这个矩阵从 `SimulationMesh` 获取：

```cpp
// SimulationMesh / SimulationMeshCore 新增：
EigenSupport::SpMatD getHessianSparsityTemplate() const;
```

这是 sim_mesh 上新 method——它本身就持有 DOF 结构信息，推导出 Hessian 模板在概念上属于 mesh 的职责。不耦合具体 elastic model。

Python API：

```python
pin = pgo.energy.VertexAttachment(
    sim_mesh=sim_mesh,               # 提供 DOF 数和 Koff sparsity 模板
    vertex_indices=[0, 1, 2],        # (m,) int64
    target_positions=targets,         # (m, 3) float64
    coeff=1e6,                        # scalar; per-vertex coeffs 不在 M3 scope
    is_displacement=True,
)

# Static solve: 构造后不改目标
# Dynamic loop: 每帧推进
pin.set_targets(new_targets)

# 组合进 EnergySet
total = pgo.energy.EnergySet([
    (elastic, 1.0),
    (pin,     1.0),
])
```

`state_kind == "displacement"`。

**不绑定姊妹类：** `MultiVertexConstrainedRigidMotion`、`BarycentricCoordinateSlidingSoftConstraints`、`MultipleVertexSlidingSoftConstraints`、`MultipleVertexPullingSoftConstraintsPOrder` 列入 `future_work.md`。

**Cross-plan dependency：** `deformation_fem_api_refactor.plan.md` 需在 `SimulationMeshCore` 接口上新增 `getHessianSparsityTemplate()`（若尚未隐含）。M3 期间，`VertexAttachment` 绑定可临时接受 `(rows, cols, vals, shape)` 作为 `Koff` 的兜底输入，与 `QuadraticEnergy` 的 A 矩阵兜底路径一致。

## 目标 C++ API

```cpp
// nonlinearOptimization/potentialEnergy.h          (unchanged interface)
// nonlinearOptimization/energySet.h                (new, replaces public PotentialEnergies)
// nonlinearOptimization/evaluation.h               (new)
// genericPotentialEnergies/linearPotentialEnergy.h     (modified: owning by value)
// genericPotentialEnergies/quadraticPotentialEnergy.h  (modified: owning by value)

namespace pgo::NonlinearOptimization
{

class EnergySet : public PotentialEnergy, public LineSearchAwareEnergy
{
public:
  struct Term {
    std::shared_ptr<const PotentialEnergy> energy;
    double weight = 1.0;
  };

  EnergySet(int numDofs, std::vector<Term> terms);

  int numTerms() const;
  const Term &term(int i) const;
  void setWeight(int i, double w);

  // PotentialEnergy + LineSearchAwareEnergy interface, implemented directly.
};

double evaluateValue(const PotentialEnergy &energy, EigenSupport::ConstRefVecXd x);
EigenSupport::VXd evaluateGradient(const PotentialEnergy &energy, EigenSupport::ConstRefVecXd x);
EigenSupport::SpMatD evaluateHessian(const PotentialEnergy &energy, EigenSupport::ConstRefVecXd x);
MaxStepResult evaluateMaxStep(const PotentialEnergy &energy, EigenSupport::ConstRefVecXd x, EigenSupport::ConstRefVecXd dx);
std::vector<int> dofsOf(const PotentialEnergy &energy);

}  // namespace pgo::NonlinearOptimization

// LinearPotentialEnergy / QuadraticPotentialEnergy 改 owning by value（详见 §3）。
// MultipleVertexPulling 改 owning by value（详见 §8）。
```

`PotentialEnergies` header / 实现整体被 `EnergySet` 替代后删除；不保留 alias、不保留 transition 期 typedef。`addPotentialEnergy` / `init` 在重命名后也一并消失。

## Python API 定稿草案

```python
import pypgo as pgo
import numpy as np

elastic = pgo.energy.deformation_energy(sim_mesh, ...)
floor   = pgo.contact.FloorEnergy(...)
b       = np.zeros(elastic.num_dofs); b[1::3] = -9.81
force   = pgo.energy.LinearEnergy(b)

total = pgo.energy.EnergySet([
    (elastic, 1.0),
    (floor,   1.0),
    (force,  -1.0),
])

assert total.num_dofs == elastic.num_dofs
assert total.state_kind == "displacement"

u = total.zero_state()
e = total.value(u)
g = total.gradient(u)       # np.ndarray (num_dofs,)
H = total.hessian(u)         # pypgo.sparse.SparseMatrix
rows, cols, vals = H.to_coo()

total.set_weight(2, 0.0)     # 暂时关掉 LinearEnergy term
```

约束：

- `EnergySet([...])` 内部立刻完成 C++ 构造 + `init()`；Python 没有“先 add 再 init”的形态。
- `total.hessian(u)` 永远返回完整建好拓扑的 sparse；用户不需要管 fixed/non-fixed topology。
- `EnergySet` Python class 通过 `shared_ptr<const PotentialEnergy>` 持 child；child energy 的 lifetime 由 child 自身管理，删掉 Python `elastic` 变量后 `total` 仍可用。
- `LinearEnergy` / `QuadraticEnergy` 绑 owning 版本：构造时 copy NumPy data 到 C++ owned 存储；Python NumPy 数组生命周期与 energy 解耦。
- `EnergySet.set_weight(i, w)` 走 `EnergySet::setWeight`。
- `total.dofs` 是 `np.ndarray[int64]`，由 `dofsOf(*total)` 计算。

`pypgo.energy` 模块表面（M3 首批）：

```text
pypgo.energy
  PotentialEnergy           # 只读 handle，无 Python 子类化
  EnergySet
  LinearEnergy              # owning
  QuadraticEnergy           # owning
  VertexAttachment          # owning, displacement state
  deformation_energy(...)   # factory, returns DeformationEnergy
```

不暴露：`PotentialEnergies`、`addPotentialEnergy`、`init`、`hessianInPlace`、`hessianAlloc`、`isHessianTopologyFixed`、`getDOFs(out)`。

## File Map

### 新增

- `src/core/nonlinearOptimization/energySet.h`
- `src/core/nonlinearOptimization/energySet.cpp`
- `src/core/nonlinearOptimization/evaluation.h`
- `src/core/nonlinearOptimization/evaluation.cpp`
- `tests/src/core/nonlinearOptimization/energySet_gtest.cpp`
- `tests/src/core/nonlinearOptimization/evaluation_gtest.cpp`
- `tests/src/core/genericPotentialEnergies/linearPotentialEnergy_ownership_gtest.cpp`
- `tests/src/core/genericPotentialEnergies/quadraticPotentialEnergy_ownership_gtest.cpp`
- `tests/src/core/constraintPotentialEnergies/multiVertexPulling_ownership_gtest.cpp`（见 Task E9）

### 修改

- `src/core/nonlinearOptimization/CMakeLists.txt`：编译新增源，移除已删除的 `potentialEnergies.cpp`。
- `src/core/genericPotentialEnergies/linearPotentialEnergy.h/.cpp`：reference 成员 → owning by value，ctor 改 by-value，`W` 类型从 `const double *` 改为 `EigenSupport::VXd`。
- `src/core/genericPotentialEnergies/quadraticPotentialEnergy.h/.cpp`：同上；保留 6 个 ctor 的算式语义。
- `src/core/constraintPotentialEnergies/multiVertexPullingSoftConstraints.h/.cpp`：raw pointer / `const SpMatD &` → owning by value；`setTargetPos` / `setCoeff` multi-overload 简化（见 Task E9）。
- `src/tools/sim/runIPCSim/setup/attachmentSetup.cpp`：ctor caller 迁移 (by value + move)；`setTargetPos` → `setTargetPositions`。
- `src/tools/sim/runIPCSim/solver/staticSolve.cpp`：`setTargetPos` → `setTargetPositions`；`PotentialEnergies` → `EnergySet` 同时把 `pullingEnergies` 作为 child 加入。
- `src/tools/sim/runIPCSim/app/loop.cpp`、`session.cpp`：`setTargetPos` → `setTargetPositions`。
- `src/tools/sim/runIPCSim/solver/staticSolve.cpp`：`PotentialEnergies` → `EnergySet`；`LinearPotentialEnergy(staticForce)` → `LinearPotentialEnergy(std::move(staticForce))`。
- `src/core/genericPotentialEnergies/laplacianProblem.cpp`：`PotentialEnergies` → `EnergySet`；`QuadraticPotentialEnergy(sys)` 保持（自动 copy `sys`）。
- `src/tools/sim/runIPCSim/contact/contactBackend.h`、`ipcContactBackend.cpp`、`legacyPenaltyContact.cpp`：引用参数 rename。
- `src/python/pypgo/bindings/energy_bindings.cpp`：绑定 `EnergySet`、`LinearPotentialEnergy`、`QuadraticPotentialEnergy`、`pypgo.energy.PotentialEnergy` handle。
- `pypgo/energy.py` (M3 module)：补 `EnergySet`、`LinearEnergy`、`QuadraticEnergy`、`VertexAttachment`（Task E9）、`state_kind` / `zero_state` 约束。
- `tests/pypgo/test_energy.py`（M3 已规划该文件位置）：补 EnergySet / Linear/Quadratic owning / VertexAttachment / state_kind 覆盖。

### 删除

- `src/core/nonlinearOptimization/potentialEnergies.h`
- `src/core/nonlinearOptimization/potentialEnergies.cpp`
- 所有 `using PotentialEnergies_p = ...` / forward decl 的 `class PotentialEnergies;` 同步清掉。

### 不动 / 后置

- `potentialEnergyFromConstraintFunctions` 不在 M3 绑定范围。
- `lineSearchAwareEnergy` 接口不变；`EnergySet` 继承它。
- `SmoothRSEnergy`：M9 之后单独评估。
- `ConstraintPotentialEnergies` 姊妹类（`MultiVertexConstrainedRigidMotion`、`BarycentricCoordinateSlidingSoftConstraints`、`MultipleVertexSlidingSoftConstraints`、`MultipleVertexPullingSoftConstraintsPOrder`）：M3 不绑定，列入 `future_work.md`。
- `ConstraintFunctions::hessian/createHessian/hessianDirect` 系列同名方法：跟 `PotentialEnergy` 平行的另一套抽象；M3 不在 E0 rename 范围内，等到 constraint API 进入 Python 时再做对称 rename，详见 `future_work.md`。

## Task 拆分

每个 task 必须以 C++ test 起步（characterization 或 unit test），通过后再加 Python binding 和 Python test。

### Task E0: Hessian API 跨代码库 rename

**必须在 E1 之前完成。** 这是一次性 mechanical refactor，要求一个系列 commit 内主干始终可编译。

- `src/core/nonlinearOptimization/potentialEnergy.h`：
  - `virtual void hessian(...)` → `virtual void hessianInPlace(...)`（pure）。
  - `virtual void hessianDirect(...)` → `virtual void hessian(...)`，base 默认实现改为 `hessianAlloc(H); hessianInPlace(x, H);`。
  - `virtual void createHessian(...)` → `virtual void hessianAlloc(...)`（pure）。
  - `gradient_hessian` 默认实现里的 `hessianDirect(x, hess)` 改为 `hessian(x, hess)`。
- 所有 `PotentialEnergy` subclass override（用 grep 列表锁定）：
  - `genericPotentialEnergies/linearPotentialEnergy.h`、`quadraticPotentialEnergy.h`
  - `geometryPotentialEnergies/*.h`（centerOfMass / smoothRS / vertexAffine / triangleAffine / surfaceSmoothness / surfaceTriangleDeformation 等）
  - `constraintPotentialEnergies/multiVertexConstrainedRigidMotion.h/.cpp`、`multiVertexPullingSoftConstraints.h/.cpp`、`potentialEnergyAligningMeshConnectivity.h/.cpp`
  - `contact/mappedSurfacePotentialEnergy.{h,cpp}`、`legacy_penalty/*.h`
  - `solidDeformationModel/deformationModelEnergy.{h,cpp}`
  - `nonlinearOptimization/potentialEnergies.{h,cpp}`（即将在 E3 中变成 `EnergySet`；本任务内先就地 rename）
  - `nonlinearOptimization/potentialEnergyFromConstraintFunctions.{h,cpp}`、`lagrangian.{h,cpp}`
- 所有 caller：
  - `nonlinearOptimization/NewtonSolver.cpp`：`createHessian` → `hessianAlloc`、`hessian(x, H)` → `hessianInPlace(x, H)`、`hessianDirect(x, H)` → `hessian(x, H)`。注意区分 Newton 热路径用的是 in-place 版本。
  - `nonlinearOptimization/finiteDifference.cpp`、`naturalCubicSplineFitting.cpp`、`knitroOptimizer.cpp`、`nonlinearProblem.cpp`、`constraintFunctionsAssember.cpp`：按调用语义对应 rename。
  - `simulation/implicitBackwardEulerTimeIntegratorHelper.{h,cpp}`、`TRBDF2TimeIntegratorHelper.{h,cpp}`、`timeIntegrator.cpp`：同上。
  - `tools/sim/runIPCSim/setup/{shell,volume,legacy}Setup.cpp`：按调用语义对应。
  - `python/pypgo/bindings/energy_bindings.cpp`：rename。
  - `c/pgo_c.cpp`：rename（虽然该文件已计划随 C-style wrapper 删除，但删除前主干必须可编译，所以本任务必须改）。
- 不改 `ConstraintFunctions::hessian/createHessian/hessianDirect` 系列：constraint API 跟 `PotentialEnergy` 的方法是平行的两套抽象，不在本 rename 范围内。如果需要保持对称命名，作为单独 follow-up task（详见 `future_work.md`）。
- 测试：
  - 改完后所有现有 `_gtest.cpp` 应只需文本替换即可通过（`createHessian` → `hessianAlloc`、`hessian(x,H)` 调用点视语义对应）；语义不变。
  - `runIPCSim` 端到端跑代表性 scene（small static、medium static、tet IPC 动态），与重命名前 git commit 数值结果 max-diff < 1e-12。
  - **Perf regression test**：以上 3 个 scene 的 wall-clock 与 baseline commit 对比，max regression < 5%；用来 catch B1 风险，见下面 commit 策略说明。
- Commit 策略：**原子 mechanical rename**，不用临时 forwarder。
  - 原因：旧 `hessian(x, H)` 和新 `hessian(x, H)` **签名完全相同但语义不同**（旧的假设 H 已建好、是 hot path 一行；新的是 `hessianAlloc + hessianInPlace`，每次都重新分配 sparsity）。如果在 commit 1 引入旧名 forwarder，caller 漏改时编译成功但每次 iter 都重新分配 sparse 模板，是 **silent perf regression** (5–10x slowdown)，没有任何 fail signal。
  - `override` keyword 能 catch subclass 漏改 `hessian(x,H)` / `createHessian(H)`（pure virtual 名字变了），但 caller 漏改 catch 不了。
  - 实施步骤：
    - Step 1：单个 sed-based pass 在 working tree 上 mechanical rename 全树：
      - `\bhessianDirect\b` → `hessian`
      - `\bcreateHessian\b` → `hessianAlloc`
      - 然后 base class header 内：旧 pure virtual `void hessian(ConstRefVecXd, SpMatD&) const = 0;` → `void hessianInPlace(ConstRefVecXd, SpMatD&) const = 0;`；以及旧 default `void hessian(...) const` body 改成调 `hessianAlloc + hessianInPlace`。
      - subclass header 内：旧 `void hessian(...) override` → `void hessianInPlace(...) override`。
      - subclass cpp 内：旧定义 `void XXXEnergy::hessian(...)` → `void XXXEnergy::hessianInPlace(...)`。
      - caller 调用点：旧的 hot-path `energy->hessian(x, H)` → `energy->hessianInPlace(x, H)`（手工 review 确认这是 hot-path 调用而非 one-shot）。
    - Step 2：编译；`override` keyword 会 catch 多数 subclass 漏改；不通过的逐个修。
    - Step 3：runIPCSim parity + perf regression 跑通后，单 commit 提交。
  - 单 commit 完成。中间不留过渡态，不允许 forwarder 名字残留。

### Task E1: 引入 `evaluation.h` helper

（依赖 Task E0 已完成；本 task 直接使用新名字。）

- 新增 `evaluation.h/.cpp`。
- 实现 `evaluateValue`、`evaluateGradient`、`evaluateHessian`、`evaluateMaxStep`、`dofsOf`。`validateStateSize` 为 `.cpp` 内部 helper，不在 header 公开。
- `evaluateHessian` 实现极简：调 `energy.hessian(x, H)`（新语义 = 旧 `hessianDirect`）即可——base 把 fixed / non-fixed 分支封装在 `hessian` 内（fixed 走默认 `hessianAlloc + hessianInPlace`，non-fixed 走 subclass override）。helper 内**不写** `isHessianTopologyFixed()` if 分支。
- 新增 `evaluation_gtest.cpp`：
  - 一个 fixed-topology fake energy（继承 `PotentialEnergy`，只实现 `hessianAlloc` / `hessianInPlace`，不 override `hessian`），验证 `evaluateHessian` 走基类默认 dispatch 返回正确结果；
  - 一个 non-fixed-topology fake energy（override `hessian` 且 `isHessianTopologyFixed() == 0`），验证 `evaluateHessian` 走 override 路径；
  - 两者输出的 dense 表达与手算一致。
- 现有 `PotentialEnergies` / 调用方暂不切换。

### Task E2: 把 `Linear/QuadraticPotentialEnergy` 改 owning

- 修改 `linearPotentialEnergy.h/.cpp`：
  - 成员 `const VXd &b` → `VXd b_`；ctor 改 `explicit LinearPotentialEnergy(VXd b)`，body 内 `b_(std::move(b))`。
  - 其余实现不动。
- 修改 `quadraticPotentialEnergy.h/.cpp`：
  - 成员 `const SpMatD &A` → `SpMatD A_`；`const VXd *b` → `std::optional<VXd> b_`；`const double *W`（仅出现在 ctor 参数）→ `std::optional<VXd> W_` 作为新成员。
  - 6 个 ctor 全部改 by value，签名调整：
    - `(SpMatD A)`
    - `(SpMatD A, int inParentheses)`
    - `(SpMatD A, VXd W, int inParentheses)`
    - `(SpMatD A, VXd b)`
    - `(SpMatD A, VXd b, int inParentheses)`
    - `(SpMatD A, VXd b, VXd W, int inParentheses)`
  - 算式语义不变；内部 `ATA_` / `bTA_` / `cache_` precompute 路径不变。
- 迁移 caller（共 2 个；第 3 个在 `pgo_c.cpp` 即将随 C-style wrapper 整体删除）：
  - `src/tools/sim/runIPCSim/solver/staticSolve.cpp:63`：`LinearPotentialEnergy(staticForce)` → `LinearPotentialEnergy(std::move(staticForce))`。
  - `src/core/genericPotentialEnergies/laplacianProblem.cpp:53`：`QuadraticPotentialEnergy(sys)` 不变（自动按 by-value 拷贝 `sys`）；如果需要避免拷贝可改 `QuadraticPotentialEnergy(SpMatD(sys))` 显式表达 copy 意图。
- 新增 ownership 测试：
  - `linearPotentialEnergy_ownership_gtest.cpp`：构造后立即让原 `VXd b_input` 出作用域（destructive scope test），`energy.func / gradient` 仍正确。
  - `quadraticPotentialEnergy_ownership_gtest.cpp`：同上，针对 A、b、W 三个数据成员分别测试。
  - 数值一致性对比：与 git 上重命名前的旧实现（保留一个临时 oracle program 在 reference commit）跑相同 `A`/`b`/`x`，比较 `func`/`gradient`/`hessian` max-diff < 1e-12。

### Task E3: 把 `PotentialEnergies` 改名为 `EnergySet` 并改造 ctor

- 新增 `energySet.h/.cpp`，把 `potentialEnergies.cpp` 的实现整体搬过来，类名改为 `EnergySet`，namespace 不变 (`pgo::NonlinearOptimization`)。
- ctor 签名改为 `EnergySet(int numDofs, std::vector<Term> terms)`，内部按 `terms` 顺序填充原 `potentialEnergies` 字段、`energyCoeffs` 字段，然后立即跑原 `init()` 的全部逻辑（不再公开 `init`）。
- 删除 `addPotentialEnergy` / `init` / `setEnergyCoeffs` 公共方法；改为只读访问 + `setWeight(i, w)`。
- `EnergySet::Term::energy` 是 `shared_ptr<const PotentialEnergy>`。内部 vector 字段 `potentialEnergies` 类型从 `PotentialEnergy_p`（non-const）改为 `PotentialEnergy_const_p`；如有非 const 接口调用需求，用 `const_cast` 限制在 ctor 实现内（数值代码本身只读）。
- 删除 `src/core/nonlinearOptimization/potentialEnergies.h/.cpp`。CMakeLists 同步更新。
- 不保留 `PotentialEnergies` typedef / alias。
- 新增 `energySet_gtest.cpp`：
  - 构造时立即可求值（无需用户调 `init`）。
  - 多 term，单 term，零 term 边界（零 term 抛 `std::invalid_argument`）。
  - 含 fixed-topology + non-fixed-topology child 混合时 `evaluateHessian(set, x)` 返回完整 Hessian。
  - `term(i).energy.use_count()` 在外部 `shared_ptr` 释放后仍 >= 1。
  - 与重命名前 `PotentialEnergies`（在 git checkout 前一个 commit）数值结果一致：用相同 children + weights，比较 value/gradient/hessian 在多组 random `x` 下的 max-diff。这一对比测试可单独写为一个 driver 程序保留若干 commit，不必作为长期 gtest。

### Task E3a: 迁移内部 `PotentialEnergies` 调用点

- 构造形态（`make_shared<PotentialEnergies>(n)` + add + init）改为 `make_shared<EnergySet>(n, std::vector<EnergySet::Term>{...})`：
  - `src/tools/sim/runIPCSim/solver/staticSolve.cpp:65`
  - `src/core/genericPotentialEnergies/laplacianProblem.cpp:55`
- 引用参数 (`PotentialEnergies &`) 改名为 `EnergySet &`：
  - `src/tools/sim/runIPCSim/contact/contactBackend.h:13,49`（forward decl + virtual method）
  - `src/tools/sim/runIPCSim/contact/ipcContactBackend.cpp:41`
  - `src/tools/sim/runIPCSim/contact/legacyPenaltyContact.cpp:150`
- 审计：上述虚函数实现内部目前都没有真正调用 `addPotentialEnergy` / `init` / `setEnergyCoeffs` mutating 路径；如审计发现存在 mutation，需在本 task 决定是否把 `setWeight` 之外的 mutator 重新提供给 backend，或重构 backend 不需要 mutation。
- `runIPCSim` 端到端：与重命名前比较一个静态 IPC scene 的最终 deform state（用 git stash + 跑两遍 + diff），确认数值一致。

### Task E3 + E3a Commit 序列

E3（新增 `EnergySet`、删除 `PotentialEnergies`）和 E3a（caller 迁移）共享同一 commit 序列，保证主干始终可编译：

- **Commit 1**：新增 `src/core/nonlinearOptimization/energySet.h/.cpp`，`PotentialEnergies.h/.cpp` 暂时保留共存。新增 `EnergySet_p` / `EnergySet_const_p` typedef 取代 `PotentialEnergies_p` 系列。CMakeLists 编译两份。
- **Commit 2**：迁移构造点（`staticSolve.cpp:65`、`laplacianProblem.cpp:55`）到 `EnergySet`。
- **Commit 3**：迁移引用参数 (`PotentialEnergies &` → `EnergySet &`) 的 3 处 caller (`contactBackend.h`、`ipcContactBackend.cpp`、`legacyPenaltyContact.cpp`)。
- **Commit 4**：删除 `potentialEnergies.h/.cpp`、`PotentialEnergies_p` typedef、所有 `class PotentialEnergies;` forward decl；CMakeLists 移除编译条目；`grep -rn "PotentialEnergies\b" src/` 验证 0 hit。
- 中间每个 commit 主干都可编译并跑通现有 gtest。

### Task E4: Python binding — `pypgo.energy.PotentialEnergy` handle

- 在 `energy_bindings.cpp` 暴露一个不可子类化的 `PotentialEnergy` Python 类型，内部持 `std::shared_ptr<const PotentialEnergy>`。
- 暴露只读属性 / 方法：
  - `num_dofs` → `getNumDOFs`
  - `dofs` → `dofsOf(...)` 返回 `np.ndarray[int64]`
  - `value(x)` / `gradient(x)` / `hessian(x)` / `max_step(x, dx)`：全部走 `evaluation.h`
  - `zero_state()` → 返回 `np.zeros(num_dofs, dtype=np.float64)`
- 不暴露 `func` / `func_grad` / `hessianInPlace` / `hessianAlloc` / `isHessianTopologyFixed`（拓扑分支由 `evaluation.h` 内部吸收，Python 用户无需感知）。`hessian(x, H)` 这个 C++ 安全入口本身也不直接暴露给 Python，因为 Python 需要 by-value 返回而不是 out-param——用户走的是 `pypgo.energy.PotentialEnergy.hessian(x) -> SparseMatrix`，内部调 `evaluateHessian`。
- 新增 `tests/pypgo/test_energy.py::test_potential_energy_handle_basic_methods`，用 `QuadraticPotentialEnergy`（E2 改造后的 owning 版本）作为最小可绑定 energy 验证。
- Python energy 类加 `__repr__`：格式 `"ClassName({num_dofs} DOFs)"`，EnergySet 格式 `"EnergySet({n} terms, {num_dofs} DOFs, state_kind='{kind}')"`。

### Task E5: Python binding — `LinearEnergy` / `QuadraticEnergy`

- 绑 `PredefinedPotentialEnergies::LinearPotentialEnergy` 为 `pypgo.energy.LinearEnergy(b)`，接受 NumPy `(n,) float64`，binding 把 NumPy data 转成 `EigenSupport::VXd` 后 `std::move` 进 ctor。
- 绑 `PredefinedPotentialEnergies::QuadraticPotentialEnergy` 为 `pypgo.energy.QuadraticEnergy(A, b=None)`。`A` 输入 dispatch 顺序：
  1. 若 M2 `pypgo.sparse.SparseMatrix` 已可用：接受该类型；
  2. 兜底：接受 `(rows, cols, vals, shape)` 四元组（NumPy `int64`/`int64`/`float64` + `(int, int)`），binding 内组装成 Eigen sparse；
  3. 若 SciPy 可选依赖存在：通过 `to_coo` adapter 走 (2)。
  这一兜底路径只在 binding 层做，C++ ctor 始终接受 `EigenSupport::SpMatD`。
- 不暴露 `setDOFs(const std::vector<int> &dofs)`（partial-DOF Linear/Quadratic energy）：M3 只支持全 DOF energy；`allDOFs` 在 ctor 内自动填充。setDOFs 的使用场景（Knitro/constraint 问题用 partial DOF）不在 M3 scope。
- Python 端约定 `state_kind == "generic"`。
- 测试：构造 → 立即释放 Python 引用的输入数组 → 仍能求值；以及 (rows, cols, vals, shape) 兜底路径数值与 (1) 路径一致。

### Task E6: Python binding — `EnergySet`

- 绑 `EnergySet` 为 `pypgo.energy.EnergySet`，接受 `list[tuple[PotentialEnergy, float]]` 或 `list[PotentialEnergy]`（默认 weight 1.0）。
- 必须显式传入 `num_dofs`，或允许通过第一个 term 的 `num_dofs` 推断；推断不一致时抛 `ValueError`。
- 暴露 `num_terms`、`term(i)`（返回原 Python energy 对象 + weight）、`set_weight(i, w)`。
- 测试：
  - 默认 weight 1.0；
  - `set_weight` 影响后续 `value`；
  - 混合 fixed/non-fixed topology child 时 `hessian(u)` 形状和 nnz 正确；
  - Python 端 `elastic = ...; total = EnergySet([elastic]); del elastic; total.value(u)` 仍可用；
  - C++ ctor `throw std::invalid_argument`（零 term、DOF 数不一致）正确 propagate 为 Python `ValueError`。

### Task E7: `state_kind` / `zero_state` 公约

- 在 `pypgo/energy.py` 给每个具体 energy class 写 `state_kind` 常量。
- `EnergySet.state_kind` 规则：
  - 空 terms 不允许构造（ctor 内抛 `ValueError`）；
  - 单 term → 沿用该 term；
  - 多 term 全部相同 → 沿用之；
  - 否则 → `"generic"`。
- 测试：`deformation_energy(...).state_kind == "displacement"`、`LinearEnergy(...).state_kind == "generic"`、`EnergySet([elastic, floor]).state_kind == "displacement"`、`EnergySet([elastic, quadratic]).state_kind == "generic"`、`EnergySet([])` raises `ValueError`。

### Task E9: `MultipleVertexPulling` 改 owning + Python `VertexAttachment`

- 修改 `src/core/constraintPotentialEnergies/multiVertexPullingSoftConstraints.h/.cpp`：
  - 成员 `const SpMatD &Koff`（来自基类 `PotentialEnergyAligningMeshConnectivity`）→ owning `SpMatD Koff_` + 传递给基类（需审计基类接口）。
  - `const double *restPositionsAll` → `VXd restpAll_`。
  - `const int *vertexIndices` → `std::vector<int> vertexIndices_`。
  - `const double *tgt` → `VXd tgtp_`。
  - `const double *bcCoeff` → scalar `coeffAll_`（删 `setCoeff(const double *v)` 和 `setMasks(const double *v)` 的 per-vertex overload）。
  - `int isDisp` flag → `bool isDisplacement_`。
  - ctor 改 by value + move。
  - `setTargetPos(const double *tgt)` → `setTargetPositions(VXd tgt)`。
  - `hessianInPlace` / `hessianAlloc` override rename 与 E0 一起完成。
- 迁移 caller（共 5 处）：
  - `src/tools/sim/runIPCSim/setup/attachmentSetup.cpp:42`：ctor 改为 by value + move 传入 copy 后的 `Koff` / `restPositionsAll` / `vertexIndices` / `tgtPositions`。`setCoeff(scalar)` 保持不变。
  - `src/tools/sim/runIPCSim/solver/staticSolve.cpp:28`：`setTargetPos` → `setTargetPositions`。
  - `src/tools/sim/runIPCSim/app/loop.cpp:27`：同上。
  - `src/tools/sim/runIPCSim/app/session.cpp:34`：`addImplicitForceModel` caller——审计该调用是否需要 `const PotentialEnergy *` 的 const 转换（当前 pulling energy 通过 `shared_ptr<MultipleVertexPulling>` 传入，E0 后需确认 `addImplicitForceModel` 接受 `shared_ptr<const PotentialEnergy>`）。
  - `src/tools/sim/runIPCSim/solver/staticSolve.cpp:68`：E3a 迁移 `PotentialEnergies` → `EnergySet` 时 pullingEnergies 作为 child 加入。
- 新增 ownership 测试：`multiVertexPulling_ownership_gtest.cpp`：
  - 构造后立即让传入的 `VXd restPositionsAll` / `VXd tgt` / `SpMatD Koff` / `vector<int> vertexIndices` 出作用域 → `func` / `gradient` 仍正确。
  - 数值一致性：与 git 上重命名前的旧实现跑相同 fixture，`func` / `gradient` max-diff < 1e-12。
- Python binding：
  - 绑为 `pypgo.energy.VertexAttachment`。
  - ctor 接 `sim_mesh`（从 `SimulationMeshCore` 取 `Koff` 和 `numDofs`）或 `Koff` 兜底（`(rows, cols, vals, shape)` 或 `pypgo.sparse.SparseMatrix`，优先 sim_mesh）。
  - 暴露 `set_targets(np.ndarray)`。
  - `state_kind == "displacement"`。
  - 测试：构造 → 求值 → `set_targets` 更新 target → 求值变化；与 EnergySet 组合。
- Cross-plan dependency：
  - `deformation_fem_api_refactor.plan.md`：确保 `SimulationMeshCore` 暴露 `hessianSparsityTemplate()` 方法（若未隐含）。如该 plan 进度滞后，M3 VertexAttachment binding 临时走 Koff 兜底路径。

### Task E8: 清理 / 文档

- 在 `api_coverage.md` 把 "多能量组合" 一行从 `PotentialEnergies` 更新到 `EnergySet`，并把 `Linear/Quadratic energy` 加入覆盖矩阵（标 M3）。
- 在 `numpy_data_contract.md` 补 `state_kind` / `zero_state` 约定，以及 `EnergySet` 构造时输入 list 的 dtype 规则。
- 不写 release notes：M3 是首次公开 binding。

## 验收标准

- C++ 测试全部通过；旧 `PotentialEnergies` 数值结果与 `EnergySet` 在相同 children + weights 下一致（同一 fixture 跑两遍）。
- `python -m pytest tests/pypgo/test_energy.py` 全部通过，覆盖：handle、Linear/Quadratic owning、EnergySet 单/多/0 term、weight 更新、混合 topology、child lifetime、state_kind。
- `pypgo.energy` 公开名集合不出现 `PotentialEnergies`、`addPotentialEnergy`、`init`、`hessianInPlace`、`hessianAlloc`、`isHessianTopologyFixed`。
- C++ 全树 `grep -rn "hessianDirect\|createHessian" src/` 不再有 hit。
- 没有 `std::shared_ptr<void>` 出现在 `energy_bindings.cpp` 或 `nonlinearOptimization` 公共 header。
- C++ 代码全树 `grep -rn "PotentialEnergies\b" src/` 不再有 hit（只剩 `PredefinedPotentialEnergies` / `ConstraintPotentialEnergies` 这类 namespace 名字属于同名前缀）。
- `runIPCSim` 静态/动态端到端数值结果与重构前一致（Task E3a 的 stash + diff 验证通过）。
- `runIPCSim` 3 组代表性 scene（small static、medium static、tet IPC 动态）wall-clock 与 baseline commit 对比，max regression < 5%。

## Dependencies & Execution Order

### 外部依赖

- 不依赖 contact / solver plan 的任何任务。
- 不依赖 M2 `pypgo.sparse.SparseMatrix` 已完成：Task E5 提供 `(rows, cols, vals, shape)` 兜底路径。
- `pypgo._core` 框架（M1）必须可用。

### 内部任务依赖

```text
E0 (Hessian API rename)        ── 必须最先；跨 ~40 个文件 mechanical refactor，
                                  完成后所有后续 task 用新名字
                          │
E1 (evaluation.h)              ─┐
E2 (Linear/Quadratic 改 owning) ─┤          ┐
E9 (MultipleVertexPulling)     ──┤ 依赖 E0；三者可并行（ownership 改法一致）
                          │     │          │
E3 (EnergySet rename + ctor 改造) ◄┘          ┤  依赖 E0 + E1（child 求值），E2/E9 仅在测试时用作 child 样本
                          │                │
E3a (内部调用点迁移)      ── 必须与 E3 同系列 commit，保证主干编译
                          │                │
E4 (PotentialEnergy 绑定)─┤── 依赖 E1 + E3a 已完成（C++ 主干稳定），需要 owning energy（E2/E9）做测试样本
                          │                │
E5 (Linear/Quadratic 绑定)── 依赖 E2、E4
E9b (VertexAttachment 绑定)── 依赖 E9、E4（与 E5 可并行）
                          │                │
E6 (EnergySet 绑定)       ── 依赖 E3、E4、E5、E9b
                          │
E7 (state_kind)          ── 依赖 E5、E6、E9b
                          │
E8 (文档)                ── 最后
```

### 推荐顺序

E0 → E1 ‖ E2 ‖ E9（并行）→ E3 → E3a → E4 → E5 ‖ E9b（并行）→ E6 → E7 → E8。

E0 必须最先完成；E1 / E2 / E9 可三路并行（ownership 改造是一样的 pattern）；E3 和 E3a 同一系列 commit；E5 与 E9b 可并行；其余串行。

### 输出（供下游 plan 使用）

- C++: `EnergySet`、`evaluateValue/Gradient/Hessian/MaxStep` helper、owning 化的 `Linear/QuadraticPotentialEnergy`、owning 化的 `MultipleVertexPulling`。
- Python: `pypgo.energy.PotentialEnergy`（handle）、`EnergySet`、`LinearEnergy`、`QuadraticEnergy`、`VertexAttachment`。
- Contact plan 使用 `pypgo.energy.PotentialEnergy` 作为绑定基类、`EnergySet` 做端到端测试。
- Solver plan 使用 `pypgo.energy.PotentialEnergy` 作为输入类型、`evaluation.h` 做 Hessian 求值。
