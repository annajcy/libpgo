# Energy API Refactor Plan

> **状态日期：** 2026-05-28
> **适用范围：** C++ `nonlinearOptimization` energy 边界重构 + Python `pypgo.energy` binding。
> **执行约束：** 不重写数值内核（`func` / `gradient` / `hessian` 的内部计算保持不变）；本计划只重构 energy 的 ownership、组合、生命周期、求值边界，让 Python binding 不需要在外面打补丁。

## 目标

把 `PotentialEnergy` 周围的“可绑定边界”收干净，让 `pypgo.energy` 的 Python API 能写成：

```python
import pypgo as pgo

elastic = pgo.energy.deformation_energy(...)
force   = pgo.energy.external_force(f)

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
- `hessian()` 与 `hessianDirect()` 的拓扑固定/非固定区分；
- 任何 `shared_ptr<void>` / `keepAlive_` 形式的“binding 层补洞” ownership；
- `getDOFs(out)` 这类 C 风格 out-param 接口；
- `LinearPotentialEnergy` / `QuadraticPotentialEnergy` 持有的外部 `const VXd &` / `const SpMatD &` 引用。

## 当前问题

相关文件：

- `src/core/nonlinearOptimization/potentialEnergy.h`
- `src/core/nonlinearOptimization/potentialEnergies.h`
- `src/core/nonlinearOptimization/potentialEnergies.cpp`
- `src/core/nonlinearOptimization/linearPotentialEnergy.h`（如存在；目前定义在 quadratic 兄弟文件内）
- `src/core/nonlinearOptimization/quadraticPotentialEnergy.h`
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

## 非目标

- 不重写 `func` / `gradient` / `hessian` 的数值实现；只动 ownership、组合、求值 facade。
- 不引入 Python 子类化 `PotentialEnergy` 的 trampoline。Python-defined energy 列入 `future_work.md`，本计划不涉及。
- 不重构 `EnergyOptimizer::minimize` / IPOPT / Knitro / constraints 入口；solver 部分见 `solver_api_refactor.plan.md`。
- 不为旧 `PotentialEnergies` 公开 API 做向后兼容；M3 是首次公开 binding，没有调用方需要保护，可以直接破坏式迁移到 `EnergySet`。
- 不为 `SmoothRSEnergy` (MKL-gated) 提前规划 Python 表面；该 energy 在 M9 之后再决定。
- 不在本计划解决 contact energy 的 ownership 细节，那部分留给 `contact_api_refactor.plan.md`；本计划只保证 contact energy 能以 `shared_ptr<const PotentialEnergy>` 形态被组合进 `EnergySet`。

## 设计决策

### 1. `PotentialEnergies` → `EnergySet`

把 `PotentialEnergies` 重命名为 `EnergySet`，让它**自己**就是组合 energy；不再分两阶段构造、不再暴露 `addPotentialEnergy` / `init`。

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
};
}  // namespace pgo::NonlinearOptimization
```

构造内部完成的工作：

- 保存 `terms_`（owning `shared_ptr<const PotentialEnergy>`）。
- 用现有 `PotentialEnergies::init()` 的逻辑建 `hessianAll` 模板、`hessianMatrixMappings`、`allDOFs`、`isQuadraticEnergy`、`hasHessianVectorProduct`。
- `isHessianTopologyFixed()` 由 children 决定：任一 child 不固定就不固定。
- `hessianDirect` 内部根据 children 拓扑性质走当前 mapping / `createHessian` + accumulate 路径。

`PotentialEnergies` 仍可在 transition 内作为 internal alias 存在（实现复用），但 public header 只暴露 `EnergySet`。`runIPCSim` / 内部代码原本组合 energy 的位置改用 `EnergySet`。

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

### 3. 通用 energy：提供 owning 版本

新增两个 owning wrapper：

```cpp
namespace pgo::NonlinearOptimization
{
class OwnedLinearPotentialEnergy : public PotentialEnergy
{
public:
  explicit OwnedLinearPotentialEnergy(EigenSupport::VXd b);
  // implements PotentialEnergy by delegating to internal LinearPotentialEnergy
private:
  EigenSupport::VXd b_;
  LinearPotentialEnergy impl_;  // references b_
};

class OwnedQuadraticPotentialEnergy : public PotentialEnergy
{
public:
  OwnedQuadraticPotentialEnergy(
    EigenSupport::SpMatD A,
    std::optional<EigenSupport::VXd> b = std::nullopt);
private:
  EigenSupport::SpMatD A_;
  std::optional<EigenSupport::VXd> b_;
  QuadraticPotentialEnergy impl_;
};
}
```

旧 `LinearPotentialEnergy` / `QuadraticPotentialEnergy` 保留给内部、非 Python 的 hot path；Python binding **只**绑 owning 版本。

不直接重构旧 `Linear/QuadraticPotentialEnergy` 持有引用的语义，避免动到现存调用方。

### 4. 求值 facade：`evaluation.h`

提供一组无状态 helper，包掉 fixed/non-fixed Hessian 拓扑分支、out-param、NumPy 转换需要的 by-value 返回：

```cpp
namespace pgo::NonlinearOptimization
{
void validateStateSize(const PotentialEnergy &energy, EigenSupport::ConstRefVecXd x);

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

`evaluateHessian` 内部统一：

```cpp
EigenSupport::SpMatD H;
if (energy.isHessianTopologyFixed()) {
  energy.createHessian(H);
  energy.hessian(x, H);
} else {
  energy.hessianDirect(x, H);
}
return H;
```

Python binding 一律走这层 helper；不直接调 `PotentialEnergy::hessian`。

### 5. Python-facing handle 是 `shared_ptr<const PotentialEnergy>`

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

### 6. State convention 在 Python wrapper 层声明

C++ base 类不强制区分 displacement / generic vector。Python wrapper 暴露一个 metadata 属性：

```python
energy.state_kind  # "displacement" or "generic"
energy.zero_state()
```

- `deformation_energy` / contact / `external_force`：`state_kind == "displacement"`。
- `linear` / `quadratic`：`state_kind == "generic"`。

`zero_state()` 返回 `np.zeros(num_dofs, dtype=np.float64)`。

C++ 这层不维护 enum；只在 Python wrapper class 上写常量。

## 目标 C++ API

```cpp
// nonlinearOptimization/potentialEnergy.h          (unchanged interface)
// nonlinearOptimization/energySet.h                (new, replaces public PotentialEnergies)
// nonlinearOptimization/evaluation.h               (new)
// nonlinearOptimization/ownedGenericEnergies.h     (new)

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

  // PotentialEnergy interface (delegates to existing PotentialEnergies impl).
};

class OwnedLinearPotentialEnergy : public PotentialEnergy {
public:
  explicit OwnedLinearPotentialEnergy(EigenSupport::VXd b);
};

class OwnedQuadraticPotentialEnergy : public PotentialEnergy {
public:
  OwnedQuadraticPotentialEnergy(
    EigenSupport::SpMatD A,
    std::optional<EigenSupport::VXd> b = std::nullopt);
};

void validateStateSize(const PotentialEnergy &energy, EigenSupport::ConstRefVecXd x);
double evaluateValue(const PotentialEnergy &energy, EigenSupport::ConstRefVecXd x);
EigenSupport::VXd evaluateGradient(const PotentialEnergy &energy, EigenSupport::ConstRefVecXd x);
EigenSupport::SpMatD evaluateHessian(const PotentialEnergy &energy, EigenSupport::ConstRefVecXd x);
MaxStepResult evaluateMaxStep(const PotentialEnergy &energy, EigenSupport::ConstRefVecXd x, EigenSupport::ConstRefVecXd dx);
std::vector<int> dofsOf(const PotentialEnergy &energy);

}  // namespace pgo::NonlinearOptimization
```

旧 `PotentialEnergies` 在 transition 期可作为 private implementation type 保留，但不再出现在 public header。`addPotentialEnergy` / `init` 不允许出现在 binding 表面。

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
  deformation_energy(...)   # factory, returns DeformationEnergy
  external_force(b)         # convenience alias for LinearEnergy(-b)
```

不暴露：`PotentialEnergies`、`addPotentialEnergy`、`init`、`hessianDirect`、`createHessian`、`getDOFs(out)`。

## File Map

### 新增

- `src/core/nonlinearOptimization/energySet.h`
- `src/core/nonlinearOptimization/energySet.cpp`
- `src/core/nonlinearOptimization/evaluation.h`
- `src/core/nonlinearOptimization/evaluation.cpp`
- `src/core/nonlinearOptimization/ownedGenericEnergies.h`
- `src/core/nonlinearOptimization/ownedGenericEnergies.cpp`
- `tests/src/core/nonlinearOptimization/energySet_gtest.cpp`
- `tests/src/core/nonlinearOptimization/evaluation_gtest.cpp`
- `tests/src/core/nonlinearOptimization/ownedGenericEnergies_gtest.cpp`
- `tests/pypgo/test_energy.py`（M3 已规划，本计划补 EnergySet / owning / state_kind 覆盖）

### 修改

- `src/core/nonlinearOptimization/CMakeLists.txt`：编译新增源。
- `src/core/nonlinearOptimization/potentialEnergies.{h,cpp}`：保留作为 `EnergySet` 的实现细节；从公共 include 表中移除（或标记 internal）。
- `src/python/pypgo/bindings/energy_bindings.cpp`：绑定 `EnergySet`、owning energies、`pypgo.energy.PotentialEnergy` handle。
- `pypgo/energy.py` (M3 module)：补 `EnergySet`、`LinearEnergy`、`QuadraticEnergy`、`external_force`、`state_kind` / `zero_state` 约束。

### 不动 / 后置

- `linearPotentialEnergy` / `quadraticPotentialEnergy` 内部实现保持不变。
- `potentialEnergyFromConstraintFunctions` 不在 M3 绑定范围。
- `lineSearchAwareEnergy` 接口不变；`EnergySet` 继承它。
- `SmoothRSEnergy`：M9 之后单独评估。

## Task 拆分

每个 task 必须以 C++ test 起步（characterization 或 unit test），通过后再加 Python binding 和 Python test。

### Task E1: 引入 `evaluation.h` helper

- 新增 `evaluation.h/.cpp`。
- 实现 `validateStateSize`、`evaluateValue`、`evaluateGradient`、`evaluateHessian`、`evaluateMaxStep`、`dofsOf`。
- `evaluateHessian` 必须覆盖 `isHessianTopologyFixed() == 1` 和 `== 0` 两条路径。
- 新增 `evaluation_gtest.cpp`：用一个 fixed-topology fake energy + 一个 non-fixed-topology fake energy 验证两个分支都返回相同的 dense 表达。
- 现有 `PotentialEnergies` / 调用方暂不切换。

### Task E2: 新增 owning generic energies

- 实现 `OwnedLinearPotentialEnergy` / `OwnedQuadraticPotentialEnergy`：内部组合现有 `LinearPotentialEnergy` / `QuadraticPotentialEnergy` 并 own 数据 buffer。
- 测试：与现有 `LinearPotentialEnergy` / `QuadraticPotentialEnergy` 做 `func` / `gradient` / `hessian` 数值一致性比对（同样的 `A`、`b`、`x` 输入）。
- 测试：构造后立即释放原始 `Eigen::VXd b_input`，仍能正确求值（验证 ownership）。

### Task E3: 重构 `PotentialEnergies` → `EnergySet`

- 新增 `energySet.h/.cpp`。
- `EnergySet` 构造接受 `(int numDofs, std::vector<Term> terms)`，内部完成原 `PotentialEnergies::init()` 的全部工作。
- 实现可走两条路：
  1. `EnergySet` 内部组合一个 `PotentialEnergies` 实例并在 ctor 内 `add + init`；或
  2. 直接把 `potentialEnergies.cpp` 的实现搬过来。
  第一版用方案 1，避免动数值代码。
- `setWeight` 透传到内部 `setEnergyCoeffs`。
- 新增 `energySet_gtest.cpp`：
  - 构造时立即可求值（无需用户调 `init`）。
  - 多 term，单 term，零 term 边界。
  - 含 fixed-topology + non-fixed-topology child 混合时 `evaluateHessian(set, x)` 返回完整 Hessian。
  - `term(i).energy.use_count()` 在外部 `shared_ptr` 释放后仍 >= 1。
- `runIPCSim` 等内部调用点本计划不迁移；它们继续用 `PotentialEnergies` 直到独立 cleanup。

### Task E4: Python binding — `pypgo.energy.PotentialEnergy` handle

- 在 `energy_bindings.cpp` 暴露一个不可子类化的 `PotentialEnergy` Python 类型，内部持 `std::shared_ptr<const PotentialEnergy>`。
- 暴露只读属性 / 方法：
  - `num_dofs` → `getNumDOFs`
  - `dofs` → `dofsOf(...)` 返回 `np.ndarray[int64]`
  - `is_hessian_topology_fixed`
  - `value(x)` / `gradient(x)` / `hessian(x)` / `max_step(x, dx)`：全部走 `evaluation.h`
  - `zero_state()` → 返回 `np.zeros(num_dofs, dtype=np.float64)`
- 不暴露 `func` / `func_grad` / `createHessian` / `hessianDirect`。
- 新增 `tests/pypgo/test_energy.py::test_potential_energy_handle_basic_methods`，用 `OwnedQuadraticEnergy` 作为最小可绑定 energy 验证。

### Task E5: Python binding — `LinearEnergy` / `QuadraticEnergy`

- 绑 `OwnedLinearPotentialEnergy` 为 `pypgo.energy.LinearEnergy(b)`。
- 绑 `OwnedQuadraticPotentialEnergy` 为 `pypgo.energy.QuadraticEnergy(A, b=None)`，`A` 接受 `pypgo.sparse.SparseMatrix` 或 SciPy CSR / COO（通过 M2 转换辅助）。
- 提供 `pypgo.energy.external_force(b)` 别名：等价于 `LinearEnergy(-b)`。
- Python 端约定 `state_kind == "generic"`。
- 测试：构造 → 立即释放 Python 引用的输入数组 → 仍能求值。

### Task E6: Python binding — `EnergySet`

- 绑 `EnergySet` 为 `pypgo.energy.EnergySet`，接受 `list[tuple[PotentialEnergy, float]]` 或 `list[PotentialEnergy]`（默认 weight 1.0）。
- 必须显式传入 `num_dofs`，或允许通过第一个 term 的 `num_dofs` 推断；推断不一致时抛 `ValueError`。
- 暴露 `num_terms`、`term(i)`（返回原 Python energy 对象 + weight）、`set_weight(i, w)`。
- 测试：
  - 默认 weight 1.0；
  - `set_weight` 影响后续 `value`；
  - 混合 fixed/non-fixed topology child 时 `hessian(u)` 形状和 nnz 正确；
  - Python 端 `elastic = ...; total = EnergySet([elastic]); del elastic; total.value(u)` 仍可用。

### Task E7: `state_kind` / `zero_state` 公约

- 在 `pypgo/energy.py` 给每个具体 energy class 写 `state_kind` 常量。
- `EnergySet.state_kind` 规则：所有 child `state_kind` 相同则采用之；否则 `"generic"`。
- 测试：`deformation_energy(...).state_kind == "displacement"`、`LinearEnergy(...).state_kind == "generic"`、`EnergySet([elastic, floor]).state_kind == "displacement"`、`EnergySet([elastic, quadratic]).state_kind == "generic"`。

### Task E8: 清理 / 文档

- 在 `api_coverage.md` 把 "多能量组合" 一行从 `PotentialEnergies` 更新到 `EnergySet`，并把 `Linear/Quadratic energy` 加入覆盖矩阵（标 M3）。
- 在 `numpy_data_contract.md` 补 `state_kind` / `zero_state` 约定，以及 `EnergySet` 构造时输入 list 的 dtype 规则。
- 不写 release notes：M3 是首次公开 binding。

## 验收标准

- C++ 测试全部通过；旧 `PotentialEnergies` 数值结果与 `EnergySet` 在相同 children + weights 下一致（同一 fixture 跑两遍）。
- `python -m pytest tests/pypgo/test_energy.py` 全部通过，覆盖：handle、Linear/Quadratic owning、EnergySet 单/多/0 term、weight 更新、混合 topology、child lifetime、state_kind。
- `pypgo.energy` 公开名集合不出现 `PotentialEnergies`、`addPotentialEnergy`、`init`、`hessianDirect`、`createHessian`。
- 没有 `std::shared_ptr<void>` 出现在 `energy_bindings.cpp` 或 `nonlinearOptimization` 公共 header。
- `runIPCSim` 既有行为不受影响（内部仍可用 `PotentialEnergies` 直到独立 cleanup task 拆掉）。
