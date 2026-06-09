# pypgo Handle Architecture Refactor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [x]`) syntax for tracking.

**Goal:** 统一 `pypgo` Python facade 与 C++ binding 的对象模型，使每个 Python facade 的 `_handle` 都保存对应的 concrete `PyXXXX` C++ peer，并让 binding 注册文件只声明绑定，不承载具体实现。

**Architecture:** `PyXXXX` peers 保留真实继承体系：例如 `PyNewtonOptimizer : PyOptimizer`、`PyEnergySet : PyPotentialEnergy`、`PyStatefulContactEnergy : PyPotentialEnergy`。Python 侧直接通过 `_handle` 使用继承接口；不再提供 Python 侧 `as_potential_energy()` / `_potential_handle` / `as_optimizer()` 空上行转换。C++ 内部仍保留有意义的桥接方法，例如 `PyPotentialEnergy::potentialEnergyHandle()` 返回 core `std::shared_ptr<const NonlinearOptimization::PotentialEnergy>`，`PyOptimizer::asOptimizer()` 返回 core `Optimization::Optimizer&`。

**Tech Stack:** Python, C++17, nanobind, CMake, pytest, libpgo `nonlinearOptimization` / `simulation` / `contact` core modules.

---

## 设计约定

### Handle 语义

`_handle` 只能表示 concrete C++ peer：

```python
linear._handle               # _core.PyOwnedPotentialEnergy
energy_set._handle           # _core.PyEnergySet
deformation_energy._handle   # _core.PyDeformationEnergy
optimizer._handle            # _core.PyNewtonOptimizer
problem._handle              # _core.PyOptimizationProblem
```

因为 concrete peers 继承 abstract peers，所以 Python 消费方直接使用 `_handle`：

```python
problem = _core.PyOptimizationProblem(objective._handle)
data = optimizer._handle.solve(problem._handle, x0_arr)
simulation._handle.step(force, fixed_arr, has_fixed, optimizer._handle)
```

不要在 Python 侧新增这些空适配：

```python
energy._handle.as_potential_energy()
energy._potential_handle
optimizer._handle.as_optimizer()
```

### C++ 内部桥接

C++ peer 可以提供内部桥接方法，但这些方法服务于 core API，不是 Python facade 的常规路径：

```cpp
class PyPotentialEnergy {
public:
  virtual std::shared_ptr<const NO::PotentialEnergy> potentialEnergyHandle() const = 0;
};

class PyOptimizer {
public:
  virtual NOO::Optimizer &asOptimizer() = 0;
};
```

`potentialEnergyHandle()` 用于 `OptimizationProblem.objective`、`EnergySet::Term.energy`、dynamic simulation persistent terms。`asOptimizer()` 用于 `DynamicStepper::step(..., Optimizer&)`。

### Concrete Peer 规则

`_handle` 必须是 instantiable 的 concrete peer（不是抽象 base）。是否为某个 facade 单独建一个 `PyXXXX` 类，按**行为驱动**决定，而不是机械地一对一：

- 有类型特有行为、mutable 操作或额外 metadata 的 facade，建专属 typed peer，并保存具体 C++ 类型，避免 `dynamic_cast` / `const_cast`：`PyVertexAttachmentEnergy`（`setTargetPositions`）、`PyStatefulContactEnergy` 系列（`beginStep` / `setMovingObstacleTime`）、`PyEnergySet`（`numTerms` / `setWeight`）、`PyDeformationEnergy`（plastic 接口）。
- 没有类型特有行为的 generic energy（`LinearEnergy`、`QuadraticEnergy`、`ConstraintPenalty`、`ConstraintViolationPenalty`）共用 `PyOwnedPotentialEnergy`，不为每个再造空标签类。Python facade 自身已区分类型，C++ 侧没有需要区分它们的边界。

```cpp
class PyOwnedPotentialEnergy final : public PyPotentialEnergy {
  std::shared_ptr<const NO::PotentialEnergy> energy_;  // generic，无类型特有行为
};

class PyVertexAttachmentEnergy final : public PyPotentialEnergy {
  std::shared_ptr<ConstraintPotentialEnergies::MultipleVertexPulling> energy_;  // typed
  void setTargetPositions(nb::ndarray<nb::numpy, const double> targetPositions);
};
```

### Binding 零实现规则

C++ binding 注册文件不得实现业务 wrapper class，不得包含算法/helper 实现，也不得在 `.def(...)` / `.def_prop_*` / `m.def(...)` 中写 lambda。实现放入对应 `core.h` / `core.cpp`、core library，或 `PyXXXX` 成员函数。

允许的 binding 形态：

```cpp
.def("matvec", &PySparseMatrix::matvec)
m.def("create_sparse_matrix", &PySparseMatrix::create)
```

不允许的 binding 形态：

```cpp
.def("matvec", [](const PySparseMatrix &A, nb::ndarray<nb::numpy, const double> x) {
  auto xMap = pgo::python::ndarrayToVectorMapXd(x);
  ...
})
```

这条规则包括 trivial lambda，例如 module `build_info`、capability flag、enum/string conversion、property getter/setter lambda。统一挪成 named function，保留可 grep 的硬规则。

### 命名规则

Python-visible C++ peer 名统一使用 `Py` 前缀：

```text
_core.PyPotentialEnergy
_core.PyConstraintFunctions
_core.PyOptimizer
_core.PySparseMatrix
```

迁移时必须把旧 `_core.PotentialEnergy` / `_core.ConstraintFunctions` 引用纳入 grep 清单。

### Bounds 单一真相源

`OptimizationProblem.variable_bounds` 可以继续作为 Python API 存在，但 `Optimizer.solve()` 前必须把 Python `variable_bounds` 同步到 C++ `_handle`。这样用户直接赋值：

```python
problem.variable_bounds = Bounds(lower=lower, upper=upper)
```

不会静默失效。

### Optimizer 配置（构造期定死）

`NewtonOptimizer` 等 optimizer facade 不在 Python 侧保存可变 options。构造时校验参数、一次性构建 `_core.PyNewtonOptimizer` 存入 `_handle`，此后 `_handle` 即唯一真相源并保持稳定（不在每次 solve 重建）。不提供构造后修改 options 的接口；要换参数就新建一个 optimizer。

这与 bounds 的差异是有意的：bounds 是用户在 Python 侧逐步装配的 problem 输入（`fix_variables` 含实际数组逻辑），所以保留 Python authoring surface + solve 前同步；optimizer options 只是直传 C++ 的标量配置，没有 Python 侧逻辑，因此直接以 C++ peer 为唯一真相源，不做 Python 侧镜像。

---

## 文件结构

新增文件：

- `src/python/pypgo/bindings/solver/core.h`: `PyOptimizationProblem`、`PyOptimizer`、`PyNewtonOptimizerOptions`、`PyNewtonOptimizer`、solver result dict 转换。
- `src/python/pypgo/bindings/solver/core.cpp`: solver peer 实现。
- `src/python/pypgo/bindings/energy/peer.h`: `PyPotentialEnergy` base 与 energy concrete peers。
- `src/python/pypgo/bindings/energy/peer.cpp`: energy peer 实现。
- `src/python/pypgo/bindings/contact/core.h`: contact surface 与 contact energy peers。
- `src/python/pypgo/bindings/contact/core.cpp`: contact peer 实现。
- `src/python/pypgo/bindings/implicit/core.h` and `implicit/core.cpp`: implicit wrappers and named binding helpers.
- `src/python/pypgo/bindings/mesh/geo_core.h` and `mesh/geo_core.cpp`: mesh geo named helpers.
- `src/python/pypgo/bindings/mesh/volume_core.cpp`: volume mesh helper implementations currently in `volume_bindings.cpp`.

重点修改文件：

- `src/python/pypgo/CMakeLists.txt`: 新增 `.cpp` 加入 `PYPGO_BINDING_SOURCES`。
- `src/python/pypgo/bindings/energy/core.h`: 迁移为 polymorphic `PyPotentialEnergy` base 或转发到 `energy/peer.h`。
- `src/python/pypgo/bindings/energy/bindings.cpp`: 只绑定 `PyXXXX` classes/factories。
- `src/python/pypgo/bindings/sparse/core.h`: `matvec`、`matmat`、factory helper 放到 `PySparseMatrix`。
- `src/python/pypgo/bindings/sparse/bindings.cpp`: 删除 Eigen 运算 lambda，只绑定成员函数。
- `src/python/pypgo/bindings/constraints/constraint_core.h`: 统一为 `PyConstraintFunctions` 命名，可保留 base peer。
- `src/python/pypgo/bindings/constraints/constraint_bindings.cpp`: 删除 sparse-to-Eigen 和 factory 实现，只绑定 named functions。
- `src/python/pypgo/bindings/solver/solver_bindings.cpp`: 只绑定 `PyOptimizationProblem` / `PyOptimizer` / `PyNewtonOptimizer`。
- `src/python/pypgo/bindings/simulation/bindings.cpp`: `PyDynamicSimulation::step` 接收 `PyOptimizer`，不再接收 Newton 参数散列表。
- `pypgo/energy.py`: `_handle` 为 concrete peer；不再使用 `_potential_handle`。
- `pypgo/fem/energy.py`: `DeformationEnergy._handle` 改为 `PyDeformationEnergy`。
- `pypgo/contact.py`: contact surface 和 contact energy peers 统一使用 `_handle`。
- `pypgo/solver.py`: 增加 Python `Optimizer` base；`OptimizationProblem`、`NewtonOptimizer` 持有 C++ peer。
- `pypgo/sim.py`: `DynamicSimulation.step` 接收 `solver.Optimizer`。
- `pypgo/torch.py`: `inner_optimizer` 类型检查改为 `solver.Optimizer`。
- `pypgo/sim_builders.py`: 类型标注从 `NewtonOptimizer` 放宽为 `Optimizer`。

---

## Task 1: Energy Base And Concrete Peers

**Files:**

- Create: `src/python/pypgo/bindings/energy/peer.h`
- Create: `src/python/pypgo/bindings/energy/peer.cpp`
- Modify: `src/python/pypgo/bindings/energy/core.h`
- Modify: `src/python/pypgo/bindings/energy/bindings.cpp`
- Modify: `src/python/pypgo/CMakeLists.txt`
- Modify: `pypgo/energy.py`
- Modify: `tests/pypgo/test_energy.py`

- [x] **Step 1: 写失败测试，确认 `_handle` 是 concrete peer 且直接继承 abstract peer**

在 `tests/pypgo/test_energy.py` 增加：

```python
def test_energy_handles_are_concrete_peers_and_abstract_peers():
    import pypgo._core as _core
    from pypgo import energy

    linear = energy.LinearEnergy([1.0, 2.0])
    quadratic = energy.QuadraticEnergy([[1.0, 0.0], [0.0, 2.0]])
    total = energy.EnergySet([(linear, 1.0), (quadratic, 2.0)])

    assert isinstance(linear._handle, _core.PyOwnedPotentialEnergy)
    assert isinstance(quadratic._handle, _core.PyOwnedPotentialEnergy)
    assert isinstance(total._handle, _core.PyEnergySet)
    assert isinstance(linear._handle, _core.PyPotentialEnergy)
    assert isinstance(total._handle, _core.PyPotentialEnergy)
    assert not hasattr(linear, "_potential_handle")
    assert not hasattr(linear._handle, "as_potential_energy")
```

- [x] **Step 2: 运行失败测试**

Run:

```bash
pytest tests/pypgo/test_energy.py::test_energy_handles_are_concrete_peers_and_abstract_peers -q
```

Expected: fail because concrete peers are not exposed and Python still uses abstract `_core.PotentialEnergy` style handles.

- [x] **Step 3: 设计 C++ `PyPotentialEnergy` base**

在 `energy/peer.h` 中声明 polymorphic base：

```cpp
#pragma once

#include "eigen_numpy.h"
#include "../sparse/core.h"

#include "evaluation.h"
#include "potentialEnergy.h"
#include "constraintPotentialEnergies/multipleVertexPulling.h"
#include "linearPotentialEnergy.h"
#include "quadraticPotentialEnergy.h"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

#include <memory>
#include <string>
#include <vector>

namespace nb = nanobind;
namespace NO = pgo::NonlinearOptimization;

class PyPotentialEnergy
{
public:
  virtual ~PyPotentialEnergy() = default;
  virtual std::shared_ptr<const NO::PotentialEnergy> potentialEnergyHandle() const = 0;

  int numDofs() const;
  nb::ndarray<nb::numpy, std::int64_t> dofs() const;
  std::string stateKind() const;
  double value(nb::ndarray<nb::numpy, const double> x) const;
  nb::ndarray<nb::numpy, double> gradient(nb::ndarray<nb::numpy, const double> x) const;
  PySparseMatrix hessian(nb::ndarray<nb::numpy, const double> x) const;
  NO::StepConstraint maxStep(
    nb::ndarray<nb::numpy, const double> x,
    nb::ndarray<nb::numpy, const double> dx) const;
  nb::ndarray<nb::numpy, double> zeroState() const;
  std::string repr() const;
};
```

`energy/core.h` 可以 include `peer.h`，或迁移旧 `PyPotentialEnergy` 定义到 `peer.h` 后保持 include 路径兼容。

- [x] **Step 4: 增加 concrete energy peers**

在 `energy/peer.h` 继续声明：

```cpp
// Generic peer：用于没有类型特有行为的 energy（Linear / Quadratic / 两个 constraint penalty）。
class PyOwnedPotentialEnergy final : public PyPotentialEnergy
{
public:
  explicit PyOwnedPotentialEnergy(std::shared_ptr<const NO::PotentialEnergy> energy);
  std::shared_ptr<const NO::PotentialEnergy> potentialEnergyHandle() const override;

private:
  std::shared_ptr<const NO::PotentialEnergy> energy_;
};

// Typed peer：只为有类型特有行为/mutable 操作的 energy 建，保存具体 C++ 类型，避免 dynamic_cast。
class PyVertexAttachmentEnergy final : public PyPotentialEnergy
{
public:
  explicit PyVertexAttachmentEnergy(
    std::shared_ptr<pgo::ConstraintPotentialEnergies::MultipleVertexPulling> energy):
    energy_(std::move(energy)) {}
  std::shared_ptr<const NO::PotentialEnergy> potentialEnergyHandle() const override { return energy_; }
  void setTargetPositions(nb::ndarray<nb::numpy, const double> targetPositions);

private:
  std::shared_ptr<pgo::ConstraintPotentialEnergies::MultipleVertexPulling> energy_;
};
```

只为有类型特有行为的 energy 建 typed peer（Task 1 内只有 `PyVertexAttachmentEnergy`）。Linear / Quadratic / constraint penalty 共用 `PyOwnedPotentialEnergy`，不造空标签类，也不需要 `PyTypedPotentialEnergy` 模板。若后续某个 generic energy 需要类型特有行为，再单独拆 typed peer。

- [x] **Step 5: 实现 `energy/peer.cpp`**

将旧 `PyPotentialEnergy` 求值逻辑迁移为成员函数，所有 `handle_` 使用改成：

```cpp
const auto energy = potentialEnergyHandle();
```

`PyVertexAttachmentEnergy::setTargetPositions` 直接调用具体类型，不使用 `dynamic_cast`：

```cpp
void PyVertexAttachmentEnergy::setTargetPositions(nb::ndarray<nb::numpy, const double> targetPositions)
{
  auto target = pgo::python::ndarrayToVectorXd(targetPositions);
  energy_->setTargetPositions(std::move(target));
}
```

- [x] **Step 6: 让 factories 返回 concrete peers**

在 `energy/bindings.cpp` 或 `energy/peer.cpp` 中的 named factory 返回具体 peer：

```cpp
std::shared_ptr<PyOwnedPotentialEnergy> createLinearEnergy(nb::ndarray<nb::numpy, const double> b)
{
  auto bVec = pgo::python::ndarrayToVectorXd(b);
  auto energy = std::make_shared<pgo::PredefinedPotentialEnergies::LinearPotentialEnergy>(std::move(bVec));
  return std::make_shared<PyOwnedPotentialEnergy>(std::move(energy));
}
```

`_create_linear_energy`、`_create_quadratic_energy_*` 和 constraint penalty factory 都返回 `std::shared_ptr<PyOwnedPotentialEnergy>`。只有 `_create_vertex_attachment` 返回 `std::shared_ptr<PyVertexAttachmentEnergy>`。

- [x] **Step 7: 改造 `PyEnergySet`**

`PyEnergySet` 继承 `PyPotentialEnergy` 并直接持有 concrete `EnergySet`：

```cpp
class PyEnergySet final : public PyPotentialEnergy
{
public:
  explicit PyEnergySet(std::shared_ptr<NO::EnergySet> set);
  std::shared_ptr<const NO::PotentialEnergy> potentialEnergyHandle() const override;
  int numTerms() const;
  void setWeight(int i, double w);
  std::string repr() const;

private:
  std::shared_ptr<NO::EnergySet> set_;
};
```

`createEnergySet` 接收 `std::shared_ptr<PyPotentialEnergy>`，使用 `term->potentialEnergyHandle()`。

- [x] **Step 8: 绑定 base-before-derived，不绑定 `as_potential_energy`**

`energy/bindings.cpp` 绑定：

```cpp
nb::class_<PyPotentialEnergy, std::shared_ptr<PyPotentialEnergy>>(m, "PyPotentialEnergy")
  .def("__repr__", &PyPotentialEnergy::repr)
  .def_prop_ro("num_dofs", &PyPotentialEnergy::numDofs)
  .def("dofs", &PyPotentialEnergy::dofs)
  .def_prop_ro("state_kind", &PyPotentialEnergy::stateKind)
  .def("value", &PyPotentialEnergy::value)
  .def("gradient", &PyPotentialEnergy::gradient)
  .def("hessian", &PyPotentialEnergy::hessian)
  .def("max_step", &PyPotentialEnergy::maxStep)
  .def("zero_state", &PyPotentialEnergy::zeroState);

nb::class_<PyOwnedPotentialEnergy, PyPotentialEnergy, std::shared_ptr<PyOwnedPotentialEnergy>>(m, "PyOwnedPotentialEnergy");
nb::class_<PyVertexAttachmentEnergy, PyPotentialEnergy, std::shared_ptr<PyVertexAttachmentEnergy>>(m, "PyVertexAttachmentEnergy")
  .def("set_target_positions", &PyVertexAttachmentEnergy::setTargetPositions);
nb::class_<PyEnergySet, PyPotentialEnergy, std::shared_ptr<PyEnergySet>>(m, "PyEnergySet")
  .def("__repr__", &PyEnergySet::repr)
  .def("set_weight", &PyEnergySet::setWeight)
  .def_prop_ro("num_terms", &PyEnergySet::numTerms);
```

- [x] **Step 9: 更新 CMake**

在 `src/python/pypgo/CMakeLists.txt` energy 段加入：

```cmake
bindings/energy/peer.cpp
bindings/energy/bindings.cpp
```

- [x] **Step 10: 修改 Python `PotentialEnergy` base**

`pypgo/energy.py` 中：

```python
class PotentialEnergy:
    def __init__(self, handle):
        if not isinstance(handle, _core.PyPotentialEnergy):
            raise TypeError(
                f"handle must be a _core.PyPotentialEnergy, got {type(handle).__name__}"
            )
        object.__setattr__(self, "_handle", handle)
```

`num_dofs`、`value`、`gradient`、`hessian`、`max_step`、`zero_state` 直接委托 `self._handle`。

保留现有 `__setattr__` / `__delattr__` immutability guard；`ConstraintPenalty`、`VertexAttachment`、`EnergySet` 等子类继续使用 `object.__setattr__` 写私有字段。

- [x] **Step 11: 修改 concrete Python energies**

`LinearEnergy`、`QuadraticEnergy`、`ConstraintPenalty`、`ConstraintViolationPenalty`、`VertexAttachment` 继续 `super().__init__(factory_result)`，但 factory result 是 concrete peer。

`VertexAttachment.set_targets()` 改为直接调用 concrete peer 成员：

```python
self._handle.set_target_positions(tgt)
```

删除旧 C++ free function binding `_set_vertex_attachment_target_positions`。

`EnergySet.__init__` 改为：

```python
cpp_terms.append((energy._handle, float(weight)))
handle = _core._create_energy_set(cpp_terms)
super().__init__(handle)
```

删除 `_core_handle` 字段，`num_terms`、`set_weight` 和 `__repr__` 直接委托 `self._handle`。

- [x] **Step 12: 跑 energy 测试**

Run:

```bash
cmake --build build --target pypgo_core
pytest tests/pypgo/test_energy.py -q
```

Expected: all energy tests pass.

---

## Task 2: Solver Peer And Python Solver Facade

**Files:**

- Create: `src/python/pypgo/bindings/solver/core.h`
- Create: `src/python/pypgo/bindings/solver/core.cpp`
- Modify: `src/python/pypgo/bindings/solver/solver_bindings.cpp`
- Modify: `src/python/pypgo/CMakeLists.txt`
- Modify: `pypgo/solver.py`
- Modify: `tests/pypgo/test_solver.py`

- [x] **Step 1: 写失败测试，覆盖 C++ peer、Python facade 和 solve 行为**

在 `tests/pypgo/test_solver.py` 增加：

```python
def test_optimizer_problem_peers_and_newton_solve():
    import numpy as np
    import pypgo._core as _core
    from pypgo import energy, solver

    objective = energy.QuadraticEnergy([[2.0]], [-4.0])
    problem = solver.OptimizationProblem(objective=objective)
    optimizer: solver.Optimizer = solver.NewtonOptimizer(max_iterations=10, gradient_tolerance=1e-10)

    assert isinstance(problem._handle, _core.PyOptimizationProblem)
    assert isinstance(optimizer, solver.Optimizer)
    assert isinstance(optimizer._handle, _core.PyNewtonOptimizer)
    assert isinstance(optimizer._handle, _core.PyOptimizer)
    assert not hasattr(optimizer._handle, "as_optimizer")

    result = optimizer.solve(problem, np.array([0.0], dtype=np.float64))

    assert result.converged
    assert result.status == solver.SolveStatus.CONVERGED
    np.testing.assert_allclose(result.x, [2.0], atol=1e-8)
```

- [x] **Step 2: 运行失败测试**

Run:

```bash
pytest tests/pypgo/test_solver.py::test_optimizer_problem_peers_and_newton_solve -q
```

Expected: fail because solver peers and Python facade are not wired.

- [x] **Step 3: 新增 `solver/core.h`**

写入 `src/python/pypgo/bindings/solver/core.h`：

```cpp
#pragma once

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include "../energy/peer.h"
#include "eigen_numpy.h"
#include "solver/newton/NewtonOptimizer.h"
#include "solver/service/optimizationProblem.h"
#include "solver/service/optimizerUtils.h"
#include "solver/service/optimizationResult.h"

#include <memory>
#include <string>
#include <vector>

namespace nb = nanobind;
namespace NOO = pgo::NonlinearOptimization::Optimization;

class PyOptimizationProblem
{
public:
  explicit PyOptimizationProblem(std::shared_ptr<PyPotentialEnergy> objective);

  const NOO::OptimizationProblem &handle() const { return problem_; }
  NOO::OptimizationProblem &handle() { return problem_; }

  void setVariableBounds(
    nb::ndarray<nb::numpy, const double> lower,
    bool hasLower,
    nb::ndarray<nb::numpy, const double> upper,
    bool hasUpper);

  void fixVariables(std::vector<int> dofs, nb::ndarray<nb::numpy, const double> values, int numDofs);

private:
  std::shared_ptr<PyPotentialEnergy> objectiveOwner_;
  NOO::OptimizationProblem problem_;
};

struct PyNewtonOptimizerOptions
{
  int maxIterations = 50;
  double gradientTolerance = 1e-6;
  bool damping = true;
  std::string lineSearch = "backtrack";
  int verbose = 0;
  int sparseSolverKind = 0;
};

class PyOptimizer
{
public:
  virtual ~PyOptimizer() = default;
  virtual NOO::Optimizer &asOptimizer() = 0;
  nb::dict solve(const PyOptimizationProblem &problem, nb::ndarray<nb::numpy, const double> x0);
};

class PyNewtonOptimizer final : public PyOptimizer
{
public:
  explicit PyNewtonOptimizer(PyNewtonOptimizerOptions options);
  NOO::Optimizer &asOptimizer() override { return optimizer_; }

private:
  NOO::NewtonOptimizer optimizer_;
};

nb::dict optimizationResultToDict(NOO::OptimizationResult result);
NOO::NewtonOptimizer::Options makeNewtonOptions(const PyNewtonOptimizerOptions &options);
```

- [x] **Step 4: 新增 `solver/core.cpp`**

实现中使用 Task 1 已存在的 `PyPotentialEnergy::potentialEnergyHandle()`：

```cpp
PyOptimizationProblem::PyOptimizationProblem(std::shared_ptr<PyPotentialEnergy> objective):
  objectiveOwner_(std::move(objective))
{
  if (!objectiveOwner_)
    throw nb::type_error("objective must not be None");
  problem_.objective = objectiveOwner_->potentialEnergyHandle();
}
```

把当前 `solver_bindings.cpp` 中的 `parseLineSearch`、`diagnosticsToDict`、`resultToDict` 移到 `solver/core.cpp`。旧散参 `makeNewtonOptimizerOptions(...)` 不做机械搬迁，替换为新签名 `makeNewtonOptions(const PyNewtonOptimizerOptions &options)`。

`PyOptimizer::solve`：

```cpp
nb::dict PyOptimizer::solve(
  const PyOptimizationProblem &problem,
  nb::ndarray<nb::numpy, const double> x0)
{
  auto x0Map = pgo::python::ndarrayToVectorMapXd(x0);
  NOO::OptimizationResult result;
  {
    nb::gil_scoped_release release;
    result = asOptimizer().solve(problem.handle(), x0Map);
  }
  return optimizationResultToDict(std::move(result));
}
```

`PyNewtonOptimizer` 构造期一次性建 `NOO::NewtonOptimizer`，不保存 Python options 镜像：

```cpp
PyNewtonOptimizer::PyNewtonOptimizer(PyNewtonOptimizerOptions options):
  optimizer_(makeNewtonOptions(options))
{
}
```

- [x] **Step 5: 重写 `solver_bindings.cpp` 为纯 binding**

Binding 不暴露 Python 侧 `as_optimizer`：

```cpp
nb::class_<PyOptimizer, std::shared_ptr<PyOptimizer>>(m, "PyOptimizer")
  .def("solve", &PyOptimizer::solve);

nb::class_<PyNewtonOptimizer, PyOptimizer, std::shared_ptr<PyNewtonOptimizer>>(m, "PyNewtonOptimizer")
  .def(nb::init<PyNewtonOptimizerOptions>(), nb::arg("options"));
```

保留 `PyOptimizationProblem` 和 `PyNewtonOptimizerOptions` bindings。`PyNewtonOptimizerOptions` 必须支持默认构造，并以 `def_rw` 暴露 `max_iterations`、`gradient_tolerance`、`damping`、`line_search`、`verbose`、`sparse_solver_kind`，供 Python `NewtonOptimizer.__init__` 填充。

- [x] **Step 6: 更新 CMake**

在 `src/python/pypgo/CMakeLists.txt` solver 段加入：

```cmake
bindings/solver/core.cpp
bindings/solver/solver_bindings.cpp
```

- [x] **Step 7: 修改 `pypgo/solver.py`**

新增 Python `Optimizer` base：

```python
class Optimizer:
    def __init__(self, handle) -> None:
        if not isinstance(handle, _core.PyOptimizer):
            raise TypeError(f"handle must be a _core.PyOptimizer, got {type(handle).__name__}")
        self._handle = handle

    def solve(self, problem: OptimizationProblem, x0: np.ndarray | Sequence[float]) -> SolverResult:
        if not isinstance(problem, OptimizationProblem):
            raise TypeError("problem must be an OptimizationProblem")
        problem._sync_variable_bounds_to_handle()
        x0_arr = float_vector("x0", x0)
        return _result_from_core(self._handle.solve(problem._handle, x0_arr))
```

`OptimizationProblem.__init__`：

```python
self.objective = objective
self.variable_bounds = Bounds()
self._handle = _core.PyOptimizationProblem(objective._handle)
```

增加同步方法：

```python
def _sync_variable_bounds_to_handle(self) -> None:
    lower = (
        np.empty(0, dtype=np.float64)
        if self.variable_bounds.lower is None
        else float_vector("variable_bounds.lower", self.variable_bounds.lower)
    )
    upper = (
        np.empty(0, dtype=np.float64)
        if self.variable_bounds.upper is None
        else float_vector("variable_bounds.upper", self.variable_bounds.upper)
    )
    self._handle.set_variable_bounds(
        lower,
        self.variable_bounds.lower is not None,
        upper,
        self.variable_bounds.upper is not None,
    )
```

`fix_variables()` 保持更新 Python `variable_bounds`，并在末尾调用 `_sync_variable_bounds_to_handle()`。

`NewtonOptimizer` 改为 thin wrapper（**不再是 dataclass**），构造期校验参数、一次性构建 C++ peer 存入 `_handle`，此后 `_handle` 稳定：

```python
class NewtonOptimizer(Optimizer):
    def __init__(self, *, max_iterations=50, gradient_tolerance=1e-6,
                 damping=True, line_search="backtrack", verbose=0,
                 sparse_solver="auto"):
        if sparse_solver not in _SPARSE_SOLVERS:
            raise ValueError(f"sparse_solver must be one of {list(_SPARSE_SOLVERS)}, got {sparse_solver!r}")
        if line_search not in _LINE_SEARCH_METHODS:
            raise ValueError(f"line_search must be one of {sorted(_LINE_SEARCH_METHODS)}, got {line_search!r}")

        options = _core.PyNewtonOptimizerOptions()
        options.max_iterations = int(max_iterations)
        options.gradient_tolerance = float(gradient_tolerance)
        options.damping = bool(damping)
        options.line_search = str(line_search)
        options.verbose = int(verbose)
        options.sparse_solver_kind = _SPARSE_SOLVERS[sparse_solver]
        super().__init__(_core.PyNewtonOptimizer(options))
```

不提供构造后修改 options 的接口（无 dataclass 字段、无 setter、无 `_sync_handle_before_solve`）。`Optimizer.solve()` 直接用稳定的 `self._handle`；要换参数就新建一个 `NewtonOptimizer`。若将来需要只读访问，可加 property 委托到 C++ peer，但当前没有读者，先不加。

- [x] **Step 8: 删除 `solve_newton`，更新 `__all__`**

删除 `solve_newton` 函数、`__all__` 中的 `"solve_newton"`，以及随之失去引用的 `replace` import 和 `_as_fixed_dofs` helper。规范用法改为「构造 problem → 实例化 optimizer → solve」：

```python
problem = solver.OptimizationProblem(objective=energy)
problem.fix_variables([2], [9.0], num_dofs=x0.size)   # 需要固定 DOF 时
result = solver.NewtonOptimizer(damping=False).solve(problem, x0)
```

`__all__` 加入 `"Optimizer"`。把 `tests/pypgo/test_solver.py::test_solve_newton_shim_uses_object_api` 改写成上面的 object API 流程以保留 `fix_variables` 覆盖；删除只验证 `solve_newton(options={...})` 报错的那个用例。

- [x] **Step 9: 构建并跑 solver tests**

Run:

```bash
cmake --build build --target pypgo_core
pytest tests/pypgo/test_solver.py -q
```

Expected: all solver tests pass.

---

## Task 3: FEM Energy Peer

**Files:**

- Modify: `src/python/pypgo/bindings/energy/bindings.cpp`
- Modify: `src/python/pypgo/bindings/energy/peer.h`
- Modify: `src/python/pypgo/bindings/energy/peer.cpp`
- Modify: `pypgo/fem/energy.py`
- Modify: `tests/pypgo/test_deformation_energy.py`

- [x] **Step 1: 写失败测试**

在 `tests/pypgo/test_deformation_energy.py` 中复用该文件已有 helper，新增 top-level 测试：

```python
def test_deformation_energy_handle_is_concrete_peer():
    import pypgo._core as _core
    import pypgo.fem as pf

    sim = _make_tet_sim_mesh()
    state = _make_state(sim)
    e = pf.deformation_energy(state, formulation=pf.TetP1())

    assert isinstance(e._handle, _core.PyDeformationEnergy)
    assert isinstance(e._handle, _core.PyPotentialEnergy)
```

- [x] **Step 2: 运行失败测试**

Run:

```bash
pytest tests/pypgo/test_deformation_energy.py::test_deformation_energy_handle_is_concrete_peer -q
```

Expected: fail because Python currently stores `_core` plus abstract `_handle`.

- [x] **Step 3: 让 `PyDeformationEnergy` 继承 `PyPotentialEnergy`**

`PyDeformationEnergy` 存 concrete `std::shared_ptr<SolidDeformationModel::DeformationModelEnergy>`，实现：

```cpp
std::shared_ptr<const NO::PotentialEnergy> potentialEnergyHandle() const override
{
  return energy_;
}
```

删除内部 `std::shared_ptr<PyPotentialEnergy> handle_`。

- [x] **Step 4: 绑定继承关系**

```cpp
nb::class_<PyDeformationEnergy, PyPotentialEnergy, std::shared_ptr<PyDeformationEnergy>>(m, "PyDeformationEnergy")
  .def_prop_ro("num_vertices", &PyDeformationEnergy::numVertices)
  .def_prop_ro("num_plastic_dofs", &PyDeformationEnergy::numPlasticDofs)
  .def("rest_position", &PyDeformationEnergy::restPosition)
  .def("plastic_gradient", &PyDeformationEnergy::plasticGradient)
  .def("plastic_hessian", &PyDeformationEnergy::plasticHessian)
  .def("plastic_jacobian", &PyDeformationEnergy::plasticJacobian);
```

- [x] **Step 5: 修改 Python `DeformationEnergy`**

```python
def __init__(self, core):
    if not isinstance(core, _core.PyDeformationEnergy):
        raise TypeError(...)
    super().__init__(core)
```

所有 `self._core` 使用改成 `self._handle`。

- [x] **Step 6: 处理 `PlasticMaterialEnergy`**

若 C++ factory 仍返回 generic owned energy，Python `PlasticMaterialEnergy._handle` 可以是 `_core.PyOwnedPotentialEnergy`。若需要一一对应到更细类型，新增 `PyPlasticMaterialEnergy : PyPotentialEnergy` 并让 factory 返回它。测试至少确认：

```python
assert isinstance(plastic_energy._handle, _core.PyPotentialEnergy)
```

- [x] **Step 7: 跑 FEM energy tests**

Run:

```bash
cmake --build build --target pypgo_core
pytest tests/pypgo/test_deformation_energy.py tests/pypgo/test_deformation_energy_private.py -q
```

Expected: tests pass.

---

## Task 4: Contact Peer And Contact Field Naming

**Files:**

- Create: `src/python/pypgo/bindings/contact/core.h`
- Create: `src/python/pypgo/bindings/contact/core.cpp`
- Modify: `src/python/pypgo/bindings/contact/bindings.cpp`
- Modify: `src/python/pypgo/CMakeLists.txt`
- Modify: `pypgo/contact.py`
- Modify: `tests/pypgo/test_contact.py`

- [x] **Step 1: 写失败测试**

在 `tests/pypgo/test_contact.py` 增加：

```python
def test_contact_surface_and_energy_handles_are_concrete_peers():
    import numpy as np
    import pypgo._core as _core
    from pypgo import contact

    surface = contact.ContactSurface.identity(
        np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0]], dtype=np.float64)
    )
    triangles = np.array([[0, 1, 2]], dtype=np.int64)
    e = contact.SampledPenaltyEnergy(surface, triangles)

    assert isinstance(surface._handle, _core.PyContactSurface)
    assert isinstance(e._handle, _core.PySampledPenaltyContactEnergy)
    assert isinstance(e._handle, _core.PyStatefulContactEnergy)
    assert isinstance(e._handle, _core.PyPotentialEnergy)
```

- [x] **Step 2: 运行失败测试**

Run:

```bash
pytest tests/pypgo/test_contact.py::test_contact_surface_and_energy_handles_are_concrete_peers -q
```

Expected: fail because `ContactSurface` still uses `_core` and contact energies store `_contact_core`.

- [x] **Step 3: 建立 contact peer hierarchy**

`contact/core.h`：

```cpp
class PyContactSurface
{
public:
  explicit PyContactSurface(Contact::ContactSurfaceSpec spec);
  const Contact::ContactSurfaceSpec &spec() const;
  int numSurfaceVertices() const;
  int numSurfaceDofs() const;
  int numSimulationDofs() const;

private:
  Contact::ContactSurfaceSpec spec_;
};

class PyStatefulContactEnergy : public PyPotentialEnergy
{
public:
  explicit PyStatefulContactEnergy(std::shared_ptr<Contact::StatefulContactEnergy> energy);
  std::shared_ptr<const NO::PotentialEnergy> potentialEnergyHandle() const override;
  void beginStep(double time, double timestep, nb::object previous);
  bool isStepDependent() const;

protected:
  std::shared_ptr<Contact::StatefulContactEnergy> energy_;
};

class PySampledPenaltyContactEnergy final : public PyStatefulContactEnergy { using PyStatefulContactEnergy::PyStatefulContactEnergy; };
class PyFrictionalSampledPenaltyContactEnergy final : public PyStatefulContactEnergy { using PyStatefulContactEnergy::PyStatefulContactEnergy; };
class PyIPCContactEnergy final : public PyStatefulContactEnergy
{
public:
  using PyStatefulContactEnergy::PyStatefulContactEnergy;
  void setMovingObstacleTime(double time);
};
```

- [x] **Step 4: 让 factories 返回 concrete peers**

`_create_contact_surface_identity` / `_create_contact_surface_embedded` 返回 `PyContactSurface`。

`_create_sampled_penalty_contact_energy` 返回 `std::shared_ptr<PySampledPenaltyContactEnergy>`。`_create_ipc_contact_energy` 返回 `std::shared_ptr<PyIPCContactEnergy>`。`_create_frictional_sampled_penalty_contact_energy` 返回 `std::shared_ptr<PyFrictionalSampledPenaltyContactEnergy>`。

Floor energy 若没有 stateful 行为，可返回 typed `PyFloorContactEnergy : PyPotentialEnergy` 或 `PyOwnedPotentialEnergy`。

- [x] **Step 5: 修改 Python contact**

`ContactSurface` dataclass 字段从 `_core` 改为 `_handle`。所有 `surface._core` 使用改成 `surface._handle`。

`_StatefulContactMixin.begin_step`：

```python
self._handle.begin_step(float(time), float(timestep), previous)
```

`is_step_dependent`：

```python
return bool(self._handle.is_step_dependent)
```

删除 `_contact_core`。`IPCEnergy.set_moving_obstacle_time` 改成 `self._handle.set_moving_obstacle_time(float(time))`。

- [x] **Step 6: 跑 contact tests**

Run:

```bash
cmake --build build --target pypgo_core
pytest tests/pypgo/test_contact.py -q
```

Expected: tests pass.

---

## Task 5: Simulation Consumes Optimizer Base

**Files:**

- Modify: `src/python/pypgo/bindings/simulation/bindings.cpp`
- Create or Modify: `src/python/pypgo/bindings/simulation/core.h`
- Create: `src/python/pypgo/bindings/simulation/core.cpp`
- Modify: `src/python/pypgo/CMakeLists.txt`
- Modify: `pypgo/sim.py`
- Modify: `pypgo/torch.py`
- Modify: `pypgo/sim_builders.py`
- Modify: `tests/pypgo/test_dynamic_stepper.py`
- Modify: `tests/pypgo/test_sparse_and_sim.py`

- [x] **Step 1: 写失败测试**

在 `tests/pypgo/test_dynamic_stepper.py` 中复用已有 `_rest_state()` / `_spring()` helper，新增：

```python
def test_dynamic_step_accepts_optimizer_base():
    class WrappedOptimizer(pgo.solver.Optimizer):
        pass

    n = 1
    sim = DynamicSimulation(
        mass=np.eye(n),
        state=_rest_state(n),
        timestep=0.05,
        energy=_spring(n),
        integrator="implicit_euler",
    )
    newton = pgo.solver.NewtonOptimizer(max_iterations=1)
    optimizer = WrappedOptimizer(newton._handle)

    frame = sim.step(optimizer=optimizer)

    assert frame.frame_index == 0
    assert frame.solver_result.raw_status_code is not None
```

- [x] **Step 2: 运行失败测试**

Run after Task 2 is complete and before Task 5 implementation:

```bash
pytest tests/pypgo/test_dynamic_stepper.py::test_dynamic_step_accepts_optimizer_base -q
```

Expected: fail because current `DynamicSimulation.step` still requires `pypgo.solver.NewtonOptimizer`, not the `Optimizer` base.

- [x] **Step 3: 修改 C++ `PyDynamicSimulation::step` 签名**

从散装 Newton 参数改为：

```cpp
nb::dict step(
  nb::ndarray<nb::numpy, const double> externalForce,
  nb::ndarray<nb::numpy, const double> fixedValues,
  bool hasFixedValues,
  std::shared_ptr<PyOptimizer> optimizer)
```

实现中：

```cpp
if (!optimizer)
  throw nb::type_error("optimizer must not be None");
result = stepper_->step(state_, request, optimizer->asOptimizer());
```

- [x] **Step 4: 移动 simulation helper**

`PyDynamicSimulation` class、`buildSparse`、`parseIntegrator`、`solverResultToDict` 移到 `simulation/core.h` / `simulation/core.cpp`。`simulation/bindings.cpp` 只保留 bindings。

- [x] **Step 5: 修改 Python `pypgo/sim.py`**

Dynamic simulation 构造时 energy handle 改为：

```python
handle = energy._handle
```

`DynamicSimulation.step` 类型改为：

```python
optimizer: _solver.Optimizer | None = None,
```

检查改为：

```python
optimizer = optimizer if optimizer is not None else _solver.NewtonOptimizer()
if not isinstance(optimizer, _solver.Optimizer):
    raise TypeError("optimizer must be a pypgo.solver.Optimizer")
```

调用改为：

```python
data = self._handle.step(force, fixed_arr, has_fixed, optimizer._handle)
```

如果 `DynamicSimulation` 仍暂时使用 `_sim` 字段，本任务中同步改成 `_handle`，避免 Task 7 重复触碰。

- [x] **Step 6: 修改 `pypgo/torch.py` 和 `pypgo/sim_builders.py`**

`StaticEquilibriumLayer.__init__`：

```python
inner_optimizer: solver.Optimizer | None = None,
```

检查：

```python
if not isinstance(self.inner_optimizer, solver.Optimizer):
    raise TypeError("inner_optimizer must be a pypgo.solver.Optimizer")
```

`VolumeIPCSimulationBuild.optimizer` 标注改为 `_solver.Optimizer`。默认构造仍使用 `_solver.NewtonOptimizer(...)`。

- [x] **Step 7: 跑 simulation tests**

Run:

```bash
cmake --build build --target pypgo_core
pytest tests/pypgo/test_dynamic_stepper.py tests/pypgo/test_sparse_and_sim.py tests/pypgo/test_plastic_shape_match.py -q
```

Expected: tests pass.

---

## Task 6: Constraint Peer Naming And Adapter Removal

**Files:**

- Modify: `src/python/pypgo/bindings/constraints/constraint_core.h`
- Create or Modify: `src/python/pypgo/bindings/constraints/core.cpp`
- Modify: `src/python/pypgo/bindings/constraints/constraint_bindings.cpp`
- Modify: `src/python/pypgo/CMakeLists.txt`
- Modify: `pypgo/constraints.py`
- Modify: `pypgo/energy.py`
- Modify: `tests/pypgo/test_constraints.py`

- [x] **Step 1: 写失败测试**

在 `tests/pypgo/test_constraints.py` 增加：

```python
def test_constraint_handle_uses_py_prefix_and_no_python_adapter():
    import pypgo._core as _core
    from pypgo import constraints
    from pypgo.sparse import SparseMatrix

    A = SparseMatrix.from_coo((1, 2), [0], [1], [3.0])
    c = constraints.Linear(A)

    assert isinstance(c._handle, _core.PyConstraintFunctions)
    assert not hasattr(c, "_constraint_handle")
    assert not hasattr(c._handle, "as_constraint_functions")
```

- [x] **Step 2: 运行失败测试**

Run:

```bash
pytest tests/pypgo/test_constraints.py::test_constraint_handle_uses_py_prefix_and_no_python_adapter -q
```

Expected: fail because current binding still exposes the old constraint peer naming and/or Python still carries adapter fields.

- [x] **Step 3: C++ 统一命名**

把 binding class 名从 `"ConstraintFunctions"` 改为 `"PyConstraintFunctions"`。若需要保留兼容 alias，可在同一 task 中明确：

```cpp
nb::class_<PyConstraintFunctions, std::shared_ptr<PyConstraintFunctions>>(m, "PyConstraintFunctions")
  ...
```

不再绑定 `as_constraint_functions`。

- [x] **Step 4: 移动 constraints helper**

`sparseMatrixToEigen`、`createLinearConstraint`、`createConstraintSet` 移到 `constraints/core.cpp` 或 `PyConstraintFunctions` static factory。

`constraint_bindings.cpp` 只保留:

```cpp
m.def("_create_linear_constraint", &createLinearConstraint);
m.def("_create_constraint_function_set", &createConstraintSet);
```

- [x] **Step 5: 修改 Python `constraints.py` 和 `energy.py`**

`ConstraintFunction.__init__` 检查 `_core.PyConstraintFunctions`。`ConstraintFunctionSet` 和 penalty energy factories 直接传 `constraint._handle`。

- [x] **Step 6: 跑 constraints tests**

Run:

```bash
cmake --build build --target pypgo_core
pytest tests/pypgo/test_constraints.py tests/pypgo/test_energy.py -q
```

Expected: tests pass.

---

## Task 7: Mechanical `_core_obj` / `_core` Field Rename

**Files:**

- Modify: `pypgo/sparse.py`
- Modify: `pypgo/mesh/data.py`
- Modify: `pypgo/mesh/geo/core.py`
- Modify: `pypgo/mesh/volume/core.py`
- Modify: `pypgo/sim.py`
- Modify: `pypgo/fem/elastic.py`
- Modify: `pypgo/fem/plastic.py`
- Modify: `pypgo/fem/formulations.py`
- Modify: `pypgo/fem/energy.py`
- Modify: `pypgo/fem/fields.py`
- Modify: `pypgo/implicit.py`
- Modify: tests that directly inspect `_core_obj`

- [x] **Step 1: 写兼容测试**

在 `tests/pypgo/test_binding_infrastructure.py` 增加：

```python
def test_public_facades_use_handle_name_for_core_peer():
    from pypgo.sparse import SparseMatrix

    s = SparseMatrix.from_coo((1, 1), [0], [0], [1.0])

    assert hasattr(s, "_handle")
    assert not hasattr(s, "_core_obj")
```

- [x] **Step 2: 运行失败测试**

Run:

```bash
pytest tests/pypgo/test_binding_infrastructure.py::test_public_facades_use_handle_name_for_core_peer -q
```

Expected: fail because many wrappers still use `_core_obj`.

- [x] **Step 3: 安全机械替换 private peer 字段**

逐文件替换 instance fields：

```text
self._core_obj -> self._handle
obj._core_obj  -> obj._handle
self._core     -> self._handle
obj._core      -> obj._handle
_core_handle   -> _handle when it is an object peer field
_contact_core  -> _handle when it is a contact peer field
_sim           -> _handle when it is PyDynamicSimulation
```

`_contact_core` 应该已在 Task 4 删除，`_sim` 应该已在 Task 5 删除；Task 7 只验证没有残留，再决定是否需要触碰这些字段。

不要替换 module alias 或 module access：

```python
import pypgo._core as _core
_core.PySparseMatrix
_core.create_sparse_matrix
```

不要用全局 regex 盲替换 `_core.`。只替换对象字段访问。

- [x] **Step 4: 更新 helper 函数**

例如 `pypgo/contact.py::_sparse_core` 改为读取：

```python
handle = getattr(value, "_handle", None)
```

`pypgo/sparse.py::as_sparse_matrix` 保留接受 `_core.PySparseMatrix`，但返回对象内部字段为 `_handle`。

- [x] **Step 5: 更新 tests**

测试中直接访问 `_core_obj` 的地方改为 `_handle`。如果测试目的是验证私有字段，断言 concrete C++ 类型：

```python
assert isinstance(mesh._handle, _core.PyTriMeshData)
```

- [x] **Step 6: 跑基础 wrapper tests**

Run:

```bash
pytest tests/pypgo/test_binding_infrastructure.py tests/pypgo/test_mesh_geo.py tests/pypgo/test_simulation_mesh.py tests/pypgo/test_sparse_and_sim.py tests/pypgo/test_implicit.py -q
```

Expected: tests pass.

---

## Task 8: Binding Registration Zero Implementation Cleanup

**Files:**

- Modify: `src/python/pypgo/bindings/module.cpp`
- Modify: `src/python/pypgo/bindings/animation/animation_io.cpp`
- Modify: `src/python/pypgo/bindings/animation/animation_io_disabled.cpp`
- Modify: `src/python/pypgo/bindings/sparse/core.h`
- Modify: `src/python/pypgo/bindings/sparse/bindings.cpp`
- Modify: `src/python/pypgo/bindings/implicit/bindings.cpp`
- Create or Modify: `src/python/pypgo/bindings/implicit/core.h`
- Create or Modify: `src/python/pypgo/bindings/implicit/core.cpp`
- Modify: `src/python/pypgo/bindings/mesh/geo_bindings.cpp`
- Create or Modify: `src/python/pypgo/bindings/mesh/geo_core.h`
- Create or Modify: `src/python/pypgo/bindings/mesh/geo_core.cpp`
- Modify: `src/python/pypgo/bindings/mesh/volume_bindings.cpp`
- Modify: `src/python/pypgo/bindings/mesh/volume_core.h`
- Create or Modify: `src/python/pypgo/bindings/mesh/volume_core.cpp`
- Modify: `src/python/pypgo/bindings/fem/elastic_bindings.cpp`
- Modify: `src/python/pypgo/bindings/fem/formulation_bindings.cpp`
- Modify: `src/python/pypgo/bindings/energy/bindings.cpp`
- Modify: `src/python/pypgo/bindings/contact/bindings.cpp`
- Modify: `src/python/pypgo/bindings/solver/solver_bindings.cpp`
- Modify: `src/python/pypgo/bindings/simulation/bindings.cpp`
- Modify: `src/python/pypgo/bindings/constraints/constraint_bindings.cpp`
- Modify: `src/python/pypgo/bindings/parallel/bindings.cpp`

**Known offender inventory from the current tree:**

```text
src/python/pypgo/bindings/sparse/bindings.cpp
  - local create_sparse_matrix helper
  - .def("matvec", lambda) and .def("matmat", lambda)

src/python/pypgo/bindings/implicit/bindings.cpp
  - vec3 conversion helpers, OpenVDB shared conversion helper
  - constructor/property/eval/sample/bounds lambdas for GridSpec, ImplicitField, GridField, SphereField, BoxField, OpenVDBOptions
  - m.def lambdas for implicit_offset, extract_marching_cubes, has_openvdb, build/extract OpenVDB

src/python/pypgo/bindings/mesh/geo_bindings.cpp
  - flatten/export/report helper functions
  - mesh creation, OBJ IO, mesher, cleanup, remesh helper implementations
  - vertices/elements/triangles/tets/cubes property lambdas
  - m.def lambdas for component queries, cleanup, CGAL wrappers

src/python/pypgo/bindings/mesh/volume_bindings.cpp
  - PyVeg* payload structs and PyBarycentricEmbedding class
  - material conversion, volume mesh IO, mass matrix, simulation mesh factory helper implementations
  - export_* and PyVegPayload property lambdas

src/python/pypgo/bindings/energy/bindings.cpp
  - PyParameterField, PyEnergySet, PyDeformationEnergy classes
  - parse/material/field/energy factory helper implementations
  - capsule owner lambdas inside local classes

src/python/pypgo/bindings/contact/bindings.cpp
  - PyContactSurface and PyStatefulContactEnergy classes
  - contact surface, floor/contact parameter parse, obstacle parse, energy factory implementations

src/python/pypgo/bindings/simulation/bindings.cpp
  - PyDynamicSimulation class
  - sparse builder, integrator parser, solver result dict conversion

src/python/pypgo/bindings/solver/solver_bindings.cpp
  - line search parser, result dict conversion, temporary solve helper implementations

src/python/pypgo/bindings/constraints/constraint_bindings.cpp
  - sparse-to-Eigen conversion and constraint factory implementations

src/python/pypgo/bindings/fem/formulation_bindings.cpp
  - formulation mass/body-force/surface-embedding helper implementations

src/python/pypgo/bindings/fem/elastic_bindings.cpp
  - num_channels lambda

src/python/pypgo/bindings/animation/animation_io.cpp
  - capability lambdas and adapter lambdas around AnimationLoader / StressFieldVDBExporter

src/python/pypgo/bindings/animation/animation_io_disabled.cpp
  - disabled capability and throwing lambdas

src/python/pypgo/bindings/module.cpp
  - build_info lambda

src/python/pypgo/bindings/parallel/bindings.cpp
  - thread getter/setter helper implementations should move to a small parallel core wrapper
```

- [x] **Step 1: 写 sparse binding 零实现回归检查**

Run:

```bash
rg -n "\\.def\\([^\\n]*\\[|create_sparse_matrix\\(" src/python/pypgo/bindings/sparse/bindings.cpp
```

Expected before cleanup: reports `matvec` / `matmat` lambdas and the local `create_sparse_matrix` helper.

- [x] **Step 2: 将 sparse 逻辑移动到 `PySparseMatrix`**

在 `src/python/pypgo/bindings/sparse/core.h` 增加:

```cpp
static PySparseMatrix create(
    int rows,
    int cols,
    const std::vector<int>& rowIndices,
    const std::vector<int>& colIndices,
    const std::vector<double>& values);

nb::ndarray<nb::numpy, double> matvec(nb::ndarray<nb::numpy, const double> x) const;
nb::ndarray<nb::numpy, double> matmat(nb::ndarray<nb::numpy, const double> B) const;
```

实现可以放在 header inline 或新 `sparse/core.cpp`，但不得放在 `sparse/bindings.cpp`。

- [x] **Step 3: 将 `sparse/bindings.cpp` 改成纯绑定**

```cpp
nb::class_<PySparseMatrix>(m, "PySparseMatrix")
    .def("rows", &PySparseMatrix::rows)
    .def("cols", &PySparseMatrix::cols)
    .def("nnz", &PySparseMatrix::nnz)
    .def("to_coo", &PySparseMatrix::toCOO)
    .def("to_dense", &PySparseMatrix::toDense)
    .def("matvec", &PySparseMatrix::matvec)
    .def("matmat", &PySparseMatrix::matmat);

m.def("create_sparse_matrix", &PySparseMatrix::create);
```

- [x] **Step 4: 搜索所有 binding 注册文件中的局部业务实现**

Run:

```bash
rg -n "class Py|struct Py|parse[A-Z]|ToDict|create[A-Z]|make[A-Z]|build[A-Z]|compute[A-Z]|load_|save_|handle\\(\\) const|handle_|\\.def\\([^\\n]*\\[|m\\.def\\([^\\n]*\\[|\\.def_prop_[a-z]+\\([^\\n]*\\[" \
  src/python/pypgo/bindings/*/*bindings.cpp \
  src/python/pypgo/bindings/*/bindings.cpp \
  src/python/pypgo/bindings/module.cpp
```

Expected before cleanup: reports the known offender inventory above. Expected after cleanup: no local `Py...` class/struct, no helper implementation, and no `.def` / `.def_prop_*` / `m.def` lambda with implementation remains in binding registration files.

- [x] **Step 5: 按模块移动残留实现**

```text
implicit/bindings.cpp -> implicit/core.h and implicit/core.cpp or PyImplicit... wrappers
mesh/geo_bindings.cpp -> mesh/geo_core.h and mesh/geo_core.cpp or core mesh library
mesh/volume_bindings.cpp -> mesh/volume_core.h and mesh/volume_core.cpp
fem/formulation_bindings.cpp -> formulation_core.cpp or PyVolumetricFormulation members
fem/elastic_bindings.cpp -> PyElasticModel::numChannels(const PySimulationMesh&)
animation_io.cpp / disabled -> named functions or wrapper members
module.cpp build_info -> named buildInfo() function
parallel/bindings.cpp -> parallel/core.h or named PyParallel facade functions
constraints/constraint_bindings.cpp -> constraints/core.cpp or PyConstraintFunctions factories
```

- [x] **Step 6: 保留纯绑定声明**

每个 binding registration file 允许包含：

```cpp
nb::class_<...>(...)
m.def(...)
```

不允许包含跨语言对象实现、状态持有 class、算法参数解析函数、Eigen/solver/contact 计算逻辑，或带业务逻辑的 lambda。

- [x] **Step 7: 构建检查**

Run:

```bash
cmake --build build --target pypgo_core
```

Expected: build succeeds.

- [x] **Step 8: 验证 binding 零实现检查通过**

Run:

```bash
rg -n "class Py|struct Py|\\.def\\([^\\n]*\\[|m\\.def\\([^\\n]*\\[|\\.def_prop_[a-z]+\\([^\\n]*\\[" \
  src/python/pypgo/bindings/*/*bindings.cpp \
  src/python/pypgo/bindings/*/bindings.cpp \
  src/python/pypgo/bindings/module.cpp
```

Expected: no output.

---

## Task 9: Docs, Grep Cleanup, And Full Validation

**Files:**

- Modify: `docs/pypgo/fem/energy.md`
- Modify: relevant docs under `docs/pypgo/`
- Modify: notebook generator scripts under `pypgo/examples/scripts/` only if they mention `_handle` / `_core_obj`

- [x] **Step 1: 搜索旧术语**

Run:

```bash
rg -n "_core_obj|_core_handle|_contact_core|_potential_handle|as_potential_energy|as_optimizer|_core\\.PotentialEnergy|_core\\.ConstraintFunctions|\\.handle\\)|_newton_optimizer_solve|_solve_newton|solve_newton|_sync_handle_before_solve|_sync_options_to_handle|PyLinearEnergy|PyQuadraticEnergy|NewtonOptimizer instance|must be a pypgo\\.solver\\.NewtonOptimizer" pypgo tests docs src/python/pypgo/bindings
```

Expected: matches are either removed or intentionally documented compatibility references.

- [x] **Step 2: 更新 docs**

把旧说法：

```text
All Python energy classes hold a PyPotentialEnergy handle internally
```

改为：

```text
Each Python facade stores its concrete C++ PyXXXX peer in `_handle`.
Energy peers inherit `_core.PyPotentialEnergy`, and C++ internals use `potentialEnergyHandle()` when a core `PotentialEnergy` pointer is required.
```

- [x] **Step 3: 跑 focused tests**

Run:

```bash
pytest tests/pypgo/test_solver.py tests/pypgo/test_energy.py tests/pypgo/test_deformation_energy.py tests/pypgo/test_contact.py tests/pypgo/test_dynamic_stepper.py tests/pypgo/test_constraints.py -q
```

Expected: all focused tests pass.

- [x] **Step 4: 跑全量 pypgo tests**

Run:

```bash
pytest tests/pypgo -q
```

Expected: all tests pass, or optional dependency tests skip for unavailable build features.

- [x] **Step 5: 记录迁移摘要**

在最终 PR / commit message 中写明：

```text
- `_handle` now means concrete C++ PyXXXX peer.
- Energy peers inherit PyPotentialEnergy; Python no longer uses as_potential_energy/_potential_handle.
- Optimizers preserve PyOptimizer inheritance and simulation consumes the base type.
- Binding registration files now only bind named PyXXXX classes and functions.
```

---

## 推荐提交顺序

1. `refactor(pybind): introduce energy peers`
2. `refactor(solver): add optimizer and problem peers`
3. `refactor(pybind): add fem energy peers`
4. `refactor(contact): add contact peers`
5. `refactor(sim): consume optimizer base`
6. `refactor(constraints): normalize constraint peer naming`
7. `refactor(pypgo): rename core peer fields to handle`
8. `refactor(pybind): remove binding registration implementations`
9. `docs(pypgo): document handle peer architecture`

---

## 风险与缓解

- nanobind base-class ownership 风险：所有 polymorphic `PyXXXX` class 使用 `std::shared_ptr` holder，并在 binding 中声明 base-before-derived。计划不再暴露 Python 侧 `as_potential_energy()` / `as_optimizer()`，避免空上行转换的 `shared_ptr(this)` double-free 陷阱。
- C++ core const/mutable 能量风险：dynamic simulation 需要 mutable `StepAwareEnergy`，contact peer 内部保存 mutable stateful energy，`potentialEnergyHandle()` 返回 const abstract view。
- Optimizer 构造期定死：`NewtonOptimizer` 不再是 dataclass，改为 thin wrapper，构造期校验并一次性建 `_core.PyNewtonOptimizer` 存入 `_handle`，之后稳定；不提供构造后改 options 的路径，换参数需新建 optimizer。
- Bounds 同步风险：`Optimizer.solve()` 必须调用 `problem._sync_variable_bounds_to_handle()`，防止 Python `variable_bounds` 与 C++ problem 脱节。
- 私有字段迁移风险：先完成语义变更，再机械改 `_core_obj`，避免语义重构和命名重构混在同一个提交里。
- Binding 零实现成本：所有 lambda 都要挪成 named functions，包括 trivial metadata/capability lambdas。收益是架构规则可由单一 grep 验收。
- 现有工作区 dirty 风险：执行前先查看 `git status --short`，每个任务只 stage 自己改过的文件。

---

## 自审结果

**需求覆盖检查：**

- `_handle` 一一对应 concrete C++ peer：Task 1、Task 2、Task 3、Task 4、Task 7 覆盖。
- 删除 Python 侧空 adapter：设计约定、Task 1、Task 2、Task 9 grep 覆盖。
- C++ 内部 core bridge：Task 1 的 `potentialEnergyHandle()`、Task 2/5 的 `asOptimizer()` 覆盖。
- C++ binding 侧补 `PyXXXX` 对象：Task 1、Task 2、Task 3、Task 4、Task 6 覆盖。
- binding 文件不写类实现、helper 实现或业务 lambda：Task 8 明确清理标准，并列出当前已知违规清单。
- optimizer 继承体系保留：Task 2 建立 `PyOptimizer` / `PyNewtonOptimizer` 和 Python `Optimizer` / `NewtonOptimizer`，Task 5 让 simulation 消费 base。
- bounds 单一真相源：Task 2 明确 solve 前同步。
- optimizer 配置单一真相源：Task 2 optimizer 构造期定死，C++ `_handle` 为唯一真相源，无 Python 侧可变 options；`solve_newton` 删除，规范路径为构造 problem → 实例化 optimizer → solve。
- 命名统一：设计约定、Task 6、Task 9 grep 覆盖 `_core.PotentialEnergy` / `_core.ConstraintFunctions`。

**占位符扫描：**

未使用未定占位语、延后实现占位语或空泛测试说明。测试步骤均指向现有测试文件、helper 和具体断言目标。

**类型一致性检查：**

计划中统一使用 C++ `potentialEnergyHandle()` 返回 core energy pointer；Python 不使用 `_potential_handle`。Optimizer 统一使用 C++ `PyOptimizer` / `PyNewtonOptimizer` 与 Python `Optimizer` / `NewtonOptimizer`，C++ 内部使用 `asOptimizer()`；`NewtonOptimizer` 构造期定死 options，C++ `_handle` 为唯一真相源。Constraint peer 统一使用 `_core.PyConstraintFunctions`。

---

## 执行完成记录

2026-06-07 implementation pass completed:

- C++ binding registration files now contain only binding declarations / init functions; business helpers, trivial capability flags, `build_info`, and algorithm calls are moved to named core functions.
- Python facades use `_handle` for concrete C++ peers; legacy `_core_obj`, `_core_handle`, `_contact_core`, `_potential_handle`, Python-side `as_potential_energy()`, and Python-visible `as_optimizer()` have no implementation-side residue.
- Focused verification passed:
  - `cmake --build build/pypgo --target pypgo_core`
  - binding zero-implementation grep
  - legacy terminology grep, excluding this plan and negative tests
  - `python -m pytest ...` focused regression: 175 passed
