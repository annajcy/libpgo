# Constraints API Refactor Plan

> **状态日期：** 2026-05-29
> **适用范围：** C++ `ConstraintFunctions` / constrained solver boundary cleanup + future Python `pypgo.constraints` API.
> **执行约束：** 不重写 IPOPT / Knitro / Newton 数值内核；本计划只整理 constraint function ownership、evaluation API、solver-service 对接方式和 Python public surface。Newton M3 仍只支持 fixed DOF，不支持 hard inequality constraints。

## 目标

把当前 C++ hard-constraint 抽象整理成一个适合 Python 和 solver service 使用的边界：

```python
import pypgo as pgo

linear = pgo.constraints.Linear(
    A,                 # pypgo.sparse.SparseMatrix or scipy/COO adapter
    offset=d,          # constraint value is A @ x + d
)

eq = pgo.constraints.Bounded(
    linear,
    lower=0.0,
    upper=0.0,
)

result = pgo.solver.minimize(
    energy,
    x0=x0,
    method=pgo.solver.IpoptOptions(),
    constraints=eq,
)
```

本计划不是 M3 Newton 的前置依赖。M3 只需要 solver service 预留：

```cpp
struct NonlinearConstraints
{
  ConstraintFunctions_const_p functions;
  EigenSupport::VXd lower;
  EigenSupport::VXd upper;
};
```

等 IPOPT / Knitro 进入 Python 时，再实现 `pypgo.constraints` 和 `pypgo.solver.minimize` constrained path。

## 当前问题

相关文件：

- `src/core/nonlinearOptimization/constraintFunction.h`
- `src/core/nonlinearOptimization/constraintFunctions.h/.cpp`
- `src/core/nonlinearOptimization/constraintFunctionsAssember.h/.cpp`
- `src/core/nonlinearOptimization/linearConstraintFunctions.h/.cpp`
- `src/core/nonlinearOptimization/potentialEnergyFromConstraintFunctions.h/.cpp`
- `src/core/nonlinearOptimization/lagrangian.h/.cpp`
- `src/core/nonlinearOptimization/minimizeEnergy.h/.cpp`
- deformation-specific constraints:
  - `src/core/solidDeformationModel/tetVolumeConstraintFunctions.*`
  - `src/core/solidDeformationModel/segmentChainConstraintFunctions.*`
  - `src/core/solidDeformationModel/segmentBinormalConstraintFunctions.*`
  - `src/core/solidDeformationModel/prescribedPrincipleStressConstraintFunctions.*`

具体别扭点：

### 1. `LinearConstraintFunctions` 借用 sparse matrix

```cpp
class LinearConstraintFunctions : public ConstraintFunctions
{
protected:
  const EigenSupport::SpMatD &jacConst;
  EigenSupport::VXd d;
};
```

Python binding 不能安全暴露这种引用成员。第一步必须改成 owning：

```cpp
EigenSupport::SpMatD jacConst_;
EigenSupport::VXd offset_;
```

并用 destructive lifetime test 锁住：构造后释放输入 sparse matrix，constraint 仍能 evaluation。

### 2. `ConstraintFunctionsAssembler` 与旧 `PotentialEnergies` 是同构问题

当前用法是：

```cpp
ConstraintFunctionsAssembler assembler(nAll);
assembler.addConstraint(...);
assembler.addConstraint(...);
assembler.init();
```

这和旧 `PotentialEnergies` 的 `make -> add -> init` 生命周期是同一种问题。约束组合对象应该像 `EnergySet` 一样，自己就是组合 constraint function：构造函数接完整 children 列表，内部立即完成原 `init()` 的 template / mapping 构建，不再公开 `addConstraint` / `init`。

当前仓库内显式 caller 主要是 `naturalCubicSplineFitting.cpp`，可以像 Energy plan 的 E3/E3a 一样一次性迁移到新 `ConstraintSet`。不保留长期 `ConstraintFunctionsAssembler` alias；两个名字描述同一件事会成为长期负担。

### 3. Constraint evaluation 命名还停留在旧风格

`ConstraintFunctions` 仍使用：

```cpp
createJacobian(jac)
createHessian(hess)
jacobian(x, jac)
hessian(x, lambda, hess)
```

Energy plan 已经说明 `ConstraintFunctions::hessian/createHessian/hessianDirect` 不在 M3 rename 范围内，等 constraint API 进入 Python 时再做对称整理。Python 不应该直接看到 `create*` / out-param allocation 细节。

### 4. Hard constraints、soft constraints、contact energy 容易混淆

- `ConstraintFunctions` 是 hard constrained optimization 的 `C(x)`。
- `PotentialEnergyConstraintFunctions` 是把 constraint residual 转成 soft penalty energy：`0.5 * ||C(x)||^2`。
- `pypgo.contact` 里的 floor / IPC 是 contact/barrier energy，不是 `ConstraintFunctions`。

Python 模块必须分清：

- `pypgo.constraints`：hard constraint functions and bounds.
- `pypgo.energy`：soft penalty energy wrapper, if exposed.
- `pypgo.contact`：floor / IPC / obstacle energy.

### 5. `Lagrangian` 暴露了 `[x, lambda]` augmented state

`Lagrangian` 把 constrained problem 转成对 `[x, lambda]` 的 saddle objective。这个内部 state convention 对 Python 用户不直观，不能直接暴露。Python constrained solve 应该只接受 `x0`，并在 result 中返回 `lambda`。

### 6. Deformation-specific constraints 不适合作为 first public API

`TetVolumeConstraintFunctions`、segment constraints、prescribed stress constraints 都依赖 deformation model internals、rest geometry、parameter layout 或 specialized solver workflows。它们应等 deformation FEM API 稳定后再单独设计，不进入第一版 generic constraints API。

## 非目标

- 不让 Newton M3 支持 hard inequality bounds 或 nonlinear constraints。
- 不把 `ConstraintFunctions` 直接暴露成可 Python subclass 的 nanobind trampoline。
- 不绑定 deformation-specific constraints 作为第一版 public API。
- 不把 contact floor / IPC 改成 `ConstraintFunctions`。
- 不把 `Lagrangian` 暴露给 Python。
- 不绑定 legacy `EnergyOptimizer::minimize` 宽签名。
- 不在本计划实现 IPOPT / Knitro backend 本身；只定义 constraints 与 solver service 的对接形状。
- 不支持 Python callback 在 solver inner loop 中高频计算 constraints。Python-defined constraints 留到 future work，并且必须是 coarse-grained protocol。

## 关键设计决策

### 1. `pypgo.constraints` 只表达 hard constraint functions

Python 第一版 public surface：

```text
pypgo.constraints
  ConstraintFunctionSet
  Linear
  Bounded
```

其中：

- `Linear(A, offset=None)` 表达 vector-valued function `C(x) = A @ x + offset`。
- `ConstraintFunctionSet([c1, c2, ...])` 拼接多个 vector-valued constraints。
- `Bounded(functions, lower, upper)` 给 `C(x)` 加 bound，表达 `lower <= C(x) <= upper`。

`Bounded` 才是 solver API 接收的对象；裸 `Linear` / `ConstraintFunctionSet` 只负责 evaluation。

### 2. Bounds 属于 solver problem，不属于 constraint function 本体

C++ 保持两层：

```cpp
struct ConstraintFunctionSet
{
  ConstraintFunctions_const_p functions;
};

struct BoundedConstraints
{
  ConstraintFunctions_const_p functions;
  EigenSupport::VXd lower;
  EigenSupport::VXd upper;
};
```

solver service 里的 `NonlinearConstraints` 与 `BoundedConstraints` 语义一致。这样同一个 `Linear(A, d)` 可以被用于：

- equality: `lower == upper == 0`
- one-sided inequality: `lower = 0, upper = +inf`
- interval inequality: `lower <= C(x) <= upper`
- soft penalty: `pgo.energy.ConstraintPenalty(linear)`

### 3. Box bounds 和 constraint functions 分开

变量 box bounds：

```text
lower_x <= x <= upper_x
```

不是 `ConstraintFunctions`，而是 solver problem 的 `BoxBounds`。`fixed_dofs` 是 `BoxBounds` 中 `lower_x[i] == upper_x[i]` 的 compact special case，但 Newton M3 只支持 `FixedVariables` reduced-system 路径。

几何 bbox 如果做 hard constraint，可以用 `ConstraintFunctions` 表达；如果做 penalty/barrier，应该作为 `PotentialEnergy` 暴露。

### 4. First C++ cleanup：owning linear constraints

`LinearConstraintFunctions` 必须改成 owning by value：

```cpp
class LinearConstraintFunctions : public ConstraintFunctions
{
public:
  LinearConstraintFunctions(EigenSupport::SpMatD C, EigenSupport::VXd offset);

  void setOffset(EigenSupport::VXd offset);

protected:
  EigenSupport::SpMatD jacConst_;
  EigenSupport::SpMatD lambdahZero_;
  EigenSupport::VXd offset_;
};
```

保留旧 ctor 可作为临时 compatibility wrapper，但 Python binding 只走 owning ctor。`func(x)` 继续保持当前语义：

```text
g = C * x + offset
```

### 5. `ConstraintFunctionsAssembler` → `ConstraintSet`（仿照 `EnergySet`）

把 `ConstraintFunctionsAssembler` 替换为 `ConstraintSet`，让它自己就是组合 constraint function；不再分两阶段构造、不再暴露 `addConstraint` / `init`。目标 C++ 表面：

```cpp
class ConstraintSet : public ConstraintFunctions
{
public:
  struct Term
  {
    ConstraintFunctions_const_p functions;
  };

  ConstraintSet(int numDofs, std::vector<Term> terms);

  int numTerms() const;
  const Term &term(int index) const;
};
```

构造规则：

- `terms` 按输入顺序拼接 rows，row order 是 public contract。
- `terms.empty()` 抛 `std::invalid_argument`。
- 每个 `Term::functions` 必须非空，且 `functions->getJacobianTemplate().cols() == numDofs`。
- `ConstraintSet` 持有 `shared_ptr<const ConstraintFunctions>`，child lifetime 由 C++ shared ownership 保证，不靠 Python keep-alive。
- 原 `ConstraintFunctionsAssembler` 的 sparse template / mapping 构建逻辑搬进 `ConstraintSet` ctor。
- 删除 `constraintFunctionsAssember.h/.cpp` 后不保留 typedef / alias；内部 caller 迁移到 `ConstraintSet`。

Python `ConstraintFunctionSet([...])` 必须一次性构造完成，不提供 `add_constraint()` / `init()`。

### 6. Evaluation helper 隐藏 allocation/out-param

新增 C++ helper：

```cpp
EigenSupport::VXd evaluateConstraintValues(
  const ConstraintFunctions &constraints,
  EigenSupport::ConstRefVecXd x);

EigenSupport::SpMatD evaluateConstraintJacobian(
  const ConstraintFunctions &constraints,
  EigenSupport::ConstRefVecXd x);

EigenSupport::SpMatD evaluateConstraintHessian(
  const ConstraintFunctions &constraints,
  EigenSupport::ConstRefVecXd x,
  EigenSupport::ConstRefVecXd multipliers);
```

Python `constraint.value(x)` / `jacobian(x)` / `hessian(x, multipliers)` 只走这些 helpers。用户不看 `createJacobian` / `createHessian`。

### 7. Soft penalty wrapper 属于 `pypgo.energy`

`PotentialEnergyConstraintFunctions` 可以绑定成：

```python
penalty = pgo.energy.ConstraintPenalty(
    constraints,
    weight=1.0,
)
```

它返回 `PotentialEnergy`，可加入 `EnergySet` 并由 Newton 求解。第一版只支持 zero-target residual penalty：

```text
E(x) = 0.5 * weight * ||C(x)||^2
```

若需要 target / lower-upper violation penalty，应单独设计，不和 hard constraint bound 混用。

### 8. Solver integration 后置到 constrained backend

`pypgo.solver.solve_newton` 不接受 constraints。

未来 constrained solve 形态：

```python
result = pgo.solver.minimize(
    energy,
    x0=x0,
    method=pgo.solver.IpoptOptions(),
    constraints=pgo.constraints.Bounded(linear, lower=0.0, upper=0.0),
    bounds=pgo.solver.BoxBounds(lower=xlow, upper=xhi),
)
```

C++ service 使用：

```cpp
OptimizationProblem problem;
problem.energy = energy;
problem.constraints = NonlinearConstraints{ functions, lower, upper };
problem.bounds = BoxBounds{ xlow, xhi };
```

`OptimizationResult.lambda` 和 `OptimizationResult.constraintValues` 只在 constrained backend 中填充。

## Python API 草案

```python
import numpy as np
import pypgo as pgo

A = pgo.sparse.SparseMatrix.from_coo(rows, cols, values, shape=(m, n))
d = np.zeros(m)

c = pgo.constraints.Linear(A, offset=d)

x = np.zeros(n)
values = c.value(x)          # shape (m,)
J = c.jacobian(x)            # pypgo.sparse.SparseMatrix
H = c.hessian(x, np.ones(m)) # zero sparse matrix for linear constraints

eq = pgo.constraints.Bounded(c, lower=0.0, upper=0.0)

penalty = pgo.energy.ConstraintPenalty(c, weight=1.0)
total = pgo.energy.EnergySet([(energy, 1.0), (penalty, 10.0)])
```

Shape and dtype rules:

- `x`: `(num_dofs,)`, `float64`
- `offset`: `(num_constraints,)`, `float64`
- `lower` / `upper`: scalar broadcast or `(num_constraints,)`, `float64`
- `multipliers`: `(num_constraints,)`, `float64`
- sparse matrices use `pypgo.sparse.SparseMatrix` or accepted COO/SciPy adapters from the sparse plan.

## File Map

### 新增

- `src/core/nonlinearOptimization/constraintEvaluation.h`
- `src/core/nonlinearOptimization/constraintEvaluation.cpp`
- `src/core/nonlinearOptimization/constraintSet.h`
- `src/core/nonlinearOptimization/constraintSet.cpp`
- `src/python/pypgo/bindings/constraint_bindings.cpp`
- `pypgo/constraints.py`
- `tests/src/core/linearConstraintFunctions_ownership_gtest.cpp`
- `tests/src/core/constraintEvaluation_gtest.cpp`
- `tests/pypgo/test_constraints.py`

### 修改

- `src/core/nonlinearOptimization/linearConstraintFunctions.h/.cpp`
- `src/core/nonlinearOptimization/CMakeLists.txt`
- `src/core/nonlinearOptimization/naturalCubicSplineFitting.cpp`
- `src/python/pypgo/CMakeLists.txt`
- `src/python/pypgo/bindings/module.cpp`
- `pypgo/__init__.py`
- `tests/pypgo/test_package_scaffold.py`
- `plan/python_api_migration/solver_api_refactor.plan.md`（当 constrained solver backend 进入时同步）
- `plan/python_api_migration/api_coverage.md`

### 不动

- `NewtonSolver.h/.cpp` 数值内核。
- `contact/` IPC / floor energy。
- deformation-specific constraint implementations，直到 deformation public API 稳定。
- `ConstraintFunction` scalar base 暂不进入 Python public API；如果未来需要，先做 vector-valued adapter 再加入 `ConstraintSet::Term`。
- `Lagrangian` public exposure；它只作为 backend internal。

### 删除

- `src/core/nonlinearOptimization/constraintFunctionsAssember.h`
- `src/core/nonlinearOptimization/constraintFunctionsAssember.cpp`
- `ConstraintFunctionsAssembler_p` / `ConstraintFunctionsAssembler_const_p` typedef

## Task 拆分

### Task CN1: Constraint API audit

- 审计 `ConstraintFunctions`、`LinearConstraintFunctions`、`ConstraintFunctionsAssembler`、`PotentialEnergyConstraintFunctions`、`Lagrangian` 当前 semantics。
- 确认 `LinearConstraintFunctions::func(x)` 的语义是 `C*x + d`。
- 列出所有 current callers，判断 owning change 的迁移影响。
- 确认 `ConstraintFunctionsAssembler` caller 可迁移到 `ConstraintSet(int numDofs, vector<Term>)`，当前至少包括 `naturalCubicSplineFitting.cpp`。
- 明确哪些 deformation-specific constraints 依赖 unstable deformation internals，保持非 public。
- 输出：更新本 plan 的 audit notes；不写实现代码。

### Task CN2: Owning `LinearConstraintFunctions`

- 把 `LinearConstraintFunctions` 改成 owning `SpMatD` + `VXd`。
- 新增 `setOffset(VXd)`，不再暴露 `setd(ConstRefMatXd)` 给 Python。
- 保持旧 caller 编译；必要时保留 temporary overload。
- 测试：
  - 构造后释放输入 sparse/vector，`func` / `jacobian` / `hessian` 仍正确。
  - `func(x)` 与 `C*x+d` 一致。
  - linear hessian and hessian-vector are zero.

### Task CN3: `ConstraintFunctionsAssembler` → `ConstraintSet`

- 新增 `constraintSet.h/.cpp`，把 `constraintFunctionsAssember.cpp` 的组合逻辑整体搬过来，类名改为 `ConstraintSet`。
- ctor 签名改为 `ConstraintSet(int numDofs, std::vector<ConstraintSet::Term> terms)`，内部按 `terms` 顺序填充 children，然后立即跑原 `init()` 的全部逻辑。
- 删除 public `addConstraint` / `init`。
- `ConstraintSet::Term::functions` 是 `shared_ptr<const ConstraintFunctions>`。
- 迁移 `naturalCubicSplineFitting.cpp`：旧的 `make_shared<ConstraintFunctionsAssembler>(nAll) + add + init` 改为 `make_shared<ConstraintSet>(nAll, vector<Term>{...})`。
- 删除 `constraintFunctionsAssember.h/.cpp`、assembler typedef、CMake 条目；不保留 alias。
- 测试：
  - 构造时立即可求值，无需用户调 `init`。
  - 零 term 抛 `std::invalid_argument`。
  - 多个 linear constraints 拼接后 value row order 稳定。
  - Jacobian shape / nnz / values 正确。
  - Hessian zero for all-linear set.
  - child `shared_ptr` 在外部释放后，`ConstraintSet` 仍可 evaluation。
  - `naturalCubicSplineFitting` 相关 focused test/build 仍通过。
  - DOF 数不一致抛 `invalid_argument`。

### Task CN3a: Commit sequence for `ConstraintSet`

仿照 Energy plan E3/E3a，保证中间主干可编译：

- **Commit 1**：新增 `constraintSet.h/.cpp`，`constraintFunctionsAssember.h/.cpp` 暂时保留共存；CMakeLists 编译两份。
- **Commit 2**：迁移 `naturalCubicSplineFitting.cpp` 到 `ConstraintSet`。
- **Commit 3**：删除 `constraintFunctionsAssember.h/.cpp`、typedef、CMake 条目；`rg "ConstraintFunctionsAssembler|constraintFunctionsAssember" src tests` 验证 0 hit。

### Task CN4: Constraint evaluation helpers

- 新增 `constraintEvaluation.h/.cpp`。
- 提供 by-value helpers：
  - `evaluateConstraintValues`
  - `evaluateConstraintJacobian`
  - `evaluateConstraintHessian`
- 测试：
  - helper 输出与 direct out-param API 一致。
  - invalid `x` / multipliers shape 抛 `invalid_argument`。
  - helper 不泄漏 template buffers or mutable aliases。

### Task CN5: Python `pypgo.constraints`

- 新增 `pypgo/constraints.py`。
- 绑定：
  - `ConstraintFunctionSet`
  - `Linear`
  - `Bounded`
- Public methods:
  - `num_dofs`
  - `num_constraints`
  - `value(x)`
  - `jacobian(x)`
  - `hessian(x, multipliers)`
  - `is_linear`
- 测试：
  - linear values match NumPy / SciPy.
  - scalar lower/upper broadcast in `Bounded`.
  - lower/upper shape mismatch raises `ValueError`.
  - Python input arrays/sparse matrices are copied/owned safely.
  - public module imports lazily through `pypgo.constraints`.

### Task CN6: Soft penalty energy wrapper

- Bind `PotentialEnergyConstraintFunctions` as `pypgo.energy.ConstraintPenalty`.
- First API only supports zero residual penalty with scalar `weight`.
- Test:
  - value equals `0.5 * weight * ||C(x)||^2`.
  - gradient / hessian finite-difference check for linear constraints.
  - can be inserted into `EnergySet` and solved by Newton as a soft constraint.

### Task CN7: Solver constrained path integration

- Only start after solver service has `IpoptOptions` / `KnitroOptions`.
- Add Python `pypgo.solver.minimize(..., constraints=Bounded(...))`.
- Map `Bounded` to C++ `NonlinearConstraints`.
- Fill `OptimizationResult.lambda` / `constraintValues` when backend provides them.
- Tests:
  - simple linear equality constrained quadratic.
  - bound-only quadratic if backend supports `BoxBounds`.
  - Newton path rejects constraints with a clear error.

## 验收标准

- `LinearConstraintFunctions` no longer borrows Python-provided sparse data.
- Python users never call `createJacobian`, `createHessian`, `addConstraint`, or `init`.
- `pypgo.constraints.Linear(A, d).value(x)` equals `A @ x + d`.
- `pypgo.constraints.Bounded` validates lower/upper shape and preserves bound semantics.
- `pypgo.energy.ConstraintPenalty` works with `EnergySet` and Newton as a soft constraint.
- `pypgo.contact` remains independent of hard constraint APIs.
- `pypgo.solver.solve_newton` continues to reject hard constraints.
- `pypgo.solver.minimize` constrained path is not exposed until a constrained backend is implemented.

## Dependencies & Execution Order

```text
CN1 audit
  └─ CN2 owning LinearConstraintFunctions
       └─ CN3 ConstraintSet
            └─ CN3a delete assembler
                 └─ CN4 evaluation helpers
                      ├─ CN5 Python pypgo.constraints
                      └─ CN6 ConstraintPenalty energy
                           └─ CN7 constrained solver integration
```

External dependencies:

- `pypgo.sparse` must exist before Python `Linear(A, offset)` can accept sparse matrices.
- `pypgo.energy.PotentialEnergy` / `EnergySet` must exist before `ConstraintPenalty`.
- `pypgo.solver.minimize` constrained path waits for IPOPT / Knitro service backend.
- deformation-specific constraints wait for deformation FEM API stabilization.

## 输出（供下游使用）

- C++:
  - owning `LinearConstraintFunctions`
  - `ConstraintSet`
  - constraint evaluation helpers
- Python:
  - `pypgo.constraints.Linear`
  - `pypgo.constraints.ConstraintFunctionSet`
  - `pypgo.constraints.Bounded`
  - `pypgo.energy.ConstraintPenalty`
- Future:
  - `pypgo.solver.minimize(..., constraints=...)`
  - deformation-specific hard constraints after FEM API stabilizes
  - Python-defined custom constraints through a later protocol/trampoline design
