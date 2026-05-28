# Solver API Refactor Plan

> **状态日期：** 2026-05-28
> **适用范围：** C++ `NewtonSolver` / `EnergyOptimizer` 边界重构 + Python `pypgo.solver` binding。
> **执行约束：** 不重写 Newton solver 数值内核（damping、line search、linear solve 路径保持不变）；本计划只重构 solver 的构造接口、参数表达、结果格式、与 `EnergySet` 的对接方式。

## 目标

让 Python 用户可以一行 solve，同时返回不可变的 result object：

```python
import pypgo as pgo

total = pgo.energy.EnergySet([(elastic, 1.0), (floor, 1.0)])
u0 = total.zero_state()

result = pgo.solver.solve_newton(
    total,
    x0=u0,
    max_iter=50,
    tol=1e-6,
    fixed_dofs=[0, 1, 2],
    fixed_values=None,
    damping=True,
    line_search="backtrack",
    verbose=0,
)

u = result.x
print(result.status, result.converged, result.iterations)
print(result.final_gradient_norm)
print(result.diagnostics)
```

不让 Python 看到：

- `NewtonSolver(const double *x, SolverParam sp, energy, ...)` 这种裸指针构造函数；
- `solve(double *x, ...)` 原地改写输入数组的副作用；
- `SolverParam` / `SST_SUBITERATION_LINE_SEARCH` / `LSM_BACKTRACK` 这类 C++ enum 裸名；
- `fixedValues` nullable raw pointer；
- `EnergyOptimizer::minimize` 的 IPOPT / Knitro / Newton 多态 dispatcher 全部 signature；
- `stepFunc` callback（第一版不绑，避免 GIL 问题）。

## 当前问题

相关文件：

- `src/core/nonlinearOptimization/NewtonSolver.h/.cpp`
- `src/core/nonlinearOptimization/minimizeEnergy.h/.cpp`
- `src/core/nonlinearOptimization/solverResult.h/.cpp`
- `src/core/nonlinearOptimization/solveDiagnostics.h`
- `src/core/nonlinearOptimization/lineSearch.h/.cpp`
- `src/core/nonlinearOptimization/lineSearchAwareEnergy.h`
- `src/python/pypgo/bindings/solver_bindings.cpp`（M3 新增）

具体别扭点：

### 1. `NewtonSolver::solve(double *x, ...)` 原地改写输入

```cpp
SolverResult solve(double *x, int numIter, double epsilon, int verbose);
```

Python 用户预期 `solve_newton` 返回新 `x`，不该让用户猜 `u0` 是否被改了。Binding 必须：

- 先 `copy` 一份 `x0` 到 mutable `VXd`；
- 传给 `NewtonSolver::solve`；
- 返回 copy + result 打包。

### 2. 构造也吃 `const double *x`，`solve()` 又吃 `double *x`

```cpp
NewtonSolver(const double *x, SolverParam sp, PotentialEnergy_const_p energy_,
  const std::vector<int> &fixedDOFs, const double *fixedValues_ = nullptr);
```

构造时传的 `x` 只用来初始化内部存储和 `fixedValues` 默认值。这个 lifecycle 对 Python 不直观：Python 应该只传 `x0` + fixed DOFs + 参数。

facade 内部再统一构造 `NewtonSolver`；不暴露两个阶段的 `x`。

### 3. `SolverParam` 裸 struct + C++ enum

```cpp
struct SolverParam {
  double alpha = 0.5;
  SolverSubiterationType sst = SST_SUBITERATION_LINE_SEARCH;
  LineSearchMethod lsm = LSM_BACKTRACK;
  int stopAfterIncrease = 1;
  int addDamping = 0;
};
```

Python 侧应该有一个稳定的 `NewtonOptions` dataclass；C++ enum 名只存在于 binding 内部翻译，不对 Python 暴露。

### 4. `fixedValues` 是 nullable raw pointer

`const double *fixedValues_ = nullptr` 表示“不传时默认用 x0[fixedDOFs]”。Python 传入 `None` 时应触发这个语义，而不是把 `nullptr` 硬塞出去。

### 5. 没有 solver-service facade

现有调用方必须：

1. 构造 `NewtonSolver` visitor；
2. 调 `solve`；
3. 自己管理 `x` buffer 的拷贝和诊断查询。

Python 期望一个无状态函数 `solveNewton(energy, x0, options)`。

### 6. `EnergyOptimizer::minimize` 接口太宽

```cpp
SolverResult minimize(RefVecXd x, PotentialEnergy_const_p energy,
  ConstRefVecXd xlow, ConstRefVecXd xhi,
  RefVecXd lambda, RefVecXd g, ConstraintFunctions_const_p constraints,
  ConstRefVecXd clow, ConstRefVecXd chi,
  SolverType solverType, int maxIter, double eps, int verbose);
```

把 Newton、IPOPT、Knitro、带约束 / 不带约束全部塞进同一组重载。M3 只需要 Newton 无约束路径。这条保留给后续 `pypgo.solver.minimize` 通用入口，但不在本计划实现。

### 7. `SolveDiagnostics` 嵌套有 mutable 缓存

诊断信息在 Newton solver 内部是 mutable，但 Python 用户只读 inspection。需要 flatten 成不暴露 Eigen 内部状态的 Python dataclass。

## 非目标

- 不实现 `pypgo.solver.minimize` 通用入口（IPOPT / Knitro / constraints）。这些列入 M9 future work 或后续独立 plan。
- 不暴露 `EnergyOptimizer::minimizeUsingNewton` 裸接口。
- 不绑定 `stepFunc` callback（GIL、exception propagation、性能开销都未评估，列入 `future_work.md`）。
- 不实现 `pypgo.solver.NewtonSolver` class（reusable stateful solver）。第一版只暴露 stateless `solve_newton` function；class 在 static/dynamic loop 稳定后再设计。
- 不修改 `NewtonSolver` 内部 `SolveContext` / `StepStrategy` 结构。
- 不迁移 `runIPCSim` 内部的 Newton 调用到新 facade；内部 solver 调用保持不变。

## 设计决策

### 1. First API：stateless `solve_newton`

第一版只暴露一个 function，不接受 solver state 复用。Python API 是：

```python
pgo.solver.solve_newton(
    energy,                # PotentialEnergy handle
    x0,                    # np.ndarray (n,) float64
    max_iter=50,
    tol=1e-6,
    fixed_dofs=None,       # list[int] or None
    fixed_values=None,     # np.ndarray or None → use x0[fixed_dofs]
    damping=True,
    line_search="backtrack",
    verbose=0,
) -> SolverResult
```

复用 symbolic factorization 等优化留给后续 `NewtonSolver` class（M6+ internal static/dynamic loop 需要时再设计 API 表面）。

### 2. NewtonOptions dataclass

```python
@dataclass(frozen=True)
class NewtonOptions:
    max_iter: int = 50
    tol: float = 1e-6
    damping: bool = True
    line_search: str = "backtrack"
    verbose: int = 0
```

`line_search` 接受 `"golden" | "brents" | "backtrack" | "simple"`，binding 内映射到 `LineSearchMethod` enum。

`solve_newton` 的关键字参数全部落入 `NewtonOptions` 或直接传给 solver facade。

### 3. SolveStatus enum

```python
class SolveStatus(enum.IntEnum):
    CONVERGED = ...
    MAX_ITERATIONS = ...
    LINE_SEARCH_FAILED = ...
    STEP_TOO_SMALL = ...
    NON_FINITE = ...
    LINEAR_SOLVE_FAILED = ...
    # ... exact values determined by C++ SolveStatus parity audit
```

不直接绑 C++ `SolveStatus` 裸值；在 binding helper 内做 enum 翻译并保证 parity test 覆盖所有值。

### 4. SolverResult / SolveDiagnostics

Python 不可变 value object：

```python
@dataclass(frozen=True)
class SolveDiagnostics:
    num_iterations: int
    final_gradient_norm: float
    final_gradient_max_norm: float
    final_energy: float
    # ... fields populated from C++ SolveDiagnostics after solve completes

@dataclass(frozen=True)
class SolverResult:
    x: np.ndarray               # (n,) float64, solution DOF vector
    status: SolveStatus
    converged: bool
    iterations: int
    final_gradient_norm: float
    final_gradient_max_norm: float | None
    diagnostics: SolveDiagnostics
```

`SolverResult.x` 是 **copy**；删掉 `result` 后不影响任何 C++ 内部状态。

### 5. C++ facade：`solveNewton`

在 C++ 侧加一个无状态 adapter function，内部构造 NewtonSolver + solve + 打包结果：

```cpp
namespace pgo::NonlinearOptimization
{

struct NewtonOptions
{
  int maxIterations = 50;
  double tolerance = 1e-6;
  bool damping = true;
  int lineSearchMethod = 2;   // matches LSM_BACKTRACK
  int verbose = 0;
};

struct NewtonResult
{
  SolverResult solver;      // existing struct
  EigenSupport::VXd x;      // solution copy
};

NewtonResult solveNewton(
  std::shared_ptr<const PotentialEnergy> energy,
  EigenSupport::ConstRefVecXd x0,
  const NewtonOptions &options,
  const std::vector<int> *fixedDofs = nullptr,
  const double *fixedValues = nullptr);

}  // namespace pgo::NonlinearOptimization
```

内部实现：

```cpp
NewtonResult solveNewton(
  std::shared_ptr<const PotentialEnergy> energy,
  ConstRefVecXd x0,
  const NewtonOptions &opt,
  const std::vector<int> *fixedDofsPtr,
  const double *fixedValuesPtr)
{
  EigenSupport::VXd x = x0;

  NewtonSolver::SolverParam sp;
  sp.addDamping = opt.damping ? 1 : 0;
  sp.lsm = static_cast<NewtonSolver::LineSearchMethod>(opt.lineSearchMethod);
  sp.sst = NewtonSolver::SST_SUBITERATION_LINE_SEARCH;

  std::vector<int> fixedDofs = fixedDofsPtr ? *fixedDofsPtr : std::vector<int>{};

  NewtonSolver solver(x.data(), sp, energy, fixedDofs, fixedValuesPtr);

  SolverResult result = solver.solve(x.data(), opt.maxIterations, opt.tolerance, opt.verbose);
  return {result, std::move(x)};
}
```

### 6. `fixed_values=None` 语义

Python `fixed_values=None` → binding 传 `nullptr` 给 `solveNewton`，等价于“用 x0[fixedDOFs]”。`NewtonSolver` 内部在 fixedValues 为 `nullptr` 时已经用构造函数里的 `x` 做了 copy；facade 不额外处理。

### 7. 暂不绑定 `minimize`

`EnergyOptimizer::minimize` 的 binding 留在 `pypgo.solver.minimize` 占位，但 M3 只是 stub（抛 `NotImplementedError`），供后续 IPOPT/Knitro/constrained minimize 接上。

## 目标 C++ API

```cpp
// nonlinearOptimization/NewtonSolver.h            (unchanged — NewtonSolver class)
// nonlinearOptimization/solveNewton.h             (new facade header, small)
// nonlinearOptimization/solveNewton.cpp           (new facade impl)

namespace pgo::NonlinearOptimization
{

struct NewtonOptions
{
  int maxIterations = 50;
  double tolerance = 1e-6;
  bool damping = true;
  int lineSearchMethod = 2;   // LSM_BACKTRACK
  int verbose = 0;
};

struct NewtonResult
{
  SolverResult solver;
  EigenSupport::VXd x;
};

NewtonResult solveNewton(
  std::shared_ptr<const PotentialEnergy> energy,
  EigenSupport::ConstRefVecXd x0,
  const NewtonOptions &options,
  const std::vector<int> *fixedDofs = nullptr,
  const double *fixedValues = nullptr);

}  // namespace pgo::NonlinearOptimization
```

不修改 `NewtonSolver` 的 public 接口；不新开 header 声明 `EnergyOptimizer` 成员。

## Python API 定稿草案

```python
import pypgo as pgo
import numpy as np

energy = pgo.energy.deformation_energy(...)
u0 = energy.zero_state()

result = pgo.solver.solve_newton(
    energy,
    x0=u0,
    max_iter=50,
    tol=1e-6,
    fixed_dofs=[0, 1, 2],
    fixed_values=None,           # defaults to u0[0:3]
    damping=True,
    line_search="backtrack",
    verbose=0,
)

assert isinstance(result, pgo.solver.SolverResult)
assert result.status == pgo.solver.SolveStatus.CONVERGED
assert result.converged
assert result.x.shape == u0.shape
assert isinstance(result.x, np.ndarray)
# assert that u0 is NOT modified
assert np.allclose(u0, energy.zero_state())

# Inspect diagnostics
print(result.iterations)
print(result.final_gradient_norm)
print(result.final_gradient_max_norm)
print(result.diagnostics.num_iterations)
```

约束：

- `u0` 必须 **不被修改**；binding 内部 copy。
- `result.x` 必须是新 `np.ndarray`；内部 copy 自 `NewtonResult.x`。
- `fixed_values=None` 等价于使用 `x0[fixed_dofs]`。
- 如果 `energy.dofs` 与 `fixed_dofs` 的交集不为 `fixed_dofs`，binding 可选抛 `ValueError`；第一版不强求（信任 C++ 内部已有 assert/check）。
- `verbose > 0` 的 stdout 输出走 C++ 打印（第一版不做 Python-logging 重定向）。

`pypgo.solver` M3 表面：

```text
pypgo.solver
  SolveStatus             # IntEnum
  SolveDiagnostics        # frozen dataclass
  SolverResult            # frozen dataclass
  NewtonOptions           # frozen dataclass
  solve_newton            # stateless function
  minimize                # stub (raises NotImplementedError)
```

不暴露：`NewtonSolver`（class）、`EnergyOptimizer`、`SolverParam`、`SolverSubiterationType`、`LineSearchMethod`、`stepFunc`、`getx`、`getSolveDiagnostics`。

## File Map

### 新增

- `src/core/nonlinearOptimization/solveNewton.h`
- `src/core/nonlinearOptimization/solveNewton.cpp`
- `src/python/pypgo/bindings/solver_bindings.cpp`
- `pypgo/solver.py`
- `tests/src/core/nonlinearOptimization/solveNewton_gtest.cpp`
- `tests/pypgo/test_solver.py`（M3 已规划，本计划补具体 test 清单）

### 修改

- `src/core/nonlinearOptimization/CMakeLists.txt`：编入 `solveNewton.cpp`。
- `src/python/pypgo/CMakeLists.txt`：编入 `solver_bindings.cpp`。
- `src/python/pypgo/bindings/module.cpp`：注册 `pypgo.solver` 子模块。
- `pypgo/__init__.py`：导出 `pypgo.solver`。
- `plan/python_api_migration/api_coverage.md`：更新 Newton solver / minimize / solver status 条目。

### 不动

- `NewtonSolver.h/.cpp` 实现。
- `minimizeEnergy.h/.cpp` 接口（不下沉到 binding）。
- `lineSearch.h/.cpp`、`lineSearchAwareEnergy.h`、`barrierFunction.h`、`constraintFunctions*`。
- `IpoptOptimizer*`、`knitroOptimizer*`。

## Task 拆分

### Task S1: NewtonSolver parity audit

- 读 `solveDiagnostics.h`、`solverResult.h`、`NewtonSolver.cpp` 中 `SolveStatus` 和 `SolverResult` / `SolveDiagnostics` 的完整字段。
- 确认 Python `SolveStatus` 枚举值映射：列出现有所有 `SolveStatus` 值及其语义。
- 确认 `SolveDiagnostics` 字段清单（`numIterations`、`energyHistory`、`gradNormHistory`、`spdFailureCount` 等），决定 M3 暴露哪些。
- 产出：在本 plan 补一节 `### SolveStatus mapping table` 和 `### SolveDiagnostics field selection`。
- 不写代码。

### Task S2: 实现 `solveNewton` facade

- 新增 `solveNewton.h/.cpp`：实现 `NewtonOptions`、`NewtonResult`、`solveNewton(...)`。
- Facade 内部：
  - 用 `x0` copy 构造 `VXd x = x0`；
  - 翻译 `NewtonOptions` → `SolverParam`；
  - 构造 `NewtonSolver(x.data(), sp, energy, fixedDofs, fixedValuesPtr)`；
  - 调 `solver.solve(x.data(), ...)`；
  - 返回 `NewtonResult{solverResult, std::move(x)}`。
- 新增 `solveNewton_gtest.cpp`：
  - 用现有 `QuadraticPotentialEnergy`（或 `OwnedQuadraticPotentialEnergy` from Task E2）做最小 warm-start 测试；
  - 验证 `x0` 不被修改（传 const reference 后比较）；
  - 验证 `result.x` 是新的 buffer（地址 != x0.data()）；
  - fixed DOFs 不给 → 无 fixed DOFs 收敛；
  - fixed DOFs 给 → fixed 元素不变，其余收敛。

### Task S3: Python SolveStatus + SolveDiagnostics + SolverResult

- 在 `pypgo/solver.py` 写 `SolveStatus(IntEnum)`、`SolveDiagnostics`、`SolverResult` dataclass。
- 在 `solver_bindings.cpp` 写：
  - `_core` 级别的 enum 翻译 helper：C++ `SolveStatus` → Python `int` → `SolveStatus(value)`；
  - `SolverResult` → 返回 dict / Python attr object。
- 测试：
  - 每个 `SolveStatus` 值都能通过 `int(status)` / `SolveStatus(value)` roundtrip；
  - 拿一个实际收敛的 Newton solve，检查 `SolverResult` 字段齐全且 `converged == True`。

### Task S4: Python `solve_newton`

- 在 `solver_bindings.cpp` 绑 `solveNewton`：
  - `solve_newton(energy: PotentialEnergy, x0: np.ndarray, ...)`；
  - 所有关键字参数映射到 `NewtonOptions`；
  - `fixed_dofs` → `std::vector<int>`；
  - `fixed_values=None` → `nullptr`；
  - `line_search="backtrack"` / `"golden"` / `"brents"` / `"simple"` → `LineSearchMethod` int。
- 返回 `SolverResult` Python object。
- 在 `pypgo/solver.py` 写 `NewtonOptions` dataclass + `solve_newton` Python wrapper（只做 docstring / type hint / normalization）。

### Task S5: `minimize` stub

- 在 `pypgo/solver.py` 加入 `def minimize(...) -> NotImplementedError`，signature 照 `EnergyOptimizer::minimize` 通用接口写。
- 只挂 stub；不绑 C++ 实现。

### Task S6: Solver 端到端 test + 清理

- `tests/pypgo/test_solver.py` 补充：
  - `test_newton_solves_quadratic__warm_start`：已知解在 x=0 的二次能量，`u0` 随机初始化，验证收敛到 0；
  - `test_newton_solves_quadratic__fixed_dofs`：fixed_dofs 元素等于 x0，其余输出为 0；
  - `test_x0_not_mutated`：`u0` copy 前后比；
  - `test_line_search_keywords`：四个 option 都接受；
  - `test_verbose_does_not_crash`：`verbose=1` 不抛异常。
- 更新 `api_coverage.md` solver 条目。

## 验收标准

- `tests/src/core/nonlinearOptimization/solveNewton_gtest.cpp` 全部通过。
- `python -m pytest tests/pypgo/test_solver.py` 全部通过。
- `pypgo.solver` 公开名集合不出现 `NewtonSolver`、`SolverParam`、`EnergyOptimizer`、`SST_*`、`LSM_*`。
- `solve_newton(total, u0)` 后 `u0` 不被修改。
- `result.x` 是有效 numpy array，独立于 C++ solver 内部 buffer。
- `solveNewton.h` 不暴露 `NewtonSolver` 的 mutable 成员或 `solver->getx()` 的引用。
- `runIPCSim` 内部 Newton 调用不受 facade 引入影响。
