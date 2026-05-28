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

### 4. SolverResult

Python 不可变 value object，flat 设计，不双层嵌套：

```python
@dataclass(frozen=True)
class SolverResult:
    x: np.ndarray               # (n,) float64, solution DOF vector (copy)
    status: SolveStatus
    converged: bool
    iterations: int
    final_energy: float
    final_gradient_norm: float
    final_gradient_max_norm: float
    # 其它 C++ SolveDiagnostics 字段按需逐项纳入；不嵌套为子 dataclass
```

字段命名规则：

- 所有 scalar 用 `final_*` 前缀；
- 一个 `SolverResult` 字段只对应 C++ `SolveDiagnostics` / `SolverResult` 的一个语义；
- 不暴露 history 数组（gradient norm history、energy history）作为 first-class 字段；如未来确实需要，作为 optional `traces` 字段加入，而不是开第二层 dataclass。

`SolverResult.x` 是 **copy**；删掉 `result` 后不影响任何 C++ 内部状态。`final_gradient_max_norm` 在 solve 完成时总有值（即使 solve 失败也有最后一次评估值），不是 nullable。

### 5. C++ facade：`solveNewton`

在 C++ 侧加一个无状态 adapter function，内部构造 NewtonSolver + solve + 打包结果。`NewtonOptions` 直接使用 `NewtonSolver::LineSearchMethod` enum，避免 raw int：

```cpp
namespace pgo::NonlinearOptimization
{

struct NewtonOptions
{
  int maxIterations = 50;
  double tolerance = 1e-6;
  bool damping = true;
  NewtonSolver::LineSearchMethod lineSearchMethod = NewtonSolver::LSM_BACKTRACK;
  int verbose = 0;
  // 注意：第一版固定 SST_SUBITERATION_LINE_SEARCH；其余 SST 模式（STATIC_DAMPING / ONE）
  // 在 Python 侧尚无用户场景，留待后续以 keyword 形式新增。
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
  sp.lsm = opt.lineSearchMethod;
  sp.sst = NewtonSolver::SST_SUBITERATION_LINE_SEARCH;

  std::vector<int> fixedDofs = fixedDofsPtr ? *fixedDofsPtr : std::vector<int>{};

  NewtonSolver solver(x.data(), sp, energy, fixedDofs, fixedValuesPtr);

  SolverResult result = solver.solve(x.data(), opt.maxIterations, opt.tolerance, opt.verbose);
  return {result, std::move(x)};
}
```

### 6. `fixed_values=None` 语义

Python `fixed_values=None` → binding 传 `nullptr` 给 `solveNewton`，期望 `NewtonSolver` 内部 fallback 到 `x0[fixedDOFs]`。

**该 fallback 行为必须在 Task S1 通过读 `NewtonSolver.cpp` 验证：** 若 NewtonSolver 不内置该 fallback，facade 必须显式从 `x0` 拷贝出 `fixedValues` 再传入；如果内置，facade 透传即可。审计结论写入 Task S1 输出节。

### 7. 不暴露 `minimize`

M3 不为 `EnergyOptimizer::minimize` 提供任何 Python 占位（stub 反而引导用户写无法工作的代码）。当 IPOPT / Knitro / constrained minimize 真正纳入 Python 时再设计 API 表面。

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
  NewtonSolver::LineSearchMethod lineSearchMethod = NewtonSolver::LSM_BACKTRACK;
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
  SolverResult            # frozen dataclass, flat
  NewtonOptions           # frozen dataclass
  solve_newton            # stateless function
```

不暴露：`NewtonSolver`（class）、`EnergyOptimizer`、`SolverParam`、`SolverSubiterationType`、`LineSearchMethod`、`stepFunc`、`getx`、`getSolveDiagnostics`、`minimize`（包括 stub）。

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
- 选定 `SolverResult` flat 字段集（status / converged / iterations / final_energy / final_gradient_norm / final_gradient_max_norm + 任何确实需要的标量），写入 plan。
- **必须验证：** `NewtonSolver(const double *x, ..., fixedValues_ == nullptr)` 时，内部是否 fallback 到 `x[fixedDOFs]` 作为 fixed 值。读 `NewtonSolver.cpp` 的 `applyFixedValues` / ctor 实现，把结论写入 plan。如果不 fallback，Task S2 facade 必须显式构造 fixedValues。
- 产出：在本 plan 补两节 "### SolveStatus mapping table"、"### SolverResult field selection"、"### fixedValues fallback 行为"。
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

### Task S3: Python SolveStatus + SolverResult

- 在 `pypgo/solver.py` 写 `SolveStatus(IntEnum)`、`SolverResult` frozen dataclass（按 S1 选定的 flat 字段集）。
- 在 `solver_bindings.cpp` 写：
  - `_core` 级别的 enum 翻译 helper：C++ `SolveStatus` → Python `int` → `SolveStatus(value)`；
  - C++ `NewtonResult` → Python `SolverResult` 字段映射函数。
- 测试：
  - 每个 `SolveStatus` 值都能通过 `int(status)` / `SolveStatus(value)` roundtrip；
  - 拿一个实际收敛的 Newton solve，检查 `SolverResult` 字段齐全且 `converged == True`、`status == CONVERGED`、`iterations > 0`。

### Task S4: Python `solve_newton`

- 在 `solver_bindings.cpp` 绑 `solveNewton`：
  - `solve_newton(energy: PotentialEnergy, x0: np.ndarray, ...)`；
  - 所有关键字参数映射到 `NewtonOptions`；
  - `fixed_dofs` → `std::vector<int>`；
  - `fixed_values=None` → `nullptr`；
  - `line_search="backtrack"` / `"golden"` / `"brents"` / `"simple"` → `LineSearchMethod` int。
- 返回 `SolverResult` Python object。
- 在 `pypgo/solver.py` 写 `NewtonOptions` dataclass + `solve_newton` Python wrapper（只做 docstring / type hint / normalization）。

### Task S5: Solver 端到端 test + 清理

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

## Dependencies & Execution Order

### 外部依赖

- **Energy plan** 的 Task E0（Hessian API rename）必须先完成：
  - NewtonSolver.cpp 大量使用 `createHessian` / `hessian` / `hessianDirect`，E0 内会把它们 rename 成 `hessianAlloc` / `hessianInPlace` / `hessian`；本 plan 的 Task S2 facade 实现直接基于新名字写。
  - Solver C++ Task S1（audit）可以在 E0 进行时**并行启动**——它只读 NewtonSolver 的 SolveStatus / 字段，不依赖 hessian API 名字。
  - Solver C++ Task S2（facade 实现）必须在 E0 完成之后启动。
- **Energy plan** 的 Task E1（`evaluation.h`）和 E4（`pypgo.energy.PotentialEnergy` binding）必须先完成：
  - `solve_newton` 接受的 `energy` 参数是 `pypgo.energy.PotentialEnergy`；E4 才给出该 Python 类型；
  - 端到端测试 `solve_newton(EnergySet([...]))` 需要 E6（EnergySet binding）；
  - 单元测试可只用 Energy E2 / E5 提供的 `QuadraticEnergy` 作为 minimal energy。
- Contact plan 不是 Solver plan 的依赖。Solver 完成后，Contact plan 的 C5 端到端测试才能验证 IPC + Newton solve 联合行为。

### 内部任务依赖

```text
S1 (audit)            ─ 纯审计；可在 Energy 任何阶段并行
                       │
S2 (C++ facade + test)── 依赖 S1 的 fallback 验证结论
                       │
S3 (Python SolveStatus/SolverResult)── 依赖 S1（确定字段集）
                       │
S4 (Python solve_newton)── 依赖 S2 + S3 + Energy E4
                       │
S5 (端到端 test)        ── 依赖 S4 + Energy E5（QuadraticEnergy）；可选依赖 E6（EnergySet 多 term solve）
```

### 推荐顺序

S1 → S2 → S3 → S4 → S5。S1 / S2 可与 Energy E1–E3 并行（纯 C++）；S3 / S4 / S5 必须排在 Energy E4 / E5 / E6 完成之后。

### 输出（供下游使用）

- C++: `solveNewton(energy, x0, options, ...)`、`NewtonOptions`、`NewtonResult`。
- Python: `pypgo.solver.solve_newton`、`SolverResult`、`SolveStatus`、`NewtonOptions`。
- Contact plan C5 端到端测试用 `pgo.solver.solve_newton(EnergySet([elastic, floor, ipc]))` 验证 IPC + Newton。
- M5 静态 solve、M6 dynamic loop 直接使用 `solve_newton` 作为底层。
