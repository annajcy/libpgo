# Solver API Refactor Plan

> **状态日期：** 2026-05-29
> **适用范围：** C++ solver service boundary + Python `pypgo.solver` Newton binding。
> **执行约束：** 不重写 Newton 数值内核；damping、line search、linear solve 的数学路径、IPC active-set scope、现有 `NewtonSolver` 默认行为保持不变。本计划只重构 solver 的长期服务边界、backend 分层、参数表达、结果格式、以及 Python 入口。

## 目标

建立一个长期可复用的 C++ optimization service：

```cpp
OptimizationResult minimize(
  const OptimizationProblem &problem,
  EigenSupport::ConstRefVecXd x0,
  const NewtonOptions &options);
```

M3 只实现 Newton backend，但 C++ 问题/结果结构必须能自然承接后续 IPOPT / Knitro / constrained minimize。Python 第一版仍提供一个易用入口：

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
    fixed_values=None,       # fixed to x0[fixed_dofs]
    damping=True,
    line_search="backtrack",
    verbose=0,
)

u = result.x
print(result.status, result.converged, result.iterations)
print(result.final_objective)
print(result.final_gradient_norm)
print(result.diagnostics)
```

实现上分成两层 backend：

```text
OptimizationBackend
  NewtonOptimizationBackend
    NewtonSolver
      NewtonLineSearchPolicy
        Golden / Brent / Backtracking / Simple
      NewtonSparseSolverBackend
        Auto / EigenSimplicialLDLT / MKLPardiso / OrigPardiso
  IpoptOptimizationBackend     // future
  KnitroOptimizationBackend    // future
```

`OptimizationBackend` 处理“用哪个 optimizer 解这个 problem”；`NewtonLineSearchPolicy` 只处理 Newton line-search mode 的 alpha 选择；`NewtonSparseSolverBackend` 只处理 Newton 每步里的稀疏线性系统。三者不是同一层，不用同一个 enum 或 switch。

不让 Python 用户看到：

- `NewtonSolver(const double *x, SolverParam sp, energy, ...)` 裸指针构造函数；
- `solve(double *x, ...)` 原地改写输入数组的副作用；
- `SolverParam` / `SST_SUBITERATION_LINE_SEARCH` / `LSM_BACKTRACK` 等 legacy C++ enum 名；
- `fixedValues` nullable raw pointer；
- `EnergyOptimizer::minimize` 的 IPOPT / Knitro / Newton 宽签名；
- `stepFunc` callback（第一版不绑定，避免 GIL 和 exception propagation 问题）。

## 当前问题

相关文件：

- `src/core/nonlinearOptimization/NewtonSolver.h/.cpp`
- `src/core/nonlinearOptimization/optimizationService.h/.cpp`（M3 新增）
- `src/core/nonlinearOptimization/optimizationBackend.h/.cpp`（M3 新增）
- `src/core/nonlinearOptimization/newtonLineSearchPolicy.h/.cpp`（M3 新增）
- `src/core/nonlinearOptimization/newtonSparseSolverBackend.h/.cpp`（M3 新增）
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

Python 用户预期 `solve_newton` 返回新 `x`，不应该让用户猜 `x0` 是否被改。新的 C++ service 也必须遵守同一契约：

- 输入 `x0` 是只读 initial state；
- service 内部 copy 成 mutable `VXd x`；
- legacy `NewtonSolver` 只接收 service-owned buffer；
- `OptimizationResult.x` 是 owned solution copy。

### 2. legacy constructor 和 solve lifecycle 对外不直观

```cpp
NewtonSolver(const double *x, SolverParam sp, PotentialEnergy_const_p energy,
  const std::vector<int> &fixedDOFs, const double *fixedValues = nullptr);
```

构造时传的 `x` 用于初始化内部 storage；`solve()` 又接收另一个 mutable `x`。这个两阶段生命周期不适合作为 Python 或长期 C++ service 边界。新的 `minimize(problem, x0, options)` 负责统一构造 legacy solver、执行 solve、打包结果。

### 3. `fixedValues == nullptr` 不是可靠默认语义

当前 `NewtonSolver::setFixedDOFs` 会直接把 `fixedValues_` 映射成 Eigen vector，`applyFixedValues()` 再按 `fixedValues[i]` 读取。固定 DOF 非空时传 `nullptr` 不表示“用 x0 默认值”，而是不安全的裸指针行为。

新 service 必须显式实现：

```text
FixedVariables.values == nullopt  =>  fixedValues[i] = x0[dofs[i]]
```

并且传给 `NewtonSolver` 的 fixed values 永远来自一个 service-owned buffer。

### 4. `EnergyOptimizer::minimize` 接口过宽

```cpp
SolverResult minimize(RefVecXd x, PotentialEnergy_const_p energy,
  ConstRefVecXd xlow, ConstRefVecXd xhi,
  RefVecXd lambda, RefVecXd g, ConstraintFunctions_const_p constraints,
  ConstRefVecXd clow, ConstRefVecXd chi,
  SolverType solverType, int maxIter, double eps, int verbose);
```

这个接口把 objective、bounds、constraints、backend choice、solver-specific options 和 output buffers 混在一个签名里。M3 不绑定它，也不以它作为新的 C++ service 表面。后续 IPOPT / Knitro 可以迁移到新的 `OptimizationProblem` / `OptimizationResult` 结构。

### 5. result 分层不清

现有 `SolverResult` 表达 solver status、iterations、raw backend code、diagnostics 和可选 final gradient stats。它不包含 final objective value。Python result 需要 solution vector 和 objective-level output，因此新 service 增加 `OptimizationResult`，不要把问题级字段硬塞进 legacy `SolverResult`。

### 6. optimizer backend、Newton line search 和 Newton sparse solver 分散在不同分支里

当前 optimizer 分发集中在 `EnergyOptimizer::minimize(...)` 的 `SolverType` switch-like `if/else`，部分 caller 又直接调用 `minimizeUsingKnitro(...)`。Newton 内部 sparse linear solver 的选择则写在 `NewtonSolver.h/.cpp` 的 `PGO_HAS_MKL` / `PGO_HAS_ORIG_PARDISO` 条件编译里。

另外，Newton line-search mode 已经有 service-level `LineSearchMethod`，但具体执行仍在 `NewtonSolver::runLineSearchStep(...)` 里用 `LSM_GOLDEN` / `LSM_BRENTS` / `LSM_BACKTRACK` / `LSM_SIMPLE` 分支。`StepStrategy` 已经抽出了“line-search / static-damping / full-step”这一级 subiteration mode，line-search 内部的 alpha policy 也应该同样收口。

这导致几个问题：

- 上层 service 如果继续复用 `SolverType`，未来 IPOPT / Knitro / Newton options 会继续塞进一个宽签名；
- `NewtonSolver.h` 直接暴露 MKL / Orig Pardiso wrapper 类型，调用侧不关心的编译条件泄漏进核心 solver class。
- `runLineSearchStep(...)` 同时负责 feasible alpha、IPC active-set freeze、trial energy、line-search method dispatch、diagnostics，职责偏胖，后续新增或替换 line-search mode 会继续长分支。

新设计把 optimizer backend selection 收口到 `OptimizationBackend`；把 Newton line-search method selection 收口到 `NewtonLineSearchPolicy` factory；把 Newton sparse solver selection 收口到 `NewtonSparseSolverBackend` factory。`if/#if` 不要求完全消失，但只能出现在很小的 adapter/factory 实现里。

## 非目标

- 不新增 C++ `solveNewton(...)` convenience API；C++ 长期入口直接使用 `minimize(problem, x0, NewtonOptions)`。
- 不实现 Python `pypgo.solver.minimize` 通用入口；M3 只暴露 `solve_newton`。
- 不实现 IPOPT / Knitro service backend；只预留 type shape。
- 不实现 runtime plugin registry；M3 只需要 typed overload + 小型 factory，不引入全局 backend 注册系统。
- 不把 `OptimizationBackend`、`NewtonLineSearchPolicy`、`NewtonSparseSolverBackend` 或 sparse solver selection 暴露到 Python。
- 不新增 Wolfe / strong Wolfe / Goldstein line search，也不改变 golden / Brent / backtracking / simple 的默认数值参数。
- 不把自定义 Python line-search callback 暴露出去。
- 不迁移 `EnergyOptimizer::minimize` 的所有 caller；它保留到后续独立 plan。
- 不绑定 `stepFunc` callback。
- 不暴露 stateful `pypgo.solver.NewtonSolver` class。
- 不修改 `NewtonSolver` 内部 `SolveContext` / `StepStrategy` / line-search 数值流程；只允许把 line-search method dispatch 和 sparse linear solver 成员替换成 adapter。
- 不迁移 `runIPCSim` 内部 Newton 调用到新 service；内部 runtime solver 迁移留给后续。

## 设计决策

### 1. C++ 长期边界是 optimization service，不是 `solveNewton`

新增文件：

- `src/core/nonlinearOptimization/optimizationService.h`
- `src/core/nonlinearOptimization/optimizationService.cpp`

核心 public API：

```cpp
namespace pgo::NonlinearOptimization
{

enum class LineSearchMethod
{
  Golden,
  Brents,
  Backtrack,
  Simple
};

enum class NewtonSparseSolverKind
{
  Auto,
  EigenSimplicialLDLT,
  MKLPardiso,
  OrigPardiso
};

struct NewtonSparseSolverOptions
{
  NewtonSparseSolverKind kind = NewtonSparseSolverKind::Auto;
};

struct SolverControl
{
  int maxIterations = 50;
  double tolerance = 1e-6;
  int verbose = 0;
};

struct NewtonOptions
{
  SolverControl control;
  bool damping = true;
  LineSearchMethod lineSearch = LineSearchMethod::Backtrack;
  NewtonSparseSolverOptions sparseSolver;
};

struct FixedVariables
{
  std::vector<int> dofs;
  std::optional<EigenSupport::VXd> values;
};

struct BoxBounds
{
  EigenSupport::VXd lower;
  EigenSupport::VXd upper;
};

struct NonlinearConstraints
{
  ConstraintFunctions_const_p functions;
  EigenSupport::VXd lower;
  EigenSupport::VXd upper;
};

struct OptimizationProblem
{
  PotentialEnergy_const_p energy;
  std::optional<FixedVariables> fixedVariables;
  std::optional<BoxBounds> bounds;                  // future IPOPT/Knitro
  std::optional<NonlinearConstraints> constraints;  // future IPOPT/Knitro
};

struct OptimizationResult
{
  SolverResult solver;
  EigenSupport::VXd x;

  bool hasFinalObjective = false;
  double finalObjective = 0.0;

  std::optional<EigenSupport::VXd> lambda;            // future constrained backend
  std::optional<EigenSupport::VXd> constraintValues;  // future constrained backend
};

OptimizationResult minimize(
  const OptimizationProblem &problem,
  EigenSupport::ConstRefVecXd x0,
  const NewtonOptions &options);

}  // namespace pgo::NonlinearOptimization
```

`optimizationService.h` 不应该暴露 `NewtonSolver::SolverParam`、`NewtonSolver::LineSearchMethod`、raw pointer fixed values 或 mutable solver internals。`optimizationService.cpp` 内部负责把 service-level `LineSearchMethod` 翻译成 legacy `NewtonSolver::LineSearchMethod`。

Header ownership：

- `NewtonSparseSolverKind` / `NewtonSparseSolverOptions` 定义在 `newtonSparseSolverBackend.h`，`optimizationService.h` include 该 header 并把 options 放进 `NewtonOptions`。
- `NewtonSparseSolverBackend` interface 和 factory 也定义在 `newtonSparseSolverBackend.h`，但它们只给 `NewtonSolver` / service implementation 使用，不绑定到 Python。
- `NewtonLineSearchPolicy` interface 和 factory 定义在 `newtonLineSearchPolicy.h`，只给 `NewtonSolver` implementation 使用；service public header 只暴露 `LineSearchMethod` enum，不暴露 policy class。
- `optimizationBackend.h` include `optimizationService.h`，因此只依赖 service-level problem/result/options types；`optimizationService.h` 不 include `optimizationBackend.h`，避免 service public header 反向暴露 backend implementation。

### 2. optimizer backend 是 typed adapter，不再扩展 `SolverType` 宽签名

新增内部 backend 层：

- `src/core/nonlinearOptimization/optimizationBackend.h`
- `src/core/nonlinearOptimization/optimizationBackend.cpp`

核心形状：

```cpp
class OptimizationBackend
{
public:
  virtual ~OptimizationBackend() = default;

  virtual OptimizationResult solve(
    const OptimizationProblem &problem,
    EigenSupport::ConstRefVecXd x0) = 0;
};

class NewtonOptimizationBackend final : public OptimizationBackend
{
public:
  explicit NewtonOptimizationBackend(NewtonOptions options);

  OptimizationResult solve(
    const OptimizationProblem &problem,
    EigenSupport::ConstRefVecXd x0) override;
};
```

`minimize(problem, x0, NewtonOptions)` 的实现只做一件事：

```cpp
OptimizationResult minimize(
  const OptimizationProblem &problem,
  EigenSupport::ConstRefVecXd x0,
  const NewtonOptions &options)
{
  return NewtonOptimizationBackend(options).solve(problem, x0);
}
```

未来 IPOPT / Knitro 不通过 `SolverType` 加新分支，而是增加 typed overload：

```cpp
OptimizationResult minimize(
  const OptimizationProblem &problem,
  EigenSupport::ConstRefVecXd x0,
  const IpoptOptions &options);

OptimizationResult minimize(
  const OptimizationProblem &problem,
  EigenSupport::ConstRefVecXd x0,
  const KnitroOptions &options);
```

并各自落到 `IpoptOptimizationBackend` / `KnitroOptimizationBackend`。这样 backend-specific options 不需要塞进一个 union-like 参数结构，也不会把 IPOPT/Knitro 的 config file、feasibility tolerance、callback 等混入 Newton API。

M3 只实现 `NewtonOptimizationBackend`。`IpoptOptimizationBackend` / `KnitroOptimizationBackend` 只作为未来设计方向写在 plan 里，不新增空壳类。

### 3. Newton line-search policy 是 Newton 内部 alpha policy

新增：

- `src/core/nonlinearOptimization/newtonLineSearchPolicy.h`
- `src/core/nonlinearOptimization/newtonLineSearchPolicy.cpp`

核心形状：

```cpp
enum class NewtonLineSearchKind
{
  Golden,
  Brents,
  Backtrack,
  Simple
};

struct NewtonLineSearchInput
{
  EigenSupport::ConstRefVecXd x;
  EigenSupport::ConstRefVecXd direction;  // already feasible-alpha scaled
  EigenSupport::ConstRefVecXd gradient;
  double currentEnergy = 0.0;
  double trialEnergyAtAlphaOne = 0.0;
  int maxIterations = 0;
};

struct NewtonLineSearchResult
{
  double alpha = 1.0;
  double energy = 0.0;
  int iterations = 0;
};

class NewtonLineSearchPolicy
{
public:
  virtual ~NewtonLineSearchPolicy() = default;

  virtual double maxProbeAlpha() const = 0;
  virtual NewtonLineSearchResult search(const NewtonLineSearchInput &input) = 0;
};

std::unique_ptr<NewtonLineSearchPolicy> createNewtonLineSearchPolicy(
  NewtonLineSearchKind kind,
  int numDofs,
  LineSearch::EvaluateFunction evaluate);
```

Policy 只负责从 feasible-scaled Newton direction 里选择 line-search alpha，不负责：

- `energy->computeMaxStepLimit(...)`；
- `LineSearchAwareEnergy::beginLineSearch/endLineSearch` active-set freeze；
- trial full-step energy 的第一次 evaluation；
- `SolveDiagnostics::recordLineSearch(...)`；
- `StepTooSmall` / `LineSearchFailed` / `NonFinite` status 处理。

这些仍由 `NewtonSolver::runLineSearchStep(...)` 管，因为它们是 Newton step acceptance lifecycle，不是某个 line-search 算法自己的事情。

`NewtonLineSearchPolicy::maxProbeAlpha()` 用来保留现有 active-set freeze 规则：

```text
Backtracking / Simple -> 1.0
Golden / Brent        -> infinity
```

`runLineSearchStep(...)` 使用 `policy.maxProbeAlpha()` 判断是否允许冻结 `LineSearchAwareEnergy`。这等价于当前 `lineSearchMethodMaxAlpha(...)` 的行为，不改变 IPC / line-search-aware 能量的有效 alpha window。

`NewtonLineSearchKind` 是内部 enum。`optimizationService.cpp` 继续把 service-level `LineSearchMethod` 翻译成 legacy `NewtonSolver::LineSearchMethod`；`NewtonSolver.cpp` 再把 legacy enum 翻译成 `NewtonLineSearchKind` 来创建 policy。这样不需要把 `NewtonLineSearchPolicy` 纳入 public service API，也不要求 Python 暴露新的 policy 类型。

四个 concrete policy：

- `GoldenLineSearchPolicy`：包装现有 `LineSearch::golden(...)`。
- `BrentLineSearchPolicy`：包装现有 `LineSearch::BrentsMethod(...)`。
- `BacktrackingLineSearchPolicy`：包装现有 `LineSearch::backtrackingWithInitialValue(...)`，保持 `kBacktrackArmijo` / `kBacktrackShrink` / `kBacktrackInitAlpha` 默认值。
- `SimpleLineSearchPolicy`：保留现有 simple halving loop 和 `kSimpleLineSearchMaxIter`。

M3 不改变 service-level `LineSearchMethod` enum，也不把 policy 类型暴露到 Python。Python 仍然只接受：

```text
"golden" | "brents" | "backtrack" | "simple"
```

### 4. Newton sparse solver backend 是 Newton 内部 adapter

新增：

- `src/core/nonlinearOptimization/newtonSparseSolverBackend.h`
- `src/core/nonlinearOptimization/newtonSparseSolverBackend.cpp`

核心形状：

```cpp
class NewtonSparseSolverBackend
{
public:
  virtual ~NewtonSparseSolverBackend() = default;

  virtual void analyze(const EigenSupport::SpMatD &A) = 0;
  virtual bool factorize(const EigenSupport::SpMatD &A) = 0;
  virtual bool solve(const EigenSupport::SpMatD &A, double *x, double *rhs) = 0;
  virtual const char *name() const = 0;
};

std::unique_ptr<NewtonSparseSolverBackend> createNewtonSparseSolverBackend(
  const NewtonSparseSolverOptions &options,
  const EigenSupport::SpMatD &A);
```

`NewtonSolver` 改成只持有：

```cpp
std::unique_ptr<NewtonSparseSolverBackend> solver;
NewtonSparseSolverOptions sparseSolverOptions;
```

`NewtonSolver.h` 不再 include `EigenMKLPardisoSupport.h` / `EigenOrigPardisoSupport.h`，也不再按宏声明三种不同的 solver member 类型。`PGO_HAS_MKL` / `PGO_HAS_ORIG_PARDISO` 只允许出现在 `newtonSparseSolverBackend.cpp` 里。

`NewtonSparseSolverKind::Auto` 保持当前默认优先级：

```text
OrigPardiso if PGO_HAS_ORIG_PARDISO
else MKLPardiso if PGO_HAS_MKL
else EigenSimplicialLDLT
```

显式请求不可用 backend 时抛 `std::invalid_argument`，不要静默 fallback。例如没有 MKL 时请求 `MKLPardiso`，应该直接报错；只有 `Auto` 可以 fallback 到可用实现。

M3 Python 不暴露 `NewtonSparseSolverKind`。Python `solve_newton(...)` 始终使用 `Auto`，这样第一版用户不需要理解线性求解器依赖。

### 5. `FixedVariables` 是 first-class compact helper

`BoxBounds` 是通用 optimization 表达，但 first API 主要服务 Newton 和 Python fixed DOF。M3 使用 `FixedVariables` 表达固定变量：

```cpp
FixedVariables{
  .dofs = {0, 1, 2},
  .values = std::nullopt
}
```

语义：

- `dofs.empty()`：无固定变量。
- `values == std::nullopt`：固定到 `x0[dofs]`。
- `values.has_value()`：固定到显式给定值；`values->size()` 必须等于 `dofs.size()`。
- `dofs` 输入顺序允许任意排列，但 service 在传给 legacy `NewtonSolver` 前必须 canonicalize 成升序，因为 `EigenSupport::removeRows` 依赖 sorted fixed DOF。
- canonicalize 时 fixed value 必须跟随对应 dof 一起排序。
- duplicate dof、越界 dof、values 长度不匹配都抛 `std::invalid_argument`。

M3 中 `BoxBounds` 和 `NonlinearConstraints` 只是未来 backend 的问题结构；`NewtonOptimizationBackend` 遇到它们必须明确抛 `std::invalid_argument`，而不是静默忽略。

### 6. Newton backend 只支持 unconstrained objective + fixed variables

M3 的 Newton service 支持：

```text
minimize:
  objective E(x)
  initial x0
  optional FixedVariables
  NewtonOptions
```

不支持：

- inequality `BoxBounds`；
- nonlinear constraints；
- lambda / constraint values output；
- alternative subiteration modes；
- step callbacks；
- reusable symbolic factorization state。

这些限制必须在 `NewtonOptimizationBackend::solve(...)` 入口处检查并报错。

### 7. Python `solve_newton` 是 wrapper，不是 C++ API 镜像

Python public API：

```python
@dataclass(frozen=True)
class NewtonOptions:
    max_iter: int = 50
    tol: float = 1e-6
    damping: bool = True
    line_search: str = "backtrack"
    verbose: int = 0


def solve_newton(
    energy: pgo.energy.PotentialEnergy,
    x0: np.ndarray,
    *,
    max_iter: int = 50,
    tol: float = 1e-6,
    fixed_dofs: Sequence[int] | None = None,
    fixed_values: np.ndarray | Sequence[float] | None = None,
    damping: bool = True,
    line_search: str = "backtrack",
    verbose: int = 0,
) -> SolverResult:
    ...
```

Python wrapper 构造 `OptimizationProblem` + `NewtonOptions`，然后走同一个 C++ `minimize(..., NewtonOptions)` binding。Python 不暴露 C++ `OptimizationProblem`，也不暴露通用 `minimize` stub。

`line_search` 支持：

```text
"golden" | "brents" | "backtrack" | "simple"
```

binding 内映射到 service-level `LineSearchMethod`，再由 service cpp 映射到 legacy Newton enum。

### 8. Result 分层：`SolverResult` 保持 solver 语义，`OptimizationResult` 拥有问题级输出

C++ `SolverResult` 不新增 `finalEnergy`。它继续表示：

- `status`
- `iterations`
- `rawStatusCode`
- `hasFinalGradientStats`
- `finalGradientNorm`
- `finalGradientMaxNorm`
- `diagnostics`

新 `OptimizationResult` 表示：

- final `x`
- underlying `SolverResult`
- optional `finalObjective`
- future `lambda` / `constraintValues`

M3 service 在 legacy solver 返回后，对返回的 `x` 做一次 final objective evaluation：

```cpp
const double f = problem.energy->func(result.x);
if (std::isfinite(f)) {
  result.hasFinalObjective = true;
  result.finalObjective = f;
}
```

若 final objective 非 finite，则 `hasFinalObjective=false`。这次额外 evaluation 是 service contract 的一部分，不改变 Newton 数值路径。

Python `SolverResult` 是 immutable value object，字段建议为：

```python
@dataclass(frozen=True)
class SolverResult:
    x: np.ndarray
    status: SolveStatus
    converged: bool
    iterations: int
    raw_status_code: int
    final_objective: float | None
    final_gradient_norm: float | None
    final_gradient_max_norm: float | None
    diagnostics: SolveDiagnostics
```

`final_gradient_*` 只有 C++ `solver.hasFinalGradientStats` 为真时才是 `float`，否则为 `None`。不要再承诺失败路径总有 final gradient stats。

### 9. SolveStatus mapping table

Python `SolveStatus(IntEnum)` 与 C++ `SolveStatus` 保持 int parity：

| Python name | C++ value | int |
| --- | --- | ---: |
| `CONVERGED` | `SolveStatus::Converged` | 0 |
| `MAX_ITERATIONS` | `SolveStatus::MaxIterations` | 1 |
| `LINE_SEARCH_FAILED` | `SolveStatus::LineSearchFailed` | 2 |
| `STEP_TOO_SMALL` | `SolveStatus::StepTooSmall` | 3 |
| `NON_FINITE` | `SolveStatus::NonFinite` | 4 |
| `LINEAR_SOLVE_FAILED` | `SolveStatus::LinearSolveFailed` | 5 |
| `EXTERNAL_SOLVER_FAILURE` | `SolveStatus::ExternalSolverFailure` | 100 |
| `UNSUPPORTED_BACKEND` | `SolveStatus::UnsupportedBackend` | 101 |

Binding test 必须覆盖所有值。

### 10. Error handling

C++ service 抛 `std::invalid_argument`：

- `problem.energy == nullptr`
- `x0.size() != problem.energy->getNumDOFs()`
- `options.control.maxIterations < 0`
- `options.control.tolerance < 0`
- `options.control.verbose < 0`
- `FixedVariables.values.size() != FixedVariables.dofs.size()`
- fixed dof 越界
- fixed dof 重复
- `problem.bounds.has_value()` with `NewtonOptions`
- `problem.constraints.has_value()` with `NewtonOptions`
- `BoxBounds.lower/upper` shape invalid（future helper 可先验证，但 Newton 入口仍拒绝）
- `NonlinearConstraints.lower/upper` shape invalid（future helper 可先验证，但 Newton 入口仍拒绝）
- `NonlinearConstraints.functions == nullptr`（future helper 可先验证，但 Newton 入口仍拒绝）

Python binding 将这些自然映射为 `ValueError`。未知 `line_search` string 也抛 `ValueError`。

## 目标 C++ API

```cpp
// nonlinearOptimization/optimizationService.h

namespace pgo::NonlinearOptimization
{

enum class LineSearchMethod
{
  Golden,
  Brents,
  Backtrack,
  Simple
};

enum class NewtonSparseSolverKind
{
  Auto,
  EigenSimplicialLDLT,
  MKLPardiso,
  OrigPardiso
};

struct NewtonSparseSolverOptions
{
  NewtonSparseSolverKind kind = NewtonSparseSolverKind::Auto;
};

struct SolverControl
{
  int maxIterations = 50;
  double tolerance = 1e-6;
  int verbose = 0;
};

struct NewtonOptions
{
  SolverControl control;
  bool damping = true;
  LineSearchMethod lineSearch = LineSearchMethod::Backtrack;
  NewtonSparseSolverOptions sparseSolver;
};

struct FixedVariables
{
  std::vector<int> dofs;
  std::optional<EigenSupport::VXd> values;
};

struct BoxBounds
{
  EigenSupport::VXd lower;
  EigenSupport::VXd upper;
};

struct NonlinearConstraints
{
  ConstraintFunctions_const_p functions;
  EigenSupport::VXd lower;
  EigenSupport::VXd upper;
};

struct OptimizationProblem
{
  PotentialEnergy_const_p energy;
  std::optional<FixedVariables> fixedVariables;
  std::optional<BoxBounds> bounds;
  std::optional<NonlinearConstraints> constraints;
};

struct OptimizationResult
{
  SolverResult solver;
  EigenSupport::VXd x;
  bool hasFinalObjective = false;
  double finalObjective = 0.0;
  std::optional<EigenSupport::VXd> lambda;
  std::optional<EigenSupport::VXd> constraintValues;
};

OptimizationResult minimize(
  const OptimizationProblem &problem,
  EigenSupport::ConstRefVecXd x0,
  const NewtonOptions &options);

}  // namespace pgo::NonlinearOptimization
```

不新增：

- `solveNewton.h/.cpp`
- C++ `solveNewton(...)`
- Python `minimize(...)`
- Python `NewtonSolver` class

## Python API 定稿草案

```python
import pypgo as pgo
import numpy as np

energy = pgo.energy.QuadraticEnergy(np.eye(3))
u0 = np.array([1.0, 2.0, 3.0])

result = pgo.solver.solve_newton(
    energy,
    x0=u0,
    fixed_dofs=[2],
    fixed_values=None,       # fixes x[2] to u0[2] == 3.0
)

assert isinstance(result, pgo.solver.SolverResult)
assert result.status == pgo.solver.SolveStatus.CONVERGED
assert result.converged
assert result.x.shape == u0.shape
assert np.allclose(u0, [1.0, 2.0, 3.0])  # x0 is not mutated
assert result.x[2] == 3.0
```

`pypgo.solver` M3 public surface：

```text
pypgo.solver
  SolveStatus
  SolveDiagnostics
  SolverResult
  NewtonOptions
  solve_newton
```

不暴露：

```text
NewtonSolver
SolverParam
EnergyOptimizer
OptimizationProblem
OptimizationResult
FixedVariables
BoxBounds
NonlinearConstraints
OptimizationBackend
NewtonSparseSolverBackend
NewtonLineSearchPolicy
NewtonSparseSolverKind
NewtonSparseSolverOptions
SST_*
LSM_*
minimize
```

## File Map

### 新增

- `src/core/nonlinearOptimization/optimizationService.h`
- `src/core/nonlinearOptimization/optimizationService.cpp`
- `src/core/nonlinearOptimization/optimizationBackend.h`
- `src/core/nonlinearOptimization/optimizationBackend.cpp`
- `src/core/nonlinearOptimization/newtonLineSearchPolicy.h`
- `src/core/nonlinearOptimization/newtonLineSearchPolicy.cpp`
- `src/core/nonlinearOptimization/newtonSparseSolverBackend.h`
- `src/core/nonlinearOptimization/newtonSparseSolverBackend.cpp`
- `src/python/pypgo/bindings/solver_bindings.cpp`
- `pypgo/solver.py`
- `tests/src/core/optimizationService_gtest.cpp`
- `tests/pypgo/test_solver.py`

### 修改

- `src/core/nonlinearOptimization/CMakeLists.txt`：编入 `optimizationService.*`、`optimizationBackend.*`、`newtonLineSearchPolicy.*`、`newtonSparseSolverBackend.*`。
- `src/core/nonlinearOptimization/NewtonSolver.h/.cpp`：把 line-search method dispatch 替换成 `NewtonLineSearchPolicy`，把 concrete sparse solver member 替换成 `NewtonSparseSolverBackend`，并把 MKL / Orig Pardiso 条件编译移出 header。
- `tests/src/core/CMakeLists.txt`：新增 `optimizationService_gtest`。
- `src/python/pypgo/CMakeLists.txt`：编入 `solver_bindings.cpp`，并确保 `pypgo_core` 显式链接 `nonlinearOptimization`。
- `src/python/pypgo/bindings/module.cpp`：注册 solver bindings。
- `pypgo/__init__.py`：把 `"solver"` 加入 lazy public modules。
- `tests/pypgo/test_package_scaffold.py`：更新 `__all__` 期望。
- `plan/python_api_migration/api_coverage.md`：更新 Newton solver / minimize / solver status 条目。

### 不动

- `NewtonSolver` 的数值流程、line-search 默认数值参数、damping 策略和默认 sparse solver 行为。
- `minimizeEnergy.h/.cpp` public API（不绑定，不迁移）。
- `lineSearch.h/.cpp`、`lineSearchAwareEnergy.h`。
- `IpoptOptimizer*`、`knitroOptimizer*`。
- `runIPCSim` 内部 solver 调用。

## Task 拆分

### Task S1: Solver status/result parity audit

- 读 `solverResult.h/.cpp`、`solveDiagnostics.h`、`NewtonSolver.cpp`。
- 确认 `SolveStatus` table 与当前 C++ enum 完全一致。
- 确认 `SolverResult` 当前没有 final objective 字段，且 final gradient stats 由 `hasFinalGradientStats` gate。
- 确认 `fixedValues == nullptr` 对非空 fixed DOF 不安全，不是 fallback 语义。
- 确认 `EigenSupport::removeRows` 对 fixed DOF 使用 `std::binary_search`，service 必须排序 fixed DOF。
- 产出：如果审计发现 plan 和代码事实不一致，先修 plan，不写实现代码。

### Task S2: C++ optimization service + tests

- 新增 `optimizationService.h/.cpp`。
- 新增 `optimizationBackend.h/.cpp`：
  - 定义内部 `OptimizationBackend` interface；
  - 实现 `NewtonOptimizationBackend`；
  - `minimize(problem, x0, NewtonOptions)` 委托到 `NewtonOptimizationBackend(options).solve(...)`；
  - 不引入新的 `SolverType` enum，不把 backend choice 和 backend-specific options 塞进一个宽签名。
- 新增 `newtonLineSearchPolicy.h/.cpp`：
  - 定义内部 `NewtonLineSearchKind`；
  - 定义 `NewtonLineSearchInput` / `NewtonLineSearchResult`；
  - 定义 `NewtonLineSearchPolicy` interface；
  - 实现 `GoldenLineSearchPolicy`、`BrentLineSearchPolicy`、`BacktrackingLineSearchPolicy`、`SimpleLineSearchPolicy`；
  - Golden / Brent / Backtracking 复用现有 `LineSearch` class，不重写算法；
  - Simple policy 搬出现有 halving loop，不改变 `kSimpleLineSearchMaxIter`；
  - 实现 `createNewtonLineSearchPolicy(kind, numDofs, evaluate)`；
  - `maxProbeAlpha()` 保持当前 active-set freeze 判定：Backtracking/Simple 为 `1.0`，Golden/Brent 为 `infinity`。
- 新增 `newtonSparseSolverBackend.h/.cpp`：
  - 定义 `NewtonSparseSolverKind` / `NewtonSparseSolverOptions`；
  - 定义 `NewtonSparseSolverBackend` interface；
  - 实现 Eigen `SimplicialLDLT` adapter；
  - 在编译条件允许时实现 MKL Pardiso adapter；
  - 在编译条件允许时实现 Orig Pardiso adapter；
  - 实现 `createNewtonSparseSolverBackend(options, A)`；
  - `Auto` 保持当前 Orig Pardiso -> MKL Pardiso -> Eigen fallback 的默认优先级；
  - 显式请求不可用 backend 时抛 `std::invalid_argument`。
- 修改 `NewtonSolver.h/.cpp`：
  - constructor 增加 source-compatible 的 optional `NewtonSparseSolverOptions` 参数，默认 `Auto`；
  - header 不再 include `EigenMKLPardisoSupport.h` 或 `EigenOrigPardisoSupport.h`；
  - member 新增 `std::unique_ptr<NewtonLineSearchPolicy>`，替代 `LineSearchHandle` / `nativeLineSearch` 作为 line-search method dispatch 点；
  - member 从 preprocessor-specific concrete solver type 改成 `std::unique_ptr<NewtonSparseSolverBackend>`；
  - constructor 中把 legacy `solverParam.lsm` 翻译成内部 `NewtonLineSearchKind`，并创建 policy；
  - `runLineSearchStep(...)` 保留 feasible alpha、`LineSearchAwareEnergy` scope、trial energy、diagnostics、status handling，只把 alpha search 委托给 policy；
  - `makeLinearSolver(A11)` 改为调用 factory；
  - `solveReducedNewtonDirection(...)` 只调用 backend 的 `factorize/solve`；
  - 保持 `A11` 构造、fixed DOF reduction、damping、line-search 数值行为、diagnostics 行为不变。
- 实现 service-level types：
  - `LineSearchMethod`
  - `NewtonSparseSolverKind`
  - `NewtonSparseSolverOptions`
  - `SolverControl`
  - `NewtonOptions`
  - `FixedVariables`
  - `BoxBounds`
  - `NonlinearConstraints`
  - `OptimizationProblem`
  - `OptimizationResult`
- 实现 `minimize(problem, x0, NewtonOptions)`：
  - validate energy and x0 size；
  - reject `bounds` and `constraints` for Newton M3；
  - normalize fixed variables；
  - copy `x0` to owned mutable `VXd x`；
  - translate service `NewtonOptions` to legacy `NewtonSolver::SolverParam`；
  - construct `NewtonSolver(x.data(), sp, problem.energy, fixedDofs, fixedValues.data(), options.sparseSolver)`；
  - call `solver.solve(x.data(), ...)`；
  - compute `finalObjective` once from returned `x` if finite；
  - return `OptimizationResult` with owned `x`。
- Add `optimizationService_gtest.cpp`:
  - warm-start quadratic solve does not mutate `x0`；
  - result `x.data() != x0.data()`；
  - implicit fixed values use `x0[dofs]`；
  - explicit fixed values override `x0[dofs]`；
  - unsorted `fixed_dofs` are accepted and canonicalized；
  - duplicate fixed dof throws；
  - out-of-range fixed dof throws；
  - values length mismatch throws；
  - negative `maxIterations` / `tolerance` / `verbose` throws；
  - `bounds` with Newton throws；
  - `constraints` with Newton throws；
  - all four line-search methods are accepted at service level；
  - line-search policies preserve current accepted alpha / accepted energy on a deterministic quadratic fixture；
  - Backtracking/Simple report `maxProbeAlpha() == 1.0`；
  - Golden/Brent report non-finite `maxProbeAlpha()`；
  - default sparse solver option is `Auto` and preserves current fallback behavior；
  - explicit unavailable sparse solver backend throws when the build lacks it；
  - successful solve sets `hasFinalObjective == true`。

### Task S3: Python solver value objects

- Add `pypgo/solver.py`:
  - `SolveStatus(IntEnum)` with exact parity values；
  - `SolveDiagnostics` frozen dataclass；
  - `SolverResult` frozen dataclass；
  - `NewtonOptions` frozen dataclass；
  - `solve_newton` wrapper signature and docstring。
- Python `SolverResult.x` must be a NumPy array owned independently of C++ solver internals.
- `final_objective`, `final_gradient_norm`, `final_gradient_max_norm` are `float | None`。
- Update `pypgo/__init__.py` and package scaffold tests to include `solver`。

### Task S4: Python binding for `solve_newton`

- Add `solver_bindings.cpp` and register it from `module.cpp`。
- Binding input:
  - `energy: pypgo.energy.PotentialEnergy` from Energy plan E4；
  - `x0: np.ndarray` shape `(n,)`, dtype `float64`；
  - `fixed_dofs: list[int] | None`；
  - `fixed_values: np.ndarray | None`；
  - `max_iter`, `tol`, `damping`, `line_search`, `verbose`。
- Binding behavior:
  - convert `x0` to service-owned or const mapped vector without mutating Python input；
  - convert fixed values to `FixedVariables` with `values=nullopt` when Python passes `None`；
  - parse line-search strings to service `LineSearchMethod`；
  - call C++ `minimize(problem, x0, NewtonOptions)`；
  - convert `OptimizationResult` to Python `SolverResult`。
- Error behavior:
  - invalid shapes / dtype / unknown line search / invalid fixed variables raise `ValueError`；
  - unsupported `bounds` / `constraints` are not reachable from Python M3 API。

### Task S5: End-to-end tests + docs cleanup

- Add `tests/pypgo/test_solver.py`:
  - `test_newton_solves_quadratic__warm_start`；
  - `test_newton_solves_quadratic__implicit_fixed_values_from_x0`；
  - `test_newton_solves_quadratic__explicit_fixed_values`；
  - `test_x0_not_mutated`；
  - `test_result_x_is_independent_numpy_array`；
  - `test_line_search_keywords`；
  - `test_status_roundtrip_for_all_values`；
  - `test_final_gradient_stats_are_optional`；
  - `test_invalid_fixed_values_length_raises`；
  - `test_invalid_line_search_raises`；
  - `test_verbose_does_not_crash`。
- Update `api_coverage.md`:
  - Newton solver maps to `pypgo.solver.solve_newton`；
  - general minimize remains future work；
  - solver service C++ boundary is `optimizationService.h`。

## 验收标准

- `optimizationService_gtest` 全部通过。
- `python -m pytest tests/pypgo/test_solver.py` 全部通过。
- `solve_newton(energy, x0)` 不修改 `x0`。
- `fixed_values=None` 固定到 `x0[fixed_dofs]`。
- `result.x` 是独立 NumPy array。
- Python `final_gradient_norm` / `final_gradient_max_norm` 在 C++ 没有 final stats 时为 `None`。
- `pypgo.solver` 公开名集合不出现 `NewtonSolver`、`SolverParam`、`EnergyOptimizer`、`OptimizationProblem`、`FixedVariables`、`BoxBounds`、`NonlinearConstraints`、`OptimizationBackend`、`NewtonLineSearchPolicy`、`NewtonSparseSolverBackend`、`NewtonSparseSolverKind`、`NewtonSparseSolverOptions`、`SST_*`、`LSM_*`、`minimize`。
- `optimizationService.h` 不暴露 legacy raw pointer fixed values 或 `NewtonSolver::SolverParam`。
- `optimizationService.cpp` 不使用 `EnergyOptimizer::SolverType` 做 backend 分发；Newton 路径通过 `NewtonOptimizationBackend`。
- `runLineSearchStep(...)` 不再用 `LSM_GOLDEN` / `LSM_BRENTS` / `LSM_BACKTRACK` / `LSM_SIMPLE` 分支执行具体算法，而是委托给 `NewtonLineSearchPolicy`。
- `LineSearchAwareEnergy` active-set freeze 规则不变：只有 `policy.maxProbeAlpha() <= maxValidLineSearchAlpha()` 时冻结。
- Backtracking 的 `kBacktrackArmijo` / `kBacktrackShrink` / `kBacktrackInitAlpha` 默认值不变。
- `NewtonSolver.h` 不 include `EigenMKLPardisoSupport.h` 或 `EigenOrigPardisoSupport.h`，也不声明 preprocessor-specific concrete solver member。
- Newton sparse solver 的 `PGO_HAS_MKL` / `PGO_HAS_ORIG_PARDISO` 条件编译集中在 `newtonSparseSolverBackend.cpp`。
- `NewtonSparseSolverKind::Auto` 的默认选择保持旧行为：Orig Pardiso 优先于 MKL Pardiso，二者都不可用时使用 Eigen `SimplicialLDLT`。
- `runIPCSim` 内部 Newton 调用不受本 service 引入影响。

## Dependencies & Execution Order

### 外部依赖

- **Energy plan E0**（Hessian API rename）不是 C++ service 类型设计的直接依赖，但如果 E0 正在进行，S2 实现应基于当前主干可编译状态。
- **Energy plan E4**（`pypgo.energy.PotentialEnergy` handle）是 Python S4 的前置依赖。
- **Energy plan E5**（`QuadraticEnergy` binding）是 Python solver tests 的前置依赖。
- **Energy plan E6**（`EnergySet` binding）是 multi-energy end-to-end solver 示例的前置依赖，但不是最小 `QuadraticEnergy` solver test 的硬依赖。
- Contact plan 不是本 plan 的前置依赖；Contact C5 可在 solver 完成后用 `pgo.solver.solve_newton(EnergySet([...]))` 做联合验证。

### 内部任务依赖

```text
S1 (audit)
  └─ S2 (C++ optimization service)
       ├─ S3 (Python value objects)
       └─ S4 (Python binding; depends on Energy E4)
            └─ S5 (Python E2E tests; depends on Energy E5)
```

### 推荐顺序

S1 -> S2 -> S3 -> S4 -> S5。

S2 可与 Energy E1-E3 并行；S4/S5 必须等 Energy E4/E5。

## 输出（供下游使用）

- C++:
  - `OptimizationProblem`
  - `FixedVariables`
  - `NewtonOptions`
  - `NewtonSparseSolverKind`
  - `NewtonSparseSolverOptions`
  - `OptimizationResult`
  - `minimize(problem, x0, NewtonOptions)`
- Python:
  - `pypgo.solver.solve_newton`
  - `pypgo.solver.SolverResult`
  - `pypgo.solver.SolveDiagnostics`
  - `pypgo.solver.SolveStatus`
  - `pypgo.solver.NewtonOptions`
- Future:
  - IPOPT / Knitro backends add typed overloads using the same `OptimizationProblem` / `OptimizationResult` shape.
  - Python `pypgo.solver.minimize` is designed only when bounds/constraints and backend-specific options are actually implemented.
