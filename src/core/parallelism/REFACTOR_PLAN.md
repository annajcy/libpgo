# parallelism 跨线程控制重构计划

## 模块上下文

`src/core/parallelism` 为 core 层提供 `parallelFor`、`parallelFor3D`、TBB/串行后端、嵌套 BLAS
线程策略、TBB 并行度限制和进程 CPU affinity。仓库根目录当前没有高层 `REFACTOR_PLAN.md`，本计划以该模块及其直接调用面为边界。

当前外部调用关系包括：

- `implicitSurface` 和 `solidDeformationModel` 调用 `parallelFor`。
- Python binding 暴露 worker limit、CPU affinity limit 和 runtime info。
- `EigenOrigPardisoSupport` 和 `KnitroSolverWrapper` 读取 `workerLimit()`，把它当成 solver 线程数。
- benchmark 和 gtest 使用 `ScopedWorkerLimit` 临时改变进程级配置。

项目使用 C++20。重构目标是让 TBB 控制在多调用线程下具有明确语义，同时保持 core 不依赖 OpenMP。

## 当前结构

| 文件 | 当前职责 | 主要问题 |
|---|---|---|
| `parallelOptions.h` | 循环选项、runtime 查询、进程 setter、scoped setter | per-call、per-executor 和 process-wide 概念混杂 |
| `parallelFor.h` | `parallelFor`/`parallelFor3D` 模板入口 | 只能隐式使用可变全局 TBB 环境 |
| `parallelFor.cpp` | TBB/串行执行、BLAS guard、`global_control`、affinity | 调度、资源上限和平台代码集中在一个实现文件 |
| `CMakeLists.txt` | TBB、MKL、Accelerate 条件链接 | 暂无结构性问题 |

当前 `RuntimeState` 持有一个可替换的 `tbb::global_control`。mutex 能保护一次状态更新，但不能保护
`ScopedWorkerLimit` 的整个生命周期。两个线程交错创建 scoped limit 时，会互相改变运行中任务的全局限制，并可能恢复到错误的最终值。

CPU affinity 也存在同类问题，而且影响范围更大：当前实现修改整个进程中已有线程，并在 Linux 上通过 watchdog 继续修改新线程。

## 设计原则

新体系遵循以下原则：

1. **进程级 TBB ceiling 稳定不变。** `tbb::global_control` 在 runtime 初始化时创建，生命周期覆盖所有 pgo parallel work，不在运行中替换。
2. **executor 局部上限不可变。** `ParallelExecutor` 持有固定参数的 `tbb::task_arena`，可以安全地被多个调用线程共享。
3. **调用显式选择 executor。** 无 executor overload 仅作为 default executor 的便捷入口。
4. **同一 TBB runtime 内自然组合。** nested TBB loop 默认继承当前 arena，不创建新 arena 或新 worker pool。
5. **BLAS threading 单独建模。** `NestedKernelPolicy` 继续决定已知 BLAS kernel 是 Suppress 还是 Inherit。
6. **不自动控制 mixed runtime。** ordinary third-party API 内部使用 OpenMP 时，TBB ceiling 无法限制其线程；core 不引入 OpenMP API、tag 或自动 fallback。
7. **CPU placement 完全交给外层资源管理。** 删除 libpgo 的 C++/Python affinity API 和 Linux watchdog，依赖 Slurm、cgroup、container cpuset、`taskset` 或 OS。
8. **只记录实际 arena participants。** 每个 executor 使用 `task_scheduler_observer` 区分 TBB workers 和 external threads；不做容量预测、admission control 或 CPU utilization 采样。

## 新体系总览

```text
process
└── ParallelRuntime
    ├── process-lifetime tbb::global_control(N)
    ├── default ParallelExecutor
    │   ├── tbb::task_arena(M0)
    │   └── ArenaObserver
    ├── ParallelExecutor A
    │   ├── tbb::task_arena(M1)
    │   └── ArenaObserver
    └── ParallelExecutor B
        ├── tbb::task_arena(M2)
        └── ArenaObserver
```

这里有两个不同的上限：

- `global_control(N)` 最多允许 `N - 1` 个 oneTBB worker 同时 active；application threads 不计入该 worker 上限。
- `task_arena(M)` 最多允许 `M` 个线程同时参与该 arena 的工作；这个数量包含 TBB workers 和进入 arena 的 application threads。

`task_arena(M)` 是容量上限，不是线程预留或私有线程池。多个 arena 共享 oneTBB scheduler 的 workers。

## 建议 API

```cpp
namespace pgo::parallel {

struct RuntimeOptions {
  // nullopt 在首次初始化时解析为 oneTBB default_concurrency。
  std::optional<int> maxTbbConcurrency;
};

struct ExecutorOptions {
  // nullopt 解析为所属 runtime 的 resolved max concurrency。
  std::optional<int> maxConcurrency;
};

struct RuntimeInfo {
  bool initialized = false;
  bool usingDefaultConcurrency = false;
  std::optional<int> maxConcurrency;
  int defaultConcurrency = 1;
  int effectiveTbbMaxAllowedParallelism = 1;
  int tbbWorkerCeiling = 0;
  int currentWorkerParticipants = 0;
  int currentExternalParticipants = 0;
  int currentTotalParticipants = 0;
  int peakTotalParticipants = 0;
  bool participantPressureObserved = false;
};

class ParallelExecutor;

class ParallelRuntime {
public:
  ParallelRuntime(const ParallelRuntime &) = delete;
  ParallelRuntime &operator=(const ParallelRuntime &) = delete;

  ParallelExecutor createExecutor(ExecutorOptions options = {});
  ParallelExecutor defaultExecutor() const;
  RuntimeInfo info() const;

private:
  explicit ParallelRuntime(RuntimeOptions options);
  std::shared_ptr<detail::RuntimeState> state_;

  friend ParallelRuntime &initializeRuntime(RuntimeOptions);
  friend ParallelRuntime &runtime();
};

// 一次性初始化；相同 resolved concurrency 幂等，不同值抛 std::logic_error。
ParallelRuntime &initializeRuntime(RuntimeOptions options = {});

// 返回唯一 runtime；尚未初始化时使用 default concurrency lazy 初始化。
ParallelRuntime &runtime();

// 两者均不初始化 runtime。
int defaultConcurrency();
RuntimeInfo runtimeInfo();

class ParallelExecutor {
public:
  ParallelExecutor(const ParallelExecutor &) noexcept = default;
  ParallelExecutor &operator=(const ParallelExecutor &) noexcept = default;

  std::optional<int> maxConcurrency() const noexcept;

private:
  std::shared_ptr<detail::ExecutorState> state_;
};

template<class Fn>
void parallelFor(
  const ParallelExecutor &executor,
  int begin,
  int end,
  const Options &options,
  Fn &&fn);

}  // namespace pgo::parallel
```

公共头文件不暴露 TBB 类型。TBB build 中：

- `RuntimeState` 持有 process-lifetime `tbb::global_control`。
- `ExecutorState` 持有初始化后不再修改的 `tbb::task_arena`、绑定该 arena 的 `ArenaObserver`，以及其所属 `RuntimeState` 的共享引用。
- `parallelFor` 通过 `arena.execute()` 进入执行域，再调用 `tbb::parallel_for`。

无 TBB build 保留同一公共 API，但所有 executor 都使用串行后端。

## Runtime 语义

### Global Control

`global_control(N)` 的精确语义是最多允许 `N - 1` 个 TBB workers active。它不限制：

- application 自己创建的 threads；
- OpenMP、Accelerate 或其他 runtime 的 threads；
- 进程中已经创建但当前 parked/sleeping 的 TBB workers。

因此 `RuntimeInfo` 只报告初始化状态、配置和 pgo arena participation：

```text
initialized
usingDefaultConcurrency
maxConcurrency               // 初始化后 resolved N；初始化前 nullopt
defaultConcurrency           // oneTBB info::default_concurrency()
effectiveTbbMaxAllowedParallelism // global_control::active_value()，包含外部 control 的最小值规则
active TBB worker ceiling   // effective value - 1
current worker participants // observer 看到的 pgo arena TBB workers
current external participants
current total participants
peak total participants
participant pressure observed
```

这些 participant 指标不是 CPU utilization，也不统计 pgo arena 外的 TBB/OpenMP/Accelerate threads。

### Runtime 生命周期

`global_control` 不再由 `ScopedWorkerLimit` 临时创建和销毁，而是覆盖整个 runtime 生命周期。这样避免运行中的任务突然获得或失去 worker supply。

libpgo 进程内只有一个 `RuntimeState` 和一个 pgo-owned `global_control`。`ParallelRuntime` 不公开 constructor；`initializeRuntime()` 和 `runtime()` 返回同一个 process runtime。oneTBB 仍可能受到进程内其他模块创建的 `global_control` 影响，因此 diagnostics 同时报告 pgo resolved value 和 oneTBB effective value。

初始化必须采用同步的一次性状态转换并在完整构造后原子发布。并发查询只能观察到完整的 uninitialized 或 initialized snapshot，不能看到已经设置 `initialized=true`、但 control/default executor 尚未就绪的中间状态。

### Automatic 默认值

`RuntimeOptions::maxTbbConcurrency == nullopt` 时，在首次初始化中执行：

```cpp
const int resolved = tbb::info::default_concurrency();
```

随后仍显式创建唯一 control 和 default arena：

```text
global_control(max_allowed_parallelism, resolved)
default task_arena(resolved)
```

oneTBB default concurrency 表示当前 library configuration/topology 下默认 arena 的最大 concurrency，通常对应进程可见的 logical CPUs，并考虑已有 process affinity。它不保证把只有 CPU quota、没有 cpuset 的 cgroup 自动换算成较小值。无 TBB build 的 resolved/default concurrency 均为 1。

`defaultConcurrency()` 和未初始化状态的 `runtimeInfo()` 只查询该候选值，不构造 pgo runtime。第一次无 executor `parallelFor` 调用通过 `runtime()` lazy 初始化；C++/Python 用户若要限制 concurrency，必须更早调用一次性 initialization API。

`defaultConcurrency()` 是当前 oneTBB 环境的只读诊断查询，外部若改变 process affinity，其返回值可能变化；runtime 初始化时只解析并保存一次 `maxConcurrency`，之后不会跟随该查询值变化。`RuntimeInfo.defaultConcurrency` 报告查询时的候选值，`RuntimeInfo.maxConcurrency` 报告固定的 resolved runtime value。

## Python Runtime API

Python module import 不初始化 native runtime。第一版公开三个函数和一个 immutable value type：

```python
from dataclasses import dataclass

@dataclass(frozen=True, slots=True)
class RuntimeInfo:
    initialized: bool
    using_default_concurrency: bool
    max_concurrency: int | None
    default_concurrency: int
    effective_tbb_max_allowed_parallelism: int
    tbb_worker_ceiling: int
    current_worker_participants: int
    current_external_participants: int
    current_total_participants: int
    peak_total_participants: int
    participant_pressure_observed: bool

def default_concurrency() -> int:
    """查询 oneTBB 默认值，不初始化 runtime。"""

def initialize(*, max_concurrency: int | None = None) -> RuntimeInfo:
    """一次性初始化唯一 process runtime，并返回当前信息。"""

def runtime_info() -> RuntimeInfo:
    """只读查询；未初始化时也不产生初始化副作用。"""
```

使用方式：

```python
from pypgo import parallel

# 显式限制；必须早于第一个触发 pgo parallel work 的算法调用。
parallel.initialize(max_concurrency=16)

# 或采用 oneTBB default concurrency。
parallel.initialize()
```

用户完全不调用 `initialize()` 时，第一个 pgo parallel algorithm lazy 初始化 automatic runtime。

重复初始化按 resolved concurrency 判断：相同 resolved value 幂等并返回现有 `RuntimeInfo`；不同值抛
`RuntimeError`，错误信息同时报告 existing/requested values。算法已经触发 automatic initialization 后，再请求不同显式值同样报错。

`runtime_info()` 在初始化前返回：

```text
initialized=false
using_default_concurrency=false
max_concurrency=None
default_concurrency=<oneTBB query result>
effective_tbb_max_allowed_parallelism=<当前 oneTBB active_value；无 TBB 时为 1>
participant counters=0
```

初始化前的 effective value 只是无副作用的 oneTBB 环境快照，不表示 pgo 已经建立 control。初始化后该字段表示 pgo control 与进程内其他 controls 按 oneTBB 最小值规则合成后的实际值。

当前 Python `set_worker_limit()`、`get_worker_limit()`、`worker_limit()` context manager 和所有 affinity API 全部删除，不提供 alias 或 deprecated wrapper。nanobind 仅暴露私有 initialization/default/info primitives；公开 dataclass 和参数校验由 `pypgo/parallel.py` 实现。

## Executor 语义

### 共享 Executor

多个调用线程共享 `ParallelExecutor(8)` 时，它们提交到同一个 arena；所有调用合计最多有 8 个 arena participants。没有可用槽位的 application thread 会等待进入 arena。

这是默认推荐模式，因为它给一个组件或请求组提供明确的共享执行预算。

### 独立 Executor

两个独立 executor 分别拥有 `task_arena(4)` 和 `task_arena(8)`。它们没有预留 12 个 workers，只是各自声明局部容量；实际 workers 来自同一个 TBB scheduler，并受 process-level `global_control` 约束。

允许 executor 的 `maxConcurrency` 大于 runtime 的 `maxTbbConcurrency`，也允许多个 executor 的局部上限之和超过 runtime ceiling。创建 executor 不拒绝、不告警；arena 保留自身配置值，实际 worker supply 由 `global_control` 调节。

`global_control` 不统计 application threads，因此多个独立 arena 不能提供严格的“全进程参与计算线程数不超过 N”保证。第一版不实现 admission control 或容量预测，只通过 observer 报告实际进入 pgo arenas 的 participants。

### Executor 析构

`ParallelExecutor` 析构释放 executor 对 arena 的引用，但不负责终止 oneTBB worker pool。worker 可能离开 arena 后继续以 parked/sleeping 状态常驻，等待其他 arena 复用。

第一版只提供 blocking `parallelFor`，不提供 `enqueue`。`arena.execute()` 和 `parallel_for` 返回后任务已经完成；同时，`shared_ptr<ExecutorState>` 保证 active call 持有 executor state，避免 arena 与运行中调用发生生命周期竞争。`ExecutorState` 对 `RuntimeState` 的共享引用保证 runtime handle 先析构时，process-level `global_control` 仍至少存活到最后一个 executor 和 active call 结束。

库代码不得在 executor 析构时调用 `task_scheduler_handle::finalize()`；finalize 是整个 oneTBB scheduler 的终止操作，会影响进程内其他 TBB 用户。

## Arena Participant Observer

每个 TBB executor 创建一个绑定自身 arena 的 `tbb::task_scheduler_observer`。oneTBB 在线程进入/离开 arena 时调用 observer，并通过 `isWorker` 区分线程来源：

```cpp
class ArenaObserver final : public tbb::task_scheduler_observer {
public:
  ArenaObserver(tbb::task_arena &arena, RuntimeCounters &counters);

  void on_scheduler_entry(bool isWorker) noexcept override;
  void on_scheduler_exit(bool isWorker) noexcept override;
};
```

observer callback 只执行无阻塞 atomic 更新：

```text
isWorker=true  -> currentWorkerParticipants
isWorker=false -> currentExternalParticipants
两者都更新     -> currentTotalParticipants、peakTotalParticipants
```

callback 中禁止日志、mutex、内存分配和异常。超过 effective runtime concurrency 时只设置
`participantPressureObserved=true`；core 不自动输出 warning，由上层通过 `RuntimeInfo` 查询并决定日志策略。文档必须说明该 flag 不表示 TBB worker ceiling 失效，超出的 participants 可能是 external threads。

observer 记录 scheduler participation，不表示线程正在占用 CPU。worker 可能在 arena 中等待或因 delayed-leave
策略暂时停留；第一版不采集 process CPU time、run queue、context switches 或 active body 数量。

`ExecutorState` 的成员顺序要让 observer 先析构、arena 后析构：

```cpp
struct ExecutorState {
  tbb::task_arena arena;
  ArenaObserver observer;
};
```

C++ 逆序析构使 observer 先停止观察；observer 析构会等待正在执行的 entry/exit callback 完成。

由于 worker 可能因 delayed-leave 暂留 arena，`parallelFor` 返回后 current counters 不要求立即归零。实现应让每个 observer 同时维护本地计数；executor teardown 先完整销毁 observer、等待 callback 结束，再从 runtime aggregate 注销该 observer 尚存的本地计数，保证 executor 销毁后不残留 telemetry。

## Nested TBB 语义

默认 nested loop 必须继承当前 arena：

```cpp
parallelFor(executor, 0, m, [&](int i) {
  parallelFor(0, n, [&](int j) {
    compute(i, j);
  });
});
```

内层只创建新的 TBB tasks，不创建新 arena 或新 worker pool。实现需要在进入 pgo executor 时记录当前执行上下文；无 executor overload 检测到当前上下文后，直接在当前 arena 调用 `tbb::parallel_for`。

第一版建议禁止在一个 active pgo executor 内显式切换到另一个 executor。跨 arena blocking nesting 虽然 oneTBB 可以表达，但会增加等待、资源竞争和生命周期推理成本，当前没有实际需求。

## Nested BLAS Kernel

`NestedKernelPolicy` 保持两个模式：

```text
Suppress（默认）
    outer pgo TBB 拥有并行度
    worker body 内的已知 BLAS kernel 请求单线程

Inherit
    保留调用线程已有的 BLAS threading 设置
```

平台行为：

- Linux MKL-TBB：process `global_control` 是最终 TBB worker ceiling。现有 benchmark 表明 Suppress/Inherit 均未出现 outer × inner worker 爆炸，且 Suppress 更快。
- macOS Accelerate：`BLASSetThreading` 是 thread-local；Suppress 可在 worker body 中保存、设置 single、恢复。Inherit 使用独立于 TBB 的 Accelerate threading，TBB runtime 无法给出统一线程上限。
- 直接在 pgo parallel 区域外调用 BLAS 不受 `NestedKernelPolicy` 影响。

迁移前仍需增加 arena 版本的 MKL-TBB benchmark，验证 MKL 内部 TBB work 在显式 arena 下的线程峰值和性能。即使 arena 继承不完整，process-level `global_control` 仍保留为最终 worker ceiling。

## Mixed Parallel Runtime 风险

本模块不尝试自动控制 mixed runtime。特别是以下普通调用可能隐藏 OpenMP：

```cpp
parallelFor(..., [&](int i) {
  thirdPartyApi(i);  // 实现内部可能进入 OpenMP parallel region
});
```

此时可能形成 TBB outer × OpenMP inner。`global_control` 和 `task_arena` 只能控制 TBB，无法限制 third-party API 内部的 OpenMP team。反方向 OpenMP outer → TBB inner 也有风险：OpenMP workers 对 TBB 而言属于 application threads，不计入 `global_control` 的 worker ceiling。

第一版只做以下约束：

- core 不包含 `omp.h`，不链接或调用 OpenMP runtime API。
- 不增加 mixed-runtime tag、自动检测、自动 serial fallback 或 runtime coordination。
- 文档要求不要从 `pgo::parallelFor` body 调用已知内部使用 OpenMP 的 API。
- 必须混用时，由调用方重构并行所有权、使用 third-party 的 serial 配置，或在应用启动环境配置对应 runtime。

这是一项已知性能和线程数风险，不是 `ParallelRuntime` 能提供的安全保证。

## CPU Affinity

CPU affinity 控制线程可运行的位置，不控制线程创建数量或 active parallelism。当前 `setCpuAffinityLimit(N)` 按排序选择前 N 个可用 CPU，并通过 process-wide 操作和 Linux watchdog 影响其他 native runtime；它不理解物理核、SMT 或 NUMA topology。

此次重构直接删除全部 affinity 功能，不保留兼容层或只读诊断：

- 删除 `supportsCpuAffinityLimit()`、`setCpuAffinityLimit()`、`cpuAffinityLimit()` 和 `ScopedCpuAffinityLimit`。
- 删除 `RuntimeInfo` 中的 `cpuAffinityLimit`、`currentCpuAffinityCpus`。
- 删除 Linux affinity watchdog 及 Linux/Windows affinity 平台实现。
- 删除全部 Python get/set/reset/supports bindings。
- 删除 affinity tests 和用户文档。

CPU placement 完全由 Slurm、cgroup、container cpuset、`taskset` 或 OS scheduler 管理。

## Solver 线程配置解耦

当前 `EigenOrigPardisoSupport::setParam()` 和 `KnitroSolverWrapper::init()` 读取 `workerLimit()`，把 TBB runtime 配置当作 solver 线程数。这是隐藏耦合。

重构后：

- `ParallelRuntime::maxTbbConcurrency` 只表示 TBB scheduler ceiling。
- PARDISO、Knitro 等 solver 通过各自 options/constructor 显式接收 `numThreads`。
- 应用层可以从同一个资源配置生成 runtime 与 solver options，但 core 模块之间不共享可变全局线程数。

## 职责与耦合分析

建议拆分职责：

- `ParallelRuntime`：process-level TBB ceiling 和 runtime diagnostics。
- `ParallelExecutor`：不可变 arena 和执行域生命周期。
- `ArenaObserver`：记录 arena worker/external participation，并安全汇总 runtime telemetry。
- `parallelFor`：range 分块、当前 executor 继承和 body 调用。
- `NestedKernelGuard`：thread-local MKL/Accelerate suppression。
- solver options：第三方 solver 自己的线程数。

模块中没有需要重构的继承层次。`ParallelExecutor` 使用组合和 PIMPL：

- 公共头文件不暴露 TBB 类型。
- `shared_ptr<ExecutorState>` 提供廉价、跨线程共享的值语义句柄。
- arena 参数初始化后固定，热路径不修改配置。

这里不需要 CRTP、虚接口或复杂 policy template；后端差异通过 `.cpp` 中的条件编译处理更直接。

## 建议目录

```text
src/core/parallelism/
  CMakeLists.txt
  parallelRuntime.h
  parallelRuntime.cpp
  parallelExecutor.h
  parallelExecutor.cpp
  parallelOptions.h
  parallelFor.h
  parallelFor.cpp
  parallelInternal.h
  REFACTOR_PLAN.md
```

实现按职责拆为三个 `.cpp`：runtime 负责 singleton/global control/diagnostics，executor 负责 arena/observer，parallelFor 负责 TLS nesting、TBB/serial dispatch 和 BLAS guard。`parallelInternal.h` 是模块私有头文件，不作为公共 API 安装。Nested kernel guard 与 chunk 执行边界紧密耦合，不再单独拆文件。

## 第一阶段冻结决策

### 1. Default Runtime/Executor 析构

推荐 default executor 随 default runtime 一次构造后保持不变。需要不同局部并行度的 C++ 调用方创建显式 executor，不原子替换 default executor。

第一版使用 intentionally retained process-lifetime state，不在静态析构阶段销毁 default runtime/executor。显式 executor handle 仍正常析构自己的 arena/observer state。这样避免其他静态对象析构期间再次进入 parallel API 时发生顺序问题，也不承诺主动关闭 oneTBB scheduler。

### 2. `reserved_slots`

`task_arena` 默认 `reserved_slots=1`，为 application thread 保留进入 arena 的能力。

第一版固定为 1，不暴露公共配置。只有出现 enqueue-only 或多-master 的具体需求后再开放。

### 3. Nested 显式切换 Executor

第一版检测 active pgo executor：

- 未显式传 executor的 nested loop 继承当前 executor。
- 显式传入同一个 executor 直接执行。
- 显式传入不同 executor 抛 `std::logic_error`。

后续只有在存在真实的跨 arena nesting use case 和 benchmark 后才放宽。

### 4. Participant Warning 策略

第一版不自动向 `stderr` 或 logging 输出 warning，只在 `RuntimeInfo` 中设置
`participantPressureObserved`。observer callback 只设置 atomic 状态，调用方根据结构化 diagnostics 决定是否记录日志。

peak 和 pressure flag 使用 runtime-lifetime 累积语义，不提供 `resetTelemetry()`；benchmark 需要 fresh state 时继续使用现有 isolated runner。

### 5. Python Executor

第一版不向 Python 暴露 `ParallelExecutor` 或 `NestedKernelPolicy`，所有 Python algorithms 使用 default executor。只有出现 Python 多请求需要独立 arena 的真实 use case 后，才设计 executor object/context API。

### 6. ABI 与命名

public C++ 名称遵循当前 camelCase 风格；Python 使用 snake_case。不把行为迁移与全仓命名迁移耦合。

## 具体重构步骤

1. **[M] [med-risk] 引入唯一的 process-lifetime `ParallelRuntime`。** 通过 `initializeRuntime()` 显式初始化，或由首次 parallel work 按 `tbb::info::default_concurrency()` lazy 初始化；解析后创建唯一固定 `global_control`，不提供 setter/reset。
2. **[M] [med-risk] 引入 `ParallelExecutor`。** 使用不可变 `task_arena`，验证同一 executor 被多个调用线程共享。
3. **[M] [med-risk] 增加 `ArenaObserver` telemetry。** 汇总 worker/external/current/peak participants，并在 callback 中只使用 atomic 操作。
4. **[M] [med-risk] 增加显式 executor overload。** 让无 executor overload 使用固定 default executor。
5. **[M] [med-risk] 实现 nested current-arena 继承。** 增加 same-executor nesting 和 different-executor rejection 测试。
6. **[M] [high-risk] 扩展 MKL-TBB benchmark。** 在 process ceiling 16 下验证不同 arena concurrency 的 Suppress/Inherit participant telemetry 和性能。
7. **[S] [med-risk] 删除旧 worker API。** 删除 C++ `setWorkerLimit()`、`workerLimit()`、`ScopedWorkerLimit` 及 Python get/set/reset bindings；tests/benchmarks 改用 runtime + executor。
8. **[S] [med-risk] 删除 CPU affinity。** 删除全部 C++/Python API、平台实现、Linux watchdog、tests 和 docs。
9. **[M] [med-risk] 解耦 PARDISO/Knitro 线程配置。** solver 显式接收 `numThreads`，不读取 parallel runtime。
10. **[M] [med-risk] 重建 Python runtime API 和文档。** 删除 mutable worker/affinity bindings，加入无副作用查询、一次性 `initialize()`、冻结的 `RuntimeInfo`，并解释 participant telemetry 和 mixed runtime 风险。
11. **[M] [low-risk] 拆分实现文件。** 行为稳定后再进行目录级物理拆分。

## API 迁移

旧代码：

```cpp
{
  pgo::parallel::ScopedWorkerLimit limit(4);
  pgo::parallel::parallelFor(0, n, options, body);
}
```

新代码：

```cpp
auto &runtime = pgo::parallel::initializeRuntime({ .maxTbbConcurrency = 16 });
const auto executor = runtime.createExecutor({ .maxConcurrency = 4 });

pgo::parallel::parallelFor(executor, 0, n, options, body);
```

重复调用复用 executor。需要多个调用共享总 arena 容量时，共享同一个 executor；需要不同局部上限时，创建独立 executor，同时接受 application threads 不受 global worker ceiling 统计的语义。

这是 breaking API migration，不设置兼容期：

- 删除 `setWorkerLimit()`、`workerLimit()` 和 `ScopedWorkerLimit`。
- 删除全部 CPU affinity C++/Python API。
- benchmark/tests 使用显式 runtime + executor。
- solver 通过自己的 options 接收线程数。
- Python 提供无副作用的 `default_concurrency()`、`runtime_info()` 和一次性 `initialize(max_concurrency=None)`；不暴露 mutable setter/reset/context manager。

## 测试矩阵

### 测试目标拆分

process-level `global_control`、default runtime 和累计 telemetry 会在同一进程中互相影响。测试目标按 runtime 配置拆分，避免一个 gtest binary 内反复重建全局环境：

```text
parallelism_core_gtest       // range、options、显式 executor、nested 语义
parallelism_observer_gtest   // 固定小 ceiling，observer 和生命周期
parallelism_pressure_gtest   // global concurrency=1 的确定性 external pressure
parallelism_blas_gtest       // 条件编译的 MKL/Accelerate integration
```

CTest 继续通过 `gtest_discover_tests` 独立运行测试；同一 target 内的测试也不得依赖执行顺序。

### 纯功能与错误传播

- `parallelFor`、`parallelFor3D` 覆盖空 range、单元素、非整除 grain、每个 index/coordinate 恰好访问一次。
- `parallelFor3D` 的无效尺寸和索引容量溢出保持现有行为。
- body 抛异常时，blocking `parallelFor` 向调用方传播异常；runtime 和 executor 随后仍可复用。
- `maxTbbConcurrency <= 0`、`maxConcurrency <= 0` 抛 `std::invalid_argument`。
- 无 TBB build 使用相同 API 串行执行，observer fields 为零或 unavailable 的约定保持一致。

### Runtime 与 Executor

- `defaultConcurrency()` 和未初始化时的 `runtimeInfo()` 都不创建 runtime、`global_control` 或 default executor。
- 首次无 executor 的 `parallelFor` 使用 `tbb::info::default_concurrency()` lazy 初始化；显式 `initializeRuntime()` 使用指定值初始化。
- automatic 和显式 runtime 都报告正确的 resolved max concurrency；TBB worker ceiling 等于 `effectiveTbbMaxAllowedParallelism - 1`。
- 重复初始化为相同 resolved value 幂等；不同 resolved value 抛 `std::logic_error`。automatic 与显式值相等时也视为同一配置，并保留首次初始化来源。
- 多线程同时调用 `runtime()`/`initializeRuntime()` 时只发布一个 `RuntimeState` 和一个 pgo-owned `global_control`；相同 resolved value 的调用拿到同一 runtime 地址，冲突配置由先完成初始化者获胜，其余调用确定性报错。
- 创建 `maxConcurrency` 等于和大于 runtime ceiling 的 executor 均成功；arena 保留请求值，实际 workers 不突破 global ceiling。
- 多个线程共享同一个 executor 并发调用，结果正确，observer 看到的该 arena participants 始终不超过 `maxConcurrency`。
- 两个独立 executor 同时运行时配置互不覆盖，runtime counters 汇总两个 observers。
- 复制 executor handle 后销毁原 handle，剩余 handle 继续工作；active call 持有 state，调用期间销毁外部 handles 不发生 use-after-free。
- executor 析构不调用 scheduler finalize，之后新 executor 仍能复用 oneTBB scheduler。

### Observer 分类

external participant 使用 `global_control(1)` + `task_arena(1)` 验证：调用线程进入 arena 时
`isWorker=false`，worker count 保持零。

worker participant 分两层测试：

- `ArenaObserver` 低层测试直接向绑定 arena `enqueue` 一个任务，验证 callback 收到 `isWorker=true`；该测试只验证 observer adapter，不扩大 public executor API。
- `parallelFor` 集成测试提交足够多且短暂阻塞的 chunks，在 effective concurrency 大于一时确认至少观察到一个 worker；使用有超时的 gate，资源不足时 skip，禁止依赖固定 worker 数量。

所有 observer 测试只断言：

- current counters 永不为负；
- `currentTotal = currentWorker + currentExternal` 的稳定快照成立；
- `peakTotal >= currentTotal` 且 peak 只增不减；
- local participants 不超过 arena max concurrency；
- worker participants 不超过 effective global worker ceiling。

不要求 `parallelFor` 返回后 current counters 立即归零，因为 delayed-leave 允许 worker 暂留 arena。executor teardown 完整销毁 observer 后，runtime aggregate 必须移除该 executor 的本地计数。

### 确定性 Participant Pressure

使用一个独立进程场景：

```text
runtime global concurrency = 1
executor A arena concurrency = 1
executor B arena concurrency = 1
两个 std::thread 同时进入各自 arena，并在 body 内通过 gate 保持重叠
```

该场景不允许 TBB workers，但两个 application threads 可以分别进入两个 arena。稳定断言：

```text
currentWorkerParticipants == 0
currentExternalParticipants == 2
currentTotalParticipants == 2
effectiveTbbMaxAllowedParallelism == 1
participantPressureObserved == true
peakTotalParticipants >= 2
```

core 不输出 warning；测试只断言结构化 flag，不依赖并发 `stderr` capture。

### Nested TBB

- implicit inner `parallelFor` 继承 current executor/arena，inner 中的 `this_task_arena::max_concurrency()` 等于 executor arena 配置。
- 显式传入同一 executor 可以 nested 执行。
- 显式传入不同 executor 在调度前抛 `std::logic_error`，不进入第二个 arena。
- nested loop 只由同一个 observer 计数，不创建额外 executor state。
- nested body 抛异常后 current-executor context 正确恢复，后续 default/executor 调用不受污染。

### Nested BLAS Integration

Linux MKL-TBB 条件测试：

- runtime ceiling 和 arena limit 使用小值，Suppress/Inherit 都完成 DGEMM 且 observer worker count 不超过 global ceiling。
- Suppress 在 worker body 中观察到 local single-thread 语义并在退出后恢复。
- Inherit 保留调用线程原设置。
- 不在 gtest 中断言性能快慢；性能和矩阵阈值继续由 Google Benchmark 记录。

macOS Accelerate 条件测试：

- Suppress 在 body 内观察到 `BLAS_THREADING_SINGLE_THREADED`，调用后恢复原 threading。
- Inherit 保留 multi-thread setting。
- observer 只统计 TBB arena participants，不把 Accelerate threads 误报为 TBB workers。

### API 删除与 Python

- 全仓编译确认不再存在 `setWorkerLimit()`、`workerLimit()`、`ScopedWorkerLimit` 和 affinity symbols。
- Python module 不再暴露 worker get/set/reset 或任何 affinity binding。
- import Python module 不初始化 runtime；`default_concurrency()` 和初始化前的 `runtime_info()` 也无副作用。
- `initialize()` 默认使用自动并行度，`initialize(max_concurrency=N)` 使用显式值；相同 resolved value 重复调用幂等，不同值抛 `RuntimeError`。
- 首次 Python parallel algorithm 可 lazy 初始化；此后不同值的 `initialize()` 必须被拒绝。
- Python `RuntimeInfo` 是 `frozen=True, slots=True` 的值对象，只包含约定的 runtime/participant telemetry fields，不包含旧 worker/affinity keys。
- 初始化前 `RuntimeInfo.max_concurrency is None`、participant counters 为零；初始化后报告首次配置来源、resolved/effective concurrency 和累积 telemetry。
- 每个 Python 初始化场景放在独立 subprocess 中测试，不依赖 teardown 重置 process singleton。
- mixed runtime 只做文档审查，不把 OpenMP dependency 或行为测试加入 core test target。

### Stress 与工具测试

- stress test 使用多个 application threads 反复共享/复制 executor、运行 nested loops、注入异常并销毁 handles；最终结果正确，aggregate counters 不下溢或泄漏。
- 在可用平台增加 ThreadSanitizer CI 运行 observer/runtime stress；oneTBB 自身的已知 suppression 与项目代码问题分开处理。
- observer overhead 使用 Google Benchmark 单独比较，不设置脆弱的 CI 性能 pass/fail 阈值。

### 禁止的脆弱断言

- 不断言 scheduler 一定启动恰好 N 个 workers。
- 不用 process resident thread count 推导 active participants。
- 不要求调用返回后 worker 立即离开 arena。
- 不用 `cpu_time / real_time` 作为 observer 正确性的判据。
- 不在单元测试中比较 Suppress/Inherit 性能。

## 风险评估

- **MKL-TBB arena 继承仍需实验。** `global_control` ceiling 保留，因此 worker 总量有最终约束；arena 局部语义需 benchmark 证明。
- **多个 arena 不是严格进程配额器。** application threads 不计入 `global_control` worker ceiling，不能把 N 宣称为进程总 active threads 上限。
- **worker 常驻会污染 thread-count 指标。** executor 析构不保证 OS threads 退出；benchmark 必须区分 resident threads 与实际执行并行度。
- **observer 不是 CPU profiler。** participant 可能等待或因 delayed-leave 暂留 arena；telemetry 不等于 CPU utilization。
- **observer 只覆盖 pgo arenas。** 直接 TBB、未继承当前 arena 的 MKL work 和其他 runtime 不计入 counters。
- **observer callback 属于调度热路径。** 必须保持 `noexcept`、无阻塞并 benchmark 原子更新开销。
- **mixed runtime 不受控制。** ordinary third-party API 内部 OpenMP 可能与 TBB 形成 oversubscription；文档约束无法替代调用图审计。
- **solver 配置迁移跨模块。** PARDISO/Knitro 默认线程数变化可能造成性能回归。
- **直接删除配置 API 是 breaking change。** C++/Python 调用方必须迁移到一次性初始化和只读 diagnostics；Python 测试必须用 subprocess 隔离 singleton 场景。
- **多份 TBB runtime 会破坏统一 ceiling。** 插件或静态链接若加载彼此独立的 TBB runtime，每份 runtime 可能拥有自己的 worker pool；构建和部署应确保共享同一 oneTBB runtime。

## 优先级矩阵

| 优先级 | 工作 | 影响 | 风险 |
|---|---|---|---|
| P0 | process-lifetime runtime + immutable executor 语义 | 消除跨线程全局覆盖 | 中 |
| P0 | MKL-TBB arena benchmark | 验证核心假设 | 中高 |
| P0 | arena participant observer | 获取实际 TBB/external participation | 中 |
| P1 | nested current-arena 继承 | 避免隐式切换 arena | 中 |
| P1 | 删除旧 worker 和 affinity API | 收紧公共契约 | 中 |
| P1 | solver 线程配置解耦 | 消除隐藏全局依赖 | 中 |
| P2 | Python migration | 保持用户体验 | 中 |
| P3 | 文件拆分和样式整理 | 改善维护性 | 低 |

## 样式规范化

当前模块沿用仓库的 camelCase 文件名、函数名和两空格缩进，与统一 snake_case/四空格目标不一致。本次不做样式性重命名，避免扩大公共 API 和 diff。新增接口遵循现有仓库风格；全仓命名统一应单独进行机械重构。

## 推荐第一阶段范围

第一阶段只实现以下内容：

- 一个 process-lifetime `ParallelRuntime`，持有不可动态替换的 `global_control`。
- 可跨线程共享的 immutable `ParallelExecutor`，每个 executor 持有 `task_arena`。
- 每个 executor 持有 `ArenaObserver`，runtime 汇总实际 worker/external/current/peak participants。
- 显式 executor overload 和 default executor 兼容入口。
- nested TBB 默认继承 current arena。
- 保持 `NestedKernelPolicy::Suppress` 默认值。
- arena 版本 MKL-TBB benchmark。
- 直接删除旧 worker setter/getter/scoped API 和全部 CPU affinity API。
- Python 同步迁移到无副作用查询、一次性 `initialize()` 和 immutable `RuntimeInfo`；不暴露 executor。
- 文档明确 mixed runtime 风险，不实现自动协调。

第一阶段不实现 Python executor、admission control、CPU utilization 或更复杂的多-arena 资源管理；后续只在出现真实调用场景和测量依据后扩展。

## 参考资料

- [oneTBB `global_control`](https://uxlfoundation.github.io/oneTBB/main/specification/source/task_scheduler/scheduling_controls/global_control_cls.html)
- [oneTBB information queries and `default_concurrency`](https://uxlfoundation.github.io/oneTBB/main/specification/source/info_namespace.html)
- [oneTBB `task_arena`](https://uxlfoundation.github.io/oneTBB/main/specification/source/task_scheduler/task_arena/task_arena_cls.html)
- [oneTBB `task_scheduler_observer`](https://uxlfoundation.github.io/oneTBB/main/specification/source/task_scheduler/task_arena/task_scheduler_observer_cls.html)
- [oneTBB scheduler finalization](https://uxlfoundation.github.io/oneTBB/main/specification/source/task_scheduler/scheduling_controls/task_scheduler_handle_cls.html)
- [oneTBB scheduler initialization migration guide](https://uxlfoundation.github.io/oneTBB/main/tbb_userguide/Migration_Guide/Task_Scheduler_Init.html)
- [oneTBB mixing threading runtimes](https://www.intel.com/content/www/us/en/docs/onetbb/developer-guide-api-reference/2023-0/appendix-b-mixing-with-other-threading-packages.html)
