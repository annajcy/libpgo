# Parallelism Core Refactor Plan

> **状态日期：** 2026-06-02
> **适用范围：** C++ `src/core` 中简单并行循环的统一封装；从 `ImplicitField::sampleToGrid` 开始迁移。
> **执行约束：** 不重写复杂 TBB reduction / TLS / concurrent container 算法；第一阶段只统一 embarrassingly parallel 的 `parallel_for`。

## 后续计划执行规则

从本计划落地后，所有后续 Python API migration / C++ refactor plan 遇到 TBB/OpenMP 并行循环时，默认规则是：

- 简单 `parallel_for` / 三维逐点循环必须使用 `src/core/parallelism` 提供的 `pgo::parallel::parallelFor*` API。
- 业务模块不得新增直接 `#include <tbb/...>`、`tbb::parallel_for` 或 `#pragma omp parallel for`。
- 如果遇到 `parallel_reduce`、TLS、`concurrent_vector`、锁、custom partitioner 等复杂模式，不要临时在业务模块里复制 TBB/OpenMP 用法；先判断是否暂缓迁移，或为 `core/parallelism` 增加一个窄的专用抽象。
- Python API 不暴露底层 backend 选择；Python 入口只保留局部 `num_threads` 或未来的 coarse-grained runtime config。

## 目标

在 `src/core/parallelism` 新增一个轻量并行能力核心库，让业务代码不再直接 include `tbb/...` 或写 `#pragma omp`：

```cpp
#include "parallelism/parallelFor.h"

pgo::parallel::parallelFor3D(nx, ny, nz,
  { .numThreads = numThreads },
  [&](int x, int y, int z) {
    // independent work item
  });
```

统一后的调用点只表达“这里可以并行”，不关心 backend 是 TBB、OpenMP 还是串行 fallback。

## 当前问题

### 1. 并行 backend 暴露在业务代码里

当前仓库里大量源文件直接 include：

- `<tbb/parallel_for.h>`
- `<tbb/blocked_range.h>`
- `<tbb/enumerable_thread_specific.h>`
- `<tbb/spin_mutex.h>`
- OpenMP `#pragma omp parallel for`

这会带来几个长期问题：

- 调用点反复写 backend 细节，`numThreads` 语义不统一。
- TBB/OpenMP 头文件扩散到各个模块，模块边界变脏。
- 后续要统一 nested parallelism、Python binding 调用策略、profiling hook 或 deterministic mode 时没有公共入口。
- OpenMP 的 pragma 只能靠宏分支散落在 `.cpp` 中，维护成本高。

### 2. `ImplicitField::sampleToGrid` 是第一个明显迁移点

当前文件：

- `src/core/implicitSurface/core/ImplicitField.h`
- `src/core/implicitSurface/core/ImplicitField.cpp`

`sampleToGrid` 现在使用：

```cpp
#ifdef USE_OPENMP
#pragma omp parallel for collapse(3) ...
#else
serial loop
#endif
```

在没有 OpenMP 的 build 中，即使项目已经有 TBB，也会退回串行。这个函数是典型的逐 grid point 独立采样，适合作为 `core/parallelism` 的第一个落点。

### 3. TBB 已经是事实上的核心依赖

当前顶层 `CMakeLists.txt` 已经 include `CMakeModules/third-party/tbb.cmake`，而 `eigenSupport` 公开链接 `TBB::tbb`。许多 core 模块也已经直接使用 TBB。

因此第一版 `core/parallelism` 不需要把 TBB 当成“可有可无”的实验依赖；它可以作为默认 backend，同时保留 OpenMP backend 和 serial fallback。

## 非目标

- 不在第一阶段迁移 `tbb::parallel_reduce`。
- 不抽象 `tbb::enumerable_thread_specific`。
- 不抽象 `tbb::concurrent_vector`。
- 不抽象 `tbb::spin_mutex` 或其他锁。
- 不强行替换使用 `tbb::blocked_range`、custom partitioner、复杂 TLS 的性能敏感代码。
- 不让 Python 用户直接选择某个 C++ backend。
- 不在 Python 层暴露任意 parallel-for callback API。

这些能力可以未来单独设计。第一阶段只做简单 `parallelFor` / `parallelFor3D`。

## 推荐设计

### 1. 新增 core library

新增目录：

```text
src/core/parallelism/
  CMakeLists.txt
  parallelOptions.h
  parallelFor.h
  parallelFor.cpp
```

建议 namespace：

```cpp
namespace pgo::parallel {

enum class Backend {
  Auto,
  Serial,
  TBB,
  OpenMP,
};

struct Options {
  int numThreads = 0;
  Backend backend = Backend::Auto;
  int grainSize = 0;
};

}  // namespace pgo::parallel
```

`numThreads` 统一语义：

| 值 | 语义 |
| --- | --- |
| `0` | 使用 backend 默认并行度 |
| `1` | 强制串行 |
| `>1` | 请求并限制最大并行度 |

`Backend::Auto` 默认选择：

```text
TBB if available -> OpenMP if available -> Serial
```

原因：TBB 对库内部组合、nested parallelism 和 task scheduler 更友好；OpenMP 作为兼容 backend 保留。

### 2. 用 chunk API 隐藏 TBB/OpenMP 头文件

如果 `parallelFor` 直接在 header 里调用 `tbb::parallel_for`，业务代码虽然不用手写 TBB include，但 TBB 头文件仍会被公共 header 传递 include。

更干净的第一版方案是把 backend 调度做成 chunk API：

```cpp
namespace pgo::parallel::detail {

using ChunkBody = std::function<void(int begin, int end)>;

void parallelForChunks(int begin, int end,
  const Options &options,
  const ChunkBody &body);

}  // namespace pgo::parallel::detail
```

public template 只在每个 chunk 内串行调用用户 lambda：

```cpp
template<class Fn>
void parallelFor(int begin, int end, const Options &options, Fn &&fn)
{
  detail::parallelForChunks(begin, end, options,
    [&](int chunkBegin, int chunkEnd) {
      for (int i = chunkBegin; i < chunkEnd; ++i)
        fn(i);
    });
}
```

这样：

- `parallelFor.h` 只需要 include `parallelOptions.h` 和 `<functional>`。
- `parallelFor.cpp` 才 include `<tbb/parallel_for.h>`、`<tbb/global_control.h>`、OpenMP headers / pragmas。
- `std::function` 的开销按 chunk 发生，不按 element 发生；对 `sampleToGrid` 这种中粗粒度循环足够合适。

`parallelFor3D` 在 public header 中线性化：

```cpp
template<class Fn>
void parallelFor3D(int nx, int ny, int nz, const Options &options, Fn &&fn)
{
  parallelFor(0, nx * ny * nz, options, [&](int index) {
    const int x = index % nx;
    const int y = (index / nx) % ny;
    const int z = index / (nx * ny);
    fn(x, y, z);
  });
}
```

### 3. backend 选择和错误策略

`parallelForChunks` 行为：

- `numThreads == 1` 或 range 为空：直接串行。
- `Backend::Serial`：串行。
- `Backend::TBB` 且 TBB 可用：使用 `tbb::parallel_for`。
- `Backend::OpenMP` 且 OpenMP 可用：使用 `#pragma omp parallel for schedule(static)`。
- 指定 backend 不可用：抛出 `std::runtime_error`，不要静默降级。
- `Backend::Auto`：按可用性选择；如果都不可用则串行。

TBB 限制线程数：

```cpp
tbb::global_control control(
  tbb::global_control::max_allowed_parallelism,
  static_cast<std::size_t>(options.numThreads));
```

OpenMP 限制线程数：

```cpp
#pragma omp parallel for num_threads(options.numThreads)
```

`grainSize`：

- `0` 表示使用默认 chunk 策略。
- TBB backend 可映射到 `tbb::blocked_range<int>(begin, end, grainSize)`。
- OpenMP backend 可用静态 chunk size。
- 第一版也可以先只支持 `0`，但字段预留后迁移调用点更方便。

## CMake 设计

新增 `src/core/parallelism/CMakeLists.txt`：

```cmake
set(PARALLELISM_HEADERS
  parallelOptions.h
  parallelFor.h)

set(PARALLELISM_SOURCES
  parallelFor.cpp)

set(PARALLELISM_DEPS pgoLogging)

if(TARGET TBB::tbb)
  list(APPEND PARALLELISM_DEPS TBB::tbb)
  set(PGO_PARALLELISM_HAS_TBB 1)
endif()

add_libpgo_lib(parallelism "${PARALLELISM_SOURCES}" "${PARALLELISM_HEADERS}")
target_link_libraries(parallelism PUBLIC ${PARALLELISM_DEPS})

if(PGO_PARALLELISM_HAS_TBB)
  target_compile_definitions(parallelism PRIVATE PGO_PARALLELISM_HAS_TBB)
endif()

if(OPENMP_FOUND)
  target_compile_definitions(parallelism PRIVATE PGO_PARALLELISM_HAS_OPENMP)
endif()
```

注意：

- `parallelism` 不应该依赖 `eigenSupport`，否则会和大量 core 模块形成反向依赖。
- `parallelism` 可以依赖 `pgoLogging`，也可以完全不依赖 logging。第一版建议先不输出 runtime log。
- 需要在顶层 core add_subdirectory 列表中加入 `parallelism`，并让后续模块显式链接它。

## 第一阶段迁移：`ImplicitField::sampleToGrid`

### 当前目标

把 `src/core/implicitSurface/core/ImplicitField.cpp` 改为：

```cpp
#include "parallelism/parallelFor.h"

GridField ImplicitField::sampleToGrid(const GridSpec &spec, int numThreads) const
{
  GridField grid(spec);
  const int resolution = spec.resolution;
  const V3d delta = (spec.bmax - spec.bmin) / static_cast<double>(resolution - 1);

  pgo::parallel::parallelFor3D(resolution, resolution, resolution,
    { .numThreads = numThreads },
    [&](int x, int y, int z) {
      const V3d p = spec.bmin + delta.cwiseProduct(V3d(x, y, z).cast<double>());
      grid.at(x, y, z) = eval(p);
    });

  return grid;
}
```

同时更新 `ImplicitField.h` 注释：

```text
0: use default parallel backend
1: force serial loop
>1: request that many worker threads
```

`implicitSurface` 的 CMake 依赖增加 `parallelism`。

## 后续迁移范围

### 适合优先迁移

优先迁移满足以下条件的循环：

- 每个 iteration 独立。
- 不使用 TLS、spin mutex、concurrent container。
- 不依赖 custom partitioner。
- 只是 `tbb::parallel_for(0, n, lambda)` 或简单 `blocked_range<int>`。

候选模块：

- `src/core/interpolationCoordinates/*`
- `src/core/mesh/triMeshPseudoNormal.cpp` 中简单 vertex/triangle loops
- `src/core/volumetricMesh/volumetricMesh.cpp` 中简单 target-location loops
- `src/core/implicitSurface/*`

### 暂缓迁移

这些代码先保留 TBB 直接使用：

- `tbb::parallel_reduce`
- `tbb::enumerable_thread_specific`
- `tbb::concurrent_vector`
- 带 `tbb::spin_mutex` 的 sparse assembly
- 明确使用 `tbb::static_partitioner()` / `auto_partitioner()` 调优的热点

等第一阶段稳定后，再设计 `parallelReduce` 或 `ThreadLocal<T>`，不要提前抽象。

## Python API 建议

### 不建议暴露底层 parallel-for API

不建议在 Python 暴露：

```python
pgo.parallel.parallel_for(...)
```

原因：

- Python callback 会被 GIL 和跨语言调用开销限制，不能作为 C++ 数值内核的高频并行机制。
- backend 选择是 C++ runtime 策略，不应该成为普通 Python 用户必须理解的 API。
- TBB/OpenMP 的差异暴露给 Python 后会增加兼容性和文档成本。

### 可以暴露轻量全局配置

如果 Python API 需要控制并行度，推荐只暴露配置级 API：

```python
import pypgo as pgo

pgo.runtime.set_num_threads(8)
pgo.runtime.num_threads()
pgo.runtime.set_parallel_backend("auto")  # optional; advanced only
```

但第一版可以不做全局 API，而是在已有计算入口保留局部参数：

```python
grid = field.sample_to_grid(spec, num_threads=8)
```

推荐顺序：

1. C++ `core/parallelism` 先稳定。
2. Python 继续使用局部 `num_threads` 参数。
3. 等多个 Python 模块都需要统一线程控制时，再新增 `pypgo.runtime`。

不要在第一版 Python API 中公开 `Backend.TBB` / `Backend.OpenMP` 给普通用户。若需要调试，可放到 advanced runtime config。

## 测试计划

### 1. C++ unit tests

新增测试文件或扩展现有 implicit surface 测试：

- `parallelFor` serial path：`numThreads = 1`，结果顺序和内容正确。
- `parallelFor` default path：结果正确。
- `parallelFor3D` index mapping：`x/y/z` 覆盖完整空间，无重复无遗漏。
- 指定不可用 backend：抛出明确异常。
- `sampleToGrid(numThreads=1)` 和 `sampleToGrid(numThreads=0/2)` 数值一致。

可以增加一个 thread-observing test field：

```cpp
class ThreadRecordingField : public ImplicitField {
public:
  double eval(const V3d &p) const override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    threads_.insert(std::this_thread::get_id());
    return p[0] + p[1] + p[2];
  }
};
```

注意这个测试只应该在 TBB/OpenMP 可用时断言 `threads_.size() > 1`，否则只断言结果正确。

### 2. Build verification

建议命令：

```bash
conda run -n libpgo cmake --preset base
conda run -n libpgo cmake --build --preset base --target implicitSurface_gtest -j8
conda run -n libpgo ctest --test-dir build/base -R "Parallelism|ImplicitField|GridField" --output-on-failure
```

pypgo 侧：

```bash
conda run -n libpgo cmake --preset pypgo
conda run -n libpgo cmake --build --preset pypgo -j8
conda run -n libpgo python -m pytest tests/pypgo/test_implicit.py -q
```

## 迁移步骤

### P1. 新增 `core/parallelism` 基础库

`[M] [med-risk]`

- 新增 `parallelOptions.h`、`parallelFor.h`、`parallelFor.cpp`。
- CMake 中注册 `parallelism` target。
- 实现 `Backend::Serial`、`Backend::Auto`、TBB backend、OpenMP backend。
- 添加基础 C++ tests。

风险：

- CMake dependency 顺序需要处理好，避免 `parallelism` 反向依赖 `eigenSupport`。
- OpenMP 可用性当前使用全局 `OPENMP_FOUND`，需要确认变量在子目录可见。

### P2. 迁移 `ImplicitField::sampleToGrid`

`[S] [low-risk]`

- `ImplicitField.cpp` include `parallelism/parallelFor.h`。
- 删除 OpenMP pragma 分支。
- `implicitSurface` 链接 `parallelism`。
- 更新 `ImplicitField.h` 中 `numThreads` 注释。
- 跑 implicit surface C++ tests 和 Python implicit tests。

风险：

- 若 `eval()` 实现不是 thread-safe，TBB 并行会暴露已有问题。当前 `SphereField`、`BoxField`、`GridField`、lazy boolean/offset 都应是 const read-only；`MeshUnsignedDistanceField::sampleToGrid` 已有自己的 batch override 时需单独确认。

### P3. 审计简单 TBB loops

`[M] [low-risk]`

- 用 `rg "<tbb/parallel_for.h>|tbb::parallel_for|#pragma omp"` 列出候选。
- 标记三类：
  - simple loop：可迁移
  - reduce/TLS/lock：暂缓
  - performance-tuned：暂缓
- 每次只迁移一个模块，避免全仓库行为同时变化。

### P4. 逐模块迁移 simple loop

`[L] [med-risk]`

推荐顺序：

1. `implicitSurface`
2. `interpolationCoordinates`
3. `mesh` 中简单 vertex/triangle loops
4. `volumetricMesh` 中无锁 simple loops
5. `simulation` 中无 reduction 的 simple loops

每个模块迁移后单独跑对应 C++ tests。

### P5. 评估是否需要 Python runtime API

`[S] [low-risk]`

等至少两个 Python public modules 都需要统一线程控制后，再决定是否新增：

```python
pypgo.runtime.set_num_threads(n)
pypgo.runtime.num_threads()
```

第一版不暴露 `parallel_for` 给 Python。

## 风险评估

### 1. nested parallelism

TBB 默认能较好处理 nested parallelism，但如果外层 solver 和内层 sampling 都并行，仍可能 oversubscribe。`Options::numThreads` 和未来的 global runtime config 可以作为控制入口。

### 2. OpenMP 与 MKL/Pardiso

顶层 CMake 已注释 “openmp may cause mkl pardiso error”。因此 OpenMP backend 应保留，但默认不应优先于 TBB。指定 `Backend::OpenMP` 时要让调用者明确承担风险。

### 3. header 隔离

chunk-based type erasure 可以避免业务代码直接 include TBB/OpenMP，但也带来一次 chunk-level `std::function` 调用开销。对简单数值循环一般可以接受；若某个热点对 overhead 极敏感，可以保留直接 TBB 或后续提供 header-only fast path。

### 4. 行为确定性

简单逐点写入独立 buffer 的循环是确定性的。涉及浮点 reduction 的循环不要在第一阶段迁移，否则 reduction order 会改变结果。

## 完成标准

- `src/core/parallelism` 存在并被 CMake 构建。
- `ImplicitField::sampleToGrid` 不再直接使用 OpenMP pragma 或 TBB include。
- `sampleToGrid(numThreads=0/1/>1)` 数值结果与旧实现一致。
- `src/core/implicitSurface` 只 include `parallelism/parallelFor.h`，不直接 include TBB/OpenMP。
- `rg "<tbb/parallel_for.h>|#pragma omp" src/core/implicitSurface` 无命中。
- C++ implicit surface tests 和 Python implicit tests 通过。
