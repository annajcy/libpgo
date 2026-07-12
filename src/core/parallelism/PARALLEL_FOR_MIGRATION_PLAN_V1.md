# pgo::parallel parallel_for V1 迁移计划

## Source Spec

本计划基于以下已确认设计与讨论：

- `src/core/parallelism/REFACTOR_PLAN.md` 定义的唯一 `ParallelRuntime`、不可变
  `ParallelExecutor`、executor 继承和 `NestedKernelPolicy` 语义。
- libpgo core 不引入或调用 OpenMP runtime API；mixed TBB/OpenMP runtime 只作为文档化风险。
- 仓库最终目标是：除 `src/core/parallelism` 的 TBB backend、parallelism 白盒测试和专用 benchmark
  外，生产代码不直接调用 TBB 调度 API。
- V1 先迁移当前 `pgo::parallel::parallelFor` 能无损表达的调用；无法表达的调用保留并形成后续 API
  需求清单，不在本轮临时扩展公共 API。

## Objective

将生产代码中可由现有 `pgo::parallel::parallelFor(int, int, Options, Fn)` 表达的裸
`tbb::parallel_for` 收敛到 `pgo::parallel`，使这些循环统一遵循 process runtime ceiling、default
executor、nested executor 继承和 participant telemetry。

V1 同时产出一份精确的 deferred inventory，作为 V2 是否增加 typed index、chunk body、partitioner、
reduction 和 thread-local API 的事实依据。

## Non-Goals

- 不新增 `pgo::parallel` 公共 API。
- 不迁移 `tbb::parallel_reduce`、`tbb::enumerable_thread_specific`、`tbb::combinable`、
  `tbb::spin_mutex`、TBB allocator 或 observer/arena diagnostics。
- 不改变算法、数据布局、归约顺序、异常行为或浮点容差。
- 不把 `size_t`、`Eigen::Index` 或模板 index 强制窄化为 `int`。
- 不把依赖 range-local scratch 的循环改成每 index 分配 scratch。
- 不模拟 `tbb::static_partitioner` 的调度或 determinism 语义。
- 不在本轮决定哪些历史循环应启用 nested BLAS suppression。
- 不要求 V1 后生产代码中的全部 TBB include/link dependency 消失；仍有 deferred TBB primitive 的模块继续保留依赖。

## Repo Findings

截至计划编写时，`src/core/parallelism` 之外的生产代码包含：

| TBB primitive | 调用/引用数 | 文件数或说明 |
|---|---:|---|
| `tbb::parallel_for` | 160 | 39 个文件 |
| `tbb::parallel_reduce` | 21 | V1 不迁移 |
| `tbb::enumerable_thread_specific` | 46 | V1 不迁移 |
| `tbb::combinable` | 13 | V1 不迁移 |
| `tbb::static_partitioner` | 50 | 相关循环 V1 不迁移 |

裸 `parallel_for` 分布在 C API 和以下 core 模块：

- `constraintPotentialEnergies`
- `contact`，包括 sampled penalty 和 IPC
- `eigenSupport`
- `geometryPotentialEnergies`
- `interpolationCoordinates`
- `mesh`
- `nonlinearOptimization`
- `solidDeformationModel`
- `volumetricMesh`

当前只有 `solidDeformationModel`、`implicitSurface` 和 Python module 已显式依赖 `parallelism`。
迁移其他模块时需要增加实现依赖。`parallelism` 本身不依赖上述 core library，因此按当前 target graph
加入依赖不会形成反向环。

现有 `parallelFor` 的边界是：

- index 必须是 `int`，range 为 `[begin, end)`。
- `Options::grainSize` 可表达 blocked range 的 grain size，但不能表达显式 partitioner。
- public body 是 per-index callback；内部 chunk callback 不是公共 API。
- 默认 `NestedKernelPolicy::Suppress` 会改变历史裸 TBB 循环中的 nested BLAS 行为。

因此 V1 的迁移调用统一显式指定 `NestedKernelPolicy::Inherit`。这既保持原有 nested kernel 行为，也把
“调度 runtime 迁移”和“nested BLAS 策略优化”拆成两个可独立验证的变化。后续只有经过 benchmark 或
明确语义审计的调用点才改为 `Suppress`。

## Eligibility Rules

每个裸 `tbb::parallel_for` 必须按以下规则分类，分类结果记录到本计划的 inventory 附录或单独的
`PARALLEL_FOR_DEFERRED.md`。不得只按文本形式批量替换。

### V1 可迁移

满足以下全部条件的调用必须在 V1 迁移：

1. 迭代空间可证明落在 `int`，且原调用本身使用 `int` boundary 或 `blocked_range<int>`。
2. 每个 index 的工作相互独立；共享写入已经由原代码通过不相交位置、原子量或现有锁保证。
3. range body 只是 `for (int i = range.begin(); i < range.end(); ++i)` 的包装；range 对象没有其他用途。
4. range 外没有按 chunk 构造并复用的 mutable scratch、accumulator 或第三方 context。
5. 没有显式 `static_partitioner`、`affinity_partitioner` 或依赖稳定 index-to-thread mapping 的逻辑。
6. 不读取 `this_task_arena::current_thread_index()`，不按 scheduler thread index 索引 scratch。
7. 不属于 reduction；循环完成后的 merge 若依赖 ETS/combinable，则整个调用留给后续迁移。

迁移形式统一为：

```cpp
pgo::parallel::parallelFor(
  begin, end,
  pgo::parallel::Options{
    .nestedKernelPolicy = pgo::parallel::NestedKernelPolicy::Inherit,
  },
  [&](int i) {
    // 原 per-index body
  });
```

原 `blocked_range<int>(begin, end, grain)` 若满足其他条件，则同时设置 `.grainSize = grain`。原调用未
指定 grain 时保持 `grainSize == 0`，使用当前 pgo backend 的默认行为。

### V1 延后

遇到以下任一情况，保留原 TBB 调用并登记唯一原因：

| Deferred category | 典型形态 | 后续可能需要的能力 |
|---|---|---|
| `typed-index` | `size_t`、`Eigen::Index`、模板 `IDX` | checked typed overload 或明确的 index policy |
| `chunk-scratch` | range 外创建 vector/context，range 内复用 | public `parallelForChunks` 或 scratch factory |
| `partitioner` | `static_partitioner`/`affinity_partitioner` | partitioner policy，或证明可删除该语义 |
| `tls-coupled` | ETS/combinable/thread-index scratch | pgo-owned TLS/reduction abstraction |
| `reduce` | `parallel_reduce` 或 parallel-for 后 merge | `parallelReduce` 设计 |
| `non-index-range` | 自定义 range 或二维 range | typed/custom range API |

同一调用若命中多个原因，记录最先阻止安全迁移的主原因，并可附 secondary reasons。

## Phases

### Phase 1: 建立逐调用 inventory 和迁移护栏

**Purpose**

冻结迁移基线，避免实施过程中漏掉调用、误把 TBB primitive 的文本引用当成可迁移循环，或新增未审计
的裸 `parallel_for`。

**Expected files**

- `src/core/parallelism/PARALLEL_FOR_DEFERRED.md`（新增）
- 仓库现有测试基础设施中增加一个只检查 production source 的静态检查脚本和 CTest entry

**Implementation steps**

1. 用 `rg` 重新生成 160 个 production 调用的基线；忽略注释中的死代码，但在 inventory 标记并单独删除
   明确无用的注释调用。
2. 为每个调用记录 `file:line`、原 range/index type、partitioner、scratch/TLS coupling、分类结果。
3. 静态检查仅禁止新增未登记的裸 `tbb::parallel_for`；允许 `src/core/parallelism` backend、白盒测试、
   benchmark 和 deferred inventory 中已有的调用。
4. 护栏采用稳定的 `path + enclosing function + occurrence` 标识或每文件预期数量，不依赖容易漂移的精确
   行号。

**Validation budget:** `static`

**Done criteria**

- 每个 production 调用恰好有一个 `migrate-v1` 或 deferred category。
- inventory 总数与 `rg` 基线一致。
- 护栏能对一个临时新增的裸调用失败，并在删除临时调用后通过。

### Phase 2: 迁移低耦合 per-index 循环

**Purpose**

先处理边界为 `int`、body 直接按 index 写不相交输出、没有 TLS/partitioner 的低风险调用，验证依赖接入
方式和显式 `Inherit` 策略。

**Expected modules/files**

- `src/c/pgo_c.cpp`
- `src/core/interpolationCoordinates/barycentricCoordinates.cpp`
- `src/core/volumetricMesh/generateMassMatrix.cpp`
- `src/core/volumetricMesh/volumetricMesh.cpp`
- `src/core/mesh/labelOuterTets.cpp`
- `src/core/mesh/meshIntersection.cpp`
- `src/core/mesh/triMeshPseudoNormal.cpp` 中符合 eligibility rules 的调用
- `src/core/mesh/boundingVolumeTree.cpp` 中符合 eligibility rules 的调用
- 上述 module 的 `CMakeLists.txt`

**Implementation steps**

1. 增加 `parallelism/parallelFor.h` include，替换 eligible 调用并显式设置 `Inherit`。
2. 对简单 `blocked_range<int>` 展开 range wrapper，只移动原 per-index body，不改其内部计算。
3. `size_t`、ETS、显式 partitioner 和 chunk-local scratch 调用留在原地并更新 deferred inventory。
4. 仅当一个 translation unit 已无 TBB 使用时删除其 TBB include；仅当整个 target 已无 TBB 使用时删除
   direct `TBB::tbb` dependency。
5. target 对 `parallelism` 的依赖按实际公开面设置：只在 `.cpp` 使用时为 `PRIVATE`；安装/public header
   出现 pgo parallel 类型时才为 `PUBLIC`。

**Tests and validation**

- 构建 `pgo_c`、`mesh`、`volumetricMesh` 及其依赖。
- 运行 mesh、volumetric mesh、C API 现有测试；缺少专门测试的算法至少增加 serial-result parity test。
- 在 runtime concurrency 为 1 和大于 1 的子进程中各跑一次代表性路径，结果必须一致。

**Validation budget:** `targeted-tests`

**Done criteria**

- 本阶段 inventory 中所有 `migrate-v1` 调用已迁移。
- 结果和异常行为不变；无新的 direct TBB target dependency。

### Phase 3: 迁移数值与能量模块的 eligible 循环

**Purpose**

迁移 Eigen-heavy 和能量计算中的 per-index 循环，同时避免误迁 reduction、typed index 和 scratch-coupled
路径。

**Expected modules/files**

- `src/core/eigenSupport/EigenSupport.cpp`
- `src/core/nonlinearOptimization/energy/energySet.cpp`
- `src/core/constraintPotentialEnergies/*.cpp`
- `src/core/geometryPotentialEnergies/surfaceSmoothnessAbsoluteMeanCurvature.cpp`
- `src/core/geometryPotentialEnergies/surfaceTriangleDeformation.cpp`
- `src/core/solidDeformationModel/constraints/tetVolumeConstraintFunctions.cpp`
- `src/core/solidDeformationModel/simulation/tetMeshOccupation.cpp`
- 对应 module `CMakeLists.txt`

**Implementation steps**

1. 只迁移 inventory 中 boundary 已是 `int` 且不依赖 TLS/reduction/partitioner 的调用。
2. `Eigen::Index`、`IDX`、`size_t` 调用不做 cast；登记为 `typed-index`。
3. 保留 `parallel_reduce` 和为其准备/消费 ETS、combinable 的相邻循环，避免拆散同一并行算法。
4. nested `parallel_for` 若内外均 eligible，则一起迁移；inner 无 executor overload 会自然继承 outer executor。
5. 显式 `Inherit` 保持 Eigen/MKL/Accelerate kernel 的历史线程策略；本阶段不根据 body 猜测是否可
   `Suppress`。

**Tests and validation**

- 运行 constraint、energy、solver、solid deformation model 的现有 gtest。
- 对组装 matrix/vector 的代表性路径比较 serial 与 configured-parallel 结果，使用现有数值容差。
- 对 nested eligible 路径增加测试，确认不会切换 executor，且结果与迁移前基线一致。

**Validation budget:** `targeted-tests`

**Done criteria**

- 所列模块中全部 `migrate-v1` 调用完成迁移。
- typed/reduce/TLS/partitioner 调用保持原实现且完整登记。

### Phase 4: 迁移 contact、sampled penalty 和 IPC eligible 循环

**Purpose**

处理调用数量最多、共享数据结构和 broad-phase 模板较多的模块，并以更强的 correctness validation
覆盖高风险迁移。

**Expected modules/files**

- `src/core/contact/surfaceDofMap.cpp`
- `src/core/contact/sampled_penalty/kernels/*.cpp`
- `src/core/contact/ipc/external/obstaclePoseCache.cpp`
- `src/core/contact/ipc/broadPhase/surfaceIPCBroadPhaseInternal.h`
- `src/core/contact/ipc/core/*.cpp`
- `src/core/contact/CMakeLists.txt`

**Implementation steps**

1. 优先迁移 AABB 构建、独立 pair/sample evaluation、按 row 写不相交输出等简单循环。
2. 逐个确认共享 vector/map 的写入方式；依赖 ETS、锁、后续 merge 或隐含顺序的调用保持 deferred。
3. internal header 中的 eligible template 调用迁移后，保证所有实例化 target 都能看到
   `parallelism/parallelFor.h`，并正确传播 link dependency。
4. 与 `parallel_reduce` 共用 scratch、candidate list 或 active-set cache 的并行阶段作为整体 deferred，
   不只替换其中表面上简单的一段。
5. 每完成 sampled penalty、IPC broad phase、IPC assembler 一个子域就运行对应测试，避免把 race 或
   数值偏差积累到阶段末尾。

**Tests and validation**

- 运行全部 `tests/src/core/contact/*gtest.cpp` 对应 target。
- 对 empty geometry、single primitive、degenerate primitive、self/external contact 和 cached refresh 路径
  做 serial/parallel parity。
- 重复运行 race-sensitive contact tests；建议至少 20 次 targeted repetition。

**Validation budget:** `full-suite`

**Done criteria**

- contact inventory 中全部 `migrate-v1` 调用完成迁移。
- contact 全套测试和重复测试通过，无 sanitizer/race symptom。

### Phase 5: 依赖收敛、全仓验证与 V2 输入

**Purpose**

确认 V1 没有遗漏，删除已经不需要的 direct TBB 依赖，并把剩余问题收敛为 API 设计数据而不是散落的
TODO。

**Expected files**

- 各受影响 module 的 `CMakeLists.txt`
- `src/core/parallelism/PARALLEL_FOR_DEFERRED.md`
- 静态检查/CTest entry
- 必要的模块测试

**Implementation steps**

1. 重新运行 inventory 命令；剩余 production `parallel_for` 必须与 deferred manifest 一一对应。
2. 删除 translation unit 中无用的 TBB headers；删除 target 中已无任何 TBB primitive 所需的 direct
   `TBB::tbb` link。
3. 检查 include/link visibility，避免仅因其他 PUBLIC dependency 偶然传递 TBB 或 `parallelism`。
4. 汇总每个 deferred category 的数量、涉及模块和代表调用；不在 V1 中直接选择 V2 API。
5. 记录迁移前后 build/test 命令和 raw call count delta。

**Validation budget:** `full-suite`

**Done criteria**

- 所有符合 V1 eligibility rules 的 production 调用已迁移。
- 剩余裸 `parallel_for` 全部存在于 deferred manifest，且静态护栏禁止新增未登记调用。
- full build、full CTest、Python tests 和 `git diff --check` 通过。
- macOS Accelerate 和 Linux MKL-TBB nested benchmark 不要求重新做完整 sweep，但至少各做一次 smoke run，
  确认显式 `Inherit` 的迁移路径没有改变 benchmark/runtime 初始化行为。

## Validation

实施时先根据当前 preset 名称确认命令，不硬编码不存在的 build directory。预期验证层级如下：

```bash
# Inventory baseline / final delta
rg -n --glob '!src/core/parallelism/**' '\btbb::parallel_for\b' src

# Configure/build: 使用 libpgo conda 环境和仓库 base preset
conda run -n libpgo cmake --preset base
conda run -n libpgo cmake --build --preset base -j

# Full native tests（仓库当前没有 test preset）
conda run -n libpgo ctest --test-dir build/base --output-on-failure

# Python tests（以仓库当前可用命令为准）
conda run -n libpgo python -m pytest tests/pypgo

git diff --check
```

每个 phase 还应运行对应 module 的 targeted tests。若 base preset 的实际 build/test preset 名称不同，实施者
先用 `cmake --list-presets=all` 记录 repo truth，再使用存在的命令，不修改 preset 只为匹配计划文本。

建议在支持的 CI/local 配置中增加一次 sanitizer run；V1 不以所有平台 sanitizer 可用为硬性完成条件，
但 contact 阶段出现 nondeterministic failure 时必须先完成 sanitizer/race diagnosis。

## Risks and Rollback

1. **Nested BLAS 行为变化。** 默认 `Suppress` 与裸 TBB 不同。V1 通过所有迁移调用显式 `Inherit` 控制
   风险；遗漏显式 policy 视为迁移缺陷。
2. **Partitioning/performance 变化。** pgo backend 使用 `blocked_range<int>` 和 auto partitioning；显式
   static/affinity partitioner 不迁移。普通 direct index overload 仍可能有细微分块差异，因此 correctness
   通过后保留 benchmark 对比。
3. **Index 窄化。** 禁止为了调用现有 API 将 `size_t`/`Eigen::Index` 静默 cast 为 `int`。若能够用已有
   invariant 证明上界，仍先记录证据并在 plan review 中确认，实施者不自行扩大范围。
4. **Range-local allocation 回归。** 不把 chunk scratch 变成 per-index allocation；相关调用整体 deferred。
5. **Dependency visibility。** 新增 `parallelism` target dependency 时可能暴露已有的偶然传递依赖。每个
   target 独立构建，并在移除 TBB link 后重新 configure。
6. **并发 bug 被调度变化暴露。** 原算法若依赖偶然分块/顺序，迁移后可能失败。发生时回退该单个调用到
   deferred inventory，不回滚已经验证通过的其他模块。

本迁移按 module batch 保持小提交/小 diff。回滚单位是单个调用或单个 module batch，不使用全仓重置，
也不混入算法重构。

## Plan Drift Triggers

遇到以下情况停止对应调用或阶段并报告，不自行设计新 API：

- eligible 调用需要 `size_t`、`Eigen::Index`、custom range 或超过 `INT_MAX`。
- 保持性能需要 chunk-local scratch，而 public per-index body 无法表达。
- correctness 或 determinism 依赖 static/affinity partitioner。
- 调用与 ETS/combinable/reduce 生命周期耦合，无法独立替换。
- 显式 `Inherit` 仍改变 MKL/Accelerate 行为，或迁移触发 mixed OpenMP runtime 风险。
- 添加 `parallelism` dependency 形成实际 CMake target cycle。
- serial/parallel parity、contact repetition 或 full suite 出现无法归因于已知基线的失败。
- 为完成 V1 必须修改公共 ABI、Python API 或 `ParallelRuntime` 生命周期语义。

## Done Criteria

V1 只有同时满足以下条件才完成：

1. 基线中的 160 个 production `tbb::parallel_for` 全部被分类。
2. 所有满足 eligibility rules 的调用使用现有 `pgo::parallel::parallelFor`，并显式采用
   `NestedKernelPolicy::Inherit`。
3. 所有剩余调用都在 deferred manifest 中，并有稳定标识和单一主原因。
4. 没有发生 index 窄化、partitioner 语义删除、chunk scratch 粒度变化或隐式 nested BLAS policy 变化。
5. 不再需要 TBB primitive 的 translation unit/target 已删除相应 include/direct dependency。
6. 静态护栏、targeted tests、full native suite、Python tests 和 diff check 通过。
7. handoff 报告迁移数量、剩余数量、按 deferred category 的分布、测试结果和任何 benchmark delta。

## Reviewer Handoff

- **Decision source:** `src/core/parallelism/REFACTOR_PLAN.md` 和本轮“先迁移现有 API 能表达的调用、再讨论
  新 API”的用户决策。
- **Repo areas inspected:** `src/core/parallelism` public/backend API；production `src` 下所有裸
  `tbb::parallel_for`；相关 module `CMakeLists.txt`；现有 core/contact/solid deformation tests。
- **Planner assumptions:** V1 以行为保持为先，因此统一显式 `Inherit`；`int` per-index loop 是现有 API
  唯一无争议的迁移单元；测试和 benchmark 可在 `libpgo` conda 环境运行。
- **Reviewer must verify:** 160/39 inventory 数字；每个 listed module 的真实 target dependency direction；
  `parallelFor` 默认 grain/partitioning 与 direct TBB overload 的可接受差异；是否接受 V1 全部显式
  `Inherit`；静态护栏的稳定标识方案。
- **Validation budget:** 低风险模块 targeted tests，contact 和最终阶段 full suite；原因是调度替换可能
  暴露潜在数据竞争，单纯编译不足以验收。
- **Known risks/drift:** typed index、chunk scratch、partitioner、TLS/reduce coupling、CMake cycle、nested
  kernel policy 和 nondeterministic contact failures。
- **Suggested next skill:** `review-plan`。审查通过或用户明确接受后，再使用 `implement-plan`。
