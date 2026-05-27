# Phase 1A Plan: `SurfaceIPCCore` 抽取 + `CIPCPotentialEnergy` 薄 wrapper 化

Source plan: `plan/ipc/ipc_friction.plan.md`  
Target phase: `3.1 / A. 抽取 surface-space IPC core`

## Summary

`Phase 1A` 只做一件事：把当前 `src/core/contact/CIPC.{h,cpp}` 里已经验证过的 surface-space IPC 数学抽成一个独立的 `SurfaceIPCCore`，并在同一批中把 `CIPCPotentialEnergy` 改造成兼容旧 API 的薄 wrapper。

这一批完成后，`runShellSim` 不改调用入口，但其底层 self-contact 数学将不再直接由 `CIPCPotentialEnergy` 自己维护，而是统一委托给 `SurfaceIPCCore`。这样后续 `Phase 1B` 再接 `EmbeddedSurfaceIPCPotentialEnergy` 和 `runIPCSim` 时，不会再复制第二套 IPC 内核。

## Scope Lock

### In Scope

- 新增 `SurfaceIPCCore`，承接当前 `CIPC` 中真正属于 surface-space IPC 内核的职责：
  - surface topology 初始化
  - PT / EE pair build
  - barrier energy / gradient / Hessian
  - contact-feasible CCD max step
  - `computeEnergy / computeGradient / computeHessian / computeAll`
- 把 `CIPCPotentialEnergy` 改造成 compatibility wrapper：
  - 保留现有构造函数形状
  - 保留 `isInputDisp` 语义
  - 保留 `setMesh / func / gradient / hessianDirect / computeMaxStepSize`
  - 保留 `getPTPairs / getEEPairs`
- 让现有 `runShellSim` 在不改用户入口的前提下复用新 core。
- 同时落地并测试仓库级 profiling 基础设施，并在 contact 路径接入首批 section scope；运行时默认可关闭，本批不要求完整 profiling 报表输出。

### Out Of Scope

- 不新增 `EmbeddedSurfaceIPCPotentialEnergy`
- 不新增 `runIPCSim`
- 不接 tet / cubic runtime path
- 不实现 external contact
- 不实现 friction
- 不处理 inversion-free max step
- 不做 broad phase / spatial hash 优化重写

## Phase Boundary And Completion Standard

`Phase 1A` 的完成标准不是 “统一 IPC 已可用于 shell/tet/cubic”，而是更窄的：

- 仓库里只剩下一套真正的 surface-space IPC 数学实现，即 `SurfaceIPCCore`
- `CIPCPotentialEnergy` 不再直接维护 PT/EE、barrier、CCD、Hessian 主逻辑
- `runShellSim` 继续可用，并作为新 core 的第一条真实回归路径
- 核心数值结果在 shell 路径上与当前实现对齐
- 仓库级 profiling 基础设施已存在、可编译、可测试，contact 路径的首批 section 名称与插桩位置已固定

只要以上条件成立，`Phase 1A` 就可合入；`W`、simulation-space pull-back、tet/cubic 接线全部留给下一批。

## Key Design Decisions

### 1. `SurfaceIPCCore` 是新的 single source of truth

新类固定命名为：

```cpp
class SurfaceIPCCore
```

并放在现有命名空间下：

```cpp
namespace pgo::Contact::CIPC
```

它是仓库唯一的 surface-space IPC 内核，负责：

- surface mesh topology
- active PT/EE pair build
- barrier 数学
- CCD-based contact max step
- energy / gradient / Hessian / combined compute

它不负责：

- `isInputDisp`
- simulation-space DOF
- embedding `W`
- floor penalty

### 2. `CIPCPotentialEnergy` 只保留 legacy 兼容职责

`CIPCPotentialEnergy` 在 `Phase 1A` 后的定位固定为：

- shell/reference wrapper
- legacy API compatibility layer
- floor penalty 持有者

它不再拥有以下核心算法实现：

- `buildEdges`
- `buildAdjacency`
- `buildAreaWeights`
- `findCollisionPairs`
- `computeEnergy`
- `computeGradient`
- `computeHessian`
- `computeAll`
- self-contact 部分的 `computeMaxStepSize`

这些全部迁移到 `SurfaceIPCCore`。

### 3. floor penalty 不进入 `SurfaceIPCCore`

当前 `CIPC` 里唯一不属于统一 IPC 主链路的行为，是 `useFloor / floorHeight / floorKappa` 这组 legacy floor penalty。

本批固定采用下面的拆分：

- self-contact barrier / CCD：进入 `SurfaceIPCCore`
- floor penalty：继续留在 `CIPCPotentialEnergy` wrapper 内

这样做的原因是：

- source plan 的 core 语义是 “embedded-surface IPC 几何 + barrier 内核”
- floor penalty 不是 PT/EE barrier pair，也不会出现在未来 unified self/external/friction 设计里
- 把它继续留在 wrapper，可在不污染 core 的前提下保持旧 shell 路径行为兼容

### 4. wrapper 保留 legacy public field，core 收敛为可变的 `Parameters + setter/getter`

虽然当前仓库没有现成调用在外部直接修改 `dhat / kappa / eps_ee / slackness`，但 `CIPCPotentialEnergy` 目前把这些做成了 public field。

为避免引入静默行为变化，`Phase 1A` 固定采用下面的兼容策略：

- `CIPCPotentialEnergy` 继续保留这些 public field
- wrapper 在每个计算入口前调用一次 `syncCoreParametersFromWrapper()`
- `SurfaceIPCCore` 不再暴露这些参数为 public field，而是收敛为 private storage + `Parameters` struct + setter/getter
- wrapper 将当前自己的：
  - `dhat`
  - `kappa`
  - `eps_ee`
  - `slackness`
  打包后通过 `core.setParameters(...)` 同步到 `SurfaceIPCCore`
- `Phase 1A` 明确**不**把 `SurfaceIPCCore` 的参数设计成 immutable config；在当前 wrapper 仍保留 legacy public field、且每次 `const` 入口都需要同步参数的前提下，mutable parameter state 是有意选定的设计，而不是过渡状态

这保证：

- legacy wrapper API 不会因为 “参数在构造后改动但 core 没同步” 而失效
- 新增的 `SurfaceIPCCore` 从第一版开始就拥有清晰的参数 ownership，而不是继续沿用 public mutable field 风格
- 不会为了追求过早 immutable 而引入额外的 config 重建、对象替换或 wrapper 语义变更
- 在 wrapper 路径上，`CIPCPotentialEnergy` 是 core 参数的唯一 single source of truth；外部不得绕开 wrapper 直接写 `core.setParameters(...)`

此外，`setParameters(const Parameters &)` 的语义在 `Phase 1A` 中固定为：

- 只更新参数 storage
- 不触发任何 re-preparation、cache invalidation 或 mesh-side rebuild
- topology / adjacency / area weights 只在 `setMesh(...)` 中构造
- `dhat / kappa / eps_ee / slackness` 不参与 topology 预处理，因此参数更新与 `setMesh(...)` 解耦

### 5. active pair 只允许“重算后暴露”，不允许“跨调用复用”

`SurfaceIPCCore` 允许持有最近一次计算生成的 `ptPairs_ / eePairs_`，因为：

- `getPTPairs / getEEPairs` 需要观测结果
- wrapper 也要继续转发这些 accessors

但它**不允许**把这些当成跨调用 cache 使用。具体规则固定为：

- 每次 `computeEnergy / computeGradient / computeHessian / computeAll / computeMaxStepSize` 都重新 build 当前所需 active pairs
- pair vectors 的存在只是为了当前调用结果可被读取
- 不引入 `cached_x_surf`、dirty bit、跨调用复用判断或“如果位置没变就跳过 pair build”的逻辑

### 6. `findCollisionPairs(...)` 不再作为 public API 保留

当前 `CIPCPotentialEnergy` 把 `findCollisionPairs(const VXd &)` 暴露为 public 成员，但仓库内没有外部调用方；现有使用全部发生在 `CIPC.cpp` 内部。

`Phase 1A` 固定采用下面的收口策略：

- 从 `CIPCPotentialEnergy` 的 public API 中删除 `findCollisionPairs(const VXd &)`
- `getPTPairs / getEEPairs` 继续保留，作为“最近一次计算结果”的唯一对外观测口
- `SurfaceIPCCore` 内部仍可保留 `findCollisionPairs(...)` 这一 helper 名称，但默认只作为 private 实现细节，不作为对外能力暴露

除非后续测试或调试被明确证明需要直接驱动 pair build，否则不为 wrapper 或 core 保留一个多余的 public forwarder。

### 7. `mutable core` 是 wrapper API 设计的一部分

`CIPCPotentialEnergy` 的 `func / gradient / hessianDirect / computeMaxStepSize` 都必须继续保留为 `const`，因为它们遵循 `PotentialEnergy` 现有接口语义。

同时，`Phase 1A` 又要求 wrapper 在这些 `const` 入口里先执行：

- `syncCoreParametersFromWrapper() const`

把 wrapper 上的参数同步到真正干活的 `SurfaceIPCCore`。因此：

- `core` 必须声明为 `mutable SurfaceIPCCore core;`
- 这不是偶然的实现细节，而是 wrapper API 设计的一部分
- 目的不是放松 const-correctness，而是允许 wrapper 在 delegate 前更新内部实现状态，同时不改变其对外 `const` 接口语义

### 8. floor 从 fused 路径改为 wrapper post-pass 是有意的边界选择

`Phase 1A` 之后，floor penalty 不再与 self-contact barrier 共享同一个 fused 计算块；固定改为：

1. `SurfaceIPCCore` 先完成 self-contact 的 energy / gradient / Hessian
2. `CIPCPotentialEnergy` wrapper 再单独补 floor contribution

这条变化的设计意图是职责边界清晰化，而不是性能优先。其约束固定为：

- floor 的数学语义保持不变，最终结果仍然是 `self-contact + floor`
- 当 `useFloor == false` 时，不引入额外成本
- 当 `useFloor == true` 时，wrapper 额外增加一次 `O(numVerts)` 的顶点扫描来补 floor 项
- 已确认这点额外开销对当前 shell 规模可忽略

因此，后续 profiling 若看到 floor 对应一个独立的 post-pass section / timer，应视为设计上预期行为，而不是意外性能回退。

### 9. `CIPC_autogen.h / CIPC_autogen_ll.h` 的 include ownership 固定迁到 core

`Phase 1A` 之后，`CIPC_autogen.h` 与 `CIPC_autogen_ll.h` 只允许由 `surfaceIPCCore.cpp` 直接 include，用于支撑 surface-space IPC 数学实现。

对应约束固定为：

- `surfaceIPCCore.cpp` 负责直接依赖这两个 autogen 头
- `CIPC.cpp` 在 `Phase 1A` 后不得再直接 include 或直接依赖这两个头
- 如果 `CIPC.cpp` 在迁移完成后仍然保留对 autogen 头的直接依赖，应视为 `SurfaceIPCCore` 抽取未完成

这条 ownership 规则与 “`CIPC.cpp` 应显著缩小并只保留 wrapper 输入转换、参数同步、委托与 floor 补项” 的验收标准绑定在一起。

## Concrete API Shape

### `SurfaceIPCCore`

新增文件：

- `src/core/contact/surfaceIPCCore.h`
- `src/core/contact/surfaceIPCCore.cpp`

首版接口固定如下：

```cpp
class SurfaceIPCCore
{
public:
  struct Parameters
  {
    double dhat = 1e-1;
    double kappa = 0.1;
    double eps_ee = 0.0;
    double slackness = 1.0;
  };

  SurfaceIPCCore() = default;
  explicit SurfaceIPCCore(const Parameters &params);

  void setParameters(const Parameters &params);
  Parameters getParameters() const;

  void setMesh(const MXd &V, const MXi &F);

  double computeEnergy(EigenSupport::ConstRefVecXd x_surf) const;
  void computeGradient(EigenSupport::ConstRefVecXd x_surf, EigenSupport::RefVecXd g_surf) const;
  void computeHessian(EigenSupport::ConstRefVecXd x_surf, EigenSupport::SpMatD &H_surf) const;
  void computeAll(EigenSupport::ConstRefVecXd x_surf, double &energy, VXd &g_surf, SpMatD &H_surf) const;
  double computeMaxStepSize(EigenSupport::ConstRefVecXd x_surf, EigenSupport::ConstRefVecXd dx_surf) const;

  const std::vector<PTPair> &getPTPairs() const;
  const std::vector<EEPair> &getEEPairs() const;

  int getNumSurfaceVertices() const;
  int getNumSurfaceDOFs() const;
};
```

注意：

- 这里所有 `x_surf / dx_surf` 都是 absolute surface positions / displacements
- 不引入 `isInputDisp`
- `computeAll` 继续保留，因为当前 `CIPC.cpp` 已经有一套单次 broad phase 的 combined path
- `findCollisionPairs(...)` 不出现在 public 接口里；它默认是 core 的 private helper
- 参数 ownership 固定通过 `Parameters` struct 与 `setParameters/getParameters` 暴露；不再使用 public mutable field
- 当前版本明确采用“可变参数 + setter/getter”而不是 immutable config
- `setParameters(...)` 只更新参数 storage，不触发任何 mesh/topology re-preparation

### `CIPCPotentialEnergy`

`CIPC.h` 仍保留：

- 原构造函数
- public legacy params
- `setMesh`
- `func`
- `gradient`
- `hessian`
- `createHessian`
- `hessianDirect`
- `computeMaxStepSize`
- `getPTPairs`
- `getEEPairs`

其中：

- `hessian()` 与 `createHessian()` 继续作为 throw override 保留
- 这不是 wrapper 的功能职责，而是 `PotentialEnergy` 基类的 pure virtual 契约要求
- `Phase 1A` 只迁移 self-contact 数学实现，不改变这两个 legacy override 的存在性
- `findCollisionPairs(const VXd &)` 不再作为 wrapper public 成员保留
- `core` 明确声明为 `mutable SurfaceIPCCore core;`，以便 `syncCoreParametersFromWrapper() const` 能在 wrapper 的 `const` 入口中执行参数同步
- `syncCoreParametersFromWrapper() const` 通过 `core.setParameters(...)` 同步 wrapper public field，而不是直接写 core 成员
- floor contribution 固定通过 wrapper post-pass 补入，而不再与 self-contact core 保持 fused 实现

新增 wrapper 私有辅助函数：

```cpp
void syncCoreParametersFromWrapper() const;
VXd toSurfacePositions(EigenSupport::ConstRefVecXd x) const;
VXd toSurfaceDisplacements(EigenSupport::ConstRefVecXd dx) const;
double computeFloorEnergy(const VXd &x_surf) const;
void addFloorGradient(const VXd &x_surf, EigenSupport::RefVecXd grad) const;
void addFloorHessian(const VXd &x_surf, EigenSupport::SpMatD &hess) const;
```

这里的 `toSurfaceDisplacements(dx)` 在 `Phase 1A` 其实就是直接返回 `dx`，但仍然单独封装，原因是：

- wrapper 代码更容易读
- 下一批 adapter/pull-back 接线时，这个位置正好是自然扩展点

其中 `syncCoreParametersFromWrapper() const` 的推荐形状为：

```cpp
void CIPCPotentialEnergy::syncCoreParametersFromWrapper() const
{
  SurfaceIPCCore::Parameters params;
  params.dhat = dhat;
  params.kappa = kappa;
  params.eps_ee = eps_ee;
  params.slackness = slackness;
  core.setParameters(params);
}
```

wrapper-level 代码与测试都必须遵守 single source of truth 规则：

- wrapper-level 测试只允许通过 wrapper public field 改参数
- 不允许在 wrapper-level 测试或调用路径中绕开 wrapper 直接写 `core.setParameters(...)`
- core-level gtest 不受此限制，可直接对独立 `SurfaceIPCCore` 调用 `setParameters(...)`

## File Ownership Plan

### New file: `src/core/contact/surfaceIPCCore.h`

这个头文件成为以下声明的新归属地：

- `PTDistType`
- `EEDistType`
- `PTPair`
- `EEPair`
- `distance::*`
- `barrier::*`
- `ccd::*`
- `projectToPSD(...)`
- `SurfaceIPCCore`

`CIPC.h` 改为包含这个头文件，而不是继续自己声明整套 low-level IPC 类型。

### New file: `src/core/contact/surfaceIPCCore.cpp`

这个源文件承接以下实现：

- `packPair(...)`
- `AABB`
- `SpatialHash`
- `SurfaceIPCCore::setMesh`
- `SurfaceIPCCore::buildEdges`
- `SurfaceIPCCore::buildAdjacency`
- `SurfaceIPCCore::buildAreaWeights`
- `SurfaceIPCCore::findCollisionPairs`
- `SurfaceIPCCore::computeMaxStepSize`
- `SurfaceIPCCore::computeEnergy`
- `SurfaceIPCCore::computeGradient`
- `SurfaceIPCCore::computeHessian`
- `SurfaceIPCCore::computeAll`

以及当前 `CIPC.cpp` 中已有的：

- PT / EE distance implementation
- barrier implementation
- CCD implementation
- PSD projection

原则是：`surfaceIPCCore.cpp` 持有完整的 IPC 数学实现；`CIPC.cpp` 不再含这些大块数值代码。

这里的 `SurfaceIPCCore::findCollisionPairs` 指的是 core 内部 private helper，而不是新的 public API。

此外，`CIPC_autogen.h / CIPC_autogen_ll.h` 的直接 include ownership 也在这一批一并迁到 `surfaceIPCCore.cpp`；`CIPC.cpp` 在 Phase 1A 后不得再直接依赖它们。

### Modified file: `src/core/contact/CIPC.h`

保留 wrapper 类声明，并缩减为：

- legacy constructor + public params
- legacy `PotentialEnergy` interface
- `mutable SurfaceIPCCore core;`
- wrapper-owned state：
  - `isInputDisp`
  - `restPosition`
  - `allDOFs_`
  - `useFloor / floorHeight / floorKappa`

从该头文件删除：

- topology containers
- pair containers
- `findCollisionPairs(const VXd &)`
- `buildEdges / buildAdjacency / buildAreaWeights`
- `computeEnergy / computeGradient / computeHessian / computeAll`
- `vtx(...)`

### Modified file: `src/core/contact/CIPC.cpp`

该文件在 `Phase 1A` 后只保留：

- wrapper 输入语义转换
- wrapper -> core 参数同步
- wrapper -> core 委托
- floor penalty 的 energy / grad / Hessian 补项

它的尺寸应显著缩小；如果 `CIPC.cpp` 里还保留了大段 PT / EE / barrier / CCD 数学实现，说明 `Phase 1A` 没做干净。

### Modified file: `src/core/contact/CMakeLists.txt`

增加：

- `surfaceIPCCore.h`
- `surfaceIPCCore.cpp`

`contact` library 继续暴露原来的 `CIPC.h`，不改 target 名称。

此外，`Phase 1A` 还要同步处理 `src/core/profiling/` 的工程归属，使 `scopedProfileSection.h` 能被 `contact` 以及后续模块稳定 include。

## Profiling Infrastructure Plan

source plan 原本把 profiling 视为 Phase 4 的 instrumentation 议题，但在当前实现顺序下，profiling 基础设施本身已经变成 `Phase 1A` 的直接依赖。因此，`Phase 1A` 现在就落地并测试一套可复用的仓库级 profiling 基础设施；Phase 4A 不再“发明 profiler”，而是负责打开它、补齐剩余路径接线、输出统计并据此做优化。

本批固定做法：

- profiling 基础设施按“全仓库可复用”的方向设计，而不是只服务 `SurfaceIPCCore`
- `Phase 1A` 同时新增仓库级 profiling 模块：`src/core/profiling/`
- 基础设施层提供真实可用、默认关闭的 `ScopedProfileSection(std::string_view)` RAII 实现，而不是仅保留 no-op 占位符
- 基础设施层不使用模块私有枚举作为 section id，而是使用零拷贝的 `std::string_view`
- 不使用热路径动态 `std::string` 作为 section id，避免不必要的分配、拷贝和拼写漂移
- 本批要求对 profiling 基础设施本身增加独立单测；contact 只接首批 section，不要求本批就完成所有路径的输出与报表

推荐拆分为两层：

1. 仓库级通用基础设施
   - 例如 `ScopedProfileSection(std::string_view name)`
   - `Phase 1A` 中由 `src/core/profiling/scopedProfileSection.h/.cpp` 提供真实实现
   - 默认状态下可关闭采集，但符号、状态管理、聚合与测试能力必须在本阶段存在
   - Phase 4A 可以扩展其输出、汇总、开关与统计维度，但不再重定义基础 RAII 机制

2. contact 模块内的 section name 常量
   - 放在 contact 内部 header 中集中定义
   - 使用 `inline constexpr std::string_view`
   - 例如：

```cpp
class ScopedProfileSection
{
public:
  explicit ScopedProfileSection(std::string_view name);
  ~ScopedProfileSection();
};

namespace contact::profile
{
inline constexpr std::string_view kPairBuildStatic = "contact.surface.pair_build.static";
inline constexpr std::string_view kPairBuildSwept = "contact.surface.pair_build.swept";
inline constexpr std::string_view kMaxStepPT = "contact.surface.max_step_pt";
inline constexpr std::string_view kMaxStepEE = "contact.surface.max_step_ee";
inline constexpr std::string_view kEnergy = "contact.surface.energy";
inline constexpr std::string_view kGradient = "contact.surface.gradient";
inline constexpr std::string_view kHessian = "contact.surface.hessian";
inline constexpr std::string_view kCombined = "contact.surface.combined";
inline constexpr std::string_view kWrapperSync = "contact.wrapper.sync";
inline constexpr std::string_view kFloorPostPass = "contact.wrapper.floor_post_pass";
}
```

对应文件布局建议为：

- 仓库级 profiler 基础设施：
  - `src/core/profiling/scopedProfileSection.h`
  - `src/core/profiling/scopedProfileSection.cpp`
- contact 内部命名常量：`src/core/contact/surfaceIPCProfiling.h`

`src/core/profiling/` 在 `Phase 1A` 也需要同步完成 CMake 归属整理：

- 新目录必须挂到合适的父级 CMake 结构中，而不是只创建文件但不纳入工程组织
- profiling 模块应在 `Phase 1A` 就有清晰的 target / 源文件归属，保证 `contact` 及后续模块能稳定 include / link
- 若仓库现有 CMake 结构更适合单独建 profiling 子目录或独立库，也应在 `Phase 1A` 同步落地，而不是留到实现时临场决定

`surfaceIPCProfiling.h` 的定位固定为：

- contact 模块内部可复用
- 不作为面向用户的公共 API 设计
- 允许 `SurfaceIPCCore`、当前 wrapper、以及后续 `EmbeddedSurfaceIPCPotentialEnergy` 共享同一套 section 名字

section name 的命名规范也在 `Phase 1A` 一并固定：

- `contact.surface.*`：保留给 `SurfaceIPCCore` 内部的 surface-space IPC 数学阶段
- `contact.wrapper.*`：保留给 `CIPCPotentialEnergy` wrapper 的输入转换、参数同步、floor post-pass 等外围阶段
- 后续若引入 simulation-space adapter，则优先使用独立前缀，例如 `contact.adapter.*`
- `contact.surface.pair_build.static`：固定指 `findCollisionPairs(...)` 的静态 broad phase / active pair build
- `contact.surface.pair_build.swept`：固定指 `computeMaxStepSize(...)` 内 swept broad phase 的候选对构建
- 不再使用语义模糊的单一 `contact.surface.pair_build`

命名目标是让 profiling 输出在不看实现细节的情况下，也能一眼区分：

- 哪些时间属于 surface-space IPC core
- 哪些时间属于 legacy wrapper 或后续 adapter 的外围开销

这样做的目的有三点：

- 先把稳定的 section 边界与命名规范定下来
- 现在就把仓库级 profiling 基础设施做实并测通，避免 `Phase 1A` 之后继续依赖假符号或 no-op 占位逻辑
- 为后续 Phase 1B 及更广泛的仓库级 profiling 复用留出直接扩展路径

## Implementation Batches

### Batch 1: 拆声明与文件边界

目标：

- 创建 `surfaceIPCCore.h/.cpp`
- 创建 `src/core/profiling/` 与真实可用的仓库级 profiling 基础设施
- 把 low-level IPC 声明从 `CIPC.h` 移过去
- 先保证工程重新编译

验收点：

- `contact` target 能编过
- profiling 基础设施 target / include 归属清楚，相关测试能编过
- `CIPC.h` 只剩 wrapper 相关声明

### Batch 2: 迁 topology / pair build / CCD max step

目标：

- 先把 `setMesh` 与 topology helpers 迁到 core
- 再把 `findCollisionPairs` 与 `computeMaxStepSize` 迁到 core

原因：

- 这是 wrapper 和 core 最清晰的一刀
- 先把最强的“surface-only”部分挪走，可以尽快暴露边界问题

验收点：

- wrapper 的 `setMesh` 只是把 mesh 数据给 core，并同步自己的 `restPosition / allDOFs_`
- wrapper 的 `computeMaxStepSize` 只做：
  - 参数同步
  - `x -> x_surf`
  - `dx -> dx_surf`
  - floor 不参与 max step
  - 委托 core

### Batch 3: 迁 energy / gradient / Hessian / computeAll

目标：

- 完整搬走 self-contact 数学主体
- wrapper 只补 floor 项并转发 pair getters

验收点：

- `func` 路径：`toSurfacePositions -> core.computeEnergy -> floor energy`
- `gradient` 路径：`core.computeGradient -> addFloorGradient`
- `hessianDirect` 路径：`core.computeHessian -> addFloorHessian`
- 若保留 `computeAll` 作为 wrapper 内部复用点，也必须先 core 后 floor，而不是重写一套第二版本
- 这意味着 floor 从 fused 路径改为 post-pass；`useFloor == false` 时零额外成本，`useFloor == true` 时新增一次顶点扫描，属于有意的边界选择

### Batch 4: 回归与测试收口

目标：

- 增加 core-level gtests
- 增加 wrapper-level等价性测试
- 保证现有 `runShellSim` CLI regression 继续通过

## Test Plan

## New tests

新增目录：

- `tests/src/core/contact/CMakeLists.txt`
- `tests/src/core/contact/surfaceIPCCore_gtest.cpp`
- `tests/src/core/contact/cipcPotentialEnergy_gtest.cpp`
- `tests/src/core/profiling/CMakeLists.txt`
- `tests/src/core/profiling/scopedProfileSection_gtest.cpp`

并在 `tests/src/core/CMakeLists.txt` 中添加：

```cmake
add_subdirectory(contact)
add_subdirectory(profiling)
```

### `scopedProfileSection_gtest.cpp`

至少覆盖：

- profiling 基础设施在默认关闭时可安全构造 / 析构，不影响调用路径
- 打开采集后，重复进入同名 section 能产生稳定的聚合记录
- 不同 section name 以字符串标识独立聚合，不发生名称串扰
- snapshot / reset 一类测试辅助接口若被设计进基础设施，也必须在本阶段得到覆盖

这些测试的目标不是验证绝对耗时数值，而是验证：

- RAII 生命周期正确
- 默认关闭与显式开启语义正确
- 以 `std::string_view` 标识 section 的聚合逻辑正确

### `surfaceIPCCore_gtest.cpp`

至少覆盖：

- 小型 shell-like 三角网格上 energy/gradient FD 校验
- 小型 shell-like 三角网格上 gradient/Hessian FD 校验
- 构造会发生接近碰撞的 `x_surf, dx_surf`，验证 `computeMaxStepSize() < 1`
- 调用 `computeEnergy / computeGradient / computeHessian` 后，`getPTPairs / getEEPairs` 可读且内容稳定

所有 FD 校验统一固定为：

- 使用中心差分
- 步长 `h = 1e-5`
- gradient relative tolerance `1e-4`
- Hessian relative tolerance `1e-3`
- 评估点应选择在 barrier 接近激活但尚未进入奇异区的位置，避免贴近 `d = 0` 的数值不稳定区域

这些参数属于 `Phase 1A` 的固定测试策略，不在实现阶段临时调参决定；若未来需要修改，必须作为显式测试策略调整处理。

### `cipcPotentialEnergy_gtest.cpp`

至少覆盖：

- 在不引入 simulation-to-surface embedding 变换的前提下，wrapper 与 `SurfaceIPCCore` 的 self-contact数值结果对齐
- `isInputDisp == true` 路径：
  - `x` 被解释为位移
  - wrapper 的结果等于 `core(rest + x)` 加 floor 项
- `isInputDisp == false` 路径：
  - `x` 被解释为绝对位置
  - wrapper 的结果等于 `core(x)` 加 floor 项
- 修改 wrapper 的 public `dhat / kappa / eps_ee / slackness` 后再次计算，结果确实会通过 `core.setParameters(...)` 同步到 core 并反映在结果中
- `getPTPairs / getEEPairs` 与 core 的最近一次结果一致
- 单独增加一个 `barrier + floor simultaneously active` 等价性 case：
  - `useFloor = true`
  - 至少一部分顶点满足 `z < floorHeight`
  - 同时存在 active PT 或 EE pair，使 self-contact barrier 非零
  - 验证 `wrapper(x) == core(x_surf) + floor_contrib(x_surf)` 在 energy / gradient / Hessian 三个层面都成立

这个 case 是 `Phase 1A` 的必需测试，而不是可选增强，因为 “floor 留在 wrapper、self-contact 留在 core” 正是本阶段的关键职责切分；如果 CI 不覆盖“barrier 与 floor 同时 active”的状态，就无法真正验证这条设计决策。

### Existing tool regression to keep passing

不新增新的 shell CLI 行为测试，但要求现有：

- `RunShellSimCliLoggingGTest.LogFlagWritesCliOutputNextToConfig`

继续通过。因为这已经能证明：

- `runShellSim` 仍可启动
- 其动态路径仍能跑到接触相关初始化与 timestep 外围逻辑

## Validation Commands

`Phase 1A` 的最小验证命令固定为：

```bash
cmake --build --preset base_no_mkl_debug --target scopedProfileSection_gtest surfaceIPCCore_gtest cipcPotentialEnergy_gtest runSim_gtest
ctest --test-dir build/base_no_mkl_debug --output-on-failure -R "ScopedProfileSection|SurfaceIPCCore|CIPCPotentialEnergy|RunShellSimCliLoggingGTest"
```

如果本地不用 `base_no_mkl_debug`，只允许替换 build preset / build dir，不允许缩减测试范围。

## Risks And Countermeasures

- 风险：wrapper 与 core 参数不同步，导致 legacy public field 或 core `Parameters` ownership 失真
  - 对策：所有 wrapper 入口统一先走 `syncCoreParametersFromWrapper()`，并只通过 `core.setParameters(...)` 更新 core 参数

- 风险：profiling 插桩计划依赖一个仓库级 RAII 基础设施，但实际代码库中并不存在对应符号
  - 对策：`Phase 1A` 同时落地 `src/core/profiling/scopedProfileSection.h/.cpp` 的真实基础实现与测试，先保证插桩位置和调用约束可编译、可复用

- 风险：floor penalty 被错误迁入 core，污染后续 unified IPC 边界
  - 对策：在文档和代码里都明确 floor 只属于 wrapper

- 风险：`CIPC.cpp` 迁移后仍残留第二份 self-contact 数学逻辑
  - 对策：把 `CIPC.cpp` 收缩为输入转换 + 委托 + floor 补项；代码评审以此为硬标准

- 风险：pair getter 语义改变，影响现有调试方式
  - 对策：保持“最近一次计算后可读取”的行为，不改 getter 名称

- 风险：为了“优化”而过早加入 cache，破坏 core 的无状态边界
  - 对策：`Phase 1A` 明确禁止任何基于上次 `x_surf` 的复用逻辑

## Assumptions

- `runShellSim` 本批不需要改 CLI、配置或用户路径语义
- floor penalty 只为兼容 legacy shell 行为保留，不视作 unified IPC 主特性
- `SurfaceIPCCore` 继续使用当前 `CIPC_autogen.h/.ll.h` 和现有 PT/EE/CCD 数学，不做重写
- 不新增并行策略变化，不在本批讨论性能优化

## Comparison To Source Plan

- source plan 的 `3.1` 同时包含 `A-E` 五个子块；本文件把它明确收窄成仅做 `A`
- source plan 的 `2.2` 说 `CIPCPotentialEnergy` 最终应变成 compatibility wrapper；本文件按已确认决策，把这一步提前并入 `Phase 1A`
- source plan 提到为 Phase 4 留 profiling hook；本文件把它进一步收敛成“仓库级 `std::string_view` profiling 基础设施 + contact 内部命名常量”的方向，避免被锁死在局部枚举实现里
- source plan 没单独处理 legacy floor penalty；本文件明确将其留在 wrapper，不进入 core
- source plan 没细化 core 参数 ownership；本文件明确把新 `SurfaceIPCCore` 收敛为 `Parameters + setter/getter`，同时保留 `CIPCPotentialEnergy` 的 legacy public field

## Blocking Questions

当前没有阻塞 `Phase 1A` 落地的未决问题。

如果后续你希望把 floor penalty 也彻底从 `CIPCPotentialEnergy` 删除，那应该作为一个单独的 cleanup 决策处理，而不是混进这批 core 抽取里。
