# Phase 1.8 Plan: `runIPCSim` surface-mapped floor penalty 与可复用 mapped-surface base

Source plan: `/Users/jinceyang/Desktop/codebase/libpgo/plan/ipc/ipc_friction.plan.md`  
Depends on:
- `/Users/jinceyang/Desktop/codebase/libpgo/plan/ipc/phase1/phase1D.plan.md`
- `/Users/jinceyang/Desktop/codebase/libpgo/plan/ipc/phase1.5/phase1.5.plan.md`
Repo-truth reference:
- `/Users/jinceyang/Desktop/codebase/libpgo/plan/ipc/phase1/phase1D.impl.md`
- `/Users/jinceyang/Desktop/codebase/libpgo/plan/ipc/phase1.5/phase1.5.impl.md`

## Summary

`Phase 1.8` 的目标是补上 `runIPCSim` 的 optional floor penalty，同时把
`EmbeddedSurfaceIPCPotentialEnergy` 里已经证明正确的
“surface embedding + pull-back” 逻辑抽成一层可复用的基础设施，
避免后续每加一个 surface-based analytic energy 都复制一份：

- `x_surf = x_surf,rest + W u_sim`
- `g_sim = W^T g_surf`
- `H_sim = W^T H_surf W`

这一步完成后，仓库里应当具备下面这条清晰能力链：

- `shell / tet / cubic` 都能在 `runIPCSim` 中通过同一套 surface embedding 接 floor penalty
- IPC self-contact 仍由 `EmbeddedSurfaceIPCPotentialEnergy` 单独负责
- floor penalty 由新的 `EmbeddedSurfaceFloorPotentialEnergy` 单独负责
- 两者共享一层新的 `MappedSurfacePotentialEnergy` base，而不是各自复制 mapping / pull-back 模板代码
- `examples/cubic/box-with-sphere-xlite` 这类当前依赖 legacy external contact 的演示，
  可以被迁移成 `examples/ipc/cubic/box-with-sphere` 下的 unified IPC + floor example，
  并通过完整的 `runIPCSim -> convertAnimation -> .abc` 流程交付验证资产

这一步明确是 **soft analytic floor penalty**，不是 analytic IPC plane barrier。
因此它的职责是“提供 floor 能量、梯度、Hessian”，而不是“把 floor 纳入 CCD / contact-feasible max step”。

## Why This Is A Separate Phase

`phase1D` 已经把 `runIPCSim` 的 unified self-contact 主链路接通了，`phase1.5` 又补上了 material-feasible max step。

但当前 repo 还有一个明显缺口：

- `runIPCSim` 只有 embedded IPC self-contact
- 没有一个干净的“surface-mapped analytic penalty”扩展点
- 如果直接把 floor 硬塞进 `EmbeddedSurfaceIPCPotentialEnergy`，
  会把 IPC barrier 和 analytic obstacle penalty 混成一个类
- 如果完全不抽公共层，新加 `floor / ceiling / side wall / plane penalty`
  时又会重复写 `W u`, `W^T g`, `W^T H W`

因此 `Phase 1.8` 单独拆出来，专门解决 **surface-mapped analytic penalty 的代码形状**，
同时把 floor 作为第一种具体落地的 penalty。

这样可以保证：

- `Phase 2` 继续专注在真正的 IPC external contact
- `Phase 3` 继续专注 friction
- floor 这个当前最需要的功能不必等待 analytic obstacle 全家桶一起落地

## Scope Lock

### In Scope

- 新增一个通用的 abstract base：
  - `src/core/contact/mappedSurfacePotentialEnergy.h/.cpp`
- 让 `EmbeddedSurfaceIPCPotentialEnergy` 改为基于该 base 实现，而不是自己维护一套 mapping / pull-back 样板代码
- 新增一个独立的 `EmbeddedSurfaceFloorPotentialEnergy`
- 在 `runIPCSim` 的 shell / tet / cubic 三条路径里，
  通过相同的 `surfaceRestPositions + W` 语义可选挂接 floor penalty
- 新增 `runIPCSim` config 字段：
  - `use-floor`
  - `floor-axis`
  - `floor-height`
  - `floor-kappa`
- 把 floor 参数接入 `runIPCSimSetup` 和 runtime log
- 为 mapped-surface floor penalty 增加 core/contact 级单测
- 为 `runIPCSim` 增加 floor-enabled 的 CLI / setup smoke test
- 把 `examples/cubic/box-with-sphere-xlite` 迁移为一个新的 IPC cubic floor example：
  - 源 mesh 与动画语义来自 `examples/cubic/box-with-sphere-xlite`
  - legacy `external-objects` 接触被 floor penalty 替代
  - 新资产落在 `examples/ipc/cubic/box-with-sphere/`
  - 交付完整 `.abc` 输出
- 在 plan 中明确写入 example 验证步骤，而不是只把 example 视为“可选补充”

### Out Of Scope

- 不把 floor 纳入 `SurfaceIPCCore`
- 不实现 analytic plane barrier、analytic external obstacle、analytic wall family
- 不让 floor 参与 CCD 或 contact-feasible `computeMaxStepSize()`
- 不改 `ImplicitBackwardEulerEnergy` / `TRBDF2TimeIntegratorEnergy` 的 max-step 聚合语义
- 不重写 `CIPCPotentialEnergy`
- 不把 `runSim` / `runShellSim` 迁移到新的 floor class
- 不顺手实现 positive-side `ceiling` / wall 语义、双侧 wall、任意法向 plane
- 不引入新的 obstacle scene DSL
- 不在本 phase 里消除 `CIPCPotentialEnergy` 与新 floor class 在 floor 公式上的小规模重复

## Completion Standard

`Phase 1.8` 合入的标准固定为：

- 仓库新增一层可复用的 `MappedSurfacePotentialEnergy` base，
  并被 `EmbeddedSurfaceIPCPotentialEnergy` 实际使用
- 仓库新增 `EmbeddedSurfaceFloorPotentialEnergy`
- `runIPCSim` 在 `use-floor=true` 时，能为 shell / tet / cubic 三种路径都挂上 floor penalty
- `runIPCSim` 在 `use-floor=false` 时，行为与当前 repo-truth 保持一致
- `collisionHandler` 的 contact clamp 统计、logging 和 type-specific getter 不被 floor 功能污染
- floor penalty 的 unit test 覆盖：
  - energy
  - gradient
  - Hessian
  - 非 identity `W` 时的 pull-back
  - `computeMaxStepSize() == 1.0`
- `runIPCSim_gtest` 覆盖至少一条 floor-enabled shell smoke 和一条 floor-enabled volume smoke
- `runIPCSim` 的参数日志在 floor 启用时明确打印：
  - `use-floor`
  - `floor-axis`
  - `floor-height`
  - `floor-kappa`
- 仓库新增一个明确面向 floor 的端到端 cubic IPC example：
  - 目录：`/Users/jinceyang/Desktop/codebase/libpgo/examples/ipc/cubic/box-with-sphere/`
  - 来源：`/Users/jinceyang/Desktop/codebase/libpgo/examples/cubic/box-with-sphere-xlite/`
  - `external-objects` 被移除
  - `use-floor=true`
  - 提交完整仿真输出对应的 `.abc` 文件
- 文档、配置语义和测试假设之间不存在前后矛盾：
  - floor 是 soft penalty
  - floor 不参与 feasible alpha
  - floor 不承诺严格无穿透
  - `use-floor=true` 时必须显式提供 `floor-axis`、`floor-height` 和 `floor-kappa`

只要以上条件成立，就认为 `Phase 1.8` 完成。

## Repo Truth This Plan Builds On

当前 repo 已有以下现成事实，本 phase 直接复用：

- `src/core/contact/embeddedSurfaceIPCPotentialEnergy.cpp`
  已经实现了：
  - `u_sim -> W u_sim`
  - `x_surf = x_rest + W u_sim`
  - `g_sim = W^T g_surf`
  - `H_sim = W^T H_surf W`
- `src/core/contact/CIPC.cpp`
  已经有一套 floor penalty 公式：
  - `0.5 * floorKappa * dz^2`
  - `floorKappa * dz`
  - z-direction diagonal Hessian
- `runIPCSimSetup.cpp`
  已经统一生成：
  - `surfaceRestPositions`
  - `surfaceFromSimulationDispMap`
  - `collisionHandler`
- shell path 当前用 `W = I`
- tet / cubic 当前都用 barycentric interpolation 生成 `W`
- `tests/src/core/contact/testCIPCHelpers.h`
  已经提供 floor 的 reference helper：
  - `computeFloorEnergy`
  - `computeFloorGradient`
  - `computeFloorHessian`
- `tests/src/core/contact/embeddedSurfaceIPCPotentialEnergy_gtest.cpp`
  已经验证 embedded IPC wrapper 的 mapping / pull-back 数学
- 当前 `EmbeddedSurfaceIPCPotentialEnergy` 的实现本来就采用：
  - 每次评估构造新的 surface-space `SpMatD`
  - 再做 pull-back
  因此 `Phase 1.8` 抽 base 不会引入一类全新的 Hessian 分配模式；
  需要关心的是 floor penalty 在这一模式下的额外开销，而不是“抽基类后第一次出现 SpMat 分配”

因此 `Phase 1.8` 不是从零发明 floor 或发明 embedding，而是把：

- 已存在的 floor 公式
- 已存在的 mapped-surface pull-back 数学

整理成一套更可扩展的结构，并用一个真实的 cubic example 迁移来验证这套结构不是纸上设计。

## Key Design Decisions

### 1. 新增一层 `MappedSurfacePotentialEnergy` base，只负责 mapping / pull-back

新基类的职责固定为：

- 持有 `surfaceRestPositions_`
- 持有 `surfaceFromSimulationDispMap_`
- 持有 `simulationDOFs_`
- 负责：
  - 验证 simulation displacement size
  - 计算 `u_surf = W u_sim`
  - 计算 `x_surf = x_rest + W u_sim`
  - 从 surface-space pull back 到 simulation-space

它 **不** 负责：

- 自碰 barrier 数学
- floor 数学
- obstacle scene 管理
- contact clamp 统计
- active pair / topology / triangle mesh 语义

它暴露给派生类的 protected hook 固定为：

```cpp
virtual double computeSurfaceEnergy(EigenSupport::ConstRefVecXd surfacePositions) const = 0;
virtual void computeSurfaceGradient(
  EigenSupport::ConstRefVecXd surfacePositions,
  EigenSupport::RefVecXd surfaceGradient) const = 0;
virtual void computeSurfaceHessian(
  EigenSupport::ConstRefVecXd surfacePositions,
  EigenSupport::SpMatD &surfaceHessian) const = 0;
virtual double computeSurfaceMaxStepSize(
  EigenSupport::ConstRefVecXd surfacePositions,
  EigenSupport::ConstRefVecXd surfaceDisplacements) const;
```

默认 `computeSurfaceMaxStepSize(...)` 返回 `1.0`；
只有真正需要 contact-feasible alpha 的派生类才 override 它。

这样可以确保：

- floor 这种 soft penalty 不会被迫实现假的 CCD 逻辑
- IPC 这种 barrier-based energy 仍然可以保留自己的 `computeMaxStepSize()`

同时要把一个性能 trade-off 明确写死，避免后续实现者误解这层 API 是“性能最优解”：

- 本 phase 接受 “数学清晰度优先” 的 Hessian pull-back 路径
- 也就是 floor 先构造 surface-space 稀疏 Hessian，再通过 base 做 `W^T H_surf W`
- 这对当前 phase 是可接受的，因为它最大程度复用了已经在 embedded IPC wrapper 中验证过的 pull-back 形状
- 若后续 profiling 证明 floor Hessian pull-back 成本明显偏高，
  再单独开后续 phase，为 base 增加 triplet-level / rank-one accumulate fast path

换句话说，本 phase 不要求 floor Hessian 采用最优实现，只要求：

- 数学正确
- API 边界清楚
- 为后续优化保留空间

### 2. `EmbeddedSurfaceIPCPotentialEnergy` 保持“只做 IPC”这个边界

`EmbeddedSurfaceIPCPotentialEnergy` 在本 phase 中不再承担 floor 责任。

它的职责固定为：

- own `SurfaceIPCCore`
- 把 surface-space IPC barrier 的 `energy / gradient / Hessian / max-step`
  接到新的 mapped-surface base
- 继续保留：
  - `getContactClampCount()`
  - `getMinContactFeasibleAlphaThisSolve()`
  - `resetContactMaxStepStats()`

这一步里 **不允许**：

- 在 `EmbeddedSurfaceIPCPotentialEnergy` 中再加 `useFloor`
- 再向其构造函数塞 `floorHeight / floorKappa`
- 把 analytic penalty 和 self-contact barrier 混成一个 runtime object

原因很简单：

- IPC barrier 与 analytic floor penalty 在数学性质和 max-step 语义上不同
- 两者独立更利于日志、测试和后续扩展

### 3. 新增 `EmbeddedSurfaceFloorPotentialEnergy` 作为第一个具体的 analytic surface penalty

新的 floor 类固定命名为：

- `src/core/contact/embeddedSurfaceFloorPotentialEnergy.h`
- `src/core/contact/embeddedSurfaceFloorPotentialEnergy.cpp`

它继承 `MappedSurfacePotentialEnergy`，
只实现一个 axis-selectable Cartesian negative half-space soft penalty：

$$
E_{\text{floor}}(x_{\text{surf}})
= \sum_i \frac{1}{2} k_f \min(0, x_{i,a} - h_f)^2,\quad a \in \{x, y, z\}
$$

其 surface-space 导数固定为：

- 若 `x_{i,a} >= h_f`，该点无贡献
- 若 `x_{i,a} < h_f`：
  - gradient 只写入该点所选 axis 分量
  - Hessian 只在该点所选 axis 分量产生 `k_f`

对应的 simulation-space 结果由 base 自动 pull back：

- `g_sim = W^T g_surf`
- `H_sim = W^T H_surf W`

这里要明确钉死一个实现边界：

- `EmbeddedSurfaceFloorPotentialEnergy` 的 surface Hessian 在 surface-space 是对角的
- 但拉回到 simulation-space 后，若 `W != I`，
  则 **一般不再是对角矩阵**

因此实现里不允许“直接给 simulation 某个轴向 DOF 加对角项”的简化写法。

### 4. `runIPCSim` runtime 里保留 typed `collisionHandler`，额外 analytic penalty 走单独容器

当前 `runIPCSim` 的 `IpcSimulationContext` 只有一个：

- `std::shared_ptr<Contact::CIPC::EmbeddedSurfaceIPCPotentialEnergy> collisionHandler`

`Phase 1.8` 之后它保持不变，同时新增一个容器，例如：

```cpp
std::vector<std::shared_ptr<NonlinearOptimization::PotentialEnergy>> extraGeneralImplicitForceModels;
```

固定语义如下：

- `collisionHandler`
  - 专门用于 self-contact IPC
  - 保留其 contact clamp getter / logging 身份
- `extraGeneralImplicitForceModels`
  - 专门装 floor 这类额外 general implicit force model
  - 当前首个成员是 `EmbeddedSurfaceFloorPotentialEnergy`

主循环接线顺序固定为：

1. 先 clear general implicit models
2. 先 add `collisionHandler`
3. 再循环 add `extraGeneralImplicitForceModels`

这样可以避免：

- floor 抢走 `collisionHandler` 的语义
- 未来 analytic penalty 增加时又改主循环结构

### 5. floor config 只对 `runIPCSim` 生效，并且默认关闭

本 phase 固定新增以下 config 字段：

- `use-floor`
- `floor-axis`
- `floor-height`
- `floor-kappa`

解释规则固定为：

- `use-floor`
  - optional bool
  - default `false`
- `floor-axis`
  - 当 `use-floor=true` 时为 required string
  - 取值固定为 `x | y | z`
- `floor-height`
  - 当 `use-floor=true` 时为 required double
- `floor-kappa`
  - 当 `use-floor=true` 时为 required double

这套语义的理由是：

- `use-floor=false`
  保证旧 case 行为完全不变
- `use-floor=true` 时强制显式写出三个关键参数
  可以避免 setup / struct / log / future caller 之间出现多套默认值来源
- floor 是 analytic penalty，方向、阈值和刚度都属于 case-defining 参数
  因此本 phase 更偏向“显式配置优先”而不是“隐式继承优先”

若 `use-floor=false`，即使 config 里出现 `floor-axis` / `floor-height` / `floor-kappa`，
也只视为未启用的备用字段，不构成错误。

### 6. floor 的坐标解释固定在 “scale 后的 simulation/world 坐标”

为了避免后续实现者把 `floor-height` 理解成“未缩放 OBJ 坐标”，
本 plan 明确规定：

- `surface-mesh` 与 volume mesh 若存在 `scale`，会先按当前 repo-truth 完成缩放
- `surfaceRestPositions` 保存的是 **缩放后的** 坐标
- `floor-height` 也是在这套缩放后的坐标系里解释

也就是说：

- 若 `scale = 2.0`
- 原始 OBJ 要接触的 floor 所选轴向平面在 `y = 0`
- 且 `floor-axis = y`
- 则 `floor-height = 0.0` 仍表示缩放后世界坐标的 `y = 0`

这个语义必须写进计划，是因为 floor penalty 是解析障碍物；
一旦坐标解释摇摆，测试和例子就会互相打架。

### 7. floor 不参与 feasible-step，`computeMaxStepSize()` 固定返回 `1.0`

这一步必须明确写死，避免后面实施时有人把 floor 误接进 max-step 逻辑：

- `EmbeddedSurfaceFloorPotentialEnergy::computeSurfaceMaxStepSize(...)`
  固定返回 `1.0`
- mapped-surface base 的默认 `computeMaxStepSize(...)`
  也就是 `1.0`
- 只有 `EmbeddedSurfaceIPCPotentialEnergy` override 这个行为

这样 integrator 侧聚合结果自然是：

$$
\alpha_{\max} = \min(\alpha_{\text{contact}}, \alpha_{\text{material}}, 1.0, \ldots)
$$

这与本 phase 的产品定义完全一致：

- floor 只是 soft penalty
- 它影响 Newton system 和 line search 目标函数
- 但不提供硬可行域保证

### 8. `CIPCPotentialEnergy` 在本 phase 保持不动

虽然 `CIPC.cpp` 已经有 floor 公式，
但 `Phase 1.8` 不把它一起迁到新 base 上。

原因固定为：

- `CIPCPotentialEnergy` 带有 legacy `isInputDisp` 兼容语义
- 本 phase 的重点是给 `runIPCSim` 建一条可扩展的 mapped-surface analytic penalty 线
- 若连 `CIPCPotentialEnergy` 一起重构，scope 会从“新基类 + 新 floor 类”
  扩大成“legacy wrapper 整理”

因此本批允许保留小范围公式重复：

- `CIPCPotentialEnergy` 中仍有自己的 floor post-pass
- `EmbeddedSurfaceFloorPotentialEnergy` 实现同一数学公式

未来若需要再做统一 helper，可单独开 cleanup phase。

### 9. example 验证固定采用 `box-with-sphere-xlite -> ipc/cubic/box-with-sphere` 迁移

本 phase 不满足于“补几个 smoke test 就算 floor 可用”，
而是固定要求交付一个真实的 example migration。

迁移对象明确钉死为：

- source case:
  - `/Users/jinceyang/Desktop/codebase/libpgo/examples/cubic/box-with-sphere-xlite/`
- target case:
  - `/Users/jinceyang/Desktop/codebase/libpgo/examples/ipc/cubic/box-with-sphere/`

迁移语义固定为：

- 保留 source case 的 cubic volumetric mesh 与 display surface
- 保留“box 落到地面并稳定接触”的演示目标
- 去掉 legacy `external-objects`
- 不再依赖 `../bottom.1.obj`
- 用 `use-floor=true`、`floor-axis`、`floor-height`、`floor-kappa` 重建地面接触语义

这一步的意义不是追求和 legacy case 数值轨迹逐帧一致，
而是证明：

- floor penalty 的输入语义足以覆盖一个真实演示案例
- `runIPCSim` 的 cubic volume path 能稳定消费这类 analytic penalty
- `convertAnimation` 能继续消费新的 IPC output folder 并产出 `.abc`

因此 example migration 在本 plan 中是 **验收项**，不是“有空再补”的 polish。

## Concrete File Plan

### 1. `src/core/contact/CMakeLists.txt`

需要新增：

- `mappedSurfacePotentialEnergy.h`
- `mappedSurfacePotentialEnergy.cpp`
- `embeddedSurfaceFloorPotentialEnergy.h`
- `embeddedSurfaceFloorPotentialEnergy.cpp`

并把它们加入 `contact` library。

### 2. `src/core/contact/mappedSurfacePotentialEnergy.h/.cpp`

新增 abstract base，负责：

- common constructor validation
- `surfaceRestPositions_` flatten
- `simulationDOFs_` 初始化
- `func`
- `gradient`
- `hessian`
- `createHessian`
- `hessianDirect`
- `computeMaxStepSize`
- 以及 protected helper：
  - `computeSurfaceDisplacementsFromSimulationDisplacements(...)`
  - `computeSurfacePositionsFromSimulationDisplacements(...)`
  - `validateSimulationDisplacementSize(...)`

`hessian()` / `createHessian()` 的 public contract 保持与当前 embedded IPC wrapper 一致：

- 仍然要求走 `hessianDirect()`
- 不在本 phase 里改动整个 contact subsystem 对 Hessian API 的现有约定

### 3. `src/core/contact/embeddedSurfaceIPCPotentialEnergy.h/.cpp`

重构为继承 `MappedSurfacePotentialEnergy`。

要点固定为：

- 构造函数仍保留：
  - `surfaceRestVertices`
  - `surfaceTriangles`
  - `surfaceFromSimulationDispMap`
  - `ipcParams`
- `SurfaceIPCCore surfaceIPCCore_` 继续保留
- 只实现 surface-space hooks
- 不再自己维护：
  - surface displacement mapping
  - gradient pull-back
  - Hessian pull-back

现有 getter、profiling section 和 max-step 统计保持原样。

### 4. `src/core/contact/embeddedSurfaceFloorPotentialEnergy.h/.cpp`

新增 floor penalty 类。

建议额外定义一个很小的参数结构：

```cpp
struct FloorPenaltyParameters
{
  FloorAxis floorAxis = FloorAxis::INVALID;
  double floorHeight = std::numeric_limits<double>::quiet_NaN();
  double floorKappa = std::numeric_limits<double>::quiet_NaN();
};
```

理由是：

- 避免构造函数尾部堆两个裸 double
- 后续若加 `isEnabled`、`name` 或 analytic plane 参数时更容易演进
- 更重要的是避免 struct 默认值和 config 默认值形成两套不一致语义

这里的 contract 需要进一步钉死：

- `FloorPenaltyParameters` 只是一个 transport struct，不承担任何默认值决策
- `runIPCSimSetup` 在 `use-floor=true` 时必须显式读到并填满：
  - `floor-axis`
  - `floor-height`
  - `floor-kappa`
- 若 `use-floor=true` 但缺任一字段，setup 直接报 config error
- setup 在构造 `EmbeddedSurfaceFloorPotentialEnergy` 前必须显式填满该 struct
- floor 类构造时应 assert / validate：
  - `floorAxis` 是 `x | y | z` 中之一
  - `floorHeight` finite
  - `floorKappa` finite 且正

这样可以防止未来新调用方漏填参数后，意外落回某个隐式默认值。

该类只需要 `surfaceRestVertices` 与 `surfaceFromSimulationDispMap`；
不需要 triangles。

### 5. `src/tools/runSim/runIPCSimSetup.h/.cpp`

`IpcSimulationContext` 需要新增：

- `std::vector<std::shared_ptr<NonlinearOptimization::PotentialEnergy>> extraGeneralImplicitForceModels;`

`buildShellIpcSimulation(...)` 与 `buildVolumeIpcSimulation(...)`
都需要新增 floor config 解析逻辑：

- 读取 `use-floor`
- 解析 `floor-axis`
- 解析 `floor-height`
- 解析 `floor-kappa`
- 若启用，则创建 `EmbeddedSurfaceFloorPotentialEnergy` 并 push 到 `extraGeneralImplicitForceModels`

这里有两个细节必须固定：

1. shell path 也走同一逻辑
   - 使用 `W = I`
   - 不做 shell-special floor 分支
2. floor potential 的构造输入必须与 collision handler 完全共享同一份：
   - `surfaceRestVertices`
   - `surfaceFromSimulationDispMap`

不允许出现“IPC 用一份 surface embedding，floor 再自己重建一份”的双轨设计。

再额外钉死两个 setup-layer 规则：

3. floor 的构造点必须放在 setup 内、紧挨 `collisionHandler` 的构造点
   - 使用同一局部 `MXd surfaceRestVertices`
   - 不允许依赖 `IpcSimulationContext::surfaceRestPositions` 这种 flat `VXd` 再反向 reshape
4. `use-floor=true` 时，setup 层必须显式校验：
   - `floor-axis` 存在
   - `floor-height` 存在
   - `floor-kappa` 存在
   - 缺任一字段都直接报错
   这里不再引入“默认来源”分支逻辑

此外，若 `use-floor=false` 但用户仍写了 `floor-axis`、`floor-height` 或 `floor-kappa`，
本 phase 固定语义为：

- 不报错
- floor 不生效
- setup 层打印一条 info 级提示，说明这些字段已被忽略，因为 `use-floor=false`

### 6. `src/tools/runSim/runIPCSim.cpp`

主循环保持现有结构，只做最小改动：

- 继续保留 `collisionHandler`
- 每步在 add `collisionHandler` 之后，循环 add `extraGeneralImplicitForceModels`

日志部分需要扩展：

- shell / volume 的 IPC parameter print 中，补充：
  - `use-floor`
  - `floor-axis`
  - `floor-height`
  - `floor-kappa`

这里不新增 floor-specific summary logger；
避免因为 floor penalty 再发明第二套 solve summary。

### 7. `tests/src/core/contact/CMakeLists.txt`

新增：

- `embeddedSurfaceFloorPotentialEnergy_gtest`

不需要为 `MappedSurfacePotentialEnergy` 单独新增 fake-derived test target；
因为：

- 现有 `embeddedSurfaceIPCPotentialEnergy_gtest`
  已经覆盖 IPC 派生类的 mapped-surface 数学
- 新的 floor gtest 会覆盖 floor 派生类的 mapped-surface 数学

两者合起来足够构成 base regression gate。

### 8. `tests/src/core/contact/embeddedSurfaceFloorPotentialEnergy_gtest.cpp`

新增 floor 单测，至少覆盖：

- `W = I` 时，energy / gradient / Hessian 在 `x / y / z` 三个 axis 上都与 `testCIPCHelpers.h` 一致
- 一般稀疏 `W` 时，结果与手工 `W^T g_surf`、`W^T H_surf W` 对齐
- 所有点都在 floor 上方时，输出全零
- `computeMaxStepSize(...) == 1.0`
- 维度不匹配时抛出合适异常
- 非法 `floor-axis` 时抛出合适异常

### 9. `tests/src/tools/runIPCSim_gtest.cpp`

扩展 config builder 和 smoke test，至少新增：

- floor-enabled shell zero/one-step smoke
- floor-enabled tet 或 cubic zero/one-step smoke
- floor log 包含 `use-floor` / `floor-axis` / `floor-height` / `floor-kappa`
- `use-floor=true` 但缺 `floor-axis`、`floor-height` 或 `floor-kappa` 时，CLI 直接失败

明确不写以下 brittle test：

- 不断言“结果绝不低于 floor”
- 不断言“floor 一定触发”

因为这些都不是 soft penalty 的 contract。

### 10. `examples/ipc/*`

本 phase 的 example deliverable 固定为新增一整个 case：

- `/Users/jinceyang/Desktop/codebase/libpgo/examples/ipc/cubic/box-with-sphere/`

而不是只在已有 squash case 上顺手开一个 `use-floor` 开关。

该目录最终应包含至少：

- cubic simulation mesh：
  - `box-with-sphere-xlite.veg` 或按 repo 命名规范重命名后的对应文件
- display surface mesh：
  - `box-with-sphere.obj`
- `runIPCSim` config：
  - `box-ipc.json`
- `convertAnimation` config：
  - `anim.json`
- 完整仿真输出目录：
  - `ret-box-with-sphere-ipc/` 或文档中约定的最终输出目录名
- Alembic cache：
  - 一个 committed `.abc` 文件

example config 设计规则固定为：

- 不再出现 `external-objects`
- 明确启用 `use-floor=true`
- `floor-axis` 必须显式给出
- `floor-height` 必须按缩放后的世界坐标写清楚
- `floor-kappa` 必须显式给出
- `anim.json` 的 mesh `name` 与最终 `.abc` 文件名保持一致

README 层面的规则也固定为：

- `examples/ipc/README.md` 必须新增该 case 的介绍
- case 说明里明确写出：
  - 它来源于 `examples/cubic/box-with-sphere-xlite`
  - legacy external contact 已被 floor penalty 替代
  - 运行命令
  - `convertAnimation` 命令
  - 最终 `.abc` 文件路径

## Testing Plan

### Core Contact Tests

必须覆盖两层回归：

1. 旧的 embedded IPC wrapper 在抽基类之后行为不变
2. 新的 floor wrapper 在 identity / non-identity `W` 下都正确

建议 test matrix：

- `embeddedSurfaceIPCPotentialEnergy_gtest`
  - 原有测试全部保留
  - 如构造函数签名或 include 路径变化，仅做最小同步
- `embeddedSurfaceFloorPotentialEnergy_gtest`
  - `IdentityEmbeddingMatchesSurfaceFormula`
  - `SparseEmbeddingPullsBackGradientAndHessian`
  - `InactiveAboveFloorReturnsZero`
  - `MaxStepSizeIsUnity`
  - `RejectsWrongSimulationVectorSize`

### Tool / CLI Tests

`runIPCSim_gtest` 需要新增两类验证：

- config/setup 级：
- `use-floor=true` 能创建 floor model
- `use-floor=true` 缺 `floor-axis` 时失败
- `use-floor=true` 缺 `floor-height` 时失败
- `use-floor=true` 缺 `floor-kappa` 时失败
- 运行级：
  - shell + floor 的 one-step smoke
  - volume + floor 的 one-step smoke

如果只选一条 volume smoke，优先 cubic box-squash；
因为它正对应当前用户的主要使用场景。

### Manual Validation

建议在实现完成后手动跑至少以下 case：

- `examples/ipc/cubic/box-squash/box-ipc.json`
- `examples/ipc/tet/box-squash/box-ipc.json`
- `examples/ipc/cubic/box-with-sphere/box-ipc.json`

其中前两个现有 `box-squash` case 在本 phase 的语义固定为：

- 保持 `use-floor=false`
- 不承担新的 floor 演示职责
- manual validation 只用于确认 floor 功能没有把现有 IPC case 回归打坏
- 不要求把它们改造成 floor-enabled case

观察点固定为：

- 能跑通
- 对 `box-with-sphere`，log 中能看到 `floor-axis=y`
- 输出 `retXXXX.obj` 符合“有地板支撑”的预期趋势

对 `box-with-sphere` example 还要额外检查：

- `convertAnimation examples/ipc/cubic/box-with-sphere/anim.json`
  能成功生成 `.abc`
- `.abc` 文件路径与 `anim.json` 中的 mesh `name` 一致
- 该 example 不再依赖 `bottom.1.obj` 或其他 legacy external obstacle mesh
- `.abc` 文件大小应维持在 repo 可接受的 example 资产量级内
  - 当前目标是与现有 committed example binary 同一量级
  - 不接受明显失控的超大缓存（例如数百 MB 级别）

但 manual validation 只作为 sanity check，
不把“严格不穿透”当成验收标准。

### Example Deliverable Validation

除 unit / CLI tests 外，本 phase 还要求一条明确的 example-level 验证链：

1. 从 `examples/cubic/box-with-sphere-xlite` 提取需要的 mesh 与动画语义
2. 在 `examples/ipc/cubic/box-with-sphere/` 下生成新的 IPC config 与 animation config
3. 用 `runIPCSim` 跑完整 simulation，而不是只跑 0/1-step smoke
4. 用 `convertAnimation` 生成 `.abc`
5. 将 `.abc` 作为 example 交付资产的一部分提交

这条验证链的意义固定为：

- 证明 floor energy 不只是“测试里可用”
- 证明它可以替代至少一个真实 legacy external-contact example
- 证明 IPC 输出资产链路没有因为 analytic floor penalty 而断掉

## Rollout Order

建议按下面顺序落地，保证每一步都可单独验证：

1. 新增 `MappedSurfacePotentialEnergy`
2. 把 `EmbeddedSurfaceIPCPotentialEnergy` 迁到新 base 上，但不加 floor
3. 让原有 embedded IPC tests 全绿，确认无行为回归
4. 新增 `EmbeddedSurfaceFloorPotentialEnergy`
5. 补 floor 的 core/contact tests
6. 扩 `runIPCSimSetup` / `IpcSimulationContext`
7. 在 `runIPCSim.cpp` 里挂 `extraGeneralImplicitForceModels`
8. 补 `runIPCSim_gtest`
9. 最后再更新示例 config 和 README

这个顺序的意义是：

- 先把抽象层站稳
- 再接新功能
- 避免一边重构 IPC wrapper，一边同时怀疑 floor 数学或 CLI wiring

其中第 9 步在本 phase 中不是可选项，而必须至少覆盖
`examples/ipc/cubic/box-with-sphere` 这个端到端 example 交付。

## Risks And Controls

### Risk 1: 基类抽象过头，反而把 contact / floor 语义搅混

控制策略：

- base 只持有 mapping / pull-back
- surface-space 数学全部留给派生类
- 不在 base 中出现 `SurfaceIPCCore`、`floorHeight` 这类具体语义

### Risk 2: 把 floor 的 surface-space对角 Hessian 错写成 simulation-space 对角 Hessian

控制策略：

- 文档和测试都明确写 `H_sim = W^T H_surf W`
- 必须有非 identity `W` 的 Hessian 测试

### Risk 3: floor 意外进入 max-step 或 contact clamp 统计

控制策略：

- `collisionHandler` 和 `extraGeneralImplicitForceModels` 分开存
- floor 类不暴露 clamp getter
- floor `computeMaxStepSize()` 固定 `1.0`

### Risk 4: 默认值语义不清，导致 example / test / runtime log 三方不一致

控制策略：

- 计划中取消 `use-floor=true` 下的隐式默认值
- floor 参数在启用时必须显式给出
- test 明确验证缺字段时报错，而不是验证任何隐式补全

### Risk 5: 顺手重构 `CIPCPotentialEnergy` 导致 scope 膨胀

控制策略：

- 本 phase 明确禁止动 `CIPCPotentialEnergy`
- 若后面真要统一 legacy shell wrapper，再单独开 phase

### Risk 6: floor Hessian pull-back 数学正确但可能偏慢

风险来源：

- floor 的 surface-space Hessian 只有 active vertex 的所选 axis 对角项
- 但如果完全按统一路径做：
  - 先构造 `H_surf`
  - 再做 `W^T H_surf W`
  则每步都要付出一次额外的 sparse pull-back 成本
- 对 tet/cubic 的 barycentric `W`，这条路径数学最清楚，但不一定是最终最优实现

控制策略：

- 本 phase 明确接受“先正确、后优化”
- 在 plan 中把这视为已知 trade-off，而不是隐藏成本
- 若 profiling 证明它是热点，再后续增加：
  - rank-one accumulate
  - triplet-level fast path
  - 或 base 的可选 `accumulateSurfaceHessianPullback(...)` 扩展点

本 phase 不因为提前追求这个优化而破坏抽象边界。

## Follow-up After This Phase

`Phase 1.8` 完成后，仓库会多出一个非常有价值的扩展点：

- `MappedSurfacePotentialEnergy`

这会让后续几个方向变得更顺：

- analytic ceiling penalty
- analytic side-wall penalty
- 一般化 `MappedSurfacePlanePenaltyEnergy`
- 更丰富的 `runIPCSim` 解析障碍物场景

但这些都不是本 phase 的交付内容。

本 phase 完成后，最近的自然后续应当仍然是：

- 若要做“严格 floor 不穿透”，去设计 analytic plane barrier / external IPC
- 若只要更多 soft obstacle，继续在 mapped-surface base 上增量添加具体 penalty 类
- 当前 floor 不参与 friction；
  若未来需要“floor friction”，不应在这个 soft penalty 上硬补，
  而应沿 analytic plane barrier / external IPC 的路线设计

## Final Deliverable Shape

本计划要求最终交付同时包含：

- contact core 结构调整
- floor penalty 新类
- `runIPCSim` setup/runtime 接线
- unit tests
- CLI smoke tests
- 必要的 example / README 更新

而且这些交付必须满足同一个清晰叙事：

- `MappedSurfacePotentialEnergy` 解决公共 mapping 问题
- `EmbeddedSurfaceIPCPotentialEnergy` 继续只负责 IPC
- `EmbeddedSurfaceFloorPotentialEnergy` 只负责 floor
- `runIPCSim` 用组合方式把它们接起来

只要后续实现严格遵守这个边界，`Phase 1.8` 就能平滑推进，不会和 `phase1D / 1.5 / 2 / 3`
之间发生语义冲突。
