# IPC Self/External Contact 与 Friction 统一计划

> **For implementers:** 这份计划的目标不是继续堆叠现有 `legacy` contact handler，而是把 `shell / tet / cubic`
> 三条网格类型统一到同一套 **embedded-surface IPC** 主链路上，并通过一个新的统一入口 `runIPCSim`
> 暴露给用户；在此基础上再补 external contact、friction，
> 最后再做 broad phase / spatial hash 优化。

## 1. 摘要

目标按下面顺序推进：

1. **Phase 1**：把 `shell / tet / cubic` 的 **frictionless self-contact** 统一到同一套 IPC 框架；
2. **Phase 1.5**：补上 **inversion-free / material-feasible max step**，让 line search 同时维护 contact 可行性和材料可行性；
3. **Phase 2**：在统一后的 contact engine 上实现 **frictionless IPC external contact**；
4. **Phase 3**：在统一的 self/external IPC 接触对之上补 **统一 friction**，并同时支持 `ImplicitBackwardEuler` 与 `TRBDF2`；
5. **Phase 4**：在整条 pipeline 正确跑通后，先做 **profiling / instrumentation**，再做 **spatial hash / broad phase 优化**。

这份计划明确选择以下路线：

- 保留现有 `src/core/contact/CIPC.{h,cpp}` 中已经验证过的 **PT / EE 距离、barrier、CCD、Hessian 组装** 作为几何内核；
- 不再让 `CIPCPotentialEnergy` 直接承担 “表面顶点 DOF 就是全局 DOF” 的假设；
- 新增一个统一入口 `runIPCSim.cpp` 和对应 target，作为 `shell / tet / cubic` 的 IPC 专用 driver；
- 现有 `runSim` 与 `runShellSim` 在本计划内不改用户路径语义，继续作为 legacy/reference 入口保留；
- 新增一层 **embedded-surface adapter**，让 `shell` 走 $W = I$，`tet / cubic` 走一般的 $u_{\text{surf}} = W\, u_{\text{sim}}$；
- 在 external contact 阶段只做 **deformable body vs static/kinematic triangle-mesh obstacle**，不在本计划里实现 deformable-vs-deformable 双向 external IPC；
- friction 阶段明确做成 **integrator-aware semi-implicit friction**，不把 `v = (x^{n+1} - x^n) / h` 写死到 IPC friction 代码里。

当前仓库事实：

- `runShellSim` 已经在走 `CIPCPotentialEnergy`，但只覆盖 shell 且主要是法向 self-contact；
- `runSim` 的 tet/cubic 主路径仍在用 `TriangleMeshSelfContactHandler` / `TriangleMeshExternalContactHandler` 这条 legacy contact 线；
- 仓库当前没有一个同时支持 `shell / tet / cubic` 的统一 IPC CLI 入口；
- `PointPenetrationEnergy` 与 `PointTrianglePairCouplingEnergyWithCollision` 已有一套 friction smoothing，可复用其数值公式，但不再复用其 contact pair 来源；
- `DeformationModelEnergy::computeMaxStepSize()` 目前恒为 `1.0`，因此现有 contact-feasible line search 不等于 inversion-free line search；
- `Phase 1.5` 当前实现额外支持一个 config 开关
  `enable-material-max-step`（默认 `true`）；若显式设为 `false`，则旁路材料 max-step clamp，只保留 contact side 的缩步；
- `Phase 1.5` 当前实现也支持 config 字段 `loglevel`（默认 `info`），用于分级输出
  `materialClampCount / contactClampCount / lastMaterialAlpha / lastContactAlpha / finalAlpha`
  这些 max-step 观测信号；
- `TRBDF2` 的试探速度不满足 $v = (x^{n+1} - x^n) / h$，因此 friction 必须做 stage-local 适配。

## 2. 关键接口与类型决策

### 2.1 Contact 内核分层

本计划不直接在当前 `CIPCPotentialEnergy` 上继续堆功能，而是拆成两层：

1. **SurfaceIPCCore**（新类，非 `PotentialEnergy`）
   - 只处理 surface-space 的几何与 barrier 数学；
   - 负责 surface topology、PT/EE pair build、barrier energy/grad/hess、CCD、contact-feasible max step；
   - 输入始终是：
     - surface 当前坐标 `x_surf`
     - surface 试探位移 `dx_surf`
     - optional external surface 当前/上一时刻坐标
   - 输出固定为 surface-space 的 `energy`、`g_surf`、`H_surf` 与 `alpha_contact`；
   - 接口语义固定为：
     - `computeEnergy(x_surf)` 返回标量 `energy`
     - `computeGradient(x_surf, g_surf)` 写入长度为 `3 * num_surface_vertices` 的 surface-space 向量
     - `computeHessian(x_surf, H_surf)` 写入大小为 `(3 * num_surface_vertices) x (3 * num_surface_vertices)` 的 surface-space 稀疏矩阵
     - `computeAll(x_surf, energy, g_surf, H_surf)` 同时返回上述三者，且三者都仍然在 surface-space
     - `computeMaxStepSize(x_surf, dx_surf)` 返回 contact-feasible 的 `alpha_contact`
   - `SurfaceIPCCore` **不**返回 simulation-space 的 gradient / Hessian，也**不**知道 embedding `W`；
   - 不知道 volume DOF，也不直接参与 `runIPCSim` / `runSim` / `runShellSim` 的积分器接口。

2. **EmbeddedSurfaceIPCPotentialEnergy**（新类，继承 `PotentialEnergy`）
   - 工作在 simulation-space 的广义坐标；
   - 对当前 `shell / tet / cubic` solid 路径，这个广义坐标固定为位移
     $$
     u_{\text{sim}};
     $$
     积分器与 Newton 仍然优化 `u_{\text{sim}}`，不直接优化 absolute position；
   - 负责：
     - 先将 simulation-space 位移转成 absolute position：
       $$
       x_{\text{sim}} = x_{\text{rest}} + u_{\text{sim}}
       $$
     - 再将其投影到 contact surface：
       $$
       x_{\text{surf}} = x_{\text{surf,rest}} + W\, u_{\text{sim}}
       $$
     - 对应的试探增量固定为：
       $$
       dx_{\text{surf}} = W\, du_{\text{sim}}
       $$
     - $g_{\text{sim}} = W^\top g_{\text{surf}}$
     - $H_{\text{sim}} = W^\top H_{\text{surf}} W$
   - 设计约束固定为：
     - adapter 层负责 “位移 $\rightarrow$ absolute position” 的转换；
     - `SurfaceIPCCore` 永远只看 absolute surface positions `x_surf` 与 `dx_surf`；
     - `SurfaceIPCCore` 不允许再持有 `isInputDisp` 这一类“输入是位移还是位置”的模式开关；
   - `shell` 使用单位映射 $W = I$；
   - `tet / cubic` 使用由 surface embedding 生成的稀疏矩阵 `W`。
   - 这里的 `W` 不需要新发明生成逻辑；首期直接复用现有
     `InterpolationCoordinates::BarycentricCoordinates::generateInterpolationMatrix()`
     产出的稀疏插值矩阵。
   - 这一点对 `cubic` 不是 plan 的额外假设，而是**仓库现状**：
     当前 `runSim.cpp` 已经通过同一条 `VolumetricMesh + BarycentricCoordinates::generateInterpolationMatrix()`
     路径为 tet 与 cubic 构造 surface/sample embedding；本计划只是把这条既有事实显式纳入 unified IPC 架构。

#### Contact state 的所有权

为避免 barrier active pair 与 friction lagged state 混在同一个类里，本计划固定采用以下 ownership 规则：

- `SurfaceIPCCore` 拥有 **当前几何状态下** 的 barrier/contact 工作集：
  - 当前 `x_surf` 对应的 active PT/EE pairs；
  - 当前 pair-local geometry、barrier energy/gradient/Hessian 所需的临时数据；
  - 当前 `computeMaxStepSize(x_surf, dx_surf)` 所需的 CCD candidate / active pair 信息；
- 这些数据的生命周期只覆盖“当前一次 `x_surf` 评估”，语义上属于
  $$
  \text{ActivePairs}(x_{\text{surf}})
  $$
  这一纯几何/纯 barrier 计算结果；
- `SurfaceIPCCore` 可以在**单次调用内部**复用当前 active pair / local geometry，
  但**不允许**在调用之间持有 lagged friction state，也不负责 timestep/stage 生命周期管理。
- 缓存边界固定为：
  - `SurfaceIPCCore` 只允许做**单次调用内部**的工作区复用，例如在一次 `computeAll(x_surf, ...)`
    中复用同一份 active pair / local geometry 来计算 energy、gradient、Hessian；
  - `SurfaceIPCCore` **不允许**持有跨调用 cache，例如 `cached_x_surf`、`cached_active_pairs`
    或任何“上一次 `func/gradient/hessianDirect` 调用结果”的成员缓存；
  - 凡是要跨 `func(x)` / `gradient(x)` / `hessianDirect(x)` 调用复用的 cache，
    一律只能放在 wrapper / adapter 层，例如 `EmbeddedSurfaceIPCPotentialEnergy`
    或 legacy `CIPCPotentialEnergy` wrapper；
  - 这样 `SurfaceIPCCore` 保持近似无状态，而 Phase 4 的 wrapper-level cache 设计不会与 core 的职责边界冲突。

friction 相关的跨调用状态固定由 adapter / friction-energy 层拥有：

- `EmbeddedSurfaceIPCPotentialEnergy` 或其内部的 IPC friction energy 负责持有：
  - lagged friction pair snapshot；
  - lagged tangent frame；
  - lagged closest-point barycentric weights；
  - lagged normal force magnitude $\lambda_k^n$；
  - stage-aware external obstacle reference state（若存在 kinematic obstacle）；
- 这些数据的生命周期覆盖一个 timestep 或一个 TRBDF2 stage，
  语义上属于“上一收敛状态/当前 stage 开始时刻冻结下来的 friction reference state”；
- refresh 规则固定为：
  - `ImplicitBackwardEuler`：每个 timestep 开头，从最近一次收敛状态刷新一次；
  - `TRBDF2`：`TR` stage 开头刷新一次，`BDF2` stage 开头再刷新一次；
  - 同一 stage 的 Newton 内层中只读，不刷新。

single source of truth 规则固定为：

- 当前 active pair 的 single source of truth 始终是 `SurfaceIPCCore`；
- friction **不允许**自行再做一套 pair build；
- friction 的 lagged state 必须从 `SurfaceIPCCore` 在“最近一次收敛状态 / stage begin 状态”下导出的规范化 pair snapshot 构建；
- 这样可以确保 barrier 与 friction 使用相同的 pair identity、primitive ordering 与 closest-point 约定。

### 2.2 现有 `CIPC` 的迁移策略

- 现有 `CIPCPotentialEnergy` **不删除，也不保持为第二套独立 IPC 内核实现**；
- 它要改成一个**很薄的 compatibility wrapper**：
  - 外部 API 尽量保持不变；
  - 内部改为持有 `SurfaceIPCCore`；
  - 自己不再维护一整套独立的 PT/EE、barrier、CCD、Hessian 逻辑；
  - 这样仓库最终只有一套真正的 IPC 数学内核，即 `SurfaceIPCCore`。
- compatibility wrapper 的职责固定为：
  - 保留现有构造函数形状与旧调用方语义，尤其保留 `isInputDisp` 这一旧 API；
  - 保留 `setMesh(V, F)`、`func`、`gradient`、`hessianDirect`、`computeMaxStepSize` 这些现有入口；
  - 在 wrapper 内部先根据 `isInputDisp` 把旧输入语义转成 surface absolute position：
    - 若 `isInputDisp == true`，则
      $$
      x_{\text{surf}} = x_{\text{rest}} + u
      $$
    - 若 `isInputDisp == false`，则
      $$
      x_{\text{surf}} = x
      $$
  - 对 `computeMaxStepSize(x, dx)` 同样在 wrapper 内完成：
    - `x -> x_surf`
    - `dx -> dx_surf`
    - 然后统一委托给 `SurfaceIPCCore`；
  - 因为这条 legacy shell 路径本质上是 `W = I`，所以不需要额外的 embedding pull-back。
- 这样做的目的固定为：
  - 现有 `runShellSim` / 相关 shell 调用方在本计划内不必立刻改 API；
  - `SurfaceIPCCore` 能通过 legacy shell 路径得到真实回归验证；
  - 避免仓库长期维护两套分叉的 IPC 实现。
- `CIPCPotentialEnergy` 在本计划中的定位是：
  - **兼容壳 / reference wrapper**
  - 不是新的 unified IPC 主入口类型
  - 不承担新的 tet/cubic/external/friction 主路径接线
- 新的 unified IPC 路径一律使用 `EmbeddedSurfaceIPCPotentialEnergy`，由 `runIPCSim` 负责创建和接线。

### 2.3 入口与配置策略

- 新增 `src/tools/runSim/runIPCSim.cpp` 和对应 build target，作为 **唯一的 unified IPC CLI 入口**；
- `runIPCSim` 不支持 legacy contact，也不引入 `"contact-model"` 开关；
- `runSim` 与 `runShellSim` 保持当前行为，不在本计划内切换默认 contact 实现；
- IPC 路径的配置文件策略固定为：
  - 不修改原有 legacy `*.json` 配置；
  - 每个 case 的 IPC 配置使用独立文件，命名约定推荐为 `*-ipc.json`；
  - 不推荐在目录中只放一个语义模糊的通用 `ipc.json`；
  - `runIPCSim` 首期允许一个可选字段
    - `base-config`
    用来继承对应的 legacy case 配置；
  - 若存在 `base-config`，则解析顺序固定为：
    1. 先读取 base legacy config
    2. 再用 `*-ipc.json` 中的字段覆盖/补充
  - 这样可以最大程度避免复制整份 legacy config，同时保证 old config file 本身不被修改。
- `runIPCSim` 首期沿用 `runSim`/`runShellSim` 现有 JSON 键名：
  - `surface-mesh`
  - `tet-mesh` / `cubic-mesh`
  - `external-objects`
  - `contact-friction-coeff`
  但其语义固定为 IPC 路径；
- 对 IPC 专有参数，配置命名固定采用 `ipc-` 前缀，避免与 legacy contact 参数混淆；
- 首期至少支持：
  - `ipc-dhat`
  - `ipc-kappa`
- `ipc-dhat / ipc-kappa` 当前版本固定解释为 **统一作用于 self-contact 与 external contact 的全局 IPC 参数**；
- 本计划当前版本**不**引入
  - `ipc-self-dhat`
  - `ipc-external-dhat`
  - `ipc-self-kappa`
  - `ipc-external-kappa`
  这类分开配置；
- 若用户未显式提供 `ipc-dhat / ipc-kappa`，则 `runIPCSim` 必须使用 documented heuristic 生成默认值；
- 无论来自 config 还是 heuristic，`runIPCSim` 都必须在运行时打印最终采用的 `ipc-dhat / ipc-kappa` 及其来源。
- `runIPCSim` 的 mesh-type 判定规则固定为：
  - 若存在 `tet-mesh`，则按 tet volume simulation 处理；
  - 否则若存在 `cubic-mesh`，则按 cubic volume simulation 处理；
  - 否则若不存在 `tet-mesh` 和 `cubic-mesh`，但存在 `surface-mesh`，则按 shell simulation 处理；
  - 若三者都不存在，则直接报配置错误；
  - 不允许同时给出 `tet-mesh` 与 `cubic-mesh`；
- 因此在 shell 模式下，JSON 只提供 `surface-mesh` 就足够；在 tet/cubic 模式下，`surface-mesh` 仍然是必需项，用于 contact surface 与输出表面。
- 是否在未来让 `runIPCSim` 取代旧入口、或把 IPC 改成默认用户路径，不属于本计划范围；
- 本计划完成后的默认行为固定为：
  - `runIPCSim` 提供 unified IPC 路径；
  - `runSim` / `runShellSim` 继续保持 legacy/reference 行为；
- 默认值翻转、旧入口下线、配置迁移与样例重写，必须作为后续单独的 cleanup / RFC / migration 议题处理，不在本计划内顺带完成。

### 2.4 Friction 的 integrator 适配接口

新增轻量接口（名称可微调，但语义固定）：

```cpp
struct TrialVelocityModel
{
  double scale;              // v_trial = scale * (u - u_ref)
  EigenSupport::VXd x_ref;   // simulation-space reference generalized coordinate
};

class TrialVelocityAwarePotentialEnergy
{
public:
  virtual ~TrialVelocityAwarePotentialEnergy() = default;
  virtual void setTrialVelocityModel(const TrialVelocityModel &model) = 0;
};

enum class ContactSolveStage
{
  BE_STEP,
  TR_STAGE,
  BDF2_STAGE
};

struct ContactStageContext
{
  ContactSolveStage stage;
  double t_start;
  double t_end;
  double dt_stage;
};

class StageAwareContactPotentialEnergy
{
public:
  virtual ~StageAwareContactPotentialEnergy() = default;
  virtual void setContactStageContext(const ContactStageContext &ctx) = 0;
};
```

设计选择固定为：

- friction 在 v1 只支持 **global trial velocity 对当前优化变量是 scalar * identity 的仿射映射**；
- stage-aware contact / friction 还必须接收一个 `ContactStageContext`，
  用来显式标识当前求解属于：
  - `BE_STEP`
  - `TR_STAGE`
  - `BDF2_STAGE`
  以及对应的 `t_start / t_end / dt_stage`；
- `TrialVelocityModel` 的 `x_ref` 语义固定为 **simulation-space reference generalized coordinate**；
- 对当前 `shell / tet / cubic` solid 路径，这个 generalized coordinate 就是 displacement，因此这里的
  `x_ref` 应理解为 `u_ref_sim`，而不是 absolute position；
- `TrialVelocityModel` 本身始终工作在 simulation space；它不能直接传入 friction core；
- adapter 必须先将其转译为 surface-space 的 reference position：
  $$
  x_{\text{ref,surf}} = x_{\text{surf,rest}} + W\, u_{\text{ref,sim}}
  $$
  并配合
  $$
  x_{\text{surf}} = x_{\text{surf,rest}} + W\, u_{\text{sim}}
  $$
  得到
  $$
  v_{\text{trial,surf}}
    = \text{scale}\,(x_{\text{surf}} - x_{\text{ref,surf}})
    = \text{scale}\,W\,(u_{\text{sim}} - u_{\text{ref,sim}});
  $$
- friction 内核只允许消费 surface-space 的 `x_surf`、`x_ref_surf` 与 `scale`，
  **不允许**直接感知 embedding `W`；
- external obstacle 若是 kinematic，其 reference velocity 也必须由 adapter 按 `ContactStageContext`
  转成 **与当前 stage 对齐** 的 surface-space obstacle velocity；friction 内核不直接采样 obstacle trajectory；
- 这足以覆盖：
  - `ImplicitBackwardEuler`: $\text{scale} = 1/h$，$x_{\text{ref}} = x^n$
  - `TRBDF2` TR stage: $\text{scale} = \alpha$，$x_{\text{ref}} = q + q_{\text{vel}} / \alpha$
  - `TRBDF2` BDF2 stage: $\text{scale} = \beta_7$，$x_{\text{ref}} = q - (\beta_5\, q + \beta_6\, q_y) / \beta_7$
- 不在本计划里设计通用 $v = Bx + c$ 的稀疏线性算子接口。

### 2.5 测试组织

在现有 `tests/src/core/` 下新增 / 扩展测试目录：

- `tests/src/core/contact/CMakeLists.txt`
- `tests/src/core/contact/surfaceIPCCore_gtest.cpp`
- `tests/src/core/contact/embeddedSurfaceIPCPotentialEnergy_gtest.cpp`
- `tests/src/core/solidDeformationModel/CMakeLists.txt`
- `tests/src/core/solidDeformationModel/deformationModelEnergyMaxStep_gtest.cpp`
- `tests/src/core/contact/ipcExternalContact_gtest.cpp`
- `tests/src/core/contact/ipcFriction_gtest.cpp`

并扩展：

- `tests/src/tools/runSim_gtest.cpp`
- 新增 `runIPCSim` CLI smoke test，覆盖 shell/tet/cubic 三类输入。

## 3. 分阶段实现

## 3.1 Phase 1：统一 frictionless self-contact IPC

### 目标

把 `shell / tet / cubic` 的 self-contact 统一到同一套 frictionless IPC 主链路，并首先在新的 `runIPCSim` 入口上跑通；旧 `runSim` / `runShellSim` 仅作为 reference 与回归对照保留。

### 实现决策

#### A. 抽取 surface-space IPC core

- 从现有 `src/core/contact/CIPC.{h,cpp}` 中抽出不依赖全局 DOF 维度的部分，形成 `SurfaceIPCCore`：
  - surface topology 初始化：三角形、唯一边、邻接、面积/长度权重；
  - pair build：PT / EE；
  - barrier 数学：`b / dbds / d2bds2`；
  - local geometry：PT / EE 距离、gradient、Hessian；
  - contact-feasible `computeMaxStepSize()`；
  - `computeEnergy / computeGradient / computeHessian / computeAll`。
- 保持以下实现不重写，只迁移位置与接口：
  - `CIPC_autogen.h` / `CIPC_autogen_ll.h`
  - PT / EE 分类
  - additive CCD
  - PSD projection
- 在本批次同时落地一套 **仓库级、可测试的 profiling 基础设施与 scoped timer 接口**：
  - 基础设施在本批次就提供真实可用的 `ScopedProfileSection(std::string_view)` 与最小统计能力；
  - runtime 默认可关闭，避免对当前数值路径造成行为扰动；
  - 其目的除了给 Phase 4 的 instrumentation 留稳定挂点，也是在 Phase 1 起就统一 section 命名与插桩位置，避免后续返工。

#### B. 新增 simulation-space adapter

- 新增 `EmbeddedSurfaceIPCPotentialEnergy`，内部持有：
  - `SurfaceIPCCore core`
  - `surface_rest_positions`
  - `surface_topology`
  - `simulation_rest_positions`
  - sparse embedding `W`
  - `bool is_input_disp`
- `func / gradient / hessianDirect / computeMaxStepSize` 全部先映射到 surface，再由 `core` 处理，再映回 simulation space。
- `shell` 的实现固定走 `W = I`，不单独保留一套 shell-only 数学分支。
- 本批次就在 `EmbeddedSurfaceIPCPotentialEnergy` 的主入口函数中接入同一套 profiling scope：
  - `func`
  - `gradient`
  - `hessianDirect`
  - `computeMaxStepSize`
- runtime 默认可关闭；Phase 1-3 不要求所有路径都输出 profiling 汇总，但不再依赖 no-op 占位符。

#### C. 迁移 shell 路径

- 在 `runIPCSim.cpp` 中实现 shell 模式：
  - 输入只有 `surface-mesh`
  - 构造 shell `SimulationMesh`
  - 生成 `W = I`
  - 创建 `EmbeddedSurfaceIPCPotentialEnergy`
- `runShellSim.cpp` 本阶段不改，只保留为 reference。

#### D. 迁移 tet / cubic self-contact 路径

- `runIPCSim.cpp` 在 tet/cubic 模式下：
  - 从 `surfaceMesh + BarycentricCoordinates::generateInterpolationMatrix()` 获取 `W`
  - 创建 `EmbeddedSurfaceIPCPotentialEnergy`
  - 将其作为 general implicit model 加入时间积分器
- `BarycentricCoordinates::generateInterpolationMatrix()` 是仓库现成能力，不属于 Phase 1 的新增基础设施：
  - 声明在 `src/core/interpolationCoordinates/barycentricCoordinates.h`
  - 实现在 `src/core/interpolationCoordinates/barycentricCoordinates.cpp`
  - `runSim.cpp` 和 `tests/src/tools/runSim_gtest.cpp` 已经在使用它生成 surface embedding `W`
- `runSim.cpp` 仍保持 legacy self-contact 路径，不在本阶段删除或改语义。
- `cubic` 在本阶段不能只被当作“和 tet 类似、应该也能工作”的隐含假设；
  Phase 1 的实现必须显式验证 cubic 的
  $$
  u_{\text{surf}} = W\,u_{\text{sim}}
  $$
  与现有 cubic surface/sample 运动语义一致。

#### E. cubic 路径与 contact embedding 一致性

- `tet` 与 `cubic` 必须共用完全同一套 self-contact 接口；
- 不允许再出现 “tet 用 volume DOF，cubic 用单独 sample mapping 分支” 的特殊情况；
- `cubic` contact 只允许通过 surface embedding `W` 进入 IPC，不为 cubic 单独写 contact primitive。
- 批次 B 的第一个可合入版本必须包含一个 **cubic consistency test**，作为 tet/cubic unified path 的准入条件：
  - 对一个 cubic 小例子，构造若干 `u_sim`；
  - 验证由 `W u_sim` 得到的 surface displacement
    与现有 cubic surface/sample 路径计算出的 surface/sample motion 一致；
  - 只有该测试通过，才允许继续在 cubic 路径上接 self-contact IPC。

### Phase 1 验证

- 单元测试
  - `SurfaceIPCCore` 的 energy/grad/hessian 对 shell-like surface 做 FD 校验；
  - `EmbeddedSurfaceIPCPotentialEnergy` 在 `W = I` 时与现有 `CIPCPotentialEnergy` 的数值结果对齐；
  - 对 tet/cubic 构造小例子，验证 `g_sim` 与 `W^T g_surf` 一致，`H_sim` 与 `W^T H_surf W` 一致；
  - 对 cubic 单独增加 consistency test，验证 `W u_sim` 得到的 surface displacement
    与仓库当前 cubic surface/sample motion 语义一致；这项测试是 cubic 路径进入 unified IPC 的准入条件。
- 工具/样例验证
  - `runIPCSim` 的 shell/tet/cubic 样例都能完成单步或极少步 smoke test；
  - 除单步 smoke 外，再增加一个**最小多步非接触 sanity run**：
    - 例如 cubic 无 external、初始时无 active contact 的小例子；
    - 连续推进约 `10` 步；
    - 目的不是验证长期稳定性，而是尽早过滤 gradient 符号反、Hessian 行列装配错位、局部 PSD projection 遗漏等 assembly 级错误；
  - 当存在 active contact pairs 时，contact energy 非零，且 `computeMaxStepSize()` 的 contact half 会对搜索步长产生约束；
  - self-contact pair build、barrier、CCD、assembly 主链路全部接通，不要求长时间稳定推进。
- 回归检查
  - `runShellSim` 与 `runSim` 旧样例行为不变；
  - `tests/src/tools/runSim_gtest.cpp` 中现有 preprocessing / embedding 检查保持通过。

**Phase 1 明确不作为验收项的内容：**

- 不要求多步稳定推进；
- 不要求不翻单元；
- 不要求长时间动画结果可用；
- 不要求 external contact 与 friction 已可用。

## 3.2 Phase 1.5：补 volume inversion-free max step

### 目标

让现有 line search 同时满足：

- $\alpha_{\text{contact}}$：contact-feasible
- $\alpha_{\text{material}}$：volume inversion-free / material-feasible

最终统一为：

$$
\alpha_{\max} = \min(\alpha_{\text{contact}},\, \alpha_{\text{material}})
$$

### 实现决策

#### A. 接入点固定在 `DeformationModelEnergy::computeMaxStepSize`

- 不把 inversion 检查塞进 contact 类；
- 在 `src/core/solidDeformationModel/deformationModelEnergy.h/.cpp` 中实现真正的 `computeMaxStepSize()`；
- 同时在 `DeformationModelEnergy` 上保留一个很小的 runtime toggle，
  供 `runSim` / `runIPCSim` 从 config 显式开关材料 max-step；
- 利用已有聚合逻辑：
  - `ImplicitBackwardEulerEnergy::computeMaxStepSize()`
  - `TRBDF2TimeIntegratorEnergy::computeMaxStepSize()`
  自动取所有 implicit model 的最小步长。
- 但 implementation ownership 不再把所有 helper 堆进 `deformationModelEnergy.cpp`：
  - `pgoLogging` 持有统一 warning logger
  - `basicAlgorithms` 持有 polynomial/root-finding helper
  - `solidDeformationModel/materialMaxStepPolynomialUtils.*`
    持有 `det(A + alpha B) - eps` 的系数构造
  - concrete deformation model 持有单 element 的 local feasibility
  - `DeformationModelAssembler` 持有 element gather 与全局 `min`
  - `DeformationModelEnergy` 只保留 absolute-position 语义与 clamp 计数

#### B. 统一的 per-element 可行性判定

本 phase 只对 volume mesh 实现标量可行性函数 $\phi_{\text{ele}}(\alpha)$，要求 $\phi_{\text{ele}}(\alpha) > \epsilon$：

- `TET`
  - $\phi(\alpha) = \det(D_s(\alpha))$；
  - $D_s$ 的构造与 `TetMeshDeformationModel::prepareData()` 一致；
  - $\epsilon_{\det}$ 取 $10^{-8} \cdot |\det(D_m)|$ 的正比例阈值。

- `CUBIC`
  - 对每个 integration point 单独检查 $\det(F_{\text{ref},q}(\alpha))$；
  - $F_{\text{ref},q}$ 的定义与 `CubicMeshDeformationModel::prepareData()` 一致；
  - $\alpha_{\text{material}}$ 取所有单元所有积分点的最小可行步长。

#### C. 统一 root-find 策略

- 不为 tet/cubic 分别写解析根公式；
- 统一采用 “**先做 cubic polynomial root isolation，再对已 bracket 的最早根区间做保守 `bisection` refine**” 的根查找；
- 要求：
  - $\phi(0)$ 必须大于阈值，否则返回恢复性的极小正数，并以 `WARN` 记录非法初值；
  - 每次返回值再乘 $0.99$ safety factor；
  - 结果下限截断为 $10^{-12}$。

三次多项式系数的来源在本计划里也固定钉死：

- 对任一 `tet` element 或 `cubic` integration point，先把待检查矩阵写成
  \[
  M(\alpha)=A+\alpha B
  \]
  其中 `A` 来自当前状态下的 `D_s(0)` / `F_{\text{ref},q}(0)`，`B` 来自沿 `dx` 的线性增量；
- 系数固定按 `det` 的列多线性展开直接计算，不允许用 sampling + Vandermonde 拟合：
  - 若
    \[
    A=[a_0,\ a_1,\ a_2],\quad B=[b_0,\ b_1,\ b_2]
    \]
  - 则
    \[
    g(\alpha)=\det(A+\alpha B)-\epsilon
    = c_0 + c_1 \alpha + c_2 \alpha^2 + c_3 \alpha^3
    \]
  - 其中
    \[
    c_0=\det(a_0,a_1,a_2)-\epsilon,\quad
    c_1=\det(b_0,a_1,a_2)+\det(a_0,b_1,a_2)+\det(a_0,a_1,b_2)
    \]
    \[
    c_2=\det(b_0,b_1,a_2)+\det(b_0,a_1,b_2)+\det(a_0,b_1,b_2),\quad
    c_3=\det(b_0,b_1,b_2)
    \]
- `tet/cubic` 的差别只在 `A/B` 的构造来源，不在多项式求系数的方法。

这些数值参数不是随意拍脑袋选的，必须按下面理由固定：

- root isolation：
  - 先求三次多项式导数的实根，再切单调区间；
  - 只在已确认的 root bracket 上做 refine；
  - 这是为了避免把“整段 `[0,1]` 可行域单调”当作默认前提。
- `bisection` refine：
  - 首版刻意选择 `bisection`，不是因为它最快，而是因为实现最直接、最稳；
  - 若后续 profiling 证明它是瓶颈，再替换成 `Brent` 或更强的 bracketed refine；
  - 但 `Phase 1.5` 当前版本不把这种替换作为前置条件。
- named constants：
  - `0.99`、`1e-12`、`1e-8` 在实现中应收敛为具名常量，而不是裸字面量；
  - 其中 `0.99` 与 contact side 的 safety/slackness 语义是刻意对齐的，表示“找到边界后退一点，保持 strict interior”。
- `0.99` safety factor：
  - 用于保持 strict interior；
  - 即使 root-find 已找到“接近翻转边界”的最大可行步长，也不直接踩在 $\phi(\alpha)=0$ 或 $\phi(\alpha)=\epsilon$ 的边界上；
  - 这与当前 `CIPC` 在 CCD 后再留 `slackness` 的思路一致。
- 退化分支处理：
  - 若 `g(0) <= 0`，直接走非法初值恢复路径，不再继续 root isolation；
  - 若 `deg(g) == 0` 且常数项始终大于 `0`，直接返回 `1.0`；
  - 若 `deg(g) == 1`，直接检查唯一线性根是否落在 `(0,1]`；
  - 若 `g'` 在 `(0,1)` 无实根，则把整段 `[0,1]` 当作单调区间；
  - 若扫描完整段后始终 `g(alpha) > 0`，直接返回 `1.0`。
  - 这里的 `deg(g)` 主要是语义说明，不要求实现里真的做显式浮点阶数分派；
  - 实现可以统一走三次通用管线：求 `g'` 的至多两个实根，若最高次系数恰为 `0` 则自然退化，再按单调子区间扫描；
  - 不要求另设 `|c_3| < tol`、`|c_2| < tol` 之类 degree 判定阈值。
- `10^{-8}` 相对阈值：
  - 不是绝对面积/体积阈值，而是相对于 rest 几何量的正性阈值；
  - `tet` 用 $10^{-8} \cdot |\det(D_m)|$，
    `cubic` 用相同量级的相对 Jacobian 正性阈值；
  - 这样做是为了避免“数值上刚好非负、但实际上已经进入奇异区附近”的情况。
- `10^{-12}` 步长下限：
  - 保持与当前 `CIPC::computeMaxStepSize()` 返回风格一致；
  - 避免返回严格 `0` 导致求解器上层逻辑退化成除零、无限循环或不可恢复的“完全不前进”状态。
- 非法初值观测性：
  - 当 $\phi(0) \le \epsilon$ 时，这不是普通的小步长收缩，而是“当前状态已非法”的恢复路径；
  - 实现必须触发 `WARN`，至少带：
    - mesh type
    - element id
    - 当前 $\phi(0)$
    - 阈值 $\epsilon$
  - 非法初值 warning 固定复用 `src/core/pgoLogging/pgoLogging.h/.cpp`
    的统一 logger；
  - 不做 warning 去重；
  - 不允许静默吞掉这类状态。
- material clamp 可观测性：
  - “至少出现一次 `alpha_material < 1.0`” 不能退化成 grep 任意日志；
  - 实现应维护一个可测试的材料缩步计数通道，例如
    `DeformationModelEnergy` 内部的 `materialClampCount_`；
  - 该计数建议声明为 `mutable std::atomic<int64_t>`，避免与 `const computeMaxStepSize()` 和潜在并发调用冲突；
  - Stage 3 优先验证这个计数或其结构化 summary，而不是 grep stdout。

#### D. repo-truth 落地切分

- 主实现入口固定放在
  `src/core/solidDeformationModel/deformationModelEnergy.h/.cpp`：
  - 保持 public API 仍只有
    `computeMaxStepSize(x, dx)`；
  - 不新增新的 public 求解器接口，不把 material-feasible 逻辑扩散到 integrator 层；
  - 例外：允许新增只读的 test-facing getter，例如
    `int64_t getMaterialClampCount() const`，
    仅用于观测材料 clamp 是否触发；此类 getter 不构成长期算法 API；
  - 但 helper ownership 固定分层为：
    - `DeformationModelEnergy`
      只负责从 generalized coordinate 组装 absolute positions，并转发到 assembler；
    - `DeformationModelAssembler`
      负责 gather local `x / dx`、遍历 element、取最小值并统一打 warning；
    - `DeformationModel`
      新增局部 element 级接口
      `computeLocalMaxStepSize(const double *x_local, const double *dx_local) const`；
    - `TetMeshDeformationModel` / `CubicMeshDeformationModel`
      各自负责单 element / integration-point 的 local feasibility；
    - `KoiterDeformationModel`
      显式保持 shell defer，即返回 `1.0`。
- `ImplicitBackwardEulerEnergy::computeMaxStepSize()` 与
  `TRBDF2TimeIntegratorEnergy::computeMaxStepSize()` 当前已经会对所有 implicit model 取最小值；
  Phase 1.5 不改它们的聚合逻辑，只让 `DeformationModelEnergy` 不再恒返回 `1.0`。
- `runSim` / `runIPCSim` 当前都接受 `enable-material-max-step`，
  默认 `true`，`false` 时只关闭材料 clamp，不影响 contact max step 与其它 implicit model 的最小值聚合。
- mesh-type 分发固定直接依据
  `forceModelAssembler->getDeformationModelManager()->getMesh()->getElementType()`：
  - `SimulationMeshType::TET`
    走 tet determinant 判据；
  - `SimulationMeshType::CUBIC`
    走 cubic per-integration-point Jacobian 判据；
  - `SimulationMeshType::TRIANGLE` / `SHELL`
    在本 phase 中显式保持 `1.0`，不纳入 volume inversion-free 约束；
  - `SimulationMeshType::EDGE_QUAD`
    在本 phase 中也显式保持 `1.0`，并通过 negative test 锁住；
  - 其他 mesh type 在本 phase 不扩展新行为，保守返回 `1.0` 并保持现状。
- `DeformationModelEnergy` 评估材料可行性时，当前 absolute positions 统一按现有 repo 语义构造：
  - 若 `restPosition.size() > 0`，则
    $$
    x_{\text{abs}} = x_{\text{rest}} + u;
    $$
  - 否则直接视 `x` 为 absolute positions；
  - 试探更新固定为
    $$
    x_{\text{trial}}(\alpha)=x_{\text{abs}}+\alpha\,dx.
    $$

#### E. tet / cubic 的具体实现策略

- `shell`
  - 本 phase 明确**不**把 `SimulationMeshType::TRIANGLE` / `SHELL` 纳入 inversion-free 约束；
  - 原因是单 triangle 在 3D 中没有与 `tet/cubic` 同等明确的 orientation-preserving 符号量；
  - 因此 shell 在 `Phase 1.5` 中继续只受 contact max step 与上层 line-search finiteness check 约束；
  - 若后续需要 shell 数值保护，应单列 follow-up，例如：
    - `shell anti-collapse`
    - `shell finite-energy safeguard`
- `tet`
  - 不在 `DeformationModelEnergy` 里重复维护一套 tet 几何缓存；
  - 直接复用现有 public helper：
    - `TetMeshDeformationModel::computeDs(...)`
    - rest element 顶点坐标
  - 对 trial positions 组装
    $$
    D_s(\alpha)
    $$
    后计算
    $$
    \phi(\alpha)=\det(D_s(\alpha));
    $$
  - rest 阈值固定由同一 element 的 rest shape 生成，不依赖外部 config。
- `cubic`
  - 不在 `DeformationModelEnergy` 里手抄一份高斯点形函数导数与
    `restDmInv`；
  - 直接通过 `dynamic_cast<const CubicMeshDeformationModel *>`
    复用现有 public helper
    `computeF(local_x, materialLocationID, F)`；
  - 其中 `local_x` 的布局固定是 node-major：
    `x0, y0, z0, x1, y1, z1, ..., x7, y7, z7`；
  - 对每个 element 的每个 integration point
    `materialLocationID = 0..7`
    计算
    $$
    F_{\text{ref},q}(\alpha)
    $$
    并以
    $$
    \phi_q(\alpha)=\det(F_{\text{ref},q}(\alpha))
    $$
    判定正性；
  - element 的可行步长取所有 integration point 的最小值。

#### F. 实现顺序

1. 先补基础 ownership：
   - `pgoLogging`：统一 `WARN` 输出
   - `basicAlgorithms`：`CubicPolynomial`、root isolation、`bisection` refine
   - `solidDeformationModel/materialMaxStepPolynomialUtils.*`：
     `det(A + \alpha B)` cubic polynomial 构造与共享 feasible-alpha helper
2. 先接 `TET` 分支：
   - 几何最简单；
   - 也最适合先验证 integrator 聚合是否已正确吃到 material max step。
   - Stage 0/首个可 review 切片至少要带一个最小 `tet` case，不接受纯 skeleton 提交；
   - 但 Stage 0 只要求单 `tet` element 路径能返回真实材料 max step，不要求同批完成多 element 聚合与完整测试矩阵。
3. 再接 `CUBIC` 分支：
   - 复用 `CubicMeshDeformationModel::computeF(...)`；
   - 补 per-integration-point 最小值聚合。
4. `SimulationMeshType::TRIANGLE` / `SHELL` 在本 phase 显式保持 `1.0`，不纳入材料可行性判据。
   `SimulationMeshType::EDGE_QUAD` 也显式保持 `1.0`。
5. volume mesh 分支跑通后，再做 `runIPCSim` 多步验证，而不是在每加一个分支后就开始追完整 IPC 样例。

#### G. 测试落点与验收细化

- 新增
  `tests/src/core/solidDeformationModel/deformationModelEnergyMaxStep_gtest.cpp`
  与对应 target，覆盖：
  - tet 单单元 determinant 变号前缩步；
  - 多 tet element 时，全局结果等于最早失正 element 的步长；
  - cubic 单 hex 在任一 integration point 失正前缩步；
  - 多 cubic element / integration point 时，全局结果等于最早失正位置的步长；
  - `dx = 0` 或明显可行步时返回 `1.0`；
  - 非法初值
    $\phi(0)\le\epsilon$
    时返回受 clamp 的极小正数而非 `0`，且触发 `WARN`。
  - `SimulationMeshType::EDGE_QUAD` 在本 phase 中显式保持 `1.0`。
- 测试 fixture 尽量直接写死，减少 review 时的二义性：
  - tet canonical 例子：
    - rest tet 顶点为 `(0,0,0)`, `(1,0,0)`, `(0,1,0)`, `(0,0,1)`；
    - 仅移动第四个顶点，`dx_3 = (0,0,-2)`；
    - `deg == 0` fixture 为对 4 个顶点统一施加 `dx = (1,2,3)` 的纯刚体平移，预期返回 `1.0`；
  - cubic canonical 例子：
    - rest hex 为 unit cube；
    - bottom face 固定，top face `v4..v7` 统一施加 `(0,0,-2)`；
    - `local_x` 仍按 node-major 传给 `computeF(...)`。
- 同时增加最小 integrator 聚合 regression，覆盖：
  - `ImplicitBackwardEulerEnergy::computeMaxStepSize()` 返回 `min(material, other)`；
  - `TRBDF2TimeIntegratorEnergy::computeMaxStepSize()` 返回 `min(material, other)`。
- 这些单测首期不强依赖完整 `runIPCSim`；
  优先直接构造最小 `SimulationMesh + DeformationModelManager + DeformationModelAssembler + DeformationModelEnergy`
  链路，保证失败时定位在材料可行性本身，而不是 IPC/contact 接线。
- 在 `tests/src/core/solidDeformationModel/CMakeLists.txt` 中把
  `deformationModelEnergyMaxStep_gtest`
  纳入默认构建。
- `runIPCSim` / 工具级验证放在单测通过之后，首期只要求：
  - 一个 tet 小例子在 active contact 缩步后仍保持正体积；
  - 一个 cubic 小例子在无 external contact 时可稳定推进若干步。
  - smoke config 固定使用：
    - `examples/ipc/tet/box-hang/box-ipc.json`
    - `examples/ipc/cubic/box-hang/box-ipc.json`

#### H. 验证命令草案

- 单测 target 草案：
  ```bash
  cmake --build --preset base_no_mkl_debug --target deformationModelEnergyMaxStep_gtest
  ctest --test-dir build/base_no_mkl_debug --output-on-failure -R "DeformationModelEnergyMaxStep"
  ```
- 集成验证草案：
  ```bash
  ctest --test-dir build/base_no_mkl_debug --output-on-failure -R "DeformationModelEnergyMaxStep|RunIPCSim|RunSim"
  ```
- 若实现过程中发现 `runIPCSim` test target 尚未落地，Phase 1.5 仍以
  `deformationModelEnergyMaxStep_gtest`
  加现有相关 contact regression 为主验收，
  不把“新增 CLI gtest 尚未存在”误判成 Phase 1.5 的算法 blocker。

### Phase 1.5 验证

- tet：使用 canonical 单 tet 例子
  `(0,0,0), (1,0,0), (0,1,0), (0,0,1)` 加 `dx_3 = (0,0,-2)`，
  验证 $\det(D_s)$ 在更新后保持正；
- cubic：使用 canonical unit-cube 单 hex 例子，bottom face 固定、top face `dz = -2`，
  验证所有积分点 $\det(F_{\text{ref},q})$ 为正；
- 集成验证：
  - `runIPCSim` 的
    `examples/ipc/tet/box-hang/box-ipc.json` 与
    `examples/ipc/cubic/box-hang/box-ipc.json`
    可以稳定推进多步；
  - 在有接触时不穿透、在无接触或接触缩步后不翻单元；
  - 现有 `CIPC` contact max step 与新的 material max step 同时生效；
  - 并能通过 `materialClampCount_ > 0` 或等价结构化信号证明至少一次材料缩步发生。

## 3.3 Phase 2：frictionless IPC external contact

### 目标

在 Phase 1 的统一 contact kernel 上实现 external IPC，先覆盖 **deformable body vs static/kinematic triangle-mesh obstacle**。

### 本阶段明确范围

只做：

- 动态 `shell/tet/cubic` deformable body
- 对 static / kinematic triangle-mesh obstacles 的 external contact

不做：

- deformable-vs-deformable 双向 external IPC
- mixed deformable external contact：
  - shell-vs-tet
  - shell-vs-cubic
  - tet-vs-cubic
  - shell-vs-shell
  - tet-vs-tet
  - cubic-vs-cubic
- 刚体旋转/速度求解回写
- friction

这里的 external IPC 在本计划中固定解释为：

- **一个** 动态 deformable body（shell 或 tet 或 cubic）
- 对若干 static / kinematic triangle-mesh obstacles 的 contact

不把“两个都参与求解的 deformable object 之间的接触”归类为本计划的 external IPC。

### 实现决策

#### A. external surface 表示

在 `EmbeddedSurfaceIPCPotentialEnergy` 中增加 obstacle surface 数据；为了让 Phase 3 friction
的 lagged pair identity 在多 obstacle / 多 timestep 下稳定，Phase 2 就把数据结构固定下来，
而不是只在文字上罗列字段。

**A.1 ObstacleSurface 数据契约**

在 `cipc/external/` 下新增 `ObstacleSurface`（owning type）作为单个 kinematic obstacle 的
表示，固定持有：

- `object_id`：`int32_t`，由 `EmbeddedSurfaceIPCPotentialEnergy` 在注册时分配，全局唯一，
  作为 friction lagged state 的稳定 key 之一；
- `rest_positions`：`VXd`，size = `3 * num_obstacle_vertices`；
- `current_positions`：`VXd`，当前 timestep / stage 末 endpoint 的 obstacle 顶点位置；
- `previous_positions`：`VXd`，当前 timestep / stage 起 endpoint 的 obstacle 顶点位置；
- `triangles`：`MXi`，per-object topology，行存三角形顶点索引，索引为 local（0-based，仅
  对本 obstacle 有效），不进入 dynamic body 的顶点编号空间；
- `unique_edges`：`MXi`，由 `triangles` 派生，行存边端点 local index；
- `TrajectorySampler`：函数对象 `std::function<void(double t, RefVecXd out)>`，由
  caller（`runIPCSim`）提供，输入绝对时间 `t`，输出该时刻全部顶点位置；
- `update(t_start, t_end)`：成员函数，调用 `TrajectorySampler` 写入 `previous_positions`
  与 `current_positions`，**只**在 stage 开头被调用，不在 Newton 内层调用。

注册接口固定为：

```
int32_t EmbeddedSurfaceIPCPotentialEnergy::addObstacleSurface(
    std::shared_ptr<ObstacleSurface> obs);
```

返回值即 `object_id`，存到 `obs->object_id` 内。`removeObstacleSurface(int32_t)` 留作后期
扩展，Phase 2 不实现。

**A.2 multi-obstacle pair identity**

所有 external active pair 在 `EmbeddedSurfaceIPCPotentialEnergy` 内统一持有以下身份：

- `pair_type`：`PT` / `EE`；
- `owner_type`：`self` / `external`；
- `dynamic_side_indices`：dynamic surface 顶点 local index（surface 编号空间）；
- `obstacle_object_id`：external 时必填，self 时为哨兵 `-1`；
- `obstacle_side_indices`：external 时为该 obstacle 内 local index，self 时为空。

Phase 3 lagged state 以 `(pair_type, owner_type, dynamic_side_indices, obstacle_object_id,
obstacle_side_indices)` 的 5 元组作为 stable key；任何只用顶点索引而不带 `object_id` 的
方案都会在多 obstacle 下退化为别名碰撞，Phase 2 显式禁止。

**A.3 runIPCSim 适配层**

`runIPCSim` 的 external object 输入首期沿用当前 `runSim` / `runShellSim` 的 triangle
mesh + 平移轨迹形式；适配层负责：

- 把每个 external object 包成一个 `ObstacleSurface`；
- 用平移轨迹构造 `TrajectorySampler`（首期 `x(t) = x_rest + v * (t - t0)`）；
- 在每个 stage 开头调用 `obs->update(t_start, t_end)`。

stage-aware 调用顺序在 Phase 2 只对 BE 实现，TRBDF2 见 Phase 3。

#### B. external candidate pair 集合

external contact 使用三类 pair：

- dynamic vertex vs obstacle triangle（记 `PT_dyn_obs`）
- obstacle vertex vs dynamic triangle（记 `PT_obs_dyn`）
- dynamic edge vs obstacle edge（记 `EE_dyn_obs`）

不引入 “obstacle edge vs dynamic edge 的另一份重复集合”，统一按无序 edge-edge pair 表达。

**canonical ordering（Phase 2 固定，Phase 3 friction pair identity 依赖此约定）**：

每个 external pair 的 4 点 local configuration `(p0, p1, p2, p3)` 按 pair type 固定排布：

- `PT_dyn_obs`：`p0 = dynamic vertex`，`(p1, p2, p3) = obstacle triangle vertices`，
  triangle 顶点顺序与 `ObstacleSurface::triangles` 行内顺序一致；
- `PT_obs_dyn`：`p0 = obstacle vertex`，`(p1, p2, p3) = dynamic triangle vertices`，
  dynamic triangle 顶点顺序与 dynamic surface 拓扑行内顺序一致；
- `EE_dyn_obs`：`(p0, p1) = dynamic edge endpoints`（按 dynamic surface
  `unique_edges` 行内顺序），`(p2, p3) = obstacle edge endpoints`（按
  `ObstacleSurface::unique_edges` 行内顺序）。

每个 pair 都额外存 `(dynamic_local_slots, obstacle_local_slots)`，即 `{0..3}` 在 4 点
local configuration 中的分配集合，比如 `PT_dyn_obs` 是 `({0}, {1, 2, 3})`，
`EE_dyn_obs` 是 `({0, 1}, {2, 3})`。这两个 slot 集合在 Phase 2 固定，Phase 3 直接复用。

#### C. 梯度 / Hessian 装配策略

- 对 obstacle 侧无 DOF 的 pair：
  - 几何距离仍按 12-DOF / 4-point local configuration 计算；
  - 但全局装配时只回传 dynamic side 的导数与 Hessian 块；
  - obstacle side 视为常量，不在全局矩阵中分配列/行。

**12-DOF → dynamic-only block 的提取规则**（Phase 2 固定）：

- 设 4 点 local configuration 的 local DOF 序列为 `(p0_x, p0_y, p0_z, p1_x, ..., p3_z)`，
  共 12 维；
- `dynamic_dof_slots = {3*s + d | s ∈ dynamic_local_slots, d ∈ {0, 1, 2}}`，
  其顺序固定为 `s` 升序、`d` 升序；
- 全局梯度只装配 `g_local[dynamic_dof_slots]`，按 dynamic surface 顶点 global index 散
  布到 surface gradient；
- 全局 Hessian 只装配 `H_local[dynamic_dof_slots, dynamic_dof_slots]` 子块（先 row 选，
  再 col 选；不允许只裁 row 不裁 col，或反之）；
- 这一步在 `EmbeddedSurfaceIPCPotentialEnergy` 的 external assembly helper 中以单一
  入口实现，禁止三类 pair 各写一份散布逻辑。

#### D. external CCD / feasible max step

- `computeMaxStepSize()` 的 broad phase 扩成：
  - self PT / EE
  - external V-T / T-V / E-E
- CCD 使用相对运动：
  - dynamic side 位移来自 `dx_surf`（Newton 当前 trial direction）
  - obstacle side 位移来自当前 stage 的 kinematic displacement
    `Δx_obs = current_positions - previous_positions`
- 返回统一的 `alpha_contact`。

**Newton line search 下的 α 语义**（Phase 2 固定，避免 Phase 3 反推）：

- obstacle endpoint **固定**为当前 stage 的 `current_positions`，**不**随 α 缩放；
- Newton line search 的 α 只缩放 dynamic Newton direction `dx_surf`；
- CCD 求出的 `alpha_contact` 是 “在 obstacle 已经完成本 stage 全部位移的前提下，dynamic
  侧最多能走多少” 的上界；
- 因此每个 stage 内 obstacle 的运动等价于一次先于 Newton 的位置跳转，Newton 只在
  obstacle 已经到位的几何上做求解；
- 这条约定来自 “lagged kinematic obstacle”：obstacle 不参与 Newton DOF，没有理由让
  obstacle 的真实物理位移被一个数值化的 α 缩短；
- 副作用：若 stage 区间内 obstacle 单步位移过大，应由上层缩小 `dt`，而不是由
  `alpha_contact` 缩放 obstacle 自身。Phase 2 不实现自动 `dt` 缩放，留作 Phase 3 之后的
  上层策略。

#### E. 运行时接线

- `runIPCSim` 在存在 `external-objects` 时：
  - 不构造 `TriangleMeshExternalContactHandler`
  - 不构造 `PointPenetrationEnergy`
  - 改为把 obstacle surfaces 注册到 `EmbeddedSurfaceIPCPotentialEnergy`
- `runSim` / `runShellSim` 的 legacy external path 保留，用于数值回归。

### Phase 2 验证

**集成器范围**：Phase 2 的所有验证只在 `ImplicitBackwardEulerTimeIntegrator` 路径下进行。
当前 `runIPCSim.cpp` 构造的就是 `ImplicitBackwardEulerTimeIntegrator`，没有 TRBDF2 接线；
Phase 2 不为 TRBDF2 编写或验证 stage-aware obstacle 路径，那部分在 Phase 3C 才打通。但
A.1 中 `ObstacleSurface::update(t_start, t_end)` 的签名已经按 stage-aware 设计，Phase 3C
扩展时不需要回头改 Phase 2 的数据结构。

- 核心单测
  - static obstacle：dynamic triangle / tet surface / cubic surface 靠近平面或 box obstacle，验证 barrier 能量、梯度方向正确；
  - kinematic obstacle：障碍物做已知平移，验证 CCD 返回的 `alpha_contact` 会缩步；
  - line-search α 仅缩放 dynamic side，obstacle endpoint 不随 α 改变（对应 D 节约定）；
  - multi-obstacle pair identity：两个 `ObstacleSurface` 注册后，各自顶点的 local index
    冲突时不会混淆 pair（用 `object_id` 区分）。
- 工具/样例验证
  - `runIPCSim` 的 shell 样例与外部 `bottom.obj` 一起运行时，不再经过 `PointPenetrationEnergy`；
  - `runIPCSim` 的 tet 与 cubic 含外部障碍样例可稳定推进并输出结果。
- 回归
  - `runSim` / `runShellSim` 的 external legacy 路径保持可用。

## 3.4 Phase 3：统一补 friction（含 TRBDF2 集成）

### 目标

在统一的 IPC self/external active pair 表示上，实现：

- self friction
- external friction
- `ImplicitBackwardEuler`
- `TRBDF2`

并保持一套共享的 friction 数学实现。

### 实现决策

Phase 3 是**一个统一的 friction 里程碑**，内部按实现顺序拆成：

- **Phase 3A：friction 数学层 + integrator adapter 接口**
- **Phase 3B：`ImplicitBackwardEuler` 集成**
- **Phase 3C：`TRBDF2` 集成**

其中 3A 必须先完成，3B/3C 都依赖 3A 提供的统一接口与数据表示；3B 可以先于 3C 完成，但二者共同组成 Phase 3 的完整验收。

#### Phase 3A：friction 只建立在 lagged contact state 上

v1 friction 采用 **semi-implicit / lagged** 设计：

- 在每个 timestep（或 TRBDF2 每个 stage）开始时，用上一收敛状态构建 lagged contact state；
- 为每个 active pair 缓存：
  - lagged normal / tangent frame
  - lagged closest-point barycentric coordinates
  - lagged normal force magnitude $\lambda_k^n$（定义见下方 “lambda 助手语义”）

**lambda 助手语义**（self/external/PT/EE 统一）：

`SurfaceIPCCore` 暴露单一 helper

```
double computeLaggedNormalForceMagnitude(
    const ActivePair &pair,
    ConstRefVecXd x_surf_lagged) const;
```

返回的 $\lambda_k^n$ 固定按以下方式从 barrier 量构造：

- 设 lagged distance $d_k = d(\text{pair}, x_{\text{surf}}^{\text{lagged}})$，$\hat d$ 为
  barrier 距离阈值（与 `SurfaceIPCCore` 现有 barrier 参数同源）；
- 设 barrier 强度参数 $\kappa$（IPC barrier 系数，与 barrier energy 用的同一份）；
- $\lambda_k^n$ 定义为该 pair 上 barrier 力沿法向的标量大小：
  $$
  \lambda_k^n = \kappa \, b'(d_k, \hat d) \, \cdot \, 2 d_k,
  $$
  其中 $b(d, \hat d)$ 是仓库现有 IPC log barrier 形式，$b'$ 为对 $d$ 的导数；$2 d_k$ 因子
  来自 $d^2$ 到 $d$ 的链式法则，使得返回值是物理意义上的法向力大小（量纲为力），而
  不是带平方距离链式因子的中间量；
- helper 内部 **包含** $\kappa$，调用方不需要再乘；
- PT / EE / self / external 全部走同一个 helper，区别只在 `pair.distance(...)` 的几何表达；
- 单位与 `contact-vel-eps` 的对齐由 Phase 3A 的 friction smoothing 阈值 $k =
  \text{contact-vel-eps} \cdot dt_{\text{stage}}$ 显式处理；`computeLaggedNormalForceMagnitude`
  本身只负责力的大小，不参与速度阈值。

测试要求：

- 单 pair PT、单 pair EE 的 self 与 external 各一组数值测，比较解析推导值与 helper 返回；
- 同一 pair 在 `(self, external)` 两种 owner 下、对相同几何，helper 返回数值相等；
- $d_k \to \hat d$ 时返回值平滑趋近 0；$d_k \to 0$ 时不出现 NaN（barrier 自身的渐近行为
  由 `SurfaceIPCCore` 现有 barrier 测试保证，helper 不重复实现兜底）。
- Newton 迭代中 friction 只对当前 $x$ 的 trial velocity 线性化，不在每次内层迭代里重新更新 $\lambda$ 与 tangent。
- lagged state 的 refresh 频率固定为：
  - `ImplicitBackwardEuler`：每个 timestep 开头一次；
  - `TRBDF2`：`TR` stage 开头一次，`BDF2` stage 开头再一次；
  - 不允许在同一 stage 的 Newton 内层迭代中刷新 lagged state。
- 这条 refresh 规则不是实现细节，而是 **semi-implicit / lagged friction 的时间离散语义**：
  - `ImplicitBackwardEuler` 只有一个 solve interval，因此一整个 timestep 内只冻结一份 lagged reference state；
  - `TRBDF2` 一个 timestep 被拆成 `TR` 与 `BDF2` 两个 solve interval，因此必须在两个 stage 开头分别冻结各自的 lagged reference state；
  - Newton 内层里的
    $$
    x^{(0)} \rightarrow x^{(1)} \rightarrow x^{(2)} \rightarrow \cdots
    $$
    只是对“当前 timestep / 当前 stage 目标解”的反复试探，而不是新的时间步推进；
    因此这时若重新刷新 lagged friction state，就不再是 lagged / semi-implicit friction，而会变成另一种 repeatedly relinearized / quasi-implicit 语义；
  - 本计划固定不采用那种语义。

这一步固定不做 fully implicit friction。

#### Phase 3A：friction smoothing 复用现有仓库公式

- smoothing 函数直接复用现有：
  - `PointPenetrationEnergy`
  - `PointTrianglePairCouplingEnergyWithCollision`
  的 `C1` friction smoothing 思路；
- 但 contact pair 不再来自 legacy handlers，而是来自新的 IPC active pair 集合；
- 明确统一参数语义：
  - `contact-vel-eps` 表示速度 smoothing 的阈值；
  - friction energy 的输入是切向相对速度的模长，而不是位移模长。

#### Phase 3A：self/external 统一的 friction primitive

每个 active pair 都统一存成：

- pair type: `PT` or `EE`
- owner type: `self` or `external`
- lagged closest-point weights
- lagged tangent operator 所需几何信息
- lagged $\lambda_k^n$

对应的切向相对速度定义：

- `PT`
  - 点与三角形最近点的相对速度投影到 lagged 切平面；
- `EE`
  - 两条边最近点的相对速度投影到 lagged 切平面；
- `external`
  - obstacle side 的速度来自其 kinematic trajectory；
- `self`
  - 两侧都来自当前 body 的 global trial velocity。

对 external friction，obstacle 侧速度的定义固定为 **stage-aware 离散速度**：
- `ImplicitBackwardEuler`
  - 使用整步区间
    $$
    [t^n, t^{n+1}]
    $$
    上的 obstacle 轨迹采样：
    $$
    v_{\text{obs}}^{\text{BE}}
      = \frac{x_{\text{obs}}(t^{n+1}) - x_{\text{obs}}(t^n)}{t^{n+1} - t^n};
    $$
- `TRBDF2` `TR` stage
  - 使用当前 stage 区间
    $$
    [t_{\text{start}}^{\text{TR}}, t_{\text{end}}^{\text{TR}}]
    $$
    上的 obstacle 轨迹采样：
    $$
    v_{\text{obs}}^{\text{TR}}
      = \frac{x_{\text{obs}}(t_{\text{end}}^{\text{TR}}) - x_{\text{obs}}(t_{\text{start}}^{\text{TR}})}
             {t_{\text{end}}^{\text{TR}} - t_{\text{start}}^{\text{TR}}};
    $$
- `TRBDF2` `BDF2` stage
  - 使用当前 stage 区间
    $$
    [t_{\text{start}}^{\text{BDF2}}, t_{\text{end}}^{\text{BDF2}}]
    $$
    上的 obstacle 轨迹采样：
    $$
    v_{\text{obs}}^{\text{BDF2}}
      = \frac{x_{\text{obs}}(t_{\text{end}}^{\text{BDF2}}) - x_{\text{obs}}(t_{\text{start}}^{\text{BDF2}})}
             {t_{\text{end}}^{\text{BDF2}} - t_{\text{start}}^{\text{BDF2}}};
    $$

统一要求：

- obstacle 侧不允许在 `TRBDF2` 下偷用整步平均速度
  $$
  (x_{\text{obs}}^{n+1} - x_{\text{obs}}^n) / h
  $$
  代替 stage velocity；
- `runIPCSim` 的 external obstacle adapter 必须提供
  $$
  x_{\text{obs}}(t_{\text{start}}),\quad x_{\text{obs}}(t_{\text{end}})
  $$
  的 stage-aligned 采样；
- body 与 obstacle 两侧必须使用同一个 `ContactStageContext`，确保 friction 比较的是同一时间区间上的相对切向速度。

#### Phase 3A：friction Hessian 的局部稳定化策略

friction 的局部非线性固定写成：

$$
E_{f,k}(x)
  = \mu\,\lambda_k^n\,\phi_\varepsilon(\|r_k\|),
$$

其中

$$
r_k = J_k \,(x_{\text{surf}} - x_{\text{ref,surf}}).
$$

这里：

- `J_k` 是第 `k` 个 lagged friction pair 的 **冻结切向相对速度线性映射**；
- `J_k` 的几何内容由 pair type 与 lagged tangent/closest-point weights 决定；
- 在同一个 timestep/stage 的 Newton 内层迭代中，`J_k`、$\lambda_k^n$、tangent frame 都保持不变。

实现策略固定为：

- friction Hessian **不**依赖 global regularization 或全局 LM damping；
- 先在 pair-local 的 reduced tangential space 里计算 Hessian，再 pull-back 到 surface space；
- 若当前实现沿用仓库现有 `InnT` 投影写法，则 reduced space Hessian 是 `3x3`；
- 若以后改成显式 2D tangent basis，则 reduced space Hessian 可以是 `2x2`；
- 无论 `3x3` 还是 `2x2`，稳定化规则都完全相同。

对每个 pair，固定按以下顺序执行：

1. 构造 reduced tangential quantity
   $$
   r_k = J_k \,(x_{\text{surf}} - x_{\text{ref,surf}})
   $$
   并计算
   $$
   \phi_\varepsilon(\|r_k\|),\quad
   \nabla_{r_k} \phi_\varepsilon,\quad
   H_{r,k} = \nabla^2_{r_k} \phi_\varepsilon;
   $$
2. 先显式对称化
   $$
   H_{r,k} \leftarrow \frac{1}{2}(H_{r,k} + H_{r,k}^\top);
   $$
3. 再对 reduced Hessian 做 per-pair PSD projection
   $$
   H_{r,k}^{+} = \Pi_{\mathrm{PSD}}(H_{r,k});
   $$
4. 最后再 pull-back 到 surface space
   $$
   H_{f,k}
     = \mu\,\lambda_k^n\, J_k^\top H_{r,k}^{+} J_k.
   $$

明确要求：

- **不允许**先把所有 pair 装到全局 sparse Hessian 之后，再依赖全局 regularization 修复 friction 的非 PSD；
- PSD clamp 必须是 **per-pair local** 的，并且发生在 pull-back 之前；
- 这一策略与现有 `CIPC` barrier 的 local PSD projection 保持一致，只是 friction 在更低维的 tangential reduced space 里完成。

在 $\|r_k\| \to 0$ 的极限处，不能用 “算出 NaN 再清零” 的方式兜底。首期固定使用解析极限：

- 设 smoothing 阈值
  $$
  k = \text{contact-vel-eps} \cdot dt_{\text{stage}};
  $$
- 当 $\|r_k\| < r_{\text{tiny}}$ 时，reduced Hessian 直接取
  $$
  H_{r,k} = \frac{2}{k} I
  $$
  （维度与当前 reduced tangential space 一致）；
- 其中 `r_tiny` 是一个仅用于避免除零和 NaN 的局部数值阈值，不改变 smoothing 的数学分段。

#### Phase 3A：integrator 适配接口

新增 `TrialVelocityAwarePotentialEnergy` 接口并在 friction energy 上实现。

`ImplicitBackwardEulerTimeIntegrator`：

- 在每个 timestep 求解前设置：
  - $\text{scale} = 1/h$
  - $x_{\text{ref}} = q$
- 这对应 lecture 5 的标准假设
  $$
  v = \frac{x^{n+1} - x^n}{h},
  $$
  在代码变量里就是
  $$
  v_{\text{trial}}(u) = \frac{u - q}{h}.
  $$

`TRBDF2TimeIntegrator`：

- 在 TR stage 求解前设置：
  - $\text{scale} = \alpha$
  - $x_{\text{ref}} = q + q_{\text{vel}} / \alpha$
  - `stage = TR_STAGE`
  - `t_start = t^n`
  - `t_end = t_{\text{mid}}`
- 在 BDF2 stage 求解前设置：
  - $\text{scale} = \beta_7$
  - $x_{\text{ref}} = q - (\beta_5\, q + \beta_6\, q_y) / \beta_7$
  - `stage = BDF2_STAGE`
  - `t_start = t_{\text{mid}}`
  - `t_end = t^{n+1}`

`TRBDF2` 的 friction adapter 必须直接和 `src/core/simulation/TRBDF2TimeIntegrator.cpp` 里的 stage velocity 公式对齐，而不是口头类比成
$v = (x - x^n) / h$：

- **TR stage**
  - 代码公式见 `TRBDF2TimeIntegrator.cpp` 中
    $$
    qvel_y = \alpha (u - q) - qvel.
    $$
  - 将其整理为
    $$
    qvel_y = \alpha u - \alpha q - qvel
            = \alpha \left(u - \left(q + \frac{qvel}{\alpha}\right)\right).
    $$
  - 因此 TR stage 的 trial velocity model 固定为
    $$
    v_{\text{TR}}(u) = \alpha (u - x_{\text{ref}}^{\text{TR}}),
    \qquad
    x_{\text{ref}}^{\text{TR}} = q + \frac{qvel}{\alpha}.
    $$
  - 这就是 plan 中 `scale = alpha`、`x_ref = q + qvel / alpha` 的来源。

- **BDF2 stage**
  - 代码公式见 `TRBDF2TimeIntegrator.cpp` 中
    $$
    qvel_1 = \beta_5 q + \beta_6 q_y + \beta_7 (u - q).
    $$
  - 将其整理为
    $$
    qvel_1 = \beta_7 u + \beta_5 q + \beta_6 q_y - \beta_7 q
           = \beta_7 \left(u - \left(q - \frac{\beta_5 q + \beta_6 q_y}{\beta_7}\right)\right).
    $$
  - 因此 BDF2 stage 的 trial velocity model 固定为
    $$
    v_{\text{BDF2}}(u) = \beta_7 (u - x_{\text{ref}}^{\text{BDF2}}),
    \qquad
    x_{\text{ref}}^{\text{BDF2}}
      = q - \frac{\beta_5 q + \beta_6 q_y}{\beta_7}.
    $$
  - 这就是 plan 中 `scale = beta_7`、`x_ref = q - (\beta_5 q + \beta_6 q_y)/beta_7` 的来源。

统一要求：

- friction 模块不直接假设
  $$
  v = \frac{x - x_{\text{last}}}{h}
  $$
  或
  $$
  v = \frac{x - x^n}{h};
  $$
- friction 统一消费由 integrator 提供的
  $$
  v_{\text{trial}}(x) = \text{scale} \cdot (x - x_{\text{ref}})
  $$
  这一仿射模型；
- 除 `TrialVelocityModel` 外，integrator 还必须提供与当前求解一致的 `ContactStageContext`；
- 但这条仿射模型在 `runIPCSim` 中必须先经 adapter 从 simulation space 转成 surface space：
  $$
  x_{\text{surf}} = x_{\text{surf,rest}} + W\,u_{\text{sim}},
  \qquad
  x_{\text{ref,surf}} = x_{\text{surf,rest}} + W\,u_{\text{ref,sim}},
  $$
  从而 friction 内核实际看到的是
  $$
  v_{\text{trial,surf}}
    = \text{scale}\,(x_{\text{surf}} - x_{\text{ref,surf}});
  $$
- friction 内核只处理 surface-space 的 `x_surf / x_ref_surf / scale`，
  不允许直接拿 `W` 去计算 `W(u_{\text{sim}} - u_{\text{ref,sim}})`；
- external obstacle 的 velocity 也必须先由 adapter 按 `ContactStageContext` 转成与当前 stage 对齐的 surface-space obstacle velocity；
- `ImplicitBackwardEuler`、`TRBDF2` 只是提供不同的 `scale / x_ref`，而不是各写一套 friction 数学。

不支持的积分器策略：

- 若一个 integrator 使用了 IPC friction，但没有提供 `TrialVelocityModel`，直接在运行时抛出清晰错误；
- 若 integrator 使用了 stage-aware IPC friction/external contact，但没有提供 `ContactStageContext`，
  也直接在运行时抛出清晰错误；
- 不允许 silently 回退成 $v = (x - x_{\text{last}}) / h$。

#### Phase 3A：组装与求解策略

- friction 作为 contact 系统的一部分进入总增量势能；
- integrator 先在 simulation space 提供 `TrialVelocityModel(scale, u_ref_sim)`；
- integrator 同时提供 `ContactStageContext(stage, t_start, t_end, dt_stage)`；
- adapter 再将其转译成 surface-space 的 `x_ref_surf`，并把 `x_surf / x_ref_surf / scale` 交给 friction 内核；
- 若存在 kinematic external obstacle，adapter 还要根据同一个 `ContactStageContext`
  采样 obstacle 轨迹并构造当前 stage 的 obstacle velocity；
- 先在 surface-space 计算 friction energy / gradient / Hessian；
- 再通过 $W^\top$ / $W^\top H\, W$ 映回 simulation-space；
- friction 不改写 current contact-feasible max step；
- 继续由 Phase 1.5 的 material-feasible max step 与 Phase 1/2 的 contact-feasible max step 共同约束线搜索。

#### Phase 3B：`ImplicitBackwardEuler` 集成

- 在 `runIPCSim` 的 `ImplicitBackwardEuler` 路径上接入 IPC friction；
- 在每个 timestep 求解前设置：
  - $\text{scale} = 1/h$
  - $x_{\text{ref}} = q$
- BE 路径先完成：
  - lagged contact state 构建
  - self/external friction pair 激活
  - friction energy / gradient / Hessian 组装
  - 与现有 line search 协同工作

#### Phase 3C：`TRBDF2` 集成

- 在 `runIPCSim` 的 `TRBDF2` 路径上复用 3A 的统一 friction 接口；
- 仅允许通过 `TrialVelocityModel` 提供 stage-local trial velocity，不额外引入第二套 friction 数据结构；
- 在 TR stage 求解前设置：
  - $\text{scale} = \alpha$
  - $x_{\text{ref}} = q + q_{\text{vel}} / \alpha$
- 在 BDF2 stage 求解前设置：
  - $\text{scale} = \beta_7$
  - $x_{\text{ref}} = q - (\beta_5\, q + \beta_6\, q_y) / \beta_7$
- `TRBDF2` 集成只允许复用 3A 的 friction potential 与 lagged pair/state，不单独发明一套 TRBDF2-specific friction 数学。

### Phase 3 验证

#### Phase 3A 验证

- 数学单测
  - 单对 PT / EE friction 的 energy/grad/hess 对有限差分；
  - `lambda_k^n` 与 barrier normal force 幅值一致；
  - `contact-vel-eps` 参数变化时 energy 曲线平滑且单调；
  - reduced tangential Hessian 在 PSD projection 后特征值非负；
  - 随机 `u_sim` 下验证 friction pull-back 后的
    $$
    H_{\text{sim}}
    $$
    满足对称性，例如
    $$
    \|H_{\text{sim}} - H_{\text{sim}}^\top\|
      < 10^{-12}\,\|H_{\text{sim}}\|;
    $$
  - $\|r_k\| \approx 0$ 时 Hessian 使用解析极限，不出现 NaN / Inf。

#### Phase 3B 验证

- Integrator 验证（BE）
  - `ImplicitBackwardEuler` 下 friction 结果与 lecture 5 的 $v = (x^{n+1} - x^n) / h$ 假设一致；
- 样例验证（BE）
  - self friction：shell 表面折叠/滑动、tet/cubic 表面自接触滑动；
  - external friction：deformable body 在斜面或 kinematic obstacle 上滑动；
  - BE 在 friction 打开时能稳定推进。

#### Phase 3C 验证

- Integrator 验证（TRBDF2）
  - `TRBDF2` TR / BDF2 两个 stage 的 friction trial velocity 与 stage 公式一致；
  - 刻意对比 “错误地用 $(x - x_{\text{last}})/h$” 与新 adapter，确认数值不同且新实现与 stage velocity 一致。
- 样例验证（TRBDF2）
  - self friction 与 external friction case 在 TRBDF2 下能稳定推进。

#### Phase 3 完整验收

- BE 与 TRBDF2 都能在 friction 打开时稳定推进；
- 两个积分器共用同一套 friction 数学模块与 `TrialVelocityAwarePotentialEnergy` 接口；
- 不允许存在一套 “BE friction” 和另一套 “TRBDF2 friction” 的平行实现。

## 3.5 Phase 4：profiling + spatial hash / broad phase 优化

### 目标

在 self/external frictionless + friction pipeline 都正确跑通后，先补统一 profiling 与 instrumentation，再基于 profiling 结果优化 broad phase / spatial hash。

### 优化顺序（固定）

#### A. 先做 instrumentation + profiling

Phase 1 已经落地并测试仓库级 profiling 基础设施；Phase 4A 的任务不再是“实现 profiler”，而是把 profiling 在完整 self/external/friction pipeline 上打开、补齐剩余路径接线、稳定输出统计，并先回答“时间花在哪里”，不直接改算法。

明确约束：

- Phase 1 / 批次 A 负责实现并测试 profiling 基础设施本身，以及首批 contact section 插桩；
- Phase 1-3 不要求对所有路径都完整采集，也不要求形成最终用户可见的 profiling 汇总；
- Phase 4A 负责把 profiling 扩展到完整 pipeline，并把计时与计数结果稳定输出出来。

必须采集的指标：

- `surface_mapping_ms`
  - $W x$
  - $W^\top g$
  - $W^\top H\, W$
- `pair_build_ms`
  - 静态 broad phase
  - swept broad phase
- `num_pt_pairs`
- `num_ee_pairs`
- `num_external_pairs`
- `ccd_ms`
- `barrier_energy_ms`
- `barrier_gradient_ms`
- `barrier_hessian_ms`
- `friction_energy_ms`
- `friction_gradient_ms`
- `friction_hessian_ms`
- `material_max_step_ms`
- `newton_total_ms`
- `line_search_ms`

输出要求：

- 默认关闭；
- 通过单独的 runtime/profile 开关开启；
- 支持每步摘要输出与整次运行汇总输出；
- 不要求集成外部 profiler，也不要求图形化展示。

#### B. 在已有显式 prepared state 之上扩展，而不是另起 wrapper cache

**当前 repo truth**：`SurfaceIPCCore` 已经有显式 prepared-state API，见
`src/core/contact/ipc/core/surfaceIPCCore.h:59-91`：

- `prepareForSurfacePositions(x_surf)`
- `isPreparedFor(x_surf)`
- `invalidatePreparedState()`
- `computeEnergyWithPreparedPairs()`
- `computeGradientWithPreparedPairs(g_surf)`
- `computeHessianWithPreparedPairs(H_surf)`
- `computeAllWithPreparedPairs(...)`
- 内部 `hasPreparedState_` / `preparedPositions_` 等。

Phase 4 的方向因此固定为 **扩展 / 接线已有 prepared-state API**，而不是在 wrapper 层另
建一份 cache。理由：

- core 已经显式持有 `preparedPositions_`，再在 wrapper 层维护一份 `cached_x_surf` 会产生
  两个真相源；
- core 已经暴露 `isPreparedFor(x_surf)`，wrapper 不需要再实现一份逐元素 `x_surf` 比较；
- friction（Phase 3）在每个 stage 开头会对 lagged $x_{\text{surf}}^{\text{lagged}}$ 调用一
  次 `prepareForSurfacePositions`，barrier path 与 friction path 共享同一份 prepared
  pair 表能直接降低重复 broad phase 的次数。

**Phase 4 实际工作**：

- 在 `EmbeddedSurfaceIPCPotentialEnergy` 与 legacy `CIPCPotentialEnergy` wrapper 中，把
  `computeEnergy / computeGradient / computeHessian / computeAll` 改为：
  - 调用前先 `core->prepareForSurfacePositions(x_surf)`（内部已有 `isPreparedFor` 短路）；
  - 然后走 `computeXxxWithPreparedPairs(...)` 路径；
- 显式 invalidate 的调用点固定为：
  - 每个 timestep / TRBDF2 stage 开头；
  - external obstacle 更新 (`ObstacleSurface::update`) 之后；
  - 任何会改变 dynamic surface 拓扑或 obstacle 注册集合的事件之后；
- `invalidatePreparedState()` 在以上点由 wrapper 主动调用；core 不在 `computeXxx` 内部
  做隐式失效；
- friction（Phase 3）的 lagged state 构建也从这份 prepared pair 表读取，避免 friction
  path 再做一次 broad phase。

**显式不做**：

- 不在 `SurfaceIPCCore` 内部新增 hash-based / 容差近似的 state key（保留现有
  `isPreparedFor` 的精确比较语义）；
- 不在 wrapper 层并行维护一份 `cached_x_surf / cached_active_pairs`，以避免双真相源；
- 不引入跨不同 `x_surf` 的 temporal coherence 或 warm-start active pair set，那部分留到
  Phase 4 D 才在 profiling 证据下评估；
- 不依赖外部版本号或指针身份做 cache key；
- 优先复用现有 `computeAllWithPreparedPairs` 路径，而不是在 energy / gradient / hessian
  三处分别重建。

#### C. 再做 hash workspace 复用

- 复用 spatial hash bucket 容器、visited arrays、candidate vectors 的 capacity；
- 避免每次 `findCollisionPairs` / `computeMaxStepSize` 都重新构造一整套 `unordered_map<int64_t, vector<int>>` 风格内存。

#### D. 再做结构性优化

只有在 B/C 完成后 profiling 仍显示 broad phase 明显占主导，才做：

- flat `(cellKey, primitiveId)` entry array + sort/compact；
- PT / EE 分开并行聚合；
- backtracking 沿同一搜索方向的 candidate reuse；
- temporal coherence / warm-start active pair set。

本阶段不允许在没有 profiling 证据的前提下直接重写 hash 结构。

### Phase 4 验证

- 正确性
  - 与优化前 active pair 集合完全一致；
  - `energy / gradient / hessian / maxStepSize` 数值一致或在浮点容差内一致。
- Profiling
  - 开启 profiling 后，能够稳定输出各阶段计时与 pair 统计；
  - profiling 关闭时，不改变数值结果，且额外开销可忽略。
- 性能
  - 在 shell、tet、cubic 各选一个代表性自接触 case；
  - broad phase 用时下降，且不会把总耗时转移成更大的缓存同步开销；
  - 回归中记录 pair build 时间和总步进时间。

## 4. 实施顺序与里程碑

按提交批次执行，禁止跳阶段：

1. **批次 A**
   - 引入 `SurfaceIPCCore`
   - `CIPCPotentialEnergy` 变成 identity wrapper
   - 新增 `EmbeddedSurfaceIPCPotentialEnergy`
   - 新增 `runIPCSim` 骨架与 shell 模式
   - 落地并测试仓库级 profiling 基础设施
   - 接入首批 contact profiling section / scoped timer

2. **批次 B**
   - `runIPCSim` tet/cubic self-contact IPC 路径
   - 新 contact 单测

3. **批次 C**
   - `DeformationModelEnergy::computeMaxStepSize`
   - tet/cubic inversion-free 验证

4. **批次 D**
   - external IPC（static/kinematic obstacles）
   - shell/tet/cubic external 样例

5. **批次 E1**
   - `TrialVelocityAwarePotentialEnergy`
   - `StageAwareContactPotentialEnergy`
   - IPC friction 数学模块
   - active pair lagged state
   - BE friction 集成
   - BE 下 self/external friction 验证
   - 对应 `Phase 3A + Phase 3B`

6. **批次 E2**
   - `TRBDF2` stage-aware friction 接入
   - `ContactStageContext`
   - stage-aware obstacle velocity sampling
   - `TR` / `BDF2` lagged state refresh 接线
   - `TRBDF2` friction 对照验证
   - 对应 `Phase 3C`

7. **批次 F**
   - 开启 profiling 输出并补齐剩余路径接线
   - broad phase cache / workspace reuse
   - spatial hash / broad phase 优化

每个批次必须满足对应验证项，才能进入下一个批次。

## 5. 测试与验收矩阵

### 单元测试

- `ScopedProfileSection` / profiling 基础设施
  - 默认关闭与显式开启语义
  - section 聚合与 reset / snapshot 语义
- `SurfaceIPCCore`
  - PT / EE barrier energy/grad/hess FD
  - CCD 缩步正确性
- `EmbeddedSurfaceIPCPotentialEnergy`
  - `W = I` 与 shell wrapper 数值对齐
  - 稀疏 `W` 的 gradient / Hessian pull-back 校验
- `DeformationModelEnergy::computeMaxStepSize`
  - tet/cubic 各自的翻转保护
  - 多 element / 多 integration point 的全局 min 聚合
  - shell `computeMaxStepSize()` 在本 phase 显式保持 `1.0`
  - `EDGE_QUAD` `computeMaxStepSize()` 在本 phase 显式保持 `1.0`
- integrator max-step 聚合
  - `ImplicitBackwardEulerEnergy` / `TRBDF2TimeIntegratorEnergy` 对 material/contact/other implicit model 正确取 `min`
- `IPCFriction`
  - PT / EE friction energy/grad/hess FD
  - BE 与 TRBDF2 的 velocity adapter

### 集成测试

- `runIPCSim`
  - shell self/external IPC
  - tet self/external IPC
  - cubic self/external IPC
  - friction on/off
- `runShellSim` / `runSim`
  - legacy/reference 回归

**本计划明确不包含的集成测试：**

- shell-vs-tet external IPC
- shell-vs-cubic external IPC
- tet-vs-cubic external IPC
- 两个 deformable body 同时参与求解的任意 mixed external contact

### 验收标准

- shell/tet/cubic 三类 mesh 都能通过 `runIPCSim` 跑 self-contact；
- shell/tet/cubic 三类 mesh 都能通过 `runIPCSim` 跑 external obstacle contact；
- 开启 friction 后，BE 与 TRBDF2 都不依赖错误的 $(x - x_{\text{last}})/h$ 假设；
- line search 同时受 contact 与 inversion-free 约束；
- 其中 `Phase 1.5` 的 inversion-free 约束在本计划当前版本只要求对 tet/cubic volume mesh 生效；
- `Phase 1.5` 不保证 trial state 上所有能量项都 finite；这继续由上层 line-search finiteness check 负责；
- 这一点目前在主计划里按 repo-level 假设处理，未在 source plan 中强绑具体类名；若实现期核实发现缺失，应在 `phase1.5.impl.md` 中标记并单列 follow-up；
- `runSim` / `runShellSim` legacy 路径在迁移阶段保持可用；
- spatial hash 优化完成后，active pair 与数值结果不发生行为回归。

## 6. 默认假设

- external IPC 首期只覆盖 **static / kinematic triangle-mesh obstacles**；
- 不在本计划里实现 deformable-vs-deformable 双向 external IPC；
- 不在本计划里实现 shell-vs-tet / shell-vs-cubic / tet-vs-cubic 等 mixed deformable external contact；
- friction 首期采用 **lagged semi-implicit**，不实现 fully implicit friction；
- 新的 unified IPC 功能只通过 `runIPCSim` 暴露，不在本计划内修改 `runSim` / `runShellSim` 默认行为；
- `shell`、`tet`、`cubic` 的 contact 几何统一为 **surface triangle mesh + embedding**；
- profiling 基础设施前置到 Phase 1 / 批次 A；真正的全链路采集、输出与 profiling-driven 优化仍后置到 Phase 4。
