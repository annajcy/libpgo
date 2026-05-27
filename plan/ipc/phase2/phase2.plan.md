# Phase 2 Plan: 统一 external IPC contact（kinematic obstacle, BE only）

Source plan: `/Users/jinceyang/Desktop/codebase/libpgo/plan/ipc/ipc_friction.plan.md` (§3.3)
Depends on:
- `/Users/jinceyang/Desktop/codebase/libpgo/plan/ipc/phase1/phase1D.plan.md`
- `/Users/jinceyang/Desktop/codebase/libpgo/plan/ipc/phase1.5/phase1.5.plan.md`
- `/Users/jinceyang/Desktop/codebase/libpgo/plan/ipc/phase1.8/phase1.8.plan.md`

Repo-truth reference:
- `src/core/contact/embeddedSurfaceIPCPotentialEnergy.h/.cpp`
- `src/core/contact/mappedSurfacePotentialEnergy.h/.cpp`
- `src/core/contact/ipc/core/surfaceIPCCore.h/.cpp`
- `src/core/contact/ipc/core/surfaceIPCPairs.h`
- `src/core/contact/ipc/topology/surfaceIPCTopology.h`
- `src/tools/runSim/runIPCSim.cpp`、`runIPCSimSetup.cpp`
- legacy: `src/tools/runSim/runSim.cpp`、`runShellSim.cpp` 中的 `external-objects` 路径

---

## 0. 一句话目标

把 “一个 deformable body 对若干 kinematic triangle-mesh obstacle 的接触” 这条 legacy
路径（`TriangleMeshExternalContactHandler` + `PointPenetrationEnergy`）替换为 IPC 表达，
统一进 `EmbeddedSurfaceIPCPotentialEnergy` / `SurfaceIPCCore`，跑 `ImplicitBackwardEuler`，
保证 Phase 3 可以直接在此之上加 friction，**不需要回头改 Phase 2 的数据结构**。

Phase 2 不动 friction，不动 TRBDF2。

---

## 1. Phase 2 在整张图里的位置

| Phase | 内容 | 现状 |
| --- | --- | --- |
| 1A/BC/D | unified IPC self-contact 主链路（shell/tet/cubic） | 已完成 |
| 1.5 | material-feasible max step | 已完成 |
| 1.8 | mapped-surface analytic floor penalty + base 抽出 | 已完成 |
| **2** | **unified external IPC contact（本计划）** | **待做** |
| 3 | unified friction（self + external，BE + TRBDF2） | 待做 |
| 4 | broad-phase / cache 优化 | 待做 |

Phase 2 之前 `runIPCSim` 显式拒绝 `external-objects` 字段
（见 `runIPCSimSetup.cpp:398`、`:549`：`rejectIfPresent(..., "external contact is out of
scope for phase1D.")`）。Phase 2 的终点之一就是把这两处 `rejectIfPresent` 拆掉，并改走
IPC 路径。

### 1.1 总进度（步骤完成勾选）

- [ ] **S1** 数据结构骨架（`ObstacleSurface` + `ExternalPTPair / ExternalTPPair / ExternalEEPair`）
- [ ] **S2** `SurfaceIPCCore` 注册接口（无外部行为变化）
- [ ] **S3** external narrow phase（barrier energy / gradient / hessian）
- [ ] **S4** external CCD / max step
- [ ] **S5** `EmbeddedSurfaceIPCPotentialEnergy` wrapper
- [ ] **S6** `runIPCSimSetup` 解析 `external-objects`
- [ ] **S7** `runIPCSim` 主循环接线 + 端到端 sample
- [ ] **S8** 文档 + `phase2.impl.md` 收束
- [ ] **验收** §10 单测、端到端 sample、回归全部勾完

---

## 2. Scope Lock

### 2.1 In Scope

- 新增 `ObstacleSurface`（owning type）数据结构。
- 在 `SurfaceIPCCore` 内增加 obstacle 注册 + external pair 表达 + external broad/narrow
  phase + external CCD。
- 在 `EmbeddedSurfaceIPCPotentialEnergy` 内增加 obstacle 注册接口与 stage 入口
  （`updateObstacleStage(t_start, t_end)`）。
- `runIPCSim` 三条路径（shell / tet / cubic）通过统一的 obstacle 注册接入 IPC external
  contact，替代 legacy `TriangleMeshExternalContactHandler` + `PointPenetrationEnergy`。
- 在 `runIPCSimSetup.cpp` 的两处 `rejectIfPresent("external-objects", ...)` 改为正式解析
  external object 列表 + 平移轨迹。
- floor（Phase 1.8 `EmbeddedSurfaceFloorPotentialEnergy`）与 obstacle（Phase 2
  `EmbeddedSurfaceIPCPotentialEnergy` 内的 obstacle 注册）可在同一 config 中共存，分属
  两个独立 energy model；max-step 各自约束，不互相覆盖。
- 配套核心单测、setup smoke、`runIPCSim` 端到端 sample。

### 2.2 Out of Scope（明确不做）

- **Friction**（self / external 都不做）→ Phase 3。
- **TRBDF2** 集成 → Phase 3C。Phase 2 只验证 `ImplicitBackwardEulerTimeIntegrator`
  （`runIPCSim.cpp:291` 当前构造的就是它）。
- legacy `runSim` / `runShellSim` 的 `external-objects` 路径迁移到 IPC：保留，作为数值
  回归参考，**不**改造。
- `removeObstacleSurface` / 运行期增删 obstacle：注册接口预留 `object_id`，但不实现
  remove。
- analytic obstacle（plane / sphere / capsule）：Phase 1.8 floor penalty 已经覆盖
  floor；analytic obstacle 全家桶不在本 phase。
- temporal coherence / warm-start active pair set → Phase 4。
- obstacle 顶点级 `dt` 缩放策略 → 上层（Phase 3 之后）。

### 2.3 数值范围

- obstacle 单 stage 位移视作小量；具体阈值不在 plan 中固化，由 sample 决定。
- 单 dynamic body + 多 obstacle 是支持的；多 dynamic body 不支持（仍走 self IPC）。

---

## 3. 数据结构契约

### 3.1 `ObstacleSurface`（新增）

文件位置：`src/core/contact/ipc/external/obstacleSurface.h/.cpp`。
namespace：`pgo::Contact::CIPC`。

```cpp
class ObstacleSurface {
public:
  using TrajectorySampler = std::function<void(double t, EigenSupport::RefVecXd out)>;

  ObstacleSurface(
      EigenSupport::MXd restVertices,    // num_obstacle_vertices x 3
      EigenSupport::MXi triangles,       // num_obstacle_tris   x 3, local index
      TrajectorySampler sampler);        // sampler(t, out) writes 3*num_vertices

  // Stage-aware endpoint refresh. Only called by wrapper, never by Newton inner loop.
  void update(double tStart, double tEnd);

  int32_t                          objectId()        const { return objectId_; }
  const EigenSupport::VXd&         restPositions()   const { return rest_; }
  const EigenSupport::VXd&         currentPositions()const { return current_; }
  const EigenSupport::VXd&         previousPositions()const{ return previous_; }
  const EigenSupport::MXi&         triangles()       const { return triangles_; }
  const EigenSupport::MXi&         uniqueEdges()     const { return uniqueEdges_; }

  void setObjectId(int32_t id) { objectId_ = id; }   // called by registry

private:
  int32_t              objectId_ = -1;   // assigned by EmbeddedSurfaceIPCPotentialEnergy
  EigenSupport::VXd    rest_;
  EigenSupport::VXd    previous_;
  EigenSupport::VXd    current_;
  EigenSupport::MXi    triangles_;       // local 0-based indices
  EigenSupport::MXi    uniqueEdges_;     // derived from triangles_
  TrajectorySampler    sampler_;
};
```

**约束**：

- `triangles_` / `uniqueEdges_` 的索引只在该 obstacle 内有效，不参与 dynamic 顶点编号。
- `uniqueEdges_` 在构造时一次性派生，Phase 2 内不变（kinematic obstacle 拓扑固定）。
- **`triangles_ / uniqueEdges_` 类型选择**：用 `EigenSupport::MXi`（matrix 形），与
  `ObstacleSurface` 自身内部一致；self topology 历史上用 `std::vector<std::array<int,2>>`
  表示 edges，两边不通。Phase 2 故意不强行统一——self topology 是 self IPC 既有 API，
  改它会引入跨 phase 改动。Phase 3/4 若想跨 self / external 复用 broad-phase / friction
  helper，再决定加一层 adapter 还是把 self 一起迁到 MXi。这条留作显式技术债。
- `update(tStart, tEnd)` **只**被 wrapper 在 stage 开头调用；Newton 内层不允许调用。
- Phase 2 只实现一个 sampler 工厂：`makeLinearTrajectorySampler(restPositions, velocity, t0)`，
  语义为 `sampler(t) = rest + velocity * (t - t0)`（`t0` 默认 `0.0`，作为速度起算的参考
  时间），对应当前 `runSim` 的 `movement` 字段语义。
- **stage 内线性运动 invariant**：下游消费者（CCD / broad phase / scatter）将
  `[tStart, tEnd]` 内 obstacle motion 视作 `previous → current` 之间线性插值。Phase 2
  的 sampler 工厂天然满足；任何未来 sampler 若在 stage 区间内有曲率，**必须**由调用方
  自行切分成更小的 stage，否则 CCD 假设与实际运动悄悄不一致。

### 3.2 ExternalPair 表达

**设计决策（Phase 2 锁死，不允许 Phase 3+ 推翻）**：external PT 接触按 orientation
拆成两个独立类型 `ExternalPTPair` / `ExternalTPPair`，而**不**用单一结构 +
`Orientation` enum。`ExternalEEPair` 不拆（只有一个 orientation：dyn 边 vs obs 边）。

**理由**：单结构 + tag 的写法里，`p / t0 / t1 / t2` 这四个 `int` 字段会随 orientation
切换索引空间（dyn-surface-global ↔ obstacle-local），类型系统零保护。S3/S4 实现者若
按 self `PTPair` 的肌肉记忆访问字段，会在不触发任何 assert / OOB 的情况下读到错误
顶点位置，且只能靠最末的 "锁 DOF self 等价" 数值对比测才能发现。拆类型后字段名诚实
（`dynVertex` / `obsTri[]`）、索引空间在 call site 编译期可见、broad phase 与 scatter
都走静态分派而非 runtime tag，把这条安全问题在类型层挡掉。

在 `surfaceIPCPairs.h` 中新增三个 struct，**不**复用现有 self `PTPair` / `EEPair`：

```cpp
// Phase 2 约定："dynamic 侧字段写在前面"。

struct ExternalPTPair {                  // dyn vertex × obs triangle
  int32_t            obstacleObjectId;
  int                dynVertex;          // dyn-surface-global index
  std::array<int, 3> obsTri;             // obstacle-local indices
  double             weight;             // area(point) * area(triangle)
};

struct ExternalTPPair {                  // dyn triangle × obs vertex
  int32_t            obstacleObjectId;
  std::array<int, 3> dynTri;             // dyn-surface-global indices
  int                obsVertex;          // obstacle-local index
  double             weight;
};

struct ExternalEEPair {                  // dyn edge × obs edge
  int32_t            obstacleObjectId;
  std::array<int, 2> dynEdge;            // dyn-surface-global indices
                                         //   (按 dyn surface unique_edges 行内顺序)
  std::array<int, 2> obsEdge;            // obstacle-local indices
                                         //   (按 ObstacleSurface::uniqueEdges 行内顺序)
  double             weight;             // length(edgeA) * length(edgeB)
};
```

**索引空间约定**（每个字段单一空间，编译期固定，不随任何 tag 漂移）：

| pair type | dynamic 侧字段（基数） | obstacle 侧字段（基数） | 12-DOF local 顺序 `(p0, p1, p2, p3)` |
| --- | --- | --- | --- |
| `ExternalPTPair` | `dynVertex` (1) | `obsTri[0..2]` (3) | `(dynVertex, obsTri[0], obsTri[1], obsTri[2])` |
| `ExternalTPPair` | `dynTri[0..2]` (3) | `obsVertex` (1) | `(obsVertex, dynTri[0], dynTri[1], dynTri[2])` |
| `ExternalEEPair` | `dynEdge[0..1]` (2) | `obsEdge[0..1]` (2) | `(dynEdge[0], dynEdge[1], obsEdge[0], obsEdge[1])` |

12-DOF local 顺序 = barrier kernel 期望的 4 点输入顺序。triangle / edge 内部顶点顺序
沿用 `SurfaceIPCTopology` 与 `ObstacleSurface::triangles() / uniqueEdges()` 的行内顺序，
不重新排序。

### 3.3 Stable pair identity（Phase 3 用）

Phase 3 friction lagged state 用 5 元组作为 key；Phase 2 不缓存 lagged state，但
**索引顺序必须稳定**：

```
(pair_type, owner_type, dynamic_side_indices, obstacleObjectId, obstacle_side_indices)
```

- `pair_type ∈ {SelfPT, SelfEE, ExtPT, ExtTP, ExtEE}` — external PT 拆为 `ExtPT / ExtTP`
  反映 §3.2 的类型拆分
- `owner_type` 由 `pair_type` 前缀隐式决定（`Self*` / `Ext*`）；保留显式字段是为了
  Phase 3 的统一 dispatch
- `obstacleObjectId = -1` 时为 self pair；external pair 必为 `>= 0`
- `dynamic_side_indices` / `obstacle_side_indices` 的 cardinality 由 `pair_type` 编译期
  确定（PT: 1/3；TP: 3/1；EE: 2/2；Self*: 全 dynamic），按 §3.2 表中字段顺序排出

Phase 2 在 narrow phase 输出 pair 时即按此规则填充；Phase 3 直接读。

**pair 容器内顺序**：每个 `extPTPairs_ / extTPPairs_ / extEEPairs_` 容器内的元素顺序由
broad-phase 扫描结果决定（obstacle 注册顺序 × 各自顶点 / 三角形 / 边的线性枚举顺序）。
Phase 2 不做额外排序后处理；这条顺序在单进程内确定性，跨进程稳定性受 §3.3 末段
"obstacle 持久 key" 的同一限制。Phase 3 若需要更强保证（如字典序），届时再决定。

**进程内稳定 vs 跨进程稳定**：上述 5 元组在单次进程内稳定（`obstacleObjectId` 由
`addObstacleSurface` 按注册顺序分配，注册顺序由 setup 单 pass 决定）。**跨进程不保证
稳定** —— 用户改 `external-objects` 数组顺序、或重启续跑时 `obstacleObjectId` 可能
漂移。Phase 2 不用 lagged state，这条不影响正确性；Phase 3 friction 若需要 warm-start
跨进程持久化，可改用 `external-objects[*].filename` 派生 hash 作为 obstacle 持久 key
（此为 Phase 3 决策点，本 phase 不固化）。

---

## 4. SurfaceIPCCore 内部改动

### 4.1 新增成员

```cpp
// in SurfaceIPCCore (private)
std::vector<std::shared_ptr<ObstacleSurface>> obstacles_;
mutable std::vector<ExternalPTPair>           extPTPairs_;
mutable std::vector<ExternalTPPair>           extTPPairs_;
mutable std::vector<ExternalEEPair>           extEEPairs_;
```

**拷贝 / 赋值语义**（`SurfaceIPCCore` 已显式声明 copy ctor + `operator=`）：

- `obstacles_`：浅拷贝。多个 core 实例共享底层 `ObstacleSurface` 状态，符合
  `shared_ptr` 语义；上层若需要独立 obstacle 状态自行 deep-copy。
- `extPTPairs_ / extTPPairs_ / extEEPairs_`：拷贝时与既有 `ptPairs_ / eePairs_`
  同处理（值拷贝；同步把 `hasPreparedState_` 一并拷贝 / 重置，规则与 self 路径一致）。
- copy ctor 与 `operator=` 必须显式列出这四个新字段，避免漏写导致 S2 阶段
  "无外部行为变化" 在拷贝路径下不成立。

### 4.2 新增 public 接口

```cpp
int32_t addObstacleSurface(std::shared_ptr<ObstacleSurface> obs);
// 分配并写回 obs->setObjectId(id)，返回 id。Phase 2 不实现 remove。

void clearObstacleSurfaces();
// 配置切换 / 单测 teardown 用。

void updateObstacleStage(double tStart, double tEnd);
// 遍历 obstacles_ 调每个 obs->update(tStart, tEnd)。**只**由 wrapper 在 stage 开头
// 调用，Newton 内层不允许调。obstacle bookkeeping 只在 core 一份，wrapper 不持有
// 重复列表，避免 lifecycle 不同步。

const std::vector<ExternalPTPair>& getExternalPTPairs() const;
const std::vector<ExternalTPPair>& getExternalTPPairs() const;
const std::vector<ExternalEEPair>& getExternalEEPairs() const;
```

`prepareForSurfacePositions(x_surf)` 在已有实现基础上额外构建 `extPTPairs_` /
`extTPPairs_` / `extEEPairs_`，使用 `obstacles_[*]->currentPositions()` 作为 obstacle
侧位置。

`computeEnergy / Gradient / Hessian / All` 与 `WithPreparedPairs` 变体都需要把 external
pair 的贡献加进去（dynamic-only block，见 §5）。

`computeMaxStepLimit(x_surf, dx_surf)` 的 broad phase 扩成 self PT/EE + external PT /
TP / EE（见 §6）。

### 4.3 不动的部分

- self PT/EE 表达、self CCD、self broad-phase 内部数据结构。

### 4.4 `SurfaceIPCCore::Parameters` 拆分（Phase 2 落地）

self 与 external 的活化距离 **不共用 `dhat`**。理由：obstacle 尺度与 dynamic body
尺度可能差一个数量级以上（小 dynamic body 撞大 box obstacle 是常见配置），同一份
`dhat` 要么激活过多 external pair（性能塌方），要么激活过少（穿透）。Phase 2
直接在 `Parameters` 层拆开，避免 Phase 3/4 再回头改 schema。

```cpp
struct Parameters {
  double dhat          = 1e-1;   // self pair 活化距离
  double dhat_external = 1e-1;   // external (dyn vs obstacle) pair 活化距离
                                 //   默认与 dhat 同值，保证不注册 obstacle 时
                                 //   self-only 路径数值与 Phase 1.8 完全一致
  double kappa         = 0.1;    // shared（self + external 同一份 barrier 强度）
  double eps_ee        = 0.0;    // shared
  double slackness     = 1.0;    // shared
};
```

- `dhat_external` 只影响 external pair 的 broad / narrow phase 激活与 barrier 评估，
  self path 严格走 `dhat`，互不污染。
- `kappa / eps_ee / slackness` 暂不拆。若 Phase 4 之后发现 external pair 的 barrier
  强度需要独立调参，再单独拆 `kappa_external`；本 phase 不预留字段（YAGNI）。
- **分离每个 obstacle 独立 `dhat_external`** 仍留到 Phase 4 之后；Phase 2 所有
  external pair 共享同一份 `dhat_external`。

---

## 5. 梯度 / Hessian 装配（dynamic-only block）

### 5.1 几何评估

每个 external pair 仍按 12-DOF / 4-point local configuration 计算 barrier 能量、梯度、
Hessian。external 与 self 共享同一份 `ipcBarrier.h` / `ipcDistancePrimitives.h` 几何 kernel。

### 5.2 散布规则

local DOF 序列固定为 §3.2 表中 `(p0, p1, p2, p3)` 顺序对应的 12 维
`(p0_x, p0_y, p0_z, p1_x, p1_y, p1_z, p2_x, p2_y, p2_z, p3_x, p3_y, p3_z)`。

每类 pair 的 dynamic 侧顶点 → local slot 映射由 §3.2 表直接给出；不再需要
`dynamic_local_slots / obstacle_local_slots` 这层抽象（已被类型拆分吸收）。

- 全局梯度：对每个 (local_slot, dyn_vert_global) 对，
  `g_surf[dyn_vert_global * 3 + d] += g_local[local_slot * 3 + d]`，`d ∈ {0,1,2}`。
  每类 pair 的对照表如下：
  - `ExternalPTPair`：`(0, dynVertex)`
  - `ExternalTPPair`：`(1, dynTri[0]), (2, dynTri[1]), (3, dynTri[2])`
  - `ExternalEEPair`：`(0, dynEdge[0]), (1, dynEdge[1])`
- 全局 Hessian：在所有 (dynamic local_slot, dynamic local_slot) 对上散布对应 3×3 块到
  全局 (dyn_vert_global, dyn_vert_global) 位置。**先 row 选再 col 选**，禁止只裁单边。
- obstacle 侧 DOF 视为常量，不在 surface 全局矩阵中分配列/行。

实现要求在 `SurfaceIPCCore` 内放三个独立 helper，每个 helper 按 pair 类型静态特化，
**不**走 runtime 分支：

```cpp
// gradient
static void scatterPT(const Eigen::Matrix<double, 12, 1>& g_local,
                      int dynVertexGlobal,
                      EigenSupport::RefVecXd g_surf);

static void scatterTP(const Eigen::Matrix<double, 12, 1>& g_local,
                      const std::array<int, 3>& dynTriGlobal,
                      EigenSupport::RefVecXd g_surf);

static void scatterEE(const Eigen::Matrix<double, 12, 1>& g_local,
                      const std::array<int, 2>& dynEdgeGlobal,
                      EigenSupport::RefVecXd g_surf);

// hessian 版本签名同构：12×12 in，稀疏矩阵 dynamic-only block out
```

三类 pair 各走各的 helper；没有 orientation 分支。

### 5.3 PSD projection

external pair 的 12x12 Hessian 在裁出 dynamic-only block **之前**做 per-pair PSD
projection，与 self 路径完全一致（沿用 `ipcHessianProjection.h`）。先 PSD 再裁块，
不允许颠倒。

**已知保守性**（不在 Phase 2 解决，留作观测点写进 `phase2.impl.md` S8）：先对完整
12×12 做 PSD 再裁出 dynamic-only 子块，数学上严格保 PSD，但比 "先裁后 PSD" 更保守，
可能让 dynamic-only Newton step 在密集 external pair 下步长偏小。若 Phase 4 实测
profile 显示这条是收敛瓶颈，再评估改成 "裁后 PSD" 的代价（届时 self 路径要不要同步
也是一道独立问题）。Phase 2 不基于这个猜测做 premature optimization。

---

## 6. CCD / line search 语义

### 6.1 broad phase

`computeMaxStepLimit(x_surf, dx_surf)` 扩展：

- self PT / EE：不变。
- external PT (dyn 顶点 × obs 三角形)：扫 `dyn_vertex × obstacle_tri`，候选喂 `ExternalPTPair` narrow CCD。
- external TP (dyn 三角形 × obs 顶点)：扫 `obstacle_vertex × dyn_tri`，候选喂 `ExternalTPPair` narrow CCD。
- external EE (dyn 边 × obs 边)：扫 `dyn_edge × obstacle_edge`，候选喂 `ExternalEEPair` narrow CCD。

每个 obstacle 单独参与扫描；不同 obstacle 之间不互查（kinematic 不撞 kinematic）。

### 6.2 CCD 相对运动

- dynamic side 位移 = `dx_surf`（Newton 当前 trial direction）。
- obstacle side 位移 = `Δx_obs = current - previous`（**已经**在 stage 开头被
  `ObstacleSurface::update` 写入，不依赖 α）。

### 6.3 Newton line-search 下的 α 语义（Phase 2 固定）

- obstacle endpoint 固定为 `current_positions`；**α 不缩放 obstacle motion**。
- α 只缩放 dynamic Newton direction `dx_surf`。
- 因此 `computeMaxStepLimit` 返回的 `alpha_contact` 含义是：
  “obstacle 已完成本 stage 全部位移的前提下，dynamic 侧最多能走多少”。
- 若 stage 区间内 obstacle 单步位移过大导致 CCD 不收敛或步长过小，由上层减小 `dt`；
  Phase 2 不实现自动 `dt` 缩放。

这条约定的依据是 “lagged kinematic obstacle”：obstacle 不参与 Newton DOF，物理位移不应
被数值 α 缩短。Phase 3 friction lagged $\lambda$ 也建立在 “obstacle 已经到位” 的假设上。

---

## 7. EmbeddedSurfaceIPCPotentialEnergy 改动

### 7.1 新增接口

```cpp
class EmbeddedSurfaceIPCPotentialEnergy : public MappedSurfacePotentialEnergy {
public:
  // ... existing ctor ...

  int32_t addObstacleSurface(std::shared_ptr<ObstacleSurface> obs);  // forwards to core
  void    clearObstacleSurfaces();                                    // forwards to core
  void    updateObstacleStage(double tStart, double tEnd);            // forwards to core
  void    invalidatePreparedState();                                  // forwards to core
};
```

`updateObstacleStage` 在 `runIPCSim` 的每个 BE timestep 开头被调用一次，随后调用
`invalidatePreparedState()`，以保证 `SurfaceIPCCore::isPreparedFor(x_surf)` 不会因为
obstacle endpoint 变了却 `x_surf` 不变而误命中。

### 7.2 函数语义重申

- `func / gradient / hessian` 仍走 `MappedSurfacePotentialEnergy` 的 surface →
  simulation 映射，**simulation 侧 DOF 数不变**：obstacle 侧无 simulation DOF。
- `computeSurfaceMaxStepLimit` 直接转给 `SurfaceIPCCore::computeMaxStepLimit`，由 core
  负责 self + external 的统一 max step。

### 7.3 ABI / 兼容性

`EmbeddedSurfaceIPCPotentialEnergy` 已有构造参数不变；新增接口都是默认空状态——不注册
obstacle 时表现与 Phase 1.8 完全一致，保留 self-only 路径的数值不变性。

---

## 8. runIPCSim / runIPCSimSetup 改动

### 8.1 setup 解析

`runIPCSimSetup.cpp:398` 与 `:549` 的 `rejectIfPresent(jconfig, "external-objects", ...)`
替换为：

- 读 `external-objects` 数组；
- 每项至少包含：`filename`（obj 文件相对路径）、`movement`（V3 平移速度，单位
  与现有 `runSim` 字段一致），可选 `scale`（沿用 dynamic body 的 `scale`）；
- 加载 `Mesh::TriMeshGeo`，转换为 `MXd V` + `MXi F`，构造一个 `ObstacleSurface`，
  sampler = `makeLinearTrajectorySampler(rest=V_flat, velocity=movement, t0=0.0)`；
- 把 `ObstacleSurface` 注册到 setup 上下文里（与 `EmbeddedSurfaceIPCPotentialEnergy`
  绑定）。
- **路径解析**：`parseExternalObjects(jconfig)` helper 内部统一走
  `jconfig.resolvePath(...)`，原语与 `runSimVolumeMeshIO.cpp:69` 形式一致
  （`jconfig.resolvePath(jko["filename"].get<std::string>())`）；shell / volume / cubic
  三条路径调同一个 helper、签名统一。
  - volume 路径历史上已经通过 `RunSim::resolveRunSimPaths(jconfig)`
    （`runIPCSimSetup.cpp:561`）聚合出 `ResolvedRunSimPaths::externalObjectFilenames`，
    `parseExternalObjects` **不**消费这个聚合字段（避免 helper 签名分叉），volume 路径
    因此会对每条 `filename` 做一次冗余解析；这条冗余 cheap，可接受。聚合器字段保留
    给 legacy `runSim` 共用，本 phase 不动。
  - shell 路径 `buildShellIpcSimulation`（line 395）历史上**未**调 `resolveRunSimPaths`
    聚合器，而是按 `jconfig.getResolvedPath(...)` 逐字段就地解析；helper 在 shell 路径
    下就地解析，与历史风格一致，无需把 shell 整条迁去聚合器（无关 refactor）。
  - 不论走哪条，`filename` 在 setup 入口处一次性解析为绝对路径，下游消费者拿到的
    永远是绝对路径，不再从 jconfig 取相对路径。

新增 config 字段一览（最小集，与 `runSim` 旧字段对齐）：

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `external-objects` | array | obstacle 列表 |
| `external-objects[*].filename` | string | obj 路径（相对 config，setup 内统一解析为绝对路径） |
| `external-objects[*].movement` | float[3] | 平移速度 |
| `external-objects[*].scale` | float (optional) | 同 dynamic scale |
| `ipc-dhat-external` | float (optional) | external pair 活化距离；缺省时取 `ipc-dhat` 的同值（保证不注册 obstacle 时 self-only 数值与 Phase 1.8 严格一致）。Phase 2 全局共享一份，不支持 per-obstacle override |

shell / tet / cubic 三条 setup 路径共用同一份解析逻辑（抽 helper，不复制三遍）。

### 8.2 主循环接线（`runIPCSim.cpp`）

- 注册阶段：把每个 `ObstacleSurface` 通过
  `embeddedIPC->addObstacleSurface(obs)` 注册，记录返回的 `object_id`（首期不导出，但
  保留接口以便后续日志/调试）。
- 每个 timestep 开头：
  1. `embeddedIPC->updateObstacleStage(t_curr, t_curr + h);`
  2. `embeddedIPC->invalidatePreparedState();`
  3. 进入 Newton 求解。
- Newton 路径不变：`MappedSurfacePotentialEnergy::func/gradient/hessian/
  computeMaxStepLimit` 已经把 external 贡献吃掉。
- `runIPCSim` 当前显式构造 `ImplicitBackwardEulerTimeIntegrator`
  （`runIPCSim.cpp:291`）；Phase 2 不改这一行。

### 8.3 legacy 路径

`runSim.cpp:277` / `runShellSim.cpp:228` 的 `external-objects` 仍走
`TriangleMeshExternalContactHandler` + `PointPenetrationEnergy`，保持可运行，作为数值
回归参考。**Phase 2 不删 legacy**。

---

## 9. 实现步骤（按 PR / commit 粒度排列）

每一步都自带可编译、可运行的验证点；中间步骤不允许“断开半天再修”。

### Step S1 — 数据结构骨架（无行为变化）

- [ ] 新建 `obstacleSurface.h/.cpp`，实现 §3.1 中的所有 getter、`update`、
  `makeLinearTrajectorySampler`。
- [ ] `surfaceIPCPairs.h` 新增 `ExternalPTPair` / `ExternalTPPair` / `ExternalEEPair`
  （按 §3.2 的类型拆分，**不**走单结构 + `Orientation` enum 的路）。
- [ ] 编译通过；不修改任何调用方。
- [ ] 验证：构造一个 `ObstacleSurface`，调用 `update(0, 1)`，检查
  `previous = sampler(0)`、`current = sampler(1)`。
- [ ] 单测：`test/contact/ipc/test_obstacleSurface.cpp`。

### Step S2 — SurfaceIPCCore 注册接口（仍无外部行为）

- [ ] 在 `SurfaceIPCCore` 内增加 `obstacles_`、`extPTPairs_`、`extTPPairs_`、`extEEPairs_`
  字段；同步更新 copy ctor / `operator=` 显式列出这四个字段（见 §4.1 拷贝语义）。
- [ ] 在 `SurfaceIPCCore::Parameters` 加 `dhat_external` 字段（默认与 `dhat` 同值），
  `setParameters / getParameters` 跟进；不修改 `kappa / eps_ee / slackness`（见 §4.4）。
- [ ] 实现 `addObstacleSurface` / `clearObstacleSurfaces` / 三个 getter。
- [ ] 所有 compute* 路径在没有 obstacle 时行为与现状逐位一致（包括 `dhat_external`
  缺省等于 `dhat` 时的 self-only 数值不变性）。
- [ ] 验证：现有 `surfaceIPCCore` 单测全部通过；新增 "注册后清空，self path 不变" +
  "`dhat_external != dhat` 不影响 self pair barrier" 两条单测。

### Step S3 — external narrow phase（barrier energy / gradient / hessian）

- [ ] 在 `prepareForSurfacePositions` 中扫 external 候选 pair，分别写入
  `extPTPairs_` / `extTPPairs_` / `extEEPairs_`，字段按 §3.2 索引空间约定填
  （dyn-side 字段填 dyn-surface-global 索引，obs-side 字段填 obstacle-local 索引）。
  写每个 pair 前 `assert(obs->objectId() >= 0)`，防止 unregistered obstacle 污染
  stable identity（见 §3.3 "external pair 必为 >= 0"）。
- [ ] 实现 §5 的 `scatterPT` / `scatterTP` / `scatterEE` 三个 helper。
- [ ] `computeEnergy / Gradient / Hessian / All`（含 `WithPreparedPairs` 变体）加 external
  贡献。
- [ ] 单测 — static obstacle 平面：dynamic 顶点接近平面，验证 barrier energy / gradient
  方向符号、Hessian PSD。
- [ ] 单测 — static obstacle box：分别覆盖 PT、TP、EE 三类 pair。
- [ ] 单测 — multi-obstacle：两个 `ObstacleSurface`，各自 vertex local index 都从 0 起，
  验证 pair 身份用 `obstacleObjectId` 区分，不串台。
- [ ] 数值对比：与“等价 self 配置（把 obstacle mesh 也作为 dynamic 但锁住 DOF）”比较
  energy/gradient 数值（容差 1e-10 以内），证明 dynamic-only block 没漏项。

### Step S4 — external CCD / max step

- [ ] `computeMaxStepLimit` 的 broad phase 扩成 §6.1 三类外部 broad phase；narrow phase 调
  external PT / TP / EE CCD（沿用 `ipcCCD.h`，只是把 obstacle 侧位移替换为
  `current - previous`，dynamic 侧位移为 `dx_surf`）。
- [ ] 单测 — kinematic obstacle 做已知平移（沿 -y 方向），dynamic 静止，调用
  `computeMaxStepLimit(x_surf, dx_surf=0)`，验证 `alpha_contact < 1` 时刚好接触。
- [ ] 单测 — dynamic 沿 +y 移动、obstacle 静止：验证返回与 α 缩放对称。
- [ ] 单测 — Newton line search 缩 α：dynamic side 收缩，obstacle endpoint 不动，
  `alpha_contact` 在 `dx_surf *= α` 下线性放大（关键不变量，对应 §6.3 约定）。

### Step S5 — EmbeddedSurfaceIPCPotentialEnergy wrapper

- [ ] 在 wrapper 上加 §7.1 的接口：`addObstacleSurface`、`clearObstacleSurfaces`、
  `updateObstacleStage`、`invalidatePreparedState`。
- [ ] `addObstacleSurface` 把 obs 同步注册到内部 `surfaceIPCCore_`。
- [ ] 单测：通过 wrapper 注册 obstacle，调用 `func / gradient / hessian /
  computeMaxStepLimit`，与直接走 `SurfaceIPCCore` 的同等输入数值一致。

### Step S6 — runIPCSimSetup 解析 external-objects

- [ ] 在 `runIPCSimSetup.cpp` 拆掉两处 `rejectIfPresent`，改为正式解析。
- [ ] 抽出 `parseExternalObjects(jconfig)` helper（单一签名，**不**传聚合器，详见
  §8.1），shell / tet / cubic 三处调用 site 完全一致。
- [ ] 解析失败（文件缺失、movement 缺字段）时 fail-fast 报错，不静默吞掉。
- [ ] 单测：setup-level smoke，注入一个最小 config（dynamic + 一个 obj 形 obstacle
  （floor 形或 box 任一，**不**是 Phase 1.8 analytic floor）+ 平移轨迹），验证 setup
  后 `EmbeddedSurfaceIPCPotentialEnergy` 含正确数量 obstacle。

### Step S7 — runIPCSim 主循环接线

- [ ] 在每个 BE timestep 开头调用 `updateObstacleStage(t, t + h)` +
  `invalidatePreparedState()`。
- [ ] 端到端 sample — shell + `bottom.obj` kinematic 障碍：dynamic shell 落到 obstacle 上，
  跑若干步无穿透、无 NaN。
- [ ] 端到端 sample — tet + box obstacle：dynamic tet body 与 box 接触并被 broad phase
  解决。
- [ ] 端到端 sample — cubic + sphere obstacle：复用 phase 1.8 已有的 `box-with-sphere`
  资产格式。
- [ ] 与 legacy `runShellSim` 在相同初始条件下定性对比（不要求逐位一致，要求“同样能稳定
  推进且无穿透”）。

### Step S8 — 文档 + 验证清单收束

- [ ] 更新 `runIPCSim` README / config schema 描述。
- [ ] 在 `phase2.impl.md` 记录实际落地的字段、helper 命名、单测路径，作为后续 Phase 3 的
  repo-truth 入口。

---

## 10. 验证清单（Phase 2 验收口径）

### 10.1 单测（必过）

- [ ] `test_obstacleSurface` — sampler / update / unique_edges 推导
- [ ] `test_obstacleSurface_zero_velocity` — `makeLinearTrajectorySampler(rest, V3d::Zero(), 0.0)` +
  `update(0, h)` 后 `previous == current == rest`，作为 "静止 obstacle" 的 sampler 工厂回归基线
- [ ] `test_surfaceIPCCore_external_static_plane` — static plane barrier energy / gradient / hessian
- [ ] `test_surfaceIPCCore_external_box_PT_TP_EE` — 三类 pair 覆盖
- [ ] `test_surfaceIPCCore_external_multi_obstacle` — `obstacleObjectId` 区分相同 local index
- [ ] `test_surfaceIPCCore_external_dynamicOnlyBlock_equivalence` — 与“锁 DOF 的 self”对比
- [ ] `test_surfaceIPCCore_external_ccd` — kinematic obstacle CCD + α 不缩放 obstacle
- [ ] `test_embeddedSurfaceIPCPotentialEnergy_external` — wrapper 端到端数值一致
- [ ] `test_runIPCSimSetup_externalObjects_parse` — setup smoke
- [ ] **α 语义不变量** — 同一 `(x_surf, dx_surf, obstacle current/previous)` 下，给
  `dx_surf` 缩放 α，`alpha_contact` 应同步线性放大，obstacle endpoint 不参与缩放。

### 10.2 端到端 sample（必跑通）

- [ ] `examples/ipc/shell/.../with-obstacle-floor`（shell + 平面 obstacle）
- [ ] `examples/ipc/tet/.../with-obstacle-box`（tet + box obstacle）
- [ ] `examples/ipc/cubic/.../with-obstacle-sphere`（cubic + sphere obstacle）
- [ ] `examples/ipc/tet/.../floor-plus-obstacle`（tet + Phase 1.8 analytic floor + 一个 box obstacle
  共存）— 验证 §2.1 的 floor + obstacle 共存契约，max-step 各自约束、互不覆盖

每个 sample 都要求：
- [ ] 完整跑完配置的 timestep
- [ ] 输出 `.abc` 或等价资产
- [ ] 全程无 NaN、无穿透（通过简单 V-T 距离检查脚本验证，不要求严格证明）

### 10.3 回归

- [ ] 关闭 `external-objects` 时，`runIPCSim` 的 shell / tet / cubic self-only 路径数值与
  Phase 1.8 完全一致（关键比较：energy trace、step count）
- [ ] legacy `runSim` / `runShellSim` 的 `external-objects` 路径保持可运行，不引入 IPC 依赖

---

## 11. 风险与显式打钉

| 风险 | 处理 |
| --- | --- |
| `external-objects` 字段在 legacy 与 IPC 路径都被消费，含义可能漂移 | Phase 2 不改 legacy 的 schema；IPC 路径只读同名字段的子集，多余字段忽略并 warn |
| 多 obstacle 下 pair 数量爆炸 | broad phase 用现有 spatial hash，per-obstacle 单独扫；Phase 4 才做结构优化 |
| `isPreparedFor(x_surf)` 在 obstacle 端点变了但 `x_surf` 不变时误命中 | wrapper 在 `updateObstacleStage` 之后强制 `invalidatePreparedState()`；不依赖 core 自动检测 |
| Phase 3 friction 反推 Phase 2 数据结构 | §3.2 类型拆分（PT/TP/EE 独立类型）+ §3.3 stable identity 提前固定；`ExternalPTPair / ExternalTPPair / ExternalEEPair` 字段名与顺序 Phase 2 锁死 |
| 单结构 + `Orientation` enum 的索引空间漂移（dyn-global vs obs-local 撞同名字段） | §3.2 拆成三个独立类型，字段名直接揭示索引空间，runtime tag 整条消失；scatter / broad phase 走静态分派 |
| TRBDF2 提前介入 | §2.2 明确出 scope；`ObstacleSurface::update(tStart, tEnd)` 已经是 stage-aware 签名，Phase 3C 不需要改 Phase 2 |
| 自动 `dt` 缩放 | 出 scope；CCD 失败由上层减 `dt` |
| self / external 共用 `dhat` 在大尺度差 obstacle 下激活范围失配 | §4.4 拆出 `dhat_external`；setup 提供 `ipc-dhat-external` 字段，缺省回落到 `dhat` 保留 self-only 数值不变性 |
| `SurfaceIPCCore` 拷贝时漏拷新字段导致 S2 "无外部行为变化" 在拷贝路径下失效 | §4.1 明确列出 copy ctor / `operator=` 对四个新字段的处理规则 |
| `external-objects[*].filename` 路径基准漂移 | §8.1 `parseExternalObjects(jconfig)` helper 单一签名，内部走 `jconfig.resolvePath(...)`（与 `runSimVolumeMeshIO.cpp:69` 同原语）；volume 路径既有 `ResolvedRunSimPaths::externalObjectFilenames` 聚合保留但不被本 helper 消费（避免签名分叉），shell / volume / cubic 调用 site 一致；setup 入口一次性解析为绝对路径 |
| 非线性 obstacle sampler 与 CCD 的线性插值假设悄悄不一致 | §3.1 invariant 写明 "stage 内 obstacle motion = linear"；非线性 sampler 必须由调用方切分 stage |
| Phase 3 friction warm-start 跨进程持久化需要稳定 obstacle key | §3.3 标记 `obstacleObjectId` 仅进程内稳定；跨进程方案（如 filename hash）作为 Phase 3 决策点显式 deferred，不在 Phase 2 固化 |
| floor (Phase 1.8) 与 obstacle (Phase 2) 共存时的 max-step 互相覆盖 | 两者分属独立 energy model，各自走 `MappedSurfacePotentialEnergy::computeSurfaceMaxStepLimit`，上层 BE 主循环对所有 model 的 max-step 取 min，互不直接交叉；§10.2 加共存 sample 做端到端验证 |
| PSD-before-crop 在密集 external pair 下偏保守 | §5.3 标为已知保守性观测点，写入 `phase2.impl.md` S8；Phase 4 若 profile 命中再评估 "裁后 PSD" |

---

## 12. 与 source plan 的差异 / 回流项

本计划在 source plan §3.3 基础上增加了以下显式细化（review 反馈采纳）：

- `ObstacleSurface` owning type + `object_id` + `TrajectorySampler` + `update` 签名固定；
  `update` 的 invariant 显式写明 "stage 内 obstacle motion 视作线性"；
- External PT 按 orientation 拆为 `ExternalPTPair` / `ExternalTPPair` 两个独立类型，
  EE 保持单类型 `ExternalEEPair`；索引空间在字段名上诚实（dyn-side / obs-side），
  `Orientation` enum 整条不引入；
- 12-DOF → dynamic-only block 散布按 pair 类型三特化（`scatterPT / scatterTP /
  scatterEE`），不再用统一 `dynamic_local_slots` 抽象，"先 row 后 col" 规则保留；
- `SurfaceIPCCore::Parameters` 拆出 `dhat_external`，self 与 external 活化距离独立，
  缺省同值保留 Phase 1.8 数值不变性；`kappa / eps_ee / slackness` 暂不拆；
- `SurfaceIPCCore` 拷贝 / 赋值语义显式写出四个新字段的处理规则；
- `external-objects[*].filename` 解析由 `parseExternalObjects(jconfig)` helper 内部走
  `jconfig.resolvePath(...)`，单一签名 shell / volume / cubic 共用；既有
  `ResolvedRunSimPaths::externalObjectFilenames` 聚合保留但本 helper 不消费；setup 入口
  一次解析为绝对路径；
- §3.3 stable identity 标记 "进程内稳定，跨进程不保证"，把 obstacle 持久 key 的方案
  显式 defer 到 Phase 3；
- floor (Phase 1.8) 与 obstacle (Phase 2) 共存写进 In Scope，端到端 sample 加共存
  case；
- PSD-before-crop 的保守性作为已知观测点写入 `phase2.impl.md` S8，Phase 2 不预先优化；
- Newton line-search α 只缩放 dynamic side、obstacle endpoint 固定；
- Phase 2 验证显式 BE-only，TRBDF2 留到 Phase 3C；
- 多 obstacle pair identity 用 5 元组 stable key（Phase 3 直接读）。

source plan 与本计划如有偏差，**以本计划为准**；Phase 3 plan 在引用 Phase 2 时也应链回
本文件，而不是只引 source plan。
