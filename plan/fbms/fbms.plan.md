# FBMS Simulation-Ready Asset 与 IPC 三案例计划

> **For implementers:** 这份计划把 FBMS 几何资产生成、TetWild 体网格转换、`runIPCSim` 仿真能力扩展、以及三个实验 case 串成一条可验证流水线。实施时建议按 phase 顺序推进，每个 phase 都要留下可运行命令、中间产物和最小回归测试。

## 1. 目标与当前决策

本计划服务于 `examples/fbms/g0_b8/` 这类 FBMS 结构实验，最终要得到：

1. 一个 simulation-ready 的 `.veg` tet volumetric mesh；
2. 与 `.veg` 几何一致的 contact/display surface `.obj`；
3. 三个 `runIPCSim` 配置：
   - Case 1：对 surface 每个顶点施加朝向球心的 pressure-like force `P * area_i`；
   - Case 2：用两个 kinematic plates 挤压结构；
   - Case 3：给结构一个很大初速度，让它撞墙；
4. 每个 timestep 输出：
   - `states/deformXXXX.u`：位移、速度、加速度；
   - `surface/retXXXX.obj`：surface 变形结果；
   - `stress/von_misesXXXX.json`：单个 timestep 的 von Mises stress；`values[i]` 对应第 `i` 个 tet element。

本计划固定采用以下几项设计决策：

- **球是球壳 thickening，不是实心球。**
- **几何 union 不走 explicit offset + boolean**；改用 SDF/implicit thickening 后直接 marching cubes 出 union surface。
- **体网格生成交给 TetWild/fTetWild，并作为 `tetMesher` 的可选 backend 暴露**；用 CMake 选项 `PGO_TET_MESHER_USE_TET_WILD` 控制是否启用，当前默认开启。仓库现状采用 `tetMesher --config path/to/tetmesh.json` 的 JSON 入口，TetWild backend 通过 fTetWild C++ API 从 surface 直接产出 simulation-ready `.veg`，不再把 `.msh` 作为默认流水线中间文件。
- **surface 整理用现有 `remeshSurface cgal_iso`**，放在 marching cubes 和 TetWild 之间。
- **仿真入口统一走 `runIPCSim` volume path**，不为 FBMS 单独复制一个 simulation driver。
- **external contact / friction 服从 `plan/ipc/ipc_friction.plan.md` 的 Phase 2/3 方向**；Case 2 和正式 Case 3 都依赖 kinematic external IPC contact。
- **Case 2/3 先用升级后的 floor penalty 做 prototype**，把材料/timestep/stress 输出/碰撞响应链路调稳后再切到 external IPC contact。floor energy 升级包括 (a) 支持 `lower`/`upper` 两侧、(b) 支持每帧 kinematic 高度变化、(c) 配置升级为 `floors[]` 多 floor 数组（一次性破坏式迁移，不保留旧字段）。Phase C (pressure force) 与 Phase D (floor 升级 + Case 2/3 prototype) 互不依赖，可并行推进。

## 2. Repo Truth

当前仓库已经有这些基础：

- `scripts/generate_bounding_sphere.py`
  - 读取 OBJ 顶点；
  - 用 AABB center + max vertex distance 生成外接球 OBJ；
  - 默认 `lat=64, lon=128, padding=1e-9`，能复现 `g0_b8_fbms_bounding_sphere.obj` 的 center/radius。
- `src/core/libiglInterface`
  - `computeDistanceField(...)`；
  - `computeMarchingCubes(...)`。
- `src/tools/remeshSurface/remeshSurface.cpp`
  - `cgal_iso` 可做 isotropic remeshing；
  - 当前 `--edge-length` 是相对输入 average edge length 的 scale，不是绝对长度。
- `src/tools/tetMesher/tetMesher.cpp`
  - 当前入口是 `tetMesher --config path/to/tetmesh.json`；
  - 旧的 `tetMesher tetgen ...` / `tetMesher tetwild ...` subcommand 已被拒绝；
  - 支持 `backend=tetwild` 和 `backend=tetgen`；
  - `tetwild` backend 被 `PGO_TET_MESHER_USE_TET_WILD` 编译选项保护；未启用时 JSON 指定 `backend=tetwild` 会给出明确运行时报错；
  - 支持 `output_surface`，会从生成后的 tet mesh 提取边界 surface OBJ。
- `src/tools/tetMesher/generateTetMeshSurfaceMesh.cpp`
  - 已有独立工具可从 `.veg` 提取 tet boundary surface 并写 OBJ；
  - 该能力已收敛进 `tetMesher` 的 `output_surface` 公共保存逻辑，FBMS pipeline 不需要额外跑散落的后处理命令。
- `src/tools/runSim/runIPCSim.cpp`
  - 已有 unified IPC volume path；
  - 支持 tet/cubic volume mesh + embedded contact surface；
  - 支持 `init-vel`；
  - 支持 material max-step；
  - 支持 floor penalty；当前仅支持单 floor、固定触发条件 `coord[axis] < height`（lower side），且 height 在构造后不可变；Phase D 会升级为多 floor + lower/upper side + 每帧 kinematic height；
  - 已支持 `states/`、`surface/`、`stress/` timestep 输出目录；
  - 已支持 volume path 的 `output-von-mises` stress JSON；
  - 当前显式拒绝 `external-objects`，external IPC contact 仍待实现。
- `src/core/solidDeformationModel/tetMeshDeformationModel.h/.cpp`
  - 单个 tet 的 `TetMeshDeformationModel::vonMisesStress(...)` 已实现；
- `src/core/solidDeformationModel/deformationModelAssembler.cpp`
  - `DeformationModelAssembler::computeVonMisesStresses(...)` 已补齐 assembler 层调用；
  - tet path 输出单个积分点 stress，cubic path 取积分点最大值。

## 3. Simulation-Ready 资产生成 Pipeline（与仓库现状对齐）

### 3.1 目标产物（当前默认目录）

当前仓库推荐把 FBMS 派生产物放在 job/case 目录，而不是直接平铺在 `examples/fbms/g0_b8/`。

以 `g0_b8 + r128_default` 为例：

```text
examples/fbms/generated/r128_default/g0_b8/
  union_shell_raw.obj
  union_shell_remesh.obj
  stats.json
  tetmesh.json
  union_shell.veg
  union_shell_tet_surface.obj
```

说明：

- 源输入仍在 `examples/fbms/g0_b8/`：`g0_b8_fbms.obj` 与 `g0_b8_fbms_bounding_sphere.obj`；
- `union_shell.veg` 作为体网格输入；
- `union_shell_remesh.obj` 可作为 surface-mesh；
- `union_shell_tet_surface.obj` 是从生成后的 tet mesh 提取的边界面，和 `.veg` 拓扑严格一致；
- 当前默认 pipeline 不产出 `.msh` 中间文件。

### 3.2 外接球生成

外接球脚本已可用，命令保持如下：

```bash
python3 scripts/generate_bounding_sphere.py \
  --input examples/fbms/g0_b8/g0_b8_fbms.obj \
  --output examples/fbms/g0_b8/g0_b8_fbms_bounding_sphere.obj \
  --method icosphere \
  --subdivisions 5
```

验收：

- 输出 OBJ header 中 `center/radius` 与当前输入模型一致；
- `sphere_parameterization=icosphere`；
- subdivisions=5 时，sphere 规模应为 `10242` 顶点、`20480` 面；
- 所有 FBMS 顶点到 center 的距离不超过 radius。

### 3.3 SDF thickening + union surface（已实现）

`generateFBMSUnionSurface` 已实现并接入构建；支持：

- `--fbms`、`--sphere`、`--fbms-thickness`、`--sphere-thickness`、`--resolution`、`--padding-ratio`、`--output-surface`；
- 场函数：

```text
f_fbms(x) = unsigned_distance_to_fbms(x) - fbms_thickness / 2
f_sphere_shell(x) = abs(norm(x - center) - radius) - sphere_thickness / 2
f_union(x) = min(f_fbms(x), f_sphere_shell(x))
```

- sphere 参数优先从 OBJ header 读取，失败时回退到 bbox+max distance；
- 输出日志包括 resolution、bbox、raw 顶点/面数量、field min/max。

推荐走批处理脚本（会同时生成 raw + remesh + stats）：

```bash
examples/fbms/generate_shell_assets.py \
  --job r128_default \
  --skip-existing
```

如需单独手工跑 `g0_b8/r128_default` 的 raw（与当前 `shell_assets.json` 一致）：

```bash
build/base_no_mkl/bin/generateFBMSUnionSurface \
  --fbms examples/fbms/g0_b8/g0_b8_fbms.obj \
  --sphere examples/fbms/g0_b8/g0_b8_fbms_bounding_sphere.obj \
  --fbms-thickness 0.025 \
  --sphere-thickness 0.025 \
  --resolution 128 \
  --padding-ratio 0.08 \
  --output-surface examples/fbms/generated/r128_default/g0_b8/union_shell_raw.obj
```

验收：

- raw surface 非空；
- `stats.json` 中 `assets.raw.vertices/faces` 大于 0；
- raw bbox 接近球壳厚化后的期望范围。

### 3.4 Surface remeshing（已实现）

当前 remesh 仍使用 `remeshSurface cgal_iso`，且由 `generate_shell_assets.py` 统一调度。

手工命令（`g0_b8/r128_default`）：

```bash
build/base_no_mkl/bin/remeshSurface cgal_iso \
  --input-mesh examples/fbms/generated/r128_default/g0_b8/union_shell_raw.obj \
  --output-mesh examples/fbms/generated/r128_default/g0_b8/union_shell_remesh.obj \
  --edge-length 0.75 \
  --sharp-edge-angle 180
```

调参说明（现状不变）：

- `--edge-length` 是相对输入平均边长的比例，不是绝对长度；
- `1.0` 近似保持原密度，`0.75` 默认推荐，`0.5` 更细，`1.5` 更粗。

验收：

- remesh OBJ 非空且可加载；
- `stats.json` 中 `assets.remesh.vertices/faces` 大于 0；
- remesh bbox 与 raw bbox 接近，无明显收缩异常。

### 3.5 `tetMesher` 现状：JSON 配置入口 + TetWild C++ API

这部分与旧计划有关键变更，当前实现以仓库代码为准：

- `tetMesher` 入口为 JSON 配置：

```text
tetMesher --config path/to/tetmesh.json
```

- 旧的 `tetMesher tetgen ...` / `tetMesher tetwild ...` 子命令入口已被拒绝；
- CMake 选项 `PGO_TET_MESHER_USE_TET_WILD` 当前默认值是 `ON`；
- 启用时通过 `FetchContent` 拉取 fTetWild，固定 commit：`d7d99bb4387a07895b9adce058dc7305f6b6e5ab`，并链接 `FloatTetwild`；
- TetWild 路径为 C++ API 直连（`floatTetWild::tetrahedralization`），不走子进程；
- 当前实现没有 `keep-msh`/`.msh` 导出参数。

当前 JSON 结构（仓库已用）：

```json
{
  "version": 1,
  "backend": "tetwild",
  "input_mesh": "union_shell_remesh.obj",
  "output_mesh": "union_shell.veg",
  "output_surface": "union_shell_tet_surface.obj",
  "print_stats": true,
  "quiet": true,
  "tetwild": {
    "lr": 0.05,
    "epsr": 0.001,
    "stop_energy": 10,
    "max_threads": 8
  }
}
```

字段说明：

- 公共字段：`version`、`backend`、`input_mesh`、`output_mesh`、`output_surface`、`print_stats`、`quiet`；
- `backend=tetwild` 支持：`lr`、`la`（与 `lr` 互斥）、`epsr`、`stop_energy`、`max_threads`；
- `backend=tetgen` 支持：`tetgen.command`；
- 相对路径按配置文件所在目录解析。

推荐直接复用现成配置：

```bash
build/base_no_mkl/bin/tetMesher \
  --config examples/fbms/generated/r128_default/g0_b8/tetmesh.json
```

可选 TetGen 示例：

```bash
build/base_no_mkl/bin/tetMesher \
  --config examples/fbms/generated/r128_default/g0_b3/tetmesh_tetgen.json
```

未启用 TetWild 时行为（当前实现）：

- `PGO_TET_MESHER_USE_TET_WILD=OFF` 仍可构建 `tetMesher`；
- 当 JSON 指定 `backend=tetwild` 时，运行时报错：

```text
tetwild backend is not enabled. Reconfigure with -DPGO_TET_MESHER_USE_TET_WILD=ON.
```

验收：

- `tetMesher --config .../tetmesh.json` 能生成非空 `.veg`；
- 若配置了 `output_surface`，能同步生成非空 boundary OBJ；
- `.veg` 的 reload check 通过；
- `PGO_TET_MESHER_USE_TET_WILD=OFF` 下 `tetgen` 配置仍可运行；
- 旧 subcommand 入口被拒绝，提示使用 `--config`。

## 4. `runIPCSim` 仿真基础能力扩展

### 4.1 输出目录布局

当前状态：已实现。

`runIPCSim` 的 timestep 输出已经从 output root 拆到语义清晰的子目录。旧布局曾直接写：

```text
<output>/deformXXXX.u
<output>/retXXXX.obj
```

当前布局为：

```text
<output>/states/deformXXXX.u
<output>/surface/retXXXX.obj
<output>/stress/von_misesXXXX.json
```

已实现行为：

- `runIPCSim` 在清空或创建 output folder 后，创建 `states/`、`surface/`、`stress/` 三个子目录；
- `restart-from-u=true` 时，从 `<output>/states/deformXXXX.u` 查找 restart state；
- 每步 deformation state 写到 `<output>/states/deformXXXX.u`；
- 每个 dump frame 的 surface OBJ 写到 `<output>/surface/retXXXX.obj`；
- von Mises JSON 写到 `<output>/stress/von_misesXXXX.json`；
- `runIPCSim.log` 继续保留在 `<output>/runIPCSim.log`，方便一眼找到主日志；
- 现有测试中检查 root-level `deformXXXX.u` / `retXXXX.obj` 的断言已同步迁移到新子目录。

已验证：

- 单步 tet IPC smoke case 生成 `states/deform0000.u`、`surface/ret0000.obj`；
- `restart-from-u=true` 能从 `states/` 子目录恢复；
- `dump-interval > 1` 时，`states/` 仍每步写，`surface/` 只按 dump interval 写；
- output root 不再混放 timestep 数据文件。

### 4.2 von Mises stress 输出

当前状态：已实现。

目标：每个 timestep 输出每个 tet element 一个 von Mises scalar。当前实现也支持 cubic volume path，并对 cubic element 取积分点最大 von Mises。

已修改：

```text
src/core/solidDeformationModel/deformationModelAssembler.cpp
src/tools/runSim/runIPCSim.cpp
tests/src/core/solidDeformationModel/deformationModelAssembler_gtest.cpp
tests/src/tools/runIPCSim_gtest.cpp
```

已实现行为：

- `DeformationModelAssembler::computeVonMisesStresses(...)`：
  - 对每个 element 提取 absolute local positions；
  - 提取该 element 的 plastic/elastic params；
  - 调 `femModels[ele]->prepareData(...)`；
  - 调 `femModels[ele]->vonMisesStress(...)`；
  - tet 单元 `nPt == 1`，直接写 `elementStresses[ele]`；
  - cubic 若 `nPt > 1`，写 element 内积分点最大值；本 FBMS tet path 固定只依赖 tet 正确。
- `runIPCSim` 支持 config：

```json
"output-von-mises": true
```

- 当启用后，创建：

```text
<output>/stress/von_misesXXXX.json
```

- 输出格式：

```json
{
  "frame": 12,
  "time": 0.006,
  "stress_type": "von_mises",
  "location": "tet_element",
  "values": [
    0.0,
    12.345,
    8.901
  ]
}
```

已验证：

- 单步 tet IPC smoke case 生成 `<output>/stress/von_mises0000.json`；
- JSON 可解析；
- `values.size()` 等于 tet element 数；
- `frame` 等于对应 timestep index；
- `time` 等于 `frame * timestep`；
- 全零位移、无外力、无重力时 stress 接近 0；
- Case 1 pressure force 运行后 stress 出现非零值（待 4.3 完成后验证）。

### 4.3 Surface pressure force

当前状态：下一步可实施；4.1/4.2 的输出与 stress 前置能力已经完成。

目标：支持 Case 1 的朝向中心压力载荷。

推荐先作为 `runIPCSim` volume-only 功能实现，不单独新增 PotentialEnergy：

```text
surface force f_surf_i = pressure * area_i * normalize(center - x_rest_i)
simulation force f_sim = W^T f_surf
external force = M g + ramp(frame) * f_sim
```

新增 config：

```json
"surface-pressure-force": {
  "enabled": true,
  "center": [0.0, 0.0, 0.0],
  "pressure": 1000.0,
  "ramp-steps": 20
}
```

实现要求：

- 仅对 volume path 生效；shell path 若出现该字段，应忽略 disabled 配置，或对 enabled 配置给出明确错误；
- `area_i` 用 surface mesh rest triangles 的 one-third face area 累加；
- direction 用 rest position 指向 center 的单位向量；
- 如果某个 vertex 距 center 小于 `1e-12`，该点 force 置零并打印 warning；
- `ramp(frame) = min(1, (frame + 1) / ramp_steps)`；
- `pressure` 可以为正；正值表示朝向 center；
- 如果后续需要 outward pressure，可用负 pressure，不新增方向字段。
- 当前 `runIPCSim` 仍要求 `fixed-vertices` 字段存在；Case 1 使用 `"fixed-vertices": []` 即可，不需要额外空文件。

验收：

- 在一个简单 sphere/tet smoke test 中，`f_sim` 维度等于 simulation DOF；
- `surface-pressure-force.enabled=false` 时回归现有行为；
- `pressure=0` 时输出与无 pressure case 一致；
- FBMS Case 1 能完成至少 10 timestep，并输出 stress。

### 4.4 External IPC contact

Case 2 和正式 Case 3 依赖 `plan/ipc/ipc_friction.plan.md` 的 Phase 2。

本计划只固定 FBMS 所需最小 external contact 能力：

- deformable embedded surface vs kinematic/static triangle obstacle；
- frictionless normal IPC 先完成；
- obstacle 每步可由配置定义 rigid translation；
- obstacle surface 的当前坐标进入 IPC contact pair build；
- obstacle 不参与 simulation DOF。

`runIPCSimSetup.cpp` 当前对 `external-objects` 调 `rejectIfPresent(...)`，Phase 2 完成后要移除这个拒绝，并解析：

```json
"external-objects": [
  {
    "filename": "plate_left.obj",
    "motion": {
      "translation-start": [-1.2, 0.0, 0.0],
      "translation-end": [-0.8, 0.0, 0.0]
    }
  }
]
```

验收：

- 一个 static wall smoke case 能阻止 volume surface 穿透；
- 一个 moving plate smoke case 能推动 deformable body；
- `runIPCSim` max-step summary 中 contact clamp 在强接触时可观测；
- Case 2/3 不再依赖 floor penalty。

### 4.5 Floor penalty 升级（Case 2/3 prototype 共用）

当前状态：待实施。这是 Phase D 的核心，独立于 4.3 的 pressure force，可与 Phase C 并行推进。

目标：
- 修正当前 `EmbeddedSurfaceFloorPotentialEnergy` 只支持「lower side」（`coord < height` 触发，向 +axis 推）的限制；
- 支持每帧改 `floorHeight`，让 floor 作为 kinematic 「移动平板」；
- 配置层支持多个 floor，让 Case 2 (two-plate squash) 可以同时挂两块对挤的 floor。

意图是用升级后的 floor energy 同时承担 Case 2 (two-plate squash) 和 Case 3 (wall impact) 的 prototype，把材料/timestep/stress 输出/碰撞响应链路调稳，**Case 2/3 的正式 external IPC 版本仍按 Phase E 推进**，floor prototype 不替代它。floor energy 不是 IPC、没有 exact CCD、不带 friction —— 这部分语义在升级前后不变，不要在 prototype 阶段补它。

#### 4.5.1 Energy class 改动

文件：

- `src/core/contact/embeddedSurfaceFloorPotentialEnergy.h`
- `src/core/contact/embeddedSurfaceFloorPotentialEnergy.cpp`

新增 `FloorSide` 枚举与 `floorHeight` setter：

```cpp
enum class FloorSide : int { LOWER = +1, UPPER = -1 };

struct FloorPenaltyParameters {
  FloorAxis floorAxis = FloorAxis::INVALID;
  FloorSide floorSide = FloorSide::LOWER;
  double floorHeight = NaN;
  double floorKappa  = NaN;
};

void   setFloorHeight(double h);   // 仅改 height，不允许改 axis/side/kappa
double floorHeight() const;
```

惩罚条件统一为：`s = sign(side); dz_eff = s * (coord[axis] - height)`，仅当 `dz_eff < 0` 时积分 `0.5 * kappa * dz_eff^2`：

- `LOWER` (+1)：`coord < height` 触发，gradient 沿 +axis（与现行行为一致）；
- `UPPER` (-1)：`coord > height` 触发，gradient 沿 -axis。

实现要点：

- Hessian 项保持 `kappa`（二阶不变号）；
- `setFloorHeight` 只更新 `params_.floorHeight`，class 内部不持有任何 frame/time 状态；motion schedule 由 runIPCSim 主循环负责；
- `floorAxis` 不可变；只有 height 需要每步更新；
- 构造期对 `side != LOWER && side != UPPER` 抛错。

#### 4.5.2 配置 schema：`floors[]`

`runIPCSim` 顶层从单 floor 字段切换到 `floors` 数组：

```json
"floors": [
  {
    "axis": "x",
    "side": "lower",
    "kappa": 1.0e6,
    "motion": {
      "height-start": -1.3,
      "height-end":   -0.75,
      "frame-start": 20,
      "frame-end":   220
    }
  },
  {
    "axis": "x",
    "side": "upper",
    "height": 1.2,
    "kappa": 1.0e6
  }
]
```

字段：

- `axis`：`"x"` / `"y"` / `"z"`；必填；
- `side`：`"lower"` 或 `"upper"`；可选，默认 `"lower"`；
- `kappa`：必填；
- `height`：静态 floor 用；与 `motion` 互斥，必须二选一；
- `motion`（可选）：包含 `height-start`、`height-end`、`frame-start`、`frame-end`；
  - 内插策略：linear；frame ∈ [frame-start, frame-end] 之间线性，外侧 clamp（frame < frame-start → height-start，frame > frame-end → height-end）；
  - 推荐 `frame-start ≥ 1`，给材料留 settle 帧，避免起步速度跳变冲击 line search；
  - 暂不实现 smoothstep / 其他 easing；如果后续观察到 line search 频繁失败再加 `"easing": "smoothstep"`，本阶段先 linear。

旧字段处理（一次性破坏式迁移，**不保留 backwards compat**）：

- 解析时若仍出现 `use-floor` / `floor-axis` / `floor-height` / `floor-kappa` 顶层字段，直接抛错并提示改用 `floors[]`；
- 同步更新 `tests/src/tools/runIPCSim_gtest.cpp` 等所有现有用例到新 schema；
- 巡查 `examples/` 下所有 runIPCSim config，发现旧字段一并迁移。

#### 4.5.3 runIPCSim 主循环改动

`runIPCSimSetup.cpp`：

- 删除 `parseFloorConfig` 单 floor 路径；
- 新增 `parseFloorsConfig(jconfig) -> std::vector<ParsedFloor>`，每个 `ParsedFloor` 持 params + 可选 motion schedule + `shared_ptr<EmbeddedSurfaceFloorPotentialEnergy>`；
- 现有 volume / shell path 添加 floor energy 时遍历该列表，逐个 `add(...)`。

`runIPCSim.cpp` 主时间步循环开头：

```cpp
for (auto& f : floors) {
  if (f.motion.has_value())
    f.energy->setFloorHeight(interpolateLinear(*f.motion, frameIdx));
}
```

注意：floor 在一个 frame 的整次 line search 里 frozen，仅 frame 之间跳变。这在「单帧 floor 位移远小于结构尺度」时是无害近似；如果 motion 速度过大（单帧 height 跳变接近材料尺度），先压 motion 时长而不是细分 frame 内插。

#### 4.5.4 验收

1. Energy class unit test：单 tet + lower floor、单 tet + upper floor，各自验证 force 方向；`setFloorHeight` 能改变下一次能量评估结果；
2. Setup parser test：`floors: []` 空数组合法；缺 `axis`/`kappa` 报错；`height` 与 `motion` 同时存在或都缺失报错；旧 `use-floor`/`floor-*` 顶层字段被显式拒绝；
3. runIPCSim smoke：单 tet + 一个 moving upper floor，`frame-start=10, frame-end=50`，高度从结构上方移到结构内部，确认 tet 被压扁，且 stress 输出非零；
4. 已有 floor-related 回归测试更新到新 schema 后通过。

## 5. 三个 FBMS Test Cases

### 5.1 Case 1: Inward pressure

文件：

```text
examples/fbms/generated/r128_default/g0_b8/g0_b8_case1_pressure-ipc.json
```

推荐初始配置：

```json
{
  "tet-mesh": "union_shell.veg",
  "surface-mesh": "union_shell_remesh.obj",
  "fixed-vertices": [],
  "g": [0, 0, 0],
  "init-vel": [0, 0, 0],
  "init-disp": [0, 0, 0],
  "scale": 1.0,
  "timestep": 0.0005,
  "num-timestep": 200,
  "damping-params": [0.0, 0.0],
  "sim-type": "dynamic",
  "solver-eps": 1e-5,
  "solver-max-iter": 30,
  "elastic-material": "stable-neo",
  "dump-interval": 1,
  "output": "case1_pressure_output",
  "ipc-dhat": 0.002,
  "ipc-kappa": 3000.0,
  "enable-material-max-step": true,
  "output-von-mises": true,
  "surface-pressure-force": {
    "enabled": true,
    "center": [0.0, 0.0, 0.0],
    "pressure": 1000.0,
    "ramp-steps": 20
  }
}
```

说明：

- 推荐把 config 放在 `examples/fbms/generated/r128_default/g0_b8/`，让 `tet-mesh`、`surface-mesh`、`output` 都能使用相对路径。
- `fixed-vertices` 字段仍需要存在；空数组是当前 Case 1 的推荐写法。
- 验收关注整体 inward deformation 和 stress 分布，不要求外接球保持刚性。

### 5.2 Case 2: Two-plate squash

#### 5.2.1 Floor prototype（Phase D）

文件：

```text
examples/fbms/generated/r128_default/g0_b8/g0_b8_case2_squash_floor-prototype-ipc.json
```

思路：用两个 axis-aligned 半空间 floor 模拟两块平行板，从结构两侧对称收缩。本版本用于跑通材料、timestep、stress 输出与挤压响应链路；不替代 5.2.2 的 external IPC 正式版。

推荐配置（具体 height 数值需对照 `union_shell.veg` 的 bounding sphere radius，下面以 r ≈ 1.0 为假设）：

```json
{
  "tet-mesh": "union_shell.veg",
  "surface-mesh": "union_shell_remesh.obj",
  "fixed-vertices": [],
  "g": [0, 0, 0],
  "init-vel": [0, 0, 0],
  "init-disp": [0, 0, 0],
  "scale": 1.0,
  "timestep": 0.0005,
  "num-timestep": 300,
  "damping-params": [0.0, 0.0],
  "sim-type": "dynamic",
  "solver-eps": 1e-5,
  "solver-max-iter": 40,
  "elastic-material": "stable-neo",
  "dump-interval": 1,
  "output": "case2_squash_floor_prototype_output",
  "ipc-dhat": 0.002,
  "ipc-kappa": 3000.0,
  "enable-material-max-step": true,
  "output-von-mises": true,
  "floors": [
    {
      "axis": "x",
      "side": "lower",
      "kappa": 1.0e6,
      "motion": {
        "height-start": -1.3,
        "height-end":   -0.75,
        "frame-start": 20,
        "frame-end":   220
      }
    },
    {
      "axis": "x",
      "side": "upper",
      "kappa": 1.0e6,
      "motion": {
        "height-start":  1.3,
        "height-end":    0.75,
        "frame-start":  20,
        "frame-end":   220
      }
    }
  ]
}
```

设计说明：

- 两个 floor 对称，`frame-start=20` 给材料先 settle 一段；motion 跨 200 帧，对应 0.1 s @ `timestep=5e-4`；
- 起步 height 在 bounding sphere 外侧，保证无初始穿透；
- `kappa=1.0e6` 起调，看穿透量与 line search 表现增减；
- 不开 pressure force、不开 gravity、不开 init-vel；
- `output-von-mises: true`，挤压区域 stress 是主要观测量。

验收：

- 两个 floor inward motion 后 structure 被压缩；
- 无明显 tunneling（surface 顶点 x 坐标始终在 `[lower.height, upper.height]` 之间，允许 dhat 量级误差）；
- 挤压阶段 von Mises 在接触区域附近升高；
- material max-step 不长时间钉在极小值；如频繁触发，先调小 motion 速度（拉长 frame 跨度），再降 timestep。

已知局限（不在本 prototype 范围内修）：

- 半空间不是有限板，结构在 axis 方向之外的滑移没有对应几何约束（FBMS 在 bounding sphere 内部，axis-aligned 半空间在结构 bbox 外侧的覆盖默认是足够的）；
- 没有 friction，结构沿切向可能漂移；如观察到对称破坏，加少量对称 anchor 顶点而不是急着加 friction；
- 没有 exact CCD；高 kappa + 高 motion 速度可能让 line search 失败。

#### 5.2.2 External IPC contact 版本（Phase E）

文件：

```text
examples/fbms/generated/r128_default/g0_b8/g0_b8_case2_plate_squash-ipc.json
examples/fbms/generated/r128_default/g0_b8/plate_left.obj
examples/fbms/generated/r128_default/g0_b8/plate_right.obj
```

几何：

- 两个 plate 是 thin box 或 single-plane thickened mesh；
- 建议 plate 尺寸覆盖外接球直径的 `1.5x`；
- plate 初始位置在 sphere shell 外侧，保证初始无穿透；
- plate motion 对称向内。

配置结构：

```json
{
  "tet-mesh": "union_shell.veg",
  "surface-mesh": "union_shell_remesh.obj",
  "fixed-vertices": [],
  "g": [0, 0, 0],
  "init-vel": [0, 0, 0],
  "init-disp": [0, 0, 0],
  "scale": 1.0,
  "timestep": 0.0005,
  "num-timestep": 300,
  "damping-params": [0.0, 0.0],
  "sim-type": "dynamic",
  "solver-eps": 1e-5,
  "solver-max-iter": 40,
  "elastic-material": "stable-neo",
  "dump-interval": 1,
  "output": "case2_plate_squash_output",
  "ipc-dhat": 0.002,
  "ipc-kappa": 3000.0,
  "enable-material-max-step": true,
  "output-von-mises": true,
  "external-objects": [
    {
      "filename": "plate_left.obj",
      "motion": {
        "translation-start": [-1.3, 0, 0],
        "translation-end": [-0.75, 0, 0]
      }
    },
    {
      "filename": "plate_right.obj",
      "motion": {
        "translation-start": [1.3, 0, 0],
        "translation-end": [0.75, 0, 0]
      }
    }
  ]
}
```

验收：

- plate inward motion 后 structure 被压缩；
- no-tunneling：surface 不穿过 plates；
- contact clamp 统计在挤压阶段非零或至少 contact energy 明显生效；
- von Mises 在接触区域附近升高；
- 与 5.2.1 floor prototype 输出对比，挤压量级一致即可。

### 5.3 Case 3: High-speed wall impact

#### 5.3.1 Floor prototype（Phase D）

文件：

```text
examples/fbms/generated/r128_default/g0_b8/g0_b8_case3_wall_impact_floor-prototype-ipc.json
```

思路：用一个静态 `upper` floor 模拟 +x 方向的墙，结构以高初速度撞向它。

```json
{
  "tet-mesh": "union_shell.veg",
  "surface-mesh": "union_shell_remesh.obj",
  "fixed-vertices": [],
  "g": [0, 0, 0],
  "init-vel": [50, 0, 0],
  "init-disp": [0, 0, 0],
  "scale": 1.0,
  "timestep": 0.0002,
  "num-timestep": 300,
  "damping-params": [0.0, 0.0],
  "sim-type": "dynamic",
  "solver-eps": 1e-5,
  "solver-max-iter": 50,
  "elastic-material": "stable-neo",
  "dump-interval": 1,
  "output": "case3_wall_impact_floor_prototype_output",
  "ipc-dhat": 0.002,
  "ipc-kappa": 3000.0,
  "enable-material-max-step": true,
  "output-von-mises": true,
  "floors": [
    {
      "axis": "x",
      "side": "upper",
      "height": 1.2,
      "kappa": 1.0e6
    }
  ]
}
```

设计说明（关键修正）：

- `side: "upper"` 是相对于旧 plan 的语义修正：升级前的 floor energy 只支持 lower side，按旧 schema 写 `floor-axis: x, floor-height: 1.2` 会让所有 `x < 1.2` 的顶点（即初始全部顶点）被向 +x 推；正确的「+x 方向墙挡回」语义需要 `upper` side（`coord > height` 时往 -x 推）。
- `init-vel = [50, 0, 0]`、`timestep = 2e-4` → 单帧位移 0.01，相对 bounding sphere radius ~1 是百分之一量级，line search 应能跟上；
- 若 line search 频繁失败：先压 `init-vel`（如 25），再压 `timestep`，最后才动 `kappa`。

验收：

- 高速撞击不穿墙（surface 顶点 x ≤ 1.2 + dhat 量级误差）；
- material max-step 不频繁钉在极小值；
- stress 峰值出现在撞击区域并沿结构传播；
- 输出序列能转换成 `.abc` 或直接用 OBJ sequence 可视化。

#### 5.3.2 External IPC wall 版本（Phase E）

正式版改为 external IPC wall：

```json
"external-objects": [
  {
    "filename": "wall.obj",
    "motion": {
      "translation-start": [0, 0, 0],
      "translation-end": [0, 0, 0]
    }
  }
]
```

验收：

- 同 5.3.1 撞击不穿墙、stress 链路一致；
- contact clamp 在撞击瞬间可观测；
- 与 5.3.1 floor prototype 对比，撞击区域 stress 量级一致即可（floor prototype 没有 plate 几何边界，差异主要在边缘）。

## 6. 实施阶段建议（当前状态）

### Phase A: Asset pipeline（已完成）

1. `scripts/generate_bounding_sphere.py` 命令与输出已固化。
2. `generateFBMSUnionSurface` 已实现。
3. `examples/fbms/generate_shell_assets.py` 已串起 raw surface、remesh、stats 输出。
4. `tetMesher --config .../tetmesh.json` 已通过 TetWild C++ API 生成 `.veg`。
5. `output_surface` 已能同步导出 tet boundary OBJ。
6. `examples/fbms/generated/r128_default/g0_b8/` 已有 `union_shell.veg`、`union_shell_remesh.obj`、`union_shell_tet_surface.obj`。

### Phase B: Stress output（已完成）

1. `DeformationModelAssembler::computeVonMisesStresses(...)` 已补齐。
2. `runIPCSim` 已增加 `output-von-mises`。
3. 已增加 assembler 与 `runIPCSim` smoke/regression tests。
4. 下一次接 Case 1 时，用 FBMS `.veg` 跑 1 step，确认 `<output>/stress/von_mises0000.json` 的 `values` 长度等于 tet 数。

### Phase C: Case 1 pressure（下一步）

1. 增加 `surface-pressure-force` config parser。
2. 计算 surface vertex area 和 inward force。
3. 通过 `surfaceFromSimulationDispMap.transpose()` 得到 simulation DOF force。
4. 每步按 ramp 更新 external force：`M g + ramp(frame) * f_sim`。
5. 写 `examples/fbms/generated/r128_default/g0_b8/g0_b8_case1_pressure-ipc.json`。
6. 跑 10~20 step 作为调参起点，并确认 stress 非零。

### Phase D: Floor energy 升级 + Case 2/3 prototype

可与 Phase C 并行；落地顺序如下：

1. 改 `EmbeddedSurfaceFloorPotentialEnergy`：加 `FloorSide` + `setFloorHeight`；按 4.5.1 实现 lower/upper 两侧的能量/梯度/Hessian。
2. 加 energy class unit test：lower、upper 各一例验证 force 方向；`setFloorHeight` 后能量变化。
3. 改 `runIPCSimSetup.cpp`：删 `parseFloorConfig`，写 `parseFloorsConfig`；老 `use-floor`/`floor-*` 顶层字段被显式拒绝；解析 `motion` 块。
4. 改 `runIPCSim.cpp` 主循环：每 frame 开头 `setFloorHeight(interpolateLinear(motion, frame))`。
5. 迁移现有 floor-related 测试与 example config 到新 schema；运行 `runIPCSim_gtest` 确认不回归。
6. 写 `g0_b8_case3_wall_impact_floor-prototype-ipc.json`（5.3.1），跑通高速撞击，调 `timestep/init-vel/kappa`。
7. 写 `g0_b8_case2_squash_floor-prototype-ipc.json`（5.2.1），跑通双 floor 对挤；如发现切向漂移，加少量对称 anchor 顶点，不要在本阶段加 friction。
8. 用 stress 输出验证 Case 2/3 的撞击/挤压响应链路。

### Phase E: External IPC contact

1. 按 `plan/ipc/ipc_friction.plan.md` Phase 2 实现 kinematic external IPC。
2. 移除 `runIPCSimSetup.cpp` 对 `external-objects` 的拒绝。
3. 增加 plate/wall obstacle motion parser。
4. 写 two-plate squash 和 wall impact 正式 configs（5.2.2、5.3.2）。
5. 跑 Case 2、Case 3 正式版本，与 Phase D 的 floor prototype 输出做量级对比。

### Phase F: Friction and polish

1. 按 `plan/ipc/ipc_friction.plan.md` Phase 3 补 friction。
2. 给 Case 2 plate 和 Case 3 wall 增加 friction 参数。
3. 输出 contact diagnostics：active pair count、max penetration proxy、contact clamp count。
4. 增加 post-processing helper，把 stress 和 OBJ sequence 对齐可视化。

## 7. 推荐验证命令

构建需要的 targets：

```bash
cmake --preset base_no_mkl
cmake --build build/base_no_mkl --target generateFBMSUnionSurface remeshSurface tetMesher runIPCSim
```

资产生成：

```bash
python3 scripts/generate_bounding_sphere.py \
  --input examples/fbms/g0_b8/g0_b8_fbms.obj \
  --output examples/fbms/g0_b8/g0_b8_fbms_bounding_sphere.obj \
  --method icosphere \
  --subdivisions 5

build/base_no_mkl/bin/generateFBMSUnionSurface \
  --fbms examples/fbms/g0_b8/g0_b8_fbms.obj \
  --sphere examples/fbms/g0_b8/g0_b8_fbms_bounding_sphere.obj \
  --fbms-thickness 0.025 \
  --sphere-thickness 0.025 \
  --resolution 128 \
  --padding-ratio 0.08 \
  --output-surface examples/fbms/generated/r128_default/g0_b8/union_shell_raw.obj

build/base_no_mkl/bin/remeshSurface cgal_iso \
  --input-mesh examples/fbms/generated/r128_default/g0_b8/union_shell_raw.obj \
  --output-mesh examples/fbms/generated/r128_default/g0_b8/union_shell_remesh.obj \
  --edge-length 0.75 \
  --sharp-edge-angle 180

build/base_no_mkl/bin/tetMesher \
  --config examples/fbms/generated/r128_default/g0_b8/tetmesh.json
```

也可以直接走批处理脚本生成 raw/remesh/stats：

```bash
examples/fbms/generate_shell_assets.py \
  --job r128_default \
  --skip-existing
```

仿真（Phase C / D — pressure + floor prototype）：

```bash
build/base_no_mkl/bin/runIPCSim examples/fbms/generated/r128_default/g0_b8/g0_b8_case1_pressure-ipc.json --log
build/base_no_mkl/bin/runIPCSim examples/fbms/generated/r128_default/g0_b8/g0_b8_case2_squash_floor-prototype-ipc.json --log
build/base_no_mkl/bin/runIPCSim examples/fbms/generated/r128_default/g0_b8/g0_b8_case3_wall_impact_floor-prototype-ipc.json --log
```

仿真（Phase E — external IPC 正式版）：

```bash
build/base_no_mkl/bin/runIPCSim examples/fbms/generated/r128_default/g0_b8/g0_b8_case2_plate_squash-ipc.json --log
build/base_no_mkl/bin/runIPCSim examples/fbms/generated/r128_default/g0_b8/g0_b8_case3_wall_impact-ipc.json --log
```

## 8. 主要风险与处理策略

- **SDF resolution 太低导致 FBMS 细节丢失**：先用 `128` 调通，再用 `256/384` 生成正式资产；比较 raw/remesh surface 的 bbox、face count 和视觉细节。
- **remeshing shrink 或破坏薄结构**：优先试 `--edge-length 1.0`，再试 `0.75`；如果 `cgal_iso` 破坏结构，TetWild 直接吃 raw surface。
- **TetWild API 或 FetchContent 变化**：当前固定 fTetWild commit `d7d99bb4387a07895b9adce058dc7305f6b6e5ab`；升级时先跑 `tetMesher_gtest` 和 FBMS `tetmesh.json` smoke。
- **surface embedding 失败**：确认 `surface-mesh` 与 `tet-mesh` 几何处在同一尺度和坐标系；`scale` 默认固定为 `1.0`。
- **Case 1 无约束导致整体漂移**：pressure 理论上近似对称，但离散误差可能导致刚体漂移；必要时加极弱 anchor 或移除每步合力的均值分量。
- **高速撞击数值不稳**：先减小 timestep，再降低 initial velocity，最后调大 damping；不要先盲目增大 contact stiffness。
- **von Mises stress 单位/量级不直观**：先用简单 tet stretch/compression smoke case 做 sanity，再看 FBMS 复杂 case。

## 9. 完成标准

本计划完成时应满足：

- `examples/fbms/generated/r128_default/g0_b8/` 中有可复现命令生成的 `union_shell_remesh.obj` 和 `union_shell.veg`；
- `tetMesher --config .../tetmesh.json` 能稳定生成 `.veg` 和 `union_shell_tet_surface.obj`；
- `runIPCSim` 能对 FBMS volume path 输出 `surface/retXXXX.obj`、`states/deformXXXX.u` 和 `stress/von_misesXXXX.json`；
- Case 1 pressure 能跑通并产生 inward deformation；
- Floor energy 升级（lower/upper side、`setFloorHeight`、`floors[]` schema）已合入并通过 unit + smoke test；旧 `use-floor`/`floor-*` 字段被显式拒绝；
- Case 2 two-plate squash 至少有 floor prototype（双 floor 对挤），external IPC 完成后切到 plate mesh；
- Case 3 wall impact 至少有 floor prototype（单 upper floor），external IPC 完成后切到 wall mesh；
- 每个 case 都有独立 `*-ipc.json`，输出目录互不覆盖；
- README 或 plan 中记录了资产生成和仿真命令，后续可以从干净 build 重跑。
