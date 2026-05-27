# Phase 1BC Implementation Record

Source plan: `/Users/jinceyang/Desktop/codebase/libpgo/plan/ipc/phase1BC.plan.md`

## Implemented

`Phase 1BC` 已按 shell-only unified IPC 首版落地，当前 repo-truth 为：

- 新增 `EmbeddedSurfaceIPCPotentialEnergy`，作为新的 simulation-space IPC adapter
  - 固定接受 simulation displacement
  - 单阶段初始化
  - 持有 `SurfaceIPCCore`、flatten 后的 `surfaceRestPositions_`、`surfaceFromSimulationDispMap_`
  - `simulationDOFs_` 固定为 `[0, numSimulationDOFs)`
  - `hessian()` / `createHessian()` 延续 wrapper 语义，直接抛异常
- adapter profiling 已接到真实入口
  - `contact.adapter.func`
  - `contact.adapter.gradient`
  - `contact.adapter.hessian_direct`
  - `contact.adapter.max_step`
  - `contact.adapter.map_to_surface`
  - `contact.adapter.pullback_gradient`
  - `contact.adapter.pullback_hessian`
- 新增 `runIPCSim`
  - 首版只支持 shell
  - 复用 `runShellSim` 的 shell FEM + implicit backward Euler 主链
  - self-contact 改为通过 `EmbeddedSurfaceIPCPotentialEnergy` 用 `addGeneralImplicitForceModel(...)` 接入
  - 不接 floor / external / friction / tet / cubic
- 新增首版 IPC-only shell config：
  - `/Users/jinceyang/Desktop/codebase/libpgo/examples/ipc/shell/shell-ipc.json`
- 新增独立测试：
  - `/Users/jinceyang/Desktop/codebase/libpgo/tests/src/core/contact/embeddedSurfaceIPCPotentialEnergy_gtest.cpp`
  - `/Users/jinceyang/Desktop/codebase/libpgo/tests/src/tools/runIPCSim_gtest.cpp`

## Code Shape

实现后的职责边界固定为：

- `src/core/contact/embeddedSurfaceIPCPotentialEnergy.h/.cpp`
  - ctor 内做 surface mesh / embedding 维度与索引校验
  - `surfacePositions = surfaceRestPositions + surfaceFromSimulationDispMap * simulationDisplacements`
  - `simulationGradient = surfaceFromSimulationDispMap^T * surfaceGradient`
  - `simulationHessian = surfaceFromSimulationDispMap^T * surfaceHessian * surfaceFromSimulationDispMap`
  - `computeMaxStepSize()` 仅做 contact-side max step pull-forward / pull-back
- `src/tools/runSim/runIPCSim.cpp`
  - shell-only CLI 入口
  - 显式读取并打印 `ipc-dhat / ipc-kappa / eps_ee / slackness`
  - 显式拒绝 legacy `contact-*`、`external-objects`、tet/cubic、非零 `init-disp`
  - `num-timestep <= 1` 时避免 `ratio = frame / (numSimSteps - 1)` 的除零问题

`runIPCSim` 首版 config 语义当前已固定为：

- 必填：
  - `surface-mesh`
  - `fixed-vertices`
  - `g`
  - `init-vel`
  - `init-disp`
  - `scale`
  - `timestep`
  - `num-timestep`
  - `damping-params`
  - `sim-type`
  - `solver-eps`
  - `solver-max-iter`
  - `elastic-material`
  - `dump-interval`
  - `output`
  - `ipc-dhat`
  - `ipc-kappa`
- 固定默认：
  - `eps_ee = 0.0`
  - `slackness = 1.0`
- 显式拒绝：
  - `tet-mesh`
  - `cubic-mesh`
  - `external-objects`
- 兼容但忽略：
  - `contact-stiffness`
  - `contact-samples`
  - `contact-sample`
  - `contact-friction-coeff`
  - `contact-vel-eps`

## Tests And Validation

本批新增并保留通过的测试：

- `/Users/jinceyang/Desktop/codebase/libpgo/tests/src/core/contact/embeddedSurfaceIPCPotentialEnergy_gtest.cpp`
- `/Users/jinceyang/Desktop/codebase/libpgo/tests/src/tools/runIPCSim_gtest.cpp`

覆盖点固定为：

- `W = I` 时 `EmbeddedSurfaceIPCPotentialEnergy` 对齐 `CIPCPotentialEnergy(isInputDisp=true)`
- 手工稀疏 `surfaceFromSimulationDispMap` 的 gradient / Hessian pull-back 数学
- `contact.adapter.*` profiling section 存在性
- `runIPCSim --log`
- `num-timestep = 0`
- `num-timestep = 1`
- 缺失 `ipc-dhat / ipc-kappa` 的 fatal config error

本地验证命令：

```bash
cmake --build --preset base_no_mkl_debug --target \
  embeddedSurfaceIPCPotentialEnergy_gtest \
  cipcPotentialEnergy_gtest \
  cipcProfiling_gtest \
  surfaceIPCCore_gtest \
  runIPCSim_gtest \
  runSim_gtest

ctest --test-dir build/base_no_mkl_debug --output-on-failure -R \
  "EmbeddedSurfaceIPCPotentialEnergy|CIPCPotentialEnergy|CIPCProfiling|SurfaceIPCCore|RunIPCSim|RunShellSimCliLogging"

build/base_no_mkl_debug/bin/runIPCSim examples/ipc/shell/shell-ipc.json --log
```

本地验证结果：通过。

## Intentional Drift From Source Plan

相对 source plan，当前实现有两点实现期明确化：

- `runIPCSim` 对 `external-objects` 仍显式拒绝，但对 legacy `contact-*` 改为兼容忽略
  - 这样 shell IPC config 不需要为了迁移去删掉历史字段，同时这些字段也不会影响 phase1BC 的 IPC-only 路径
- `init-disp` 在首版中保留字段，但只接受全零
  - 这样既保留与现有 shell config 表面的字段连续性，又不把非零初始位移语义偷偷扩进本批

其余大方向与 source plan 保持一致：

- 不做 tet / cubic
- 不做 floor / external / friction
- 不做 `base-config`
- 不给新 adapter 保留 `isInputDisp`
