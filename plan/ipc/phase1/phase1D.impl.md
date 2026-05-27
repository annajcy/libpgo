# Phase 1D Implementation Record

Source plan: `/Users/jinceyang/Desktop/codebase/libpgo/plan/ipc/phase1D.plan.md`

## Implemented

`Phase 1D` 已按 `runIPCSim` tet/cubic unified IPC 落地，当前 repo-truth 为：

- `runIPCSim` 不再是 shell-only
  - 同一 CLI 入口现在支持 `shell / tet / cubic`
  - 主循环统一走 implicit backward Euler + unified self-contact IPC
- 新增 setup helper：
  - `/Users/jinceyang/Desktop/codebase/libpgo/src/tools/runSim/runIPCSimSetup.h`
  - `/Users/jinceyang/Desktop/codebase/libpgo/src/tools/runSim/runIPCSimSetup.cpp`
- 新增统一 `RunIPCSim::IpcSimulationContext`
  - `M`
  - `simulationRestPosition`
  - `surfaceRestPositions`
  - `surfaceFromSimulationDispMap`
  - `surfaceMesh`
  - `elasticEnergy`
  - `pullingEnergies`
  - `pullingTargets`
  - `pullingTargetRests`
  - `collisionHandler`
  - 以及 shell/volume elastic 路径所需的 ownership keep-alive：
    - `simulationMeshOwner`
    - `deformationModelManagerOwner`
    - `deformationModelAssemblerOwner`
- `EmbeddedSurfaceIPCPotentialEnergy` 继续只构造一次、每帧只 clear/re-add 同一个 handler
- tet/cubic volume path 已接到现有 `runSim` volume helpers：
  - `RunSim::parseVolumeMeshInputConfig(...)`
  - `RunSim::resolveRunSimPaths(...)`
  - `RunSim::loadValidatedVolumeMesh(...)`
  - `RunSim::initializeVolumetricSimulation(...)`
  - `InterpolationCoordinates::BarycentricCoordinates::generateInterpolationMatrix()`

## Runtime Contract

当前 `runIPCSim` 的 phase1D 运行时语义已固定为：

- shell path 保持 phase1BC 语义
  - `W = I`
  - `elastic-material = koiter-stvk`
  - `scale ~= 1` guard 继续保留
  - shell mass matrix 继续 `libiglInterface::computeMassMatrix(...)` 后乘 `100`
  - `ipc-heuristic=true` 继续允许
- volume path 复用现有 volume contract
  - `elastic-material` 仅支持 `stable-neo` / `stvk-vol`
  - `koiter-stvk` 在 tet/cubic 显式报错
  - plastic 继续默认 `VOLUMETRIC_DOF6`
  - `ipc-heuristic=true` 在 tet/cubic 显式报错，即使同时给了 `ipc-dhat / ipc-kappa` 也不 fallback
  - tet/cubic 必须显式给 `ipc-dhat` 与 `ipc-kappa`
  - volume 允许 `scale != 1`，并对 volumetric mesh 与 surface mesh 共用同一个 `scale`
- shared config/runtime 语义
  - `init-disp` 继续只接受全零
  - `initialVel` 广播到全部 simulation vertices
  - `fixed-vertices` 始终解释为 simulation vertex indices
  - `fixed-vertices` 继续走 `MultipleVertexPulling` soft attachment，不切到 `TimeIntegrator::setFixedVertices(...)`
  - `external-objects` 仍显式拒绝
  - restart 仍只读写 `deformXXXX.u`
- output 语义
  - shell 输出表面：`surfaceRestPositions + u`
  - tet/cubic 输出表面：`surfaceRestPositions + W * u`

## Examples And Tests

本批新增并保留通过的 examples / tests：

- examples
  - `/Users/jinceyang/Desktop/codebase/libpgo/examples/ipc/tet/box-ipc.json`
  - `/Users/jinceyang/Desktop/codebase/libpgo/examples/ipc/tet/box.veg`
  - `/Users/jinceyang/Desktop/codebase/libpgo/examples/ipc/tet/box.obj`
  - `/Users/jinceyang/Desktop/codebase/libpgo/examples/ipc/tet/box-fixed.txt`
  - `/Users/jinceyang/Desktop/codebase/libpgo/examples/ipc/cubic/box-ipc.json`
  - `/Users/jinceyang/Desktop/codebase/libpgo/examples/ipc/cubic/box.veg`
  - `/Users/jinceyang/Desktop/codebase/libpgo/examples/ipc/cubic/box.obj`
  - `/Users/jinceyang/Desktop/codebase/libpgo/examples/ipc/cubic/box-fixed.txt`
- tests
  - `/Users/jinceyang/Desktop/codebase/libpgo/tests/src/tools/runIPCSim_gtest.cpp`
  - `runIPCSim_gtest` 现已覆盖：
    - shell regression
    - tet/cubic `num-timestep=0` / `1` smoke
    - tet/cubic non-unit-scale smoke
    - tet/cubic reject tests（`ipc-heuristic`、shell-only material、缺失 `ipc-dhat/kappa`）
    - tet/cubic production-wiring embedding gate

embedding consistency gate 当前已固定为：

- 使用新的 IPC example config 作为唯一输入面：
  - `/Users/jinceyang/Desktop/codebase/libpgo/examples/ipc/tet/box-ipc.json`
  - `/Users/jinceyang/Desktop/codebase/libpgo/examples/ipc/cubic/box-ipc.json`
- production 路径直接取 `buildVolumeIpcSimulation(...)` 返回的 `context.surfaceFromSimulationDispMap`
- baseline 在测试中从同一个 IPC example config 加载 mesh 后显式构造 `BarycentricCoordinates`
- test harness 会显式补齐：
  - `pgo::Logging::init()`
  - `pgo::Mesh::initPredicates()`
  以匹配 CLI 入口的全局初始化前置条件

## Build And Validation

本批实际验证命令：

```bash
cmake --build --preset base_no_mkl_debug --target \
  runIPCSim \
  runIPCSim_gtest \
  runSim_gtest \
  embeddedSurfaceIPCPotentialEnergy_gtest

ctest --test-dir build/base_no_mkl_debug --output-on-failure -R \
  "RunIPCSim|RunSim|EmbeddedSurfaceIPCPotentialEnergy"

build/base_no_mkl_debug/bin/runIPCSim examples/ipc/tet/box-ipc.json
build/base_no_mkl_debug/bin/runIPCSim examples/ipc/cubic/box-ipc.json
```

本地验证结果：通过。

## Implementation Notes

这批落地里有两个实现期确认并保留在 repo-truth 的点：

- `IpcSimulationContext` 必须显式保留 shell/volume elastic 链路的 ownership
  - 仅保留 `elasticEnergy` 不够稳
  - 需要把 `SimulationMesh`、`DeformationModelManager`、`DeformationModelAssembler` 一并挂在 context 上，避免 setup helper 返回后 shell/volume elastic 路径出现悬空引用
- 直接在 gtest 进程里调用 production helper 时，不能假设 CLI 已经做过全局初始化
  - builder-level tests 需要显式初始化 logging / predicates，才能与真实 CLI 前置条件对齐
