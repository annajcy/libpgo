# Phase 1.8 Implementation Record

Source plan: `/Users/jinceyang/Desktop/codebase/libpgo/plan/ipc/phase1.8/phase1.8.plan.md`

## Implemented

`Phase 1.8` 已按 “mapped-surface base + 独立 floor penalty + runIPCSim optional floor wiring” 落地，当前 repo truth 为：

- `src/core/contact/mappedSurfacePotentialEnergy.h/.cpp`
  新增 `MappedSurfacePotentialEnergy`
  - 统一持有 `surfaceRestPositions_`
  - 统一持有 `surfaceFromSimulationDispMap_`
  - 统一实现
    - `x_surf = x_rest + W u`
    - `g_sim = W^T g_surf`
    - `H_sim = W^T H_surf W`
  - `func / gradient / hessianDirect / computeMaxStepSize` 都走这层公共模板
  - 默认 `computeSurfaceMaxStepSize(...) == 1.0`
- `src/core/contact/embeddedSurfaceIPCPotentialEnergy.h/.cpp`
  现已继承 `MappedSurfacePotentialEnergy`
  - 仍然只负责 `SurfaceIPCCore`
  - 继续保留
    - `getContactClampCount()`
    - `getMinContactFeasibleAlphaThisSolve()`
    - `resetContactMaxStepStats()`
  - 不再自己维护 mapping / pull-back 样板代码
- `src/core/contact/embeddedSurfaceFloorPotentialEnergy.h/.cpp`
  新增 `EmbeddedSurfaceFloorPotentialEnergy`
  - 输入参数通过 `FloorPenaltyParameters`
  - 在 surface space 上实现 axis-selectable Cartesian floor penalty
  - 通过 base 自动 pull back 到 simulation DOF
  - `computeMaxStepSize(...)` 保持 `1.0`
- `src/tools/runSim/runIPCSimSetup.h/.cpp`
  - `IpcSimulationContext` 新增
    `extraGeneralImplicitForceModels`
  - shell / volume setup 都支持解析：
    - `use-floor`
    - `floor-axis`
    - `floor-height`
    - `floor-kappa`
  - 语义固定为：
    - `use-floor=false` 默认关闭
    - `use-floor=true` 时 `floor-axis` / `floor-height` / `floor-kappa` 必填
    - 若 `use-floor=false` 但仍写了 floor 字段，会打 info log 并忽略
  - floor potential 在 setup 内、紧挨 `collisionHandler` 构造，
    直接复用同一份 `surfaceRestVertices` 与 `surfaceFromSimulationDispMap`
- `src/tools/runSim/runIPCSim.cpp`
  - `collisionHandler` 继续作为 typed IPC handler 单独保留
  - extra floor model 通过
    `extraGeneralImplicitForceModels`
    循环 `addGeneralImplicitForceModel(...)`
  - contact clamp summary 仍只来自 IPC handler，不受 floor 污染

## Tests And Validation

本批新增并通过的测试：

- `/Users/jinceyang/Desktop/codebase/libpgo/tests/src/core/contact/embeddedSurfaceFloorPotentialEnergy_gtest.cpp`
  - identity embedding 在 `x / y / z` 三个 axis 上对 reference floor `E/g/H`
  - 非 identity `W` 的 `W^T g / W^T H W`
  - 非有限 floor 参数报错
  - 非法 floor axis 报错
- `/Users/jinceyang/Desktop/codebase/libpgo/tests/src/tools/runIPCSim_gtest.cpp`
  新增 floor 相关覆盖：
  - floor-enabled shell log 包含 `use-floor / floor-axis / floor-height / floor-kappa`
  - floor-enabled shell one-step smoke
  - floor-enabled cubic one-step smoke
  - `use-floor=true` 缺 `floor-axis` / `floor-height` / `floor-kappa` 时 setup 失败
  - floor-enabled volume setup 会生成额外 general implicit force model

本地验证命令与结果：

```bash
cmake --build --preset base_no_mkl_debug --target \
  embeddedSurfaceFloorPotentialEnergy_gtest \
  embeddedSurfaceIPCPotentialEnergy_gtest \
  runIPCSim_gtest \
  runIPCSim

ctest --test-dir build/base_no_mkl_debug --output-on-failure -R \
  "(EmbeddedSurfaceIPC|EmbeddedSurfaceFloor|RunIPCSim)"

cmake --build --preset base_no_mkl_release --target runIPCSim convertAnimation

build/base_no_mkl/bin/runIPCSim --log examples/ipc/cubic/box-with-sphere/box-ipc.json
build/base_no_mkl/bin/convertAnimation examples/ipc/cubic/box-with-sphere/anim.json
```

本地验证结果：通过。

- `ctest`：
  - 34/34 passed
- example runtime：
  - `runIPCSim` 完整跑完 `2000` timestep
  - `ret-box-with-sphere-ipc/` 在导出前产生 `2200` 个中间 frame/state 文件
  - `convertAnimation` 成功导出
    `/Users/jinceyang/Desktop/codebase/libpgo/examples/ipc/cubic/box-with-sphere/box-with-sphere-ipc-cubic.abc`
  - 导出 `.abc` 约 `7.5M`
  - CLI log
    `/Users/jinceyang/Desktop/codebase/libpgo/examples/ipc/cubic/box-with-sphere/box-ipc.log`
    约 `3.4M`
  - floor-enabled log 首行明确包含：
    - `use-floor=true`
    - `floor-axis=y`
    - `floor-height=-1`
    - `floor-kappa=1000`
  - example 交付前已清理中间 `ret-box-with-sphere-ipc/` 输出目录，只保留 committed `.log` 与 `.abc`

## Example Deliverable

本批新增 example：

- directory:
  `/Users/jinceyang/Desktop/codebase/libpgo/examples/ipc/cubic/box-with-sphere/`
- committed assets:
  - `box-with-sphere.veg`
  - `box-with-sphere.obj`
  - `box-ipc.json`
  - `anim.json`
  - `box-ipc.log`
  - `box-with-sphere-ipc-cubic.abc`

其当前 runtime contract 为：

- legacy `external-objects` 已移除
- floor 通过：
  - `use-floor=true`
  - `floor-axis=y`
  - `floor-height=-1.0`
  - `floor-kappa=1000.0`
- gravity 恢复 source case 的 `g=[0,-9.81,0]`
- self-contact 通过：
  - `ipc-dhat=0.002`
  - `ipc-kappa=3000.0`

## Intentional Drift From Earlier Intermediate State

在 axis-selectable floor follow-up 之前，repo 中曾短暂存在一个中间状态：

- floor 曾仅支持 world-space `z` 方向
- `examples/ipc/cubic/box-with-sphere/box-ipc.json` 一度改成
  `g = [0, 0, -9.81]`

这一步现在已经被正式 supersede：

- `EmbeddedSurfaceFloorPotentialEnergy` 已支持 `x / y / z` 三个 Cartesian axis
- example 已恢复到 source case 的 `y` 方向语义：
  - `floor-axis = y`
  - `g = [0, -9.81, 0]`
- 仍然不支持任意法向 analytic plane / ceiling / 双侧 wall
