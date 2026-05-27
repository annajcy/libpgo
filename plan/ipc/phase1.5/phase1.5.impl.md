# Phase 1.5 Implementation Record

Source plan: `/Users/jinceyang/Desktop/codebase/libpgo/plan/ipc/phase1.5/phase1.5.plan.md`

## Implemented

`Phase 1.5` 已按“volume-only material max step”落地，当前 repo truth 为：

- `src/core/solidDeformationModel/deformationModelEnergy.h/.cpp`
  不再把 `computeMaxStepSize(...)` 写成恒等于 `1.0`
- `DeformationModelEnergy::computeMaxStepSize(...)` 现已对
  `SimulationMeshType::TET` 与 `SimulationMeshType::CUBIC`
  计算真实的材料可行步长
- 这次 ownership refactor 保持了同样的算法语义，但把实现分层为：
  - `pgoLogging`
    - 统一 logger `WARN`
  - `basicAlgorithms`
    - `CubicPolynomial`
    - critical-point isolation
    - bracketed `bisection`
  - `solidDeformationModel/materialMaxStepPolynomialUtils.*`
    - `det(A + alpha B) - eps` 的系数构造
    - 共享的 feasible-alpha helper
  - concrete deformation model
    - 单 element `computeLocalMaxStepSize(...)`
  - `DeformationModelAssembler`
    - local gather 与全局 `min`
  - `DeformationModelEnergy`
    - absolute-position 语义、`PotentialEnergy` 入口和 `materialClampCount_`
- `tet` 路径使用：
  - 当前 absolute positions 上的 `D_s(0)`
  - 沿 `dx` 的 `delta D_s`
  - `eps_det = 1e-8 * abs(det(D_m))`
  - `g(alpha) = det(D_s(alpha)) - eps_det`
- `cubic` 路径使用：
  - integration point 上的 `F_q(0)`
  - 沿 `dx` 的 `delta F_q`
  - `eps_det = 1e-8`
  - `g_q(alpha) = det(F_q(alpha)) - eps_det`
- 三次系数固定按 `det(A + alpha B)` 的列多线性展开直接构造；
  当前实现没有使用 sampling + Vandermonde
- root finding 已按 plan 收成：
  - `g'(alpha)` root isolation
  - 单调子区间扫描
  - 对最早 bracketed root 用保守 `bisection` refine
  - 最终乘 `kMaxStepInteriorSafety = 0.99`
  - 下限 clamp 到 `kMaxStepMinClamp = 1e-12`
- `phi(0) <= eps` 的非法初值路径已落地：
  - 返回恢复性极小正数
  - 对每次非法初值都发出 `WARN`
  - warning payload 至少包含 mesh type、element id、`phi(0)`、`eps`
- `DeformationModelEnergy` 现已暴露只读观测接口
  `getMaterialClampCount() const`
  并用 `mutable std::atomic<int64_t>` 统计 `< 1.0` 的材料缩步次数
- `DeformationModelEnergy` 现已同时暴露：
  - `getMinMaterialFeasibleAlphaThisSolve() const`
  - `resetMaterialMaxStepStats()`
  并在每次 `computeMaxStepSize(...)` 调用时记录当前 solve 中最小的 `alpha_material`
- `DeformationModelEnergy` 现已支持 runtime toggle：
  - `setEnableMaterialMaxStep(bool)`
  - `isMaterialMaxStepEnabled() const`
  - 默认仍为 `true`
  - 当 config 显式给出 `enable-material-max-step=false` 时，材料侧 `computeMaxStepSize(...)` 直接返回 `1.0`
    且不递增 `materialClampCount_`
- `runSim` 与 `runIPCSim` 现都接受同一个 JSON 字段：
  - `enable-material-max-step`
  - 默认 `true`
  - `false` 时仅关闭材料 inversion-free max-step clamp，不影响 contact max step
- `SimulationMeshType::SHELL` 继续显式返回 `1.0`
- `SimulationMeshType::TRIANGLE` / `EDGE_QUAD` 在代码路径中也继续显式返回 `1.0`
- IPC contact 路径现已新增对称的 max-step 观测：
  - `SurfaceIPCCore::getContactClampCount() const`
  - `SurfaceIPCCore::getMinContactFeasibleAlphaThisSolve() const`
  - `SurfaceIPCCore::resetContactMaxStepStats() const`
  - `CIPCPotentialEnergy` / `EmbeddedSurfaceIPCPotentialEnergy` 透传这些 getter/reset
- integrator 聚合结果现已可读：
  - `ImplicitBackwardEulerEnergy::getMinFeasibleAlphaThisSolve() const`
  - `ImplicitBackwardEulerEnergy::getMinLineSearchAlphaThisSolve() const`
  - `ImplicitBackwardEulerEnergy::getMinEffectiveAlphaThisSolve() const`
  - `TRBDF2TimeIntegratorEnergy::getMinFeasibleAlphaThisSolve() const`
  - `TRBDF2TimeIntegratorEnergy::getMinLineSearchAlphaThisSolve() const`
  - `TRBDF2TimeIntegratorEnergy::getMinEffectiveAlphaThisSolve() const`
- `runSim` / `runIPCSim` 现已支持 config 字段 `loglevel`，默认 `trace`，并按以下分级输出 max-step 观测：
  - `trace`
    - 每次 `alpha_material < 1` / `alpha_contact < 1` 都打印 component-level clamp 日志
    - 并保留每个 timestep 的 info-level summary
  - `debug` / `info`
    - 每个 timestep 打 summary：
      `materialClampCount`
      `minMaterialFeasibleAlphaThisSolve`
      `minContactFeasibleAlphaThisSolve`（IPC）
      `minFeasibleAlphaThisSolve`
      `minLineSearchAlphaThisSolve`
      `minEffectiveAlphaThisSolve`
  - `warn`
    - 不打 summary，只保留 abnormal warnings
- warning 语义现已收口为：
  - `material`
    - `illegal initial state` 始终 `WARN`
    - 非 illegal 且 `0 < alpha_material < 0.01` 时 `WARN`
  - `contact`
    - `0 < alpha_contact < 0.01` 时 `WARN`

## Code Shape

实现后的 `Phase 1.5` 代码边界固定为：

- `/Users/jinceyang/Desktop/codebase/libpgo/src/core/solidDeformationModel/deformationModelEnergy.h`
  - `computeMaxStepSize(...)`
  - `getMaterialClampCount() const`
- `/Users/jinceyang/Desktop/codebase/libpgo/src/core/pgoLogging/pgoLogging.h/.cpp`
  - 统一 logger `WARN`
- `/Users/jinceyang/Desktop/codebase/libpgo/src/core/basicAlgorithms/polynomialRootUtils.h/.cpp`
  - `CubicPolynomial`
  - `findCriticalPointsInUnitInterval(...)`
  - `findFirstBoundaryRootByBisection(...)`
- `/Users/jinceyang/Desktop/codebase/libpgo/src/core/solidDeformationModel/materialMaxStepPolynomialUtils.h/.cpp`
  - `buildDeterminantCubicFromAffineMatrixPath(...)`
  - `findConservativeFeasibleAlpha(...)`
- `/Users/jinceyang/Desktop/codebase/libpgo/src/core/solidDeformationModel/tetMeshDeformationModel.h/.cpp`
  - `computeLocalMaxStepSize(...)`
- `/Users/jinceyang/Desktop/codebase/libpgo/src/core/solidDeformationModel/cubicMeshDeformationModel.h/.cpp`
  - `computeLocalMaxStepSize(...)`
- `/Users/jinceyang/Desktop/codebase/libpgo/src/core/solidDeformationModel/koiterDeformationModel.h/.cpp`
  - explicit shell defer via `computeLocalMaxStepSize(...) == 1.0`
- `/Users/jinceyang/Desktop/codebase/libpgo/src/core/solidDeformationModel/deformationModelAssembler.h/.cpp`
  - `computeMaxStepSize(...)`
- `/Users/jinceyang/Desktop/codebase/libpgo/src/core/solidDeformationModel/deformationModelEnergy.cpp`
  - `assembleAbsolutePositions(...)`
  - `assembleDirectionSlice(...)`
  - thin `computeMaxStepSize(...)` forwarding to assembler

integrator 侧没有新增 special-case：

- `ImplicitBackwardEulerEnergy::computeMaxStepSize()`
- `TRBDF2TimeIntegratorEnergy::computeMaxStepSize()`

继续按已有逻辑对 implicit models 取最小值；`Phase 1.5` 只是让
`DeformationModelEnergy` 终于提供了非平凡的 material max step。

## Tests And Validation

本批新增并通过的测试：

- `/Users/jinceyang/Desktop/codebase/libpgo/tests/src/core/basicAlgorithms/polynomialRootUtils_gtest.cpp`
- `/Users/jinceyang/Desktop/codebase/libpgo/tests/src/core/solidDeformationModel/deformationModelEnergyMaxStep_gtest.cpp`

覆盖内容包括：

- `deg = 0/1/2/3` 的 polynomial root helper
- 无 critical point / endpoint root / monotone interval + bisection refine
- `tet` 单单元缩步
- `tet` 材料 max-step disable 后返回 `1.0`
- `tet` 非法初值 warning + recovery clamp
- `tet` 多 element 取最小值
- `tet` 纯刚体平移 `deg == 0` 返回 `1.0`
- `cubic` 单 hex 缩步
- `cubic` 明显可行步返回 `1.0`
- `cubic` 多 element / integration point 取最小值
- `shell` 继续返回 `1.0`
- `runSimFEMSetup` 能把 disabled material max-step 传到 runtime
- `runIPCSim` volume setup 能把 `enable-material-max-step=false` 传到 runtime
- `runSimCliLogging::resolveConfiguredLogLevel(...)` 默认回落到 `trace`，并能正确解析 `trace / warn`
- `SurfaceIPCCore` 的 `contactClampCount / minContactFeasibleAlphaThisSolve / reset` 与 small-alpha warning
- `runIPCSim` 在 `info / debug / warn` 三个 `loglevel` 下的 summary 分级行为
- `ImplicitBackwardEulerEnergy::computeMaxStepSize()` 返回 `min(material, other)`
- `TRBDF2TimeIntegratorEnergy::computeMaxStepSize()` 返回 `min(material, other)`

同时补跑了现有 `runIPCSim` volume smoke：

- `RunIPCSimCliGTest.TetOneTimestepSmokeWritesDeformAndRet`
- `RunIPCSimCliGTest.CubicOneTimestepSmokeWritesDeformAndRet`

最终本地验证命令：

```bash
cmake --preset base_no_mkl_debug
cmake --build --preset base_no_mkl_debug --target deformationModelEnergyMaxStep_gtest surfaceIPCCore_gtest runSim_gtest runIPCSim_gtest
ctest --test-dir build/base_no_mkl_debug --output-on-failure -R "(DeformationModelEnergyMaxStep|SurfaceIPCCoreGTest|RunSimCliLoggingGTest|RunIPCSimCliGTest)"
```

本地验证结果：通过。

## Intentional Drift From Source Plan

相对 source plan，这次实现有两条实现期确认后的收窄：

- source plan 希望把 `TRIANGLE` / `EDGE_QUAD` 的 `1.0` 行为也做成 runtime negative test；
  当前 deformation-model manager / assembler 栈并不能为这两类 mesh type 正常实例化 `DeformationModelEnergy`
  所需的 FEM model，所以本批只对 `SHELL` 做了 runtime 负测试。
  `TRIANGLE` / `EDGE_QUAD` 继续通过显式代码分支保持 `1.0`，并在此处把这个限制记成 repo truth。
- source plan 早期版本只要求 `materialClampCount_` 或等价 summary 能观测到材料缩步；
  当前 repo truth 已进一步落成对称的 material/contact/solve-summary 观测，
  并通过 config 字段 `loglevel` 暴露分级日志。
- source plan 的早期版本假设 helper 会长期留在 `deformationModelEnergy.cpp`；
  当前 repo truth 已改成 logging / math / model / assembler / energy 分层。
  这是 behavior-preserving refactor，不是算法升级；`0.99 / 1e-12 / 1e-8`、
  illegal-initial-state 恢复语义和 integrator 聚合结果保持不变。
