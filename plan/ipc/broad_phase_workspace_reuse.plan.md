# Broad Phase Prepared Active Pairs 中文优化计划

> **给自动化实现者:** 实现本计划时使用 `superpowers:subagent-driven-development` 或 `superpowers:executing-plans`。所有任务用 checkbox (`- [ ]`) 跟踪。

**目标:** 对同一个 surface positions `x_surf`，只构建一次 IPC self-contact active PT/EE pairs，然后让 energy / gradient / hessian 复用这份 prepared active pairs，避免重复 build/query spatial hash。

**架构:** `SurfaceIPCCore` 增加显式 prepared state：`prepareForSurfacePositions(x_surf)` 负责保存 positions 并构建 active pairs；`computeEnergyWithPreparedPairs()` / `computeGradientWithPreparedPairs()` / `computeHessianWithPreparedPairs()` 消费 prepared state，不再接收 `x_surf`。wrapper 层增加 exact cache：只有 surface positions 逐元素完全相等时复用 prepared state。

**技术栈:** C++17, Eigen, TBB, GoogleTest, 现有 `src/core/contact` CMake target。

---

## 设计前提

- 默认 `dhat` 在运行时不变；它是 mesh/core 配置期参数。
- `kappa` / `eps_ee` 不影响 active pair 集合，只影响 barrier 数值。
- `slackness` 只影响 max-step，不影响 active pair 集合。
- `setMesh()` 必须 invalidate prepared state。
- `setParameters()` 可以保守 invalidate prepared state；这不是热路径。
- prepared state 只针对 self-contact static active pairs，不覆盖 `computeMaxStepSize()` 的 swept broad phase。
- 不做 temporal coherence，不做近似相等，不做 hash-based state key。
- 不改变 PT/EE pair ordering，不改变 broad phase filtering，不改变 barrier / CCD 数学。

## 当前重复路径

现在 `SurfaceIPCCore` 的三个 public compute API 都会重新 build active pairs：

- `computeEnergy(x)` -> `findCollisionPairs(x)` -> static broad phase build/query
- `computeGradient(x)` -> `findCollisionPairs(x)` -> static broad phase build/query
- `computeHessian(x)` -> `findCollisionPairs(x)` -> static broad phase build/query

因此同一个 Newton state 上，如果 solver 依次调用：

```cpp
func(x);
gradient(x);
hessianDirect(x);
```

就会对同一个 `x_surf` 重复三次：

- vertex / triangle / edge AABB build
- triangle hash insert + vertex query
- edge hash insert + edge query
- PT/EE distance threshold filtering

本计划优先消除这类重复逻辑工作。之前讨论的 `SpatialHashGrid` workspace/capacity reuse 只减少分配开销，不能减少 query 次数；它放到后续阶段。

## 文件范围

主要修改：

- `src/core/contact/ipc/core/surfaceIPCCore.h`
- `src/core/contact/ipc/core/surfaceIPCCore.cpp`
- `src/core/contact/CIPC.h`
- `src/core/contact/CIPC.cpp`
- `src/core/contact/embeddedSurfaceIPCPotentialEnergy.h`
- `src/core/contact/embeddedSurfaceIPCPotentialEnergy.cpp`
- `src/core/contact/mappedSurfacePotentialEnergy.h`
- `src/core/contact/mappedSurfacePotentialEnergy.cpp`

测试修改：

- `tests/src/core/contact/surfaceIPCCore_gtest.cpp`
- `tests/src/core/contact/cipcPotentialEnergy_gtest.cpp`
- `tests/src/core/contact/embeddedSurfaceIPCPotentialEnergy_gtest.cpp`

不作为本计划主线修改：

- `src/core/contact/ipc/broadPhase/spatialHashGrid.h/.cpp`
- `src/core/contact/ipc/broadPhase/surfaceIPCSelfBroadPhase.h/.cpp`
- `src/core/contact/ipc/core/surfaceIPCMaxStep.h/.cpp`

## 目标 API

在 `SurfaceIPCCore` 中新增：

```cpp
void prepareForSurfacePositions(EigenSupport::ConstRefVecXd x_surf) const;
bool isPreparedFor(EigenSupport::ConstRefVecXd x_surf) const;
void invalidatePreparedState() const;

double computeEnergyWithPreparedPairs() const;
void computeGradientWithPreparedPairs(EigenSupport::RefVecXd g_surf) const;
void computeHessianWithPreparedPairs(EigenSupport::SpMatD &H_surf) const;
void computeAllWithPreparedPairs(double &energy, VXd &g_surf, SpMatD &H_surf) const;
```

保留旧 API：

```cpp
double computeEnergy(EigenSupport::ConstRefVecXd x_surf) const;
void computeGradient(EigenSupport::ConstRefVecXd x_surf, EigenSupport::RefVecXd g_surf) const;
void computeHessian(EigenSupport::ConstRefVecXd x_surf, EigenSupport::SpMatD &H_surf) const;
void computeAll(EigenSupport::ConstRefVecXd x_surf, double &energy, VXd &g_surf, SpMatD &H_surf) const;
```

旧 API 改为 convenience wrapper：

```cpp
double SurfaceIPCCore::computeEnergy(EigenSupport::ConstRefVecXd x_surf) const
{
  prepareForSurfacePositions(x_surf);
  return computeEnergyWithPreparedPairs();
}
```

`computeGradient()` / `computeHessian()` / `computeAll()` 同理。

## Prepared State 设计

在 `SurfaceIPCCore` 私有成员中新增：

```cpp
mutable bool hasPreparedState_ = false;
mutable EigenSupport::VXd preparedPositions_;
```

现有成员继续作为 prepared active pair 输出：

```cpp
mutable std::vector<PTPair> ptPairs_;
mutable std::vector<EEPair> eePairs_;
```

`prepareForSurfacePositions(x_surf)` 的职责：

```cpp
preparedPositions_ = x_surf;
findCollisionPairs(preparedPositions_);
hasPreparedState_ = true;
```

`isPreparedFor(x_surf)` 必须做精确比较：

```cpp
return hasPreparedState_ &&
  preparedPositions_.size() == x_surf.size() &&
  (preparedPositions_.array() == x_surf.array()).all();
```

`invalidatePreparedState()` 的职责：

```cpp
hasPreparedState_ = false;
preparedPositions_.resize(0);
ptPairs_.clear();
eePairs_.clear();
```

`compute*WithPreparedPairs()` 必须先检查 prepared state。建议使用内部 helper：

```cpp
void requirePreparedState() const;
```

第一版可在未 prepared 时抛出 `std::logic_error`，错误信息明确：

```cpp
"SurfaceIPCCore prepared active pairs are missing. Call prepareForSurfacePositions() first."
```

## Wrapper 精确缓存

### Legacy wrapper: `CIPCPotentialEnergy`

新增私有 mutable cache：

```cpp
mutable EigenSupport::VXd cachedSurfacePositions_;
mutable bool hasCachedSurfacePositions_ = false;
```

新增 helper：

```cpp
void ensurePreparedForSurfacePositions(const VXd &x_surf) const;
void invalidatePreparedSurfaceCache() const;
```

`ensurePreparedForSurfacePositions(x_surf)`：

```cpp
if (!hasCachedSurfacePositions_ ||
    cachedSurfacePositions_.size() != x_surf.size() ||
    !(cachedSurfacePositions_.array() == x_surf.array()).all()) {
  core.prepareForSurfacePositions(x_surf);
  cachedSurfacePositions_ = x_surf;
  hasCachedSurfacePositions_ = true;
  return;
}

if (!core.isPreparedFor(x_surf))
  core.prepareForSurfacePositions(x_surf);
```

`func()`：

```cpp
syncCoreParametersFromWrapper();
const VXd x_surf = toSurfacePositions(x);
ensurePreparedForSurfacePositions(x_surf);
return core.computeEnergyWithPreparedPairs() + computeFloorEnergy(x_surf);
```

`gradient()`：

```cpp
syncCoreParametersFromWrapper();
const VXd x_surf = toSurfacePositions(x);
ensurePreparedForSurfacePositions(x_surf);
core.computeGradientWithPreparedPairs(grad);
addFloorGradient(x_surf, grad);
```

`hessianDirect()`：

```cpp
syncCoreParametersFromWrapper();
const VXd x_surf = toSurfacePositions(x);
ensurePreparedForSurfacePositions(x_surf);
core.computeHessianWithPreparedPairs(hess);
addFloorHessian(x_surf, hess);
```

`computeMaxStepSize()` 不使用 prepared active pairs，因为它需要 swept broad phase：

```cpp
return core.computeMaxStepSize(x_surf, dx_surf);
```

### `EmbeddedSurfaceIPCPotentialEnergy`

新增私有 mutable cache：

```cpp
mutable EigenSupport::VXd cachedSurfacePositions_;
mutable bool hasCachedSurfacePositions_ = false;
```

新增 helper：

```cpp
void ensurePreparedForSurfacePositions(EigenSupport::ConstRefVecXd surfacePositions) const;
void invalidatePreparedSurfaceCache() const;
```

`computeSurfaceEnergy()`：

```cpp
ensurePreparedForSurfacePositions(surfacePositions);
return surfaceIPCCore_.computeEnergyWithPreparedPairs();
```

`computeSurfaceGradient()`：

```cpp
ensurePreparedForSurfacePositions(surfacePositions);
surfaceIPCCore_.computeGradientWithPreparedPairs(surfaceGradient);
```

`computeSurfaceHessian()`：

```cpp
ensurePreparedForSurfacePositions(surfacePositions);
surfaceIPCCore_.computeHessianWithPreparedPairs(surfaceHessian);
```

`computeSurfaceMaxStepSize()` 保持原样，不走 prepared active pairs。

## 任务 0：优化前 baseline profiling

**文件：**

- 记录：`plan/ipc/broad_phase_prepared_pairs.baseline.md`
- 不修改源码。

- [ ] **步骤 1：选择 baseline case**

至少选择一个稳定、可重复的 IPC case。优先使用：

- `examples/ipc/cubic/box-with-sphere`
- 或现有 `runIPCSim_gtest` 中会触发 IPC self-contact 的 case
- 或一个能稳定触发 PT/EE active pairs 的 `surfaceIPCCore_gtest` / wrapper gtest

记录 case 名称、运行命令、输入配置路径、当前 git commit/hash 或工作树状态。

- [ ] **步骤 2：开启现有 profiling / logging**

使用 repo 当前已有 profiling 开关或 runtime logging 开关，不新增 benchmark target。

需要记录的 section 至少包括：

- `contact.surface.pair_build.static`
- `contact.surface.energy`
- `contact.surface.gradient`
- `contact.surface.hessian`
- `contact.adapter.func`
- `contact.adapter.gradient`
- `contact.adapter.hessian_direct`

如果 profiling 输出已经包含调用次数，记录调用次数；如果只输出耗时，就记录同一 step / 同一 Newton state 中 `pair_build.static` 出现次数。

- [ ] **步骤 3：写 baseline 记录**

新增 `plan/ipc/broad_phase_prepared_pairs.baseline.md`，格式如下：

```markdown
# Broad Phase Prepared Pairs Baseline

## Case

- Command:
- Input/config:
- Date:
- Git status summary:

## Baseline Observation

- `contact.surface.pair_build.static` count:
- `contact.surface.pair_build.static` total time:
- `contact.surface.energy` time:
- `contact.surface.gradient` time:
- `contact.surface.hessian` time:
- `contact.adapter.func` time:
- `contact.adapter.gradient` time:
- `contact.adapter.hessian_direct` time:

## Notes

- Same-state `func/gradient/hessianDirect` repeated pair build observed: yes/no
- Pair counts if visible:
- Caveats:
```

- [ ] **步骤 4：确认 baseline 不阻塞实现**

如果当前 profiling 输出不足以稳定得到所有字段，不为此扩展 profiling 框架。记录“缺失字段”和原因，然后继续任务 1。

## 任务 1：`SurfaceIPCCore` Prepared API

**文件：**

- 修改：`src/core/contact/ipc/core/surfaceIPCCore.h`
- 修改：`src/core/contact/ipc/core/surfaceIPCCore.cpp`
- 修改：`tests/src/core/contact/surfaceIPCCore_gtest.cpp`

- [ ] **步骤 1：添加先失败的测试**

在 `surfaceIPCCore_gtest.cpp` 中新增两个测试。

第一个测试验证 prepared API 与旧 API 数值一致：

```cpp
TEST(SurfaceIPCCoreGTest, PreparedPairsMatchDirectEnergyGradientHessian)
{
  SurfaceIPCCore core;
  ES::MXd V;
  ES::MXi F;
  makeTwoTriangleMesh(V, F);
  core.setMesh(V, F);

  const ES::VXd x = makeTwoTrianglePositions();

  const double directEnergy = core.computeEnergy(x);
  ES::VXd directGradient = ES::VXd::Zero(x.size());
  core.computeGradient(x, directGradient);
  ES::SpMatD directHessian;
  core.computeHessian(x, directHessian);

  core.prepareForSurfacePositions(x);
  EXPECT_TRUE(core.isPreparedFor(x));

  const double preparedEnergy = core.computeEnergyWithPreparedPairs();
  ES::VXd preparedGradient = ES::VXd::Zero(x.size());
  core.computeGradientWithPreparedPairs(preparedGradient);
  ES::SpMatD preparedHessian;
  core.computeHessianWithPreparedPairs(preparedHessian);

  EXPECT_NEAR(preparedEnergy, directEnergy, 1e-12);
  EXPECT_EQ(preparedGradient.size(), directGradient.size());
  for (int i = 0; i < preparedGradient.size(); ++i)
    EXPECT_NEAR(preparedGradient[i], directGradient[i], 1e-10);
  EXPECT_EQ(preparedHessian.rows(), directHessian.rows());
  EXPECT_EQ(preparedHessian.cols(), directHessian.cols());
  EXPECT_EQ(preparedHessian.nonZeros(), directHessian.nonZeros());
}
```

第二个测试验证未 prepared 时 consumer 抛错：

```cpp
TEST(SurfaceIPCCoreGTest, PreparedPairConsumersRequirePreparedState)
{
  SurfaceIPCCore core;
  ES::MXd V;
  ES::MXi F;
  makeTwoTriangleMesh(V, F);
  core.setMesh(V, F);

  EXPECT_THROW(core.computeEnergyWithPreparedPairs(), std::logic_error);
}
```

如果当前测试文件没有 `makeTwoTriangleMesh()` / `makeTwoTrianglePositions()`，就复用已有 fixture helper；不要新增复杂 mesh。

- [ ] **步骤 2：运行先失败的测试**

```bash
cmake --build build/base_no_mkl --target surfaceIPCCore_gtest -j2
./build/base_no_mkl/tests/src/core/contact/surfaceIPCCore_gtest
```

预期：编译失败，因为 prepared API 还不存在。

- [ ] **步骤 3：添加 header API 和成员**

在 `surfaceIPCCore.h` public 区域添加：

```cpp
void prepareForSurfacePositions(EigenSupport::ConstRefVecXd x_surf) const;
bool isPreparedFor(EigenSupport::ConstRefVecXd x_surf) const;
void invalidatePreparedState() const;

double computeEnergyWithPreparedPairs() const;
void computeGradientWithPreparedPairs(EigenSupport::RefVecXd g_surf) const;
void computeHessianWithPreparedPairs(EigenSupport::SpMatD &H_surf) const;
void computeAllWithPreparedPairs(double &energy, VXd &g_surf, SpMatD &H_surf) const;
```

在 private 区域添加：

```cpp
void requirePreparedState() const;

mutable bool hasPreparedState_ = false;
mutable VXd preparedPositions_;
```

- [ ] **步骤 4：实现 prepared state 生命周期**

在 `surfaceIPCCore.cpp` 中实现：

```cpp
void SurfaceIPCCore::invalidatePreparedState() const
{
  hasPreparedState_ = false;
  preparedPositions_.resize(0);
  ptPairs_.clear();
  eePairs_.clear();
}

bool SurfaceIPCCore::isPreparedFor(EigenSupport::ConstRefVecXd x_surf) const
{
  return hasPreparedState_ &&
    preparedPositions_.size() == x_surf.size() &&
    (preparedPositions_.array() == x_surf.array()).all();
}

void SurfaceIPCCore::prepareForSurfacePositions(EigenSupport::ConstRefVecXd x_surf) const
{
  preparedPositions_ = x_surf;
  findCollisionPairs(preparedPositions_);
  hasPreparedState_ = true;
}

void SurfaceIPCCore::requirePreparedState() const
{
  if (!hasPreparedState_)
    throw std::logic_error("SurfaceIPCCore prepared active pairs are missing. Call prepareForSurfacePositions() first.");
}
```

在 `surfaceIPCCore.cpp` 中 include：

```cpp
#include <stdexcept>
```

`setMesh()` 末尾调用：

```cpp
invalidatePreparedState();
```

`setParameters()` 末尾保守调用：

```cpp
invalidatePreparedState();
```

- [ ] **步骤 5：实现 prepared consumers**

把原来的 assembly 调用拆到 prepared consumers：

```cpp
double SurfaceIPCCore::computeEnergyWithPreparedPairs() const
{
  requirePreparedState();
  return SurfaceIPCBarrierAssembler().computeEnergy(
    preparedPositions_, ptPairs_, eePairs_, topology_.numVerts, dhat, kappa, eps_ee);
}
```

gradient / hessian / computeAll 同理，全部使用 `preparedPositions_`。

旧 API 改成：

```cpp
double SurfaceIPCCore::computeEnergy(EigenSupport::ConstRefVecXd x_surf) const
{
  Profiling::ScopedProfileSection scopedProfile(SurfaceIPCProfileSections::kEnergy);
  prepareForSurfacePositions(x_surf);
  return computeEnergyWithPreparedPairs();
}
```

`computeGradient()` / `computeHessian()` / `computeAll()` 同理。

- [ ] **步骤 6：运行 core tests**

```bash
cmake --build build/base_no_mkl --target surfaceIPCCore_gtest surfaceIPCSelfBroadPhase_gtest surfaceIPCBarrierAssembler_gtest -j2
./build/base_no_mkl/tests/src/core/contact/surfaceIPCCore_gtest
./build/base_no_mkl/tests/src/core/contact/surfaceIPCSelfBroadPhase_gtest
./build/base_no_mkl/tests/src/core/contact/surfaceIPCBarrierAssembler_gtest
```

预期：全部通过。

## 任务 2：Legacy `CIPCPotentialEnergy` 精确缓存

**文件：**

- 修改：`src/core/contact/CIPC.h`
- 修改：`src/core/contact/CIPC.cpp`
- 修改：`tests/src/core/contact/cipcPotentialEnergy_gtest.cpp`

- [ ] **步骤 1：添加 wrapper 回归测试**

在 `cipcPotentialEnergy_gtest.cpp` 中新增测试，验证连续 `func/gradient/hessianDirect` 和旧 core direct 数值一致：

```cpp
TEST(CIPCPotentialEnergyGTest, ReusesPreparedPairsAcrossEnergyGradientHessianForSameState)
{
  CIPCPotentialEnergy wrapper(0.1, 0.1, false);
  ES::MXd V;
  ES::MXi F;
  makeTwoTriangleMesh(V, F);
  wrapper.setMesh(V, F);
  const ES::VXd x = makeTwoTrianglePositions();

  const double energy0 = wrapper.func(x);
  ES::VXd gradient0 = ES::VXd::Zero(x.size());
  wrapper.gradient(x, gradient0);
  ES::SpMatD hessian0;
  wrapper.hessianDirect(x, hessian0);

  const double energy1 = wrapper.func(x);
  ES::VXd gradient1 = ES::VXd::Zero(x.size());
  wrapper.gradient(x, gradient1);
  ES::SpMatD hessian1;
  wrapper.hessianDirect(x, hessian1);

  EXPECT_NEAR(energy1, energy0, 1e-12);
  for (int i = 0; i < gradient0.size(); ++i)
    EXPECT_NEAR(gradient1[i], gradient0[i], 1e-10);
  EXPECT_EQ(hessian1.rows(), hessian0.rows());
  EXPECT_EQ(hessian1.cols(), hessian0.cols());
  EXPECT_EQ(hessian1.nonZeros(), hessian0.nonZeros());
}
```

- [ ] **步骤 2：添加 cache 成员与 helper 声明**

在 `CIPC.h` private 区域添加：

```cpp
void ensurePreparedForSurfacePositions(const VXd &x_surf) const;
void invalidatePreparedSurfaceCache() const;

mutable VXd cachedSurfacePositions_;
mutable bool hasCachedSurfacePositions_ = false;
```

- [ ] **步骤 3：实现精确缓存 helper**

在 `CIPC.cpp` 中实现：

```cpp
void CIPCPotentialEnergy::invalidatePreparedSurfaceCache() const
{
  hasCachedSurfacePositions_ = false;
  cachedSurfacePositions_.resize(0);
  core.invalidatePreparedState();
}

void CIPCPotentialEnergy::ensurePreparedForSurfacePositions(const VXd &x_surf) const
{
  const bool cacheHit = hasCachedSurfacePositions_ &&
    cachedSurfacePositions_.size() == x_surf.size() &&
    (cachedSurfacePositions_.array() == x_surf.array()).all();

  if (!cacheHit || !core.isPreparedFor(x_surf)) {
    core.prepareForSurfacePositions(x_surf);
    cachedSurfacePositions_ = x_surf;
    hasCachedSurfacePositions_ = true;
  }
}
```

`setMesh()` 末尾调用：

```cpp
invalidatePreparedSurfaceCache();
```

`syncCoreParametersFromWrapper()` 在 `core.setParameters(params);` 后调用：

```cpp
hasCachedSurfacePositions_ = false;
```

说明：`setParameters()` 会使 core prepared state invalid；wrapper cache 标记也要失效。

- [ ] **步骤 4：改 `func/gradient/hessianDirect` 消费 prepared pairs**

按本计划前文的 wrapper 精确缓存代码替换三处 direct compute 调用。

- [ ] **步骤 5：运行 legacy wrapper tests**

```bash
cmake --build build/base_no_mkl --target cipcPotentialEnergy_gtest surfaceIPCCore_gtest -j2
./build/base_no_mkl/tests/src/core/contact/cipcPotentialEnergy_gtest
./build/base_no_mkl/tests/src/core/contact/surfaceIPCCore_gtest
```

预期：全部通过。

## 任务 3：Embedded Adapter 精确缓存

**文件：**

- 修改：`src/core/contact/embeddedSurfaceIPCPotentialEnergy.h`
- 修改：`src/core/contact/embeddedSurfaceIPCPotentialEnergy.cpp`
- 修改：`tests/src/core/contact/embeddedSurfaceIPCPotentialEnergy_gtest.cpp`

- [ ] **步骤 1：添加 embedded adapter 回归测试**

在 `embeddedSurfaceIPCPotentialEnergy_gtest.cpp` 中新增测试，验证同一个 simulation displacement 下连续 energy/gradient/hessian 结果稳定：

```cpp
TEST(EmbeddedSurfaceIPCPotentialEnergyGTest, ReusesPreparedPairsAcrossEnergyGradientHessianForSameState)
{
  ES::MXd surfaceRestVertices;
  ES::MXi surfaceTriangles;
  ES::SpMatD map;
  makeEmbeddedTwoTriangleFixture(surfaceRestVertices, surfaceTriangles, map);

  EmbeddedSurfaceIPCPotentialEnergy adapter(surfaceRestVertices, surfaceTriangles, map);
  ES::VXd u = ES::VXd::Zero(map.cols());

  const double energy0 = adapter.func(u);
  ES::VXd gradient0 = ES::VXd::Zero(map.cols());
  adapter.gradient(u, gradient0);
  ES::SpMatD hessian0;
  adapter.hessianDirect(u, hessian0);

  const double energy1 = adapter.func(u);
  ES::VXd gradient1 = ES::VXd::Zero(map.cols());
  adapter.gradient(u, gradient1);
  ES::SpMatD hessian1;
  adapter.hessianDirect(u, hessian1);

  EXPECT_NEAR(energy1, energy0, 1e-12);
  for (int i = 0; i < gradient0.size(); ++i)
    EXPECT_NEAR(gradient1[i], gradient0[i], 1e-10);
  EXPECT_EQ(hessian1.rows(), hessian0.rows());
  EXPECT_EQ(hessian1.cols(), hessian0.cols());
}
```

若现有测试文件没有 `makeEmbeddedTwoTriangleFixture()`，复用当前已有 fixture 构造函数，不新增大型场景。

- [ ] **步骤 2：添加 cache 成员与 helper 声明**

在 `embeddedSurfaceIPCPotentialEnergy.h` private 区域添加：

```cpp
void ensurePreparedForSurfacePositions(EigenSupport::ConstRefVecXd surfacePositions) const;
void invalidatePreparedSurfaceCache() const;

mutable EigenSupport::VXd cachedSurfacePositions_;
mutable bool hasCachedSurfacePositions_ = false;
```

- [ ] **步骤 3：实现 embedded 精确缓存 helper**

在 `embeddedSurfaceIPCPotentialEnergy.cpp` 中实现：

```cpp
void EmbeddedSurfaceIPCPotentialEnergy::invalidatePreparedSurfaceCache() const
{
  hasCachedSurfacePositions_ = false;
  cachedSurfacePositions_.resize(0);
  surfaceIPCCore_.invalidatePreparedState();
}

void EmbeddedSurfaceIPCPotentialEnergy::ensurePreparedForSurfacePositions(
  EigenSupport::ConstRefVecXd surfacePositions) const
{
  const bool cacheHit = hasCachedSurfacePositions_ &&
    cachedSurfacePositions_.size() == surfacePositions.size() &&
    (cachedSurfacePositions_.array() == surfacePositions.array()).all();

  if (!cacheHit || !surfaceIPCCore_.isPreparedFor(surfacePositions)) {
    surfaceIPCCore_.prepareForSurfacePositions(surfacePositions);
    cachedSurfacePositions_ = surfacePositions;
    hasCachedSurfacePositions_ = true;
  }
}
```

- [ ] **步骤 4：改 surface energy/gradient/hessian 消费 prepared pairs**

```cpp
double EmbeddedSurfaceIPCPotentialEnergy::computeSurfaceEnergy(
  EigenSupport::ConstRefVecXd surfacePositions) const
{
  ensurePreparedForSurfacePositions(surfacePositions);
  return surfaceIPCCore_.computeEnergyWithPreparedPairs();
}
```

gradient / hessian 同理。

- [ ] **步骤 5：运行 embedded tests**

```bash
cmake --build build/base_no_mkl --target embeddedSurfaceIPCPotentialEnergy_gtest embeddedSurfaceFloorPotentialEnergy_gtest -j2
./build/base_no_mkl/tests/src/core/contact/embeddedSurfaceIPCPotentialEnergy_gtest
./build/base_no_mkl/tests/src/core/contact/embeddedSurfaceFloorPotentialEnergy_gtest
```

预期：全部通过。

## 任务 4：Profiling 和可观测性

**文件：**

- 修改：`src/core/contact/ipc/profiling/surfaceIPCProfiling.h`
- 修改：`src/core/contact/ipc/core/surfaceIPCCore.cpp`

- [ ] **步骤 1：添加 prepared cache section 名称**

在 `surfaceIPCProfiling.h` 中新增：

```cpp
inline constexpr std::string_view kPrepareActivePairs = "contact.surface.prepare_active_pairs";
inline constexpr std::string_view kPreparedEnergy = "contact.surface.prepared_energy";
inline constexpr std::string_view kPreparedGradient = "contact.surface.prepared_gradient";
inline constexpr std::string_view kPreparedHessian = "contact.surface.prepared_hessian";
```

- [ ] **步骤 2：插桩 prepared API**

在 `prepareForSurfacePositions()` 中包：

```cpp
Profiling::ScopedProfileSection scopedProfile(SurfaceIPCProfileSections::kPrepareActivePairs);
```

在 prepared consumers 中分别包 `kPreparedEnergy` / `kPreparedGradient` / `kPreparedHessian`。

- [ ] **步骤 3：验证 profiling 关闭时行为不变**

```bash
cmake --build build/base_no_mkl --target surfaceIPCCore_gtest cipcProfiling_gtest -j2
./build/base_no_mkl/tests/src/core/contact/surfaceIPCCore_gtest
./build/base_no_mkl/tests/src/core/contact/cipcProfiling_gtest
```

预期：全部通过。

## 任务 5：全量回归

- [ ] **步骤 1：build IPC/contact tests**

```bash
cmake --build build/base_no_mkl --target ipcGeometry_gtest surfaceIPCTopology_gtest surfaceIPCSelfBroadPhase_gtest surfaceIPCMaxStep_gtest surfaceIPCBarrierAssembler_gtest spatialHashGrid_gtest surfaceIPCCore_gtest cipcProfiling_gtest cipcPotentialEnergy_gtest embeddedSurfaceIPCPotentialEnergy_gtest embeddedSurfaceFloorPotentialEnergy_gtest runIPCSim_gtest -j2
```

- [ ] **步骤 2：run tests**

```bash
./build/base_no_mkl/tests/src/core/contact/ipcGeometry_gtest
./build/base_no_mkl/tests/src/core/contact/surfaceIPCTopology_gtest
./build/base_no_mkl/tests/src/core/contact/surfaceIPCSelfBroadPhase_gtest
./build/base_no_mkl/tests/src/core/contact/surfaceIPCMaxStep_gtest
./build/base_no_mkl/tests/src/core/contact/surfaceIPCBarrierAssembler_gtest
./build/base_no_mkl/tests/src/core/contact/spatialHashGrid_gtest
./build/base_no_mkl/tests/src/core/contact/surfaceIPCCore_gtest
./build/base_no_mkl/tests/src/core/contact/cipcProfiling_gtest
./build/base_no_mkl/tests/src/core/contact/cipcPotentialEnergy_gtest
./build/base_no_mkl/tests/src/core/contact/embeddedSurfaceIPCPotentialEnergy_gtest
./build/base_no_mkl/tests/src/core/contact/embeddedSurfaceFloorPotentialEnergy_gtest
./build/base_no_mkl/tests/src/tools/runIPCSim_gtest
```

- [ ] **步骤 3：include hygiene 和 whitespace**

```bash
rg -n '#include "(surfaceIPCCore|surfaceIPCProfiling|spatialHashGrid|CIPC_autogen|CIPC_autogen_ll)\.h"|../../surfaceIPCCore' src tests
git diff --check
```

预期：`rg` 无输出；`git diff --check` 成功。

## 验收标准

- 同一个 `x_surf` 下，wrapper 连续 `func/gradient/hessianDirect` 只需要首次 prepare active pairs。
- `computeEnergyWithPreparedPairs()` / `computeGradientWithPreparedPairs()` / `computeHessianWithPreparedPairs()` 不接收 `x_surf` 参数。
- 未调用 `prepareForSurfacePositions()` 时，prepared consumers 明确报错。
- 旧 public API 行为保持可用。
- PT/EE active pair 集合和 ordering 不变。
- `computeMaxStepSize()` 仍走 swept broad phase，不复用 static prepared pairs。
- 所有 IPC/contact 回归测试通过。

## 推荐提交边界

1. `feat(ipc): add prepared active pair API`
2. `perf(ipc): reuse prepared pairs in legacy wrapper`
3. `perf(ipc): reuse prepared pairs in embedded adapter`
4. `test(ipc): cover prepared active pair reuse`
