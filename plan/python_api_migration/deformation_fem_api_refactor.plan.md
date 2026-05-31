# Deformation FEM API Refactor Plan

> **状态日期：** 2026-06-01
> **适用范围：** C++ `solidDeformationModel` API 重构 + Python `pypgo.fem` / `pypgo.energy` deformation binding。  
> **执行约束：** 本计划只重构当前 tet P1 / hex trilinear / shell Koiter deformation 主链路并绑定到 Python；不在本计划内实现 tricubic Hermite FEM 数值内核。Hermite extension point (Task 9) 已推迟，当前 milestone 先 finalize 现有体系的 Python API。

## 目标

把当前 deformation energy 主链路从“mesh type 隐含 FEM formulation 和 DOF layout”改成显式三层：

```text
MeshTopology / SimulationMesh
  -> ElementFormulation
  -> DofLayout
  -> DeformationModelAssembler
  -> DeformationModelEnergy
```

第一阶段必须保持现有 tet/cubic 行为不变，同时把当前 cubic deformation 路径明确命名为 `hex_trilinear`。Python public API 不在 C++ 边界稳定前提前定稿；等 formulation、lifetime、DOF layout、elastic/plastic parameter layout/field 的 C++ 重构完成后，再统一确定 `pypgo.fem` / `pypgo.energy` 的最终表面，避免把过渡态发布成长期 API。模块边界固定为：FEM 构造语义全部放在 `pypgo.fem`，最终可求值/可组合的 energy 对象放在 `pypgo.energy`。

## 当前问题

当前代码已经把文件/几何 mesh 和 solver-ready deformation energy 分开，但还没有把 cell topology、element formulation、DOF layout 分开：

- `SimulationMeshType::CUBIC` 现在同时表示 8 顶点 hexahedral topology 和 8-node trilinear vertex-DOF formulation。
- `DeformationModelManager::initImpl` 同时创建 elastic material、plastic model、element deformation model，且按 `SimulationMeshType` 直接分派。
- `DeformationModelAssembler` 假设全局 DOF 为 `3 * numVertices`，局部 DOF 为 `numElementVertices * 3`，gather/scatter 全部通过 vertex id。
- `DeformationModelAssembler` 同时假设 elastic/plastic parameters 是 per-element constant array，offset 由 `ele * numParams + j` 硬编码，无法自然扩展到 quadrature/nodal/external field 或 parameter optimization。
- Python 目前有 `pypgo.mesh.veg.VolumeMesh` 和 `pypgo.sim.SimulationMesh`，但 deformation energy 还没有作为 Python-first API 暴露。
- Vega `VolumetricMesh::Material`、`SimulationMeshMaterial`、`ElasticModel` 仍是三套表达；代码里还把 material payload、本构 law、Hill 这类 active add-on 混在同一个 elastic enum 里。

相关文件：

- `src/core/solidDeformationModel/simulationMesh.h`
- `src/core/solidDeformationModel/simulationMesh.cpp`
- `src/core/solidDeformationModel/deformationModelFactory.h`
- `src/core/solidDeformationModel/deformationModelFactory.cpp`
- `src/core/solidDeformationModel/deformationModelManager.h`
- `src/core/solidDeformationModel/deformationModelManager.cpp`
- `src/core/solidDeformationModel/deformationModelAssembler.h`
- `src/core/solidDeformationModel/deformationModelAssembler.cpp`
- `src/python/pypgo/bindings/mesh_bindings.cpp`
- `pypgo/sim.py`

## 非目标

- 不在本计划中实现 `HexTricubicHermite` 元素、Hermite basis、Hermite quadrature 或 Hermite sparse assembly。
- 不改变现有 tet P1、cubic trilinear、shell Koiter 数值行为。
- 不把 Hill 当作普通 standalone passive material；Hill 在本计划中作为 `base elastic + active fiber term` 的组合项进入 API。
- 不在没有 solver-side `ElasticModel` 和导数测试的情况下把 Orthotropic 标记为 supported；本计划会把 Orthotropic 的 C++ deformation law 与测试作为一般材料承诺的一部分。
- 不迁移完整 `runIPCSim` loop；本计划只提供 solver-facing deformation energy building block。
- 不把 `DeformationModelManager`、`DeformationModelAssembler` 的内部 ownership 链直接暴露给 Python 用户。
- 不为旧 `makeDeformationModel(...)` auto-dispatch 入口做向后兼容；本计划允许一次性破坏式迁移到 topology-specific factory。

## 设计决策

### 1. Formulation 用 class hierarchy 表达，单一 factory 入口

保留现有 `SimulationMeshType::TET` / `CUBIC` / `SHELL` 作为内部 topology metadata。Formulation 使用 class hierarchy：`Formulation`（抽象基类）→ `VolumetricFormulation` / `ShellFormulation`（中间层）→ 具体 formulation 类。`VolumetricFormulation` 持有 `Basis` + `Quadrature` 并创建 `VolumetricKernel`；`ShellFormulation` 创建 `ShellKernel`。

```cpp
// 抽象基类
class Formulation {
public:
  virtual ~Formulation() = default;
  virtual std::string_view getName() const = 0;
  virtual int getNodesPerElement() const = 0;
  virtual int getLocalDofs() const = 0;
};

// Volumetric — 持有 Basis + Quadrature
class VolumetricFormulation : public Formulation {
public:
  VolumetricFormulation(std::unique_ptr<Basis> basis, std::unique_ptr<Quadrature> quad);
  const Basis &basis() const;
  const Quadrature &quadrature() const;
  std::unique_ptr<VolumetricKernel> createKernel(const double *restPositions) const;
};

// Shell — 创建 ShellKernel
class ShellFormulation : public Formulation {
public:
  std::unique_ptr<ShellKernel> createKernel(const double restX[18], const bool hasVtx[6]) const;
};

// 中间 tag 层
class TetFormulation : public VolumetricFormulation { ... };
class CubicFormulation : public VolumetricFormulation { ... };

// 具体 formulation
class P1TetFormulation : public TetFormulation { ... };         // nodes=4, dofs=12, name="tet_p1"
class LinearCubicFormulation : public CubicFormulation { ... }; // nodes=8, dofs=24, name="hex_trilinear"
class KoiterShellFormulation : public ShellFormulation { ... }; // nodes=6, dofs=18, name="shell_koiter"
```

统一 factory 入口：

```cpp
std::shared_ptr<DeformationModelEnergy> makeDeformationEnergy(
  const SimulationMesh &mesh,
  const Formulation &formulation,        // P1TetFormulation{} / LinearCubicFormulation{} / KoiterShellFormulation{}
  DeformationModelElasticMaterial elastic,
  DeformationModelPlasticMaterial plastic,
  const DeformationModelOptions &opts = {});
```

不保留旧 `makeDeformationModel(...)` auto-dispatch 入口。Python public API 要求 cubic 调用方显式传入 `pgo.fem.LinearCubic()`，并在文档中明确它映射到当前 8-node hex trilinear formulation，避免把 `SimulationMeshType::CUBIC` 拿来做隐式 auto-dispatch。

> **架构演进说明（2026-05-31）：** 原计划使用 tag struct + `FormulationTraits<T>` 模板 + C++20 concepts 的编译期 dispatch 方案。实施过程中发现该方案对 Python binding（需要运行时 formulation 选择）和 factory 内部 dispatch（需要 `dynamic_cast` 判断 formulation 类别）不友好，且 Concepts 约束在只需要三种 formulation 的场景下过度设计。最终改为 class hierarchy + 虚函数，简化了 binding 边界和 factory 实现。原 `formulationTraits.h`、`formulationConcepts.h`、`deformationFormulations.h` 已删除。

### 2. `Formulation` class hierarchy 做运行时 dispatch，不再使用 `FormulationTraits<T>` 模板

原计划使用 `FormulationTraits<T>` 模板 + C++20 concepts 做编译期 dispatch。实施中改为 class hierarchy + 虚函数：

- **`Formulation`** — 抽象基类，提供 `getName()`、`getNodesPerElement()`、`getLocalDofs()`。
- **`VolumetricFormulation`** — 持有 `Basis` + `Quadrature` 实例，提供 `createKernel(restPositions)` 创建 `VolumetricKernel`。
- **`ShellFormulation`** — 提供 `createKernel(restX, hasVtx)` 创建 `ShellKernel`（当前实现为 `KoiterShellKernel`）。

Factory 和 binding 层通过 `dynamic_cast` 判断 formulation 类别（volumetric vs shell）：

```cpp
if (auto *vf = dynamic_cast<const VolumetricFormulation *>(&formulation)) {
  // 使用 vf->basis(), vf->quadrature(), vf->createKernel(...)
} else if (auto *sf = dynamic_cast<const ShellFormulation *>(&formulation)) {
  // 使用 sf->createKernel(...)
}
```

这个设计的权衡：
- **优点：** Python binding 可以直接接收 `const Formulation &`（nanobind 天然支持多态）；factory 内部不需要 `std::variant` + `std::visit`；新增 formulation 是加一个子类而非修改模板。
- **代价：** 失去编译期 concept 验证（如"volumetric formulation 必须有 Basis"），但当前只有 3 种 formulation，运行时 `dynamic_cast` 失败即抛异常，代价可接受。

### 3. Formulation dispatch 使用 class hierarchy + 虚函数

Basis、Quadrature、Kernel 层使用虚函数做 runtime dispatch：

- `Basis` 虚基类（`formulations/basis/basis.h`）：`numNodes()`、`localDofs()`、`N()`、`dN_dxi()`、`nodeCoords()`。
- `Quadrature` 虚基类（`formulations/quadrature/quadrature.h`）：`numPoints()`、`point()`、`weight()`。
- `ShellKernel` 虚基类（`formulations/kernels/shellKernel.h`）：`compute_a_and_derivatives()`、`compute_b_and_derivatives()`、`restI()`、`restII()`、`restArea()`、`hasVtx()`。
- `TetP1Basis` / `HexTrilinearBasis` 继承 `Basis`；`TetP1DefaultQuadrature` / `GaussLegendreHexQuadrature2` 继承 `Quadrature`。
- `KoiterShellKernel` 继承 `ShellKernel`，实现 Koiter 薄壳微分几何。
- `VolumetricKernel` 为具体类（非模板），构造时接收 `const Basis &` 和 `const Quadrature &`。
- `VolumetricElementModel` 为具体类（非模板），持有 `VolumetricKernel`。
- `ShellElementModel` 为具体类（非模板），持有 `std::unique_ptr<ShellKernel>`，可适配任何 shell kernel 实现。

统一 factory 入口：

```cpp
std::shared_ptr<DeformationModelEnergy> makeDeformationEnergy(
  const SimulationMesh &mesh,
  const Formulation &formulation,     // P1TetFormulation{} / LinearCubicFormulation{} / KoiterShellFormulation{}
  DeformationModelElasticMaterial elastic,
  DeformationModelPlasticMaterial plastic,
  const DeformationModelOptions &opts = {});
```

Python/CLI binding 直接传 formulation 具体类实例给 factory。

> **架构演进说明（2026-05-31）：** 原 `formulationVariants.h`、`formulationTraits.h`、`formulationConcepts.h`、`deformationFormulations.h` 已删除。原 per-topology 模板 factory（`makeTetDeformationModel<TetP1>(...)` 等）已合并为单一 `makeDeformationEnergy()`。

### 4. 新文件按 façade / formulation / factory / material / parameter 分层

`solidDeformationModel` 根目录只保留 public façade、assembler/manager/energy 主链路，以及旧 element model wrapper。新抽象不要继续散落在根目录里，而是按职责放入子目录：

```text
  src/core/solidDeformationModel/
  simulationMesh.h/.cpp
  deformationModel.h
  deformationModelFactory.h/.cpp
  deformationModelAssembler.h/.cpp
  deformationModelEnergy.h/.cpp
  deformationModelManager.h/.cpp

  formulations/
    formulation.h/.cpp               # Formulation 抽象基类 + Volumetric/Shell + 具体类

    basis/
      basis.h                        # Basis 虚基类
      tetP1Basis.h/.cpp
      hexTrilinearBasis.h/.cpp

    quadrature/
      quadrature.h                   # Quadrature 虚基类
      tetP1DefaultQuadrature.h/.cpp
      gaussLegendreHexQuadrature.h/.cpp

    kernels/
      volumetricKernel.h/.cpp
      shellKernel.h                  # ShellKernel 虚基类
      koiterShellKernel.h/.cpp       # Koiter 薄壳 kernel 实现

    elements/
      volumetricElementModel.h/.cpp
      volumetricElementModelCacheData.h/.cpp
      shellElementModel.h/.cpp        # 通用 shell element model（持有 ShellKernel）
      shellElementModelCacheData.h
      parameterizedMaterialBlock.h

    geometry/
      tetP1Geometry.h

    dof/
      dofLayout.h
      vertex3DofLayout.h/.cpp

    parameters/
      parameterField.h
      constantParameterField.h

  factories/
    elasticModelFactory.h/.cpp
    plasticModelFactory.h/.cpp
    elementModelFactory.h/.cpp

  materials/
    elasticModelSpec.h               # future (Task 7)
    simulationMeshMaterialPayload.h  # future (Task 7)
    simulationMeshOrthotropicMaterial.h # future (Task 8)
```

测试目录镜像新结构：

```text
tests/src/core/solidDeformationModel/
  formulations/
    basis/
      tetP1Basis_gtest.cpp
      hexTrilinearBasis_gtest.cpp
    quadrature/
      tetP1DefaultQuadrature_gtest.cpp
      gaussLegendreHexQuadrature_gtest.cpp
    kernels/
      volumetricKernel_gtest.cpp
      koiterShellKernel_gtest.cpp
    elements/
      volumetricElementModel_gtest.cpp
      shellElementModel_gtest.cpp
    dof/
      vertex3DofLayout_gtest.cpp
    parameters/
      constantParameterField_gtest.cpp

  factories/
    elasticModelFactory_gtest.cpp
    elementModelFactory_gtest.cpp
```

旧 tet/cubic/shell element wrapper 已在 Task 5q 删除。

### 5. Python 第一版 deformation energy 使用位移向量作为 solver state

`DeformationModelEnergy` 持有 rest position 时，`func(x)` 内部计算 `restPosition + x`。Python API 必须把这个约定写清楚：

```python
energy.rest_position      # absolute rest positions, shape (n, 3)
u0 = energy.zero_state()  # displacement DOFs, flat shape (num_dofs,)
energy.value(u0)
energy.gradient(u0)
energy.hessian(u0)
```

如果后续需要绝对位置 convenience API，可以另加：

```python
energy.value_at_positions(x_abs)
```

不要让 Python 示例把 `rest_position` 当作 `value()` 的 state 传入。

### 6. Material payload、elastic law、active term 必须分层

当前 `material` 同时表示三种概念：

- payload：`ENuMaterial`、`MooneyRivlinMaterial`、`OrthotropicMaterial`、`HillMaterial` 这类参数数据。
- passive elastic law：`stable_neo`、`stvk`、`mooney_rivlin`、`orthotropic_stvk` 这类 `psi(F)` 计算规则。
- active term / composite term：Hill fiber 这种依赖 fiber direction、叠加在 base law 上的能量项。

重构后 C++ 和 Python 都按以下层次表达：

```text
SimulationMesh material payloads
  -> ElasticModelSpec / recipe
  -> ElasticModelFactory
  -> ElasticModel or ElasticModelCombinedMaterial
```

这意味着：

```text
ENu payload + StableNeo law                         -> StableNeo elastic model
ENu payload + StVK law                              -> StVK elastic model
MooneyRivlin payload + MooneyRivlin law             -> Mooney-Rivlin elastic model
Orthotropic payload + OrthotropicStVK law           -> Orthotropic elastic model
ENu payload + StableNeo law + Hill payload + fibers -> StableNeo + Hill active fiber composite
```

旧的 `DeformationModelElasticMaterial::HILL_STABLE_NEO` 不进入新 API；迁移时应显式写成 `ElasticModelSpec{passive=StableNeo, hillFiberTerms=[HillFiber]}`。

### 7. Python 不直接绑定 manager/assembler 链

Python 绑定一个 ownership root，例如：

```cpp
class DeformationEnergyCore
{
public:
  explicit DeformationEnergyCore(std::shared_ptr<DeformationModelEnergy> energy, ...);
  int numDofs() const;
  Eigen::VectorXd restPositionFlat() const;
  double value(...);
  Eigen::VectorXd gradient(...);
  SparseMatrixCore hessian(...);
};
```

内部仍然是：

```text
energy -> assembler -> manager -> mesh
```

但 Python 只看到 `pgo.energy.DeformationEnergy`。

### 8. `SimulationMesh` lifetime 要在 binding 前处理

Task 3 前，`SimulationMeshCore` 独占 `std::unique_ptr<SimulationMesh>`，而 `DeformationModelManager` 也会消费 `std::unique_ptr<SimulationMesh>`。Python public API 接受 `SimulationMesh` 并允许从同一个 mesh 创建多个 energy 时，manager 不能拥有或 move 这个 mesh；它必须只借用 immutable mesh。

第一版推荐把 manager 和 topology-specific factory 统一改成 `const SimulationMesh &` borrow：

```cpp
DeformationModelManager(
  const SimulationMesh &simulationMesh,
  DeformationModelPlasticMaterial plastic,
  DeformationModelElasticMaterial elastic,
  int enforceSPD,
  const double *elementFiberDirections = nullptr,
  const double *vertexFiberDirections = nullptr);

makeDeformationEnergy(mesh, P1TetFormulation{}, spec);
makeDeformationEnergy(mesh, LinearCubicFormulation{}, spec);
makeDeformationEnergy(mesh, KoiterShellFormulation{}, spec);
```

`DeformationModelManager` 内部只保存 `const SimulationMesh *simulationMesh` 作为 non-owning borrow，不再保存 `ownedMesh`。调用方必须保证 mesh owner 活得比 manager / assembler / energy 链更久。

Python binding 的 owner 形态是：

```text
SimulationMeshCore
  owns std::unique_ptr<SimulationMesh>

DeformationEnergyCore
  keeps std::shared_ptr<SimulationMeshCore> alive
  owns std::shared_ptr<DeformationModelEnergy>

DeformationModelManager
  borrows const SimulationMesh & from SimulationMeshCore
```

这样既不消费 Python `SimulationMesh`，也不需要 `SimulationMesh::clone()`。C++ core 不提供“从 `TetMesh` / `CubicMesh` / `TriMeshGeo` 直接创建 deformation energy 并暗中持有临时 mesh”的 convenience API；从 Vega/mesh geometry 到 solver-ready mesh 的转换必须先显式调用 `makeSimulationMesh(...)` 或 Python `SimulationMesh.create_*`，再把这个 owner 交给调用边界保存。这样 manager 永远只借用一个已经存在且有明确 owner 的 `SimulationMesh`。

### 9. `DeformationModelManager::initImpl` 先拆内部 factory，再公开抽象

`DeformationModelManager` 当前不只是“manager”：它同时做 per-element material/model 构造、拥有 elastic/plastic/element model storage、给 assembler 提供运行时查询。这个职责过胖，但不要在 Task 3 的 lifetime refactor 中顺手大拆；Task 3 只解决 mesh borrow 和 Python owner bridge。

第一刀只做 behavior-preserving extraction：

- `ElasticModelFactory`
- `PlasticModelFactory`
- `ElementModelFactory`

推荐目标分层是：

```text
ElementModelFactory
  = construct per-element elastic/plastic/element models from SimulationMesh + Formulation + legacy material enums/specs

DeformationModelManager  (temporary legacy holder / query surface)
  = own per-element model/material storage
  = expose getDeformationModel(ele), parameter counts, alignment matrices
  = no mesh ownership; only borrow const SimulationMesh &

DeformationModelAssembler
  = own manager/model set
  = own DofLayout after Task 6
  = run global assembly loops, cache, element weights, max-step aggregation
```

Task 5 should make `initImpl` read as orchestration and move creation policy into factories. It should not rename/remove `DeformationModelManager` yet, because assembler/tests/tools still depend on its query surface. After Task 5 and Task 6 both land, a later cleanup can rename it to something like `ElementModelSet` / `DeformationModelCollection`, or replace it with a narrower model-set object.

### 10. `DofLayout` 先服务现有 vertex DOF，再服务 Hermite

先实现 `Vertex3DofLayout`，让现有 tet/cubic/shell 走同一套 layout 接口。Hermite 后续新增 `HermiteDofLayout`，不在本计划实现。

```cpp
class DofLayout
{
public:
  virtual ~DofLayout() = default;
  virtual int numGlobalDofs() const = 0;
  virtual int numLocalDofs(int ele) const = 0;
  virtual void getGlobalDofIndices(int ele, std::vector<int> &indices) const = 0;
  virtual void gather(int ele, const double *global, double *local) const = 0;
  virtual void scatterAddGradient(int ele, const double *local, double *global) const = 0;
  virtual void addHessianSparsity(int ele, std::vector<EigenSupport::TripletD> &entries) const = 0;
  virtual void buildLocalToGlobalMatrixIndices(int ele, const EigenSupport::SpMatD &KTemplate,
    DynamicIndexMatrix &indices) const = 0;
};
```

`DeformationModelAssembler` 的 loops 仍可保留，但不再直接假设 `vertexIndices[v] * 3 + dof`。

### 11. Elastic / Plastic 参数统一通过 `ParameterField` 表达

Elastic material parameters 和 plastic parameters 在 solver 里需要表达同一件事：参数值从全局存储到积分点的完整路径。这个路径包含两部分——全局向量里的 DOF 索引（gather/scatter），以及积分点上的采样规则（constant / quadrature / nodal / external）。这两部分是 1:1 的固定配对，每种 field 实现只对应一种 layout，不存在有意义的交叉组合，因此不拆成两个独立抽象。目标抽象是：

```text
ParameterField
  = quadrature-point sampling only
  = constant / quadrature / nodal interpolation / external procedural field

OptimizableField : ParameterField
  = ParameterField + DOF layout (gather/scatter/sparsity)
  = 参数参与优化时使用；Assembler 通过 dynamic_cast 获取 dofLayout()

ElasticModel
  = local elastic law:
    Fe_q, b_q -> psi, P, dPdF, dpsi/db, dP/db, ...

PlasticModel
  = local plastic parametrization:
    a_q -> Fp_q, FpInv_q, detFp_q, dFpInv/da, d(detFp)/da, ...

ElasticBlock
  = non-owning ElasticModel pointer + non-owning ParameterField pointer

PlasticBlock
  = non-owning PlasticModel pointer + non-owning ParameterField pointer

ElementModel
  = chain rule + quadrature integration
```

接口草图：

```cpp
struct ParameterSample
{
  EigenSupport::VXd value;      // b_q for elastic, a_q for plastic
  EigenSupport::MXd dValueDLocal;
  void resize(int numChannels, int numLocalDofs);
};

class ParameterField
{
public:
  virtual ~ParameterField() = default;
  virtual int numChannels() const = 0;

  // Quadrature-point sampling — field 构造时持有全局参数数据指针，完全自包含
  // 调用者只需提供 ele 和 quadrature，无需传递参数数据
  virtual void sample(int ele, int quadratureId, ParameterSample &out) const = 0;
};

// OptimizableField — 参数参与优化的 field，额外暴露 DOF layout
// Assembler 通过 dynamic_cast<const OptimizableField *> 获取 dofLayout
class OptimizableField : public ParameterField
{
public:
  virtual const ParameterDofLayout *dofLayout() const = 0;
};

struct ElasticBlock
{
  ElasticModel *model = nullptr;          // non-owning; manager owns the model
  const ParameterField *parameters = nullptr;  // non-owning; manager owns the field
};

struct PlasticBlock
{
  PlasticModel *model = nullptr;          // non-owning; manager owns the model
  const ParameterField *parameters = nullptr;  // non-owning; manager owns the field
};
```

Ownership/data-flow rule:

- 所有指针统一为 non-owning 裸指针。Manager 统一持有 `ElasticModel`、`PlasticModel`、`ParameterField` 的全部实例。`ElasticBlock` / `PlasticBlock` / `ElementModel` 只借用，不拥有任何这些对象。
- Elastic 和 plastic 不共享 parameter field 假设。`ElasticBlock` 持有自己的 `ParameterField`，`PlasticBlock` 持有自己的 `ParameterField`；两者的 field 实现可以完全不同。
- `ParameterField` 只负责 quadrature-point sampling。参数参与优化的 field 同时实现 `OptimizableField`，额外暴露 `dofLayout()`。每种 field 实现内部固定一种 DOF layout，不存在”同一个 field 配不同 layout”或”同一个 layout 配不同 field”的场景。`ElementParameterDofLayout` / `QuadratureParameterDofLayout` / `NodalParameterDofLayout` 作为 field 内部的 private/protected 实现细节存在，不暴露为独立 public 类型。
- `DeformationModelAssembler` 持有 `const ParameterField *`（与 `ElasticBlock` 一致）。当前 Assembler 不调用 `dofLayout()`——参数优化链路尚未落地。未来需要时通过 `dynamic_cast<const OptimizableField *>` 获取 layout。
- `VolumetricElementModel` 通过 `ElasticBlock` / `PlasticBlock` 持有对应 `ParameterField` 的 non-owning 指针，在 `prepareData(...)` 内用 `parameters->sample(ele, quadrature, out)` 直接从全局参数向量采样到 quadrature point。field 构造时即持有数据指针，gather + sample 完全内化。
- `ElasticBlock` / `PlasticBlock` 不转移 `ElasticModel` / `PlasticModel` ownership；它们只把 manager/model-set 已拥有的 model pointer 和对应 `ParameterField` 组合起来。不要在 Task 6p 顺手把 material/plastic ownership 搬进 element model；owner-vector cleanup 属于 Task 5p 或后续 cleanup。
- `ElasticModel` 和 `PlasticModel` 不直接持有 `ParameterField`；它们保持 sample-level local law / local parametrization。
- Task 6p 简化 `DeformationModel::prepareData` 的 virtual signature（详见下方"prepareData API 简化"小节）。

### 11a. `prepareData` API 简化

当前 `DeformationModel::prepareData` 签名：

```cpp
virtual void prepareData(const double *x,
    const double *plasticLocal,    // Assembler 预 gather 的 per-element plastic 参数
    const double *elasticLocal,    // Assembler 预 gather 的 per-element elastic 参数
    CacheData *cache) = 0;
```

Assembler 在调用前负责三路 gather（位移 + plastic 参数 + elastic 参数）。这种设计把参数存储策略暴露给了 Assembler——Assembler 必须知道什么是 local parameter vector、怎么 gather。

新设计把参数 gather 完全内化到 `ParameterField::sample()` 中。`prepareData` 简化为：

```cpp
virtual void prepareData(const double *x, CacheData *cache) = 0;
```

Assembler 侧退化为只管位移：

```cpp
for (int ele = 0; ele < nele; ele++) {
  dofLayout->gather(ele, xGlobal, xLocal);   // 只管位移
  elementModel->prepareData(xLocal, cache);  // 参数由 element model 自行处理
}
```

子类适配：

- **`VolumetricElementModel`**——自然适配。`ele_` 在构造时注入，global params 在 `ConstantParameterField` 构造时注入。`prepareData` 内部对每个 quadrature point 调用 `elasticBlock_.parameters->sample(ele_, qv, sample)` 和 `plasticBlock_.parameters->sample(ele_, qv, sample)`，field 内部的 dofLayout 完成 gather + 采样。
- **`ShellElementModel`**——也需要引入 `ElasticBlock` / `PlasticBlock`。Shell 没有 quadrature loop，只有一个 material location，用 `sample(ele, 0, ...)` 调用采样。
- **Legacy wrapper（`TetMeshDeformationModel` / `CubicMeshDeformationModel`）**——在 Task 5q 被删除，不需要适配。

这个变更是破坏性的（所有 `DeformationModel` 子类需同步修改），但实际只涉及两个活跃子类（`VolumetricElementModel` + `ShellElementModel`），且 legacy wrapper 届时已删除。代价可控，收益是基类不再对"参数从哪里来"做任何假设——`ExternalProceduralField` 可以在 `sample()` 内部按文件/函数生成参数值，不需要任何外部 gather。
- `ParameterSample` 的 Eigen buffers 由 caller/cache 持有并复用。`ParameterField::sample(...)` 写入 `out`，不要在 quadrature loop 里按值返回 owning vectors。

命名上使用 `PlasticParameters`，不用 `PlasticState`。这里的对象只是生成 `Fp` 的参数向量，不承诺完整的 history-dependent plasticity 状态系统。未来如果需要真正的 return mapping / time integration internal state，应另起 `PlasticInternalState` 或 `HistoryState` 概念，不复用 `PlasticParameters`。

最重要的计算语义：

> Volumetric deformation-gradient element 中，真正参与本构计算的 `Fp`、`Fe`、elastic parameters、plastic parameters 只在 quadrature point 上存在。Element-constant、nodal field、external field 都只是 `ParameterField::sample(ele, quadrature, out)` 的不同实现；`ElementModel` 不直接读取”element-level material/plastic value”作为真实计算位置，也不参与参数 gather——gather 完全由 field 内部通过自己的 dofLayout 完成，数据指针在构造时注入。

第一版只实现 behavior-preserving 组合：

```text
elasticParameters:
  ConstantParameterField(numElasticChannels, nele, elasticGlobalParams)
    ← 内部使用 ElementParameterDofLayout(nele, numElasticChannels)

plasticParameters:
  ConstantParameterField(numPlasticChannels, nele, plasticGlobalParams)
    ← 内部使用 ElementParameterDofLayout(nele, numPlasticChannels)
```

这完全复刻当前 `paramsAll[ele * numParams + j]` 的 per-element constant 行为。`QuadraturePointParameterField`、`NodalInterpolatedParameterField`、`ExternalParameterField` 和 optimized parameter DOF 只作为 extension point，不在第一版行为迁移中实现。

## 目标 C++ API

### 当前状态（2026-05-31，Tasks 0–6p 完成后）

```cpp
// =================== Formulation class hierarchy ===================

class Formulation {
public:
  virtual ~Formulation() = default;
  virtual std::string_view getName() const = 0;
  virtual int getNodesPerElement() const = 0;
  virtual int getLocalDofs() const = 0;
};

class VolumetricFormulation : public Formulation {
public:
  VolumetricFormulation(std::unique_ptr<Basis> basis, std::unique_ptr<Quadrature> quad);
  const Basis &basis() const;
  const Quadrature &quadrature() const;
  std::unique_ptr<VolumetricKernel> createKernel(const double *restPositions) const;
};

class TetFormulation : public VolumetricFormulation { ... };
class CubicFormulation : public VolumetricFormulation { ... };

class P1TetFormulation : public TetFormulation { ... };          // name="tet_p1", nodes=4, dofs=12
class LinearCubicFormulation : public CubicFormulation { ... };  // name="hex_trilinear", nodes=8, dofs=24

class ShellFormulation : public Formulation {
public:
  std::unique_ptr<ShellKernel> createKernel(const double restX[18], const bool hasVtx[6]) const;
};

class KoiterShellFormulation : public ShellFormulation { ... };  // name="shell_koiter", nodes=6, dofs=18

// =================== Factory ===================

struct DeformationModelOptions {
  bool enforceSPD = true;
  bool enableMaterialMaxStep = true;
  EigenSupport::VXd elementWeights;
};

std::unique_ptr<SimulationMesh> makeSimulationMesh(const VolumetricMeshes::VolumetricMesh &mesh);

// 当前：使用 legacy elastic/plastic enum
std::shared_ptr<DeformationModelEnergy> makeDeformationEnergy(
  const SimulationMesh &mesh,
  const Formulation &formulation,
  DeformationModelElasticMaterial elastic,
  DeformationModelPlasticMaterial plastic,
  const DeformationModelOptions &opts = {});
```

### Task 7 完成后（Public API 切到 `ElasticModelSpec`）

```cpp
// =================== Material Spec (Task 7 新增) ===================

enum class MaterialPayloadKind { ENU, MOONEY_RIVLIN, ORTHOTROPIC, HILL_ACTIVATION };

enum class PassiveElasticLaw { STABLE_NEO, LINEAR, STVK, STVK_VOL, MOONEY_RIVLIN, ORTHOTROPIC_STVK, KOITER_STVK };
enum class ActiveElasticTerm { HILL_FIBER };

struct PassiveElasticSpec {
  PassiveElasticLaw law = PassiveElasticLaw::STABLE_NEO;
  int materialSlot = 0;
};

struct HillFiberSpec {
  int materialSlot = 1;
  const double *elementFiberDirections = nullptr;
  const double *vertexFiberDirections = nullptr;
};

struct ElasticModelSpec {
  PassiveElasticSpec passive;
  std::vector<HillFiberSpec> hillFiberTerms;
};

// =================== Updated Factory (Task 7 完成后) ===================

std::shared_ptr<DeformationModelEnergy> makeDeformationEnergy(
  const SimulationMesh &mesh,
  const Formulation &formulation,
  const ElasticModelSpec &elasticSpec,
  DeformationModelPlasticMaterial plastic,
  const DeformationModelOptions &opts = {});
```

旧 `DeformationModelElasticMaterial` enum 不进入 Task 7 之后的 public factory API。下面表格是人工迁移对照：

| Legacy enum | New spec |
|---|---|
| `STABLE_NEO` | `PassiveElasticSpec{STABLE_NEO, slot 0}` |
| `LINEAR` | `PassiveElasticSpec{LINEAR, slot 0}` |
| `STVK` | `PassiveElasticSpec{STVK, slot 0}` |
| `STVK_VOL` | `PassiveElasticSpec{STVK_VOL, slot 0}` |
| `MOONEY_RIVLIN` | `PassiveElasticSpec{MOONEY_RIVLIN, slot 0}` |
| `HILL_STABLE_NEO` | `PassiveElasticSpec{STABLE_NEO, slot 0}` + `HillFiberSpec{slot 1}` |
| `HILL_STVK` | `PassiveElasticSpec{STVK, slot 0}` + `HillFiberSpec{slot 1}` |
| `HILL_STVK_VOL` | `PassiveElasticSpec{STVK_VOL, slot 0}` + `HillFiberSpec{slot 1}` |

`DeformationModelPlasticMaterial` 在本 milestone 保持 enum 形式（`VOLUMETRIC_DOF6` / `VOLUMETRIC_DOF3` / `SHELL_FF_DOF1` 等），暂不引入 `PlasticParameterSpec` struct。`exposeAsOptimizationVariable` 和 `ParameterFieldKind` 非 CONSTANT 的字段保持为 future extension point。

## Python API 定稿策略

下面的 Python 形状只是 C++ Tasks 0-6p 完成后的 public API 目标，不是早期 private smoke hook 的形状。模块职责按长期边界划分：

- `pypgo.fem`：FEM/domain construction surface。包含 formulation、elastic law wrapper、plastic parametrization wrapper、deformation options，以及 deformation energy factory。
- `pypgo.energy`：通用 energy object surface。包含最终返回的 `DeformationEnergy` wrapper、未来 `EnergySet`、generic linear/quadratic energy 等可求值/可组合对象。

不要把 FEM 材料、formulation、plastic enum/string helper 或 deformation factory 放进 `pypgo.energy`。`pypgo.energy` 只知道“这是一个符合 energy protocol 的对象”，不承担 deformation/FEM 配置语义。

`pypgo.fem` / `pypgo.energy` 必须等以下 C++ 边界稳定后再统一定稿：

- manager borrow-only lifetime 和 Python owner bridge；
- `ElementModelFactory` 接管 formulation-driven element construction；
- `DofLayout` 接管 gather/scatter/sparsity；
- `ParameterField` 接管 elastic/plastic parameter gather、quadrature sampling、scatter/sparsity；
- behavior-preserving 的 legacy elastic/plastic enum wrapper 已经能通过一处 Python factory 统一翻译。

预期 public module roles：

```text
pypgo.fem      # formulations, material-law wrappers, plastic wrappers, options, factory
pypgo.energy   # final energy wrappers and generic energy composition/evaluation
```

示例：

```python
import pypgo as pgo

volume = pgo.mesh.veg.VolumeMesh.create_from_single_material(
    cube_data,
    pgo.mesh.veg.ENuMaterial(E=1e6, nu=0.45),
)
sim_mesh = pgo.sim.SimulationMesh.create_volumetric(volume)

energy = pgo.fem.deformation_energy(
    sim_mesh,
    formulation=pgo.fem.LinearCubic(),
    elastic=pgo.fem.StableNeo(),
    plastic=pgo.fem.VolumetricPlasticity(dofs=6),
)

assert isinstance(energy, pgo.energy.DeformationEnergy)

u = energy.zero_state()
value = energy.value(u)
grad = energy.gradient(u)
H = energy.hessian(u)
rows, cols, values = H.to_coo()
```

Python formulation objects（名称表达数学 formulation，不暴露 C++ 历史命名）：

```python
pgo.fem.TetP1()           # maps to C++ P1TetFormulation, name="tet_p1"
pgo.fem.LinearCubic()     # maps to C++ LinearCubicFormulation, name="hex_trilinear"
pgo.fem.KoiterShell()     # maps to C++ KoiterShellFormulation, name="shell_koiter"
```

`LinearCubic` 作为 public Python formulation 名称保留，因为它和 C++ `LinearCubicFormulation` 对齐。文档和测试必须说明它的数值含义是当前 cubic topology 上的 `hex_trilinear` formulation；它不是“任意 cubic topology 自动选择”的入口。

Python elastic law wrappers（当前 milestone 仍翻译到 legacy `DeformationModelElasticMaterial` enum string；未来 Task 7 再替换为 `ElasticModelSpec`）：

```python
pgo.fem.StableNeo()        # "stable_neo"
pgo.fem.StVK()             # "stvk"
pgo.fem.StVKVolume()       # "stvk_vol"
pgo.fem.LinearElastic()    # "linear"
pgo.fem.MooneyRivlin()     # "mooney_rivlin"; only valid with Mooney-Rivlin payload
pgo.fem.KoiterStVK()       # "koiter_stvk"; shell only
```

不要在第一版 public API 暴露 `HillStableNeo` / `HillStVK` / `HillStVKVolume` 这种 legacy composite enum 名称。Hill 只有在 Task 7 引入 `ElasticModelSpec` 和 fiber direction binding 后才作为：

```python
pgo.fem.HillFiber(base=pgo.fem.StableNeo(), hill_slot=1, element_fibers=fibers)
```

进入 public API。`OrthotropicStVK()` 同理等 Task 8 的 solver-side law 和 frame tests 完成后再暴露。

Python plastic parametrization wrappers：

```python
pgo.fem.VolumetricPlasticity(dofs=6)  # "volumetric_dof6"; tet/cubic common path
pgo.fem.VolumetricPlasticity(dofs=3)  # "volumetric_dof3"
pgo.fem.VolumetricPlasticity(dofs=0)  # "volumetric_dof0"
pgo.fem.ShellPlasticity(dofs=1)       # "shell_ff_dof1"
pgo.fem.ShellPlasticity(dofs=0)       # "shell_ff_dof0"
```

不提供 `pgo.fem.Plastic("volumetric_dof6")` 作为 public API；raw enum strings 只允许出现在 wrapper 内部和 `_core` binding 边界。

Python options wrapper：

```python
pgo.fem.DeformationOptions(
    enforce_spd=True,
    enable_material_max_step=True,
    element_weights=None,
)
```

最终定稿时应满足的规则：

- `pgo.fem.deformation_energy(...)` 是唯一 public deformation construction factory；不提供 `pgo.energy.deformation_energy(...)` alias。
- `pgo.energy.DeformationEnergy` 是返回对象类型；它暴露 `num_dofs`、`rest_position`、`zero_state()`、`value(u)`、`gradient(u)`、`hessian(u)`，并能进入未来 `pgo.energy.EnergySet`。
- Tet mesh 可显式传 `TetP1()`；如果 formulation omitted，可默认 `TetP1()`。
- Cubic mesh 必须显式传 `LinearCubic()`；不传时抛 `ValueError`，提示当前 cubic topology 有多个未来 formulation。
- Shell mesh 必须显式传 `KoiterShell()`，并使用现有 `SimulationMesh.create_shell(...)` 产生的 shell payload；shell 走 `ShellFormulation -> KoiterShellKernel -> ShellElementModel` pipeline，不进入 volumetric `Basis` / `Quadrature` / `VolumetricKernel` 抽象。
- `elastic` 和 `plastic` 第一版都要求显式传入；不要把 volumetric/shell 默认 plastic policy 藏进 `pypgo.energy`。
- 第一版 Python 把 elastic/plastic parameter field 默认成 element-constant；nodal / quadrature / external parameter fields 和 optimized elastic/plastic parameters 必须等 C++ `ParameterField` 导数测试完成后再暴露。
- `MooneyRivlin + MooneyRivlin law` 只有在对应 payload mismatch tests 通过后才能进入 public Python API。
- `base law + HillFiber`、`Orthotropic + OrthotropicStVK law` 只有在对应 C++ payload conversion、law factory、fiber/frame tests 完成后才能进入 public Python API。
- Hill 必须要求 `HillActivation` payload 和 fiber field；不能把 Hill 当作无方向的 passive law。

## Topology/Formulation Matrix

| Topology | Formulation | C++ Class | DofLayout | 状态 |
|---|---|---|---|---|
| `TetMesh` / `SimulationMeshType::TET` | `P1Tet` | `P1TetFormulation` | `Vertex3DofLayout` | `VolumetricKernel(TetP1Basis, TetP1DefaultQuadrature)` + `VolumetricElementModel`，行为严格保持 |
| `CubicMesh` / `SimulationMeshType::CUBIC` | `LinearCubic` | `LinearCubicFormulation` | `Vertex3DofLayout` | `VolumetricKernel(HexTrilinearBasis, GaussLegendreHexQuadrature2)` + `VolumetricElementModel`，行为严格保持 |
| shell `SimulationMeshType::SHELL` | `KoiterShell` | `KoiterShellFormulation` | `Vertex3DofLayout` with invalid local sentinel | `KoiterShellKernel` + `ShellElementModel`；不并入 volumetric basis/quadrature/kernel 抽象 |
| `SimulationMeshType::TRIANGLE` / `EDGE_QUAD` | none | none | none | 不在本计划 deformation energy 范围内 |

## Material Recipe Matrix

| Payload slot type | Elastic recipe | Solver model | 状态 |
|---|---|---|---|
| `SimulationMeshENuMaterial` | `StableNeo` | `ElasticModelStableNeoHookeanMaterial` | 当前行为，必须保持 |
| `SimulationMeshENuMaterial` | `Linear` | `ElasticModelLinearMaterial` | 当前行为，通过新 recipe 表达 |
| `SimulationMeshENuMaterial` | `StVK` | `ElasticModel3DSTVKMaterial` or invariant StVK path | 当前行为，通过新 recipe 表达 |
| `SimulationMeshMooneyRivlinMaterial` | `MooneyRivlin` | `ElasticModel3DMooneyRivlin` | 当前 C++ enum path 存在；Finalize 暴露前必须加 payload mismatch / smoke tests |
| `SimulationMeshOrthotropicMaterial` | `OrthotropicStVK` | new `ElasticModel3DOrthotropicStVK` | C++ law 补齐（Task 8）|
| `SimulationMeshENuMaterial` + `SimulationMeshHillMaterial` + fibers | `HillFiber(base=StableNeo/StVK/StVKVol)` | `ElasticModelCombinedMaterial` | C++ composite recipe 先落地（Task 7）|
| `SimulationMeshENuhMaterial` shell payload | `KoiterStVK` | `ElasticModel2DFundamentalFormsSTVK` + `ShellElementModel(KoiterShellKernel)` | 当前 shell 行为通过新 shell element stack 保持 |

不支持的组合必须 fail fast。例如 `StableNeo` 不能读取 `MooneyRivlin` payload，`HillFiber` 不能缺少 fiber direction，`OrthotropicStVK` 不能在 `ElasticModel3DOrthotropicStVK` 测试通过前开放。

## File Map

### 已完成（Tasks 0–6p）

- `src/core/solidDeformationModel/formulations/formulation.h`
- `src/core/solidDeformationModel/formulations/formulation.cpp`
- `src/core/solidDeformationModel/formulations/basis/basis.h`
- `src/core/solidDeformationModel/formulations/basis/tetP1Basis.h`
- `src/core/solidDeformationModel/formulations/basis/tetP1Basis.cpp`
- `src/core/solidDeformationModel/formulations/basis/hexTrilinearBasis.h`
- `src/core/solidDeformationModel/formulations/basis/hexTrilinearBasis.cpp`
- `src/core/solidDeformationModel/formulations/quadrature/quadrature.h`
- `src/core/solidDeformationModel/formulations/quadrature/tetP1DefaultQuadrature.h`
- `src/core/solidDeformationModel/formulations/quadrature/gaussLegendreHexQuadrature.h`
- `src/core/solidDeformationModel/formulations/kernels/volumetricKernel.h`
- `src/core/solidDeformationModel/formulations/kernels/volumetricKernel.cpp`
- `src/core/solidDeformationModel/formulations/kernels/shellKernel.h`
- `src/core/solidDeformationModel/formulations/kernels/koiterShellKernel.h`
- `src/core/solidDeformationModel/formulations/kernels/koiterShellKernel.cpp`
- `src/core/solidDeformationModel/formulations/elements/volumetricElementModel.h`
- `src/core/solidDeformationModel/formulations/elements/volumetricElementModel.cpp`
- `src/core/solidDeformationModel/formulations/elements/volumetricElementModelCacheData.h`
- `src/core/solidDeformationModel/formulations/elements/volumetricElementModelCacheData.cpp`
- `src/core/solidDeformationModel/formulations/elements/shellElementModel.h`
- `src/core/solidDeformationModel/formulations/elements/shellElementModel.cpp`
- `src/core/solidDeformationModel/formulations/elements/shellElementModelCacheData.h`
- `src/core/solidDeformationModel/formulations/elements/parameterizedMaterialBlock.h`
- `src/core/solidDeformationModel/formulations/geometry/tetP1Geometry.h`
- `src/core/solidDeformationModel/formulations/dof/dofLayout.h`
- `src/core/solidDeformationModel/formulations/dof/vertex3DofLayout.h`
- `src/core/solidDeformationModel/formulations/dof/vertex3DofLayout.cpp`
- `src/core/solidDeformationModel/formulations/parameters/parameterField.h`
- `src/core/solidDeformationModel/formulations/parameters/constantParameterField.h`
- `src/core/solidDeformationModel/factories/elasticModelFactory.h`
- `src/core/solidDeformationModel/factories/elasticModelFactory.cpp`
- `src/core/solidDeformationModel/factories/plasticModelFactory.h`
- `src/core/solidDeformationModel/factories/plasticModelFactory.cpp`
- `src/core/solidDeformationModel/factories/elementModelFactory.h`
- `src/core/solidDeformationModel/factories/elementModelFactory.cpp`
- `src/core/solidDeformationModel/deformationModelAssemblerCacheData.h`
- `src/python/pypgo/bindings/simulation_mesh_core.h`
- `src/python/pypgo/bindings/energy_bindings.cpp` (private `_core` hooks)
- `tests/src/core/solidDeformationModel/formulations/deformationModelFormulation_gtest.cpp`
- `tests/src/core/solidDeformationModel/formulations/basis/tetP1Basis_gtest.cpp`
- `tests/src/core/solidDeformationModel/formulations/basis/hexTrilinearBasis_gtest.cpp`
- `tests/src/core/solidDeformationModel/formulations/kernels/volumetricKernel_gtest.cpp`
- `tests/src/core/solidDeformationModel/formulations/kernels/koiterShellKernel_gtest.cpp`
- `tests/src/core/solidDeformationModel/formulations/elements/volumetricElementModel_gtest.cpp`
- `tests/src/core/solidDeformationModel/formulations/elements/shellElementModel_gtest.cpp`
- `tests/src/core/solidDeformationModel/formulations/dof/vertex3DofLayout_gtest.cpp`
- `tests/src/core/solidDeformationModel/formulations/parameters/constantParameterField_gtest.cpp`
- `tests/src/core/solidDeformationModel/factories/elasticModelFactory_gtest.cpp`
- `tests/src/core/solidDeformationModel/factories/elementModelFactory_gtest.cpp`

### Task 7 新增

- `src/core/solidDeformationModel/materials/elasticModelSpec.h`
- `src/core/solidDeformationModel/materials/simulationMeshMaterialPayload.h`

### Task 8 新增

- `src/core/solidDeformationModel/materials/simulationMeshOrthotropicMaterial.h`
- `src/core/solidDeformationModel/elasticModel3DOrthotropicStVK.h`
- `src/core/solidDeformationModel/elasticModel3DOrthotropicStVK.cpp`
- `tests/src/core/solidDeformationModel/elasticModel3DOrthotropicStVK_gtest.cpp`

### Finalize Python API 新增

- `pypgo/fem.py`
- `pypgo/energy.py`
- `tests/pypgo/test_deformation_energy.py`

### 修改

- `src/core/solidDeformationModel/CMakeLists.txt`
- `src/core/solidDeformationModel/simulationMesh.h`
- `src/core/solidDeformationModel/simulationMesh.cpp`
- `src/core/solidDeformationModel/deformationModelFactory.h`
- `src/core/solidDeformationModel/deformationModelFactory.cpp`
- `src/core/solidDeformationModel/deformationModelManager.h`
- `src/core/solidDeformationModel/deformationModelManager.cpp`
- `src/core/solidDeformationModel/deformationModelAssembler.h`
- `src/core/solidDeformationModel/deformationModelAssembler.cpp`
- `src/python/pypgo/CMakeLists.txt`
- `src/python/pypgo/bindings/module.cpp`
- `src/python/pypgo/bindings/mesh_bindings.cpp`
- `src/c/pgo_c.cpp`
- `src/tools/sim/runIPCSim/setup/femSetup.h`
- `src/tools/sim/runIPCSim/setup/femSetup.cpp`
- `src/tools/sim/runIPCSim/setup/setupCommon.h`
- `src/tools/sim/runIPCSim/setup/setupCommon.cpp`
- `src/tools/sim/runIPCSim/setup/volumeSetup.cpp`
- `src/tools/sim/runIPCSim/setup/legacySetup.cpp`
- `tests/src/tools/runSimShared_gtest.cpp`
- `pypgo/__init__.py`
- `pypgo/mesh/veg.py`
- `pypgo/sim.py`
- `tests/src/core/solidDeformationModel/deformationModelFactory_gtest.cpp`
- `tests/src/core/solidDeformationModel/deformationModelAssembler_gtest.cpp`
- `tests/src/core/solidDeformationModel/CMakeLists.txt`
- `tests/pypgo/test_simulation_mesh.py`

## Task 0: Baseline Characterization

**目标：** 在改 API 前先锁住当前行为和 public state convention。

**Files:**

- Modify: `tests/src/core/solidDeformationModel/deformationModelFactory_gtest.cpp`
- Modify: `tests/src/core/solidDeformationModel/deformationModelAssembler_gtest.cpp`
- Modify: `tests/pypgo/test_simulation_mesh.py`

- [x] Add C++ baseline test: current tet deformation energy path at zero displacement has finite near-zero energy and finite gradient, without depending on formulation tags that are introduced later.
- [x] Add C++ baseline test: current cubic deformation energy path at zero displacement has finite near-zero energy and finite gradient, without depending on formulation tags that are introduced later.
- [x] Drop manual manager/assembler-chain parity tests once topology-specific factories are thin borrow-only façades; behavior should be locked by public-contract baseline and element/assembler regression tests instead.
- [x] Add C++ characterization test: current ENu load path produces `SimulationMeshENuMaterial` payloads.
- [x] Add C++ characterization test: current Mooney-Rivlin `.veg` / `VolumeMesh` payloads exist before deformation conversion.
- [x] Add C++ characterization test: current Hill path requires an extra `SimulationMeshHillMaterial` slot and fiber directions.
- [x] Add C++ characterization test: current Orthotropic payload can be read from Vega but has no deformation `ElasticModel` yet.
- [x] Add Python test: `pypgo.mesh.veg.VolumeMesh` can carry ENu, Mooney-Rivlin, and Orthotropic regions before simulation conversion.
- [x] Add Python test note or assertion that `SimulationMesh.mesh_type == "cubic"` is topology metadata, not formulation metadata.

**Exit criteria:**

- Current behavior is covered before any refactor.
- Tests clarify that `DeformationModelEnergy` state input is displacement when rest position is stored.
- Tests document which material capabilities are payload-only today and which already reach deformation energy.
- No Task 0 test may reference formulation-specific symbols (`P1TetFormulation`, `LinearCubicFormulation`, `ElasticModelSpec`); those symbols are introduced later and must not be prerequisites for baseline characterization.

## Task 1: Extract Basis, Quadrature, Kernels, And Shared Element Model

**目标：** 先把 tet P1 和 hex trilinear 的 formulation 数学从旧 element model 中抽出来，并进一步拆成 `Basis`、`Quadrature`、`VolumetricKernel`，再让二者共同走 `VolumetricElementModel`。Basis/Quadrature/Kernel 使用虚函数 dispatch（非模板），为后续 formulation class hierarchy 做准备。

> **Note:** 实施过程中架构从 template-based `FormulationTraits<T>` + concepts 演变为 class hierarchy + 虚函数。详见 Architecture Evolution Notes。

**Files:**

- Create: `src/core/solidDeformationModel/formulations/basis/tetP1Basis.h`
- Create: `src/core/solidDeformationModel/formulations/basis/tetP1Basis.cpp`
- Create: `src/core/solidDeformationModel/formulations/basis/hexTrilinearBasis.h`
- Create: `src/core/solidDeformationModel/formulations/basis/hexTrilinearBasis.cpp`
- Create: `src/core/solidDeformationModel/formulations/quadrature/tetP1DefaultQuadrature.h`
- Create: `src/core/solidDeformationModel/formulations/quadrature/gaussLegendreHexQuadrature.h`
- Create: `src/core/solidDeformationModel/formulations/kernels/volumetricKernel.h`
- Create: `src/core/solidDeformationModel/formulations/elements/volumetricElementModel.h`
- Modify: `src/core/solidDeformationModel/tetMeshDeformationModel.h`
- Modify: `src/core/solidDeformationModel/tetMeshDeformationModel.cpp`
- Modify: `src/core/solidDeformationModel/cubicMeshDeformationModel.h`
- Modify: `src/core/solidDeformationModel/cubicMeshDeformationModel.cpp`
- Modify: `src/core/solidDeformationModel/CMakeLists.txt`
- Create: `tests/src/core/solidDeformationModel/formulations/basis/tetP1Basis_gtest.cpp`
- Create: `tests/src/core/solidDeformationModel/formulations/basis/hexTrilinearBasis_gtest.cpp`
- Create: `tests/src/core/solidDeformationModel/formulations/kernels/volumetricKernel_gtest.cpp`
- Create: `tests/src/core/solidDeformationModel/formulations/elements/volumetricElementModel_gtest.cpp`
- Modify: `tests/src/core/solidDeformationModel/CMakeLists.txt`

- [x] Add `TetP1Basis` and `HexTrilinearBasis` for reference-element interpolation:
  - node/local DOF shape metadata;
  - `N(xi)`;
  - `dN/dxi`.
- [x] Add `TetP1DefaultQuadrature` and `GaussLegendreHexQuadrature2`:
  - quadrature points in reference coordinates;
  - reference weights;
  - no material, mesh, or rest-geometry state.
- [x] Add `VolumetricKernel` with rest-shape precomputation currently embedded in `TetMeshDeformationModel` / `CubicMeshDeformationModel`.
- [x] Do not add `TetP1Kernel` / `HexTrilinearKernel` compatibility aliases. Use the concrete template instantiations directly:
  - `VolumetricKernel<TetP1Basis, TetP1DefaultQuadrature>`
  - `VolumetricKernel<HexTrilinearBasis, GaussLegendreHexQuadrature2>`
- [x] Keep kernels free of `ElasticModel`, `PlasticModel`, SVD, stress, material parameters, and energy accumulation.
- [x] Do not expose quadrature choice in Python or public factory options in this milestone; quadrature is fixed in each concrete `VolumetricFormulation` subclass.
- [x] Preserve legacy reference-domain and weight conventions exactly:
  - tet P1 uses the existing legacy tetrahedron orientation/volume convention from `TetMeshDeformationModel::computeVolume`;
  - hex trilinear uses the existing `[0, 1]^3` reference cube, node ordering from `CubicMeshDeformationModel`, 2x2x2 Gauss points at `0.5 +/- 0.5 / sqrt(3)`, reference weight `1/8` per point, and `abs(detJ) * weight`;
  - any behavior change in orientation, quadrature weight, node ordering, or local DOF ordering is a regression unless an old-vs-new test proves it is numerically identical.
- [x] Kernel API must expose at least:
  - `static constexpr int numNodes`
  - `static constexpr int localDofs`
  - `int numQuadraturePoints() const`
  - `double weightDetJ(int q) const`
  - `void computeFref(const double *xLocal, int q, double F[9]) const`
  - `void computedFrefdx(int q, double *dFdx) const`
- [x] Add kernel unit tests:
  - rest state gives `Fref == I`.
  - uniform translation leaves `Fref` unchanged.
  - affine deformation `x = A X + b` gives `Fref == A`.
  - quadrature weight sum equals element volume.
  - `computedFrefdx` matches finite difference.
- [x] Add basis/quadrature unit tests:
  - basis partition of unity;
  - `sum_i dN_i/dxi == 0`;
  - nodal interpolation property where applicable;
  - hex quadrature integrates constants and linear reference functions exactly;
  - tet default quadrature weight matches reference tet volume convention used by the legacy element.
  These checks may live in `tetP1Basis_gtest.cpp`, `hexTrilinearBasis_gtest.cpp`, or `volumetricKernel_gtest.cpp`; do not require standalone quadrature test files unless the coverage is otherwise missing.
- [x] Add `VolumetricElementModel` that implements the existing `DeformationModel` virtual interface by combining:
  - formulation kinematics from `Kernel`
  - `ElasticModel3DDeformationGradient`
  - `PlasticModel3DDeformationGradient`
  - existing plastic/material derivative paths
- [x] Add regression tests comparing old and new element models on the same tet and cubic single-element fixtures:
  - energy
  - `compute_dE_dx`
  - `compute_d2E_dx2`
  - `compute_d2E_dxda`
  - `compute_d2E_dxdb`
  - stress routines where supported
  - local material max-step where supported
- [x] Turn `TetMeshDeformationModel` into a compatibility wrapper around `VolumetricElementModel<VolumetricKernel<TetP1Basis, TetP1DefaultQuadrature>>` after regression tests pass.
- [x] Turn `CubicMeshDeformationModel` into a compatibility wrapper around `VolumetricElementModel<VolumetricKernel<HexTrilinearBasis, GaussLegendreHexQuadrature2>>` after regression tests pass.
- [x] Keep the full `examples/ipc/cubic/box/box-ipc.json` run as final smoke only; do not use it as the first regression guard.

**Exit criteria:**

- `TetP1Basis`, `HexTrilinearBasis`, default quadrature, and `VolumetricKernel` unit tests pass.
- New shared element model matches old tet and cubic element outputs within tight numerical tolerance.
- Old public element-model class names can still be used internally during migration, but their implementations delegate to the new shared element model.
- `P1TetFormulation` and `LinearCubicFormulation` can be constructed with real `Basis`, `Quadrature`, and `Kernel` types without referencing old full element models.

## Task 2: Split Deformation Factory By Topology

**目标：** 移除旧的 runtime auto-dispatch `makeDeformationModel(...)`，将 formulation 入口与 topology 关联。最终落地方案为 class hierarchy（`Formulation` → `VolumetricFormulation`/`ShellFormulation` → 具体类）和 unified factory `makeDeformationEnergy()`。本任务只拆 topology/formulation 入口，material 输入继续临时使用 legacy `DeformationModelElasticMaterial`。

> **Note:** 最初设计为 tag struct + `FormulationTraits<T>` 模板 + per-topology factory。实施中演变为 class hierarchy + unified factory。原 `formulationTraits.h`、`formulationConcepts.h`、`deformationFormulations.h` 已删除。

**Current status (2026-05-28):** Complete. Factory split to class hierarchy done, formulation metadata (`getName()`, `getNodesPerElement()`, `getLocalDofs()`) available on each concrete class.

**Files:**

- Create: `src/core/solidDeformationModel/formulations/deformationFormulations.h`
- Create: `src/core/solidDeformationModel/formulations/formulationTraits.h`
- Create: `src/core/solidDeformationModel/formulations/formulationConcepts.h`
- Create: `src/core/solidDeformationModel/formulations/dof/dofLayout.h`
- Create: `src/core/solidDeformationModel/formulations/dof/vertex3DofLayout.h`
- Modify: `src/core/solidDeformationModel/deformationModelFactory.h`
- Modify: `src/core/solidDeformationModel/deformationModelFactory.cpp`
- Modify: `src/core/solidDeformationModel/deformationModelManager.h`
- Modify: `src/core/solidDeformationModel/deformationModelManager.cpp`
- Modify: `src/c/pgo_c.cpp`
- Modify: `src/tools/sim/runIPCSim/setup/femSetup.h`
- Modify: `src/tools/sim/runIPCSim/setup/femSetup.cpp`
- Modify: `tests/src/tools/runSimShared_gtest.cpp`
- Create: `tests/src/core/solidDeformationModel/formulations/deformationModelFormulation_gtest.cpp`
- Modify: `src/core/solidDeformationModel/CMakeLists.txt`
- Modify: `tests/src/core/solidDeformationModel/CMakeLists.txt`

- [x] Add formulation concrete classes: `P1TetFormulation`, `LinearCubicFormulation`, `KoiterShellFormulation`.
- [x] Implement `Formulation` abstract base with `getName()`, `getNodesPerElement()`, `getLocalDofs()`.
- [x] Implement `VolumetricFormulation` (holds `Basis` + `Quadrature`) and `ShellFormulation` (creates `ShellKernel`).
- [x] Add `TetFormulation` and `CubicFormulation` intermediate tag classes.
- [x] Basis/Quadrature/Kernel/ElementModel 已去模板化为虚函数 dispatch。Factory 接收 `const Formulation &` 基类引用做 runtime dispatch。
- [x] Remove public `makeDeformationModel(...)` overloads.
- [x] Add unified factory `makeDeformationEnergy(const SimulationMesh &, const Formulation &, ...)`.
- [x] Manager-side element construction migrated to `ElementModelFactory` in Task 5.
- [x] Add factory bundle initialization snapshots for both `plasticParams` and `elasticParams`; volumetric deformation-gradient plastic initializes identity `Fp`, while shell Koiter uses zero/default shell plastic params.
- [x] Update construction logs/errors to include formulation names from `Formulation::getName()`, e.g. `hex_trilinear`.
- [x] Add C++ tests for all currently implemented topology/formulation rows.

**Exit criteria:**

- `makeDeformationEnergy(mesh, P1TetFormulation{}, ...)` works for tet.
- `makeDeformationEnergy(mesh, LinearCubicFormulation{}, ...)` works for cubic.
- `makeDeformationEnergy(mesh, KoiterShellFormulation{}, ...)` works for shell.
- Old `makeDeformationModel(...)` public entry is gone from the header.
- Formulation class hierarchy provides runtime dispatch via `dynamic_cast`.

## Task 3: Make `DeformationModelManager` Borrow `const SimulationMesh &`

**目标：** Python `SimulationMesh` 可以被多个 energy factory 调用复用；mesh ownership 保留在外层 root，manager / assembler / energy 链只借用 immutable `SimulationMesh`，不消费 unique ownership。

**Current status (2026-05-28):** Complete. C++ core borrow-only semantics have been applied. `DeformationModelManager` now takes `const SimulationMesh &`, `DeformationModelManagerImpl::ownedMesh` has been removed, topology-specific C++ factories take `const SimulationMesh &`, and `runIPCSim` / C API call sites keep explicit mesh owners outside the energy chain. `SimulationMeshCore` has been moved to a shared binding header and exposes `mesh() const` for future energy bindings. Energy-specific Python lifetime tests are deferred with the public/private energy binding work in Task 4 / Finalize.

**Files:**

- Modify: `src/core/solidDeformationModel/deformationModelManager.h`
- Modify: `src/core/solidDeformationModel/deformationModelManager.cpp`
- Modify: `src/core/solidDeformationModel/deformationModelFactory.h`
- Modify: `src/core/solidDeformationModel/deformationModelFactory.cpp`
- Modify: `src/c/pgo_c.cpp`
- Modify: `src/tools/sim/runIPCSim/setup/femSetup.h`
- Modify: `src/tools/sim/runIPCSim/setup/femSetup.cpp`
- Modify: `src/tools/sim/runIPCSim/setup/setup.h`
- Modify: `src/tools/sim/runIPCSim/setup/shellSetup.cpp`
- Modify: `src/tools/sim/runIPCSim/setup/volumeSetup.cpp`
- Modify: `src/tools/sim/runIPCSim/setup/legacySetup.cpp`
- Create: `src/python/pypgo/bindings/simulation_mesh_core.h`
- Modify: `src/python/pypgo/bindings/mesh_bindings.cpp`
- Modify: `tests/src/core/solidDeformationModel/deformationModelFactory_gtest.cpp`
- Modify: `tests/src/core/solidDeformationModel/deformationModelAssembler_gtest.cpp`
- Modify: `tests/src/core/solidDeformationModel/deformationModelEnergyMaxStep_gtest.cpp`
- Modify: `tests/pypgo/test_simulation_mesh.py`

- [x] Change `DeformationModelManager` constructor to take `const SimulationMesh &simulationMesh`; do not keep a public owning `std::unique_ptr<SimulationMesh>` constructor.
- [x] Remove `DeformationModelManagerImpl::ownedMesh`; store only `const SimulationMesh *simulationMesh = nullptr` as a non-owning immutable borrow.
- [x] Add null-free construction semantics: prefer reference in public constructors/factories; only internal helper pointers may exist after construction.
- [x] Update topology-specific factories to accept `const SimulationMesh &` for already solver-ready meshes.
- [x] Remove C++ convenience factories that both create a temporary `SimulationMesh` and build an energy. C++ core factories do not create hidden mesh owners; `makeSimulationMesh(...)` returns a `std::unique_ptr<SimulationMesh>`, and the caller/boundary object must keep that owner alive while any borrowed energy chain exists.
- [x] Update all in-repo manager construction call sites to keep the `SimulationMesh` owner outside the manager until the energy chain is destroyed.
- [x] Move `SimulationMeshCore` out of `mesh_bindings.cpp` into `src/python/pypgo/bindings/simulation_mesh_core.h` so `energy_bindings.cpp` can use the same C++ wrapper type.
- [x] Add `SimulationMeshCore::mesh() const -> const SimulationMesh &` in that shared header for energy construction.
- [x] Defer the `DeformationEnergyCore` `std::shared_ptr<SimulationMeshCore>` hold to Task 4 private smoke hook or the Finalize public energy binding; Task 3 provides the shared owner type needed by that binding.
- [x] Do not expose C++ mesh ownership or lifetime controls as public Python methods.
- [x] Add C++ tests: one `SimulationMesh` owner can be used to construct two independent deformation energies, and both remain evaluable while the owner is alive.
- [x] Defer Python energy lifetime tests to Task 4 private smoke hook or the Finalize public energy API, because public `pypgo.energy` is intentionally not committed in Task 3.

**Exit criteria:**

- `SimulationMeshCore` owns the C++ mesh and exposes a borrow-only `mesh() const` accessor for future energy bindings.
- C++ can create two independent energy chains from the same `SimulationMesh` owner while the owner remains alive.
- Manager / assembler / energy construction no longer requires moving a `std::unique_ptr<SimulationMesh>` into `DeformationModelManager`.

## Task 4: Defer Public Python Deformation API Until C++ Boundaries Stabilize

**目标：** 不在 C++ formulation/lifetime/DOF/parameter-field 边界仍处于过渡态时发布 `pypgo.fem` / `pypgo.energy` public API。Task 4 只允许做 Python lifetime bridge 的技术准备和 private `_core` smoke hook；最终 public Python API 在 Finalize 任务统一定稿。

**Files:**

- Optional create: `src/python/pypgo/bindings/energy_bindings.cpp`
- Modify: `src/python/pypgo/bindings/simulation_mesh_core.h`
- Modify: `src/python/pypgo/bindings/module.cpp`
- Modify: `src/python/pypgo/CMakeLists.txt`
- Do not create public `pypgo/fem.py` in Task 4
- Do not create public `pypgo/energy.py` in Task 4
- Modify: `pypgo/__init__.py`
- Modify: `pypgo/sim.py`
- Optional create: `tests/pypgo/test_deformation_energy_private.py`

- [x] Record that public `pypgo.fem` / `pypgo.energy` API is deferred until after formulation, lifetime, `DofLayout`, and `ParameterField` boundaries are stable.
- [x] Do not expose `StableNeo(material_slot=0)`, `StVK(material_slot=0)`, `KoiterStVK()`, or any other public Python recipe class in Task 4.
- [x] If a private smoke hook is needed, add topology-specific `_core` formulation payloads or parser helpers for:
  - `tet_p1`
  - `hex_trilinear`
  - `shell_koiter`
- [x] Use the shared `SimulationMeshCore` declaration from `simulation_mesh_core.h`; do not duplicate or forward-declare a private class in `energy_bindings.cpp`.
- [x] If a private smoke hook is added, map private payloads to C++ runtime variants at the binding boundary, then call the variant adapter.
- [x] Optional private `_core` hooks may exist only for regression/smoke validation and must be named as private/experimental, for example `_core._create_tet_deformation_energy_for_test(...)`.
- [x] If implemented, add `DeformationEnergyCore` with:
  - `num_dofs()`
  - `rest_position_flat()`
  - `zero_state()`
  - `value(u)`
  - `gradient(u)`
  - `hessian(u)` returning `SparseMatrixCore`
  - optional `max_step(u, du)`
- [x] Release the GIL around energy/gradient/hessian computations.
- [x] Validate NumPy input shape:
  - `u.shape == (num_dofs,)`
  - dtype coerces to `float64` in Python wrapper
  - contiguous array passed to `_core`
- [x] Do not add `pypgo.fem` formulation dataclasses in Task 4.
- [x] Do not add `pypgo.energy.deformation_energy(...)` in Task 4.
- [x] If private smoke hooks are added, tests must live under a private/experimental test name and must not document public Python API behavior.
- [x] Keep the lifetime test from Task 3: same `SimulationMeshCore` can create two private energies, and an energy remains usable after the Python `sim_mesh` variable is deleted because `DeformationEnergyCore` keeps `SimulationMeshCore` alive.

**Exit criteria:**

- No public `pypgo.fem` / `pypgo.energy` deformation API is committed in this task.
- Any private binding hook is clearly private/experimental and exists only to validate lifetime/state mechanics.
- Public Python API decisions are deferred to Finalize after C++ formulation, lifetime, DOF, and parameter-field boundaries are stable.

## Task 5: Split `DeformationModelManager::initImpl` Into Internal Factories

**目标：** 降低 manager 的职责复杂度，为 future formulation 增量接入准备边界。这一步必须 behavior-preserving：先把现有 legacy elastic/plastic enum 的分支搬进内部 factory，不同时引入 material recipe 语义迁移。

`DeformationModelManager` 在 Task 5 结束时仍可保留为 legacy model-storage/query object；不要在同一任务中做 rename/remove。真正的边界变化是：creation policy 进入 `ElasticModelFactory` / `PlasticModelFactory` / `ElementModelFactory`，manager 不再直接承载所有 dispatch 细节。Assembler 仍通过 manager 查询 element model，直到 Task 6 把 DOF gather/scatter/sparsity 迁入 `DofLayout`。

**Files:**

- Create: `src/core/solidDeformationModel/factories/elasticModelFactory.h`
- Create: `src/core/solidDeformationModel/factories/elasticModelFactory.cpp`
- Create: `src/core/solidDeformationModel/factories/plasticModelFactory.h`
- Create: `src/core/solidDeformationModel/factories/plasticModelFactory.cpp`
- Create: `src/core/solidDeformationModel/factories/elementModelFactory.h`
- Modify: `src/core/solidDeformationModel/formulations/elements/volumetricElementModel.h`
- Modify: `src/core/solidDeformationModel/deformationModelManager.cpp`
- Modify: `src/c/pgo_c.cpp`
- Modify: `src/tools/sim/runIPCSim/setup/femSetup.cpp`
- Modify: `tests/src/tools/runSimShared_gtest.cpp`
- Modify: `src/core/solidDeformationModel/CMakeLists.txt`
- Modify: `tests/src/core/solidDeformationModel/deformationModelFactory_gtest.cpp`
- Modify: `tests/src/core/solidDeformationModel/deformationModelAssembler_gtest.cpp`

- [x] Extract elastic model creation into `ElasticModelFactory`.
- [x] Extract plastic model creation into `PlasticModelFactory`.
- [x] Extract element FEM creation into `ElementModelFactory`.
- [x] Keep `DeformationModelManager` as the temporary owner/query surface for per-element models; do not rename or remove it in Task 5.
- [x] Move construction decisions out of `DeformationModelManager::initImpl`; keep runtime lookup methods such as `getDeformationModel(eleID)` on manager until a later model-set cleanup.
- [x] Consolidate remaining direct manager/assembler/energy construction call sites through the topology-specific factory or its new internal factory helpers, notably `src/c/pgo_c.cpp`, `src/tools/sim/runIPCSim/setup/femSetup.cpp`, and `tests/src/tools/runSimShared_gtest.cpp`.
- [x] Use the existing `VolumetricElementModel` wrapper from Task 1 for both `VolumetricKernel<TetP1Basis, TetP1DefaultQuadrature>` and `VolumetricKernel<HexTrilinearBasis, GaussLegendreHexQuadrature2>`.
- [x] Make `ElementModelFactory` construct `VolumetricElementModel` with runtime `ElasticModel *` and `PlasticModel *` dependencies supplied by the factories.
- [x] Do not instantiate `ElementModel<Kernel, StableNeo, Plastic6Dof>`-style combinations in the first version; avoid material/plastic template explosion.
- [x] Keep `ElasticModelFactory` consuming legacy `DeformationModelElasticMaterial` in Task 5 so this extraction can be tested independently of material recipe migration.
- [x] Do not remove `DeformationModelElasticMaterial` from the factory call path in Task 5; Task 7 performs that public API migration after payload conversion and mismatch tests exist.
- [x] Pass `const Formulation &` into `ElementModelFactory` for runtime dispatch.
- [x] Move default `plasticParams` / `elasticParams` snapshot initialization out of the topology factory helper and into focused creation helpers near `PlasticModelFactory` / `ElasticModelFactory`; preserve Task 2 behavior exactly:
  - volumetric deformation-gradient plastic params encode identity `Fp`;
  - shell Koiter plastic params stay at their zero/default stretch state;
  - shell STVK elastic params map `SimulationMeshENuhMaterial` to `(E, nu, E_bend, nu_bend, h)`.
- [x] Update construction logs/errors to include formulation names from `Formulation::getName()`, e.g. `hex_trilinear`, instead of only topology names such as `CUBIC`.
- [x] Preserve the current manager-owned material/model storage shape initially:
  - vectors of specific material/model pointer types may remain in `DeformationModelManagerImpl`.
  - `elementMaterials` and `elementFEMs` can stay borrowed pointer arrays.
- [x] Move owner-vector cleanup to Task 5p; Task 5 keeps the existing pointer-vector storage while extraction/parity tests are being validated.
- [x] Add focused tests that compare energy/gradient/Hessian before and after extraction for tet and cubic.

**Exit criteria:**

- `initImpl` reads as orchestration:
  - compute fiber axes
  - allocate storage
  - loop elements
  - create elastic
  - create plastic
  - create element model
- Element formulation selection exists in one place, not scattered through material/plastic creation.
- Elastic material selection exists in one factory, still using the legacy enum until Task 7.
- `Formulation` class hierarchy selects the kernel and element-model wrapper; material/plastic selection remains runtime-injected by the factories.

## Task 5e: Add Shell Koiter Formulation Stack

**目标：** 把 shell Koiter 也接入 formulation-driven element construction，但不把它硬塞进 volumetric `Basis` / `Quadrature` / `VolumetricKernel`。Shell 需要一套平行抽象：compile-time stencil topology + per-element fundamental-forms kernel + Koiter shell element model。Task 5e 完成后，`ShellKoiter` 不再只是 factory routing tag，而是拥有自己的 shell kernel/model stack，可在 Task 5p 中替代 `KoiterDeformationModel` runtime path。

### Design decision

Volumetric deformation-gradient stack（参考点）：

```text
Basis                              # compile-time: N(xi), dN/dxi
  -> Quadrature                    # compile-time: xi_q, w_q
  -> VolumetricKernel     # per-element: restX, weightDetJ(q), Fref(x)
  -> VolumetricElementModel
```

Shell Koiter stack 在 Task 5e 后的最终形态：

```text
ShellKernel (abstract base)         # virtual: compute_a/b_and_derivatives, restI/II, restArea, hasVtx
  -> KoiterShellKernel              # per-element: restX[6], hasVtx[6], restI, restII, restArea
                                    # + a/b differential geometry (extracted from KoiterDeformationModelInternal)
  -> ShellElementModel              # holds unique_ptr<ShellKernel> + ElasticModel2DFundamentalForms* + PlasticModel2DFundamentalForms*
```

### Class hierarchy dispatch

Formulation dispatch 使用 class hierarchy + 虚函数（详见设计决策 2-3）。`VolumetricFormulation` 持有 `Basis` + `Quadrature`；`ShellFormulation` 创建 `ShellKernel`。Factory 内部通过 `dynamic_cast<const VolumetricFormulation *>` / `dynamic_cast<const ShellFormulation *>` 分发。

### Concrete responsibility split

| 数据 / 方法 | 来源 (今天) | 去向 (Task 5e 后) |
|---|---|---|
| `oppVtx[3] = {4, 5, 3}` | `KoiterDeformationModelInternal` (instance field) | `KoiterShellKernel::oppVtx` (`static constexpr`) |
| numNodes = 6, localDofs = 18 | hard-coded in `KoiterDeformationModel::getNumVertices/DOFs` | `KoiterShellKernel::numNodes` / `::localDofs` (`static constexpr`) |
| `restX[6]`, `hasVtx[6]`, `restI`/`restII` | `KoiterDeformationModelInternal` | `KoiterShellKernel` 构造时计算并保存 |
| rest `area`（喂给 plastic via `setArea`） | `KoiterDeformationModel` ctor | `KoiterShellKernel::restArea()` |
| `compute_a_and_derivatives` | `KoiterDeformationModelInternal` 方法 | `KoiterShellKernel::compute_a_and_derivatives` |
| `compute_b_and_derivatives` | 同上 | `KoiterShellKernel::compute_b_and_derivatives` |
| `secondFundamentalFormEntries` | 同上 | `KoiterShellKernel::secondFundamentalFormEntries` (private helper) |
| `faceNormal` | 同上 | `KoiterShellKernel::faceNormal` (private helper) |
| `KoiterDeformationModelCacheData` (x[6], a/abar/b/bbar/area, elasticParams, plasticParams) | `KoiterDeformationModel` | `ShellElementModelCacheData` |
| `prepareData` / `computeEnergy` / `compute_dE_dx` / `compute_d2E_dx2` / `compute_d2E_dxda` / `compute_d2E_dxdb` / `computeLocalMaxStepSize` / `enableSPD` | `KoiterDeformationModel` overrides | `ShellElementModel` overrides |
| `set_abar` / `set_bbar` / `setArea` on plastic model | `KoiterDeformationModel` ctor side effect | `ShellElementModel` ctor side effect (移植) |
| `-10496` sentinel detection | `KoiterDeformationModel` ctor + factory + `DeformationModelManager` populates fake positions | `ElementModelFactory` builds explicit `bool hasVtx[6]` from `mesh.getVertexIndex(ele, j) < 0`；不再有 sentinel |

### Target API sketch (actual implementation)

```cpp
// formulations/kernels/shellKernel.h — abstract base
class ShellKernel {
public:
  virtual ~ShellKernel() = default;
  virtual int getNumNodes() const = 0;
  virtual int getLocalDofs() const = 0;
  virtual ES::M2d compute_a_and_derivatives(...) const = 0;
  virtual ES::M2d compute_b_and_derivatives(...) const = 0;
  virtual const ES::M2d &restI() const = 0;
  virtual const ES::M2d &restII() const = 0;
  virtual double restArea() const = 0;
  virtual const bool *hasVtx() const = 0;
};

// formulations/kernels/koiterShellKernel.h — concrete Koiter kernel
class KoiterShellKernel : public ShellKernel {
public:
  static constexpr int numNodes = 6;
  static constexpr int localDofs = 18;
  static constexpr int oppVtx[3] = { 4, 5, 3 };

  KoiterShellKernel(const double restX[18], const bool hasVtx[6]);
  // ... full Koiter differential geometry
};

// formulations/elements/shellElementModel.h — generic shell element
class ShellElementModel : public DeformationModel {
public:
  ShellElementModel(std::unique_ptr<ShellKernel> kernel,
    ElasticModel *elasticModel, PlasticModel *plasticModel);
  // ... DeformationModel virtuals
private:
  std::unique_ptr<ShellKernel> kernel_;
  ElasticModel2DFundamentalForms *elasticModel_ = nullptr;
  PlasticModel2DFundamentalForms *plasticModel_ = nullptr;
};
```

### Files

- Create: `src/core/solidDeformationModel/formulations/stencil/shellKoiterStencil.h`（header-only，仅 `static constexpr` 拓扑常量）
- Create: `src/core/solidDeformationModel/formulations/kernels/fundamentalFormsKernel.h`（header-only template，从 `koiterDeformationModel.cpp` 搬出微分几何）
- Create: `src/core/solidDeformationModel/formulations/elements/koiterShellElementModel.h`（header-only template，与 `VolumetricElementModel` 同形）
- Modify: `src/core/solidDeformationModel/formulations/formulationTraits.h`
- Modify: `src/core/solidDeformationModel/factories/elementModelFactory.h`
- Modify: `src/core/solidDeformationModel/deformationModelManager.cpp`（去掉 fill-with-`-10496` 的预处理，改成把 `getVertexIndex < 0` 信息直接交给 factory；或保留 fill 但 factory 读 mask 而非 sentinel——见 Sub-task F）
- Modify: `src/core/solidDeformationModel/CMakeLists.txt`（加 3 个 header）
- Create: `tests/src/core/solidDeformationModel/formulations/stencil/shellKoiterStencil_gtest.cpp`（trivial topology metadata 检查；可选）
- Create: `tests/src/core/solidDeformationModel/formulations/kernels/fundamentalFormsKernel_gtest.cpp`
- Create: `tests/src/core/solidDeformationModel/formulations/elements/koiterShellElementModel_gtest.cpp`
- Modify: `tests/src/core/solidDeformationModel/factories/elementModelFactory_gtest.cpp`
- Modify: `tests/src/core/solidDeformationModel/deformationModelFactory_gtest.cpp`
- Modify: `tests/src/core/solidDeformationModel/CMakeLists.txt`

### Sub-task A: ShellKernel abstract base

- [x] Create `formulations/kernels/shellKernel.h` as abstract base class for shell kinematics.
- [x] `KoiterShellKernel` inherits from `ShellKernel` — constants (`numNodes=6`, `localDofs=18`, `oppVtx={4,5,3}`) are direct static constexpr members.
- [x] Kernel and element model are non-template classes.
- [x] Parity tests cover `getNumVertices() == 6` / `getNumDOFs() == 18`.

### Sub-task B: KoiterShellKernel

- [x] Create `formulations/kernels/koiterShellKernel.h/.cpp` — concrete Koiter thin-shell kernel inheriting `ShellKernel`.
- [x] Move differential geometry methods from `koiterDeformationModel.cpp` into `KoiterShellKernel`:
  - `compute_a_and_derivatives`, `compute_b_and_derivatives`, `secondFundamentalFormEntries`, `faceNormal`, `crossMatrix`
- [x] Constructor copies positions + mask, computes and caches `restI_`, `restII_`, `restArea_`.
- [x] Kernel does not depend on `ElasticModel` / `PlasticModel` / `DeformationModelCacheData`.
- [x] Kernel unit tests in `koiterShellKernel_gtest.cpp`: rest state, translation invariance, affine map, missing neighbor, derivative FD, Hessian FD.

### Sub-task C: ShellElementModel

- [x] Create `formulations/elements/shellElementModel.h/.cpp` — non-template class holding `std::unique_ptr<ShellKernel>`.
- [x] Cache data `ShellElementModelCacheData` mirrors legacy cache 1:1.
- [x] Constructor: `dynamic_cast` elastic/plastic to `ElasticModel2DFundamentalForms*` / `PlasticModel2DFundamentalForms*`.
- [x] Constructor preserves legacy plastic-seed side effect (`set_abar`, `set_bbar`, `setArea`).
- [x] Implement all `DeformationModel` virtuals via kernel delegation.

### Sub-task D: Formulation class + ElementModelFactory wiring

- [x] Create `KoiterShellFormulation : public ShellFormulation` in `formulation.h/.cpp`:
  - `getName() = "shell_koiter"`, `getNodesPerElement() = 6`, `getLocalDofs() = 18`
- [x] `ShellFormulation::createKernel()` returns `std::unique_ptr<ShellKernel>` (当前固定为 `KoiterShellKernel`).
- [x] Update `ElementModelFactory::create()` to dispatch via `dynamic_cast<const ShellFormulation *>`:
  - build `bool hasVtx[6]` from `mesh.getVertexIndex(ele, j) < 0`（不再读 `-10496` sentinel）
  - construct `ShellElementModel` with `KoiterShellKernel`
  - keep `KOITER_FABRIC` / `KOITER_STVK` guard
- [x] Drop `#include "../koiterDeformationModel.h"` from `elementModelFactory.h`.
- [x] `KoiterDeformationModel` 仍在 `koiterDeformationModel.h/.cpp`，但 production factory 不再实例化它。

### Sub-task E: Element-model parity tests

- [x] Create `tests/.../formulations/elements/shellElementModel_gtest.cpp` with interior and boundary triangle fixtures.
- [x] Construct both `KoiterDeformationModel` (oracle) and `ShellElementModel` (new) for parity comparison.
- [x] At rest and perturbed displacement, compare energy, gradient, Hessian, `d2E/dxda`, `d2E/dxdb` to `1e-10`.
- [x] Add SPD enable test and FD sanity checks.

### Sub-task F: Manager / factory wiring for the mask

- [x] Remove `-10496` sentinel fill from manager; factory builds `hasVtx` from mesh directly.
- [x] `-10496` references removed from `deformationModelManager.cpp` and `elementModelFactory.h`.

### Sub-task G: Factory + assembler smoke tests

- [x] Extend `elementModelFactory_gtest.cpp`: verify `ShellElementModel` type via `dynamic_cast`, assert factory throws for non-Koiter elastic materials.
- [x] Extend `deformationModelFactory_gtest.cpp` shell case: end-to-end `makeDeformationEnergy(mesh, KoiterShellFormulation{}, ...)` smoke test.
- [x] Do not delete legacy oracle test path in Task 5e; Task 5q removes oracle dependencies.

**Exit criteria:**

- `ShellKernel`, `KoiterShellKernel`, and `ShellElementModel` exist as non-template classes under `formulations/`, matching the file layout in design decision 4.
- `KoiterShellFormulation` provides `createKernel()` returning `KoiterShellKernel`.
- `ElementModelFactory` returns `ShellElementModel` for shell elements via `dynamic_cast<const ShellFormulation *>` dispatch.
- Kernel unit tests cover rest state, translation invariance, affine map, missing-neighbor mask, and FD checks.
- Parity tests vs `KoiterDeformationModel` pass to `1e-10`.
- `-10496` no longer appears in `deformationModelManager.cpp` or `elementModelFactory.h`.
- `KoiterDeformationModel` source files remain in tree as oracle until Task 5q.

## Task 5p: Task 5 Closeout Cleanup

**目标：** 去掉 Task 5 留下的过渡胶水，让 production runtime path 不再先构造 legacy tet/cubic/shell element wrapper 再替换成 formulation-aware element model。本任务不直接删除 `TetMeshDeformationModel` / `CubicMeshDeformationModel` / `KoiterDeformationModel` 文件；先只移除 manager/factory runtime path 对它们的依赖。真正删除 legacy wrappers 在 Task 5q 完成。

**Dependencies:**

- Task 5e 已完成，`ShellKoiter` 已有 shell-specific `Kernel` / `ElementModel`，不再需要 `KoiterDeformationModel` 作为 runtime implementation。

**Current issue (2026-05-28):** `makeDeformationEnergy(mesh, P1TetFormulation{}, ...)` / `makeDeformationEnergy(mesh, LinearCubicFormulation{}, ...)` 已经通过 `Formulation` class hierarchy 构造最终 element model。但实现路径仍然可能是：

```text
makeDeformationEnergy(mesh, formulation, ...)
  -> DeformationModelManager::initImpl(...)
       -> first creates legacy element wrapper
  -> factory loop
       -> replaces each element with ElementModelFactory::create(...)
```

这会多构造一遍 legacy element，并且让 `DeformationModelManager::initImpl` 继续承担 topology-to-element-model dispatch。Task 5p 要把这段 runtime 过渡路径拆掉。

**Files:**

- Modify: `src/core/solidDeformationModel/deformationModelManager.h`
- Modify: `src/core/solidDeformationModel/deformationModelManager.cpp`
- Modify: `src/core/solidDeformationModel/deformationModelFactory.h`
- Modify: `src/core/solidDeformationModel/deformationModelFactory.cpp`
- Modify: `src/core/solidDeformationModel/factories/elementModelFactory.h`
- Modify: `src/core/solidDeformationModel/CMakeLists.txt`
- Modify: `tests/src/core/solidDeformationModel/factories/elementModelFactory_gtest.cpp`
- Modify: `tests/src/core/solidDeformationModel/deformationModelFactory_gtest.cpp`
- Modify: `tests/src/core/solidDeformationModel/formulations/deformationModelFormulation_gtest.cpp`
- Optional modify: `tests/src/core/solidDeformationModel/deformationModelAssembler_gtest.cpp`

- [x] Make `DeformationModelManager::initImpl` create element FEMs through `ElementModelFactory` directly:
  - `SimulationMeshType::TET` -> `ElementModelFactory::create(..., P1TetFormulation{})`;
  - `SimulationMeshType::CUBIC` -> `ElementModelFactory::create(..., LinearCubicFormulation{})`;
  - `SimulationMeshType::SHELL` -> `ElementModelFactory::create(..., KoiterShellFormulation{})`.
- [x] Remove direct `new TetMeshDeformationModel(...)`, `new CubicMeshDeformationModel(...)`, and `new KoiterDeformationModel(...)` from `DeformationModelManager::initImpl`.
- [x] Remove legacy element wrapper includes from `deformationModelManager.cpp`.
- [x] Remove the formulation-aware factory replacement loop in `makeDeformationEnergy()`:
  - do not create a temporary legacy element and then replace it.
- [x] Remove `DeformationModelManager::setDeformationModel(...)` if it has no remaining production caller.
- [x] Add/adjust tests proving public factories no longer depend on legacy wrappers:
  - `makeDeformationEnergy(mesh, P1TetFormulation{}, ...)` returns element models that are NOT `TetMeshDeformationModel`;
  - `makeDeformationEnergy(mesh, LinearCubicFormulation{}, ...)` returns element models that are NOT `CubicMeshDeformationModel`;
  - `makeDeformationEnergy(mesh, KoiterShellFormulation{}, ...)` returns element models that are NOT `KoiterDeformationModel`.
- [x] Preserve existing parity tests against legacy wrappers until replacement confidence is high; those tests may continue to instantiate `TetMeshDeformationModel` / `CubicMeshDeformationModel` / `KoiterDeformationModel` as oracle objects.
- [x] Do not delete `tetMeshDeformationModel.h/.cpp`, `cubicMeshDeformationModel.h/.cpp`, or `koiterDeformationModel.h/.cpp` in Task 5p. They are still used by:
  - oracle/parity tests;
  - `TetMeshDeformationModel::computeDs`, `computeDm`, `compute_dF_dx`, and related geometry helpers;
  - older constraint/FD/shell parity utilities that have not been migrated to formulation geometry helpers.
- [x] Optionally replace manager-owned raw pointer vectors with `std::unique_ptr` vectors after the runtime legacy dependency is gone. Keep this as a separate substep in the same task and do not mix it with behavior changes.

**Exit criteria:**

- Public topology-specific factories construct the final formulation-aware element models without first constructing legacy tet/cubic wrappers.
- `DeformationModelManager::initImpl` no longer directly mentions `TetMeshDeformationModel`, `CubicMeshDeformationModel`, or `KoiterDeformationModel`.
- `ElementModelFactory` is the only production place that maps current topology/formulation defaults to concrete element model classes.
- All Task 5 factory/parity tests and Task 2/3 deformation factory tests still pass.
- Legacy wrapper files remain available only for oracle tests and unmigrated static geometry helpers; their physical deletion is handled by Task 5q.

## Task 5q: Delete Legacy Tet/Cubic/Shell Element Wrapper Files

**目标：** 在 production runtime path 已经完全使用 `ElementModelFactory` 和 formulation-aware element models 后，迁走 legacy wrapper 中仍被外部使用的几何 helper 和 oracle tests，最终删除 `TetMeshDeformationModel` / `CubicMeshDeformationModel` / `KoiterDeformationModel` 文件。这个任务必须 behavior-preserving；不能把删除 legacy 文件和新 formulation 行为变化混在一起。

**Dependencies:**

- Task 5e 已完成，shell Koiter 已有 formulation-aware shell element model。
- Task 5p 已完成，manager/factory runtime path 不再直接或间接构造 `TetMeshDeformationModel` / `CubicMeshDeformationModel` / `KoiterDeformationModel`。
- `VolumetricElementModel` (tet P1/cubic) 和 `ShellElementModel` (shell Koiter) 已通过 parity tests 锁住行为。

**Files:**

- Delete: `src/core/solidDeformationModel/tetMeshDeformationModel.h`
- Delete: `src/core/solidDeformationModel/tetMeshDeformationModel.cpp`
- Delete: `src/core/solidDeformationModel/cubicMeshDeformationModel.h`
- Delete: `src/core/solidDeformationModel/cubicMeshDeformationModel.cpp`
- Delete: `src/core/solidDeformationModel/koiterDeformationModel.h`
- Delete: `src/core/solidDeformationModel/koiterDeformationModel.cpp`
- Create: `src/core/solidDeformationModel/formulations/geometry/tetP1Geometry.h`
- Optional create: `src/core/solidDeformationModel/formulations/geometry/tetP1Geometry.cpp`
- Optional create: `src/core/solidDeformationModel/formulations/geometry/hexTrilinearGeometry.h`
- Modify: `src/core/solidDeformationModel/tetVolumeConstraintFunctions.cpp`
- Modify: `src/core/solidDeformationModel/prescribedPrincipleStressConstraintFunctions.cpp`
- Modify: `src/core/solidDeformationModel/deformationModelFDTest.cpp`
- Modify: `tests/src/core/solidDeformationModel/deformationModelEnergyMaxStep_gtest.cpp`
- Modify: `tests/src/core/solidDeformationModel/formulations/elements/volumetricElementModel_gtest.cpp`
- Modify: `tests/src/core/solidDeformationModel/formulations/elements/koiterShellElementModel_gtest.cpp`
- Delete or rewrite: `tests/src/core/solidDeformationModel/cubicMeshDeformationModel_gtest.cpp`
- Modify: `tests/src/tools/runSimShared_gtest.cpp`
- Modify: `src/core/solidDeformationModel/CMakeLists.txt`
- Modify: `tests/src/core/solidDeformationModel/CMakeLists.txt`

- [x] Move tet static geometry helpers out of `TetMeshDeformationModel` into `formulations/geometry/tetP1Geometry.h`:
  - `computeDs(...)`;
  - `computeDm(...)`;
  - `compute_dF_dx(...)`;
  - `computeVolume(...)`;
  - any small helper that constraints/FD tests still use directly.
  - **Post-implementation note:** `computeDm` and `computeVolume` were later deleted; `tetVolumeConstraintFunctions` now uses `VolumetricKernel` for rest-geometry DmInv/dFdx. Only `computeDs` and `computeDFDx` remain in `tetP1Geometry.h`.
- [x] Update all non-test callers of `TetMeshDeformationModel::compute*` to use the new geometry helper namespace/type.
- [x] Replace tests that dynamic-cast manager elements to `TetMeshDeformationModel` / `CubicMeshDeformationModel` with checks against formulation-aware behavior:
  - use `VolumetricElementModel` directly for tet/cubic;
  - prefer public energy/assembler behavior checks over concrete class casts where possible.
- [x] Rewrite `volumetricElementModel_gtest.cpp` so legacy wrappers are not the long-term oracle:
  - keep numerical golden values or kernel-level expected values where practical;
  - compare against hand-built `VolumetricElementModel` instances rather than old wrappers;
  - if temporary oracle coverage is still needed, move it to a short-lived compatibility test and remove it before deleting files.
- [x] Delete `cubicMeshDeformationModel_gtest.cpp` or rewrite it as `hexTrilinearElementModel_gtest.cpp`.
- [x] Rewrite shell parity tests so `KoiterDeformationModel` is no longer needed as an oracle; use golden values or direct kernel/model checks from Task 5e.
- [x] Remove `tetMeshDeformationModel.*`, `cubicMeshDeformationModel.*`, and `koiterDeformationModel.*` from `src/core/solidDeformationModel/CMakeLists.txt`.
- [x] Remove old wrapper headers from public/header install lists.
- [x] Run a repository-wide search for `TetMeshDeformationModel`, `CubicMeshDeformationModel`, and `KoiterDeformationModel`; after this task, no production or test file may reference those names.

**Exit criteria:**

- `rg "TetMeshDeformationModel|CubicMeshDeformationModel|KoiterDeformationModel" src tests` returns no references except possibly historical plan text.
- The solid deformation model library builds without compiling `tetMeshDeformationModel.cpp`, `cubicMeshDeformationModel.cpp`, or `koiterDeformationModel.cpp`.
- Tet/cubic/shell energy, gradient, Hessian, stress, max-step, FD, and constraint tests pass through formulation-aware implementations.
- Public C++ headers no longer expose legacy tet/cubic/shell element wrapper classes.

### Implementation notes (2026-05-29)

During Task 5p/5q implementation, three additional cleanups beyond the original checklist were applied:

**Cache data file extraction.** The per-element cache data structs were extracted from the element model headers into dedicated files to reduce header size and clarify ownership:

- `VolumetricElementModelCacheData` moved from `volumetricElementModel.h` into `formulations/elements/volumetricElementModelCacheData.h`.
- `ShellElementModelCacheData` moved from `shellElementModel.h` into `formulations/elements/shellElementModelCacheData.h`.

**`unique_ptr` migration for cache data.** `DeformationModel::allocateCacheData()` now returns `std::unique_ptr<DeformationModelCacheData>` instead of a raw pointer. `freeCacheData()` is removed. All call sites (assembler, constraint functions, FD test, gtest files) updated accordingly. No more manual `new`/`delete` for cache data.

**Tet geometry consolidation.** `tetVolumeConstraintFunctions` now uses `VolumetricKernel<TetP1Basis, TetP1DefaultQuadrature>` for rest-geometry DmInv and dFdx computation instead of standalone helpers. `tetP1Geometry.h` is reduced to two functions that the kernel cannot cover: `tetP1ComputeDs` (runtime Ds from deformed positions) and `tetP1ComputeDFDx` (dFdx from externally-supplied DmInv in `setDmInv`). `tetP1ComputeDm` and `tetP1ComputeVolume` are deleted — their logic now lives exclusively in the kernel.

## Task 6: Introduce `DofLayout` With `Vertex3DofLayout`

**目标：** 移除 assembler 对 `3 * numVertices` 和 `numElementVertices * 3` 的硬编码依赖，同时保持现有路径行为不变。Task 6 只迁移 DOF 侧的 gather/scatter/sparsity，不重写 shell 的几何邻居语义；shell 的 missing-neighbor 由 Task 5e 的 `KoiterShellKernel` 负责，`Vertex3DofLayout` 只处理 `vid < 0` 时的 DOF 边界行为。

**Files:**

- Modify: `src/core/solidDeformationModel/formulations/dof/dofLayout.h`
- Modify: `src/core/solidDeformationModel/formulations/dof/vertex3DofLayout.h`
- Create: `src/core/solidDeformationModel/formulations/dof/vertex3DofLayout.cpp`
- Modify: `src/core/solidDeformationModel/deformationModelAssembler.h`
- Modify: `src/core/solidDeformationModel/deformationModelAssembler.cpp`
- Modify: `src/core/solidDeformationModel/deformationModelFactory.h`
- Modify: `src/core/solidDeformationModel/deformationModelFactory.cpp`
- Modify: `src/core/solidDeformationModel/CMakeLists.txt`
- Create: `tests/src/core/solidDeformationModel/formulations/dof/vertex3DofLayout_gtest.cpp`
- Modify: `tests/src/core/solidDeformationModel/deformationModelAssembler_gtest.cpp`

- [x] Complete the `DofLayout` abstract interface that Task 2 introduced as a traits-visible declaration.
- [x] Complete `Vertex3DofLayout` so it borrows the same `const SimulationMesh *` that `DeformationModelManager` borrows from the outer owner.
- [x] Move `gatherLocalPositions(...)` logic from assembler helper into `Vertex3DofLayout::gather`; preserve `vid < 0` zero-local behavior as a DOF-side concern so missing-neighbor shell slots stay zero in the gathered local vector.
- [x] Add `Vertex3DofLayout::getGlobalDofIndices(ele, indices)`; it returns one entry per local displacement DOF, with `-1` for missing shell-neighbor slots. Assembler sparse-template code must use this index list rather than calling `mesh.getVertexIndex(...)` directly.
- [x] Move gradient scatter logic into `Vertex3DofLayout::scatterAddGradient`; `vid < 0` slots must skip global write-back.
- [x] Move Hessian sparsity construction into `Vertex3DofLayout::addHessianSparsity`; `vid < 0` slots must not generate global triplets.
- [x] Move local-to-global sparse index lookup into layout helper.
- [x] Do not move shell missing-neighbor geometry into `Vertex3DofLayout`. The `KoiterShellKernel` owns missing-neighbor handling; `Vertex3DofLayout` only sees the resulting per-DOF `vid` array.
- [x] Change `DeformationModelAssembler` constructor to own `std::unique_ptr<const DofLayout>`. `DofLayout` is an assembly concern, not a manager concern; keep the manager responsible for element/material/plastic model ownership and keep gather/scatter/sparsity policy in the assembler.
- [x] Store `deformationModelManager` before `dofLayout` in `DeformationModelAssembler` so `dofLayout` is destroyed first; both manager and layout borrow the same outer-owned immutable `SimulationMesh`.
- [x] Update all assembler construction call sites to pass an explicit `Vertex3DofLayout`; do not keep an implicit compatibility constructor.
- [x] Change assembler fields:
  - `n3` -> `numDOFs`
  - `localDOFs` becomes per-element query or cached from layout
  - `nvtx` only remains if needed for legacy diagnostics
- [x] Add tests comparing old expected DOF counts:
  - tet one element: 12
  - cubic one element: 24
  - shell one triangle: `3 * num_surface_vertices` global DOFs and 18 local DOFs
- [x] Add energy/gradient/Hessian parity tests for tet, cubic, and shell after layout migration.

**Exit criteria:**

- Assembler no longer directly computes global DOF indices from vertex ids.
- Existing vertex DOF path remains numerically identical.
- Shell `vid < 0` DOF-side sentinel behavior remains identical and is owned by `Vertex3DofLayout`; shell missing-neighbor geometry stays owned by `KoiterShellKernel` from Task 5e.
- Future Hermite layout can be added without editing every assembler gather/scatter loop.

## Task 6p: Introduce `ParameterField`

**目标：** 把 elastic parameters 和 plastic parameters 的全局/局部 DOF 映射和 quadrature-point 采样统一到 `ParameterField` 抽象中，替换 assembler 的 `ele * numParams + j` 硬编码。每种 field 实现内部固定一种 DOF layout（1:1 配对，不拆成两个独立抽象）。第一版只实现 behavior-preserving 的 element-constant 参数场；不改变 tet/cubic/shell 数值行为，不开放 optimized parameter variables。

**Design contract:**

```text
ParameterField
  = quadrature-point sampling
  = internally gather globalParams -> localParams, then write value(q) and d value(q) / d localParams into caller-owned sample buffers

OptimizableField : ParameterField
  = ParameterField + DOF indexing (gather/scatter/sparsity)
  = Assembler 通过 dynamic_cast<const OptimizableField *> 获取 dofLayout()
  = expose local/global index maps for mixed sparse assembly

ElementModel
  = only consumes quadrature-point samples
  = never treats element-level elastic/plastic parameters as the true computation site
```

Volumetric deformation-gradient elements must compute `Fp_q`, `FpInv_q`, `detFp_q`, `Fe_q`, and elastic material parameter value `b_q` inside the quadrature loop. `ConstantParameterField` may return the same value for every `q`, but that is still a field sampling result, not a separate element-level path. Shell Koiter keeps its existing single material-location behavior in this task; if it is routed through the generic interfaces, use `q = 0` as the shell material location and preserve legacy outputs exactly.

**Files:**

- Create: `src/core/solidDeformationModel/formulations/parameters/elementQuadratureView.h`
- Create: `src/core/solidDeformationModel/formulations/parameters/parameterField.h`
- Create: `src/core/solidDeformationModel/formulations/parameters/constantParameterField.h`
- Create: `src/core/solidDeformationModel/formulations/elements/parameterizedMaterialBlock.h`
- Modify: `src/core/solidDeformationModel/deformationModelFactory.h`
- Modify: `src/core/solidDeformationModel/deformationModelFactory.cpp`
- Modify: `src/core/solidDeformationModel/deformationModelManager.h`
- Modify: `src/core/solidDeformationModel/deformationModelManager.cpp`
- Modify: `src/core/solidDeformationModel/deformationModelAssembler.h`
- Modify: `src/core/solidDeformationModel/deformationModelAssembler.cpp`
- Modify: `src/core/solidDeformationModel/formulations/elements/volumetricElementModel.h`
- Modify: `src/core/solidDeformationModel/factories/elementModelFactory.h`
- Modify: `src/core/solidDeformationModel/factories/elasticModelFactory.h`
- Modify: `src/core/solidDeformationModel/factories/elasticModelFactory.cpp`
- Modify: `src/core/solidDeformationModel/factories/plasticModelFactory.h`
- Modify: `src/core/solidDeformationModel/factories/plasticModelFactory.cpp`
- Modify: `src/core/solidDeformationModel/CMakeLists.txt`
- Create: `tests/src/core/solidDeformationModel/formulations/parameters/constantParameterField_gtest.cpp`
- Modify: `tests/src/core/solidDeformationModel/deformationModelFactory_gtest.cpp`
- Modify: `tests/src/core/solidDeformationModel/deformationModelAssembler_gtest.cpp`
- Modify: `tests/src/core/solidDeformationModel/formulations/elements/volumetricElementModel_gtest.cpp`
- Modify: `tests/src/core/solidDeformationModel/CMakeLists.txt`

- [x] Add `ParameterField` and `ParameterSample`:
  - `ParameterField` is the quadrature-point sampling abstraction; DOF indexing is in the `OptimizableField` subclass;
  - `ParameterField::numChannels()` returns the number of parameter channels;
  - each `OptimizableField` implementation internally owns a private `ParameterDofLayout` (element/quadrature/nodal) that matches its sampling strategy; the layout is not exposed as an independent public type;
  - `OptimizableField::dofLayout()` returns a `const ParameterDofLayout *`; only fields that participate in optimization extend `OptimizableField`
  - `ParameterSample::value` has size `numChannels`;
  - `ParameterSample::dValueDLocal` has shape `numChannels x numLocalDofs`;
  - `ParameterSample::resize(numChannels, numLocalDofs)` prepares the Eigen buffers once for reuse;
  - `ParameterField::sample(ele, quadrature, out)` fills the quadrature-point value and its derivative with respect to element-local parameter DOFs in `out`; field internally calls its own `dofLayout_->gather()` and sampling logic, so callers never see local parameter vectors;
  - `sample(...)` must not return owning Eigen objects by value from the quadrature loop.
- [x] Define `ParameterFieldKind` enum in `parameterField.h`: `CONSTANT`, `QUADRATURE_POINT`, `NODAL_INTERPOLATED`, `EXTERNAL_PROCEDURAL` (latter three are future values, not implemented in this milestone).
- [x] `ParameterField::sample` takes `int ele, int quadratureId, ParameterSample &out`. Only `ConstantParameterField` is implemented; non-constant fields deferred to future work.
- [x] Add `ConstantParameterField` (extends `OptimizableField`):
  - constructor takes `numChannels`, `numElements`, and `const double *globalParams` (non-owning, points to manager-owned data);
  - `dofLayout()` returns pointer to its internal `ElementParameterDofLayout`;
  - `sample(ele, quadrature, out)` internally calls `dofLayout_->gather(ele, globalParams_, localBuf_)` then writes `out.value = localBuf_[0:numChannels]`;
  - `dValueDLocal` is the `numChannels x numChannels` identity;
  - it does not read quadrature id, reference position, or rest position;
  - the internal `ElementParameterDofLayout` is a private implementation detail — callers interact only through `OptimizableField::dofLayout()` and `ParameterField::sample()`.
- [x] Add lightweight `ElasticBlock` and `PlasticBlock` composition types in `parameterizedMaterialBlock.h`:
  - `ElasticBlock` holds non-owning `ElasticModel *model` and `const ParameterField *parameters`;
  - `PlasticBlock` holds non-owning `PlasticModel *model` and `const ParameterField *parameters`;
  - model ownership stays in `DeformationModelManager` / the future model-set owner; Task 6p must not create a second material/plastic owner inside element models;
  - the pure `ElasticModel` / `PlasticModel` classes must not directly own `ParameterField`.
- [x] Add factory helpers to determine parameter channel counts before element model construction:
  - `PlasticModelFactory::numParameters(DeformationModelPlasticMaterial type)`;
  - `ElasticModelFactory::numParameters(const SimulationMesh &mesh, DeformationModelElasticMaterial type)` for the legacy path used through Task 6p;
  - fail fast if any element model reports a different parameter count than the factory-level count.
- [x] Create factory helpers for default parameter fields:
  - elastic: `ConstantParameterField(numElasticParams, nele, elasticGlobalParams.data())`;
  - plastic: `ConstantParameterField(numPlasticParams, nele, plasticGlobalParams.data())`;
  - when `numChannels == 0`, use a zero-channel constant field rather than special-casing `nullptr` in assembler loops.
- [x] Change topology-specific factory construction order:
  - determine `numElasticParams` and `numPlasticParams`;
  - create `ConstantParameterField` instances (owned by manager), passing `globalParamsVector.data()` in the constructor;
  - build `ElasticBlock` and `PlasticBlock` with non-owning pointers to the corresponding `ParameterField` objects; do not assume elastic and plastic fields are identical;
  - pass the blocks into `DeformationModelManager` / `ElementModelFactory` before element models are constructed;
  - pass the same `const ParameterField *` pointers to `DeformationModelAssembler`, `ElasticBlock`, and `PlasticBlock`; Assembler may later `dynamic_cast<const OptimizableField *>` when parameter optimization lands.
- [x] Rename new public/milestone-facing snapshots to `elasticParameters` and `plasticParameters`. Historical completed-task prose may keep `elasticParams` / `plasticParams`, but Task 6p and later tasks must use the full names in new APIs and tests.
- [x] Change `DeformationModelAssembler` to hold separate `const ParameterField *` instances for elastic and plastic parameters (non-owning, manager owns the fields), alongside the displacement `DofLayout` from Task 6. Assembler does not call `gather()`, `sample()`, or `dofLayout()` on the parameter fields in this milestone; the fields are stored for future parameter-optimization use when Assembler can `dynamic_cast<const OptimizableField *>` to access mixed sparse templates and gradient scatter.
- [x] Simplify `DeformationModel::prepareData` virtual signature: remove `plasticLocal` / `elasticLocal` parameters. Parameter data is injected into fields at construction time. `VolumetricElementModel::prepareData` calls `field->sample(ele, quadrature, out)` which is fully self-contained.
- [x] Add an explicit `VolumetricElementModel` constructor that takes `ElasticBlock` and `PlasticBlock`. Keep the existing constructor as a delegating compatibility constructor that creates element-constant parameter fields for `plasticModel->getNumParameters()` and `elasticModel->getNumParameters()`; this keeps direct element tests and any temporary downstream callers buildable. If Task 5q has already removed legacy wrappers by the time Task 6p runs, no wrapper-specific work is needed.
- [x] Update `ElementModelFactory::create()` to pass `ElasticBlock`/`PlasticBlock` to element model constructors. Shell `ShellElementModel` keeps the legacy single material-location parameter path in Task 6p, with assembler-side parameter layout handled by `ParameterField`.
- [x] Replace `DeformationModelAssembler::getElasticParameters` and `getPlasticParameters` direct offset logic — these methods are removed since Assembler no longer does parameter gather. Mixed sparse template construction and gradient scatter through `OptimizableField::dofLayout()` are deferred to the parameter-optimization milestone; the field infrastructure (`OptimizableField` + `ParameterDofLayout`) is in place and tested, but Assembler does not consume it yet.
- [x] Update `VolumetricElementModel` cache so volumetric plastic data is stored per quadrature point:
  - `Fp[q]`
  - `FpInv[q]`
  - `detFp[q]`
  - derivatives of `FpInv` / `detFp` with respect to local plastic sample channels at `q`.
- [x] Add cache-owned `ParameterSample` buffers for elastic and plastic sampling. Buffers may be one scratch buffer per field or one buffer per quadrature point, but repeated `prepareData(...)` calls must reuse storage instead of allocating in the inner quadrature loop.
- [x] Update `VolumetricElementModel::prepareData` so each quadrature point samples plastic parameters through `plasticBlock.parameters->sample(ele, quadrature, sample)`, which internally does gather + sampling. Then call `plasticBlock.model` on that quadrature-point sample. `PlasticModel` remains responsible for `a_q -> Fp_q/FpInv_q/detFp_q` and local derivatives.
- [x] Update `VolumetricElementModel::prepareData` so each quadrature point samples elastic parameters through `elasticBlock.parameters->sample(ele, quadrature, sample)`, which internally does gather + sampling. Then call `elasticBlock.model`; energy/stress/derivative routines use `b_q`, not a single element-level material parameter pointer.
- [x] Implement derivative chain rule in field-space form even though `ConstantParameterField` has identity `dValueDLocal`:
  - local elastic derivatives returned by `ElasticModel` are first derivatives with respect to sampled channels `b_q`;
  - local plastic derivatives returned by `PlasticModel` are first derivatives with respect to sampled channels `a_q`;
  - `VolumetricElementModel` multiplies by `dValueDLocal(q)` so `compute_dE_da`, `compute_d2E_da2`, `compute_d2E_dxda`, `compute_dE_db`, `compute_d2E_db2`, `compute_d2E_dxdb`, and `compute_d2E_dadb` return derivatives with respect to element-local parameter DOFs.
- [x] Preserve the exact element-constant behavior by making `ConstantParameterField` return identical samples for all quadrature points. Add old-vs-new tests that prove tet and cubic energy, gradient, Hessian, `compute_d2E_dxda`, and `compute_d2E_dxdb` are unchanged.
- [x] Keep `ElasticModel` and `PlasticModel` responsibilities unchanged:
  - `ElasticModel` computes local law derivatives with respect to elastic sample `b_q`;
  - `PlasticModel` computes local `Fp` parametrization derivatives with respect to plastic sample `a_q`;
  - `ElementModel` owns the chain rule from field samples to element-local derivatives.
- [x] Add tests or assertions that elastic and plastic parameter fields can have different specs. In Task 6p only `CONSTANT` is implemented, so the runtime behavior is still identical, but construction must not share one implicit field object or assume equal local parameter counts.
- [x] Do not implement non-constant field kinds: `QUADRATURE_POINT`, `NODAL_INTERPOLATED`, `EXTERNAL_PROCEDURAL`. Defining the enum values is allowed, but requesting them must throw clear `std::invalid_argument` errors.
- [x] Do not expose elastic or plastic parameters as optimizer variables in Task 6p. `exposeAsOptimizationVariable` remains a spec-level future flag until a coupled energy and regularization/constraint story exists; if a caller sets it to `true`, fail fast instead of silently ignoring it.
- [x] Add unit tests for `ConstantParameterField`:
  - returned sample value equals local parameters for multiple quadrature ids;
  - `dValueDLocal` is identity;
  - `dofLayout()` returns a valid pointer with correct `numGlobalDofs`, `numLocalDofs`, gather, scatter, and global index mapping;
  - zero-channel field returns empty value, empty derivative matrix, and a valid zero-channel layout without throwing.
- [x] Add factory/spec validation tests that unsupported `ParameterFieldKind` values and `exposeAsOptimizationVariable == true` fail with stable error messages.

**Exit criteria:**

- Assembler no longer hard-codes elastic/plastic parameter offsets as `ele * numParams + j`; it no longer gathers parameter values at all.
- `ParameterField::sample(ele, quadrature, out)` encapsulates the full path from global parameter vector to quadrature-point sample; data pointer is injected at construction time, callers never see local parameter vectors or global parameter pointers.
- Volumetric `VolumetricElementModel` computes `Fp`, `FpInv`, `detFp`, `Fe`, elastic parameters, and plastic parameters only as quadrature-point samples.
- The first implementation remains behavior-preserving for current element-constant elastic/plastic parameters.
- `ConstantParameterField` has focused unit tests covering both `sample()` (which internally calls gather + copies) and its internal `ElementParameterDofLayout`.
- Tet/cubic regression tests pass for energy, gradient, Hessian, `compute_df_da`, and `compute_df_db`.
- Element-model parameter sampling uses cache-owned `ParameterSample` buffers; the API does not force owning Eigen allocations inside quadrature loops.
- Elastic and plastic parameter fields are separate objects shared through `ElasticBlock` and Assembler (both as `const ParameterField *`); element model uses `sample()` for quadrature-point values, Assembler stores them for future parameter-optimization use via `dynamic_cast<const OptimizableField *>`.
- Task 7 can migrate material payload/recipe semantics without also deciding parameter storage or field interpolation semantics.

### Task 6p 额外清理 (2026-05-29)

实施过程中顺手做了以下与 Task 6p 核心目标（引入 ParameterField）无关但显著改善代码质量的改动：

- [x] `DeformationModelAssemblerCacheData` 从匿名 namespace 拆出到独立 `deformationModelAssemblerCacheData.h`。
- [x] `DeformationModelAssembler::data` 从裸指针改为 `std::unique_ptr<DeformationModelAssemblerCacheData>`，析构函数简化为 `= default`。
- [x] 参数数据 ownership 统一到 `DeformationModelManager`：删除 `DeformationModelEnergy` 的 `elasticParams`/`plasticParams` 成员和对应 setter；删除 `DeformationModelAssembler` 的 `setElasticParameterData`/`setPlasticParameterData`；移除 assembler 中 manager 的 `const`。调用方现在通过 `energy.assembler().getDeformationModelManager().setElasticParams(vec)` 直接操作 manager。
- [x] 删除非模板 `detail::makeDeformationModelBundle`（已被 unified factory 替代）及两个 legacy parity 测试。
- [x] `deformationModelFDTest.h/.cpp` 从 `src/core/solidDeformationModel/` 移到 `tests/src/core/solidDeformationModel/`，从生产 CMakeLists 移除。
- [x] `ElementModelFactory::create` 返回 `std::unique_ptr<DeformationModel>`（原返回裸指针），manager 的 `elementFEMs` 改为 `vector<unique_ptr<DeformationModel>>`，消除手动 `delete` 循环。
- [x] `compute_dE_da` / `compute_d2E_da2` / `compute_dE_db` / `compute_d2E_db2` / `compute_d2E_dadb` 提升为 `DeformationModel` 基类虚函数（默认空实现），`VolumetricElementModel` 标记 `override`。
- [x] `ParameterField::sample` 签名简化为 `sample(int ele, int quadratureId, ParameterSample &out)`（删除了中间 `ElementQuadratureView` wrapper）。

## Finalize: Python Deformation API for Current System

**目标：** 在 C++ formulation class hierarchy、lifetime、DofLayout、ParameterField 边界已稳定的前提下，基于当前 C++ API（仍使用 legacy `DeformationModelElasticMaterial` enum）定稿 Python public API。所有 FEM-specific construction 类型都放在 `pypgo.fem`；最终返回的可求值 energy wrapper 放在 `pypgo.energy`。Material recipe refactor (Task 7)、Orthotropic solver law (Task 8)、Hermite extension point (Task 9) 推迟到后续 milestone。

**Current status (2026-06-01):** C++ Tasks 0–6p 已完成。`makeDeformationEnergy()` 使用 `const Formulation &` + legacy `DeformationModelElasticMaterial` enum。Python 已有 private `_core` smoke hooks（`_create_tet_deformation_energy_for_test` 等），需升级为 public API。

**Files:**

- Create: `pypgo/fem.py`
- Create: `pypgo/energy.py`
- Modify: `pypgo/__init__.py`
- Modify: `src/python/pypgo/bindings/energy_bindings.cpp`
- Modify: `src/python/pypgo/bindings/module.cpp`
- Modify: `src/python/pypgo/CMakeLists.txt`
- Create: `tests/pypgo/test_deformation_energy.py`

### Python public API 形态（基于当前 C++ state，legacy enum string 仅留在 wrapper 内部）

```python
import pypgo as pgo

# ===== FEM construction surface (pypgo.fem) =====
pgo.fem.TetP1()                 # tet P1
pgo.fem.LinearCubic()           # hex trilinear — cubic 必须显式传
pgo.fem.KoiterShell()           # Koiter shell

pgo.fem.StableNeo()             # "stable_neo"
pgo.fem.StVK()                  # "stvk"
pgo.fem.StVKVolume()            # "stvk_vol"
pgo.fem.LinearElastic()         # "linear"
pgo.fem.MooneyRivlin()          # "mooney_rivlin"
pgo.fem.KoiterStVK()            # "koiter_stvk"

pgo.fem.VolumetricPlasticity(dofs=6)
pgo.fem.VolumetricPlasticity(dofs=3)
pgo.fem.VolumetricPlasticity(dofs=0)
pgo.fem.ShellPlasticity(dofs=1)
pgo.fem.ShellPlasticity(dofs=0)

pgo.fem.DeformationOptions(enforce_spd=True, enable_material_max_step=True)

# ===== Factory =====
energy = pgo.fem.deformation_energy(
    sim_mesh,
    formulation=pgo.fem.LinearCubic(),
    elastic=pgo.fem.StableNeo(),
    plastic=pgo.fem.VolumetricPlasticity(dofs=6),
)

# ===== Final energy object (pypgo.energy) =====
assert isinstance(energy, pgo.energy.DeformationEnergy)
energy.num_dofs          # int
energy.rest_position     # (num_vertices, 3) ndarray

u = energy.zero_state()    # (num_dofs,) ndarray
energy.value(u)            # float
energy.gradient(u)         # (num_dofs,) ndarray
H = energy.hessian(u)      # SparseMatrix
rows, cols, values = H.to_coo()
```

- [ ] Add `pypgo/fem.py` with:
  - `TetP1`, `LinearCubic`, `KoiterShell` formulation dataclasses mapping to C++ formulation classes;
  - `StableNeo`, `StVK`, `StVKVolume`, `LinearElastic`, `MooneyRivlin`, `KoiterStVK` elastic-law dataclasses mapping to legacy enum strings internally;
  - `VolumetricPlasticity(dofs)` and `ShellPlasticity(dofs)` wrappers; validate allowed DOF counts in Python before calling `_core`;
  - `DeformationOptions` dataclass mapping to `DeformationModelOptions`;
  - `deformation_energy(sim_mesh, formulation, elastic, plastic, options=None)` factory returning `pypgo.energy.DeformationEnergy`.
- [ ] Add `pypgo/energy.py` with `DeformationEnergy` wrapper class delegating to `_core.DeformationEnergyCore`. This module must not define FEM formulations, FEM elastic recipes, plastic wrappers, or the deformation factory.
- [ ] Upgrade `energy_bindings.cpp` from private `_core` smoke hooks to a single core construction API:
  - expose core-only formulation classes or factory helpers needed by `pypgo.fem` wrappers; names remain `_core` implementation details, not public Python API;
  - add `_core.make_deformation_energy(mesh_core, formulation_core, elastic_kind, plastic_kind, options_core)` returning `DeformationEnergyCore`;
  - keep `DeformationEnergyCore` responsible for holding `std::shared_ptr<SimulationMeshCore>` so the borrowed C++ mesh outlives the energy chain.
- [ ] Enforce Python policy:
  - `pgo.energy.deformation_energy`, `pgo.energy.StableNeo`, and `pgo.energy.Plastic` are not public names;
  - tet may omit formulation and default to `TetP1()`;
  - cubic requires explicit `LinearCubic()`;
  - shell requires explicit `KoiterShell()`;
  - elastic and plastic wrappers are required and type-checked;
  - raw enum strings are rejected at the public Python layer;
  - unsupported material/formulation/topology combos raise `ValueError` with stable messages.
- [ ] Add Python tests (`tests/pypgo/test_deformation_energy.py`):
  - `pypgo` lazy-loads `fem` and `energy`;
  - tet omitted formulation defaults to `TetP1()`;
  - tet explicit `TetP1()` + `StableNeo()` + `VolumetricPlasticity(dofs=6)` builds, `zero_state()` correct;
  - cubic `LinearCubic()` + `StVK()` + `VolumetricPlasticity(dofs=6)` builds, `num_dofs == 3 * num_vertices`;
  - cubic without formulation raises `ValueError`;
  - shell `KoiterShell()` + `KoiterStVK()` + `ShellPlasticity(dofs=1)` builds from `create_shell()`;
  - if `MooneyRivlin()` is public, ENu payload mismatch raises clearly and Mooney-Rivlin payload smoke passes;
  - `value(u0)`, `gradient(u0)`, `hessian(u0)` smoke at zero state
  - same mesh → two independent energies
  - mesh deleted → energy still usable
  - `pgo.energy` does not expose FEM construction helpers
  - raw string plastic/elastic inputs are rejected

**Exit criteria:**

- Public Python deformation API finalized for current system (tet P1, hex trilinear, shell Koiter)
- Python examples consistently use `pgo.fem.deformation_energy(...)` and `LinearCubic()`
- `pypgo.energy` only owns the final `DeformationEnergy` wrapper, not FEM construction types
- All Python tests pass with `python -m pytest -q tests/pypgo`

## Deferred: Material Payload, Elastic Recipe Refactor, Orthotropic, Hermite (原 Task 7/8/9)

**状态：** 推迟到后续 milestone。当前先 finalize 现有体系的 Python API。

- **Task 7 (Material Payload & Elastic Recipe):** 引入 `ElasticModelSpec` 替代 `DeformationModelElasticMaterial` enum；Mooney-Rivlin / Hill composite recipe migration；Orthotropic payload 转换。
- **Task 8 (Orthotropic Solver Law):** `ElasticModel3DOrthotropicStVK` 实现。
- **Task 9 (Hermite Extension Point):** 推迟。

## Verification Commands

Run C++ baseline and deformation tests:

```bash
conda run -n libpgo cmake --preset base
conda run -n libpgo cmake --build --preset base -j 8
conda run -n libpgo ctest --test-dir build/base -R "SimulationMesh|TetP1Basis|HexTrilinearBasis|Quadrature|VolumetricKernel|VolumetricElementModel|KoiterShellKernel|ShellElementModel|DeformationModelFactory|DeformationModelAssembler|DeformationModelFormulation|ElasticModelFactory|OrthotropicStVK|Vertex3DofLayout|ParameterField|ConstantParameterField" --output-on-failure
```

Run Python build and tests:

```bash
conda run -n libpgo cmake --preset python-build
conda run -n libpgo cmake --build --preset python-build -j 8
conda run -n libpgo python -m pytest -q tests/pypgo/test_simulation_mesh.py tests/pypgo/test_deformation_energy.py
```

Run broad Python smoke after binding changes:

```bash
conda run -n libpgo python -m pytest -q tests/pypgo
```

## Risk Assessment

| Risk | Impact | Mitigation |
|---|---:|---|
| `DeformationModelEnergy` state convention is misunderstood in Python | High | Expose `zero_state()` and document `value(u)` as displacement; add zero-state tests |
| Manager borrows a `SimulationMesh` that does not outlive the energy chain | High | Public factories take `const SimulationMesh &`; C++ core factories do not hide temporary mesh owners; `makeSimulationMesh(...)` returns an explicit `std::unique_ptr<SimulationMesh>` owner; Python `DeformationEnergyCore` keeps `SimulationMeshCore` alive |
| Existing manager call sites still assume moving `std::unique_ptr<SimulationMesh>` transfers lifetime into the manager | High | Task 3 updates all call sites so mesh ownership stays outside manager until energy destruction; add tests for two energies from one mesh owner |
| DofLayout migration changes sparse pattern ordering | Medium | Compare dense Hessian values, not only nnz/order; keep `findEntryOffset` tests |
| Parameter field abstraction changes material/plastic sensitivities | High | Task 6p keeps only `ConstantParameterField`, compares `compute_df_da` / `compute_df_db` against old behavior, and rejects non-constant field kinds until derivative-chain tests exist |
| Payload/law names are confused in Python | High | Keep payload classes in `pypgo.mesh.veg`, FEM law wrappers in `pypgo.fem`, and final energy objects in `pypgo.energy`; add mismatch tests |
| Hill is treated as a standalone material | High | Model Hill only as `HillFiber(base=..., hill_slot=..., fibers=...)` |
| Orthotropic law has incorrect frame convention | High | Document `R` direction and add rotated-frame tests |
| Mooney-Rivlin Vega payload maps incorrectly to solver coefficients | Medium | Centralize conversion helper and test `.veg` payload vs `SimulationMeshMooneyRivlinMaterial` coefficients |
| New files are scattered back into the module root | Medium | New formulation, factory, material, DOF, and parameter abstractions must use the directory layout in design decision 4; root only keeps façade/main-chain/legacy wrapper files |
| Tet and cubic refactors diverge into two incompatible element paths | High | Both use `VolumetricKernel` + `VolumetricElementModel` with different `Basis`/`Quadrature`; old-vs-new regression tests lock behavior |
| `ElementModel<Kernel, ElasticModel, PlasticModel>` causes template explosion | Medium | Implement as non-template `VolumetricElementModel` with runtime elastic/plastic injection via `ElasticBlock`/`PlasticBlock` |
| Python binding duplicates `SimulationMeshCore` or cannot keep its owned mesh alive | High | `SimulationMeshCore` in shared `simulation_mesh_core.h`; `DeformationEnergyCore` holds `std::shared_ptr<SimulationMeshCore>` |
| Python `LinearCubic()` 名称被误解为 topology auto-dispatch | Medium | Public Python keeps `LinearCubic()` by design, but docs/tests state it maps to C++ `LinearCubicFormulation` with internal name `hex_trilinear`; raw legacy enum strings stay behind `_core` wrappers |
| Shell DOF gather/scatter regresses on `vid < 0` slots | Medium | `Vertex3DofLayout` owns DOF-side `vid < 0` zero-local behavior in gather, scatter, and sparsity; layout-level tests cover boundary triangles |
| Shell missing-neighbor geometry semantics drift after migration | High | `KoiterShellKernel` owns `hasVtx[6]` mask and missing-neighbor handling; parity tests vs `KoiterDeformationModel` covered all cases before Task 5q deletion |

## Milestone Exit Criteria

This plan is complete when:

- C++ can build deformation energy with explicit `P1TetFormulation{}`, `LinearCubicFormulation{}`, and `KoiterShellFormulation{}` formulation objects.
- `P1Tet` and hex trilinear (`LinearCubicFormulation` in C++) both use real `Basis`/`Quadrature`/`VolumetricKernel` + `VolumetricElementModel`; shell `KoiterShell` uses `KoiterShellKernel` + `ShellElementModel`.
- Formulation class hierarchy (`Formulation` → `VolumetricFormulation`/`ShellFormulation` → concrete classes) provides runtime dispatch via virtual functions.
- Single factory entry `makeDeformationEnergy(const SimulationMesh &, const Formulation &, ...)` serves all topology/formulation combinations.
- Old public `makeDeformationModel(...)` auto-dispatch entry has been removed.
- Legacy `TetMeshDeformationModel`, `CubicMeshDeformationModel`, `KoiterDeformationModel` files are deleted.
- `DeformationModelManager` borrows `const SimulationMesh &` (non-owning).
- Manager creation logic is split into `ElasticModelFactory` / `PlasticModelFactory` / `ElementModelFactory`.
- Assembler uses `DofLayout` (specifically `Vertex3DofLayout`) for gather/scatter/sparsity.
- `ParameterField` / `ConstantParameterField` handles elastic and plastic parameter sampling at quadrature points.
- Python can construct tet P1, cubic hex trilinear, and shell Koiter deformation energy via `pgo.fem.deformation_energy()`.
- Python formulations: `pgo.fem.TetP1()`, `pgo.fem.LinearCubic()`, `pgo.fem.KoiterShell()`.
- Python FEM wrappers: `StableNeo`, `StVK`, `StVKVolume`, `LinearElastic`, `MooneyRivlin`, `KoiterStVK`, `VolumetricPlasticity`, `ShellPlasticity`.
- Python cubic deformation API requires explicit `LinearCubic()`.
- `pypgo.energy` exposes the final `DeformationEnergy` object but no FEM construction helpers.
- Material docs distinguish payload (`pypgo.mesh.veg`) from FEM law wrapper (`pypgo.fem`) and final energy object (`pypgo.energy`).
- C++ and Python tests pass with the commands above.

---

## Architecture Evolution Notes (2026-05-31)

原 plan 设计使用 `FormulationTraits<T>` 模板 + C++20 concepts + tag struct 的编译期 dispatch 方案。实施过程中改为 class hierarchy + 虚函数，原因：

- Python binding (nanobind) 天然支持多态，class hierarchy 比模板 variant 容易绑定
- 当前只有 3 种 formulation，模板 + concepts 过度设计
- Factory 内部 `dynamic_cast` 判断 volumetric vs shell 比 `std::visit` + `if constexpr` 更直观

删除的文件：
- `formulationTraits.h` — 不再需要 per-formulation 编译期 traits
- `formulationConcepts.h` — 不再需要 C++20 concepts
- `deformationFormulations.h` — 原计划统一 formulation registry，现在 formulation 类直接定义在 `formulation.h`

重命名的文件：
- `fundamentalFormsKernel.h/.cpp` → `koiterShellKernel.h/.cpp`（继承自新增的 `ShellKernel` 虚基类）
- `koiterShellElementModel.h` → `shellElementModel.h`（通用 shell element model，接受任何 `ShellKernel`）
- `koiterShellElementModelCacheData.h` → `shellElementModelCacheData.h`

合并的 factory：
- `makeTetDeformationModel<TetP1>(...)` / `makeCubicDeformationModel<HexTrilinear>(...)` / `makeShellDeformationModel<ShellKoiter>(...)` → 单一 `makeDeformationEnergy(const SimulationMesh &, const Formulation &, ...)`
