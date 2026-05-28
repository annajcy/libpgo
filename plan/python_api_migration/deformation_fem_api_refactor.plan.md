# Deformation FEM API Refactor Plan

> **状态日期：** 2026-05-27  
> **适用范围：** C++ `solidDeformationModel` API 重构 + Python `pypgo.fem` / `pypgo.energy` deformation binding。  
> **执行约束：** 本计划只重构当前 tet P1 / hex trilinear deformation 主链路并绑定到 Python；不在本计划内实现 tricubic Hermite FEM 数值内核。

## 目标

把当前 deformation energy 主链路从“mesh type 隐含 FEM formulation 和 DOF layout”改成显式三层：

```text
MeshTopology / SimulationMesh
  -> ElementFormulation
  -> DofLayout
  -> DeformationModelAssembler
  -> DeformationModelEnergy
```

第一阶段必须保持现有 tet/cubic 行为不变，同时把当前 cubic deformation 路径明确命名为 `hex_trilinear`。这样 Python API 可以先安全暴露当前可用能力，后续再接 `hex_tricubic_hermite` 时不会推翻已发布的 cubic API。

## 当前问题

当前代码已经把文件/几何 mesh 和 solver-ready deformation energy 分开，但还没有把 cell topology、element formulation、DOF layout 分开：

- `SimulationMeshType::CUBIC` 现在同时表示 8 顶点 hexahedral topology 和 8-node trilinear vertex-DOF formulation。
- `DeformationModelManager::initImpl` 同时创建 elastic material、plastic model、element deformation model，且按 `SimulationMeshType` 直接分派。
- `DeformationModelAssembler` 假设全局 DOF 为 `3 * numVertices`，局部 DOF 为 `numElementVertices * 3`，gather/scatter 全部通过 vertex id。
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

### 1. Formulation 用 tag object 表达，按 topology 拆分 factory

保留现有 `SimulationMeshType::TET` / `CUBIC` / `SHELL` 作为内部 topology metadata，但 public C++ factory 不再接收一个跨拓扑的 `ElementFormulationType`，也不使用只能表达常量的 per-topology enum。改为每类 formulation 用一个 tag object 表达，topology 由 factory 函数名表达。下面是 milestone 结束时的目标 tag 集；Task 2 只引入当前可执行的 `TetP1` / `HexTrilinear` / `ShellKoiter`，Task 9 再加入 `HexTricubicHermite` placeholder。

```cpp
struct TetP1 {};

struct HexTrilinear {};

struct HexTricubicHermite
{
  int quadratureOrder = 4;
};

struct ShellKoiter {};
```

对应构造入口也按 topology 拆开：

```cpp
makeTetDeformationModel(..., TetP1{}, ...)
makeCubicDeformationModel(..., HexTrilinear{}, ...)
makeCubicDeformationModel(..., HexTricubicHermite{ .quadratureOrder = 5 }, ...)  // Task 9 placeholder, throws until Hermite lands
makeShellDeformationModel(..., ShellKoiter{}, ...)
```

不保留旧 `makeDeformationModel(...)` auto-dispatch 入口，也不提供 `AUTO` formulation。迁移后调用方必须选择 topology-specific factory。Python public API 也要求 cubic 调用方显式传入 `pgo.fem.HexTrilinear()`，避免把 topology 名称误当 formulation 名称。

### 2. `FormulationTraits` 只做类型路由，不承载数学公式

每个 formulation tag 可以通过 traits 做编译期 dispatch：

```cpp
template<class Formulation>
struct FormulationTraits;

template<>
struct FormulationTraits<HexTrilinear>
{
  using DofLayout = Vertex3DofLayout;
  using Kernel = HexTrilinearKernel;
  using ElementModel = DeformationGradientElementModel<Kernel>;

  static constexpr int nodesPerElement = 8;
  static constexpr int localDofs = 24;
  static constexpr std::string_view name = "hex_trilinear";
};
```

`FormulationTraits` 只允许放：

- formulation 名称、维度、局部 DOF 数等轻量元信息；
- `DofLayout`、`ElementModel`、`Kernel` 等类型别名；
- feature flags，例如是否支持 material max-step、是否 vertex DOF。

具体数学公式不要放在 traits 里。公式实现放在 `ElementModel` 或独立 kernel 中：

```text
Formulation tag -> FormulationTraits -> DofLayout + ElementModel + Kernel
```

`TetP1` 和 `HexTrilinear` 都要先抽出明确 kernel，再写 traits。不要让 `FormulationTraits` 指向旧的 `TetMeshDeformationModel` / `CubicMeshDeformationModel` 作为过渡类型；这会把完整 element model 误命名成 kernel，后续读代码的人会分不清抽象层次。旧类可以临时作为 behavior oracle 或兼容 wrapper，但不能成为新 traits 的目标类型。

更准确的数学分层是：

```text
Kernel
  = formulation-specific geometry/integration formula
  = basis, quadrature, dN/dX, F(u), dF/du, detJ * weight

ElementModel<Kernel, ElasticModel, PlasticModel>
  = complete per-element energy process
  = gather local state, call Kernel, call ElasticModel/PlasticModel, accumulate E/grad/H
```

但第一版工程实现不要把 `ElasticModel` 和 `PlasticModel` 也模板化进 `FormulationTraits`，否则 `HexTrilinear x StableNeo x StVK x MooneyRivlin x Hill x PlasticNone x Plastic6Dof` 会变成组合爆炸，并且 Python/config 的运行时 recipe 很难映射。目标落地形态是：

```cpp
template<class Kernel>
class DeformationGradientElementModel;

template<>
struct FormulationTraits<HexTrilinear>
{
  using DofLayout = Vertex3DofLayout;
  using Kernel = HexTrilinearKernel;
  using ElementModel = DeformationGradientElementModel<Kernel>;
};
```

`ElementModelFactory` 负责用 `FormulationTraits<Formulation>::ElementModel` 创建元素模型，并把 runtime 构造出的 `ElasticModel *` / `PlasticModel *` 注入进去。也就是说，数学概念上可以理解为 `ElementModel<Kernel, ElasticModel, PlasticModel>`，但 C++ 第一版 API 固定为 `ElementModel<Kernel>` + runtime material/plastic dependency injection。`FormulationTraits` 不能声明具体的 `StableNeo`、`MooneyRivlin`、`HillFiber` 或 `Plastic6Dof` 类型。

第一版必须同步落地两个 deformation-gradient kernel：

```cpp
template<>
struct FormulationTraits<TetP1>
{
  using DofLayout = Vertex3DofLayout;
  using Kernel = TetP1Kernel;
  using ElementModel = DeformationGradientElementModel<Kernel>;

  static constexpr int nodesPerElement = 4;
  static constexpr int localDofs = 12;
  static constexpr std::string_view name = "tet_p1";
};

template<>
struct FormulationTraits<HexTrilinear>
{
  using DofLayout = Vertex3DofLayout;
  using Kernel = HexTrilinearKernel;
  using ElementModel = DeformationGradientElementModel<Kernel>;

  static constexpr int nodesPerElement = 8;
  static constexpr int localDofs = 24;
  static constexpr std::string_view name = "hex_trilinear";
};
```

这样 tet P1 和 hex trilinear 在新架构中是平级 formulation；区别只在 kernel 的 basis/quadrature/local DOF，而不是一个走新抽象、一个留在旧模型。

### 3. Core API 用 template/concept，运行时边界用 `std::variant`

不要引入 `TetFormulation` / `HexFormulation` / `ShellFormulation` 虚基类，也不要让核心 factory 接收基类 `const &`。那会把 formulation dispatch 变成运行时多态，最后需要 `dynamic_cast` 或 virtual 方法，削弱 `FormulationTraits<Formulation>` 的编译期类型安全。

核心 C++ factory 使用 tag object + concept/template：

```cpp
template<class F>
concept TetFormulation = std::same_as<F, TetP1>;

template<class F>
concept CubicFormulation =
  std::same_as<F, HexTrilinear>;

template<class F>
concept ShellFormulation = std::same_as<F, ShellKoiter>;

template<TetFormulation F>
DeformationModelBundle makeTetDeformationModel(
  const VolumetricMeshes::TetMesh &mesh,
  const F &formulation,
  const DeformationModelSpec &spec);

template<CubicFormulation F>
DeformationModelBundle makeCubicDeformationModel(
  const VolumetricMeshes::CubicMesh &mesh,
  const F &formulation,
  const DeformationModelSpec &spec);
```

这让错误组合在编译期失败：

```cpp
makeTetDeformationModel(tetMesh, HexTrilinear{}, spec);  // compile error
makeCubicDeformationModel(cubicMesh, TetP1{}, spec);     // compile error
```

运行时边界才使用 `std::variant`，例如 Python binding、JSON/config、CLI：

```cpp
using CubicFormulationVariant = std::variant<HexTrilinear>;

DeformationModelBundle makeCubicDeformationModel(
  const VolumetricMeshes::CubicMesh &mesh,
  const CubicFormulationVariant &formulation,
  const DeformationModelSpec &spec)
{
  return std::visit([&](const auto &f) {
    return makeCubicDeformationModel(mesh, f, spec);
  }, formulation);
}
```

`std::variant` overload 是边界 adapter，不是 core API 的主要形态。Task 2 首先只让 `CubicFormulation` / `CubicFormulationVariant` 包含已实现的 `HexTrilinear`，保证 topology-specific factory 可以在当前数值内核上闭环。Task 9 再把 `HexTricubicHermite` 加入 concept 和 variant，并且只接入显式 `not implemented` guard，不能在 Task 2 里提前制造一个没有内核的 formulation 分支。

这些 constrained template overload 的定义必须放在 `deformationModelFactory.h` 或它 include 的 `deformationModelFactory.inl` 中；`.cpp` 只放非模板 helper / implementation detail。不要把模板定义只放进 `.cpp`，否则 tests、tools、Python binding 这些调用方会在独立 translation unit 中链接失败。

### 4. 新文件按 façade / formulation / factory / material 分层

`solidDeformationModel` 根目录只保留 public façade、assembler/manager/energy 主链路，以及旧 element model wrapper。新抽象不要继续散落在根目录里，而是按职责放入子目录：

```text
src/core/solidDeformationModel/
  simulationMesh.h/.cpp
  deformationModel.h
  deformationModelFactory.h/.cpp
  deformationModelFactory.inl
  deformationModelAssembler.h/.cpp
  deformationModelEnergy.h/.cpp
  deformationModelManager.h/.cpp

  tetMeshDeformationModel.h/.cpp       # legacy-compatible wrapper
  cubicMeshDeformationModel.h/.cpp     # legacy-compatible wrapper
  koiterDeformationModel.h/.cpp        # existing shell path, move only in a later cleanup

  formulations/
    deformationFormulations.h
    formulationTraits.h
    formulationConcepts.h
    formulationVariants.h

    kernels/
      deformationGradientKernel.h
      tetP1Kernel.h/.cpp
      hexTrilinearKernel.h/.cpp
      hexTricubicHermiteKernel.h/.cpp  # future

    elements/
      deformationGradientElementModel.h
      deformationGradientElementModel.inl
      deformationGradientElementModel.cpp

    dof/
      dofLayout.h
      vertex3DofLayout.h/.cpp
      hermiteDofLayout.h/.cpp          # future

  factories/
    elasticModelFactory.h/.cpp
    plasticModelFactory.h/.cpp
    elementModelFactory.h/.cpp

  materials/
    elasticModelSpec.h
    simulationMeshMaterialPayload.h
    simulationMeshOrthotropicMaterial.h
```

测试目录镜像新结构：

```text
tests/src/core/solidDeformationModel/
  formulations/
    kernels/
      tetP1Kernel_gtest.cpp
      hexTrilinearKernel_gtest.cpp
    elements/
      deformationGradientElementModel_gtest.cpp
    dof/
      vertex3DofLayout_gtest.cpp

  factories/
    elasticModelFactory_gtest.cpp
    elementModelFactory_gtest.cpp
```

第一轮不要搬迁所有旧 `elasticModel*` / `plasticModel*` 文件；那会把核心抽象重构变成大规模 include/CMake 搬家。旧 solver law 文件先留在根目录，新 material spec、payload、factory 进入新目录。等 Python API 和 formulation 抽象稳定后，再单独做 legacy file relocation。

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
  explicit DeformationEnergyCore(DeformationModelBundle bundle);
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

当前 `SimulationMeshCore` 独占 `std::unique_ptr<SimulationMesh>`。如果 Python public API 接受 `SimulationMesh` 并创建多个 energy，就需要 clone：

```cpp
std::unique_ptr<SimulationMesh> SimulationMesh::clone() const;
```

第一版推荐：

- C++ core 支持 topology-specific factory，例如 `makeTetDeformationModel(std::unique_ptr<SimulationMesh>, TetP1{}, spec)` 和 `makeCubicDeformationModel(std::unique_ptr<SimulationMesh>, HexTrilinear{}, spec)`。
- Python `SimulationMeshCore` 通过 `mesh_->clone()` 传入 factory。
- 不消费 Python `SimulationMesh` 对象本身。

如果 `clone()` 改动面过大，可以临时先绑定 `VolumeMeshCore -> makeTetDeformationModel/makeCubicDeformationModel`，但 public API 仍应保留 `SimulationMesh` 作为目标形态，避免把 Vega volume mesh 和 solver-ready mesh 混成一个概念。

### 9. `DeformationModelManager::initImpl` 先拆内部 factory，再公开抽象

第一刀只做 behavior-preserving extraction：

- `ElasticModelFactory`
- `PlasticModelFactory`
- `ElementModelFactory`

可以先放在 `deformationModelManager.cpp` 匿名 namespace 或私有 header 中，避免把 raw pointer ownership 过早扩散为 public API。

### 10. `DofLayout` 先服务现有 vertex DOF，再服务 Hermite

先实现 `Vertex3DofLayout`，让现有 tet/cubic/shell 走同一套 layout 接口。Hermite 后续新增 `HermiteDofLayout`，不在本计划实现。

```cpp
class DofLayout
{
public:
  virtual ~DofLayout() = default;
  virtual int numGlobalDofs() const = 0;
  virtual int numLocalDofs(int ele) const = 0;
  virtual void gather(int ele, const double *global, double *local) const = 0;
  virtual void scatterAddGradient(int ele, const double *local, double *global) const = 0;
  virtual void addHessianSparsity(int ele, std::vector<EigenSupport::TripletD> &entries) const = 0;
  virtual void buildLocalToGlobalMatrixIndices(int ele, const EigenSupport::SpMatD &KTemplate,
    DynamicIndexMatrix &indices) const = 0;
};
```

`DeformationModelAssembler` 的 loops 仍可保留，但不再直接假设 `vertexIndices[v] * 3 + dof`。

## 目标 C++ API

新增或修改：

```cpp
enum class MaterialPayloadKind
{
  ENU,
  MOONEY_RIVLIN,
  ORTHOTROPIC,
  HILL_ACTIVATION,
};

enum class PassiveElasticLaw
{
  STABLE_NEO,
  LINEAR,
  STVK,
  STVK_VOL,
  MOONEY_RIVLIN,
  ORTHOTROPIC_STVK,
};

enum class ActiveElasticTerm
{
  HILL_FIBER,
};

struct MaterialSlotRef
{
  int slot = 0;
};

struct FiberFieldSpec
{
  const double *elementFiberDirections = nullptr;
  const double *vertexFiberDirections = nullptr;
};

struct PassiveElasticSpec
{
  PassiveElasticLaw law = PassiveElasticLaw::STABLE_NEO;
  MaterialSlotRef material;
};

struct HillFiberSpec
{
  MaterialSlotRef material = { 1 };
  FiberFieldSpec fibers;
};

struct ElasticModelSpec
{
  PassiveElasticSpec passive;
  std::vector<HillFiberSpec> hillFiberTerms;
};

struct DeformationModelSpec
{
  ElasticModelSpec elastic;
  DeformationModelPlasticMaterial plastic = DeformationModelPlasticMaterial::VOLUMETRIC_DOF6;
  DeformationModelOptions options;
};

template<TetFormulation F>
DeformationModelBundle makeTetDeformationModel(
  const VolumetricMeshes::TetMesh &mesh,
  const F &formulation,
  const DeformationModelSpec &spec);

template<CubicFormulation F>
DeformationModelBundle makeCubicDeformationModel(
  const VolumetricMeshes::CubicMesh &mesh,
  const F &formulation,
  const DeformationModelSpec &spec);

template<ShellFormulation F>
DeformationModelBundle makeShellDeformationModel(
  const Mesh::TriMeshGeo &surface,
  const F &formulation,
  const DeformationModelSpec &spec);

template<TetFormulation F>
DeformationModelBundle makeTetDeformationModel(
  std::unique_ptr<SimulationMesh> mesh,
  const F &formulation,
  const DeformationModelSpec &spec);

template<CubicFormulation F>
DeformationModelBundle makeCubicDeformationModel(
  std::unique_ptr<SimulationMesh> mesh,
  const F &formulation,
  const DeformationModelSpec &spec);

template<ShellFormulation F>
DeformationModelBundle makeShellDeformationModel(
  std::unique_ptr<SimulationMesh> mesh,
  const F &formulation,
  const DeformationModelSpec &spec);
```

上面是 Task 7 之后的 milestone exit API。为了让重构可验证，Task 2 到 Task 6 的 topology-specific factory 可以临时接收 legacy `DeformationModelElasticMaterial`，但这个 legacy field 只能存在于过渡 call path；Task 7 完成后 public factory 必须切到 `ElasticModelSpec`。

旧 `makeDeformationModel(...)` overload 不保留。现有调用方必须迁移到 topology-specific factory。旧 elastic enum 不进入 Task 7 之后的 public factory API；下面表格只是人工迁移对照，不实现 compatibility translator：

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

## 目标 Python API

新增 public module：

```text
pypgo.fem
pypgo.energy
```

示例：

```python
import pypgo as pgo

volume = pgo.mesh.veg.VolumeMesh(cube_data, pgo.mesh.veg.ENuMaterial(E=1e6, nu=0.45))
sim_mesh = pgo.sim.SimulationMesh.create_volumetric(volume)

energy = pgo.energy.deformation_energy(
    sim_mesh,
    formulation=pgo.fem.HexTrilinear(),
    elastic=pgo.energy.StableNeo(),
    plastic="volumetric_dof6",
)

u = energy.zero_state()
value = energy.value(u)
grad = energy.gradient(u)
H = energy.hessian(u)
rows, cols, values = H.to_coo()
```

Python formulation objects：

```python
pgo.fem.TetP1()
pgo.fem.HexTrilinear()
pgo.fem.HexTricubicHermite(...)  # added in Task 9, raises NotImplementedError until Hermite core lands
pgo.fem.ShellKoiter()
```

Python elastic law / recipe objects：

```python
pgo.energy.StableNeo(material_slot=0)
pgo.energy.StVK(material_slot=0)
pgo.energy.MooneyRivlin(material_slot=0)
pgo.energy.OrthotropicStVK(material_slot=0)
pgo.energy.HillFiber(
    base=pgo.energy.StableNeo(material_slot=0),
    hill_slot=1,
    element_fibers=fibers,
)
```

第一版规则：

- Tet mesh 可显式传 `TetP1()`；如果 formulation omitted，可默认 `TetP1()`。
- Cubic mesh 必须显式传 `HexTrilinear()`；不传时抛 `ValueError`，提示当前 cubic topology 有多个未来 formulation。
- Task 4 只承诺 `ENu + StableNeo/StVK` deformation energy，因为当前 `SimulationMesh.create_volumetric` 仍只接受 ENu solver payload；这一步先把 energy API、state convention、ownership 和 sparse return 跑通。
- Task 7 才开放 `MooneyRivlin + MooneyRivlin law` 和 `base ENu law + HillFiber`，并添加 payload/law mismatch 错误，例如 `MooneyRivlin(material_slot=0)` 但 slot 0 是 ENu payload。
- Task 8 才开放 `Orthotropic + OrthotropicStVK law`。
- Task 9 才添加 `HexTricubicHermite()` Python dataclass；`deformation_energy(..., HexTricubicHermite())` 必须抛 `NotImplementedError`，不能静默落到 trilinear。
- Hill 必须要求 `HillActivation` payload 和 fiber field；不能把 Hill 当作无方向的 passive law。

## Topology/Formulation Matrix

| Topology | Formulation | DofLayout | 状态 |
|---|---|---|---|
| `TetMesh` / `SimulationMeshType::TET` | `TetP1` | `Vertex3DofLayout` | 抽成 `TetP1Kernel` + `DeformationGradientElementModel<TetP1Kernel>`，行为必须保持 |
| `CubicMesh` / `SimulationMeshType::CUBIC` | `HexTrilinear` | `Vertex3DofLayout` | 抽成 `HexTrilinearKernel` + `DeformationGradientElementModel<HexTrilinearKernel>`，当前 cubic 行为必须显式命名 |
| `CubicMesh` / `SimulationMeshType::CUBIC` | `HexTricubicHermite` | `HermiteDofLayout` | future，当前抛 `not implemented` |
| shell `SimulationMeshType::SHELL` | `ShellKoiter` | `Vertex3DofLayout` with invalid local sentinel | 当前 shell 行为，Python deformation energy 第一版可不暴露 |
| `SimulationMeshType::TRIANGLE` / `EDGE_QUAD` | none | none | 不在本计划 deformation energy 范围内 |

## Material Recipe Matrix

| Payload slot type | Elastic recipe | Solver model | 状态 |
|---|---|---|---|
| `SimulationMeshENuMaterial` | `StableNeo` | `ElasticModelStableNeoHookeanMaterial` | 当前行为，必须保持 |
| `SimulationMeshENuMaterial` | `Linear` | `ElasticModelLinearMaterial` | 当前行为，通过新 recipe 表达 |
| `SimulationMeshENuMaterial` | `StVK` | `ElasticModel3DSTVKMaterial` or invariant StVK path | 当前行为，通过新 recipe 表达 |
| `SimulationMeshMooneyRivlinMaterial` | `MooneyRivlin` | `ElasticModel3DMooneyRivlin` | 纳入 Python 一般承诺 |
| `SimulationMeshOrthotropicMaterial` | `OrthotropicStVK` | new `ElasticModel3DOrthotropicStVK` | 本计划补齐后纳入 Python 一般承诺 |
| `SimulationMeshENuMaterial` + `SimulationMeshHillMaterial` + fibers | `HillFiber(base=StableNeo/StVK/StVKVol)` | `ElasticModelCombinedMaterial` | 纳入 Python 一般承诺，但作为 composite recipe |

不支持的组合必须 fail fast。例如 `StableNeo` 不能读取 `MooneyRivlin` payload，`HillFiber` 不能缺少 fiber direction，`OrthotropicStVK` 不能在 `ElasticModel3DOrthotropicStVK` 测试通过前开放。

## File Map

### 新增

- `src/core/solidDeformationModel/formulations/deformationFormulations.h`
- `src/core/solidDeformationModel/formulations/formulationTraits.h`
- `src/core/solidDeformationModel/formulations/formulationConcepts.h`
- `src/core/solidDeformationModel/formulations/formulationVariants.h`
- `src/core/solidDeformationModel/deformationModelFactory.inl`
- `src/core/solidDeformationModel/formulations/kernels/deformationGradientKernel.h`
- `src/core/solidDeformationModel/formulations/kernels/tetP1Kernel.h`
- `src/core/solidDeformationModel/formulations/kernels/tetP1Kernel.cpp`
- `src/core/solidDeformationModel/formulations/kernels/hexTrilinearKernel.h`
- `src/core/solidDeformationModel/formulations/kernels/hexTrilinearKernel.cpp`
- `src/core/solidDeformationModel/formulations/elements/deformationGradientElementModel.h`
- `src/core/solidDeformationModel/formulations/elements/deformationGradientElementModel.inl`
- `src/core/solidDeformationModel/formulations/elements/deformationGradientElementModel.cpp`
- `src/core/solidDeformationModel/formulations/dof/dofLayout.h`
- `src/core/solidDeformationModel/formulations/dof/vertex3DofLayout.h`
- `src/core/solidDeformationModel/formulations/dof/vertex3DofLayout.cpp`
- `src/core/solidDeformationModel/materials/elasticModelSpec.h`
- `src/core/solidDeformationModel/materials/simulationMeshMaterialPayload.h`
- `src/core/solidDeformationModel/materials/simulationMeshOrthotropicMaterial.h`
- `src/core/solidDeformationModel/factories/elasticModelFactory.h`
- `src/core/solidDeformationModel/factories/elasticModelFactory.cpp`
- `src/core/solidDeformationModel/factories/plasticModelFactory.h`
- `src/core/solidDeformationModel/factories/plasticModelFactory.cpp`
- `src/core/solidDeformationModel/factories/elementModelFactory.h`
- `src/core/solidDeformationModel/factories/elementModelFactory.cpp`
- `src/core/solidDeformationModel/elasticModel3DOrthotropicStVK.h`
- `src/core/solidDeformationModel/elasticModel3DOrthotropicStVK.cpp`
- `src/python/pypgo/bindings/simulation_mesh_core.h`
- `src/python/pypgo/bindings/energy_bindings.cpp`
- `pypgo/fem.py`
- `pypgo/energy.py`
- `tests/pypgo/test_deformation_energy.py`
- `tests/src/core/solidDeformationModel/formulations/deformationModelFormulation_gtest.cpp`
- `tests/src/core/solidDeformationModel/formulations/kernels/tetP1Kernel_gtest.cpp`
- `tests/src/core/solidDeformationModel/formulations/kernels/hexTrilinearKernel_gtest.cpp`
- `tests/src/core/solidDeformationModel/formulations/elements/deformationGradientElementModel_gtest.cpp`
- `tests/src/core/solidDeformationModel/formulations/dof/vertex3DofLayout_gtest.cpp`
- `tests/src/core/solidDeformationModel/factories/elasticModelFactory_gtest.cpp`
- `tests/src/core/solidDeformationModel/factories/elementModelFactory_gtest.cpp`
- `tests/src/core/solidDeformationModel/elasticModel3DOrthotropicStVK_gtest.cpp`

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

- [ ] Add C++ baseline test: current tet `makeDeformationModel(...)` at zero displacement has finite near-zero energy and finite gradient.
- [ ] Add C++ baseline test: current cubic `makeDeformationModel(...)` at zero displacement has finite near-zero energy and finite gradient.
- [ ] Add C++ baseline test: current cubic `makeDeformationModel(...)` facade matches manual manager/assembler chain, mirroring the existing tet factory test.
- [ ] Add C++ characterization test: current ENu load path produces `SimulationMeshENuMaterial` payloads.
- [ ] Add C++ characterization test: current Mooney-Rivlin `.veg` / `VolumeMesh` payloads exist before deformation conversion.
- [ ] Add C++ characterization test: current Hill path requires an extra `SimulationMeshHillMaterial` slot and fiber directions.
- [ ] Add C++ characterization test: current Orthotropic payload can be read from Vega but has no deformation `ElasticModel` yet.
- [ ] Add Python test: `pypgo.mesh.veg.VolumeMesh` can carry ENu, Mooney-Rivlin, and Orthotropic regions before simulation conversion.
- [ ] Add Python test note or assertion that `SimulationMesh.mesh_type == "cubic"` is topology metadata, not formulation metadata.

**Exit criteria:**

- Current behavior is covered before any refactor.
- Tests clarify that `DeformationModelEnergy` state input is displacement when rest position is stored.
- Tests document which material capabilities are payload-only today and which already reach deformation energy.
- No Task 0 test may reference `TetP1`, `HexTrilinear`, `makeTetDeformationModel`, `makeCubicDeformationModel`, or `ElasticModelSpec`; those symbols are introduced later and must not be prerequisites for baseline characterization.

## Task 1: Extract `TetP1Kernel`, `HexTrilinearKernel`, And Shared Element Model

**目标：** 先把 tet P1 和 hex trilinear 的 formulation 数学从旧 element model 中抽出来，再让二者共同走 `DeformationGradientElementModel<Kernel>`。这一步必须先于 `FormulationTraits` 和 topology-specific factory，否则 traits 会被迫引用旧 model 作为过渡胶水。

**Files:**

- Create: `src/core/solidDeformationModel/formulations/kernels/deformationGradientKernel.h`
- Create: `src/core/solidDeformationModel/formulations/kernels/tetP1Kernel.h`
- Create: `src/core/solidDeformationModel/formulations/kernels/tetP1Kernel.cpp`
- Create: `src/core/solidDeformationModel/formulations/kernels/hexTrilinearKernel.h`
- Create: `src/core/solidDeformationModel/formulations/kernels/hexTrilinearKernel.cpp`
- Create: `src/core/solidDeformationModel/formulations/elements/deformationGradientElementModel.h`
- Create: `src/core/solidDeformationModel/formulations/elements/deformationGradientElementModel.inl`
- Create: `src/core/solidDeformationModel/formulations/elements/deformationGradientElementModel.cpp`
- Modify: `src/core/solidDeformationModel/tetMeshDeformationModel.h`
- Modify: `src/core/solidDeformationModel/tetMeshDeformationModel.cpp`
- Modify: `src/core/solidDeformationModel/cubicMeshDeformationModel.h`
- Modify: `src/core/solidDeformationModel/cubicMeshDeformationModel.cpp`
- Modify: `src/core/solidDeformationModel/CMakeLists.txt`
- Create: `tests/src/core/solidDeformationModel/formulations/kernels/tetP1Kernel_gtest.cpp`
- Create: `tests/src/core/solidDeformationModel/formulations/kernels/hexTrilinearKernel_gtest.cpp`
- Create: `tests/src/core/solidDeformationModel/formulations/elements/deformationGradientElementModel_gtest.cpp`
- Modify: `tests/src/core/solidDeformationModel/CMakeLists.txt`

- [ ] Add `TetP1Kernel` with rest-shape precomputation currently embedded in `TetMeshDeformationModel`.
- [ ] Add `HexTrilinearKernel` with rest-shape precomputation currently embedded in `CubicMeshDeformationModel`.
- [ ] Keep kernels free of `ElasticModel`, `PlasticModel`, SVD, stress, material parameters, and energy accumulation.
- [ ] Kernel API must expose at least:
  - `static constexpr int numNodes`
  - `static constexpr int localDofs`
  - `int numQuadraturePoints() const`
  - `double weightDetJ(int q) const`
  - `void computeFref(const double *xLocal, int q, double F[9]) const`
  - `void computedFrefdx(int q, double *dFdx) const`
- [ ] Add kernel unit tests:
  - rest state gives `Fref == I`.
  - uniform translation leaves `Fref` unchanged.
  - affine deformation `x = A X + b` gives `Fref == A`.
  - quadrature weight sum equals element volume.
  - `computedFrefdx` matches finite difference.
- [ ] Add `DeformationGradientElementModel<Kernel>` that implements the existing `DeformationModel` virtual interface by combining:
  - formulation kinematics from `Kernel`
  - `ElasticModel3DDeformationGradient`
  - `PlasticModel3DDeformationGradient`
  - existing plastic/material derivative paths
- [ ] Add regression tests comparing old and new element models on the same tet and cubic single-element fixtures:
  - energy
  - `compute_dE_dx`
  - `compute_d2E_dx2`
  - `compute_d2E_dxda`
  - `compute_d2E_dxdb`
  - stress routines where supported
  - local material max-step where supported
- [ ] Turn `TetMeshDeformationModel` into a compatibility wrapper around `DeformationGradientElementModel<TetP1Kernel>` after regression tests pass.
- [ ] Turn `CubicMeshDeformationModel` into a compatibility wrapper around `DeformationGradientElementModel<HexTrilinearKernel>` after regression tests pass.
- [ ] Keep the full `examples/ipc/cubic/box/box-ipc.json` run as final smoke only; do not use it as the first regression guard.

**Exit criteria:**

- `TetP1Kernel` and `HexTrilinearKernel` unit tests pass.
- New shared element model matches old tet and cubic element outputs within tight numerical tolerance.
- Old public element-model class names can still be used internally during migration, but their implementations delegate to the new shared element model.
- `FormulationTraits<TetP1>` and `FormulationTraits<HexTrilinear>` can point to real kernel types without referencing old full element models.

## Task 2: Split Deformation Factory By Topology

**目标：** 移除旧的 runtime auto-dispatch `makeDeformationModel(...)`，改为 `makeTetDeformationModel` / `makeCubicDeformationModel` / `makeShellDeformationModel`。Formulation 用 tag object 表达，topology 由函数签名表达，让错误组合在函数签名层面不可表达。本任务只拆 topology/formulation 入口，material 输入可以继续临时使用 legacy `DeformationModelElasticMaterial`；Task 7 再切到 `ElasticModelSpec`。

**Files:**

- Create: `src/core/solidDeformationModel/formulations/deformationFormulations.h`
- Create: `src/core/solidDeformationModel/formulations/formulationTraits.h`
- Create: `src/core/solidDeformationModel/formulations/formulationConcepts.h`
- Create: `src/core/solidDeformationModel/formulations/formulationVariants.h`
- Create: `src/core/solidDeformationModel/formulations/dof/dofLayout.h`
- Create: `src/core/solidDeformationModel/formulations/dof/vertex3DofLayout.h`
- Create: `src/core/solidDeformationModel/deformationModelFactory.inl`
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

- [ ] Add formulation tag objects for currently implemented paths: `TetP1`, `HexTrilinear`, and `ShellKoiter`.
- [ ] Do not add `HexTricubicHermite` in Task 2; it is a Task 9 placeholder so the not-implemented branch is introduced in one place.
- [ ] Add `FormulationTraits<Formulation>` specializations for each tag.
- [ ] Make each traits specialization declare `DofLayout`, `Kernel`, and `ElementModel`; do not declare concrete elastic or plastic types in traits. `DofLayout` may be a forward-declared `Vertex3DofLayout` here; the assembler migration to actually use it happens in Task 6.
- [ ] Bind `FormulationTraits<TetP1>` to `TetP1Kernel` and `DeformationGradientElementModel<TetP1Kernel>`.
- [ ] Bind `FormulationTraits<HexTrilinear>` to `HexTrilinearKernel` and `DeformationGradientElementModel<HexTrilinearKernel>`.
- [ ] Do not point traits at legacy full element models such as `TetMeshDeformationModel` or `CubicMeshDeformationModel`.
- [ ] Add `TetFormulation`, `CubicFormulation`, and `ShellFormulation` C++20 concepts; do not add virtual formulation base classes.
- [ ] Add `makeTetDeformationModel` template constrained by `TetFormulation`.
- [ ] Add `makeCubicDeformationModel` template constrained by `CubicFormulation`.
- [ ] Add `makeShellDeformationModel` template constrained by `ShellFormulation`.
- [ ] Add runtime boundary variants:
  - `using TetFormulationVariant = std::variant<TetP1>;`
  - `using CubicFormulationVariant = std::variant<HexTrilinear>;`
  - `using ShellFormulationVariant = std::variant<ShellKoiter>;`
- [ ] Add variant adapter overloads only for binding/config code paths; core code should call the constrained templates directly.
- [ ] Put constrained template definitions in `deformationModelFactory.inl` and include it from `deformationModelFactory.h`; keep only non-template helpers in `deformationModelFactory.cpp`.
- [ ] Remove public `makeDeformationModel(...)` overloads instead of wrapping them.
- [ ] Update all in-repo call sites to use topology-specific factories, including:
  - `src/tools/sim/runIPCSim/setup/femSetup.cpp`
  - `src/c/pgo_c.cpp`
  - `tests/src/tools/runSimShared_gtest.cpp`
- [ ] Pass topology-specific formulation into `DeformationModelManager` construction path.
- [ ] Add formulation validation inside each topology-specific factory:
  - `TetP1` works only with tet topology.
  - `HexTrilinear` works with cubic topology.
  - `ShellKoiter` works only with shell topology.
- [ ] Update logs/errors that currently say only `CUBIC` to include `TRILINEAR` when creating element FEMs.
- [ ] Add C++ tests for all currently implemented topology/formulation rows and for absence of the old public auto-dispatch entry. The Hermite row is tested in Task 9 when its placeholder is introduced.
- [ ] Add compile-time tests or `static_assert`s that wrong topology/formulation combinations are not invocable.

**Exit criteria:**

- `makeCubicDeformationModel(..., HexTrilinear{}, ...)` works.
- `makeTetDeformationModel(..., TetP1{}, ...)` works.
- Old `makeDeformationModel(...)` public entry is gone from the header.
- Wrong topology/formulation combinations fail at compile time for core template calls.
- Task 2 may still carry legacy elastic enum inputs, but no public call site may still depend on runtime mesh-type auto-dispatch.

## Task 3: Add `SimulationMesh::clone()` For Python Energy Construction

**目标：** Python `SimulationMesh` 可以被多个 energy factory 调用复用，而不暴露或消费 unique ownership。

**Files:**

- Modify: `src/core/solidDeformationModel/simulationMesh.h`
- Modify: `src/core/solidDeformationModel/simulationMesh.cpp`
- Create: `src/python/pypgo/bindings/simulation_mesh_core.h`
- Modify: `src/python/pypgo/bindings/mesh_bindings.cpp`
- Modify: `tests/src/core/solidDeformationModel/simulationMesh_gtest.cpp`
- Modify: `tests/pypgo/test_simulation_mesh.py`

- [ ] Add `std::unique_ptr<SimulationMesh> clone() const`.
- [ ] Implement `SimulationMeshImpl` copy/clone that deep-copies:
  - vertices
  - elements
  - element UVs
  - element material id lists
  - materials through `SimulationMeshMaterial::clone()`
  - mesh topology type
- [ ] Add C++ test: clone preserves tet/cubic/shell counts, element vertex ids, material type and values.
- [ ] Move `SimulationMeshCore` out of `mesh_bindings.cpp` into `src/python/pypgo/bindings/simulation_mesh_core.h` so `energy_bindings.cpp` can use the same C++ wrapper type.
- [ ] Add `SimulationMeshCore::cloneMesh()` helper in that shared header for energy construction.
- [ ] Do not expose `clone()` as a public Python method unless needed for user workflows.

**Exit criteria:**

- Creating deformation energy from a Python `SimulationMesh` does not mutate or consume the Python mesh object.
- Two energy objects can be created from the same `SimulationMesh`.

## Task 4: Bind Current Deformation Energy To Python

**目标：** 暴露当前 tet P1 和 hex trilinear deformation energy，命名清楚，API 小而稳定。

**Files:**

- Create: `src/python/pypgo/bindings/energy_bindings.cpp`
- Modify: `src/python/pypgo/bindings/simulation_mesh_core.h`
- Modify: `src/python/pypgo/bindings/module.cpp`
- Modify: `src/python/pypgo/CMakeLists.txt`
- Create: `pypgo/fem.py`
- Create: `pypgo/energy.py`
- Modify: `pypgo/__init__.py`
- Modify: `pypgo/sim.py`
- Create: `tests/pypgo/test_deformation_energy.py`

- [ ] Add topology-specific `_core` formulation payloads or parser helpers for:
  - `tet_p1`
  - `hex_trilinear`
  - `shell_koiter`
- [ ] Use the shared `SimulationMeshCore` declaration from `simulation_mesh_core.h`; do not duplicate or forward-declare a private class in `energy_bindings.cpp`.
- [ ] Map Python formulation dataclasses to C++ runtime variants at the binding boundary, then call the variant adapter.
- [ ] Add `_core.create_tet_deformation_energy(simulation_mesh, tet_formulation, elastic, plastic, options)`.
- [ ] Add `_core.create_cubic_deformation_energy(simulation_mesh, cubic_formulation, elastic, plastic, options)`.
- [ ] Add `_core.create_shell_deformation_energy(simulation_mesh, shell_formulation, elastic, plastic, options)` only if shell deformation energy is exposed in this milestone.
- [ ] Add `DeformationEnergyCore` with:
  - `num_dofs()`
  - `rest_position_flat()`
  - `zero_state()`
  - `value(u)`
  - `gradient(u)`
  - `hessian(u)` returning `SparseMatrixCore`
  - optional `max_step(u, du)`
- [ ] Release the GIL around energy/gradient/hessian computations.
- [ ] Validate NumPy input shape:
  - `u.shape == (num_dofs,)`
  - dtype coerces to `float64` in Python wrapper
  - contiguous array passed to `_core`
- [ ] Add `pypgo.fem` formulation dataclasses:
  - `TetP1`
  - `HexTrilinear`
  - `ShellKoiter`
- [ ] Add `pypgo.energy.deformation_energy(...)` wrapper that maps Python strings/classes to `_core`.
- [ ] Enforce Python policy:
  - cubic requires explicit `HexTrilinear()`
  - Task 4 accepts only `StableNeo(material_slot=0)` and `StVK(material_slot=0)` over ENu payloads
  - unsupported recipes such as `MooneyRivlin`, `HillFiber`, and `OrthotropicStVK` raise clear `NotImplementedError` / `ValueError` until Task 7 or Task 8
- [ ] Add Python tests:
  - tet energy builds and `zero_state()` has correct shape.
  - cubic `HexTrilinear()` energy builds and has `num_dofs == 3 * num_vertices`.
  - cubic without formulation raises helpful `ValueError`.
  - `MooneyRivlin`, `HillFiber`, and `OrthotropicStVK` fail with messages that point to the later material tasks.
  - `value`, `gradient`, `hessian.to_coo()` smoke tests pass at zero state and a small perturbation.
  - same `SimulationMesh` can create two independent energies.

**Exit criteria:**

- User can build current deformation energy from Python without seeing manager/assembler.
- Current cubic path is publicly named `hex_trilinear`.
- Python examples use displacement vector state, not absolute rest positions.
- Task 4 Python support is intentionally limited to ENu-backed passive laws; Mooney-Rivlin, Hill, Orthotropic, and Hermite are not claimed until their later tasks complete.

## Task 5: Split `DeformationModelManager::initImpl` Into Internal Factories

**目标：** 降低 manager 的职责复杂度，为 future formulation 增量接入准备边界。这一步必须 behavior-preserving：先把现有 legacy elastic/plastic enum 的分支搬进内部 factory，不同时引入 material recipe 语义迁移。

**Files:**

- Create: `src/core/solidDeformationModel/factories/elasticModelFactory.h`
- Create: `src/core/solidDeformationModel/factories/elasticModelFactory.cpp`
- Create: `src/core/solidDeformationModel/factories/plasticModelFactory.h`
- Create: `src/core/solidDeformationModel/factories/plasticModelFactory.cpp`
- Create: `src/core/solidDeformationModel/factories/elementModelFactory.h`
- Create: `src/core/solidDeformationModel/factories/elementModelFactory.cpp`
- Modify: `src/core/solidDeformationModel/formulations/elements/deformationGradientElementModel.h`
- Modify: `src/core/solidDeformationModel/formulations/elements/deformationGradientElementModel.cpp`
- Modify: `src/core/solidDeformationModel/deformationModelManager.cpp`
- Modify: `src/core/solidDeformationModel/CMakeLists.txt`
- Modify: `tests/src/core/solidDeformationModel/deformationModelFactory_gtest.cpp`
- Modify: `tests/src/core/solidDeformationModel/deformationModelAssembler_gtest.cpp`

- [ ] Extract elastic model creation into `ElasticModelFactory`.
- [ ] Extract plastic model creation into `PlasticModelFactory`.
- [ ] Extract element FEM creation into `ElementModelFactory`.
- [ ] Use the existing `DeformationGradientElementModel<Kernel>` wrapper from Task 1 for both `TetP1Kernel` and `HexTrilinearKernel`.
- [ ] Make `ElementModelFactory` construct `DeformationGradientElementModel<Kernel>` with runtime `ElasticModel *` and `PlasticModel *` dependencies supplied by the factories.
- [ ] Do not instantiate `ElementModel<Kernel, StableNeo, Plastic6Dof>`-style combinations in the first version; avoid material/plastic template explosion.
- [ ] Keep `ElasticModelFactory` consuming legacy `DeformationModelElasticMaterial` in Task 5 so this extraction can be tested independently of material recipe migration.
- [ ] Do not remove `DeformationModelElasticMaterial` from the factory call path in Task 5; Task 7 performs that public API migration after payload conversion and mismatch tests exist.
- [ ] Pass the formulation tag object or its `FormulationTraits` type into `ElementModelFactory`.
- [ ] Preserve the current manager-owned storage shape initially:
  - vectors of specific material/model pointer types may remain in `DeformationModelManagerImpl`.
  - `elementMaterials` and `elementFEMs` can stay borrowed pointer arrays.
- [ ] After behavior tests pass, optionally replace owning pointer vectors with `std::unique_ptr` vectors in a separate substep.
- [ ] Add focused tests that compare energy/gradient/Hessian before and after extraction for tet and cubic.

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
- `FormulationTraits` selects the formulation kernel and generic element-model wrapper; material/plastic selection remains runtime-injected by the factories.

## Task 6: Introduce `DofLayout` With `Vertex3DofLayout`

**目标：** 移除 assembler 对 `3 * numVertices` 和 `numElementVertices * 3` 的硬编码依赖，同时保持现有路径行为不变。

**Files:**

- Modify: `src/core/solidDeformationModel/formulations/dof/dofLayout.h`
- Modify: `src/core/solidDeformationModel/formulations/dof/vertex3DofLayout.h`
- Create: `src/core/solidDeformationModel/formulations/dof/vertex3DofLayout.cpp`
- Modify: `src/core/solidDeformationModel/deformationModelAssembler.h`
- Modify: `src/core/solidDeformationModel/deformationModelAssembler.cpp`
- Modify: `src/core/solidDeformationModel/deformationModelFactory.inl`
- Modify: `src/core/solidDeformationModel/deformationModelFactory.cpp`
- Modify: `src/core/solidDeformationModel/CMakeLists.txt`
- Create: `tests/src/core/solidDeformationModel/formulations/dof/vertex3DofLayout_gtest.cpp`
- Modify: `tests/src/core/solidDeformationModel/deformationModelAssembler_gtest.cpp`

- [ ] Complete the `DofLayout` abstract interface that Task 2 introduced as a traits-visible declaration.
- [ ] Complete `Vertex3DofLayout` so it wraps a `SimulationMesh`.
- [ ] Move `gatherLocalPositions(...)` logic from assembler helper into `Vertex3DofLayout::gather`.
- [ ] Move gradient scatter logic into `Vertex3DofLayout::scatterAddGradient`.
- [ ] Move Hessian sparsity construction into `Vertex3DofLayout::addHessianSparsity`.
- [ ] Move local-to-global sparse index lookup into layout helper.
- [ ] Change `DeformationModelAssembler` constructor to own or borrow a `std::unique_ptr<const DofLayout>`.
- [ ] Update all assembler construction call sites to pass an explicit `Vertex3DofLayout`; do not keep an implicit compatibility constructor.
- [ ] Change assembler fields:
  - `n3` -> `numDOFs`
  - `localDOFs` becomes per-element query or cached from layout
  - `nvtx` only remains if needed for legacy diagnostics
- [ ] Add tests comparing old expected DOF counts:
  - tet one element: 12
  - cubic one element: 24
  - shell one triangle: `3 * num_surface_vertices` global DOFs and 18 local DOFs
- [ ] Add energy/gradient/Hessian parity tests for tet and cubic after layout migration.

**Exit criteria:**

- Assembler no longer directly computes global DOF indices from vertex ids.
- Existing vertex DOF path remains numerically identical.
- Future Hermite layout can be added without editing every assembler gather/scatter loop.

## Task 7: Material Payload And Elastic Recipe Refactor

**目标：** 把材料参数、passive law、Hill active term 拆开，让 Mooney-Rivlin、Hill、Orthotropic 可以作为 Python deformation API 的一般材料承诺，而不是散落在 `.veg` I/O、`SimulationMeshMaterial`、`DeformationModelElasticMaterial` 三套表达里。

**Files:**

- Modify: `src/core/solidDeformationModel/simulationMesh.cpp`
- Modify: `src/core/solidDeformationModel/simulationMesh.h`
- Create: `src/core/solidDeformationModel/materials/simulationMeshMaterialPayload.h`
- Create: `src/core/solidDeformationModel/materials/simulationMeshOrthotropicMaterial.h`
- Create: `src/core/solidDeformationModel/materials/elasticModelSpec.h`
- Modify: `src/core/solidDeformationModel/factories/elasticModelFactory.h`
- Modify: `src/core/solidDeformationModel/factories/elasticModelFactory.cpp`
- Modify: `src/core/solidDeformationModel/deformationModelManager.h`
- Modify: `src/core/solidDeformationModel/deformationModelManager.cpp`
- Modify: `src/core/solidDeformationModel/deformationModelFactory.h`
- Modify: `src/core/solidDeformationModel/deformationModelFactory.inl`
- Modify: `src/core/solidDeformationModel/deformationModelFactory.cpp`
- Modify: `src/c/pgo_c.cpp`
- Modify: `src/tools/sim/runIPCSim/setup/femSetup.h`
- Modify: `src/tools/sim/runIPCSim/setup/femSetup.cpp`
- Modify: `src/tools/sim/runIPCSim/setup/setupCommon.h`
- Modify: `src/tools/sim/runIPCSim/setup/setupCommon.cpp`
- Modify: `src/tools/sim/runIPCSim/setup/volumeSetup.cpp`
- Modify: `src/tools/sim/runIPCSim/setup/legacySetup.cpp`
- Modify: `src/python/pypgo/bindings/mesh_bindings.cpp`
- Modify: `pypgo/mesh/veg.py`
- Modify: `pypgo/energy.py`
- Modify: `tests/src/core/solidDeformationModel/simulationMesh_gtest.cpp`
- Create: `tests/src/core/solidDeformationModel/factories/elasticModelFactory_gtest.cpp`
- Modify: `tests/pypgo/test_deformation_energy.py`

- [ ] Add `MaterialPayloadKind` introspection to `SimulationMeshMaterial`.
- [ ] Add `SimulationMeshOrthotropicMaterial` with `E1/E2/E3`, `nu12/nu23/nu31`, `G12/G23/G31`, and row-major `R`.
- [ ] Update `loadTetMesh` / `loadCubicMesh` to convert all supported Vega material payloads:
  - ENu -> `SimulationMeshENuMaterial`
  - Mooney-Rivlin -> `SimulationMeshMooneyRivlinMaterial`
  - Orthotropic -> `SimulationMeshOrthotropicMaterial`
- [ ] Preserve per-element material region mapping when converting multi-material Vega volume meshes.
- [ ] Add `ElasticModelSpec`, `PassiveElasticSpec`, `HillFiberSpec`, and `FiberFieldSpec`.
- [ ] Remove legacy elastic enum usage from new deformation factory code:
  - manually migrate `STABLE_NEO` call sites to `StableNeo(slot 0)`
  - manually migrate `MOONEY_RIVLIN` call sites to `MooneyRivlin(slot 0)`
  - manually migrate `HILL_STABLE_NEO` call sites to `StableNeo(slot 0) + HillFiber(slot 1)`
  - apply the same manual mapping for `HILL_STVK` and `HILL_STVK_VOL`
- [ ] Update runIPCSim setup and C API adapters to build explicit `ElasticModelSpec` values instead of passing `DeformationModelElasticMaterial` into public factories.
- [ ] Make `ElasticModelFactory` validate payload/law compatibility before allocating models.
- [ ] Define Mooney-Rivlin payload mapping:
  - either support Vega `mu01/mu10/v1` directly with a dedicated conversion helper;
  - or convert to `SimulationMeshMooneyRivlinMaterial(N, M, Cpq, D)` with documented `Cpq/D` layout.
- [ ] Add Hill as an active term:
  - require base passive payload at slot 0;
  - require `SimulationMeshHillMaterial` at `hill_slot`;
  - require element or vertex fiber directions;
  - produce `ElasticModelCombinedMaterial` internally.
- [ ] Add Python elastic recipe wrappers:
  - `StableNeo`
  - `StVK`
  - `MooneyRivlin`
  - `OrthotropicStVK`
  - `HillFiber(base=..., hill_slot=..., element_fibers=...)`
- [ ] Add Python tests for payload/law mismatch errors.
- [ ] Move the deferred Python material tests from Task 4 into this task:
  - Mooney-Rivlin energy builds when the mesh payload is Mooney-Rivlin and fails clearly when payload/law mismatch.
  - Hill fiber energy builds when base payload, hill payload, and fiber directions are present.
- [ ] Add C++ tests that `ElasticModelFactory` builds ENu, Mooney-Rivlin, and Hill composite models from explicit specs.

**Exit criteria:**

- `SimulationMesh` can carry ENu, Mooney-Rivlin, Orthotropic, and Hill payloads without losing type information.
- `ElasticModelFactory` is the only place that maps payload + recipe to solver `ElasticModel`.
- Mooney-Rivlin and Hill composite deformation energies are buildable from Python and covered by smoke/FD tests.
- Orthotropic is visible in the same recipe API but only reports supported after Task 8 lands.

## Task 8: Add Orthotropic Solver Elastic Law

**目标：** 把 Orthotropic 从 `.veg`/payload 支持推进到 solver-ready deformation law，使 `pgo.energy.OrthotropicStVK()` 成为真实支持，而不是只读写材料参数。

**Files:**

- Create: `src/core/solidDeformationModel/elasticModel3DOrthotropicStVK.h`
- Create: `src/core/solidDeformationModel/elasticModel3DOrthotropicStVK.cpp`
- Modify: `src/core/solidDeformationModel/factories/elasticModelFactory.cpp`
- Modify: `src/core/solidDeformationModel/CMakeLists.txt`
- Create: `tests/src/core/solidDeformationModel/elasticModel3DOrthotropicStVK_gtest.cpp`
- Modify: `tests/src/core/solidDeformationModel/factories/elasticModelFactory_gtest.cpp`
- Modify: `tests/pypgo/test_deformation_energy.py`

- [ ] Define the exact Orthotropic law first. Recommended first law: small/finite strain StVK-style orthotropic material in the local material frame `R`.
- [ ] Build local-frame stiffness matrix from `E1/E2/E3`, `nu12/nu23/nu31`, `G12/G23/G31`; validate positive definiteness or fail early.
- [ ] Implement `compute_psi`, `compute_P`, and `compute_dPdF`.
- [ ] Decide and document whether `R` maps local-to-world or world-to-local; add tests that catch transposed use.
- [ ] Add finite-difference tests:
  - `P` vs finite difference of `psi`
  - `dPdF` vs finite difference of `P`
  - isotropic-equivalent parameters match isotropic StVK within tolerance
  - rotated material frame changes anisotropic response as expected
- [ ] Add assembler-level smoke tests for tet and hex trilinear with `OrthotropicStVK`.
- [ ] Enable Python `pgo.energy.OrthotropicStVK()` only after these tests pass.
- [ ] Move the deferred Python Orthotropic test from Task 4 into this task: `OrthotropicStVK` builds deformation energy and fails clearly on payload/law mismatch or invalid stiffness.

**Exit criteria:**

- Orthotropic has a real solver-side `ElasticModel`, not just payload conversion.
- Python `OrthotropicStVK` builds deformation energy and computes value/gradient/Hessian.
- Payload/law mismatch and invalid stiffness parameters fail with actionable errors.

## Task 9: Prepare Hermite Extension Point Without Implementing Hermite

**目标：** 让 future `HexTricubicHermite` 接入点明确，同时当前行为安全失败。

**Files:**

- Modify: `src/core/solidDeformationModel/formulations/deformationFormulations.h`
- Modify: `src/core/solidDeformationModel/formulations/formulationTraits.h`
- Modify: `src/core/solidDeformationModel/formulations/formulationConcepts.h`
- Modify: `src/core/solidDeformationModel/formulations/formulationVariants.h`
- Modify: `src/core/solidDeformationModel/deformationModelFactory.inl`
- Modify: `src/core/solidDeformationModel/factories/elementModelFactory.cpp`
- Modify: `src/python/pypgo/bindings/energy_bindings.cpp`
- Modify: `pypgo/fem.py`
- Modify: `pypgo/energy.py`
- Modify: `tests/src/core/solidDeformationModel/formulations/deformationModelFormulation_gtest.cpp`
- Modify: `tests/pypgo/test_deformation_energy.py`

- [ ] Add `HexTricubicHermite` tag options and `FormulationTraits<HexTricubicHermite>` specialization.
- [ ] Extend `CubicFormulation` and `CubicFormulationVariant` to include `HexTricubicHermite`; this is intentionally delayed from Task 2 so the unsupported branch is introduced together with its tests.
- [ ] Add C++ implementation guard:
  - `makeCubicDeformationModel(..., HexTricubicHermite{...}, ...)` recognizes the request but throws `std::logic_error("hex_tricubic_hermite is not implemented")`.
- [ ] Add Python `HexTricubicHermite` dataclass with placeholder options:
  - `quadrature_order`
  - future `continuity_policy`
  - future `dof_layout`
- [ ] Python wrapper maps it to C++ only to receive `NotImplementedError` / `RuntimeError` with a stable message.
- [ ] Cross-reference existing Hermite implementation plans:
  - `plan/tricubic_hermit_plastic_field_fem.plan.md`
  - `plan/tricubic_hermite_plastic_field_simulation_integration.plan.md`

**Exit criteria:**

- Users can see the future API name.
- Calling it cannot accidentally fall back to `hex_trilinear`.

## Task 10: Documentation And Examples

**目标：** 更新 Python migration docs，让 deformation energy 位于 M3 energy/solver milestone 里，但 C++ API refactor 前置。

**Files:**

- Modify: `plan/python_api_migration/milestones.md`
- Modify: `plan/python_api_migration/api_coverage.md`
- Modify: `plan/python_api_migration/future_work.md`
- Optional: add example under `pypgo/examples/scripts/`

- [ ] In `milestones.md`, add this plan as the detailed M3 deformation energy subplan.
- [ ] In `api_coverage.md`, mark deformation energy as planned with explicit `tet_p1` / `hex_trilinear` and phase-specific material support rows: ENu in Task 4, Mooney-Rivlin/Hill in Task 7, Orthotropic in Task 8.
- [ ] In `future_work.md`, list full Hermite support as future work dependent on `DofLayout`; do not list Orthotropic as future work after Task 8 lands.
- [ ] Add a small Python example:

  ```python
  energy = pgo.energy.deformation_energy(
      sim_mesh,
      formulation=pgo.fem.HexTrilinear(),
      elastic=pgo.energy.StableNeo(),
      plastic="volumetric_dof6",
  )
  u = energy.zero_state()
  print(energy.value(u))
  ```

**Exit criteria:**

- Migration docs do not describe cubic deformation as generic cubic FEM.
- Python examples consistently use `HexTrilinear()`.
- Material docs distinguish payload (`pypgo.mesh.veg`) from elastic recipe (`pypgo.energy`).

## Verification Commands

Run C++ baseline and deformation tests:

```bash
conda run -n libpgo cmake --preset base
conda run -n libpgo cmake --build --preset base -j 8
conda run -n libpgo ctest --test-dir build/base -R "SimulationMesh|TetP1Kernel|HexTrilinearKernel|DeformationGradientElementModel|DeformationModelFactory|DeformationModelAssembler|DeformationModelFormulation|ElasticModelFactory|OrthotropicStVK|Vertex3DofLayout" --output-on-failure
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
| `SimulationMesh::clone()` misses multi-material element data | Medium | Clone `elementMaterialID` lists and material clones; add shell/material tests |
| Manager factory split changes raw pointer lifetime | High | First extraction keeps existing storage ownership; RAII conversion is separate substep |
| DofLayout migration changes sparse pattern ordering | Medium | Compare dense Hessian values, not only nnz/order; keep `findEntryOffset` tests |
| Payload/law names are confused in Python | High | Keep payload classes in `pypgo.mesh.veg`, recipe classes in `pypgo.energy`; add mismatch tests |
| Hill is treated as a standalone material | High | Model Hill only as `HillFiber(base=..., hill_slot=..., fibers=...)` |
| Orthotropic law has incorrect frame convention | High | Document `R` direction and add rotated-frame tests |
| Mooney-Rivlin Vega payload maps incorrectly to solver coefficients | Medium | Centralize conversion helper and test `.veg` payload vs `SimulationMeshMooneyRivlinMaterial` coefficients |
| New files are scattered back into the module root | Medium | New formulation, factory, material, and DOF abstractions must use the directory layout in design decision 4; root only keeps façade/main-chain/legacy wrapper files |
| Formula implementation leaks into `FormulationTraits` | Medium | Traits only hold type aliases/metadata; basis, quadrature, `F`, and derivatives live in kernels/models with FD tests |
| Tet and cubic refactors diverge into two incompatible element paths | High | Extract `TetP1Kernel` and `HexTrilinearKernel` before traits; both must use `DeformationGradientElementModel<Kernel>` and old-vs-new regression tests |
| `ElementModel<Kernel, ElasticModel, PlasticModel>` causes template explosion | Medium | Keep the mathematical model in docs, but implement first version as `DeformationGradientElementModel<Kernel>` with runtime elastic/plastic injection |
| Formulation API drifts into virtual base-class dispatch | Medium | Core API uses tag + concept/templates; `std::variant` appears only at Python/config boundaries |
| Topology-specific template factories are defined only in `.cpp` | High | Put constrained template definitions in `deformationModelFactory.inl` included by the public header; `.cpp` only holds non-template helpers |
| Python binding duplicates `SimulationMeshCore` or cannot access its owned mesh | High | Move `SimulationMeshCore` into shared `simulation_mesh_core.h` and expose a C++ `cloneMesh()` helper for energy bindings |
| Python Task 4 overclaims material support before payload/recipe migration | High | Task 4 supports only ENu-backed passive laws; Mooney-Rivlin/Hill land in Task 7 and Orthotropic lands in Task 8 |
| `HexTricubicHermite` silently uses trilinear path | High | Explicit implementation guard must throw `not implemented` |
| Shell path breaks because it uses negative local vertex sentinel | Medium | `Vertex3DofLayout` must preserve `vid < 0` zero-local behavior |

## Milestone Exit Criteria

This plan is complete when:

- C++ can build deformation energy with explicit `TetP1{}` and `HexTrilinear{}` formulation tags.
- `TetP1` and `HexTrilinear` both use real kernel types plus `DeformationGradientElementModel<Kernel>`, not legacy full element models hidden behind traits.
- Wrong topology/formulation combinations fail at compile time in core C++.
- Topology-specific factory template definitions are available from headers through `deformationModelFactory.inl`, so tests, tools, C API adapters, and Python bindings can instantiate them without linker surprises.
- Runtime formulation selection is isolated to `std::variant` boundary adapters for Python/config.
- `FormulationTraits` binds only `DofLayout`, `Kernel`, `ElementModel`, and metadata; it does not bind concrete elastic or plastic model types.
- New formulation/factory/material/DOF files follow the `formulations/`, `factories/`, and `materials/` directory split; legacy wrapper files may remain at the module root during migration.
- Old public `makeDeformationModel(...)` auto-dispatch entry has been removed.
- Python can construct tet P1 and cubic hex trilinear deformation energy.
- Python cubic deformation API requires or visibly records `HexTrilinear`.
- Python deformation energy supports ENu, Mooney-Rivlin, Hill fiber composite, and OrthotropicStVK through explicit elastic recipe objects.
- Manager creation logic is split into elastic/plastic/element factories.
- Material payload conversion and elastic law construction are centralized in `SimulationMesh` payload conversion plus `ElasticModelFactory`.
- Assembler uses `DofLayout` for gather/scatter/sparsity on the existing vertex path.
- Unsupported `HexTricubicHermite` is recognized but fails explicitly.
- C++ and Python tests pass with the commands above.
