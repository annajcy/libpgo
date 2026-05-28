# Deformation FEM API Refactor Plan

> **状态日期：** 2026-05-28  
> **适用范围：** C++ `solidDeformationModel` API 重构 + Python `pypgo.fem` / `pypgo.energy` deformation binding。  
> **执行约束：** 本计划只重构当前 tet P1 / hex trilinear / shell Koiter deformation 主链路并绑定到 Python；不在本计划内实现 tricubic Hermite FEM 数值内核。

## 目标

把当前 deformation energy 主链路从“mesh type 隐含 FEM formulation 和 DOF layout”改成显式三层：

```text
MeshTopology / SimulationMesh
  -> ElementFormulation
  -> DofLayout
  -> DeformationModelAssembler
  -> DeformationModelEnergy
```

第一阶段必须保持现有 tet/cubic 行为不变，同时把当前 cubic deformation 路径明确命名为 `hex_trilinear`。Python public API 不在 C++ 边界稳定前提前定稿；等 formulation、lifetime、material recipe、DOF layout 的 C++ 重构完成后，再统一确定 `pypgo.fem` / `pypgo.energy` 的最终表面，避免把过渡态发布成长期 API。

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
  using Basis = HexTrilinearBasis;
  using Quadrature = GaussLegendreHexQuadrature2;
  using Kernel = DeformationGradientKernel<Basis, Quadrature>;
  using ElementModel = DeformationGradientElementModel<Kernel>;

  static constexpr int nodesPerElement = 8;
  static constexpr int localDofs = 24;
  static constexpr std::string_view name = "hex_trilinear";
};
```

Formulation 不强行统一 trait shape——每类 formulation 按实际需要的类型集合声明别名，通过 C++20 concept 做编译期验证。trait 即文档，没有 `NoBasis` / `NoQuadrature` 这类 sentinel：

```cpp
// formulationConcepts.h — per-category concept，保证编译期安全

// Volumetric deformation-gradient formulation: 需要 Basis + Quadrature
template<class F>
concept VolumetricFormulationCategory = requires {
  typename FormulationTraits<F>::Basis;
  typename FormulationTraits<F>::Quadrature;
  typename FormulationTraits<F>::Kernel;
  typename FormulationTraits<F>::ElementModel;
};

// Shell formulation: 需要 ElementStencil，不需要 Basis/Quadrature
template<class F>
concept ShellFormulationCategory = requires {
  typename FormulationTraits<F>::ElementStencil;
  typename FormulationTraits<F>::Kernel;
  typename FormulationTraits<F>::ElementModel;
};
```

运行时边界（Python binding、config）通过 `std::visit` + `if constexpr` 分发：

```cpp
std::visit([&](const auto &f) {
  using F = std::decay_t<decltype(f)>;
  if constexpr (VolumetricFormulationCategory<F>) {
    // 编译期安全访问 Traits::Basis、Traits::Quadrature
  } else if constexpr (ShellFormulationCategory<F>) {
    // 编译期安全访问 Traits::ElementStencil
  }
}, formulationVariant);
```

**为什么有的 formulation 需要 `ElementStencil`，有的不需要？**

`ElementStencil` 捕捉的是**超出 `numNodes` 能表达的那部分 per-element 拓扑信息**。关键差异是节点是否"同质"：

**Volumetric P1（tet P1 / hex trilinear）— 不需要**

所有节点完全同质：都是 vertex node，每个 node 贡献 3 个位移 DOF，全部使用，没有缺失。`Basis` 的 `numNodes` + 标准 `mesh.getVertex(ele, localNode)` 已经足够。没有额外的 per-element 拓扑数据需要编码。

```cpp
// TetP1: 从 Basis + mesh 就能拿到全部信息
//   numNodes = 4, 全部都是 vertex node, 全部存在
//   → 不需要 ElementStencil（Basis + mesh 足够）
for (int j = 0; j < 4; j++) {
  mesh.getVertex(ele, j, restPosition.segment<3>(j * 3).data());
}
```

**Shell Koiter — 需要**

6 个节点但不平等：nodes 0-2 是三角形顶点（始终存在），nodes 3-5 是对边邻居（边界三角形上可能缺失）。额外需要的信息：

```cpp
// ShellKoiterStencil 编码了这些 B→asis + mesh 给不出的信息：
struct ShellKoiterStencil {
  static constexpr int numNodes = 6;
  static constexpr int localDofs = 18;

  // 核心：oppVtx 映射 — 每个三角形边的对边邻居是谁
  // oppVtx[0] = 4 → node 4 是 edge (1,2) 的对边邻居
  // oppVtx[1] = 5 → node 5 是 edge (0,2) 的对边邻居
  // oppVtx[2] = 3 → node 3 是 edge (0,1) 的对边邻居
  static constexpr int oppVtx[3] = { 4, 5, 3 };
};

// 构造 kernel 时还需要 hasVtx[6] — 哪些节点存在
// hasVtx = {1,1,1,  1,0,1}  → node 4 缺失（边界边）
//                                  secondFundamentalFormEntries
//                                  跳过 node 4 的贡献
```

```text
          2                          Node roles:
         /|\                         0,1,2 = triangle vertices (always present)
        / | \                        3 = opp neighbor of edge (0,1)
       /  |  \                       4 = opp neighbor of edge (1,2)
      3   |   5                      5 = opp neighbor of edge (0,2)
       \  |  /
        \ | /                        Boundary case:
         \|/                         hasVtx[4]=false → edge (1,2) has no
          1                          opposite neighbor → skip its contribution
```

**Hex Tricubic Hermite（未来）— 不需要**

最标准的 conforming full hex tricubic Hermite：8 个 corner，每个 corner 固定 8 个 Hermite mode × 3 空间分量 = 192 local DOF。所有 element 完全一样——没有缺 DOF、没有 edge/face enrichment、没有 hanging node、local corner 顺序固定。local DOF 结构是固定的、完整的、同质的、能直接从 `mesh.getVertex(e, corner)` + mode + component 唯一确定。这里真正需要的是 `HermiteDofLayout`，不是 `ElementStencil`：

```cpp
// 标准 conforming full tricubic Hermite:
//   不需要 ElementStencil — local DOF 由 corner + mode + component 唯一确定
//   HermiteDofLayout 负责 vertexDof(v, mode, component) → global DOF
for (int c = 0; c < 8; ++c) {
  int v = mesh.getVertex(e, c);
  for (int m = 0; m < 8; ++m) {       // val, ∂ξ, ∂η, ∂ζ, ∂²ξη, ∂²ηζ, ∂²ξζ, ∂³ξηζ
    for (int a = 0; a < 3; ++a) {
      int gdof = hermiteDofLayout.vertexDof(v, m, a);
    }
  }
}
```

等未来引入 edge/face enrichment、hanging node、extraordinary point、derivative reconstruction、interface discontinuity、variable modes、orientation transform 时，再引入对应的 Hermite stencil。不要因为 Hermite 看起来高级就强行加 stencil。

判断标准：**如果 local DOF = `mesh.getVertex(e, corner)` + 固定 mode + component 就能唯一确定，就不需要 `ElementStencil`。只有当 local DOF 还依赖 neighbor / edge / face / orientation / missing mask / constraint / chart 时才需要。**

**结论：** `ElementStencil` 存在的意义是编码"同质顶点列表之外"的 per-element 拓扑——节点角色分化、节点缺失掩码、DOF 组分映射。Basis 只管插值，不该背不该由它背的拓扑语义。P1 和标准 conforming Hermite 都不需要，因为它们的 local DOF 完全由 `corner vertices + 固定 mode + component` 唯一确定；Shell Koiter 需要，因为每个 element 的 neighbor 可能缺失、角色不对等。

Concrete trait specializations——每个只声明自己实际需要的别名，trait 即文档：

```cpp
// Volumetric: 有 Basis + Quadrature，没有 ElementStencil
template<>
struct FormulationTraits<TetP1>
{
  using DofLayout = Vertex3DofLayout;
  using Basis = TetP1Basis;
  using Quadrature = TetP1DefaultQuadrature;
  using Kernel = DeformationGradientKernel<Basis, Quadrature>;
  using ElementModel = DeformationGradientElementModel<Kernel>;

  static constexpr int nodesPerElement = 4;
  static constexpr int localDofs = 12;
  static constexpr std::string_view name = "tet_p1";
};

template<>
struct FormulationTraits<HexTrilinear>
{
  using DofLayout = Vertex3DofLayout;
  using Basis = HexTrilinearBasis;
  using Quadrature = GaussLegendreHexQuadrature2;
  using Kernel = DeformationGradientKernel<Basis, Quadrature>;
  using ElementModel = DeformationGradientElementModel<Kernel>;

  static constexpr int nodesPerElement = 8;
  static constexpr int localDofs = 24;
  static constexpr std::string_view name = "hex_trilinear";
};

// Shell: 有 ElementStencil，没有 Basis/Quadrature
template<>
struct FormulationTraits<ShellKoiter>
{
  using DofLayout = Vertex3DofLayout;
  using ElementStencil = ShellKoiterStencil;
  using Kernel = FundamentalFormsKernel<ElementStencil>;
  using ElementModel = KoiterShellElementModel<Kernel>;

  static constexpr int nodesPerElement = 6;
  static constexpr int localDofs = 18;
  static constexpr std::string_view name = "shell_koiter";
};
```

为何这比 sentinel 统一 shape 更好：`FormulationTraits<ShellKoiter>` 不声明 `Basis` / `Quadrature`，泛型代码如果错误访问 `Traits::Basis` 会直接编译失败——这正是我们想要的，因为 shell 没有 reference-element 插值。`VolumetricFormulationCategory` concept 用 `requires { typename Traits::Basis; }` 在重载决议时已经筛选掉了 `ShellKoiter`，后续代码不需要再做 `if constexpr (!std::same_as<Basis, NoBasis>)` 的二次检查。新增 formulation 类别（beam、membrane 等）是加法——只加它需要的 alias + 一个 concept，不影响现有 trait shape。

具体数学公式不要放在 traits 里。公式实现放在 `ElementModel` 或独立 kernel 中：

```text
Formulation tag
  -> FormulationTraits
       -> DofLayout + (per-concept aliases) + Kernel + ElementModel
```

`TetP1` 和 `HexTrilinear` 都要先抽出明确 kernel，再写 traits。不要让 `FormulationTraits` 指向旧的 `TetMeshDeformationModel` / `CubicMeshDeformationModel` 作为过渡类型；这会把完整 element model 误命名成 kernel，后续读代码的人会分不清抽象层次。旧类可以临时作为 behavior oracle 或兼容 wrapper，但不能成为新 traits 的目标类型。

`ShellKoiter` 是本 milestone 的 supported shell formulation，但它的数学不进入 volumetric `Basis` / `Quadrature` / `DeformationGradientKernel`。Task 5e 会给 shell 建一套平行的 shell-specific stack：`ShellKoiterStencil -> FundamentalFormsKernel -> KoiterShellElementModel<Kernel>`。它的 traits 只声明 `DofLayout` / `ElementStencil` / `Kernel` / `ElementModel`——不声明 `Basis` / `Quadrature`，因为 shell 不走 reference-domain integral，concept `ShellFormulationCategory` 不要求它们。未来 FE-style shell（DKT、MITC、subdivision、IGA、solid-shell）如果有真正的 reference-element 插值，可以在它们的 traits 里加上 `Basis` / `Quadrature`，concept 自然扩展或 union。

更准确的数学分层是：

```text
Basis
  = reference element field interpolation
  = node/DOF shape, N(xi), dN/dxi

Quadrature
  = integration point policy on the reference element
  = xi_q, w_q

Kernel<Basis, Quadrature>
  = rest-geometry precomputation plus deformation-gradient kinematics
  = dN/dX, F(u), dF/du, detJ * weight

ElementModel<Kernel, ElasticModel, PlasticModel>
  = complete per-element energy process
  = gather local state, call Kernel, call ElasticModel/PlasticModel, accumulate E/grad/H
```

这里不要把 `Basis` 命名成裸 `Field`。`Field` 容易和 plastic field、fiber field、displacement field 混淆；本计划采用 `Basis` / `ReferenceElementBasis` 表达 FEM basis function field。`Quadrature` 是独立策略，这样同一个 basis 后续可以接 full integration、reduced integration、selective reduced integration 或 Hermite higher-order quadrature，而不需要复制 basis 实现。

但第一版工程实现不要把 `ElasticModel` 和 `PlasticModel` 也模板化进 `FormulationTraits`，否则 `HexTrilinear x StableNeo x StVK x MooneyRivlin x Hill x PlasticNone x Plastic6Dof` 会变成组合爆炸，并且 Python/config 的运行时 recipe 很难映射。目标落地形态是：

```cpp
template<class Basis, class Quadrature>
class DeformationGradientKernel;

template<class Kernel>
class DeformationGradientElementModel;

template<>
struct FormulationTraits<HexTrilinear>
{
  using DofLayout = Vertex3DofLayout;
  using Basis = HexTrilinearBasis;
  using Quadrature = GaussLegendreHexQuadrature2;
  using Kernel = DeformationGradientKernel<Basis, Quadrature>;
  using ElementModel = DeformationGradientElementModel<Kernel>;
};
```

`ElementModelFactory` 负责用 `FormulationTraits<Formulation>::ElementModel` 创建元素模型，并把 runtime 构造出的 `ElasticModel *` / `PlasticModel *` 注入进去。也就是说，数学概念上可以理解为 `ElementModel<Kernel, ElasticModel, PlasticModel>`，但 C++ 第一版 API 固定为 `ElementModel<Kernel>` + runtime material/plastic dependency injection。`FormulationTraits` 不能声明具体的 `StableNeo`、`MooneyRivlin`、`HillFiber` 或 `Plastic6Dof` 类型。

第一版必须同步落地两个 deformation-gradient formulation 组合：

```cpp
template<>
struct FormulationTraits<TetP1>
{
  using DofLayout = Vertex3DofLayout;
  using Basis = TetP1Basis;
  using Quadrature = TetP1DefaultQuadrature;
  using Kernel = DeformationGradientKernel<Basis, Quadrature>;
  using ElementModel = DeformationGradientElementModel<Kernel>;

  static constexpr int nodesPerElement = 4;
  static constexpr int localDofs = 12;
  static constexpr std::string_view name = "tet_p1";
};

template<>
struct FormulationTraits<HexTrilinear>
{
  using DofLayout = Vertex3DofLayout;
  using Basis = HexTrilinearBasis;
  using Quadrature = GaussLegendreHexQuadrature2;
  using Kernel = DeformationGradientKernel<Basis, Quadrature>;
  using ElementModel = DeformationGradientElementModel<Kernel>;

  static constexpr int nodesPerElement = 8;
  static constexpr int localDofs = 24;
  static constexpr std::string_view name = "hex_trilinear";
};
```

这样 tet P1 和 hex trilinear 在新架构中是平级 formulation；区别只在 basis/quadrature/local DOF，而不是一个走新抽象、一个留在旧模型。第一版不把 quadrature 做成 Python/API 可配置项，只在 C++ traits 中固定默认组合，避免当前 migration 变成通用 FEM framework 重写。

所有模板类和模板 factory 的实现直接写在对应 header 中；本计划不新增 `.inl` 文件，也不新增 `TetP1Kernel` / `HexTrilinearKernel` 这类兼容 alias。需要具体 kernel 类型时直接写 `DeformationGradientKernel<TetP1Basis, TetP1DefaultQuadrature>` 或 `DeformationGradientKernel<HexTrilinearBasis, GaussLegendreHexQuadrature2>`，避免后续又多一层命名胶水。

### 3. Core API 用 template/concept，运行时边界用 `std::variant`

不要引入 per-formulation 虚基类，也不要让核心 factory 接收基类 `const &`。那会把 formulation dispatch 变成运行时多态，最后需要 `dynamic_cast` 或 virtual 方法，削弱 `FormulationTraits<Formulation>` 的编译期类型安全。

核心 C++ factory 使用 tag object + per-tag concept 约束（按 mesh type 分组的 concept，支持 `||` 扩展）：

```cpp
template<class F>
concept TetFormulation = std::same_as<F, TetP1>;

template<class F>
concept CubicFormulation = std::same_as<F, HexTrilinear>;
// 未来加 Hermite: std::same_as<F, HexTrilinear> || std::same_as<F, HexTricubicHermite>

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

`std::variant` overload 是边界 adapter，不是 core API 的主要形态。Task 2 首先只让 `CubicFormulationVariant` 包含已实现的 `HexTrilinear`，保证 topology-specific factory 可以在当前数值内核上闭环。Task 9 再把 `HexTricubicHermite` 加入 concept 和 variant，并且只接入显式 `not implemented` guard，不能在 Task 2 里提前制造一个没有内核的 formulation 分支。

这些 constrained template overload 的定义必须直接放在 `deformationModelFactory.h` 中；本计划不引入 `.inl` 文件。`.cpp` 只放非模板 helper / implementation detail。不要把模板定义只放进 `.cpp`，否则 tests、tools、Python binding 这些调用方会在独立 translation unit 中链接失败。

### 4. 新文件按 façade / formulation / factory / material 分层

`solidDeformationModel` 根目录只保留 public façade、assembler/manager/energy 主链路，以及旧 element model wrapper。新抽象不要继续散落在根目录里，而是按职责放入子目录：

```text
  src/core/solidDeformationModel/
  simulationMesh.h/.cpp
  deformationModel.h
  deformationModelFactory.h/.cpp
  deformationModelAssembler.h/.cpp
  deformationModelEnergy.h/.cpp
  deformationModelManager.h/.cpp

  tetMeshDeformationModel.h/.cpp       # legacy-compatible wrapper
  cubicMeshDeformationModel.h/.cpp     # legacy-compatible wrapper
  koiterDeformationModel.h/.cpp        # legacy shell oracle until Task 5q deletes it

  formulations/
    deformationFormulations.h
    formulationTraits.h
    formulationConcepts.h
    formulationVariants.h

    kernels/
      deformationGradientKernel.h
      fundamentalFormsKernel.h
      hexTricubicHermiteKernel.h/.cpp  # future

    basis/
      tetP1Basis.h/.cpp
      hexTrilinearBasis.h/.cpp
      hexTricubicHermiteBasis.h/.cpp  # future

    quadrature/
      tetP1DefaultQuadrature.h
      gaussLegendreHexQuadrature.h
      hermiteHexQuadrature.h          # future

    elements/
      deformationGradientElementModel.h  # header-only
      koiterShellElementModel.h

    geometry/
      tetP1Geometry.h                  # Task 5q, after legacy tet helper migration

    stencil/
      shellKoiterStencil.h

    dof/
      dofLayout.h
      vertex3DofLayout.h/.cpp
      hermiteDofLayout.h/.cpp          # future

  factories/
    elasticModelFactory.h/.cpp
    plasticModelFactory.h/.cpp
    elementModelFactory.h          # header-only template factory

  materials/
    elasticModelSpec.h
    simulationMeshMaterialPayload.h
    simulationMeshOrthotropicMaterial.h
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
      deformationGradientKernel_gtest.cpp
      fundamentalFormsKernel_gtest.cpp
    elements/
      deformationGradientElementModel_gtest.cpp
      koiterShellElementModel_gtest.cpp
    dof/
      vertex3DofLayout_gtest.cpp

  factories/
    elasticModelFactory_gtest.cpp
    elementModelFactory_gtest.cpp
```

第一轮不要搬迁所有旧 `elasticModel*` / `plasticModel*` 文件；那会把核心抽象重构变成大规模 include/CMake 搬家。旧 solver law 文件先留在根目录，新 material spec、payload、factory 进入新目录。旧 tet/cubic/shell element wrapper 在 Task 5q 删除；等 Python API 和 formulation 抽象稳定后，再单独做 solver law file relocation。

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

makeTetDeformationModel(const SimulationMesh &mesh, TetP1{}, spec);
makeCubicDeformationModel(const SimulationMesh &mesh, HexTrilinear{}, spec);
makeShellDeformationModel(const SimulationMesh &mesh, ShellKoiter{}, spec);
```

`DeformationModelManager` 内部只保存 `const SimulationMesh *simulationMesh` 作为 non-owning borrow，不再保存 `ownedMesh`。调用方必须保证 mesh owner 活得比 manager / assembler / energy 链更久。

Python binding 的 owner 形态是：

```text
SimulationMeshCore
  owns std::unique_ptr<SimulationMesh>

DeformationEnergyCore
  keeps std::shared_ptr<SimulationMeshCore> alive
  owns DeformationModelBundle / DeformationModelEnergy

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
  const SimulationMesh &mesh,
  const F &formulation,
  const DeformationModelSpec &spec);

template<CubicFormulation F>
DeformationModelBundle makeCubicDeformationModel(
  const SimulationMesh &mesh,
  const F &formulation,
  const DeformationModelSpec &spec);

template<ShellFormulation F>
DeformationModelBundle makeShellDeformationModel(
  const SimulationMesh &mesh,
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

## Python API 定稿策略

下面的 Python 形状只是 C++ 重构完成后的目标草案，不是 Task 4 需要发布的 public API。`pypgo.fem` / `pypgo.energy` 必须等以下 C++ 边界稳定后再统一定稿：

- manager borrow-only lifetime 和 Python owner bridge；
- `ElementModelFactory` 接管 formulation-driven element construction；
- `DofLayout` 接管 gather/scatter/sparsity；
- material payload / elastic recipe / active term 分层完成。

预期 public module：

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
pgo.energy.KoiterStVK()
pgo.energy.HillFiber(
    base=pgo.energy.StableNeo(material_slot=0),
    hill_slot=1,
    element_fibers=fibers,
)
```

最终定稿时应满足的规则：

- Tet mesh 可显式传 `TetP1()`；如果 formulation omitted，可默认 `TetP1()`。
- Cubic mesh 必须显式传 `HexTrilinear()`；不传时抛 `ValueError`，提示当前 cubic topology 有多个未来 formulation。
- Shell mesh 必须显式传 `ShellKoiter()`，并使用现有 `SimulationMesh.create_shell(...)` 产生的 shell payload；shell 走 Task 5e 的 `ShellKoiterStencil -> FundamentalFormsKernel -> KoiterShellElementModel` pipeline，不进入 volumetric `Basis` / `Quadrature` / `DeformationGradientKernel` 抽象。
- Public Python API 不按 Task 4 / Task 7 / Task 8 分批承诺材料支持；等 C++ material recipe migration 完成后，一次性决定 first public release 支持哪些 recipes。
- `MooneyRivlin + MooneyRivlin law`、`base law + HillFiber`、`Orthotropic + OrthotropicStVK law` 只有在对应 C++ payload conversion、law factory、mismatch tests 完成后才能进入 public Python API。
- `HexTricubicHermite()` 是否作为 future-facing Python dataclass 暴露，由最终 API checkpoint 决定；如果暴露，`deformation_energy(..., HexTricubicHermite())` 必须抛 `NotImplementedError`，不能静默落到 trilinear。
- Hill 必须要求 `HillActivation` payload 和 fiber field；不能把 Hill 当作无方向的 passive law。

## Topology/Formulation Matrix

| Topology | Formulation | DofLayout | 状态 |
|---|---|---|---|
| `TetMesh` / `SimulationMeshType::TET` | `TetP1` | `Vertex3DofLayout` | 抽成 `DeformationGradientKernel<TetP1Basis, TetP1DefaultQuadrature>` + `DeformationGradientElementModel<Kernel>`，行为必须严格保持 |
| `CubicMesh` / `SimulationMeshType::CUBIC` | `HexTrilinear` | `Vertex3DofLayout` | 抽成 `DeformationGradientKernel<HexTrilinearBasis, GaussLegendreHexQuadrature2>` + `DeformationGradientElementModel<Kernel>`，当前 cubic 行为必须显式命名并严格保持 |
| `CubicMesh` / `SimulationMeshType::CUBIC` | `HexTricubicHermite` | `HermiteDofLayout` | future，当前抛 `not implemented` |
| shell `SimulationMeshType::SHELL` | `ShellKoiter` | `Vertex3DofLayout` with invalid local sentinel | Task 5e 抽成 `ShellKoiterStencil` + `FundamentalFormsKernel` + `KoiterShellElementModel<Kernel>`；不并入 volumetric basis/quadrature/kernel 抽象 |
| `SimulationMeshType::TRIANGLE` / `EDGE_QUAD` | none | none | 不在本计划 deformation energy 范围内 |

## Material Recipe Matrix

| Payload slot type | Elastic recipe | Solver model | 状态 |
|---|---|---|---|
| `SimulationMeshENuMaterial` | `StableNeo` | `ElasticModelStableNeoHookeanMaterial` | 当前行为，必须保持 |
| `SimulationMeshENuMaterial` | `Linear` | `ElasticModelLinearMaterial` | 当前行为，通过新 recipe 表达 |
| `SimulationMeshENuMaterial` | `StVK` | `ElasticModel3DSTVKMaterial` or invariant StVK path | 当前行为，通过新 recipe 表达 |
| `SimulationMeshMooneyRivlinMaterial` | `MooneyRivlin` | `ElasticModel3DMooneyRivlin` | C++ mapping/test 先落地，Task 10 决定是否进入 first public Python API |
| `SimulationMeshOrthotropicMaterial` | `OrthotropicStVK` | new `ElasticModel3DOrthotropicStVK` | 本计划补齐 C++ law 后，Task 10 决定是否进入 first public Python API |
| `SimulationMeshENuMaterial` + `SimulationMeshHillMaterial` + fibers | `HillFiber(base=StableNeo/StVK/StVKVol)` | `ElasticModelCombinedMaterial` | C++ composite recipe 先落地，Task 10 决定 public Python 表面 |
| `SimulationMeshENuhMaterial` shell payload | `KoiterStVK` | `ElasticModel2DFundamentalFormsSTVK` + `KoiterShellElementModel<FundamentalFormsKernel<...>>` | 当前 shell 行为通过 Task 5e 新 shell element stack 保持，Task 10 决定 public Python 表面 |

不支持的组合必须 fail fast。例如 `StableNeo` 不能读取 `MooneyRivlin` payload，`HillFiber` 不能缺少 fiber direction，`OrthotropicStVK` 不能在 `ElasticModel3DOrthotropicStVK` 测试通过前开放。

## File Map

### 新增

- `src/core/solidDeformationModel/formulations/deformationFormulations.h`
- `src/core/solidDeformationModel/formulations/formulationTraits.h`
- `src/core/solidDeformationModel/formulations/formulationConcepts.h`
- `src/core/solidDeformationModel/formulations/formulationVariants.h`
- `src/core/solidDeformationModel/formulations/basis/tetP1Basis.h`
- `src/core/solidDeformationModel/formulations/basis/tetP1Basis.cpp`
- `src/core/solidDeformationModel/formulations/basis/hexTrilinearBasis.h`
- `src/core/solidDeformationModel/formulations/basis/hexTrilinearBasis.cpp`
- `src/core/solidDeformationModel/formulations/quadrature/tetP1DefaultQuadrature.h`
- `src/core/solidDeformationModel/formulations/quadrature/gaussLegendreHexQuadrature.h`
- `src/core/solidDeformationModel/formulations/kernels/deformationGradientKernel.h`
- `src/core/solidDeformationModel/formulations/kernels/fundamentalFormsKernel.h`
- `src/core/solidDeformationModel/formulations/elements/deformationGradientElementModel.h`
- `src/core/solidDeformationModel/formulations/elements/koiterShellElementModel.h`
- `src/core/solidDeformationModel/formulations/stencil/shellKoiterStencil.h`
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
- `src/core/solidDeformationModel/elasticModel3DOrthotropicStVK.h`
- `src/core/solidDeformationModel/elasticModel3DOrthotropicStVK.cpp`
- `src/python/pypgo/bindings/simulation_mesh_core.h`
- `src/python/pypgo/bindings/energy_bindings.cpp`
- `pypgo/fem.py`
- `pypgo/energy.py`
- `tests/pypgo/test_deformation_energy.py`
- `tests/src/core/solidDeformationModel/formulations/deformationModelFormulation_gtest.cpp`
- `tests/src/core/solidDeformationModel/formulations/basis/tetP1Basis_gtest.cpp`
- `tests/src/core/solidDeformationModel/formulations/basis/hexTrilinearBasis_gtest.cpp`
- `tests/src/core/solidDeformationModel/formulations/quadrature/tetP1DefaultQuadrature_gtest.cpp`
- `tests/src/core/solidDeformationModel/formulations/quadrature/gaussLegendreHexQuadrature_gtest.cpp`
- `tests/src/core/solidDeformationModel/formulations/kernels/deformationGradientKernel_gtest.cpp`
- `tests/src/core/solidDeformationModel/formulations/kernels/fundamentalFormsKernel_gtest.cpp`
- `tests/src/core/solidDeformationModel/formulations/elements/deformationGradientElementModel_gtest.cpp`
- `tests/src/core/solidDeformationModel/formulations/elements/koiterShellElementModel_gtest.cpp`
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

- [x] Add C++ baseline test: current tet `makeTetDeformationModel(..., TetP1{}, ...)` at zero displacement has finite near-zero energy and finite gradient.
- [x] Add C++ baseline test: current cubic `makeCubicDeformationModel(..., HexTrilinear{}, ...)` at zero displacement has finite near-zero energy and finite gradient.
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
- No Task 0 test may reference `TetP1`, `HexTrilinear`, `makeTetDeformationModel`, `makeCubicDeformationModel`, or `ElasticModelSpec`; those symbols are introduced later and must not be prerequisites for baseline characterization.

## Task 1: Extract Basis, Quadrature, Kernels, And Shared Element Model

**目标：** 先把 tet P1 和 hex trilinear 的 formulation 数学从旧 element model 中抽出来，并进一步拆成 `Basis`、`Quadrature`、`DeformationGradientKernel<Basis, Quadrature>`，再让二者共同走 `DeformationGradientElementModel<Kernel>`。这一步必须先于 `FormulationTraits` 和 topology-specific factory，否则 traits 会被迫引用旧 model 作为过渡胶水。

**Files:**

- Create: `src/core/solidDeformationModel/formulations/basis/tetP1Basis.h`
- Create: `src/core/solidDeformationModel/formulations/basis/tetP1Basis.cpp`
- Create: `src/core/solidDeformationModel/formulations/basis/hexTrilinearBasis.h`
- Create: `src/core/solidDeformationModel/formulations/basis/hexTrilinearBasis.cpp`
- Create: `src/core/solidDeformationModel/formulations/quadrature/tetP1DefaultQuadrature.h`
- Create: `src/core/solidDeformationModel/formulations/quadrature/gaussLegendreHexQuadrature.h`
- Create: `src/core/solidDeformationModel/formulations/kernels/deformationGradientKernel.h`
- Create: `src/core/solidDeformationModel/formulations/elements/deformationGradientElementModel.h`
- Create: `src/core/solidDeformationModel/formulations/elements/deformationGradientElementModel.cpp`
- Modify: `src/core/solidDeformationModel/tetMeshDeformationModel.h`
- Modify: `src/core/solidDeformationModel/tetMeshDeformationModel.cpp`
- Modify: `src/core/solidDeformationModel/cubicMeshDeformationModel.h`
- Modify: `src/core/solidDeformationModel/cubicMeshDeformationModel.cpp`
- Modify: `src/core/solidDeformationModel/CMakeLists.txt`
- Create: `tests/src/core/solidDeformationModel/formulations/basis/tetP1Basis_gtest.cpp`
- Create: `tests/src/core/solidDeformationModel/formulations/basis/hexTrilinearBasis_gtest.cpp`
- Create: `tests/src/core/solidDeformationModel/formulations/quadrature/tetP1DefaultQuadrature_gtest.cpp`
- Create: `tests/src/core/solidDeformationModel/formulations/quadrature/gaussLegendreHexQuadrature_gtest.cpp`
- Create: `tests/src/core/solidDeformationModel/formulations/kernels/deformationGradientKernel_gtest.cpp`
- Create: `tests/src/core/solidDeformationModel/formulations/elements/deformationGradientElementModel_gtest.cpp`
- Modify: `tests/src/core/solidDeformationModel/CMakeLists.txt`

- [x] Add `TetP1Basis` and `HexTrilinearBasis` for reference-element interpolation:
  - node/local DOF shape metadata;
  - `N(xi)`;
  - `dN/dxi`.
- [x] Add `TetP1DefaultQuadrature` and `GaussLegendreHexQuadrature2`:
  - quadrature points in reference coordinates;
  - reference weights;
  - no material, mesh, or rest-geometry state.
- [x] Add `DeformationGradientKernel<Basis, Quadrature>` with rest-shape precomputation currently embedded in `TetMeshDeformationModel` / `CubicMeshDeformationModel`.
- [x] Do not add `TetP1Kernel` / `HexTrilinearKernel` compatibility aliases. Use the concrete template instantiations directly:
  - `DeformationGradientKernel<TetP1Basis, TetP1DefaultQuadrature>`
  - `DeformationGradientKernel<HexTrilinearBasis, GaussLegendreHexQuadrature2>`
- [x] Keep kernels free of `ElasticModel`, `PlasticModel`, SVD, stress, material parameters, and energy accumulation.
- [x] Do not expose quadrature choice in Python or public factory options in this milestone; quadrature is selected through `FormulationTraits`.
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
- [x] Add `DeformationGradientElementModel<Kernel>` that implements the existing `DeformationModel` virtual interface by combining:
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
- [x] Turn `TetMeshDeformationModel` into a compatibility wrapper around `DeformationGradientElementModel<DeformationGradientKernel<TetP1Basis, TetP1DefaultQuadrature>>` after regression tests pass.
- [x] Turn `CubicMeshDeformationModel` into a compatibility wrapper around `DeformationGradientElementModel<DeformationGradientKernel<HexTrilinearBasis, GaussLegendreHexQuadrature2>>` after regression tests pass.
- [x] Keep the full `examples/ipc/cubic/box/box-ipc.json` run as final smoke only; do not use it as the first regression guard.

**Exit criteria:**

- `TetP1Basis`, `HexTrilinearBasis`, default quadrature, and `DeformationGradientKernel<Basis, Quadrature>` unit tests pass.
- New shared element model matches old tet and cubic element outputs within tight numerical tolerance.
- Old public element-model class names can still be used internally during migration, but their implementations delegate to the new shared element model.
- `FormulationTraits<TetP1>` and `FormulationTraits<HexTrilinear>` can point to real `Basis`, `Quadrature`, and `Kernel` types without referencing old full element models.

## Task 2: Split Deformation Factory By Topology

**目标：** 移除旧的 runtime auto-dispatch `makeDeformationModel(...)`，改为 `makeTetDeformationModel` / `makeCubicDeformationModel` / `makeShellDeformationModel`。Formulation 用 tag object 表达，topology 由函数签名表达，让错误组合在函数签名层面不可表达。本任务只拆 topology/formulation 入口，material 输入可以继续临时使用 legacy `DeformationModelElasticMaterial`；Task 7 再切到 `ElasticModelSpec`。

**Current status (2026-05-28):** Complete for topology/formulation factory split. The remaining direct manager/assembler call-site consolidation and construction log naming cleanup are moved to Task 5, where `ElementModelFactory` owns the relevant construction policy.

**Files:**

- Create: `src/core/solidDeformationModel/formulations/deformationFormulations.h`
- Create: `src/core/solidDeformationModel/formulations/formulationTraits.h`
- Create: `src/core/solidDeformationModel/formulations/formulationConcepts.h`
- Create: `src/core/solidDeformationModel/formulations/formulationVariants.h`
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

- [x] Add formulation tag objects for currently implemented paths: `TetP1`, `HexTrilinear`, and `ShellKoiter`.
- [x] Do not add `HexTricubicHermite` in Task 2; it is a Task 9 placeholder so the not-implemented branch is introduced in one place.
- [x] Add `FormulationTraits<Formulation>` specializations for each tag.
- [x] Make each volumetric traits specialization declare `DofLayout`, `Basis`, `Quadrature`, `Kernel`, and `ElementModel`; do not declare concrete elastic or plastic types in traits. `DofLayout` may be a forward-declared `Vertex3DofLayout` here; the assembler migration to actually use it happens in Task 6.
- [x] Bind `FormulationTraits<TetP1>` to `TetP1Basis`, `TetP1DefaultQuadrature`, `DeformationGradientKernel<Basis, Quadrature>`, and `DeformationGradientElementModel<Kernel>`.
- [x] Bind `FormulationTraits<HexTrilinear>` to `HexTrilinearBasis`, `GaussLegendreHexQuadrature2`, `DeformationGradientKernel<Basis, Quadrature>`, and `DeformationGradientElementModel<Kernel>`.
- [x] Bind `FormulationTraits<ShellKoiter>` to existing shell-specific model construction metadata; shell does not declare volumetric `Basis` or `Quadrature` and must keep using the current `KoiterDeformationModel` path.
- [x] Do not point traits at legacy full element models such as `TetMeshDeformationModel` or `CubicMeshDeformationModel`.
- [x] Add `TetFormulation`, `CubicFormulation`, `ShellFormulation` per-tag concepts（按 mesh type 分组，支持 `||` 扩展）；constrain 对应 factory 模板。Do not add virtual formulation base classes.
- [x] Add runtime boundary variants:
  - `using TetFormulationVariant = std::variant<TetP1>;`
  - `using CubicFormulationVariant = std::variant<HexTrilinear>;`
  - `using ShellFormulationVariant = std::variant<ShellKoiter>;`
- [x] Add variant adapter overloads only for binding/config code paths; core code should call the constrained templates directly.
- [x] Put constrained template definitions directly in `deformationModelFactory.h`; keep only non-template helpers in `deformationModelFactory.cpp`.
- [x] Remove public `makeDeformationModel(...)` overloads instead of wrapping them.
- [x] Move remaining direct manager/assembler call-site consolidation to Task 5, including:
  - `src/tools/sim/runIPCSim/setup/femSetup.cpp`
  - `src/c/pgo_c.cpp`
  - `tests/src/tools/runSimShared_gtest.cpp`
- [x] Keep Task 2 factory-level formulation validation; do not force `DeformationModelManager` to become formulation-aware in this task. Manager-side element construction is migrated in Task 5 via `ElementModelFactory`.
- [x] Add formulation validation inside each topology-specific factory:
  - `TetP1` works only with tet topology.
  - `HexTrilinear` works with cubic topology.
  - `ShellKoiter` works only with shell topology.
- [x] Add factory bundle initialization snapshots for both `plasticParams` and `elasticParams`; volumetric deformation-gradient plastic initializes identity `Fp`, while shell Koiter uses zero/default shell plastic params and maps `SimulationMeshENuhMaterial` to the existing five STVK shell elastic params.
- [x] Move log/error wording cleanup that currently says only `CUBIC` to Task 5, where formulation-aware construction has access to `FormulationTraits<Formulation>::name`.
- [x] Add C++ tests for all currently implemented topology/formulation rows and for absence of the old public auto-dispatch entry. The Hermite row is tested in Task 9 when its placeholder is introduced.
- [x] Add compile-time tests or `static_assert`s that wrong topology/formulation combinations are not invocable.

**Exit criteria:**

- `makeCubicDeformationModel(..., HexTrilinear{}, ...)` works.
- `makeTetDeformationModel(..., TetP1{}, ...)` works.
- `makeShellDeformationModel(..., ShellKoiter{}, ...)` works through the existing Koiter shell path.
- Old `makeDeformationModel(...)` public entry is gone from the header.
- Wrong topology/formulation combinations fail at compile time for core template calls.
- Task 2 may still carry legacy elastic enum inputs, but no public call site may still depend on runtime mesh-type auto-dispatch.

## Task 3: Make `DeformationModelManager` Borrow `const SimulationMesh &`

**目标：** Python `SimulationMesh` 可以被多个 energy factory 调用复用；mesh ownership 保留在外层 root，manager / assembler / energy 链只借用 immutable `SimulationMesh`，不消费 unique ownership。

**Current status (2026-05-28):** Complete. C++ core borrow-only semantics have been applied. `DeformationModelManager` now takes `const SimulationMesh &`, `DeformationModelManagerImpl::ownedMesh` has been removed, topology-specific C++ factories take `const SimulationMesh &`, and `runIPCSim` / C API call sites keep explicit mesh owners outside the energy chain. `SimulationMeshCore` has been moved to a shared binding header and exposes `mesh() const` for future energy bindings. Energy-specific Python lifetime tests are deferred with the public/private energy binding work in Task 4 / Task 10.

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
- [x] Defer the `DeformationEnergyCore` `std::shared_ptr<SimulationMeshCore>` hold to Task 4 private smoke hook or Task 10 public energy binding; Task 3 provides the shared owner type needed by that binding.
- [x] Do not expose C++ mesh ownership or lifetime controls as public Python methods.
- [x] Add C++ tests: one `SimulationMesh` owner can be used to construct two independent deformation energies, and both remain evaluable while the owner is alive.
- [x] Defer Python energy lifetime tests to Task 4 private smoke hook or Task 10 public energy API, because public `pypgo.energy` is intentionally not committed in Task 3.

**Exit criteria:**

- `SimulationMeshCore` owns the C++ mesh and exposes a borrow-only `mesh() const` accessor for future energy bindings.
- C++ can create two independent energy chains from the same `SimulationMesh` owner while the owner remains alive.
- Manager / assembler / energy construction no longer requires moving a `std::unique_ptr<SimulationMesh>` into `DeformationModelManager`.

## Task 4: Defer Public Python Deformation API Until C++ Boundaries Stabilize

**目标：** 不在 C++ formulation/lifetime/material/DOF 边界仍处于过渡态时发布 `pypgo.fem` / `pypgo.energy` public API。Task 4 只允许做 Python lifetime bridge 的技术准备和 private `_core` smoke hook；最终 public Python API 在 Task 10 统一定稿。

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

- [x] Record that public `pypgo.fem` / `pypgo.energy` API is deferred until after Tasks 5, 6, 7, and 8.
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
- Public Python API decisions are deferred to Task 10 after C++ formulation, material, and DOF boundaries are stable.

## Task 5: Split `DeformationModelManager::initImpl` Into Internal Factories

**目标：** 降低 manager 的职责复杂度，为 future formulation 增量接入准备边界。这一步必须 behavior-preserving：先把现有 legacy elastic/plastic enum 的分支搬进内部 factory，不同时引入 material recipe 语义迁移。

`DeformationModelManager` 在 Task 5 结束时仍可保留为 legacy model-storage/query object；不要在同一任务中做 rename/remove。真正的边界变化是：creation policy 进入 `ElasticModelFactory` / `PlasticModelFactory` / `ElementModelFactory`，manager 不再直接承载所有 dispatch 细节。Assembler 仍通过 manager 查询 element model，直到 Task 6 把 DOF gather/scatter/sparsity 迁入 `DofLayout`。

**Files:**

- Create: `src/core/solidDeformationModel/factories/elasticModelFactory.h`
- Create: `src/core/solidDeformationModel/factories/elasticModelFactory.cpp`
- Create: `src/core/solidDeformationModel/factories/plasticModelFactory.h`
- Create: `src/core/solidDeformationModel/factories/plasticModelFactory.cpp`
- Create: `src/core/solidDeformationModel/factories/elementModelFactory.h`
- Modify: `src/core/solidDeformationModel/formulations/elements/deformationGradientElementModel.h`
- Modify: `src/core/solidDeformationModel/formulations/elements/deformationGradientElementModel.cpp`
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
- [x] Use the existing `DeformationGradientElementModel<Kernel>` wrapper from Task 1 for both `DeformationGradientKernel<TetP1Basis, TetP1DefaultQuadrature>` and `DeformationGradientKernel<HexTrilinearBasis, GaussLegendreHexQuadrature2>`.
- [x] Make `ElementModelFactory` construct `DeformationGradientElementModel<Kernel>` with runtime `ElasticModel *` and `PlasticModel *` dependencies supplied by the factories.
- [x] Do not instantiate `ElementModel<Kernel, StableNeo, Plastic6Dof>`-style combinations in the first version; avoid material/plastic template explosion.
- [x] Keep `ElasticModelFactory` consuming legacy `DeformationModelElasticMaterial` in Task 5 so this extraction can be tested independently of material recipe migration.
- [x] Do not remove `DeformationModelElasticMaterial` from the factory call path in Task 5; Task 7 performs that public API migration after payload conversion and mismatch tests exist.
- [x] Pass the formulation tag object or its `FormulationTraits` type into `ElementModelFactory`.
- [x] Move default `plasticParams` / `elasticParams` snapshot initialization out of the topology factory helper and into focused creation helpers near `PlasticModelFactory` / `ElasticModelFactory`; preserve Task 2 behavior exactly:
  - volumetric deformation-gradient plastic params encode identity `Fp`;
  - shell Koiter plastic params stay at their zero/default stretch state;
  - shell STVK elastic params map `SimulationMeshENuhMaterial` to `(E, nu, E_bend, nu_bend, h)`.
- [x] Update construction logs/errors to include formulation names from `FormulationTraits<Formulation>::name`, e.g. `hex_trilinear`, instead of only topology names such as `CUBIC`.
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
- `FormulationTraits` selects the formulation kernel and generic element-model wrapper; material/plastic selection remains runtime-injected by the factories.

## Task 5e: Add Shell Koiter Formulation Stack

**目标：** 把 shell Koiter 也接入 formulation-driven element construction，但不把它硬塞进 volumetric `Basis` / `Quadrature` / `DeformationGradientKernel`。Shell 需要一套平行抽象：compile-time stencil topology + per-element fundamental-forms kernel + Koiter shell element model。Task 5e 完成后，`ShellKoiter` 不再只是 factory routing tag，而是拥有自己的 shell kernel/model stack，可在 Task 5p 中替代 `KoiterDeformationModel` runtime path。

### Design decision

Volumetric deformation-gradient stack（参考点）：

```text
Basis                              # compile-time: N(xi), dN/dxi
  -> Quadrature                    # compile-time: xi_q, w_q
  -> DeformationGradientKernel     # per-element: restX, weightDetJ(q), Fref(x)
  -> DeformationGradientElementModel<Kernel>
```

Shell Koiter stack 在 Task 5e 后的最终形态：

```text
ShellKoiterStencil                 # compile-time topology: 6 nodes, 18 local DOFs, oppVtx mapping
  -> FundamentalFormsKernel<ElementStencil>
                                   # per-element: restX[6], hasVtx[6], restI, restII, restArea
                                   # + a/b 微分几何 (extracted from KoiterDeformationModelInternal)
  -> KoiterShellElementModel<Kernel>
                                   # holds Kernel + ElasticModel2DFundamentalForms* + PlasticModel2DFundamentalForms*
```

### Per-category concept，不统一 alias shape

每条 formulation 只声明自己实际需要的类型别名，通过 C++20 concept 做编译期验证（详见设计决策 2 的 `VolumetricFormulationCategory` / `ShellFormulationCategory`）。Task 5e 后三套 trait 的形状是：

```cpp
// Volumetric: 有 Basis + Quadrature
template<> struct FormulationTraits<TetP1> {
  using DofLayout = Vertex3DofLayout;
  using Basis = TetP1Basis;
  using Quadrature = TetP1DefaultQuadrature;
  using Kernel = DeformationGradientKernel<Basis, Quadrature>;
  using ElementModel = DeformationGradientElementModel<Kernel>;
  // ...
};

template<> struct FormulationTraits<HexTrilinear> {
  using DofLayout = Vertex3DofLayout;
  using Basis = HexTrilinearBasis;
  using Quadrature = GaussLegendreHexQuadrature2;
  using Kernel = DeformationGradientKernel<Basis, Quadrature>;
  using ElementModel = DeformationGradientElementModel<Kernel>;
  // ...
};

// Shell Koiter: 有 ElementStencil，没有 Basis/Quadrature
template<> struct FormulationTraits<ShellKoiter> {
  using DofLayout = Vertex3DofLayout;
  using ElementStencil = ShellKoiterStencil;
  using Kernel = FundamentalFormsKernel<ElementStencil>;
  using ElementModel = KoiterShellElementModel<Kernel>;
  // ...
};
```

不引入 `NoBasis` / `NoQuadrature` / `NoElementStencil`。泛型代码按 formulation category 用 `if constexpr` + concept 分发，每个分支在编译期就知道自己能用哪些 alias。

### Concrete responsibility split

| 数据 / 方法 | 来源 (今天) | 去向 (Task 5e 后) |
|---|---|---|
| `oppVtx[3] = {4, 5, 3}` | `KoiterDeformationModelInternal` (instance field) | `ShellKoiterStencil::oppVtx` (`static constexpr`) |
| numNodes = 6, localDofs = 18 | hard-coded in `KoiterDeformationModel::getNumVertices/DOFs` | `ShellKoiterStencil::numNodes` / `::localDofs` (`static constexpr`) |
| `restX[6]`, `hasVtx[6]`, `restI`/`restII` | `KoiterDeformationModelInternal` | `FundamentalFormsKernel<ElementStencil>` 构造时计算并保存 |
| rest `area`（喂给 plastic via `setArea`） | `KoiterDeformationModel` ctor | `FundamentalFormsKernel::restArea()` |
| `compute_a_and_derivatives` | `KoiterDeformationModelInternal` 方法 | `FundamentalFormsKernel::compute_a_and_derivatives` |
| `compute_b_and_derivatives` | 同上 | `FundamentalFormsKernel::compute_b_and_derivatives` |
| `secondFundamentalFormEntries` | 同上 | `FundamentalFormsKernel::secondFundamentalFormEntries` (private helper) |
| `faceNormal` | 同上 | `FundamentalFormsKernel::faceNormal` (private helper) |
| `KoiterDeformationModelCacheData` (x[6], a/abar/b/bbar/area, elasticParams, plasticParams) | `KoiterDeformationModel` | `KoiterShellElementModelCacheData<Kernel>` |
| `prepareData` / `computeEnergy` / `compute_dE_dx` / `compute_d2E_dx2` / `compute_d2E_dxda` / `compute_d2E_dxdb` / `computeLocalMaxStepSize` / `enableSPD` | `KoiterDeformationModel` overrides | `KoiterShellElementModel<Kernel>` overrides |
| `set_abar` / `set_bbar` / `setArea` on plastic model | `KoiterDeformationModel` ctor side effect | `KoiterShellElementModel<Kernel>` ctor side effect (移植) |
| `-10496` sentinel detection | `KoiterDeformationModel` ctor + factory + `DeformationModelManager` populates fake positions | `ElementModelFactory::create<ShellKoiter>` builds explicit `bool hasVtx[6]` from `mesh.getVertexIndex(ele, j) < 0`；不再有 sentinel |

### Target API sketch

```cpp
// formulations/stencil/shellKoiterStencil.h
struct ShellKoiterStencil
{
  static constexpr int numNodes = 6;
  static constexpr int localDofs = 18;
  static constexpr int numTriangleNodes = 3;
  static constexpr int oppVtx[3] = { 4, 5, 3 };
};

// formulations/kernels/fundamentalFormsKernel.h
template<class ElementStencil>
class FundamentalFormsKernel
{
public:
  static constexpr int numNodes = ElementStencil::numNodes;
  static constexpr int localDofs = ElementStencil::localDofs;

  // restX[18] holds positions for nodes 0..5 in slots [0..3), [3..6), ...
  // hasVtx[6] is true for nodes 0..2; nodes 3..5 may be missing.
  // Missing slots' restX entries are not read; caller may leave them uninitialized.
  FundamentalFormsKernel(const double restX[18], const bool hasVtx[6]);

  // First fundamental form (uses nodes 0..2 only).
  ES::M2d compute_a_and_derivatives(
    const ES::V3d x[3],
    Eigen::Matrix<double, 4, 9> *da_dx,
    ES::M9d ahess[4]) const;

  // Second fundamental form (uses all 6 nodes, with mask).
  ES::M2d compute_b_and_derivatives(
    const ES::V3d x[6],
    Eigen::Matrix<double, 4, 18> *db_dx,
    ES::M18d bhess[4]) const;

  const bool *hasVtx() const { return hasVtx_; }
  const ES::M2d &restI() const { return restI_; }
  const ES::M2d &restII() const { return restII_; }
  double restArea() const { return restArea_; }

private:
  ES::V3d secondFundamentalFormEntries(const ES::V3d x[6],
    Eigen::Matrix<double, 3, 18> *derivative, ES::M18d hessian[3]) const;
  ES::V3d faceNormal(const ES::V3d x0, const ES::V3d x1, const ES::V3d x2,
    Eigen::Matrix<double, 3, 9> *derivative, ES::M9d hessian[3]) const;
  static ES::M3d crossMatrix(const Eigen::Vector3d &v);

  ES::V3d restX_[6];
  bool hasVtx_[6];
  ES::M2d restI_, restII_;
  double restArea_;
};

// formulations/elements/koiterShellElementModel.h
template<class Kernel>
class KoiterShellElementModel : public DeformationModel
{
public:
  static constexpr int numNodes = Kernel::numNodes;        // 6
  static constexpr int localDofs = Kernel::localDofs;      // 18

  KoiterShellElementModel(const double restX[18], const bool hasVtx[6],
    ElasticModel *elasticModel, PlasticModel *plasticModel);

  // DeformationModel virtuals (mirrors KoiterDeformationModel exactly).
  DeformationModelCacheData *allocateCacheData() const override;
  void freeCacheData(DeformationModelCacheData *data) const override;
  void prepareData(const double *x, const double *param,
    const double *materialParam, DeformationModelCacheData *cd) const override;
  double computeEnergy(const DeformationModelCacheData *cd) const override;
  void compute_dE_dx(const DeformationModelCacheData *cd, double *grad) const override;
  void compute_d2E_dx2(const DeformationModelCacheData *cd, double *hess) const override;
  void compute_d2E_dxda(const DeformationModelCacheData *cd, double *hess) const override;
  void compute_d2E_dxdb(const DeformationModelCacheData *cd, double *hess) const override;
  void enableSPD(int enable) override;
  int getNumVertices() const override { return numNodes; }
  int getNumDOFs() const override { return localDofs; }
  LocalMaxStepResult computeLocalMaxStepSize(const double *, const double *) const override;

private:
  Kernel kernel_;
  ElasticModel2DFundamentalForms *elasticModel_ = nullptr;
  PlasticModel2DFundamentalForms *plasticModel_ = nullptr;
  int enableSPD_ = 0;
};
```

### Files

- Create: `src/core/solidDeformationModel/formulations/stencil/shellKoiterStencil.h`（header-only，仅 `static constexpr` 拓扑常量）
- Create: `src/core/solidDeformationModel/formulations/kernels/fundamentalFormsKernel.h`（header-only template，从 `koiterDeformationModel.cpp` 搬出微分几何）
- Create: `src/core/solidDeformationModel/formulations/elements/koiterShellElementModel.h`（header-only template，与 `DeformationGradientElementModel<Kernel>` 同形）
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

### Sub-task A: ShellKoiterStencil

- [ ] Create `formulations/stencil/shellKoiterStencil.h` with `numNodes = 6`, `localDofs = 18`, `numTriangleNodes = 3`, `oppVtx[3] = {4, 5, 3}` 全部 `static constexpr`。
- [ ] No runtime state, no member functions; this is the compile-time topology constant set，analogous to `TetP1Basis::numNodes` style metadata.
- [ ] Add a trivial smoke test asserting these constants match `KoiterDeformationModel::getNumVertices() == 6` and `getNumDOFs() == 18`（可与 kernel/element model 测试合并）。

### Sub-task B: FundamentalFormsKernel

- [ ] Create `formulations/kernels/fundamentalFormsKernel.h` as a header-only template `FundamentalFormsKernel<ElementStencil>`.
- [ ] Move these methods verbatim from `koiterDeformationModel.cpp` into the kernel:
  - `compute_a_and_derivatives(const ES::V3d x[3], Eigen::Matrix<double, 4, 9>*, ES::M9d ahess[4])`
  - `compute_b_and_derivatives(const ES::V3d x[6], Eigen::Matrix<double, 4, 18>*, ES::M18d bhess[4])` — drop the `int hasVtx[6]` parameter; the kernel reads `hasVtx_` from its own member.
  - `secondFundamentalFormEntries(const ES::V3d x[6], Eigen::Matrix<double, 3, 18>*, ES::M18d hessian[3])` — same hasVtx change.
  - `faceNormal(const ES::V3d, const ES::V3d, const ES::V3d, Eigen::Matrix<double, 3, 9>*, ES::M9d[3])`
  - `crossMatrix(...)`
- [ ] Constructor `FundamentalFormsKernel(const double restX[18], const bool hasVtx[6])` copies positions and mask into members, then computes and caches:
  - `restI_ = compute_a_and_derivatives(restX_ as V3d[3], nullptr, nullptr)`
  - `restII_ = compute_b_and_derivatives(restX_ as V3d[6], nullptr, nullptr)`
  - `restArea_ = 0.5 * (restX_[1] - restX_[0]).cross(restX_[2] - restX_[0]).norm()`
- [ ] Document the contract: missing-neighbor slots in `restX_` are not read; `hasVtx_[i] == false` for `i in [3,5]` makes the kernel skip that opposite-normal contribution exactly as the legacy code does.
- [ ] Kernel does not depend on `ElasticModel` / `PlasticModel` / `DeformationModelCacheData`; this matches the volumetric `Kernel` invariant.
- [ ] Kernel unit tests in `fundamentalFormsKernel_gtest.cpp`:
  - **Rest state**: `restI == compute_a(restX_as_V3d[3])` and `restII == compute_b(restX_as_V3d[6])` (round-trip identity).
  - **Translation invariance**: translating all 6 positions by a constant vector leaves `a`, `b`, derivatives unchanged.
  - **Affine map**: applying a known 3x3 linear map to all positions produces `a` consistent with `a_legacy(A*x)` from `KoiterDeformationModelInternal::compute_a_and_derivatives`.
  - **Missing neighbor**: with `hasVtx[3] = false`, the resulting `b` matches the legacy code's masked output exactly.
  - **Derivative FD**: `da/dx` and `db/dx` match finite differences of `a` and `b` to `1e-6`.
  - **Hessian FD**: `d2a/dx2` and `d2b/dx2` match finite differences of `da/dx` and `db/dx`.
- [ ] All tests run on a small fixture: one curved 3-triangle patch with a known geometry, plus one boundary triangle (only nodes 3..4 missing).

### Sub-task C: KoiterShellElementModel

- [ ] Create `formulations/elements/koiterShellElementModel.h` as a header-only template `KoiterShellElementModel<Kernel>`.
- [ ] Cache data `KoiterShellElementModelCacheData<Kernel>` mirrors `KoiterDeformationModelCacheData` 1:1:
  - `ES::V3d x[6]`
  - `ES::M2d a, abar, b, bbar`
  - `ES::V18d elasticParams, plasticParams`
  - `double area`
  - `ElasticModel2DFundamentalForms *elasticModel`、`PlasticModel2DFundamentalForms *plasticModel`（与 legacy 一致地缓存指针）
- [ ] Constructor copies positions + mask into the kernel; then `dynamic_cast` elastic/plastic to `ElasticModel2DFundamentalForms*` / `PlasticModel2DFundamentalForms*` and throw on null（与 `DeformationGradientElementModel` 风格一致）。
- [ ] Constructor must preserve the legacy plastic-seed side effect:
  - `plasticModel_->set_abar(kernel_.restI())`
  - `plasticModel_->set_bbar(kernel_.restII())`
  - `plasticModel_->setArea(kernel_.restArea())`
- [ ] Implement the 5 virtual methods + `enableSPD` + `computeLocalMaxStepSize` by copying body from `KoiterDeformationModel`, swapping `ind->compute_a_and_derivatives(...)` calls for `kernel_.compute_a_and_derivatives(...)` and dropping the explicit `hasVtx` parameter (now owned by kernel).
- [ ] `computeLocalMaxStepSize` returns the same default `LocalMaxStepResult{}` (shell has no local max-step rule today).
- [ ] Element model does not implement `vonMisesStress` / `maxStrain` — `KoiterDeformationModel` does not either; keep the default base behavior.

### Sub-task D: FormulationTraits + ElementModelFactory wiring

- [ ] Update `FormulationTraits<ShellKoiter>` aliases（不声明 `Basis` / `Quadrature`——shell 不走 volumetric reference-domain integral，由 `ShellFormulationCategory` 验证）:
  - `using DofLayout = Vertex3DofLayout;`
  - `using ElementStencil = ShellKoiterStencil;`
  - `using Kernel = FundamentalFormsKernel<ElementStencil>;`
  - `using ElementModel = KoiterShellElementModel<Kernel>;`
  - `static constexpr int nodesPerElement = ElementStencil::numNodes;`
  - `static constexpr int localDofs = ElementStencil::localDofs;`
  - keep `name = "shell_koiter"`.
- [ ] `FormulationTraits<TetP1>` and `FormulationTraits<HexTrilinear>` remain unchanged（already have `Basis` / `Quadrature` / `Kernel` / `ElementModel`；no `ElementStencil` alias needed or wanted）.
- [ ] Add `VolumetricFormulationCategory` and `ShellFormulationCategory` concepts to `formulationConcepts.h`（per 设计决策 2）.
- [ ] Remove the comment `// ShellKoiter — routes to existing KoiterDeformationModel path` from `formulationTraits.h` and replace it with one describing the new shell stack and per-category concept design.
- [ ] Update `ElementModelFactory::create<ShellKoiter>` to:
  - build `ES::V18d restX` and `bool hasVtx[6]` from `mesh.getVertexIndex(ele, j) < 0`（不再读 `-10496` sentinel）；
  - construct `new typename FormulationTraits<ShellKoiter>::ElementModel(restX.data(), hasVtx, elasticModel, plasticModel)`；
  - keep the existing `KOITER_FABRIC` / `KOITER_STVK` guard, throwing on unsupported elastic materials.
- [ ] Drop `#include "../koiterDeformationModel.h"` from `elementModelFactory.h`.
- [ ] `KoiterDeformationModel` 仍在 `koiterDeformationModel.h/.cpp`，但 production factory 不再实例化它。

### Sub-task E: Element-model parity tests

- [ ] Create `tests/.../formulations/elements/koiterShellElementModel_gtest.cpp` with two fixtures:
  - **Interior triangle**: hasVtx = {1,1,1,1,1,1}, three different rest configurations (flat, gently curved, sharply curved).
  - **Boundary triangle**: at least one of hasVtx[3..5] = 0 (cover all three missing-edge cases via parameterized test).
- [ ] For each fixture, construct both `KoiterDeformationModel` (oracle) and `KoiterShellElementModel<FundamentalFormsKernel<ShellKoiterStencil>>` (new) wired to the same `ElasticModel2DFundamentalFormsSTVK` + `PlasticModel2DFundamentalForms` instances. Plastic-seed side effect happens on whichever constructor runs first; reset the plastic model between cases or instantiate independent plastic models per case.
- [ ] At rest displacement and at a small perturbed displacement, compare to `1e-10` absolute tolerance:
  - `computeEnergy`
  - `compute_dE_dx` (18-vector)
  - `compute_d2E_dx2` (18×18, both with and without `enableSPD(1)`)
  - `compute_d2E_dxda` (18 × num_plastic_params)
  - `compute_d2E_dxdb` (18 × num_elastic_params)
- [ ] Add SPD enable test: after `enableSPD(1)`, `compute_d2E_dx2` symmetric PSD eigenvalues match between old and new.
- [ ] Add FD sanity (separate from oracle comparison): `compute_dE_dx` matches finite difference of `computeEnergy` to `1e-5`.

### Sub-task F: Manager / factory wiring for the mask

- [ ] Currently `deformationModelManager.cpp:502` fills `restPosition[k] = (-10496, -10496, -10496)` and `elementModelFactory.h:79` repeats the same fill. After Sub-task D the factory builds `hasVtx` from the mesh directly. Remove the manager-side fill so missing-neighbor encoding lives in exactly one place (the factory).
- [ ] If the manager still needs to provide a per-element rest position buffer to legacy callers during the transition (e.g. before Task 5p deletes the buffer), keep the buffer but stop encoding sentinel values into it; leave missing slots as `0`. This is safe because the new element model never reads them and the legacy `KoiterDeformationModel` is no longer constructed through this path.
- [ ] Search for remaining `-10496` references after this sub-task:
  - `koiterDeformationModel.cpp` — keep (legacy oracle still uses sentinel).
  - `deformationModelManager.cpp` — must be gone.
  - `elementModelFactory.h` — must be gone.

### Sub-task G: Factory + assembler smoke tests

- [ ] Extend `factories/elementModelFactory_gtest.cpp`:
  - assert `ElementModelFactory::create<ShellKoiter>(...)` returns a `KoiterShellElementModel<...>*` for both interior and boundary elements (use `dynamic_cast` to verify type，删除 legacy 时再拆);
  - assert factory still throws for non-Koiter elastic material types.
- [ ] Extend `deformationModelFactory_gtest.cpp` shell case:
  - run `makeShellDeformationModel(mesh, ShellKoiter{}, spec)` end-to-end，
  - confirm bundle's per-element model is the new type，
  - confirm energy at zero displacement matches existing baseline within tolerance.
- [ ] Do not delete the legacy oracle test path in Task 5e; Task 5q is responsible for removing oracle dependencies entirely.

### Out of scope for Task 5e

- [ ] Do not redesign shell DOF layout; `Vertex3DofLayout` migration happens in Task 6. Shell `vid < 0` slots continue to flow through the existing assembler gather/scatter until Task 6.
- [ ] Do not migrate shell materials to `ElasticModelSpec`. `KOITER_STVK` / `KOITER_FABRIC` legacy enum remains the elastic material input to `ElementModelFactory::create<ShellKoiter>` until Task 7/Task 10 finalizes the shell recipe surface.
- [ ] Do not delete `koiterDeformationModel.h/.cpp` here; deletion + parity-test rewriting belongs to Task 5q.
- [ ] Do not change `ElasticModel2DFundamentalForms` / `PlasticModel2DFundamentalForms` interfaces.

**Exit criteria:**

- `ShellKoiterStencil`, `FundamentalFormsKernel<ElementStencil>`, and `KoiterShellElementModel<Kernel>` exist as header-only templates under the `formulations/` subtree, matching the file layout in design decision 4.
- `FormulationTraits<ShellKoiter>` declares `DofLayout` / `ElementStencil` / `Kernel` / `ElementModel` and no longer routes through `KoiterDeformationModel`. It does NOT declare `Basis` or `Quadrature`（shell doesn't use reference-domain integral；`ShellFormulationCategory` doesn't require them）. `FormulationTraits<TetP1>` and `FormulationTraits<HexTrilinear>` declare `Basis` / `Quadrature` / `Kernel` / `ElementModel` without `ElementStencil`. No sentinel/placeholder types are introduced.
- `ElementModelFactory::create<ShellKoiter>` returns `KoiterShellElementModel<...>` for every supported shell element, both interior and boundary.
- Kernel unit tests cover rest state, translation invariance, affine map, missing-neighbor mask, and FD checks on first/second-derivative outputs.
- Parity tests vs `KoiterDeformationModel` pass to `1e-10` on energy, gradient, Hessian (SPD on/off), `d2E/dxda`, and `d2E/dxdb` for both interior and boundary triangles.
- `-10496` no longer appears in `deformationModelManager.cpp` or `elementModelFactory.h`; only `koiterDeformationModel.cpp` retains it as the still-living oracle.
- All pre-existing shell factory/assembler smoke tests pass through the new path.
- `KoiterDeformationModel` source files remain in tree as an oracle for parity tests and a deletion target for Task 5q.

## Task 5p: Task 5 Closeout Cleanup

**目标：** 去掉 Task 5 留下的过渡胶水，让 production runtime path 不再先构造 legacy tet/cubic/shell element wrapper 再替换成 formulation-aware element model。本任务不直接删除 `TetMeshDeformationModel` / `CubicMeshDeformationModel` / `KoiterDeformationModel` 文件；先只移除 manager/factory runtime path 对它们的依赖。真正删除 legacy wrappers 在 Task 5q 完成。

**Dependencies:**

- Task 5e 已完成，`ShellKoiter` 已有 shell-specific `Kernel` / `ElementModel`，不再需要 `KoiterDeformationModel` 作为 runtime implementation。

**Current issue (2026-05-28):** `makeTetDeformationModel<TetP1>` / `makeCubicDeformationModel<HexTrilinear>` 已经通过 `FormulationTraits -> Basis + Quadrature + Kernel + DeformationGradientElementModel` 构造最终 element model，Task 5e 后 `ShellKoiter` 也有自己的 formulation-aware shell element model。但实现路径仍然可能是：

```text
make*DeformationModel<Formulation>
  -> DeformationModelManager::initImpl(...)
       -> first creates legacy element wrapper
  -> factory template loop
       -> replaces each element with ElementModelFactory::create<Formulation>(...)
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
- Modify: `tests/src/core/solidDeformationModel/deformationModelFormulation_gtest.cpp`
- Optional modify: `tests/src/core/solidDeformationModel/deformationModelAssembler_gtest.cpp`

- [ ] Make `DeformationModelManager::initImpl` create element FEMs through `ElementModelFactory` directly:
  - `SimulationMeshType::TET` -> `ElementModelFactory::create<TetP1>(...)`;
  - `SimulationMeshType::CUBIC` -> `ElementModelFactory::create<HexTrilinear>(...)`;
  - `SimulationMeshType::SHELL` -> `ElementModelFactory::create<ShellKoiter>(...)`.
- [ ] Remove direct `new TetMeshDeformationModel(...)`, `new CubicMeshDeformationModel(...)`, and `new KoiterDeformationModel(...)` from `DeformationModelManager::initImpl`.
- [ ] Remove `#include "tetMeshDeformationModel.h"`, `#include "cubicMeshDeformationModel.h"`, and `#include "koiterDeformationModel.h"` from `deformationModelManager.cpp` if no longer needed there.
- [ ] Remove the formulation-aware factory replacement loop in `detail::makeDeformationModelBundle<Formulation>`:
  - do not call `manager->getDeformationModel(ele)->getElasticModel()` just to recover dependencies;
  - do not create a temporary legacy element and then replace it.
- [ ] Remove `DeformationModelManager::setDeformationModel(...)` if it has no remaining production caller after the replacement loop is gone.
- [ ] Keep the non-template `detail::makeDeformationModelBundle(...)` only if tests still need a legacy parity oracle; otherwise delete it or make it a test-only helper. It must not be the production path for public topology-specific factories.
- [ ] Decide whether `DeformationModelManager` should take an internal formulation dispatch parameter in this closeout or stay topology-defaulted:
  - acceptable for Task 5p: manager maps current supported topology defaults to `TetP1`, `HexTrilinear`, `ShellKoiter`;
  - do not introduce public runtime formulation enums or virtual formulation base classes.
- [ ] Add/adjust tests proving public factories no longer depend on legacy wrappers:
  - `makeTetDeformationModel(..., TetP1{}, ...)` returns a model chain whose element model is not `TetMeshDeformationModel`;
  - `makeCubicDeformationModel(..., HexTrilinear{}, ...)` returns a model chain whose element model is not `CubicMeshDeformationModel`;
  - `makeShellDeformationModel(..., ShellKoiter{}, ...)` returns a model chain whose element model is not `KoiterDeformationModel`.
- [ ] Preserve existing parity tests against legacy wrappers until replacement confidence is high; those tests may continue to instantiate `TetMeshDeformationModel` / `CubicMeshDeformationModel` / `KoiterDeformationModel` as oracle objects.
- [ ] Do not delete `tetMeshDeformationModel.h/.cpp`, `cubicMeshDeformationModel.h/.cpp`, or `koiterDeformationModel.h/.cpp` in Task 5p. They are still used by:
  - oracle/parity tests;
  - `TetMeshDeformationModel::computeDs`, `computeDm`, `compute_dF_dx`, and related geometry helpers;
  - older constraint/FD/shell parity utilities that have not been migrated to formulation geometry helpers.
- [ ] Optionally replace manager-owned raw pointer vectors with `std::unique_ptr` vectors after the runtime legacy dependency is gone. Keep this as a separate substep in the same task and do not mix it with behavior changes.

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
- `DeformationGradientElementModel<TetP1>` 和 `DeformationGradientElementModel<HexTrilinear>` 已通过 parity tests 锁住行为。
- `KoiterShellElementModel<ShellKoiter>` 已通过 parity tests 锁住行为。

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
- Modify: `tests/src/core/solidDeformationModel/formulations/elements/deformationGradientElementModel_gtest.cpp`
- Modify: `tests/src/core/solidDeformationModel/formulations/elements/koiterShellElementModel_gtest.cpp`
- Delete or rewrite: `tests/src/core/solidDeformationModel/cubicMeshDeformationModel_gtest.cpp`
- Modify: `tests/src/tools/runSimShared_gtest.cpp`
- Modify: `src/core/solidDeformationModel/CMakeLists.txt`
- Modify: `tests/src/core/solidDeformationModel/CMakeLists.txt`

- [ ] Move tet static geometry helpers out of `TetMeshDeformationModel` into `formulations/geometry/tetP1Geometry.h`:
  - `computeDs(...)`;
  - `computeDm(...)`;
  - `compute_dF_dx(...)`;
  - `computeVolume(...)`;
  - any small helper that constraints/FD tests still use directly.
- [ ] Update all non-test callers of `TetMeshDeformationModel::compute*` to use the new geometry helper namespace/type.
- [ ] Replace tests that dynamic-cast manager elements to `TetMeshDeformationModel` / `CubicMeshDeformationModel` with checks against formulation-aware behavior:
  - use `DeformationGradientElementModel<FormulationTraits<TetP1>::Kernel>`;
  - use `DeformationGradientElementModel<FormulationTraits<HexTrilinear>::Kernel>`;
  - prefer public energy/assembler behavior checks over concrete class casts where possible.
- [ ] Rewrite `deformationGradientElementModel_gtest.cpp` so legacy wrappers are not the long-term oracle:
  - keep numerical golden values or kernel-level expected values where practical;
  - compare against hand-built `DeformationGradientElementModel` instances rather than old wrappers;
  - if temporary oracle coverage is still needed, move it to a short-lived compatibility test and remove it before deleting files.
- [ ] Delete `cubicMeshDeformationModel_gtest.cpp` or rewrite it as `hexTrilinearElementModel_gtest.cpp`.
- [ ] Rewrite shell parity tests so `KoiterDeformationModel` is no longer needed as an oracle; use golden values or direct kernel/model checks from Task 5e.
- [ ] Remove `tetMeshDeformationModel.*`, `cubicMeshDeformationModel.*`, and `koiterDeformationModel.*` from `src/core/solidDeformationModel/CMakeLists.txt`.
- [ ] Remove old wrapper headers from public/header install lists.
- [ ] Run a repository-wide search for `TetMeshDeformationModel`, `CubicMeshDeformationModel`, and `KoiterDeformationModel`; after this task, no production or test file may reference those names.

**Exit criteria:**

- `rg "TetMeshDeformationModel|CubicMeshDeformationModel|KoiterDeformationModel" src tests` returns no references except possibly historical plan text.
- The solid deformation model library builds without compiling `tetMeshDeformationModel.cpp`, `cubicMeshDeformationModel.cpp`, or `koiterDeformationModel.cpp`.
- Tet/cubic/shell energy, gradient, Hessian, stress, max-step, FD, and constraint tests pass through formulation-aware implementations.
- Public C++ headers no longer expose legacy tet/cubic/shell element wrapper classes.

## Task 6: Introduce `DofLayout` With `Vertex3DofLayout`

**目标：** 移除 assembler 对 `3 * numVertices` 和 `numElementVertices * 3` 的硬编码依赖，同时保持现有路径行为不变。Task 6 只迁移 DOF 侧的 gather/scatter/sparsity，不重写 shell 的几何邻居语义；shell 的 missing-neighbor / `-10496` sentinel 由 Task 5e 的 `ShellKoiterStencil` / `FundamentalFormsKernel` 负责，`Vertex3DofLayout` 只处理 `vid < 0` 时把对应 local DOF slot 当作零这一 DOF 边界行为。

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

- [ ] Complete the `DofLayout` abstract interface that Task 2 introduced as a traits-visible declaration.
- [ ] Complete `Vertex3DofLayout` so it borrows the same `const SimulationMesh *` that `DeformationModelManager` borrows from the outer owner.
- [ ] Move `gatherLocalPositions(...)` logic from assembler helper into `Vertex3DofLayout::gather`; preserve `vid < 0` zero-local behavior as a DOF-side concern so missing-neighbor shell slots stay zero in the gathered local vector.
- [ ] Move gradient scatter logic into `Vertex3DofLayout::scatterAddGradient`; `vid < 0` slots must skip global write-back.
- [ ] Move Hessian sparsity construction into `Vertex3DofLayout::addHessianSparsity`; `vid < 0` slots must not generate global triplets.
- [ ] Move local-to-global sparse index lookup into layout helper.
- [ ] Do not move shell missing-neighbor geometry into `Vertex3DofLayout`. The `ShellKoiterStencil` / `FundamentalFormsKernel` introduced in Task 5e owns which neighbor slots are missing and how missing neighbors enter the fundamental-form computation; `Vertex3DofLayout` only sees the resulting per-DOF `vid` array.
- [ ] Change `DeformationModelAssembler` constructor to own `std::unique_ptr<const DofLayout>`. `DofLayout` is an assembly concern, not a manager concern; keep the manager responsible for element/material/plastic model ownership and keep gather/scatter/sparsity policy in the assembler.
- [ ] Store `deformationModelManager` before `dofLayout` in `DeformationModelAssembler` so `dofLayout` is destroyed first; both manager and layout borrow the same outer-owned immutable `SimulationMesh`.
- [ ] Update all assembler construction call sites to pass an explicit `Vertex3DofLayout`; do not keep an implicit compatibility constructor.
- [ ] Change assembler fields:
  - `n3` -> `numDOFs`
  - `localDOFs` becomes per-element query or cached from layout
  - `nvtx` only remains if needed for legacy diagnostics
- [ ] Add tests comparing old expected DOF counts:
  - tet one element: 12
  - cubic one element: 24
  - shell one triangle: `3 * num_surface_vertices` global DOFs and 18 local DOFs
- [ ] Add energy/gradient/Hessian parity tests for tet, cubic, and shell after layout migration.

**Exit criteria:**

- Assembler no longer directly computes global DOF indices from vertex ids.
- Existing vertex DOF path remains numerically identical.
- Shell `vid < 0` DOF-side sentinel behavior remains identical and is owned by `Vertex3DofLayout`; shell missing-neighbor geometry stays owned by `ShellKoiterStencil` / `FundamentalFormsKernel` from Task 5e.
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
- Modify: `tests/src/core/solidDeformationModel/simulationMesh_gtest.cpp`
- Create: `tests/src/core/solidDeformationModel/factories/elasticModelFactory_gtest.cpp`

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
- [ ] Define Mooney-Rivlin payload mapping with a dedicated conversion helper:
  - input is Vega `mu01/mu10/v1`;
  - output is `SimulationMeshMooneyRivlinMaterial(N, M, Cpq, D)`;
  - document the exact `N`, `M`, `Cpq`, and `D` layout in the helper header;
  - add `.veg` payload round-trip/parity tests that lock the mapping before enabling any public Python `MooneyRivlin()` recipe in Task 10.
- [ ] Add Hill as an active term:
  - require base passive payload at slot 0;
  - require `SimulationMeshHillMaterial` at `hill_slot`;
  - require element or vertex fiber directions;
  - produce `ElasticModelCombinedMaterial` internally.
- [ ] Draft final Python elastic recipe notes, but do not expose public wrappers until Task 10:
  - `StableNeo`
  - `StVK`
  - `MooneyRivlin`
  - `OrthotropicStVK`
  - `HillFiber(base=..., hill_slot=..., element_fibers=...)`
- [ ] Add C++ tests for payload/law mismatch errors.
- [ ] Record final Python test cases for Task 10:
  - Mooney-Rivlin energy builds when the mesh payload is Mooney-Rivlin and fails clearly when payload/law mismatch.
  - Hill fiber energy builds when base payload, hill payload, and fiber directions are present.
- [ ] Add C++ tests that `ElasticModelFactory` builds ENu, Mooney-Rivlin, and Hill composite models from explicit specs.

**Exit criteria:**

- `SimulationMesh` can carry ENu, Mooney-Rivlin, Orthotropic, and Hill payloads without losing type information.
- `ElasticModelFactory` is the only place that maps payload + recipe to solver `ElasticModel`.
- Mooney-Rivlin and Hill composite deformation energies are buildable through C++ factories and covered by smoke/FD tests.
- The final Python recipe/test cases are documented for Task 10, but no public Python material API is exposed in Task 7.

## Task 8: Add Orthotropic Solver Elastic Law

**目标：** 把 Orthotropic 从 `.veg`/payload 支持推进到 solver-ready deformation law，使 final Python API 可以安全暴露 `OrthotropicStVK()`，而不是只读写材料参数。

**Files:**

- Create: `src/core/solidDeformationModel/elasticModel3DOrthotropicStVK.h`
- Create: `src/core/solidDeformationModel/elasticModel3DOrthotropicStVK.cpp`
- Modify: `src/core/solidDeformationModel/factories/elasticModelFactory.cpp`
- Modify: `src/core/solidDeformationModel/CMakeLists.txt`
- Create: `tests/src/core/solidDeformationModel/elasticModel3DOrthotropicStVK_gtest.cpp`
- Modify: `tests/src/core/solidDeformationModel/factories/elasticModelFactory_gtest.cpp`

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
- [ ] Mark `OrthotropicStVK()` as eligible for the final Python API checkpoint only after these C++ tests pass.
- [ ] Record final Python test cases for Task 10: `OrthotropicStVK` builds deformation energy and fails clearly on payload/law mismatch or invalid stiffness.

**Exit criteria:**

- Orthotropic has a real solver-side `ElasticModel`, not just payload conversion.
- C++ `OrthotropicStVK` builds deformation energy and computes value/gradient/Hessian.
- Payload/law mismatch and invalid stiffness parameters fail with actionable errors.

## Task 9: Prepare Hermite Extension Point Without Implementing Hermite

**目标：** 让 future `HexTricubicHermite` 接入点明确，同时当前行为安全失败。

**Files:**

- Modify: `src/core/solidDeformationModel/formulations/deformationFormulations.h`
- Modify: `src/core/solidDeformationModel/formulations/formulationTraits.h`
- Modify: `src/core/solidDeformationModel/formulations/formulationConcepts.h`
- Modify: `src/core/solidDeformationModel/formulations/formulationVariants.h`
- Modify: `src/core/solidDeformationModel/deformationModelFactory.h`
- Modify: `src/core/solidDeformationModel/factories/elementModelFactory.h`
- Modify: `tests/src/core/solidDeformationModel/formulations/deformationModelFormulation_gtest.cpp`

- [ ] Add `HexTricubicHermite` tag options and `FormulationTraits<HexTricubicHermite>` specialization.
- [ ] Extend `CubicFormulation` concept to `std::same_as<F, HexTrilinear> \|\| std::same_as<F, HexTricubicHermite>` and `CubicFormulationVariant` to include `HexTricubicHermite`；this is intentionally delayed from Task 2 so the unsupported branch is introduced together with its tests.
- [ ] Add C++ implementation guard:
  - `makeCubicDeformationModel(..., HexTricubicHermite{...}, ...)` recognizes the request but throws `std::logic_error("hex_tricubic_hermite is not implemented")`.
- [ ] Record final Python `HexTricubicHermite` dataclass options for Task 10, but do not expose the public dataclass in Task 9:
  - `quadrature_order`
  - future `continuity_policy`
  - future `dof_layout`
- [ ] Record final Python wrapper behavior for Task 10: mapping it to C++ must receive `NotImplementedError` / `RuntimeError` with a stable message.
- [ ] Cross-reference existing Hermite implementation plans:
  - `plan/tricubic_hermit_plastic_field_fem.plan.md`
  - `plan/tricubic_hermite_plastic_field_simulation_integration.plan.md`

**Exit criteria:**

- C++ recognizes the future API name.
- C++ calls cannot accidentally fall back to `hex_trilinear`.
- Public Python exposure remains deferred to Task 10.

## Task 10: Finalize Python Deformation API, Documentation, And Examples

**目标：** 在 C++ formulation、lifetime、material recipe、DofLayout 边界稳定后，统一确定并绑定 `pypgo.fem` / `pypgo.energy` public deformation API，同时更新 migration docs 和 examples。不要把 Task 4/7/8/9 的过渡接口直接发布成 public API。

**Files:**

- Create/modify: `src/python/pypgo/bindings/energy_bindings.cpp`
- Modify: `src/python/pypgo/bindings/module.cpp`
- Modify: `src/python/pypgo/CMakeLists.txt`
- Create: `pypgo/fem.py`
- Create: `pypgo/energy.py`
- Modify: `pypgo/__init__.py`
- Modify: `pypgo/sim.py`
- Create: `tests/pypgo/test_deformation_energy.py`
- Modify: `plan/python_api_migration/milestones.md`
- Modify: `plan/python_api_migration/api_coverage.md`
- Modify: `plan/python_api_migration/future_work.md`
- Optional: add example under `pypgo/examples/scripts/`

- [ ] In `milestones.md`, add this plan as the detailed M3 deformation energy subplan.
- [ ] Finalize the first public `pypgo.fem` / `pypgo.energy` API shape after reviewing completed C++ Tasks 3, 5, 5e, 5p, 5q, 6, 7, and 8. Tasks 5e/5p/5q decide whether shell construction goes through `KoiterShellElementModel<FundamentalFormsKernel<...>>` instead of the legacy `KoiterDeformationModel`, which directly affects how `ShellKoiter()` binding and documentation describe the shell pipeline.
- [ ] Add `pypgo.fem` formulation dataclasses:
  - `TetP1`
  - `HexTrilinear`
  - `ShellKoiter`
  - optional `HexTricubicHermite` future-facing placeholder only if the final API checkpoint accepts exposing unsupported future formulations.
- [ ] Add `pypgo.energy` recipe/dataclass wrappers only for C++-supported recipes:
  - `StableNeo`
  - `StVK`
  - `MooneyRivlin`
  - `OrthotropicStVK`
  - `KoiterStVK`
  - `HillFiber(base=..., hill_slot=..., element_fibers=...)`
- [ ] Add `pypgo.energy.deformation_energy(...)` wrapper that maps Python dataclasses to `_core` variants/specs.
- [ ] Enforce final Python policy:
  - cubic requires explicit `HexTrilinear()`;
  - shell requires explicit `ShellKoiter()`;
  - payload/law mismatch raises clear `ValueError`;
  - if `HexTricubicHermite()` is exposed, it raises a clear not-implemented error and cannot fall back to trilinear.
- [ ] Add Python tests:
  - tet energy builds and `zero_state()` has correct shape;
  - cubic `HexTrilinear()` energy builds and has `num_dofs == 3 * num_vertices`;
  - shell `ShellKoiter()` energy builds from `SimulationMesh.create_shell(...)`, uses `KoiterStVK()`, and has `num_dofs == 3 * num_vertices`;
  - `value`, `gradient`, `hessian.to_coo()` smoke tests pass at zero state and a small perturbation;
  - same `SimulationMesh` can create two independent energies;
  - deleting the Python `sim_mesh` variable does not invalidate existing energies;
  - Mooney-Rivlin, Hill, and Orthotropic recipe tests from Tasks 7 and 8 pass.
- [ ] In `api_coverage.md`, mark deformation energy as supported only for the final Task 10 public API surface; do not document Task 4 private smoke hooks as public API.
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
- Public Python deformation API is finalized only after the C++ refactor tasks are complete enough to support it cleanly.
- Python examples consistently use `HexTrilinear()`.
- Material docs distinguish payload (`pypgo.mesh.veg`) from elastic recipe (`pypgo.energy`).

## Verification Commands

Run C++ baseline and deformation tests:

```bash
conda run -n libpgo cmake --preset base
conda run -n libpgo cmake --build --preset base -j 8
conda run -n libpgo ctest --test-dir build/base -R "SimulationMesh|TetP1Basis|HexTrilinearBasis|Quadrature|DeformationGradientKernel|DeformationGradientElementModel|FundamentalFormsKernel|KoiterShellElementModel|ShellKoiter|DeformationModelFactory|DeformationModelAssembler|DeformationModelFormulation|ElasticModelFactory|OrthotropicStVK|Vertex3DofLayout" --output-on-failure
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
| Payload/law names are confused in Python | High | Keep payload classes in `pypgo.mesh.veg`, recipe classes in `pypgo.energy`; add mismatch tests |
| Hill is treated as a standalone material | High | Model Hill only as `HillFiber(base=..., hill_slot=..., fibers=...)` |
| Orthotropic law has incorrect frame convention | High | Document `R` direction and add rotated-frame tests |
| Mooney-Rivlin Vega payload maps incorrectly to solver coefficients | Medium | Centralize conversion helper and test `.veg` payload vs `SimulationMeshMooneyRivlinMaterial` coefficients |
| New files are scattered back into the module root | Medium | New formulation, factory, material, and DOF abstractions must use the directory layout in design decision 4; root only keeps façade/main-chain/legacy wrapper files |
| Formula implementation leaks into `FormulationTraits` | Medium | Traits only hold type aliases/metadata; basis, quadrature, `F`, and derivatives live in kernels/models with FD tests |
| Basis and quadrature are coupled inside formulation-specific kernels | Medium | Split reference-element interpolation into `Basis` and integration strategy into `Quadrature`; keep `Kernel<Basis, Quadrature>` responsible for rest geometry and deformation-gradient kinematics |
| Tet and cubic refactors diverge into two incompatible element paths | High | Extract `TetP1Basis` / `HexTrilinearBasis` and default quadrature before traits; both must use explicit `DeformationGradientKernel<Basis, Quadrature>` + `DeformationGradientElementModel<Kernel>` and old-vs-new regression tests |
| `ElementModel<Kernel, ElasticModel, PlasticModel>` causes template explosion | Medium | Keep the mathematical model in docs, but implement first version as `DeformationGradientElementModel<Kernel>` with runtime elastic/plastic injection |
| Formulation API drifts into virtual base-class dispatch | Medium | Core API uses tag + concept/templates; `std::variant` appears only at Python/config boundaries |
| Topology-specific template factories are defined only in `.cpp` | High | Put constrained template definitions directly in `deformationModelFactory.h`; `.cpp` only holds non-template helpers |
| Python binding duplicates `SimulationMeshCore` or cannot keep its owned mesh alive | High | Move `SimulationMeshCore` into shared `simulation_mesh_core.h`; expose `mesh() const -> const SimulationMesh &`; make `DeformationEnergyCore` hold `std::shared_ptr<SimulationMeshCore>` |
| Python public API is finalized before C++ boundaries stabilize | High | Task 4 defers public `pypgo.fem` / `pypgo.energy`; Task 10 finalizes the public API only after formulation, lifetime, material recipe, and DofLayout refactors are stable |
| `HexTricubicHermite` silently uses trilinear path | High | Explicit implementation guard must throw `not implemented` |
| Shell DOF gather/scatter regresses on `vid < 0` slots | Medium | `Vertex3DofLayout` owns DOF-side `vid < 0` zero-local behavior in gather, scatter, and sparsity; add layout-level tests covering boundary triangles |
| Shell missing-neighbor geometry semantics drift after migration | High | `ShellKoiterStencil` / `FundamentalFormsKernel` from Task 5e own missing-neighbor handling (`-10496` sentinel or explicit mask); parity tests vs `KoiterDeformationModel` cover energy, gradient, Hessian, `df/da`, `df/db` on boundary triangles before Task 5q deletes the oracle |

## Milestone Exit Criteria

This plan is complete when:

- C++ can build deformation energy with explicit `TetP1{}`, `HexTrilinear{}`, and `ShellKoiter{}` formulation tags.
- `TetP1` and `HexTrilinear` both use real basis/quadrature/kernel types plus `DeformationGradientElementModel<Kernel>`, not legacy full element models hidden behind traits.
- Wrong topology/formulation combinations fail at compile time in core C++.
- Topology-specific factory template definitions are available directly from `deformationModelFactory.h`, so tests, tools, C API adapters, and Python bindings can instantiate them without linker surprises.
- Runtime formulation selection is isolated to `std::variant` boundary adapters for Python/config.
- `FormulationTraits` per category: volumetric formulations declare `DofLayout` / `Basis` / `Quadrature` / `Kernel` / `ElementModel`; shell formulations declare `DofLayout` / `ElementStencil` / `Kernel` / `ElementModel`. C++20 concepts（`VolumetricFormulationCategory`, `ShellFormulationCategory`）verify the required alias sets at compile time. No sentinel/placeholder aliases. Traits do not bind concrete elastic or plastic model types. Shell traits use `KoiterShellElementModel<Kernel>` from Task 5e, not the legacy `KoiterDeformationModel`.
- New formulation/factory/material/DOF files follow the `formulations/`, `factories/`, and `materials/` directory split; legacy wrapper files may remain at the module root during migration.
- Old public `makeDeformationModel(...)` auto-dispatch entry has been removed.
- Python can construct tet P1, cubic hex trilinear, and shell Koiter deformation energy.
- Python cubic deformation API requires or visibly records `HexTrilinear`.
- Python deformation energy supports ENu, shell Koiter StVK, Mooney-Rivlin, Hill fiber composite, and OrthotropicStVK through explicit elastic recipe objects.
- Manager creation logic is split into elastic/plastic/element factories.
- Material payload conversion and elastic law construction are centralized in `SimulationMesh` payload conversion plus `ElasticModelFactory`.
- Assembler uses `DofLayout` for gather/scatter/sparsity on the existing vertex path.
- Unsupported `HexTricubicHermite` is recognized but fails explicitly.
- C++ and Python tests pass with the commands above.

---

## Rework Needed for Completed Tasks

设计决策 2 的新方向（去掉 sentinel、per-category concept、`ElementStencil` 概念）和已完成 Task 1–4 的实现之间需要对齐的地方如下。

### 两层 concept 的分工（澄清）

per-tag concept 和 per-category concept **不冲突、不互相替代**——两者回答不同问题：

| 层级 | 概念 | 回答问题 | 用途 |
|---|---|---|---|
| per-tag（按 mesh type 分组）| `TetFormulation`, `CubicFormulation`, `ShellFormulation` | "哪些 formulation tag 能传进这个 topology 的 factory？" | factory 模板约束 + `\|\|` 扩展 |
| per-category（按 alias 集合分组）| `VolumetricFormulationCategory`, `ShellFormulationCategory` | "这个 formulation 有 Basis+Quadrature 还是有 ElementStencil？" | `std::visit` 内 `if constexpr` 分发 |

两者都保留。

### R-1: `formulationConcepts.h` — 加 per-category concept

**当前代码状态（Task 2 产出）：**

```cpp
// formulationConcepts.h
template<class F> concept TetFormulation = std::same_as<F, TetP1>;
template<class F> concept CubicFormulation = std::same_as<F, HexTrilinear>;
template<class F> concept ShellFormulation = std::same_as<F, ShellKoiter>;
```

**需要最终状态（同一文件，加 per-category concept）：**

```cpp
// formulationConcepts.h
// Per-tag（保留，不动）
template<class F> concept TetFormulation = std::same_as<F, TetP1>;
template<class F> concept CubicFormulation = std::same_as<F, HexTrilinear>;
template<class F> concept ShellFormulation = std::same_as<F, ShellKoiter>;

// Per-category（新增）
template<class F>
concept VolumetricFormulationCategory = requires {
  typename FormulationTraits<F>::Basis;
  typename FormulationTraits<F>::Quadrature;
  typename FormulationTraits<F>::Kernel;
  typename FormulationTraits<F>::ElementModel;
};

template<class F>
concept ShellFormulationCategory = requires {
  typename FormulationTraits<F>::ElementStencil;
  typename FormulationTraits<F>::Kernel;
  typename FormulationTraits<F>::ElementModel;
};
```

**执行约束：**
- `VolumetricFormulationCategory` **现在就加**——volumetric traits 已有 `Basis`/`Quadrature`，可编译。
- `ShellFormulationCategory` **等 Task 5e Sub-task D 再加**——依赖 `FormulationTraits<ShellKoiter>::ElementStencil`（= `ShellKoiterStencil`），这个类型要等 Task 5e Sub-task A 创建。

### R-2: 无——factory 不改

`deformationModelFactory.h` 的 `template<TetFormulation F>` / `template<CubicFormulation F>` / `template<ShellFormulation F>` 保持不变。per-tag concept 的 `||` 扩展性（将来 `CubicFormulation = HexTrilinear || HexTricubicHermite`）是正确的设计。

### 不需要 rework 的地方

| 已完成 Task | 判断 |
|---|---|
| Task 1 (basis/quadrature/kernel) | 无影响 |
| Task 2 (per-tag concepts + traits) | per-tag concept 保留；TetP1/HexTrilinear traits 无 `ElementStencil`——正好是期望形状；ShellKoiter traits 由 Task 5e 补充 |
| Task 3 (borrow-only) | 无影响 |
| Task 4 (defer Python API) | 无影响 |
