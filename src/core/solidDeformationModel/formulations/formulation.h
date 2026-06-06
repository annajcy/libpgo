#pragma once

#include "EigenDef.h"

#include <memory>
#include <string_view>

namespace pgo
{
namespace VolumetricMeshes
{
class VolumetricMesh;
}

namespace SolidDeformationModel
{

class Basis;
class Quadrature;
class VolumetricKernel;
class ShellKernel;
class SimulationMesh;
class DeformationModel;
class ElasticModel;
class PlasticModel;
class ParameterField;
class DofLayout;
enum class SimulationMeshType;

// ============================================================
// Formulation — top-level abstract base
// ============================================================

class Formulation
{
public:
  virtual ~Formulation() = default;
  virtual std::string_view getName() const = 0;
  virtual int getNodesPerElement() const = 0;
  virtual int getLocalDofs() const = 0;

  virtual std::unique_ptr<DeformationModel> createElement(
    const SimulationMesh &mesh, int ele,
    std::unique_ptr<ElasticModel> elasticModel, std::unique_ptr<PlasticModel> plasticModel,
    const ParameterField *elasticParams, const ParameterField *plasticParams) const = 0;

  virtual SimulationMeshType compatibleMeshType() const = 0;

  // DOF-layout / rest-state policy. The default reproduces the historical behavior used by every
  // current formulation (tet P1, hex trilinear, shell Koiter): one mesh vertex carries 3 DOFs
  // (Vertex3DofLayout) and the global rest state is the vertex positions (size numVertices*3).
  //
  // A future tricubic Hermite formulation overrides both: createDofLayout returns a
  // HexTricubicHermiteDofLayout (global = vertexId*24 + mode*3 + coord) and buildGlobalRestDofs
  // emits the Hermite rest field (value + derivative modes per vertex). The manager calls these
  // once at its own construction (while the formulation is alive) and caches the results, so the
  // assembler/energy never see the vertex*3 assumption.
  //
  // Invariant: buildGlobalRestDofs(mesh).size() == createDofLayout(mesh)->numGlobalDofs().
  virtual std::unique_ptr<DofLayout> createDofLayout(const SimulationMesh &mesh) const;
  virtual EigenSupport::VXd buildGlobalRestDofs(const SimulationMesh &mesh) const;
};

// ============================================================
// VolumetricFormulation — owns Basis + Quadrature, creates
// VolumetricKernel per element.
// ============================================================

class VolumetricFormulation : public Formulation
{
public:
  VolumetricFormulation(std::unique_ptr<Basis> basis, std::unique_ptr<Quadrature> quad);
  ~VolumetricFormulation() override;

  const Basis &basis() const { return *basis_; }
  const Quadrature &quadrature() const { return *quad_; }

  std::unique_ptr<VolumetricKernel> createKernel(const double *restPositions) const;

  std::unique_ptr<DeformationModel> createElement(
    const SimulationMesh &mesh, int ele,
    std::unique_ptr<ElasticModel> elasticModel, std::unique_ptr<PlasticModel> plasticModel,
    const ParameterField *elasticParams, const ParameterField *plasticParams) const override;

  // Formulation-aware dynamics operators.
  //
  // Default (VolumetricFormulation): standard lumped-mass / barycentric interpolation path
  // using GenerateMassMatrix and BarycentricCoordinates.  TricubicHermiteFormulation
  // overrides with Hermite-specific operators.
  virtual EigenSupport::SpMatD buildMassMatrix(
    const VolumetricMeshes::VolumetricMesh &mesh) const;
  virtual EigenSupport::VXd buildBodyForce(
    const VolumetricMeshes::VolumetricMesh &mesh,
    const EigenSupport::V3d &acceleration) const;
  virtual EigenSupport::SpMatD buildSurfaceEmbeddingMatrix(
    const VolumetricMeshes::VolumetricMesh &mesh,
    const EigenSupport::MXd &surfaceVertices) const;

private:
  std::unique_ptr<Basis> basis_;
  std::unique_ptr<Quadrature> quad_;
};

// ============================================================
// ShellFormulation — creates ShellKernel per element.
// ============================================================

class ShellFormulation : public Formulation
{
public:
  std::unique_ptr<ShellKernel> createKernel(
    const double restX[18], const bool hasVtx[6]) const;

  std::unique_ptr<DeformationModel> createElement(
    const SimulationMesh &mesh, int ele,
    std::unique_ptr<ElasticModel> elasticModel, std::unique_ptr<PlasticModel> plasticModel,
    const ParameterField *elasticParams, const ParameterField *plasticParams) const override;

  SimulationMeshType compatibleMeshType() const override;
};

// ============================================================
// Intermediate tag classes — one per topology category
// ============================================================

class TetFormulation : public VolumetricFormulation
{
public:
  using VolumetricFormulation::VolumetricFormulation;
  SimulationMeshType compatibleMeshType() const override;
};

class CubicFormulation : public VolumetricFormulation
{
public:
  using VolumetricFormulation::VolumetricFormulation;
  SimulationMeshType compatibleMeshType() const override;
};

// ============================================================
// Concrete formulations
// ============================================================

class P1TetFormulation : public TetFormulation
{
public:
  P1TetFormulation();
  std::string_view getName() const override;
  int getNodesPerElement() const override;
  int getLocalDofs() const override;
};

class LinearCubicFormulation : public CubicFormulation
{
public:
  LinearCubicFormulation();
  std::string_view getName() const override;
  int getNodesPerElement() const override;
  int getLocalDofs() const override;
};

// Regular-grid tricubic Hermite hex: 64 scalar basis functions (8 corners x 8 Hermite modes),
// 192 local DOFs, 4x4x4 Gauss quadrature. Overrides the DOF-layout / rest-state policy with the
// 24-DOF-per-vertex Hermite layout and synthesizes per-corner rest Hermite DOFs from element edge
// vectors (value = corner position, axis modes = edge vectors, mixed = 0). MVP scope: uniform
// axis-aligned / affine-parallelepiped grid (so shared-vertex derivative DOFs agree across
// elements and the rest deformation gradient is exactly I).
class TricubicHermiteFormulation : public CubicFormulation
{
public:
  TricubicHermiteFormulation();
  std::string_view getName() const override;
  int getNodesPerElement() const override;
  int getLocalDofs() const override;

  std::unique_ptr<DofLayout> createDofLayout(const SimulationMesh &mesh) const override;
  EigenSupport::VXd buildGlobalRestDofs(const SimulationMesh &mesh) const override;

  EigenSupport::SpMatD buildMassMatrix(
    const VolumetricMeshes::VolumetricMesh &mesh) const override;
  EigenSupport::VXd buildBodyForce(
    const VolumetricMeshes::VolumetricMesh &mesh,
    const EigenSupport::V3d &acceleration) const override;
  EigenSupport::SpMatD buildSurfaceEmbeddingMatrix(
    const VolumetricMeshes::VolumetricMesh &mesh,
    const EigenSupport::MXd &surfaceVertices) const override;

  std::unique_ptr<DeformationModel> createElement(
    const SimulationMesh &mesh, int ele,
    std::unique_ptr<ElasticModel> elasticModel, std::unique_ptr<PlasticModel> plasticModel,
    const ParameterField *elasticParams, const ParameterField *plasticParams) const override;
};

class KoiterShellFormulation : public ShellFormulation
{
public:
  std::string_view getName() const override;
  int getNodesPerElement() const override;
  int getLocalDofs() const override;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
