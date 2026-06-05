#include "formulation.h"

#include "basis/tetP1Basis.h"
#include "basis/hexTrilinearBasis.h"
#include "basis/hexTricubicHermiteBasis.h"
#include "quadrature/tetP1DefaultQuadrature.h"
#include "quadrature/gaussLegendreHexQuadrature.h"
#include "kernels/volumetricKernel.h"
#include "kernels/koiterShellKernel.h"
#include "elements/volumetricDeformationModel.h"
#include "elements/shellDeformationModel.h"
#include "dof/vertex3DofLayout.h"
#include "dof/hexTricubicHermiteDofLayout.h"
#include "../simulationMesh.h"

#include <array>
#include <vector>

namespace pgo
{
namespace SolidDeformationModel
{

// ============================================================
// Formulation — DOF-layout / rest-state policy defaults
//
// These defaults encode the historical "one vertex = 3 DOFs" behavior shared by every current
// formulation. They are the seam a tricubic Hermite formulation overrides; see formulation.h.
// ============================================================

std::unique_ptr<DofLayout> Formulation::createDofLayout(const SimulationMesh &mesh) const
{
  return std::make_unique<Vertex3DofLayout>(mesh);
}

EigenSupport::VXd Formulation::buildGlobalRestDofs(const SimulationMesh &mesh) const
{
  const int nvtx = mesh.getNumVertices();
  EigenSupport::VXd rest(nvtx * 3);
  for (int vi = 0; vi < nvtx; vi++) {
    double p[3];
    mesh.getVertex(vi, p);
    rest[vi * 3 + 0] = p[0];
    rest[vi * 3 + 1] = p[1];
    rest[vi * 3 + 2] = p[2];
  }
  return rest;
}

// ============================================================
// VolumetricFormulation
// ============================================================

VolumetricFormulation::VolumetricFormulation(
  std::unique_ptr<Basis> basis, std::unique_ptr<Quadrature> quad)
  : basis_(std::move(basis)), quad_(std::move(quad))
{
}

VolumetricFormulation::~VolumetricFormulation() = default;

std::unique_ptr<VolumetricKernel> VolumetricFormulation::createKernel(
  const double *restPositions) const
{
  return std::make_unique<VolumetricKernel>(
    restPositions, *basis_, *quad_);
}

// ============================================================
// ShellFormulation
// ============================================================

std::unique_ptr<ShellKernel> ShellFormulation::createKernel(
  const double restX[18], const bool hasVtx[6]) const
{
  return std::make_unique<KoiterShellKernel>(restX, hasVtx);
}

// ============================================================
// P1TetFormulation
// ============================================================

P1TetFormulation::P1TetFormulation()
  : TetFormulation(
      std::make_unique<TetP1Basis>(),
      std::make_unique<TetP1DefaultQuadrature>())
{
}

std::string_view P1TetFormulation::getName() const { return "tet_p1"; }
int P1TetFormulation::getNodesPerElement() const { return 4; }
int P1TetFormulation::getLocalDofs() const { return 12; }

// ============================================================
// LinearCubicFormulation
// ============================================================

LinearCubicFormulation::LinearCubicFormulation()
  : CubicFormulation(
      std::make_unique<HexTrilinearBasis>(),
      std::make_unique<GaussLegendreHexQuadrature2>())
{
}

std::string_view LinearCubicFormulation::getName() const { return "hex_trilinear"; }
int LinearCubicFormulation::getNodesPerElement() const { return 8; }
int LinearCubicFormulation::getLocalDofs() const { return 24; }

// ============================================================
// TricubicHermiteFormulation
// ============================================================

namespace
{
// Per-corner Hermite rest DOFs of an element, in the basis node order (node = corner*8 + mode):
//   mode 0 (VALUE) = corner position; modes 1/2/3 (DXI/DETA/DZETA) = the element's xi/eta/zeta edge
//   vectors; modes 4..7 (mixed) = 0. For an affine/axis-aligned hex this exactly reproduces the
//   rest geometry (so the rest deformation gradient is I). Corner ordering follows the basis.
void elementHermiteRestDofs(const SimulationMesh &mesh, int ele, std::array<double, 192> &rest)
{
  double P[8][3];
  for (int c = 0; c < 8; c++)
    mesh.getVertex(ele, c, P[c]);

  // Constant edge vectors of the (affine) hex. Corners: 0=(0,0,0) 1=(1,0,0) 3=(0,1,0) 4=(0,0,1).
  double dXi[3], dEta[3], dZeta[3];
  for (int k = 0; k < 3; k++) {
    dXi[k] = P[1][k] - P[0][k];
    dEta[k] = P[3][k] - P[0][k];
    dZeta[k] = P[4][k] - P[0][k];
  }

  rest.fill(0.0);
  for (int c = 0; c < 8; c++) {
    const double *mode[4] = { P[c], dXi, dEta, dZeta };  // modes 0..3; modes 4..7 stay zero
    for (int m = 0; m < 4; m++) {
      const int node = c * 8 + m;
      for (int k = 0; k < 3; k++)
        rest[node * 3 + k] = mode[m][k];
    }
  }
}
}  // namespace

TricubicHermiteFormulation::TricubicHermiteFormulation()
  : CubicFormulation(
      std::make_unique<HexTricubicHermiteBasis>(),
      std::make_unique<GaussLegendreHexQuadrature4>())
{
}

std::string_view TricubicHermiteFormulation::getName() const { return "hex_tricubic_hermite"; }
int TricubicHermiteFormulation::getNodesPerElement() const { return 64; }
int TricubicHermiteFormulation::getLocalDofs() const { return 192; }

std::unique_ptr<DofLayout> TricubicHermiteFormulation::createDofLayout(const SimulationMesh &mesh) const
{
  return std::make_unique<HexTricubicHermiteDofLayout>(mesh);
}

EigenSupport::VXd TricubicHermiteFormulation::buildGlobalRestDofs(const SimulationMesh &mesh) const
{
  const int nvtx = mesh.getNumVertices();
  EigenSupport::VXd rest = EigenSupport::VXd::Zero(nvtx * HexTricubicHermiteDofLayout::kDofsPerVertex);

  // Scatter each element's per-corner Hermite rest DOFs to its vertices. For a uniform/affine grid
  // every element incident to a vertex writes the same value + edge vectors, so the result is
  // consistent with what createElement builds per element (gather(globalRest) == element rest).
  for (int ele = 0; ele < mesh.getNumElements(); ele++) {
    std::array<double, 192> local;
    elementHermiteRestDofs(mesh, ele, local);
    for (int c = 0; c < 8; c++) {
      int v = mesh.getVertexIndex(ele, c);
      if (v < 0)
        continue;
      const int base = v * HexTricubicHermiteDofLayout::kDofsPerVertex;
      for (int dof = 0; dof < HexTricubicHermiteDofLayout::kDofsPerVertex; dof++)
        rest[base + dof] = local[c * HexTricubicHermiteDofLayout::kDofsPerVertex + dof];
    }
  }
  return rest;
}

std::unique_ptr<DeformationModel> TricubicHermiteFormulation::createElement(
  const SimulationMesh &mesh, int ele,
  std::unique_ptr<ElasticModel> elasticModel, std::unique_ptr<PlasticModel> plasticModel,
  const ParameterField *elasticParams, const ParameterField *plasticParams) const
{
  std::array<double, 192> restPosition;
  elementHermiteRestDofs(mesh, ele, restPosition);

  auto kernel = createKernel(restPosition.data());
  return std::make_unique<VolumetricDeformationModel>(
    ele, std::move(*kernel), std::move(elasticModel), std::move(plasticModel),
    elasticParams, plasticParams);
}

// ============================================================
// KoiterShellFormulation
// ============================================================

std::string_view KoiterShellFormulation::getName() const { return "shell_koiter"; }
int KoiterShellFormulation::getNodesPerElement() const { return 6; }
int KoiterShellFormulation::getLocalDofs() const { return 18; }

// ============================================================
// compatibleMeshType implementations
// ============================================================

SimulationMeshType TetFormulation::compatibleMeshType() const { return SimulationMeshType::TET; }
SimulationMeshType CubicFormulation::compatibleMeshType() const { return SimulationMeshType::CUBIC; }
SimulationMeshType ShellFormulation::compatibleMeshType() const { return SimulationMeshType::SHELL; }

// ============================================================
// createElement implementations
// ============================================================

std::unique_ptr<DeformationModel> VolumetricFormulation::createElement(
  const SimulationMesh &mesh, int ele,
  std::unique_ptr<ElasticModel> elasticModel, std::unique_ptr<PlasticModel> plasticModel,
  const ParameterField *elasticParams, const ParameterField *plasticParams) const
{
  // NOTE (tricubic Hermite seam): this assumes every basis node IS a mesh vertex with a 3-vector
  // rest position. A tricubic Hermite formulation has 64 basis functions but only 8 mesh corners,
  // so it overrides createElement to synthesize the 64 local rest Hermite DOFs (value + derivative
  // modes) from the 8 corner positions; VolumetricKernel then consumes them unchanged.
  const int numNodes = getNodesPerElement();
  std::vector<double> restPosition(numNodes * 3);
  for (int j = 0; j < numNodes; j++)
    mesh.getVertex(ele, j, &restPosition[3 * j]);

  auto kernel = createKernel(restPosition.data());
  return std::make_unique<VolumetricDeformationModel>(
    ele, std::move(*kernel), std::move(elasticModel), std::move(plasticModel),
    elasticParams, plasticParams);
}

std::unique_ptr<DeformationModel> ShellFormulation::createElement(
  const SimulationMesh &mesh, int ele,
  std::unique_ptr<ElasticModel> elasticModel, std::unique_ptr<PlasticModel> plasticModel,
  const ParameterField *elasticParams, const ParameterField *plasticParams) const
{
  double restPosition[18] = {};
  bool hasVtx[6];
  for (int j = 0; j < 6; j++) {
    if (mesh.getVertexIndex(ele, j) < 0) {
      hasVtx[j] = false;
    }
    else {
      hasVtx[j] = true;
      mesh.getVertex(ele, j, restPosition + 3 * j);
    }
  }

  auto kernel = createKernel(restPosition, hasVtx);
  return std::make_unique<ShellDeformationModel>(
    ele, std::move(kernel), std::move(elasticModel), std::move(plasticModel),
    elasticParams, plasticParams);
}

}  // namespace SolidDeformationModel
}  // namespace pgo
