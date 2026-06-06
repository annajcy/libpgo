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
#include "barycentricCoordinates.h"
#include "generateMassMatrix.h"
#include "volumetricMesh.h"
#include "../simulationMesh.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
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
// -- Hermite dynamics helpers -------------------------------------------------

namespace ES = EigenSupport;

constexpr int kHermiteNodes = 64;
constexpr int kHermiteModes = 8;
constexpr int kHermiteLocalDofs = 192;

const ES::MXd &hermiteUnitMass64()
{
  static const ES::MXd unitMass = [] {
    HexTricubicHermiteBasis basis;
    GaussLegendreHexQuadrature4 quadrature;
    ES::MXd M = ES::MXd::Zero(kHermiteNodes, kHermiteNodes);
    std::array<double, kHermiteNodes> H{};

    for (int q = 0; q < quadrature.numPoints(); q++) {
      double xi[3];
      quadrature.point(q, xi);
      basis.N(xi[0], xi[1], xi[2], H.data());
      const double w = quadrature.weight(q);
      for (int i = 0; i < kHermiteNodes; i++)
        for (int j = 0; j < kHermiteNodes; j++)
          M(i, j) += w * H[i] * H[j];
    }
    return M;
  }();
  return unitMass;
}

const ES::VXd &hermiteUnitBody64()
{
  static const ES::VXd unitBody = [] {
    HexTricubicHermiteBasis basis;
    GaussLegendreHexQuadrature4 quadrature;
    ES::VXd b = ES::VXd::Zero(kHermiteNodes);
    std::array<double, kHermiteNodes> H{};

    for (int q = 0; q < quadrature.numPoints(); q++) {
      double xi[3];
      quadrature.point(q, xi);
      basis.N(xi[0], xi[1], xi[2], H.data());
      const double w = quadrature.weight(q);
      for (int i = 0; i < kHermiteNodes; i++)
        b[i] += w * H[i];
    }
    return b;
  }();
  return unitBody;
}

std::vector<double> flattenSurfaceVertices(const ES::MXd &surfaceVertices)
{
  if (surfaceVertices.cols() != 3) {
    throw std::invalid_argument("surfaceVertices must have shape numVertices x 3");
  }
  std::vector<double> flat(static_cast<size_t>(surfaceVertices.rows()) * 3);
  for (Eigen::Index i = 0; i < surfaceVertices.rows(); i++)
    for (int d = 0; d < 3; d++)
      flat[static_cast<size_t>(i) * 3 + d] = surfaceVertices(i, d);
  return flat;
}

ES::SpMatD buildHermiteMassMatrix(const VolumetricMeshes::VolumetricMesh &mesh)
{
  if (mesh.getNumElementVertices() != 8) {
    throw std::invalid_argument("hex_tricubic_hermite mass requires an 8-corner cubic mesh");
  }

  const int numDofs = mesh.getNumVertices() * HexTricubicHermiteDofLayout::kDofsPerVertex;
  std::vector<ES::TripletD> entries;
  entries.reserve(static_cast<size_t>(mesh.getNumElements()) * kHermiteNodes * kHermiteNodes * 3);

  const ES::MXd &unitMass = hermiteUnitMass64();
  for (int ele = 0; ele < mesh.getNumElements(); ele++) {
    const double scale = mesh.getElementVolume(ele) * mesh.getElementDensity(ele);
    for (int a = 0; a < kHermiteNodes; a++) {
      const int va = mesh.getVertexIndex(ele, a / kHermiteModes);
      const int ma = a % kHermiteModes;
      for (int b = 0; b < kHermiteNodes; b++) {
        const double value = scale * unitMass(a, b);
        if (value == 0.0)
          continue;
        const int vb = mesh.getVertexIndex(ele, b / kHermiteModes);
        const int mb = b % kHermiteModes;
        for (int coord = 0; coord < 3; coord++) {
          entries.emplace_back(
            va * HexTricubicHermiteDofLayout::kDofsPerVertex + ma * 3 + coord,
            vb * HexTricubicHermiteDofLayout::kDofsPerVertex + mb * 3 + coord,
            value);
        }
      }
    }
  }

  ES::SpMatD M(numDofs, numDofs);
  M.setFromTriplets(entries.begin(), entries.end());
  return M;
}

ES::VXd buildHermiteBodyForce(
  const VolumetricMeshes::VolumetricMesh &mesh,
  const ES::V3d &acceleration)
{
  if (mesh.getNumElementVertices() != 8) {
    throw std::invalid_argument("hex_tricubic_hermite body force requires an 8-corner cubic mesh");
  }

  ES::VXd force = ES::VXd::Zero(mesh.getNumVertices() * HexTricubicHermiteDofLayout::kDofsPerVertex);
  const ES::VXd &unitBody = hermiteUnitBody64();

  for (int ele = 0; ele < mesh.getNumElements(); ele++) {
    const double scale = mesh.getElementVolume(ele) * mesh.getElementDensity(ele);
    for (int node = 0; node < kHermiteNodes; node++) {
      const int vertex = mesh.getVertexIndex(ele, node / kHermiteModes);
      const int mode = node % kHermiteModes;
      const int base = vertex * HexTricubicHermiteDofLayout::kDofsPerVertex + mode * 3;
      force.segment<3>(base) += scale * unitBody[node] * acceleration;
    }
  }

  return force;
}

ES::SpMatD buildHermiteSurfaceEmbeddingMatrix(
  const VolumetricMeshes::VolumetricMesh &mesh,
  const ES::MXd &surfaceVertices)
{
  if (mesh.getNumElementVertices() != 8) {
    throw std::invalid_argument("hex_tricubic_hermite surface embedding requires an 8-corner cubic mesh");
  }

  const int numTargets = static_cast<int>(surfaceVertices.rows());
  const std::vector<double> flat = flattenSurfaceVertices(surfaceVertices);
  InterpolationCoordinates::BarycentricCoordinates bc(numTargets, flat.data(), &mesh);

  if (bc.getNumElementVertices() != 8) {
    throw std::runtime_error("Hermite surface embedding expected 8 interpolation weights per target");
  }

  HexTricubicHermiteBasis basis;
  std::array<double, kHermiteNodes> H{};
  std::vector<ES::TripletD> entries;
  entries.reserve(static_cast<size_t>(numTargets) * kHermiteNodes * 3);

  for (int target = 0; target < numTargets; target++) {
    const double *w = bc.getEmbeddingWeights(target);
    const int *indices = bc.getEmbeddingVertexIndices(target);

    const double xi = w[1] + w[2] + w[5] + w[6];
    const double eta = w[2] + w[3] + w[6] + w[7];
    const double zeta = w[4] + w[5] + w[6] + w[7];
    basis.N(xi, eta, zeta, H.data());

    for (int node = 0; node < kHermiteNodes; node++) {
      const double value = H[node];
      if (value == 0.0)
        continue;
      const int vertex = indices[node / kHermiteModes];
      const int mode = node % kHermiteModes;
      for (int coord = 0; coord < 3; coord++) {
        entries.emplace_back(
          target * 3 + coord,
          vertex * HexTricubicHermiteDofLayout::kDofsPerVertex + mode * 3 + coord,
          value);
      }
    }
  }

  ES::SpMatD W(numTargets * 3, mesh.getNumVertices() * HexTricubicHermiteDofLayout::kDofsPerVertex);
  W.setFromTriplets(entries.begin(), entries.end());
  return W;
}

}  // namespace

// ============================================================
// VolumetricFormulation — dynamics operators (non-Hermite default)
// ============================================================

EigenSupport::SpMatD VolumetricFormulation::buildMassMatrix(
  const VolumetricMeshes::VolumetricMesh &mesh) const
{
  ES::SpMatD M;
  VolumetricMeshes::GenerateMassMatrix::computeMassMatrix(&mesh, M, true);
  return M;
}

EigenSupport::VXd VolumetricFormulation::buildBodyForce(
  const VolumetricMeshes::VolumetricMesh &mesh,
  const EigenSupport::V3d &acceleration) const
{
  ES::SpMatD M;
  VolumetricMeshes::GenerateMassMatrix::computeMassMatrix(&mesh, M, true);
  ES::VXd accelField(mesh.getNumVertices() * 3);
  for (int vertex = 0; vertex < mesh.getNumVertices(); vertex++)
    accelField.segment<3>(vertex * 3) = acceleration;
  return M * accelField;
}

EigenSupport::SpMatD VolumetricFormulation::buildSurfaceEmbeddingMatrix(
  const VolumetricMeshes::VolumetricMesh &mesh,
  const EigenSupport::MXd &surfaceVertices) const
{
  const int numTargets = static_cast<int>(surfaceVertices.rows());
  const std::vector<double> flat = flattenSurfaceVertices(surfaceVertices);
  InterpolationCoordinates::BarycentricCoordinates bc(numTargets, flat.data(), &mesh);
  return bc.generateInterpolationMatrix();
}

// ============================================================
// TricubicHermiteFormulation
// ============================================================

TricubicHermiteFormulation::TricubicHermiteFormulation()
  : CubicFormulation(
      std::make_unique<HexTricubicHermiteBasis>(),
      std::make_unique<GaussLegendreHexQuadrature4>())
{
}

std::string_view TricubicHermiteFormulation::getName() const { return "hex_tricubic_hermite"; }
int TricubicHermiteFormulation::getNodesPerElement() const { return 64; }
int TricubicHermiteFormulation::getLocalDofs() const { return 192; }

EigenSupport::SpMatD TricubicHermiteFormulation::buildMassMatrix(
  const VolumetricMeshes::VolumetricMesh &mesh) const
{
  return buildHermiteMassMatrix(mesh);
}

EigenSupport::VXd TricubicHermiteFormulation::buildBodyForce(
  const VolumetricMeshes::VolumetricMesh &mesh,
  const EigenSupport::V3d &acceleration) const
{
  return buildHermiteBodyForce(mesh, acceleration);
}

EigenSupport::SpMatD TricubicHermiteFormulation::buildSurfaceEmbeddingMatrix(
  const VolumetricMeshes::VolumetricMesh &mesh,
  const EigenSupport::MXd &surfaceVertices) const
{
  return buildHermiteSurfaceEmbeddingMatrix(mesh, surfaceVertices);
}

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
