#include "formulationDynamics.h"

#include "formulation.h"
#include "basis/hexTricubicHermiteBasis.h"
#include "quadrature/gaussLegendreHexQuadrature.h"
#include "barycentricCoordinates.h"
#include "generateMassMatrix.h"
#include "volumetricMesh.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string_view>

namespace pgo
{
namespace SolidDeformationModel
{
namespace
{

namespace ES = EigenSupport;

constexpr int kHermiteNodes = 64;
constexpr int kHermiteModes = 8;
constexpr int kHermiteDofsPerVertex = 24;
constexpr int kHermiteLocalDofs = 192;

bool isHermite(const Formulation &formulation)
{
  return formulation.getName() == std::string_view("hex_tricubic_hermite");
}

bool isShell(const Formulation &formulation)
{
  return formulation.getName() == std::string_view("shell_koiter");
}

void requireVolumetricFormulation(const Formulation &formulation)
{
  if (isShell(formulation)) {
    throw std::invalid_argument("formulation dynamics volume operators do not support shell_koiter");
  }
}

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

std::vector<int> modesForPolicy(HermiteBoundaryPolicy policy)
{
  switch (policy) {
    case HermiteBoundaryPolicy::Value:
      return { 0 };
    case HermiteBoundaryPolicy::First:
      return { 0, 1, 2, 3 };
    case HermiteBoundaryPolicy::All:
      return { 0, 1, 2, 3, 4, 5, 6, 7 };
  }
  throw std::invalid_argument("unknown HermiteBoundaryPolicy");
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

  const int numDofs = mesh.getNumVertices() * kHermiteDofsPerVertex;
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
            va * kHermiteDofsPerVertex + ma * 3 + coord,
            vb * kHermiteDofsPerVertex + mb * 3 + coord,
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

  ES::VXd force = ES::VXd::Zero(mesh.getNumVertices() * kHermiteDofsPerVertex);
  const ES::VXd &unitBody = hermiteUnitBody64();

  for (int ele = 0; ele < mesh.getNumElements(); ele++) {
    const double scale = mesh.getElementVolume(ele) * mesh.getElementDensity(ele);
    for (int node = 0; node < kHermiteNodes; node++) {
      const int vertex = mesh.getVertexIndex(ele, node / kHermiteModes);
      const int mode = node % kHermiteModes;
      const int base = vertex * kHermiteDofsPerVertex + mode * 3;
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
          vertex * kHermiteDofsPerVertex + mode * 3 + coord,
          value);
      }
    }
  }

  ES::SpMatD W(numTargets * 3, mesh.getNumVertices() * kHermiteDofsPerVertex);
  W.setFromTriplets(entries.begin(), entries.end());
  return W;
}

}  // namespace

EigenSupport::SpMatD buildFormulationMassMatrix(
  const VolumetricMeshes::VolumetricMesh &mesh,
  const Formulation &formulation)
{
  requireVolumetricFormulation(formulation);
  if (isHermite(formulation))
    return buildHermiteMassMatrix(mesh);

  ES::SpMatD M;
  VolumetricMeshes::GenerateMassMatrix::computeMassMatrix(&mesh, M, true);
  return M;
}

EigenSupport::VXd buildFormulationBodyForce(
  const VolumetricMeshes::VolumetricMesh &mesh,
  const Formulation &formulation,
  const EigenSupport::V3d &acceleration)
{
  requireVolumetricFormulation(formulation);
  if (isHermite(formulation))
    return buildHermiteBodyForce(mesh, acceleration);

  ES::SpMatD M;
  VolumetricMeshes::GenerateMassMatrix::computeMassMatrix(&mesh, M, true);
  ES::VXd accelField(mesh.getNumVertices() * 3);
  for (int vertex = 0; vertex < mesh.getNumVertices(); vertex++)
    accelField.segment<3>(vertex * 3) = acceleration;
  return M * accelField;
}

EigenSupport::SpMatD buildFormulationSurfaceEmbeddingMatrix(
  const VolumetricMeshes::VolumetricMesh &mesh,
  const Formulation &formulation,
  const EigenSupport::MXd &surfaceVertices)
{
  requireVolumetricFormulation(formulation);
  if (isHermite(formulation))
    return buildHermiteSurfaceEmbeddingMatrix(mesh, surfaceVertices);

  const int numTargets = static_cast<int>(surfaceVertices.rows());
  const std::vector<double> flat = flattenSurfaceVertices(surfaceVertices);
  InterpolationCoordinates::BarycentricCoordinates bc(numTargets, flat.data(), &mesh);
  return bc.generateInterpolationMatrix();
}

std::vector<int> hermiteVertexDofs(
  const std::vector<int> &vertexIds,
  HermiteBoundaryPolicy policy)
{
  const std::vector<int> modes = modesForPolicy(policy);
  std::vector<int> dofs;
  dofs.reserve(vertexIds.size() * modes.size() * 3);

  for (int vertex : vertexIds) {
    if (vertex < 0)
      throw std::invalid_argument("Hermite vertex ids must be non-negative");
    for (int mode : modes) {
      const int base = vertex * kHermiteDofsPerVertex + mode * 3;
      dofs.push_back(base + 0);
      dofs.push_back(base + 1);
      dofs.push_back(base + 2);
    }
  }

  return dofs;
}

std::vector<int> hermiteFaceDofs(
  const VolumetricMeshes::VolumetricMesh &mesh,
  int axis,
  bool maxSide,
  HermiteBoundaryPolicy policy)
{
  if (axis < 0 || axis >= 3)
    throw std::invalid_argument("Hermite face axis must be 0, 1, or 2");

  double target = mesh.getVertex(0)[axis];
  for (int vertex = 1; vertex < mesh.getNumVertices(); vertex++) {
    const double x = mesh.getVertex(vertex)[axis];
    target = maxSide ? std::max(target, x) : std::min(target, x);
  }

  const double tol = 1e-9 * std::max(1.0, std::abs(target));
  std::vector<int> vertices;
  for (int vertex = 0; vertex < mesh.getNumVertices(); vertex++) {
    if (std::abs(mesh.getVertex(vertex)[axis] - target) <= tol)
      vertices.push_back(vertex);
  }

  return hermiteVertexDofs(vertices, policy);
}

}  // namespace SolidDeformationModel
}  // namespace pgo
