#include "formulation.h"

#include "basis/tetP1Basis.h"
#include "basis/hexTrilinearBasis.h"
#include "quadrature/tetP1DefaultQuadrature.h"
#include "quadrature/gaussLegendreHexQuadrature.h"
#include "kernels/volumetricKernel.h"
#include "kernels/koiterShellKernel.h"
#include "elements/volumetricElementModel.h"
#include "elements/shellElementModel.h"
#include "../simulationMesh.h"

#include <vector>

namespace pgo
{
namespace SolidDeformationModel
{

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
// KoiterShellFormulation
// ============================================================

std::string_view KoiterShellFormulation::getName() const { return "shell_koiter"; }
int KoiterShellFormulation::getNodesPerElement() const { return 6; }
int KoiterShellFormulation::getLocalDofs() const { return 18; }

// ============================================================
// createElement implementations
// ============================================================

std::unique_ptr<DeformationModel> VolumetricFormulation::createElement(
  const SimulationMesh &mesh, int ele,
  const ElasticBlock &elasticBlock, const PlasticBlock &plasticBlock) const
{
  const int numNodes = getNodesPerElement();
  std::vector<double> restPosition(numNodes * 3);
  for (int j = 0; j < numNodes; j++)
    mesh.getVertex(ele, j, &restPosition[3 * j]);

  auto kernel = createKernel(restPosition.data());
  return std::make_unique<VolumetricElementModel>(
    ele, std::move(*kernel), elasticBlock, plasticBlock);
}

std::unique_ptr<DeformationModel> ShellFormulation::createElement(
  const SimulationMesh &mesh, int ele,
  const ElasticBlock &elasticBlock, const PlasticBlock &plasticBlock) const
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
  return std::make_unique<ShellElementModel>(
    ele, std::move(kernel), elasticBlock, plasticBlock);
}

}  // namespace SolidDeformationModel
}  // namespace pgo
