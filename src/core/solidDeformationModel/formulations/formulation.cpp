#include "formulation.h"

#include "basis/tetP1Basis.h"
#include "basis/hexTrilinearBasis.h"
#include "quadrature/tetP1DefaultQuadrature.h"
#include "quadrature/gaussLegendreHexQuadrature.h"
#include "kernels/deformationGradientKernel.h"
#include "kernels/koiterShellKernel.h"

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

std::unique_ptr<DeformationGradientKernel> VolumetricFormulation::createKernel(
  const double *restPositions) const
{
  return std::make_unique<DeformationGradientKernel>(
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

}  // namespace SolidDeformationModel
}  // namespace pgo
