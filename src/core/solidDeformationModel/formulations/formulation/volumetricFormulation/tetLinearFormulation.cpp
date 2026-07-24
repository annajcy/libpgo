#include "tetLinearFormulation.h"

#include "formulations/quadrature/tetLinearDefaultQuadrature.h"
#include "formulations/quadrature/tetDegree2Quadrature.h"
#include "formulations/shapeFunction/tetLinearShapeFunction.h"

namespace pgo
{
namespace SolidDeformationModel
{

TetLinearFormulation::TetLinearFormulation()
  : TetFormulation(
      std::make_unique<TetLinearShapeFunction>(),
      std::make_unique<TetLinearDefaultQuadrature>())
{
}

std::string_view TetLinearFormulation::getName() const { return "tet_linear"; }
int TetLinearFormulation::numBasisFunctionsPerElement() const { return 4; }
int TetLinearFormulation::getLocalDofs() const { return 12; }

const Quadrature &TetLinearFormulation::massQuadrature() const
{
  static const TetDegree2Quadrature quad;
  return quad;
}

}  // namespace SolidDeformationModel
}  // namespace pgo
