#include "tetLinearFormulation.h"

#include "formulations/quadrature/tetLinearDefaultQuadrature.h"
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
int TetLinearFormulation::getNodesPerElement() const { return 4; }
int TetLinearFormulation::getLocalDofs() const { return 12; }

}  // namespace SolidDeformationModel
}  // namespace pgo
