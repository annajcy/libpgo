#include "cubicLinearFormulation.h"

#include "formulations/quadrature/gaussLegendreHexQuadrature.h"
#include "formulations/shapeFunction/cubicLinearShapeFunction.h"

namespace pgo
{
namespace SolidDeformationModel
{

CubicLinearFormulation::CubicLinearFormulation()
  : CubicFormulation(
      std::make_unique<CubicLinearShapeFunction>(),
      std::make_unique<GaussLegendreHexQuadrature2>())
{
}

std::string_view CubicLinearFormulation::getName() const { return "cubic_linear"; }
int CubicLinearFormulation::numBasisFunctionsPerElement() const { return 8; }
int CubicLinearFormulation::getLocalDofs() const { return 24; }

}  // namespace SolidDeformationModel
}  // namespace pgo
