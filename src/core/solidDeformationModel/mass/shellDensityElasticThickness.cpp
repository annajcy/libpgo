#include "shellDensityElasticThickness.h"

#include "material/core/materialParameters.h"
#include "simulation/simulationMesh.h"

#include <stdexcept>

namespace pgo
{
namespace SolidDeformationModel
{

ShellDensityElasticThickness::ShellDensityElasticThickness(double density,
  MaterialParameterRef thickness):
  density_(density),
  thickness_(std::move(thickness))
{
  if (!(density > 0.0)) {
    throw std::invalid_argument("ShellDensityElasticThickness requires density > 0");
  }
  if (thickness_.name() != "thickness") {
    throw std::invalid_argument(
      "ShellDensityElasticThickness parameter must be named 'thickness'");
  }
}

void ShellDensityElasticThickness::validate(const SimulationMesh &mesh) const
{
  ShellMassField::validate(mesh);
  if (thickness_.field().dofLayout().numElements() != mesh.getNumElements()) {
    throw std::invalid_argument(
      "ShellDensityElasticThickness parameter field shape does not match mesh element count");
  }
}

double ShellDensityElasticThickness::arealDensity(
  int ele, MaterialParameterEvaluationView state) const
{
  if (&thickness_.field() != &state.space().elastic())
    throw std::invalid_argument("ShellDensityElasticThickness parameter must belong to the evaluation space elastic field");
  return density_ * thickness_.value(ele, 0, state);
}

void ShellDensityElasticThickness::arealDensityParameterDerivative(
  int ele, MaterialParameterEvaluationView state, double *out) const
{
  thickness_.localDerivative(ele, 0, state, out);
  const int numLocal = thickness_.field().dofLayout().numLocalDofs();
  for (int k = 0; k < numLocal; k++)
    out[k] *= density_;
}

}  // namespace SolidDeformationModel
}  // namespace pgo
