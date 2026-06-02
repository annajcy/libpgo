#pragma once

#include "../deformationModelManager.h"  // DeformationModelPlasticMaterial
#include "EigenSupport.h"

#include <memory>

namespace pgo
{
namespace SolidDeformationModel
{

class SimulationMesh;
class PlasticModel;

class PlasticModelFactory
{
public:
  // Number of plastic parameter channels for a given parametrization type.
  static int numParameters(DeformationModelPlasticMaterial type);

  // Create the plastic model for one element.
  // fiberAxesRestRow0: pointer to 9 doubles (3x3 row-major) for the element's
  // fiber axes in rest configuration. Only used by VOLUMETRIC_DOF3.
  static std::unique_ptr<PlasticModel> create(
    DeformationModelPlasticMaterial type,
    const double *fiberAxesRestRow0);

  // Initialize default plastic parameter snapshot for the full mesh.
  // Calls PlasticModel::defaultParams() on each element.
  // Returns per-element plastic params (flattened, size nele * paramsPerElement).
  static EigenSupport::VXd initializeDefaultPlasticParams(
    int nele,
    int numPlasticParams,
    PlasticModel *const *plasticModels);
};

}  // namespace SolidDeformationModel
}  // namespace pgo
