#pragma once

#include "material/fields/materialParameters.h"
#include "material/fields/materialFrameField.h"
#include "material/materialTypes.h"
#include "EigenSupport.h"

#include <memory>
#include <string>

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
  static std::string modelId(DeformationModelPlasticMaterial type);
  static DeformationModelPlasticMaterial materialFromModelId(const std::string &modelId);
  static MaterialParameterSpec parameterSpec(DeformationModelPlasticMaterial type);

  static MaterialFrameRequirement materialFrameRequirement(
    DeformationModelPlasticMaterial type);

  // Create the plastic model for one element. Material axes are columns of
  // materialToReference. VOLUMETRIC_DOF3 uses Q*S*Q^T; DOF6 ignores Q.
  static std::unique_ptr<PlasticModel> create(
    DeformationModelPlasticMaterial type,
    const MaterialFrame &materialToReference);

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
