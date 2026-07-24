#pragma once

#include "material/fields/materialParameters.h"
#include "material/fields/materialFrameField.h"
#include "material/materialTypes.h"
#include "EigenDef.h"

#include <memory>
#include <string>

namespace pgo
{
namespace SolidDeformationModel
{

class SimulationMesh;
class ElasticModel;

class ElasticModelFactory
{
public:
  // Create the elastic material model for one element. Material axes are columns
  // of materialToReference; Hill materials consume column 0.
  static std::unique_ptr<ElasticModel> create(
    const SimulationMesh &mesh,
    int ele,
    DeformationModelElasticMaterial type,
    const MaterialFrame &materialToReference);

  static MaterialFrameRequirement materialFrameRequirement(
    DeformationModelElasticMaterial type);

  static std::string modelId(DeformationModelElasticMaterial type);
  static DeformationModelElasticMaterial materialFromModelId(const std::string &modelId);
  static MaterialParameterSpec parameterSpec(
    const SimulationMesh &mesh,
    DeformationModelElasticMaterial type);

  // Initialize default elastic parameter-field values for the full mesh.
  // Returns per-element values (flattened, size nele * paramsPerElement).
  static EigenSupport::VXd initializeDefaultElasticParams(
    const SimulationMesh &mesh,
    DeformationModelElasticMaterial elastic,
    int numElasticParams);
};

}  // namespace SolidDeformationModel
}  // namespace pgo
