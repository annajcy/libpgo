#pragma once

#include "material/fields/parameterField.h"
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
class OptimizableField;

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
  static ParameterFieldSpec parameterSpec(
    const SimulationMesh &mesh,
    DeformationModelElasticMaterial type);

  static std::shared_ptr<OptimizableField> createDefaultElementwiseField(
    const SimulationMesh &mesh,
    DeformationModelElasticMaterial type);

  static std::shared_ptr<OptimizableField> createElementwiseField(
    const SimulationMesh &mesh,
    DeformationModelElasticMaterial type,
    EigenSupport::VXd values);

  // Constant (mesh-wide shared) elastic parameter field: a single set of
  // numChannels parameters used by every element. The default variant seeds the
  // shared values from element 0's material.
  static std::shared_ptr<OptimizableField> createDefaultConstantField(
    const SimulationMesh &mesh,
    DeformationModelElasticMaterial type);

  static std::shared_ptr<OptimizableField> createConstantField(
    const SimulationMesh &mesh,
    DeformationModelElasticMaterial type,
    EigenSupport::VXd values);

  // Initialize default elastic parameter-field values for the full mesh.
  // Returns per-element values (flattened, size nele * paramsPerElement).
  static EigenSupport::VXd initializeDefaultElasticParams(
    const SimulationMesh &mesh,
    DeformationModelElasticMaterial elastic,
    int numElasticParams);
};

}  // namespace SolidDeformationModel
}  // namespace pgo
