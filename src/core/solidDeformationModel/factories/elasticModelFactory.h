#pragma once

#include "../deformationModelManager.h"  // DeformationModelElasticMaterial
#include "../formulations/parameters/parameterField.h"
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
  // Create the elastic material model for one element.
  // fiberDirection: 3-vector in the element's fiber frame (row 0 of fiberAxesRest),
  // used by Hill-type materials. May be nullptr when no fiber direction is configured.
  static std::unique_ptr<ElasticModel> create(
    const SimulationMesh &mesh,
    int ele,
    DeformationModelElasticMaterial type,
    const double *fiberDirection);

  static std::string modelId(DeformationModelElasticMaterial type);
  static DeformationModelElasticMaterial materialFromModelId(const std::string &modelId);
  static ParameterFieldSpec parameterSpec(
    const SimulationMesh &mesh,
    DeformationModelElasticMaterial type);

  static std::shared_ptr<OptimizableField> createDefaultField(
    const SimulationMesh &mesh,
    DeformationModelElasticMaterial type);

  static std::shared_ptr<OptimizableField> createElementwiseField(
    const SimulationMesh &mesh,
    DeformationModelElasticMaterial type,
    EigenSupport::VXd values);

  // Initialize default elastic parameter snapshot for the full mesh.
  // Currently handles KOITER_STVK shell params; leaves others zero.
  // Returns per-element elastic params (flattened, size nele * paramsPerElement).
  static EigenSupport::VXd initializeDefaultElasticParams(
    const SimulationMesh &mesh,
    DeformationModelElasticMaterial elastic,
    int numElasticParams);
};

}  // namespace SolidDeformationModel
}  // namespace pgo
