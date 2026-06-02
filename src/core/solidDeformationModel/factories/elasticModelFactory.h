#pragma once

#include "../elasticModelResult.h"
#include "../deformationModelManager.h"  // DeformationModelElasticMaterial
#include "EigenDef.h"

namespace pgo
{
namespace SolidDeformationModel
{

class SimulationMesh;

class ElasticModelFactory
{
public:
  // Create the elastic material model for one element.
  // fiberDirection: 3-vector in the element's fiber frame (row 0 of fiberAxesRest),
  // used by Hill-type materials. May be nullptr when no fiber direction is configured.
  static ElasticModelResult create(
    const SimulationMesh &mesh,
    int ele,
    DeformationModelElasticMaterial type,
    const double *fiberDirection);

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
