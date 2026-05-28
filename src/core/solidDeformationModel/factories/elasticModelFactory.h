#pragma once

#include "../deformationModelManager.h"  // DeformationModelElasticMaterial
#include "EigenSupport.h"

namespace pgo
{
namespace SolidDeformationModel
{

class SimulationMesh;
class ElasticModel;
class ElasticModelStableNeoHookeanMaterial;
class ElasticModelLinearMaterial;
class ElasticModelHillTypeMaterial;
class ElasticModelInvariantBasedMaterial;
class ElasticModelVolumeMaterial;
class ElasticModel3DSTVKMaterial;
class ElasticModel3DMooneyRivlin;
template<int N>
class ElasticModelCombinedMaterial;
class ElasticModel2DFundamentalFormsFabric;
class ElasticModel2DFundamentalFormsSTVK;
class InvariantBasedMaterial;

// Result of creating an elastic material model for one element.
// elementMaterial is the pointer to pass to the element FEM constructor.
// Typed pointers are for ownership tracking in DeformationModelManager storage;
// only those matching the requested material type will be non-null.
struct ElasticModelResult
{
  ElasticModel *elementMaterial = nullptr;

  ElasticModelStableNeoHookeanMaterial *stableNeo = nullptr;
  ElasticModelLinearMaterial *linear = nullptr;
  ElasticModelHillTypeMaterial *hill = nullptr;
  ElasticModelInvariantBasedMaterial *invariantBased = nullptr;
  ElasticModelVolumeMaterial *volume = nullptr;
  ElasticModel3DSTVKMaterial *stvk = nullptr;
  ElasticModel3DMooneyRivlin *mooneyRivlin = nullptr;
  ElasticModelCombinedMaterial<2> *combined2 = nullptr;
  ElasticModelCombinedMaterial<3> *combined3 = nullptr;
  ElasticModel2DFundamentalFormsFabric *shellFabric = nullptr;
  ElasticModel2DFundamentalFormsSTVK *shellSTVK = nullptr;
  InvariantBasedMaterial *invariantModel = nullptr;
};

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
