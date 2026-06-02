#pragma once

namespace pgo
{
namespace SolidDeformationModel
{

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

// Ownership bag returned by SimulationMeshMaterial::createElasticModel.
// elementMaterial is the pointer passed to the element model constructor.
// Typed sub-pointers are for ownership tracking in DeformationModelManager;
// only those matching the requested type will be non-null.
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

}  // namespace SolidDeformationModel
}  // namespace pgo
