#pragma once

#include "../deformationModelManager.h"  // DeformationModelPlasticMaterial
#include "EigenSupport.h"

namespace pgo
{
namespace SolidDeformationModel
{

class SimulationMesh;
class PlasticModel;
class PlasticModel3DConstant;
class PlasticModel3D3DOF;
class PlasticModel3D6DOF;
class PlasticModel2DFundamentalForms;
class PlasticModel2DFundamentalFormsUniformStretch;

// Result of creating a plastic model for one element.
struct PlasticModelResult
{
  PlasticModel *model = nullptr;

  PlasticModel3DConstant *volConstant = nullptr;
  PlasticModel3D3DOF *vol3DOF = nullptr;
  PlasticModel3D6DOF *vol6DOF = nullptr;
  PlasticModel2DFundamentalForms *shellConstant = nullptr;
  PlasticModel2DFundamentalFormsUniformStretch *shellUniformStretch = nullptr;
};

class PlasticModelFactory
{
public:
  // Number of plastic parameter channels for a given parametrization type.
  static int numParameters(DeformationModelPlasticMaterial type);

  // Create the plastic model for one element.
  // fiberAxesRestRow0: pointer to 9 doubles (3x3 row-major) for the element's
  // fiber axes in rest configuration. Only used by VOLUMETRIC_DOF3.
  static PlasticModelResult create(
    DeformationModelPlasticMaterial type,
    const double *fiberAxesRestRow0);

  // Initialize default plastic parameter snapshot for the full mesh.
  // plasticModels[0..nele-1] are the per-element plastic model pointers.
  // Volumetric deformation-gradient plastic encodes identity Fp.
  // Shell plastic params are zero/default stretch.
  // Returns per-element plastic params (flattened, size nele * paramsPerElement).
  static EigenSupport::VXd initializeDefaultPlasticParams(
    int nele,
    int numPlasticParams,
    PlasticModel *const *plasticModels);
};

}  // namespace SolidDeformationModel
}  // namespace pgo
