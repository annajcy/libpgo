#include "plasticModelFactory.h"

#include "../simulationMesh.h"

#include "../plasticModel.h"
#include "../plasticModel3DDeformationGradient.h"
#include "../plasticModel3D3DOF.h"
#include "../plasticModel3D6DOF.h"
#include "../plasticModel3DConstant.h"

#include "../plasticModel2DFundamentalForms.h"
#include "../plasticModel2DFundamentalFormsUniformStretch.h"

#include "pgoLogging.h"
#include "EigenSupport.h"

namespace pgo::SolidDeformationModel
{
namespace ES = EigenSupport;

PlasticModelResult PlasticModelFactory::create(
  const SimulationMesh &mesh,
  int ele,
  DeformationModelPlasticMaterial type,
  const double *fiberAxesRestRow0)
{
  PlasticModelResult result;

  if (type == DeformationModelPlasticMaterial::VOLUMETRIC_DOF0) {
    ES::M3d I = ES::M3d::Identity();
    result.volConstant = new PlasticModel3DConstant(I.data());
    result.model = result.volConstant;
  }
  else if (type == DeformationModelPlasticMaterial::VOLUMETRIC_DOF3) {
    result.vol3DOF = new PlasticModel3D3DOF(fiberAxesRestRow0);
    result.model = result.vol3DOF;
  }
  else if (type == DeformationModelPlasticMaterial::VOLUMETRIC_DOF6) {
    result.vol6DOF = new PlasticModel3D6DOF();
    result.model = result.vol6DOF;
  }
  else if (type == DeformationModelPlasticMaterial::SHELL_FF_DOF0) {
    result.shellConstant = new PlasticModel2DFundamentalForms();
    result.model = result.shellConstant;
  }
  else if (type == DeformationModelPlasticMaterial::SHELL_FF_DOF1) {
    result.shellUniformStretch = new PlasticModel2DFundamentalFormsUniformStretch();
    result.model = result.shellUniformStretch;
  }
  else {
    throw std::runtime_error("unknown plastic model");
  }

  return result;
}

ES::VXd PlasticModelFactory::initializeDefaultPlasticParams(
  int nele,
  int numPlasticParams,
  PlasticModel *const *plasticModels)
{
  ES::VXd plasticParams(static_cast<Eigen::Index>(nele) * numPlasticParams);

  if (numPlasticParams > 0) {
    plasticParams.setZero();
    const ES::M3d identity = ES::M3d::Identity();
    for (int ei = 0; ei < nele; ei++) {
      const auto *pm = dynamic_cast<const PlasticModel3DDeformationGradient *>(plasticModels[ei]);
      if (pm)
        pm->toParam(identity.data(), plasticParams.data() + ei * numPlasticParams);
    }
  }

  return plasticParams;
}

}  // namespace pgo::SolidDeformationModel
