#include "plasticModelFactory.h"

#include "../plasticModel.h"
#include "../plasticModel3DDeformationGradient.h"
#include "../plasticModel3D3DOF.h"
#include "../plasticModel3D6DOF.h"
#include "../plasticModel3DConstant.h"

#include "../plasticModel2DFundamentalForms.h"
#include "../plasticModel2DFundamentalFormsUniformStretch.h"

namespace pgo::SolidDeformationModel
{
namespace ES = EigenSupport;

int PlasticModelFactory::numParameters(DeformationModelPlasticMaterial type)
{
  switch (type) {
  case DeformationModelPlasticMaterial::VOLUMETRIC_DOF0:
    return 0;
  case DeformationModelPlasticMaterial::VOLUMETRIC_DOF3:
    return 3;
  case DeformationModelPlasticMaterial::VOLUMETRIC_DOF6:
    return 6;
  case DeformationModelPlasticMaterial::SHELL_FF_DOF0:
    return 0;
  case DeformationModelPlasticMaterial::SHELL_FF_DOF1:
    return 1;
  default:
    return 0;
  }
}

PlasticModelResult PlasticModelFactory::create(
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
    // fiberAxesRestRow0 is null when no fiber directions were provided;
    // fall back to identity so the constructor's memcpy is safe.
    ES::M3d I = ES::M3d::Identity();
    result.vol3DOF = new PlasticModel3D3DOF(fiberAxesRestRow0 ? fiberAxesRestRow0 : I.data());
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
