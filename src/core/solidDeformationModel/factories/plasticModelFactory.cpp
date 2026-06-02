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

std::unique_ptr<PlasticModel> PlasticModelFactory::create(
  DeformationModelPlasticMaterial type,
  const double *fiberAxesRestRow0)
{
  if (type == DeformationModelPlasticMaterial::VOLUMETRIC_DOF0) {
    static constexpr double kIdentity[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
    return std::make_unique<PlasticModel3DConstant>(kIdentity);
  }
  else if (type == DeformationModelPlasticMaterial::VOLUMETRIC_DOF3) {
    static constexpr double kIdentity[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
    return std::make_unique<PlasticModel3D3DOF>(fiberAxesRestRow0 ? fiberAxesRestRow0 : kIdentity);
  }
  else if (type == DeformationModelPlasticMaterial::VOLUMETRIC_DOF6) {
    return std::make_unique<PlasticModel3D6DOF>();
  }
  else if (type == DeformationModelPlasticMaterial::SHELL_FF_DOF0) {
    return std::make_unique<PlasticModel2DFundamentalForms>();
  }
  else if (type == DeformationModelPlasticMaterial::SHELL_FF_DOF1) {
    return std::make_unique<PlasticModel2DFundamentalFormsUniformStretch>();
  }
  else {
    throw std::runtime_error("PlasticModelFactory::create: unknown plastic model type");
  }
}

ES::VXd PlasticModelFactory::initializeDefaultPlasticParams(
  int nele,
  int numPlasticParams,
  PlasticModel *const *plasticModels)
{
  ES::VXd plasticParams(static_cast<Eigen::Index>(nele) * numPlasticParams);
  plasticParams.setZero();
  if (numPlasticParams > 0) {
    for (int ei = 0; ei < nele; ei++)
      plasticModels[ei]->defaultParams(plasticParams.data() + ei * numPlasticParams);
  }
  return plasticParams;
}

}  // namespace pgo::SolidDeformationModel
