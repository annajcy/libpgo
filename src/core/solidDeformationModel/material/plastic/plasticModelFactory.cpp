#include "material/plastic/plasticModelFactory.h"

#include "material/plastic/plasticModel.h"
#include "material/plastic/plasticModel3D3DOF.h"
#include "material/plastic/plasticModel3D6DOF.h"
#include "material/plastic/plasticModel3DConstant.h"
#include "material/plastic/plasticModel2DFundamentalForms.h"
#include "material/plastic/plasticModel2DFundamentalFormsUniformStretch.h"
#include "simulation/simulationMesh.h"

#include <vector>

namespace pgo::SolidDeformationModel
{
namespace ES = EigenSupport;

int PlasticModelFactory::numParameters(DeformationModelPlasticMaterial type)
{
  return create(type, MaterialFrame::Identity())->getNumParameters();
}

std::unique_ptr<PlasticModel> PlasticModelFactory::create(
  DeformationModelPlasticMaterial type,
  const MaterialFrame &materialToReference)
{
  validateMaterialFrame(materialToReference);
  if (type == DeformationModelPlasticMaterial::VOLUMETRIC_DOF0) {
    static constexpr double kIdentity[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
    return std::make_unique<PlasticModel3DConstant>(kIdentity);
  }
  else if (type == DeformationModelPlasticMaterial::VOLUMETRIC_DOF3) {
    // PlasticModel3D3DOF retains the legacy row-axis internal convention R.
    // Keep that convention private at this boundary: R = Q^T.
    const MaterialFrame referenceToMaterial =
      materialToReference.transpose();
    return std::make_unique<PlasticModel3D3DOF>(referenceToMaterial);
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

MaterialFrameRequirement PlasticModelFactory::materialFrameRequirement(
  DeformationModelPlasticMaterial type)
{
  return type == DeformationModelPlasticMaterial::VOLUMETRIC_DOF3 ?
    MaterialFrameRequirement::FullFrame :
    MaterialFrameRequirement::None;
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

std::string PlasticModelFactory::modelId(DeformationModelPlasticMaterial type)
{
  switch (type) {
  case DeformationModelPlasticMaterial::VOLUMETRIC_DOF0:
    return "volumetric_dof0";
  case DeformationModelPlasticMaterial::VOLUMETRIC_DOF3:
    return "volumetric_dof3";
  case DeformationModelPlasticMaterial::VOLUMETRIC_DOF6:
    return "volumetric_dof6";
  case DeformationModelPlasticMaterial::SHELL_FF_DOF0:
    return "shell_ff_dof0";
  case DeformationModelPlasticMaterial::SHELL_FF_DOF1:
    return "shell_ff_dof1";
  default:
    throw std::runtime_error("PlasticModelFactory::modelId: unknown plastic model type");
  }
}

DeformationModelPlasticMaterial PlasticModelFactory::materialFromModelId(const std::string &modelId)
{
  if (modelId == "volumetric_dof0") return DeformationModelPlasticMaterial::VOLUMETRIC_DOF0;
  if (modelId == "volumetric_dof3") return DeformationModelPlasticMaterial::VOLUMETRIC_DOF3;
  if (modelId == "volumetric_dof6") return DeformationModelPlasticMaterial::VOLUMETRIC_DOF6;
  if (modelId == "shell_ff_dof0") return DeformationModelPlasticMaterial::SHELL_FF_DOF0;
  if (modelId == "shell_ff_dof1") return DeformationModelPlasticMaterial::SHELL_FF_DOF1;
  throw std::invalid_argument("PlasticModelFactory::materialFromModelId: unknown plastic model id: " + modelId);
}

MaterialParameterSpec PlasticModelFactory::parameterSpec(DeformationModelPlasticMaterial type)
{
  MaterialParameterSpec spec;
  spec.modelId = modelId(type);
  const int numChannels = numParameters(type);
  switch (type) {
  case DeformationModelPlasticMaterial::VOLUMETRIC_DOF6:
    spec.channelNames = { "Fxx", "Fxy", "Fxz", "Fyy", "Fyz", "Fzz" };
    break;
  case DeformationModelPlasticMaterial::VOLUMETRIC_DOF3:
    spec.channelNames = { "Fx", "Fy", "Fz" };
    break;
  case DeformationModelPlasticMaterial::SHELL_FF_DOF1:
    spec.channelNames = { "stretch" };
    break;
  default:
    for (int i = 0; i < numChannels; i++)
      spec.channelNames.push_back("parameter_" + std::to_string(i));
    break;
  }
  return spec;
}

}  // namespace pgo::SolidDeformationModel
