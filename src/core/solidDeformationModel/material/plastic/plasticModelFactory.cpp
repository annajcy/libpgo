#include "material/plastic/plasticModelFactory.h"

#include "material/plastic/plasticModel.h"
#include "material/plastic/plasticModel3D3DOF.h"
#include "material/plastic/plasticModel3D6DOF.h"
#include "material/plastic/plasticModel3DConstant.h"
#include "material/plastic/plasticModel2DFundamentalForms.h"
#include "material/plastic/plasticModel2DFundamentalFormsUniformStretch.h"
#include "simulation/simulationMesh.h"
#include "material/fields/elementwiseParameterField.h"
#include "material/fields/constantParameterField.h"

#include <vector>

namespace pgo::SolidDeformationModel
{
namespace ES = EigenSupport;

int PlasticModelFactory::numParameters(DeformationModelPlasticMaterial type)
{
  // Single source of truth: the plastic model's differentiable parameter count.
  // create() needs no mesh and the fiber axes do not affect the count, so a
  // throwaway model with a null axis is sufficient.
  return create(type, nullptr)->getNumParameters();
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

ParameterFieldSpec PlasticModelFactory::parameterSpec(DeformationModelPlasticMaterial type)
{
  ParameterFieldSpec spec;
  spec.domain = ParameterDomain::PLASTIC;
  spec.modelId = modelId(type);
  spec.numChannels = numParameters(type);
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
    break;
  }
  return spec;
}

std::shared_ptr<OptimizableField> PlasticModelFactory::createDefaultElementwiseField(
  const SimulationMesh &mesh,
  DeformationModelPlasticMaterial type)
{
  const int nele = mesh.getNumElements();
  const int np = numParameters(type);
  std::vector<std::unique_ptr<PlasticModel>> ownedPlasticModels(nele);
  std::vector<PlasticModel *> plasticModels(nele);
  for (int ei = 0; ei < nele; ei++) {
    ownedPlasticModels[ei] = create(type, nullptr);
    plasticModels[ei] = ownedPlasticModels[ei].get();
  }

  ES::VXd values = initializeDefaultPlasticParams(nele, np, plasticModels.data());
  return std::make_shared<ElementwiseParameterField>(parameterSpec(type), nele, std::move(values));
}

std::shared_ptr<OptimizableField> PlasticModelFactory::createElementwiseField(
  const SimulationMesh &mesh,
  DeformationModelPlasticMaterial type,
  ES::VXd values)
{
  return std::make_shared<ElementwiseParameterField>(
    parameterSpec(type), mesh.getNumElements(), std::move(values));
}

std::shared_ptr<OptimizableField> PlasticModelFactory::createDefaultConstantField(
  const SimulationMesh &mesh,
  DeformationModelPlasticMaterial type)
{
  const int np = numParameters(type);
  ES::VXd values(np);
  values.setZero();
  if (np > 0) {
    std::unique_ptr<PlasticModel> model = create(type, nullptr);
    model->defaultParams(values.data());
  }
  return std::make_shared<ConstantParameterField>(
    parameterSpec(type), mesh.getNumElements(), std::move(values));
}

std::shared_ptr<OptimizableField> PlasticModelFactory::createConstantField(
  const SimulationMesh &mesh,
  DeformationModelPlasticMaterial type,
  ES::VXd values)
{
  return std::make_shared<ConstantParameterField>(
    parameterSpec(type), mesh.getNumElements(), std::move(values));
}

}  // namespace pgo::SolidDeformationModel
