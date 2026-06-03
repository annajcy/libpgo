#include "elasticModelFactory.h"

#include "../simulationMesh.h"
#include "../elasticModel.h"
#include "../formulations/parameters/elementwiseParameterField.h"
#include "../formulations/parameters/constantParameterField.h"

#include <stdexcept>

namespace pgo::SolidDeformationModel
{
namespace ES = EigenSupport;

std::unique_ptr<ElasticModel> ElasticModelFactory::create(
  const SimulationMesh &mesh,
  int ele,
  DeformationModelElasticMaterial type,
  const double *fiberDirection)
{
  const SimulationMeshMaterial *auxMat = nullptr;
  if (mesh.getElementNumMaterials(ele) > 1)
    auxMat = mesh.getElementMaterial(ele, 1);

  return mesh.getElementMaterial(ele, 0)->createElasticModel(type, fiberDirection, auxMat);
}

std::string ElasticModelFactory::modelId(DeformationModelElasticMaterial type)
{
  switch (type) {
  case DeformationModelElasticMaterial::STABLE_NEO:
    return "stable_neo";
  case DeformationModelElasticMaterial::STVK:
    return "stvk";
  case DeformationModelElasticMaterial::STVK_VOL:
    return "stvk_vol";
  case DeformationModelElasticMaterial::LINEAR:
    return "linear";
  case DeformationModelElasticMaterial::MOONEY_RIVLIN:
    return "mooney_rivlin";
  case DeformationModelElasticMaterial::KOITER_STVK:
    return "koiter_stvk";
  case DeformationModelElasticMaterial::KOITER_FABRIC:
    return "koiter_fabric";
  case DeformationModelElasticMaterial::HILL_STABLE_NEO:
    return "hill_stable_neo";
  case DeformationModelElasticMaterial::HILL_STVK:
    return "hill_stvk";
  case DeformationModelElasticMaterial::HILL_STVK_VOL:
    return "hill_stvk_vol";
  default:
    throw std::runtime_error("ElasticModelFactory::modelId: unknown elastic model type");
  }
}

DeformationModelElasticMaterial ElasticModelFactory::materialFromModelId(const std::string &modelId)
{
  if (modelId == "stable_neo") return DeformationModelElasticMaterial::STABLE_NEO;
  if (modelId == "stvk") return DeformationModelElasticMaterial::STVK;
  if (modelId == "stvk_vol") return DeformationModelElasticMaterial::STVK_VOL;
  if (modelId == "linear") return DeformationModelElasticMaterial::LINEAR;
  if (modelId == "mooney_rivlin") return DeformationModelElasticMaterial::MOONEY_RIVLIN;
  if (modelId == "koiter_stvk") return DeformationModelElasticMaterial::KOITER_STVK;
  if (modelId == "koiter_fabric") return DeformationModelElasticMaterial::KOITER_FABRIC;
  if (modelId == "hill_stable_neo") return DeformationModelElasticMaterial::HILL_STABLE_NEO;
  if (modelId == "hill_stvk") return DeformationModelElasticMaterial::HILL_STVK;
  if (modelId == "hill_stvk_vol") return DeformationModelElasticMaterial::HILL_STVK_VOL;
  throw std::invalid_argument("ElasticModelFactory::materialFromModelId: unknown elastic model id: " + modelId);
}

ParameterFieldSpec ElasticModelFactory::parameterSpec(
  const SimulationMesh &mesh,
  DeformationModelElasticMaterial type)
{
  ParameterFieldSpec spec;
  spec.domain = ParameterDomain::ELASTIC;
  spec.modelId = modelId(type);
  spec.numChannels = mesh.getElementMaterial(0, 0)->numElasticParameters(type);
  if (type == DeformationModelElasticMaterial::KOITER_STVK && spec.numChannels == 5) {
    spec.channelNames = { "E_membrane", "nu_membrane", "E_bending", "nu_bending", "thickness" };
  }
  else if (type == DeformationModelElasticMaterial::KOITER_FABRIC && spec.numChannels == 12) {
    spec.channelNames = {
      "membrane_warp", "membrane_weft", "membrane_shear", "membrane_cross",
      "bend_warp", "bend_weft", "bend_shear",
      "warp_stretch", "weft_stretch", "shear_stretch",
      "fiber_coupling", "thickness"
    };
  }
  else if ((type == DeformationModelElasticMaterial::HILL_STABLE_NEO ||
            type == DeformationModelElasticMaterial::HILL_STVK ||
            type == DeformationModelElasticMaterial::HILL_STVK_VOL) &&
      spec.numChannels == 1) {
    spec.channelNames = { "activation" };
  }
  return spec;
}

std::shared_ptr<OptimizableField> ElasticModelFactory::createDefaultElementwiseField(
  const SimulationMesh &mesh,
  DeformationModelElasticMaterial type)
{
  const auto spec = parameterSpec(mesh, type);
  auto values = initializeDefaultElasticParams(mesh, type, spec.numChannels);
  return std::make_shared<ElementwiseParameterField>(spec, mesh.getNumElements(), std::move(values));
}

std::shared_ptr<OptimizableField> ElasticModelFactory::createElementwiseField(
  const SimulationMesh &mesh,
  DeformationModelElasticMaterial type,
  ES::VXd values)
{
  return std::make_shared<ElementwiseParameterField>(
    parameterSpec(mesh, type), mesh.getNumElements(), std::move(values));
}

std::shared_ptr<OptimizableField> ElasticModelFactory::createDefaultConstantField(
  const SimulationMesh &mesh,
  DeformationModelElasticMaterial type)
{
  const auto spec = parameterSpec(mesh, type);
  // Seed the shared values from element 0's material (first segment of the
  // per-element default initialization).
  ES::VXd perElement = initializeDefaultElasticParams(mesh, type, spec.numChannels);
  ES::VXd values = spec.numChannels > 0 ? ES::VXd(perElement.head(spec.numChannels)) : ES::VXd();
  return std::make_shared<ConstantParameterField>(spec, mesh.getNumElements(), std::move(values));
}

std::shared_ptr<OptimizableField> ElasticModelFactory::createConstantField(
  const SimulationMesh &mesh,
  DeformationModelElasticMaterial type,
  ES::VXd values)
{
  return std::make_shared<ConstantParameterField>(
    parameterSpec(mesh, type), mesh.getNumElements(), std::move(values));
}

ES::VXd ElasticModelFactory::initializeDefaultElasticParams(
  const SimulationMesh &mesh,
  DeformationModelElasticMaterial elastic,
  int numElasticParams)
{
  const int nele = mesh.getNumElements();
  ES::VXd elasticParams(static_cast<Eigen::Index>(nele) * numElasticParams);

  if (numElasticParams > 0) {
    elasticParams.setZero();
    if (numElasticParams == 2 &&
        (elastic == DeformationModelElasticMaterial::STABLE_NEO ||
         elastic == DeformationModelElasticMaterial::STVK ||
         elastic == DeformationModelElasticMaterial::LINEAR)) {
      for (int ei = 0; ei < nele; ei++) {
        const auto *mat = dynamic_cast<const SimulationMeshENuMaterial *>(mesh.getElementMaterial(ei, 0));
        if (!mat)
          throw std::runtime_error("ElasticModelFactory::initializeDefaultElasticParams: ENu elastic model requires SimulationMeshENuMaterial.");
        elasticParams.segment<2>(ei * 2) << mat->getE(), mat->getNu();
      }
    }
    else if (numElasticParams == 3 &&
        (elastic == DeformationModelElasticMaterial::STVK_VOL ||
         elastic == DeformationModelElasticMaterial::INV_STVK ||
         elastic == DeformationModelElasticMaterial::VOLUME)) {
      for (int ei = 0; ei < nele; ei++) {
        const auto *mat = dynamic_cast<const SimulationMeshENuMaterial *>(mesh.getElementMaterial(ei, 0));
        if (!mat)
          throw std::runtime_error("ElasticModelFactory::initializeDefaultElasticParams: ENu elastic model requires SimulationMeshENuMaterial.");
        elasticParams.segment<3>(ei * 3) << mat->getE(), mat->getNu(), mat->getCompressionRatio();
      }
    }
    else if (elastic == DeformationModelElasticMaterial::KOITER_STVK) {
      for (int ei = 0; ei < nele; ei++) {
        const auto *mat = dynamic_cast<const SimulationMeshENuhMaterial *>(mesh.getElementMaterial(ei, 0));
        if (!mat)
          throw std::runtime_error("ElasticModelFactory::initializeDefaultElasticParams: KOITER_STVK requires SimulationMeshENuhMaterial.");
        elasticParams.segment<5>(ei * 5) << mat->getE(), mat->getNu(), mat->getE(), mat->getNu(), mat->geth();
      }
    }
    else if (elastic == DeformationModelElasticMaterial::KOITER_FABRIC) {
      for (int ei = 0; ei < nele; ei++) {
        const auto *mat = dynamic_cast<const SimulationMeshENuhMaterial *>(mesh.getElementMaterial(ei, 0));
        if (!mat)
          throw std::runtime_error("ElasticModelFactory::initializeDefaultElasticParams: KOITER_FABRIC requires SimulationMeshENuhMaterial.");
        elasticParams.segment<12>(ei * 12) << 1.0, 1.0, 1.0, 1.0,
          1.0, 1.0, 1.0,
          1000.0, 1000.0, 1000.0,
          1.0, mat->geth();
      }
    }
    else if (elastic == DeformationModelElasticMaterial::HILL_STABLE_NEO ||
        elastic == DeformationModelElasticMaterial::HILL_STVK ||
        elastic == DeformationModelElasticMaterial::HILL_STVK_VOL) {
      elasticParams.setOnes();
    }
  }

  return elasticParams;
}

}  // namespace pgo::SolidDeformationModel
