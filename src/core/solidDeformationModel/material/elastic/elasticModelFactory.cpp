#include "material/elastic/elasticModelFactory.h"

#include "simulation/simulationMesh.h"
#include "material/elastic/elasticModel.h"
#include "material/elastic/elasticModelLinearMaterial.h"
#include "material/elastic/elasticModelStableNeoHookeanMaterial.h"
#include "material/elastic/elasticModel3DSTVKMaterial.h"
#include "material/elastic/elasticModelInvariantBasedMaterial.h"
#include "material/elastic/invariantBasedMaterialStVK.h"
#include "material/elastic/elasticModelVolumeMaterial.h"
#include "material/elastic/elasticModelCombinedMaterial.h"
#include "material/elastic/elasticModelHillTypeMaterial.h"
#include "material/elastic/elasticModel2DFundamentalFormsFabric.h"
#include "material/elastic/elasticModel2DFundamentalFormsSTVK.h"
#include "material/elastic/elasticModel3DMooneyRivlin.h"

#include <stdexcept>

namespace pgo::SolidDeformationModel
{
namespace ES = EigenSupport;

std::unique_ptr<ElasticModel> ElasticModelFactory::create(
  const SimulationMesh &mesh,
  int ele,
  DeformationModelElasticMaterial type,
  const MaterialFrame &materialToReference)
{
  validateMaterialFrame(materialToReference);
  const ES::V3d primaryAxis = materialToReference.col(0);
  const auto *mat = mesh.getElementMaterial(ele, 0);
  const SimulationMeshMaterial *auxMat = nullptr;
  if (mesh.getElementNumMaterials(ele) > 1)
    auxMat = mesh.getElementMaterial(ele, 1);

  // --- ENuMaterial-based types ---
  if (const auto *enu = dynamic_cast<const SimulationMeshENuMaterial *>(mat)) {
    const double mu = enu->getMuLame();
    const double lam = enu->getLambdaLame();
    const double E = enu->getE();
    const double nu = enu->getNu();
    const double J = enu->getCompressionRatio();

    switch (type) {
    case DeformationModelElasticMaterial::LINEAR:
      return std::make_unique<ElasticModelLinearMaterial>(mu, lam);
    case DeformationModelElasticMaterial::STABLE_NEO:
      return std::make_unique<ElasticModelStableNeoHookeanMaterial>(mu, lam);
    case DeformationModelElasticMaterial::STVK:
      return std::make_unique<ElasticModel3DSTVKMaterial>(mu, lam);
    case DeformationModelElasticMaterial::INV_STVK:
      return std::make_unique<ElasticModelInvariantBasedMaterial>(
        std::make_unique<InvariantBasedMaterialStVK>(E, nu, J));
    case DeformationModelElasticMaterial::VOLUME:
      return std::make_unique<ElasticModelVolumeMaterial>(J);
    case DeformationModelElasticMaterial::STVK_VOL:
      return std::make_unique<ElasticModelCombinedMaterial<2>>(
        std::make_unique<ElasticModelInvariantBasedMaterial>(
          std::make_unique<InvariantBasedMaterialStVK>(E, nu, J)),
        std::make_unique<ElasticModelVolumeMaterial>(J));
    case DeformationModelElasticMaterial::HILL_STABLE_NEO: {
      const auto *hill = dynamic_cast<const SimulationMeshHillMaterial *>(auxMat);
      if (!hill) throw std::invalid_argument("HILL_STABLE_NEO requires auxMat of type SimulationMeshHillMaterial");
      return std::make_unique<ElasticModelCombinedMaterial<2>>(
        std::make_unique<ElasticModelStableNeoHookeanMaterial>(mu, lam),
        std::make_unique<ElasticModelHillTypeMaterial>(hill->getGamma(), hill->getEact(), hill->getLo(), primaryAxis));
    }
    case DeformationModelElasticMaterial::HILL_STVK: {
      const auto *hill = dynamic_cast<const SimulationMeshHillMaterial *>(auxMat);
      if (!hill) throw std::invalid_argument("HILL_STVK requires auxMat of type SimulationMeshHillMaterial");
      return std::make_unique<ElasticModelCombinedMaterial<2>>(
        std::make_unique<ElasticModelInvariantBasedMaterial>(
          std::make_unique<InvariantBasedMaterialStVK>(E, nu, J)),
        std::make_unique<ElasticModelHillTypeMaterial>(hill->getGamma(), hill->getEact(), hill->getLo(), primaryAxis));
    }
    case DeformationModelElasticMaterial::HILL_STVK_VOL: {
      const auto *hill = dynamic_cast<const SimulationMeshHillMaterial *>(auxMat);
      if (!hill) throw std::invalid_argument("HILL_STVK_VOL requires auxMat of type SimulationMeshHillMaterial");
      return std::make_unique<ElasticModelCombinedMaterial<3>>(
        std::make_unique<ElasticModelInvariantBasedMaterial>(
          std::make_unique<InvariantBasedMaterialStVK>(E, nu, J)),
        std::make_unique<ElasticModelHillTypeMaterial>(hill->getGamma(), hill->getEact(), hill->getLo(), primaryAxis),
        std::make_unique<ElasticModelVolumeMaterial>(J));
    }
    case DeformationModelElasticMaterial::KOITER_FABRIC: {
      ES::V2d dir0(1, 0), dir1(0, 1);
      return std::make_unique<ElasticModel2DFundamentalFormsFabric>(dir0, dir1);
    }
    case DeformationModelElasticMaterial::KOITER_STVK:
      return std::make_unique<ElasticModel2DFundamentalFormsSTVK>();
    default:
      break;
    }
  }

  // --- MooneyRivlinMaterial ---
  if (const auto *mr = dynamic_cast<const SimulationMeshMooneyRivlinMaterial *>(mat)) {
    if (type == DeformationModelElasticMaterial::MOONEY_RIVLIN)
      return std::make_unique<ElasticModel3DMooneyRivlin>(mr->getN(), mr->getC(), mr->getM(), mr->getD());
  }

  throw std::invalid_argument(
    "ElasticModelFactory::create: unsupported combination of material class and elastic model type");
}

MaterialFrameRequirement ElasticModelFactory::materialFrameRequirement(
  DeformationModelElasticMaterial type)
{
  switch (type) {
  case DeformationModelElasticMaterial::HILL_STABLE_NEO:
  case DeformationModelElasticMaterial::HILL_STVK:
  case DeformationModelElasticMaterial::HILL_STVK_VOL:
    return MaterialFrameRequirement::PrimaryAxis;
  default:
    return MaterialFrameRequirement::None;
  }
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
  case DeformationModelElasticMaterial::INV_STVK:
    return "inv_stvk";
  case DeformationModelElasticMaterial::VOLUME:
    return "volume";
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
  if (modelId == "inv_stvk") return DeformationModelElasticMaterial::INV_STVK;
  if (modelId == "volume") return DeformationModelElasticMaterial::VOLUME;
  if (modelId == "linear") return DeformationModelElasticMaterial::LINEAR;
  if (modelId == "mooney_rivlin") return DeformationModelElasticMaterial::MOONEY_RIVLIN;
  if (modelId == "koiter_stvk") return DeformationModelElasticMaterial::KOITER_STVK;
  if (modelId == "koiter_fabric") return DeformationModelElasticMaterial::KOITER_FABRIC;
  if (modelId == "hill_stable_neo") return DeformationModelElasticMaterial::HILL_STABLE_NEO;
  if (modelId == "hill_stvk") return DeformationModelElasticMaterial::HILL_STVK;
  if (modelId == "hill_stvk_vol") return DeformationModelElasticMaterial::HILL_STVK_VOL;
  throw std::invalid_argument("ElasticModelFactory::materialFromModelId: unknown elastic model id: " + modelId);
}

MaterialParameterSpec ElasticModelFactory::parameterSpec(
  const SimulationMesh &mesh,
  DeformationModelElasticMaterial type)
{
  MaterialParameterSpec spec;
  spec.modelId = modelId(type);
  const int numChannels =
    create(mesh, 0, type, MaterialFrame::Identity())->getNumParameters();
  if (type == DeformationModelElasticMaterial::KOITER_STVK && numChannels == 5) {
    spec.channelNames = { "E_membrane", "nu_membrane", "E_bending", "nu_bending", "thickness" };
  }
  else if (type == DeformationModelElasticMaterial::KOITER_FABRIC && numChannels == 12) {
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
      numChannels == 1) {
    spec.channelNames = { "activation" };
  }
  else {
    for (int i = 0; i < numChannels; i++)
      spec.channelNames.push_back("parameter_" + std::to_string(i));
  }
  return spec;
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
    if (elastic == DeformationModelElasticMaterial::KOITER_STVK) {
      for (int ei = 0; ei < nele; ei++) {
        const auto *mat = dynamic_cast<const SimulationMeshENuhMaterial *>(mesh.getElementMaterial(ei, 0));
        if (!mat)
          throw std::runtime_error("ElasticModelFactory::initializeDefaultElasticParams: KOITER_STVK requires SimulationMeshENuhMaterial.");
        elasticParams.segment<5>(static_cast<Eigen::Index>(ei) * 5) << mat->getE(), mat->getNu(), mat->getE(), mat->getNu(), mat->geth();
      }
    }
    else if (elastic == DeformationModelElasticMaterial::KOITER_FABRIC) {
      for (int ei = 0; ei < nele; ei++) {
        const auto *mat = dynamic_cast<const SimulationMeshENuhMaterial *>(mesh.getElementMaterial(ei, 0));
        if (!mat)
          throw std::runtime_error("ElasticModelFactory::initializeDefaultElasticParams: KOITER_FABRIC requires SimulationMeshENuhMaterial.");
        elasticParams.segment<12>(static_cast<Eigen::Index>(ei) * 12) << 1.0, 1.0, 1.0, 1.0,
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
