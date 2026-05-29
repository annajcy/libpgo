#include "elasticModelFactory.h"

#include "../simulationMesh.h"

#include "../elasticModelCombinedMaterial.h"
#include "../elasticModelHillTypeMaterial.h"
#include "../elasticModelInvariantBasedMaterial.h"
#include "../elasticModelStableNeoHookeanMaterial.h"
#include "../elasticModelVolumeMaterial.h"
#include "../invariantBasedMaterialStVK.h"
#include "../elasticModelLinearMaterial.h"
#include "../elasticModel3DSTVKMaterial.h"
#include "../elasticModel3DMooneyRivlin.h"

#include "../elasticModel2DFundamentalFormsFabric.h"
#include "../elasticModel2DFundamentalFormsSTVK.h"

#include "pgoLogging.h"

namespace pgo::SolidDeformationModel
{
namespace ES = EigenSupport;

int ElasticModelFactory::numParameters(const SimulationMesh &mesh, DeformationModelElasticMaterial type)
{
  switch (type) {
  case DeformationModelElasticMaterial::STABLE_NEO:
    return 2;  // mu, lambda
  case DeformationModelElasticMaterial::LINEAR:
    return 2;  // mu, lambda
  case DeformationModelElasticMaterial::STVK:
    return 2;  // mu, lambda
  case DeformationModelElasticMaterial::STVK_VOL:
  case DeformationModelElasticMaterial::INV_STVK:
  case DeformationModelElasticMaterial::VOLUME:
    return 3;  // E, nu, compressionRatio (invariant params)
  case DeformationModelElasticMaterial::HILL_STABLE_NEO:
  case DeformationModelElasticMaterial::HILL_STVK:
    return 3;  // base invariant params
  case DeformationModelElasticMaterial::HILL_STVK_VOL:
    return 3;
  case DeformationModelElasticMaterial::MOONEY_RIVLIN:
    // Mooney-Rivlin params depend on N and M from the payload — use element 0 to query
    if (mesh.getNumElements() > 0) {
      const auto *mat = dynamic_cast<const SimulationMeshMooneyRivlinMaterial *>(mesh.getElementMaterial(0, 0));
      if (mat)
        return mat->getN() + mat->getM();
    }
    return 0;
  case DeformationModelElasticMaterial::KOITER_FABRIC:
    return 2;  // dir0.x, dir0.y (or similar)
  case DeformationModelElasticMaterial::KOITER_STVK:
    return 5;  // E1, nu1, E2, nu2, h
  default:
    return 0;
  }
}

ElasticModelResult ElasticModelFactory::create(
  const SimulationMesh &mesh,
  int ele,
  DeformationModelElasticMaterial type,
  const double *fiberDirection)
{
  ElasticModelResult result;

  if (type == DeformationModelElasticMaterial::HILL_STABLE_NEO) {
    const auto *mat = dynamic_cast<const SimulationMeshENuMaterial *>(mesh.getElementMaterial(ele, 0));
    PGO_ALOG(mat != nullptr);

    double mu = mat->getMuLame();
    double lambda = mat->getLambdaLame();

    result.stableNeo = new ElasticModelStableNeoHookeanMaterial(mu, lambda);

    const auto *hillMat = dynamic_cast<const SimulationMeshHillMaterial *>(mesh.getElementMaterial(ele, 1));
    PGO_ALOG(hillMat != nullptr);
    double Eact = hillMat->getEact();
    double gamma = hillMat->getGamma();
    double lo = hillMat->getLo();

    result.hill = new ElasticModelHillTypeMaterial(gamma, Eact, lo, fiberDirection);
    result.combined2 = new ElasticModelCombinedMaterial<2>(result.stableNeo, result.hill);
    result.elementMaterial = result.combined2;
  }
  else if (type == DeformationModelElasticMaterial::LINEAR) {
    const auto *mat = dynamic_cast<const SimulationMeshENuMaterial *>(mesh.getElementMaterial(ele, 0));
    PGO_ALOG(mat != nullptr);

    double mu = mat->getMuLame();
    double lambda = mat->getLambdaLame();

    result.linear = new ElasticModelLinearMaterial(mu, lambda);
    result.elementMaterial = result.linear;
  }
  else if (type == DeformationModelElasticMaterial::HILL_STVK) {
    const auto *mat = dynamic_cast<const SimulationMeshENuMaterial *>(mesh.getElementMaterial(ele, 0));
    PGO_ALOG(mat != nullptr);

    double E = mat->getE();
    double nu = mat->getNu();
    double compressionRatio = mat->getCompressionRatio();

    result.invariantModel = new InvariantBasedMaterialStVK(E, nu, compressionRatio);
    result.invariantBased = new ElasticModelInvariantBasedMaterial(result.invariantModel);

    const auto *hillMat = dynamic_cast<const SimulationMeshHillMaterial *>(mesh.getElementMaterial(ele, 1));
    PGO_ALOG(hillMat != nullptr);
    double Eact = hillMat->getEact();
    double gamma = hillMat->getGamma();
    double lo = hillMat->getLo();

    result.hill = new ElasticModelHillTypeMaterial(gamma, Eact, lo, fiberDirection);
    result.combined2 = new ElasticModelCombinedMaterial<2>(result.invariantBased, result.hill);
    result.elementMaterial = result.combined2;
  }
  else if (type == DeformationModelElasticMaterial::HILL_STVK_VOL) {
    const auto *mat = dynamic_cast<const SimulationMeshENuMaterial *>(mesh.getElementMaterial(ele, 0));
    PGO_ALOG(mat != nullptr);

    double E = mat->getE();
    double nu = mat->getNu();
    double compressionRatio = mat->getCompressionRatio();

    result.invariantModel = new InvariantBasedMaterialStVK(E, nu, compressionRatio);
    result.invariantBased = new ElasticModelInvariantBasedMaterial(result.invariantModel);

    const auto *hillMat = dynamic_cast<const SimulationMeshHillMaterial *>(mesh.getElementMaterial(ele, 1));
    PGO_ALOG(hillMat != nullptr);
    double Eact = hillMat->getEact();
    double gamma = hillMat->getGamma();
    double lo = hillMat->getLo();

    result.hill = new ElasticModelHillTypeMaterial(gamma, Eact, lo, fiberDirection);
    result.volume = new ElasticModelVolumeMaterial(compressionRatio);
    result.combined3 = new ElasticModelCombinedMaterial<3>(result.invariantBased, result.hill, result.volume);
    result.elementMaterial = result.combined3;
  }
  else if (type == DeformationModelElasticMaterial::STABLE_NEO) {
    const auto *mat = dynamic_cast<const SimulationMeshENuMaterial *>(mesh.getElementMaterial(ele, 0));
    PGO_ALOG(mat != nullptr);

    double mu = mat->getMuLame();
    double lambda = mat->getLambdaLame();

    result.stableNeo = new ElasticModelStableNeoHookeanMaterial(mu, lambda);
    result.elementMaterial = result.stableNeo;
  }
  else if (type == DeformationModelElasticMaterial::INV_STVK) {
    const auto *mat = dynamic_cast<const SimulationMeshENuMaterial *>(mesh.getElementMaterial(ele, 0));
    PGO_ALOG(mat != nullptr);

    double E = mat->getE();
    double nu = mat->getNu();
    double compressionRatio = mat->getCompressionRatio();

    result.invariantModel = new InvariantBasedMaterialStVK(E, nu, compressionRatio);
    result.invariantBased = new ElasticModelInvariantBasedMaterial(result.invariantModel);
    result.elementMaterial = result.invariantBased;
  }
  else if (type == DeformationModelElasticMaterial::STVK_VOL) {
    const auto *mat = dynamic_cast<const SimulationMeshENuMaterial *>(mesh.getElementMaterial(ele, 0));
    PGO_ALOG(mat != nullptr);

    double E = mat->getE();
    double nu = mat->getNu();
    double compressionRatio = mat->getCompressionRatio();

    result.invariantModel = new InvariantBasedMaterialStVK(E, nu, compressionRatio);
    result.invariantBased = new ElasticModelInvariantBasedMaterial(result.invariantModel);
    result.volume = new ElasticModelVolumeMaterial(compressionRatio);
    result.combined2 = new ElasticModelCombinedMaterial<2>(result.invariantBased, result.volume);
    result.elementMaterial = result.combined2;
  }
  else if (type == DeformationModelElasticMaterial::VOLUME) {
    const auto *mat = dynamic_cast<const SimulationMeshENuMaterial *>(mesh.getElementMaterial(ele, 0));
    PGO_ALOG(mat != nullptr);

    double compressionRatio = mat->getCompressionRatio();
    result.volume = new ElasticModelVolumeMaterial(compressionRatio);
    result.elementMaterial = result.volume;
  }
  else if (type == DeformationModelElasticMaterial::STVK) {
    const auto *mat = dynamic_cast<const SimulationMeshENuMaterial *>(mesh.getElementMaterial(ele, 0));
    PGO_ALOG(mat != nullptr);

    double mu = mat->getMuLame();
    double lambda = mat->getLambdaLame();

    result.stvk = new ElasticModel3DSTVKMaterial(mu, lambda);
    result.elementMaterial = result.stvk;
  }
  else if (type == DeformationModelElasticMaterial::MOONEY_RIVLIN) {
    const auto *mat = dynamic_cast<const SimulationMeshMooneyRivlinMaterial *>(mesh.getElementMaterial(ele, 0));
    PGO_ALOG(mat != nullptr);

    int N = mat->getN();
    int M = mat->getM();

    result.mooneyRivlin = new ElasticModel3DMooneyRivlin(N, mat->getC(), M, mat->getD());
    result.elementMaterial = result.mooneyRivlin;
  }
  else if (type == DeformationModelElasticMaterial::KOITER_FABRIC) {
    ES::V2d dir0(1, 0), dir1(0, 1);
    result.shellFabric = new ElasticModel2DFundamentalFormsFabric(dir0, dir1);
    result.elementMaterial = result.shellFabric;
  }
  else if (type == DeformationModelElasticMaterial::KOITER_STVK) {
    result.shellSTVK = new ElasticModel2DFundamentalFormsSTVK;
    result.elementMaterial = result.shellSTVK;
  }
  else {
    throw std::runtime_error("unknown elastic model");
  }

  return result;
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
        elasticParams.segment<5>(ei * 5) << mat->getE(), mat->getNu(), mat->getE(), mat->getNu(), mat->geth();
      }
    }
  }

  return elasticParams;
}

}  // namespace pgo::SolidDeformationModel
