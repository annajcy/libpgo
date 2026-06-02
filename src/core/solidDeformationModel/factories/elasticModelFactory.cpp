#include "elasticModelFactory.h"

#include "../simulationMesh.h"
#include "../elasticModel.h"

namespace pgo::SolidDeformationModel
{
namespace ES = EigenSupport;

std::unique_ptr<ElasticModel> ElasticModelFactory::create(
  const SimulationMesh &mesh,
  int ele,
  DeformationModelElasticMaterial type,
  const double *fiberDirection)
{
  return mesh.getElementMaterial(ele, 0)->createElasticModel(
    type, fiberDirection, mesh.getElementMaterial(ele, 1));
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
