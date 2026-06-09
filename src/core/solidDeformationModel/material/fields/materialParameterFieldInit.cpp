/*
author: Bohan Wang
copyright to USC, MIT, NUS
*/

#include "material/fields/materialParameterFieldInit.h"

#include "material/elastic/elasticModelFactory.h"
#include "material/plastic/plasticModelFactory.h"
#include "simulation/simulationMesh.h"

#include <stdexcept>

namespace pgo
{
namespace SolidDeformationModel
{
namespace ES = EigenSupport;

std::shared_ptr<OptimizableField> createElasticParameterField(
  const SimulationMesh &mesh,
  DeformationModelElasticMaterial elastic,
  ElasticFieldInit fieldInit)
{
  switch (fieldInit.type) {
  case ElasticMaterialFieldType::ELEMENTWISE:
    return fieldInit.values ?
      ElasticModelFactory::createElementwiseField(mesh, elastic, ES::VXd(*fieldInit.values)) :
      ElasticModelFactory::createDefaultElementwiseField(mesh, elastic);
  case ElasticMaterialFieldType::CONSTANT:
    return fieldInit.values ?
      ElasticModelFactory::createConstantField(mesh, elastic, ES::VXd(*fieldInit.values)) :
      ElasticModelFactory::createDefaultConstantField(mesh, elastic);
  default:
    throw std::invalid_argument("createElasticParameterField: unsupported elastic field type.");
  }
}

std::shared_ptr<OptimizableField> createPlasticParameterField(
  const SimulationMesh &mesh,
  DeformationModelPlasticMaterial plastic,
  PlasticFieldInit fieldInit)
{
  switch (fieldInit.type) {
  case PlasticMaterialFieldType::ELEMENTWISE:
    return fieldInit.values ?
      PlasticModelFactory::createElementwiseField(mesh, plastic, ES::VXd(*fieldInit.values)) :
      PlasticModelFactory::createDefaultElementwiseField(mesh, plastic);
  case PlasticMaterialFieldType::CONSTANT:
    return fieldInit.values ?
      PlasticModelFactory::createConstantField(mesh, plastic, ES::VXd(*fieldInit.values)) :
      PlasticModelFactory::createDefaultConstantField(mesh, plastic);
  default:
    throw std::invalid_argument("createPlasticParameterField: unsupported plastic field type.");
  }
}

}  // namespace SolidDeformationModel
}  // namespace pgo
