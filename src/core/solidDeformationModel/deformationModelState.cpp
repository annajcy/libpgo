/*
author: Bohan Wang
copyright to USC, MIT, NUS
*/

#include "deformationModelState.h"

#include "factories/elasticModelFactory.h"
#include "factories/plasticModelFactory.h"
#include "formulations/parameters/parameterField.h"
#include "simulationMesh.h"

#include <stdexcept>
#include <string>
#include <utility>

namespace pgo
{
namespace SolidDeformationModel
{
namespace ES = EigenSupport;

namespace
{

ES::VXd snapshot(const OptimizableField &field)
{
  const auto *layout = field.dofLayout();
  const int n = layout ? layout->numGlobalDofs() : 0;
  ES::VXd out(n);
  if (n > 0)
    out = Eigen::Map<const ES::VXd>(field.globalData(), n);
  return out;
}

void setFieldValues(OptimizableField &field, ES::ConstRefVecXd values, const char *name)
{
  const auto *layout = field.dofLayout();
  const int expected = layout ? layout->numGlobalDofs() : 0;
  if (values.size() != expected)
    throw std::invalid_argument(std::string(name) + ": values size does not match field DOF count.");
  field.setGlobalData(values.data());
}

}  // namespace

std::shared_ptr<DeformationModelState> DeformationModelState::create(
  std::shared_ptr<const SimulationMesh> mesh,
  DeformationModelElasticMaterial elastic,
  ElasticFieldInit elasticField,
  DeformationModelPlasticMaterial plastic,
  PlasticFieldInit plasticField)
{
  if (!mesh)
    throw std::invalid_argument("DeformationModelState::create: mesh must be non-null.");

  std::shared_ptr<OptimizableField> elasticParamField;
  switch (elasticField.type) {
  case ElasticMaterialFieldType::ELEMENTWISE:
    elasticParamField = elasticField.values ?
      ElasticModelFactory::createElementwiseField(*mesh, elastic, ES::VXd(*elasticField.values)) :
      ElasticModelFactory::createDefaultElementwiseField(*mesh, elastic);
    break;
  case ElasticMaterialFieldType::CONSTANT:
    elasticParamField = elasticField.values ?
      ElasticModelFactory::createConstantField(*mesh, elastic, ES::VXd(*elasticField.values)) :
      ElasticModelFactory::createDefaultConstantField(*mesh, elastic);
    break;
  default:
    throw std::invalid_argument("DeformationModelState::create: unsupported elastic field type.");
  }

  std::shared_ptr<OptimizableField> plasticParamField;
  switch (plasticField.type) {
  case PlasticMaterialFieldType::ELEMENTWISE:
    plasticParamField = plasticField.values ?
      PlasticModelFactory::createElementwiseField(*mesh, plastic, ES::VXd(*plasticField.values)) :
      PlasticModelFactory::createDefaultElementwiseField(*mesh, plastic);
    break;
  case PlasticMaterialFieldType::CONSTANT:
    plasticParamField = plasticField.values ?
      PlasticModelFactory::createConstantField(*mesh, plastic, ES::VXd(*plasticField.values)) :
      PlasticModelFactory::createDefaultConstantField(*mesh, plastic);
    break;
  default:
    throw std::invalid_argument("DeformationModelState::create: unsupported plastic field type.");
  }

  return std::shared_ptr<DeformationModelState>(new DeformationModelState(
    std::move(mesh),
    elastic,
    std::move(elasticParamField),
    plastic,
    std::move(plasticParamField)));
}

DeformationModelState::DeformationModelState(
  std::shared_ptr<const SimulationMesh> mesh,
  DeformationModelElasticMaterial elastic,
  std::shared_ptr<OptimizableField> elasticField,
  DeformationModelPlasticMaterial plastic,
  std::shared_ptr<OptimizableField> plasticField):
  mesh_(std::move(mesh)),
  elasticMaterial_(elastic),
  elasticField_(std::move(elasticField)),
  plasticMaterial_(plastic),
  plasticField_(std::move(plasticField))
{
  if (!mesh_)
    throw std::invalid_argument("DeformationModelState: mesh must be non-null.");
  if (!elasticField_ || !plasticField_)
    throw std::invalid_argument("DeformationModelState: fields must be non-null.");
}

ES::VXd DeformationModelState::elasticParameterSnapshot() const
{
  return snapshot(*elasticField_);
}

ES::VXd DeformationModelState::plasticParameterSnapshot() const
{
  return snapshot(*plasticField_);
}

void DeformationModelState::setElasticValues(ES::ConstRefVecXd values)
{
  setFieldValues(*elasticField_, values, "DeformationModelState::setElasticValues");
}

void DeformationModelState::setPlasticValues(ES::ConstRefVecXd values)
{
  setFieldValues(*plasticField_, values, "DeformationModelState::setPlasticValues");
}

}  // namespace SolidDeformationModel
}  // namespace pgo
