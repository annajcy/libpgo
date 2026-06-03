/*
author: Bohan Wang
copyright to USC, MIT, NUS
*/

#pragma once

#include "deformationModelManager.h"
#include "EigenSupport.h"

#include <memory>
#include <optional>

namespace pgo
{
namespace SolidDeformationModel
{

class OptimizableField;
class SimulationMesh;

enum class ElasticMaterialFieldType
{
  ELEMENTWISE,
};

enum class PlasticMaterialFieldType
{
  ELEMENTWISE,
};

struct ElasticFieldInit
{
  ElasticMaterialFieldType type = ElasticMaterialFieldType::ELEMENTWISE;
  std::optional<EigenSupport::VXd> values;
};

struct PlasticFieldInit
{
  PlasticMaterialFieldType type = PlasticMaterialFieldType::ELEMENTWISE;
  std::optional<EigenSupport::VXd> values;
};

class DeformationModelState
{
public:
  static std::shared_ptr<DeformationModelState> create(
    std::shared_ptr<const SimulationMesh> mesh,
    DeformationModelElasticMaterial elastic,
    ElasticFieldInit elasticField,
    DeformationModelPlasticMaterial plastic,
    PlasticFieldInit plasticField);

  const std::shared_ptr<const SimulationMesh> &mesh() const { return mesh_; }

  DeformationModelElasticMaterial elasticMaterial() const { return elasticMaterial_; }
  DeformationModelPlasticMaterial plasticMaterial() const { return plasticMaterial_; }

  std::shared_ptr<OptimizableField> elasticFieldPtr() const { return elasticField_; }
  std::shared_ptr<OptimizableField> plasticFieldPtr() const { return plasticField_; }

  OptimizableField &elasticField() { return *elasticField_; }
  OptimizableField &plasticField() { return *plasticField_; }
  const OptimizableField &elasticField() const { return *elasticField_; }
  const OptimizableField &plasticField() const { return *plasticField_; }

  EigenSupport::VXd elasticParameterSnapshot() const;
  EigenSupport::VXd plasticParameterSnapshot() const;

  void setElasticValues(EigenSupport::ConstRefVecXd values);
  void setPlasticValues(EigenSupport::ConstRefVecXd values);

private:
  DeformationModelState(
    std::shared_ptr<const SimulationMesh> mesh,
    DeformationModelElasticMaterial elastic,
    std::shared_ptr<OptimizableField> elasticField,
    DeformationModelPlasticMaterial plastic,
    std::shared_ptr<OptimizableField> plasticField);

  std::shared_ptr<const SimulationMesh> mesh_;
  DeformationModelElasticMaterial elasticMaterial_;
  std::shared_ptr<OptimizableField> elasticField_;
  DeformationModelPlasticMaterial plasticMaterial_;
  std::shared_ptr<OptimizableField> plasticField_;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
