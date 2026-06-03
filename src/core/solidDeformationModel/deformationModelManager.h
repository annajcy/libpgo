/*
author: Bohan Wang
copyright to USC, MIT, NUS
*/

#pragma once

#include "EigenDef.h"
#include "formulations/formulation.h"

#include <memory>

namespace pgo
{
namespace SolidDeformationModel
{
class SimulationMesh;
class DeformationModel;
class DeformationModelManagerImpl;
class ParameterField;
class OptimizableField;
class DofLayout;

enum class DeformationModelElasticMaterial
{
  STABLE_NEO,
  STVK_VOL,
  INV_STVK,
  LINEAR,
  VOLUME,

  HILL_STABLE_NEO,
  HILL_STVK_VOL,
  HILL_STVK,

  STVK,
  MOONEY_RIVLIN,

  KOITER_FABRIC,
  KOITER_STVK,
};

enum class DeformationModelPlasticMaterial
{
  VOLUMETRIC_DOF0 = 0,
  VOLUMETRIC_DOF3 = 1,
  VOLUMETRIC_DOF6 = 2,

  SHELL_FF_DOF0 = 3,
  SHELL_FF_DOF1 = 4,
};

class DeformationModelManager
{
public:
  DeformationModelManager(const SimulationMesh &simulationMesh,
    const Formulation &formulation,
    std::shared_ptr<OptimizableField> elasticField,
    std::shared_ptr<OptimizableField> plasticField,
    int enforceSPD = 1,
    const double *elementFiberDirections = nullptr,
    const double *vertexFiberDirections = nullptr);

  ~DeformationModelManager();
  void setEnforceSPD(int enable);
  void updateMeshRigidTransformation(const double R[9]);

  int getNumPlasticParameters() const;
  int getNumElasticParameters() const;
  EigenSupport::VXd getElasticParameterSnapshot() const;
  EigenSupport::VXd getPlasticParameterSnapshot() const;
  const SimulationMesh *getMesh() const;

  void setElementAlignedMatrix(int id, double R[9]);
  void getElementAlignedMatrix(int id, double R[9]) const;
  void getVertexAlignedMatrix(int id, double R[9]) const;

  const DeformationModel *getDeformationModel(int eleID) const;

  const ParameterField *getElasticParameterField() const;
  const ParameterField *getPlasticParameterField() const;

  std::unique_ptr<const DofLayout> createDofLayout() const;
  EigenSupport::VXd buildRestPosition() const;

protected:
  std::unique_ptr<DeformationModelManagerImpl> data;

private:
  void initBase(const SimulationMesh &simulationMesh,
    const double *elementFiberDirections,
    const double *vertexFiberDirections);

  void initImpl(DeformationModelPlasticMaterial plasticModelType,
    DeformationModelElasticMaterial elasticMaterialType,
    const Formulation &formulation);
};

}  // namespace SolidDeformationModel
}  // namespace pgo
