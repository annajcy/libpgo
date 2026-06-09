/*
author: Bohan Wang
copyright to USC, MIT, NUS
*/

#pragma once

#include "EigenDef.h"
#include "formulations/formulation.h"
#include "material/materialTypes.h"

#include <memory>

namespace pgo
{
namespace SolidDeformationModel
{
class SimulationMesh;
class DeformationModel;
class DeformationModelManagerImpl;

class DeformationModelManager
{
public:
  DeformationModelManager(std::shared_ptr<const SimulationMesh> mesh,
    DeformationModelElasticMaterial elasticMaterial,
    DeformationModelPlasticMaterial plasticModel,
    const Formulation &formulation,
    int enforceSPD = 1,
    const double *elementFiberDirections = nullptr,
    const double *vertexFiberDirections = nullptr);

  ~DeformationModelManager();
  void setEnforceSPD(int enable);
  void updateMeshRigidTransformation(const double R[9]);

  int getNumPlasticParameters() const;
  int getNumElasticParameters() const;
  const SimulationMesh *getMesh() const;

  void setElementAlignedMatrix(int id, double R[9]);
  void getElementAlignedMatrix(int id, double R[9]) const;
  void getVertexAlignedMatrix(int id, double R[9]) const;

  const DeformationModel *getDeformationModel(int eleID) const;

protected:
  std::unique_ptr<DeformationModelManagerImpl> data;

private:
  void initFiber(const double *elementFiberDirections,
    const double *vertexFiberDirections);

  void initImpl(DeformationModelPlasticMaterial plasticModelType,
    DeformationModelElasticMaterial elasticMaterialType,
    const Formulation &formulation);
};

}  // namespace SolidDeformationModel
}  // namespace pgo
