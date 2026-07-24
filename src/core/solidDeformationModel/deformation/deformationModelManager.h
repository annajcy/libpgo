/*
author: Bohan Wang
copyright to USC, MIT, NUS
*/

#pragma once

#include "EigenDef.h"
#include "formulations/formulation/formulation.h"
#include "material/materialTypes.h"
#include "material/fields/materialFrameField.h"

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
    int enforceSPD = 1);

  DeformationModelManager(std::shared_ptr<const SimulationMesh> mesh,
    DeformationModelElasticMaterial elasticMaterial,
    DeformationModelPlasticMaterial plasticModel,
    const Formulation &formulation,
    int enforceSPD,
    std::shared_ptr<const MaterialFrameField> materialFrames);

  ~DeformationModelManager();
  void setEnforceSPD(int enable);

  int getNumPlasticParameters() const;
  int getNumElasticParameters() const;
  const SimulationMesh *getMesh() const;

  const MaterialFrameField &materialFrameField() const;
  std::shared_ptr<const MaterialFrameField> materialFrameFieldPtr() const;
  MaterialFrame materialToReferenceFrame(
    int elementId, int quadratureId = 0) const;

  const DeformationModel *getDeformationModel(int eleID) const;

protected:
  std::unique_ptr<DeformationModelManagerImpl> data;

private:
  void initImpl(DeformationModelPlasticMaterial plasticModelType,
    DeformationModelElasticMaterial elasticMaterialType,
    const Formulation &formulation);
};

}  // namespace SolidDeformationModel
}  // namespace pgo
