/*
author: Bohan Wang
copyright to USC, MIT, NUS
*/

#pragma once

#include "EigenDef.h"
#include "formulations/formulation/formulation.h"
#include "material/frame/materialFrameField.h"
#include "material/model/elasticModelDefinition.h"
#include "material/model/plasticModelDefinition.h"
#include "material/runtime/materialBinding.h"
#include "material/runtime/materialAssignment.h"

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
  DeformationModelManager(
    std::shared_ptr<const SimulationMesh> mesh,
    std::shared_ptr<const MaterialBinding> materialBinding,
    const Formulation &formulation,
    bool projectHessianPSD = true);

  /// Preferred construction path: all material state is supplied by an
  /// assignment and model definitions never inspect the mesh.
  DeformationModelManager(
    std::shared_ptr<const MaterialAssignment> assignment,
    const Formulation &formulation,
    bool projectHessianPSD = true);

  ~DeformationModelManager();

  int getNumPlasticParameters() const;
  int getNumElasticParameters() const;
  const SimulationMesh &getMesh() const;

  const MaterialFrameField &materialFrameField() const;
  std::shared_ptr<const MaterialFrameField> materialFrameFieldPtr() const;
  std::shared_ptr<const ElasticModelDefinition> elasticModelDefinition() const;
  std::shared_ptr<const PlasticModelDefinition> plasticModelDefinition() const;
  std::shared_ptr<const MaterialBinding> materialBinding() const;
  std::shared_ptr<const MaterialAssignment> materialAssignment() const;
  MaterialFrame materialToReferenceFrame(
    int elementId, int quadratureId = 0) const;

  const DeformationModel &getDeformationModel(int eleID) const;

protected:
  std::unique_ptr<DeformationModelManagerImpl> data;

private:
  void initImpl(const Formulation &formulation, DeformationModelConstructionOptions options);
};

}  // namespace SolidDeformationModel
}  // namespace pgo
