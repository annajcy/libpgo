/*
author: Bohan Wang
copyright to USC, MIT, NUS
*/

#pragma once

#include "EigenDef.h"
#include "formulations/formulation/formulation.h"
#include "material/elastic/elasticModel.h"
#include "material/plastic/plasticModel.h"
#include "material/core/materialFrameField.h"

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
    std::shared_ptr<const ElasticModelConfig> elasticConfig,
    std::shared_ptr<const PlasticModelConfig> plasticConfig,
    const Formulation &formulation,
    bool projectHessianPSD = true);

  DeformationModelManager(std::shared_ptr<const SimulationMesh> mesh,
    std::shared_ptr<const ElasticModelConfig> elasticConfig,
    std::shared_ptr<const PlasticModelConfig> plasticConfig,
    const Formulation &formulation,
    bool projectHessianPSD,
    std::shared_ptr<const MaterialFrameField> materialFrames);

  ~DeformationModelManager();
  void setProjectHessianPSD(bool enable);

  int getNumPlasticParameters() const;
  int getNumElasticParameters() const;
  const SimulationMesh &getMesh() const;

  const MaterialFrameField &materialFrameField() const;
  std::shared_ptr<const MaterialFrameField> materialFrameFieldPtr() const;
  std::shared_ptr<const ElasticModelConfig> elasticModelConfig() const;
  std::shared_ptr<const PlasticModelConfig> plasticModelConfig() const;
  MaterialFrame materialToReferenceFrame(
    int elementId, int quadratureId = 0) const;

  const DeformationModel &getDeformationModel(int eleID) const;

protected:
  std::unique_ptr<DeformationModelManagerImpl> data;

private:
  void initImpl(const Formulation &formulation);
};

}  // namespace SolidDeformationModel
}  // namespace pgo
