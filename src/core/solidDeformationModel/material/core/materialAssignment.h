#pragma once

#include "optimizableParameters.h"
#include "materialParameterization.h"
#include "materialParameterData.h"
#include "materialFrameField.h"
#include "material/elastic/elasticModel.h"
#include "material/plastic/plasticModel.h"

#include <memory>

namespace pgo::SolidDeformationModel
{

class SimulationMesh;

/// Material data bound to a mesh for one simulation.  The mesh itself remains
/// geometry-only; all fixed values, optimizer values and frames live here.
class MaterialAssignment final
{
public:
  /// Bind one mesh, one structural parameterization and one projected data
  /// snapshot.  Construction initializes the runtime optimizable snapshot
  /// from initialOptimizableValues.
  MaterialAssignment(
    std::shared_ptr<const SimulationMesh> mesh,
    std::shared_ptr<const MaterialParameterization> parameterization,
    std::shared_ptr<const MaterialParameterData> parameterData,
    std::shared_ptr<const MaterialFrameField> materialFrames);

  const std::shared_ptr<const SimulationMesh> &mesh() const { return mesh_; }
  const std::shared_ptr<const ElasticModelDefinition> &elasticDefinition() const
  {
    return parameterization_->elastic().definition();
  }
  const std::shared_ptr<const PlasticModelDefinition> &plasticDefinition() const
  {
    return parameterization_->plastic().definition();
  }
  const std::shared_ptr<const FixedParameterField> &elasticFixed() const
  {
    return parameterization_->elastic().fixedField();
  }
  const std::shared_ptr<const FixedParameterField> &plasticFixed() const
  {
    return parameterization_->plastic().fixedField();
  }
  const std::shared_ptr<OptimizableParameters> &optimizableParameters() const { return optimizableParameters_; }
  const std::shared_ptr<const MaterialFrameField> &materialFrames() const { return materialFrames_; }
  const std::shared_ptr<const MaterialParameterization> &parameterization() const
  {
    return parameterization_;
  }
  const std::shared_ptr<const MaterialParameterData> &parameterData() const
  {
    return parameterData_;
  }

private:
  std::shared_ptr<const SimulationMesh> mesh_;
  std::shared_ptr<const MaterialParameterization> parameterization_;
  std::shared_ptr<const MaterialParameterData> parameterData_;
  std::shared_ptr<OptimizableParameters> optimizableParameters_;
  std::shared_ptr<const MaterialFrameField> materialFrames_;
};

}  // namespace pgo::SolidDeformationModel
