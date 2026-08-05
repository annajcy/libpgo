#pragma once

#include "material/runtime/materialState.h"
#include "material/parameterization/materialParameterization.h"
#include "material/data/materialParameterData.h"
#include "material/frame/materialFrameField.h"

#include <memory>

namespace pgo::SolidDeformationModel
{

class SimulationMesh;

/// Material data bound to a mesh for one simulation.  The mesh itself remains
/// geometry-only; structural material fields and frames live here.
class MaterialAssignment final
{
public:
  /// Bind one mesh, one structural parameterization and one projected data
  /// snapshot. Initial values are retained only as an immutable state factory
  /// input; evaluations must receive a MaterialState explicitly.
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
  const MaterialState &initialMaterialState() const { return initialMaterialState_; }
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
  MaterialState initialMaterialState_;
  std::shared_ptr<const MaterialFrameField> materialFrames_;
};

}  // namespace pgo::SolidDeformationModel
