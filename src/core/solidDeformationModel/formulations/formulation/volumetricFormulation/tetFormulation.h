#pragma once

#include "volumetricFormulation.h"

namespace pgo
{
namespace SolidDeformationModel
{

class TetFormulation : public VolumetricFormulation
{
public:
  using VolumetricFormulation::VolumetricFormulation;
  SimulationMeshType compatibleMeshType() const override;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
