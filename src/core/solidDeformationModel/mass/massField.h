#pragma once

namespace pgo
{
namespace SolidDeformationModel
{

class SimulationMesh;
enum class SimulationMeshType;

// Mass-property field consumed by formulation mass / body-force assembly.
// Separate from the constitutive material (material/) and the optimizable
// parameter fields (material/fields/): it answers "how much mass per
// integration region". Unit-correct accessors live on the domain subclasses:
// VolumeMassField::volumeDensity [kg/m^3], ShellMassField::arealDensity [kg/m^2].
class MassField
{
public:
  virtual ~MassField() = default;

  virtual bool compatibleWith(SimulationMeshType meshType) const = 0;

  // Throws std::invalid_argument on mismatch (mesh type, element counts).
  virtual void validate(const SimulationMesh &mesh) const;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
