#include "shellMassField.h"

#include "simulation/simulationMesh.h"

#include <cassert>
#include <stdexcept>

namespace pgo
{
namespace SolidDeformationModel
{

bool ShellMassField::compatibleWith(SimulationMeshType meshType) const
{
  return meshType == SimulationMeshType::SHELL;
}

ConstantShellArealDensity::ConstantShellArealDensity(double arealDensity)
  : arealDensity_(arealDensity)
{
  if (!(arealDensity > 0.0)) {
    throw std::invalid_argument("ConstantShellArealDensity requires arealDensity > 0");
  }
}

ShellDensityThickness::ShellDensityThickness(double density, double thickness)
  : density_(density), constantThickness_(thickness)
{
  if (!(density > 0.0) || !(thickness > 0.0)) {
    throw std::invalid_argument("ShellDensityThickness requires positive density and thickness");
  }
}

ShellDensityThickness::ShellDensityThickness(double density, EigenSupport::VXd thickness)
  : density_(density), elementThickness_(std::move(thickness))
{
  if (!(density > 0.0) || elementThickness_.size() == 0 ||
      (elementThickness_.array() <= 0.0).any()) {
    throw std::invalid_argument("ShellDensityThickness requires positive density and thicknesses");
  }
}

void ShellDensityThickness::validate(const SimulationMesh &mesh) const
{
  ShellMassField::validate(mesh);
  if (elementThickness_.size() != 0 &&
      static_cast<int>(elementThickness_.size()) != mesh.getNumElements()) {
    throw std::invalid_argument("ShellDensityThickness thickness size does not match mesh element count");
  }
}

double ShellDensityThickness::arealDensity(
  int ele, MaterialParameterEvaluationView) const
{
  if (elementThickness_.size() != 0) {
    assert(ele >= 0 && ele < static_cast<int>(elementThickness_.size()));
    return density_ * elementThickness_[ele];
  }
  return density_ * constantThickness_;
}

}  // namespace SolidDeformationModel
}  // namespace pgo
