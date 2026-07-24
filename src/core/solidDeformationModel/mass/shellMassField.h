#pragma once

#include "massField.h"
#include "material/fields/materialParameters.h"

#include "EigenSupport.h"

namespace pgo
{
namespace SolidDeformationModel
{

// Shell mass distribution; areal density rho*h in kg/m^2.
class ShellMassField : public MassField
{
public:
  virtual double arealDensity(int ele, MaterialStateView state) const = 0;

  bool compatibleWith(SimulationMeshType meshType) const override;
};

class ConstantShellArealDensity : public ShellMassField
{
public:
  explicit ConstantShellArealDensity(double arealDensity);

  double arealDensity(int /*ele*/, MaterialStateView /*state*/) const override
  {
    return arealDensity_;
  }

private:
  double arealDensity_ = 0.0;
};

// rho * h(e) with a fixed thickness (constant or per element).
class ShellDensityThickness : public ShellMassField
{
public:
  ShellDensityThickness(double density, double thickness);
  ShellDensityThickness(double density, EigenSupport::VXd thickness);

  void validate(const SimulationMesh &mesh) const override;
  double arealDensity(int ele, MaterialStateView state) const override;

private:
  double density_ = 0.0;
  double constantThickness_ = 0.0;
  EigenSupport::VXd elementThickness_;  // empty when constant
};

}  // namespace SolidDeformationModel
}  // namespace pgo
