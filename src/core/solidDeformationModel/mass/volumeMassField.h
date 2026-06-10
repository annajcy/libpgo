#pragma once

#include "massField.h"

#include "EigenSupport.h"

namespace pgo
{
namespace SolidDeformationModel
{

// Volumetric mass distribution; density in kg/m^3.
class VolumeMassField : public MassField
{
public:
  virtual double volumeDensity(int ele) const = 0;

  bool compatibleWith(SimulationMeshType meshType) const override;
};

class ConstantVolumeDensity : public VolumeMassField
{
public:
  explicit ConstantVolumeDensity(double density);

  double volumeDensity(int /*ele*/) const override { return density_; }

private:
  double density_ = 0.0;
};

class ElementwiseVolumeDensity : public VolumeMassField
{
public:
  explicit ElementwiseVolumeDensity(EigenSupport::VXd densities);

  void validate(const SimulationMesh &mesh) const override;
  double volumeDensity(int ele) const override { return densities_[ele]; }

private:
  EigenSupport::VXd densities_;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
