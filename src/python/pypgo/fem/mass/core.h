#pragma once

#include "mass/volumeDensityField.h"
#include "mass/shellArealDensityField.h"
#include "../../energy/core.h"

#include <memory>
#include <vector>

namespace pgo
{

class PyVolumeDensity
{
public:
  explicit PyVolumeDensity(
    std::shared_ptr<SolidDeformationModel::VolumeDensityField> field)
    : field_(std::move(field)) {}

  const SolidDeformationModel::VolumeDensityField &get() const { return *field_; }

private:
  std::shared_ptr<SolidDeformationModel::VolumeDensityField> field_;
};

std::shared_ptr<PyVolumeDensity> make_constant_volume_density(double density);
std::shared_ptr<PyVolumeDensity> make_elementwise_volume_density(
  const std::vector<double> &densities);

class PyShellArealDensity
{
public:
  explicit PyShellArealDensity(
    std::shared_ptr<SolidDeformationModel::ShellArealDensityField> field)
    : field_(std::move(field)) {}

  const SolidDeformationModel::ShellArealDensityField &get() const { return *field_; }

protected:
  std::shared_ptr<SolidDeformationModel::ShellArealDensityField> field_;
};

std::shared_ptr<PyShellArealDensity> make_constant_shell_areal_density(
  double arealDensity);
std::shared_ptr<PyShellArealDensity> make_shell_areal_density_elementwise(
  const std::vector<double> &arealDensities);
std::shared_ptr<PyShellArealDensity> make_shell_areal_density_from_density_thickness(
  double density, double thickness);
std::shared_ptr<PyShellArealDensity> make_shell_areal_density_from_density_thickness(
  double density, const std::vector<double> &thickness);

std::shared_ptr<PyShellArealDensity> make_shell_areal_density_from_elastic_parameter(
  double scale, const PyMaterialParameterRef &parameter);

}  // namespace pgo
