#pragma once

#include "mass/volumeMassField.h"
#include "mass/shellMassField.h"
#include "../../energy/core.h"

#include <memory>
#include <vector>

namespace pgo
{

class PyVolumeMassField
{
public:
  explicit PyVolumeMassField(std::shared_ptr<SolidDeformationModel::VolumeMassField> field)
    : field_(std::move(field)) {}

  const SolidDeformationModel::VolumeMassField &get() const { return *field_; }

private:
  std::shared_ptr<SolidDeformationModel::VolumeMassField> field_;
};

std::shared_ptr<PyVolumeMassField> make_constant_volume_density(double density);
std::shared_ptr<PyVolumeMassField> make_elementwise_volume_density(const std::vector<double> &densities);

class PyShellMassField
{
public:
  explicit PyShellMassField(std::shared_ptr<SolidDeformationModel::ShellMassField> field)
    : field_(std::move(field)) {}

  const SolidDeformationModel::ShellMassField &get() const { return *field_; }

protected:
  std::shared_ptr<SolidDeformationModel::ShellMassField> field_;
};

std::shared_ptr<PyShellMassField> make_constant_shell_areal_density(double arealDensity);
std::shared_ptr<PyShellMassField> make_shell_density_thickness_constant(double density, double thickness);
std::shared_ptr<PyShellMassField> make_shell_density_thickness_elementwise(
  double density, const std::vector<double> &thickness);

std::shared_ptr<PyShellMassField> make_shell_density_elastic_thickness(
  double density, const PyMaterialParameterRef &parameter);

}  // namespace pgo
