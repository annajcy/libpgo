#include "core.h"

#include "EigenSupport.h"
#include "../../energy/core.h"

namespace pgo
{

std::shared_ptr<PyVolumeDensity> make_constant_volume_density(double density)
{
  return std::make_shared<PyVolumeDensity>(
    std::make_shared<SolidDeformationModel::VolumeDensityField>(
      SolidDeformationModel::VolumeDensityField::constant(density)));
}

std::shared_ptr<PyVolumeDensity> make_elementwise_volume_density(
  const std::vector<double> &densities)
{
  EigenSupport::VXd values(static_cast<Eigen::Index>(densities.size()));
  for (size_t i = 0; i < densities.size(); i++)
    values[static_cast<Eigen::Index>(i)] = densities[i];
  return std::make_shared<PyVolumeDensity>(
    std::make_shared<SolidDeformationModel::VolumeDensityField>(
      SolidDeformationModel::VolumeDensityField::elementwise(std::move(values))));
}

std::shared_ptr<PyShellArealDensity> make_constant_shell_areal_density(
  double arealDensity)
{
  return std::make_shared<PyShellArealDensity>(
    std::make_shared<SolidDeformationModel::ShellArealDensityField>(
      SolidDeformationModel::ShellArealDensityField::constant(arealDensity)));
}

std::shared_ptr<PyShellArealDensity> make_shell_areal_density_elementwise(
  const std::vector<double> &arealDensities)
{
  EigenSupport::VXd values(static_cast<Eigen::Index>(arealDensities.size()));
  for (size_t i = 0; i < arealDensities.size(); i++)
    values[static_cast<Eigen::Index>(i)] = arealDensities[i];
  return std::make_shared<PyShellArealDensity>(
    std::make_shared<SolidDeformationModel::ShellArealDensityField>(
      SolidDeformationModel::ShellArealDensityField::elementwise(std::move(values))));
}

std::shared_ptr<PyShellArealDensity> make_shell_areal_density_from_density_thickness(
  double density, double thickness)
{
  return std::make_shared<PyShellArealDensity>(
    std::make_shared<SolidDeformationModel::ShellArealDensityField>(
      SolidDeformationModel::ShellArealDensityField::fromDensityThickness(
        density, thickness)));
}

std::shared_ptr<PyShellArealDensity> make_shell_areal_density_from_density_thickness(
  double density, const std::vector<double> &thickness)
{
  EigenSupport::VXd values(static_cast<Eigen::Index>(thickness.size()));
  for (size_t i = 0; i < thickness.size(); i++)
    values[static_cast<Eigen::Index>(i)] = thickness[i];
  return std::make_shared<PyShellArealDensity>(
    std::make_shared<SolidDeformationModel::ShellArealDensityField>(
      SolidDeformationModel::ShellArealDensityField::fromDensityThickness(
        density, std::move(values))));
}

std::shared_ptr<PyShellArealDensity> make_shell_areal_density_from_elastic_parameter(
  double scale, const PyOptimizableParameterRef &parameter)
{
  return std::make_shared<PyShellArealDensity>(
    std::make_shared<SolidDeformationModel::ShellArealDensityField>(
      SolidDeformationModel::ShellArealDensityField::fromElasticParameter(
        scale, parameter.ref())));
}

}  // namespace pgo
