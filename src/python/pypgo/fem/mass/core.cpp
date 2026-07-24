#include "core.h"

#include "mass/shellDensityElasticThickness.h"
#include "EigenSupport.h"
#include "../../energy/core.h"

namespace pgo
{

std::shared_ptr<PyVolumeMassField> make_constant_volume_density(double density)
{
  return std::make_shared<PyVolumeMassField>(
    std::make_shared<SolidDeformationModel::ConstantVolumeDensity>(density));
}

std::shared_ptr<PyVolumeMassField> make_elementwise_volume_density(const std::vector<double> &densities)
{
  EigenSupport::VXd values(static_cast<Eigen::Index>(densities.size()));
  for (size_t i = 0; i < densities.size(); i++)
    values[static_cast<Eigen::Index>(i)] = densities[i];
  return std::make_shared<PyVolumeMassField>(
    std::make_shared<SolidDeformationModel::ElementwiseVolumeDensity>(std::move(values)));
}

std::shared_ptr<PyShellMassField> make_constant_shell_areal_density(double arealDensity)
{
  return std::make_shared<PyShellMassField>(
    std::make_shared<SolidDeformationModel::ConstantShellArealDensity>(arealDensity));
}

std::shared_ptr<PyShellMassField> make_shell_density_thickness_constant(double density, double thickness)
{
  return std::make_shared<PyShellMassField>(
    std::make_shared<SolidDeformationModel::ShellDensityThickness>(density, thickness));
}

std::shared_ptr<PyShellMassField> make_shell_density_thickness_elementwise(
  double density, const std::vector<double> &thickness)
{
  EigenSupport::VXd values(static_cast<Eigen::Index>(thickness.size()));
  for (size_t i = 0; i < thickness.size(); i++)
    values[static_cast<Eigen::Index>(i)] = thickness[i];
  return std::make_shared<PyShellMassField>(
    std::make_shared<SolidDeformationModel::ShellDensityThickness>(density, std::move(values)));
}

std::shared_ptr<PyShellMassField> make_shell_density_elastic_thickness(
  double density, const PyMaterialParameterRef &parameter)
{
  return std::make_shared<PyShellMassField>(
    std::make_shared<SolidDeformationModel::ShellDensityElasticThickness>(
      density, parameter.space(), parameter.ref()));
}

}  // namespace pgo
