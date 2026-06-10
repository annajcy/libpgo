#include "core.h"

#include "EigenSupport.h"

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

}  // namespace pgo
