#pragma once

#include "mass/volumeMassField.h"

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

}  // namespace pgo
