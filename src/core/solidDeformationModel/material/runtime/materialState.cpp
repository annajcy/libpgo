#include "material/runtime/materialState.h"

namespace pgo::SolidDeformationModel
{
namespace
{
std::shared_ptr<const EigenSupport::VXd> copyValues(
  std::span<const double> values)
{
  return std::make_shared<const EigenSupport::VXd>(
    Eigen::Map<const EigenSupport::VXd>(
      values.data(), static_cast<Eigen::Index>(values.size())));
}
}

MaterialState::MaterialState():
  MaterialState(EigenSupport::VXd{}, EigenSupport::VXd{})
{
}

MaterialState::MaterialState(
  EigenSupport::VXd elasticValues,
  EigenSupport::VXd plasticValues):
  elasticValues_(
    std::make_shared<const EigenSupport::VXd>(std::move(elasticValues))),
  plasticValues_(
    std::make_shared<const EigenSupport::VXd>(std::move(plasticValues)))
{
}

MaterialState::MaterialState(
  std::shared_ptr<const EigenSupport::VXd> elasticValues,
  std::shared_ptr<const EigenSupport::VXd> plasticValues):
  elasticValues_(std::move(elasticValues)),
  plasticValues_(std::move(plasticValues))
{
}

MaterialStateView MaterialState::view() const
{
  return MaterialStateView(
    std::span<const double>(elasticValues_->data(), elasticValues_->size()),
    std::span<const double>(plasticValues_->data(), plasticValues_->size()));
}

MaterialState::operator MaterialStateView() const
{
  return view();
}

MaterialState MaterialState::withElasticValues(
  std::span<const double> elasticValues) const
{
  return MaterialState(copyValues(elasticValues), plasticValues_);
}

MaterialState MaterialState::withPlasticValues(
  std::span<const double> plasticValues) const
{
  return MaterialState(elasticValues_, copyValues(plasticValues));
}

MaterialState MaterialState::withValues(
  std::span<const double> elasticValues,
  std::span<const double> plasticValues) const
{
  return MaterialState(copyValues(elasticValues), copyValues(plasticValues));
}

}  // namespace pgo::SolidDeformationModel
