#pragma once

#include "EigenSupport.h"

#include <memory>
#include <span>

namespace pgo::SolidDeformationModel
{

class MaterialStateView;

/// Immutable element-major physical optimizable material channel values.
class MaterialState
{
public:
  MaterialState();
  MaterialState(
    EigenSupport::VXd elasticValues,
    EigenSupport::VXd plasticValues);

  const EigenSupport::VXd &elasticValues() const { return *elasticValues_; }
  const EigenSupport::VXd &plasticValues() const { return *plasticValues_; }

  MaterialStateView view() const;
  operator MaterialStateView() const;
  MaterialState withElasticValues(
    std::span<const double> elasticValues) const;
  MaterialState withPlasticValues(
    std::span<const double> plasticValues) const;
  MaterialState withValues(
    std::span<const double> elasticValues,
    std::span<const double> plasticValues) const;

private:
  MaterialState(
    std::shared_ptr<const EigenSupport::VXd> elasticValues,
    std::shared_ptr<const EigenSupport::VXd> plasticValues);

  std::shared_ptr<const EigenSupport::VXd> elasticValues_;
  std::shared_ptr<const EigenSupport::VXd> plasticValues_;
};

/// Non-owning element-major physical material channel values.
class MaterialStateView
{
public:
  MaterialStateView() = default;
  MaterialStateView(
    std::span<const double> elasticValues,
    std::span<const double> plasticValues):
    elasticValues_(elasticValues), plasticValues_(plasticValues)
  {
  }

  std::span<const double> elasticValues() const { return elasticValues_; }
  std::span<const double> plasticValues() const { return plasticValues_; }

private:
  std::span<const double> elasticValues_;
  std::span<const double> plasticValues_;
};

}  // namespace pgo::SolidDeformationModel
