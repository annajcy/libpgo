#pragma once

#include "EigenSupport.h"

#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace pgo::SolidDeformationModel::detail
{

// Transfer ownership of a material model to its concrete constitutive type,
// rejecting incompatible models up front instead of deferring the failure to
// element construction.
template<class Derived, class Base>
std::unique_ptr<Derived> checkedMaterialCast(
  std::unique_ptr<Base> model, std::string_view message)
{
  if (Derived *typed = dynamic_cast<Derived *>(model.get())) {
    model.release();
    return std::unique_ptr<Derived>(typed);
  }

  throw std::invalid_argument(std::string(message));
}

inline void validateElementDensities(
  EigenSupport::ConstRefVecXd densities, int numElements,
  std::string_view quantityLabel)
{
  if (densities.size() != numElements)
    throw std::invalid_argument(
      std::string(quantityLabel) +
      " count does not match mesh element count");
  if (!densities.allFinite() || (densities.array() <= 0.0).any())
    throw std::invalid_argument(
      std::string(quantityLabel) + " must contain finite values > 0");
}

inline std::vector<double> flattenSurfaceVertices(
  const EigenSupport::MXd &surfaceVertices)
{
  if (surfaceVertices.cols() != 3) {
    throw std::invalid_argument(
      "surfaceVertices must have shape numVertices x 3");
  }
  std::vector<double> flat(static_cast<size_t>(surfaceVertices.rows()) * 3);
  for (Eigen::Index i = 0; i < surfaceVertices.rows(); i++)
    for (int d = 0; d < 3; d++)
      flat[static_cast<size_t>(i) * 3 + d] = surfaceVertices(i, d);
  return flat;
}

}  // namespace pgo::SolidDeformationModel::detail
