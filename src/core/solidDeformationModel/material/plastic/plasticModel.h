#pragma once

#include <span>

namespace pgo
{
namespace SolidDeformationModel
{
class PlasticModel
{
public:
  virtual ~PlasticModel() = default;

  // Every evaluator must declare its parameter dimension.  A missing
  // declaration must fail at compile time instead of silently creating a
  // zero-dimensional parameter block.
  virtual int getNumParameters() const = 0;

  // Write default (rest-configuration) plastic parameters into param.
  // Zero-parameter evaluator bases override this explicitly as a no-op.
  virtual void defaultParams(std::span<double> param) const = 0;
};
}  // namespace SolidDeformationModel
}  // namespace pgo
