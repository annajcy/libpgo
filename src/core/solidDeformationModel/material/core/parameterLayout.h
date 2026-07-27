#pragma once

#include "EigenSupport.h"

#include <span>

namespace pgo
{
namespace SolidDeformationModel
{

class ParameterLayout
{
public:
  virtual ~ParameterLayout() = default;

  virtual int numElements() const = 0;
  virtual int numLocalParameters() const = 0;
  virtual int numGlobalParameters() const = 0;
  virtual int numValueRows() const = 0;

  virtual int globalParameter(int element, int localParameter) const = 0;
  virtual void gather(
    int element,
    std::span<const double> globalParameters,
    std::span<double> localParameters) const = 0;
};

class ConstantParameterLayout final : public ParameterLayout
{
public:
  ConstantParameterLayout(int numElements, int numLocalParameters);

  int numElements() const override { return numElements_; }
  int numLocalParameters() const override { return numLocalParameters_; }
  int numGlobalParameters() const override { return numLocalParameters_; }
  int numValueRows() const override
  {
    return numLocalParameters_ == 0 ? 0 : 1;
  }

  int globalParameter(int element, int localParameter) const override;
  void gather(
    int element,
    std::span<const double> globalParameters,
    std::span<double> localParameters) const override;

private:
  int numElements_ = 0;
  int numLocalParameters_ = 0;
};

class ElementwiseParameterLayout final : public ParameterLayout
{
public:
  ElementwiseParameterLayout(int numElements, int numLocalParameters);

  int numElements() const override { return numElements_; }
  int numLocalParameters() const override { return numLocalParameters_; }
  int numGlobalParameters() const override
  {
    return numElements_ * numLocalParameters_;
  }
  int numValueRows() const override
  {
    return numLocalParameters_ == 0 ? 0 : numElements_;
  }

  int globalParameter(int element, int localParameter) const override;
  void gather(
    int element,
    std::span<const double> globalParameters,
    std::span<double> localParameters) const override;

private:
  int numElements_ = 0;
  int numLocalParameters_ = 0;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
