#pragma once

#include "EigenSupport.h"

#include <span>

namespace pgo
{
namespace SolidDeformationModel
{

class ParameterDofLayout
{
public:
  virtual ~ParameterDofLayout() = default;

  virtual int numElements() const = 0;
  virtual int numLocalDofs() const = 0;
  virtual int numGlobalDofs() const = 0;
  virtual int numValueRows() const = 0;

  virtual int globalDof(int element, int localDof) const = 0;
  virtual void gather(
    int element,
    std::span<const double> globalValues,
    std::span<double> localDofValues) const = 0;
};

class ConstantParameterDofLayout final : public ParameterDofLayout
{
public:
  ConstantParameterDofLayout(int numElements, int numLocalDofs);

  int numElements() const override { return numElements_; }
  int numLocalDofs() const override { return numLocalDofs_; }
  int numGlobalDofs() const override { return numLocalDofs_; }
  int numValueRows() const override { return numLocalDofs_ == 0 ? 0 : 1; }

  int globalDof(int element, int localDof) const override;
  void gather(
    int element,
    std::span<const double> globalValues,
    std::span<double> localDofValues) const override;

private:
  int numElements_ = 0;
  int numLocalDofs_ = 0;
};

class ElementwiseParameterDofLayout final : public ParameterDofLayout
{
public:
  ElementwiseParameterDofLayout(int numElements, int numLocalDofs);

  int numElements() const override { return numElements_; }
  int numLocalDofs() const override { return numLocalDofs_; }
  int numGlobalDofs() const override { return numElements_ * numLocalDofs_; }
  int numValueRows() const override { return numLocalDofs_ == 0 ? 0 : numElements_; }

  int globalDof(int element, int localDof) const override;
  void gather(
    int element,
    std::span<const double> globalValues,
    std::span<double> localDofValues) const override;

private:
  int numElements_ = 0;
  int numLocalDofs_ = 0;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
