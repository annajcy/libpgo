#pragma once

#include <span>

namespace pgo
{
namespace SolidDeformationModel
{

class ParameterFieldMapping
{
public:
  virtual ~ParameterFieldMapping() = default;

  virtual int numInputDofs() const = 0;
  virtual int numChannels() const = 0;
  virtual bool isAffine() const = 0;

  virtual void evaluate(
    int element,
    int quadrature,
    std::span<const double> localDofValues,
    std::span<double> materialValues) const = 0;

  virtual void evaluateJacobian(
    int element,
    int quadrature,
    std::span<const double> localDofValues,
    double *output) const = 0;

  virtual void evaluateHessians(
    int element,
    int quadrature,
    std::span<const double> localDofValues,
    double *output) const = 0;
};

class IdentityParameterFieldMapping final : public ParameterFieldMapping
{
public:
  explicit IdentityParameterFieldMapping(int numChannels);

  int numInputDofs() const override { return numChannels_; }
  int numChannels() const override { return numChannels_; }
  bool isAffine() const override { return true; }

  void evaluate(
    int element,
    int quadrature,
    std::span<const double> localDofValues,
    std::span<double> materialValues) const override;
  void evaluateJacobian(
    int element,
    int quadrature,
    std::span<const double> localDofValues,
    double *output) const override;
  void evaluateHessians(
    int element,
    int quadrature,
    std::span<const double> localDofValues,
    double *output) const override;

private:
  int numChannels_ = 0;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
