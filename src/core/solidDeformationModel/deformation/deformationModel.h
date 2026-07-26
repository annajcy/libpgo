/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#pragma once

#include "deformationModelEvaluator.h"

#include <limits>
#include <memory>
#include <span>

namespace pgo
{
namespace SolidDeformationModel
{

struct DeformationModelConstructionOptions
{
  bool projectHessianPSD = false;
};

class DeformationModel
{
public:
  struct LocalMaxStepResult
  {
    double alpha = 1.0;
    bool illegalInitialState = false;
    double phi0 = std::numeric_limits<double>::quiet_NaN();
    double eps = 0.0;
    int locationId = -1;
  };

  DeformationModel() = default;
  virtual ~DeformationModel() {}

  // Generic construction hook for callers that own models polymorphically.
  // Typed callers should construct the corresponding evaluator directly.
  virtual std::unique_ptr<DeformationModelEvaluator> createEvaluator() const = 0;

  virtual int getNumElasticParameters() const = 0;
  virtual int getNumPlasticParameters() const = 0;
  virtual void defaultPlasticParams(std::span<double> params) const { (void)params; }
  virtual int getNumVertices() const = 0;
  virtual int getNumDOFs() const = 0;
  virtual LocalMaxStepResult computeLocalMaxStepSize(std::span<const double> x_local,
    std::span<const double> dx_local) const
  {
    (void)x_local;
    (void)dx_local;
    return LocalMaxStepResult{};
  }

  // advanced routines
  virtual int getNumMaterialLocations() const { return 1; }
};
}  // namespace SolidDeformationModel
}  // namespace pgo
