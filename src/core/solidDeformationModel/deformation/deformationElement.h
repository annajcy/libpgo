/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#pragma once

#include "EigenSupport.h"

#include <limits>
#include <span>
#include <stdexcept>

namespace pgo
{
namespace SolidDeformationModel
{

struct DeformationElementConstructionOptions
{
  bool projectHessianPSD = false;
};

class UnsupportedDeformationDiagnosticError : public std::logic_error
{
public:
  using std::logic_error::logic_error;
};

class DeformationElement
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

  DeformationElement() = default;
  virtual ~DeformationElement() = default;

  virtual double computeEnergy(
    std::span<const double> x,
    std::span<const double> elasticParams,
    std::span<const double> plasticParams) const = 0;
  virtual void computeDisplacementGradient(
    std::span<const double> x,
    std::span<const double> elasticParams,
    std::span<const double> plasticParams,
    EigenSupport::RefVecXd output) const = 0;
  virtual void computeDisplacementHessian(
    std::span<const double> x,
    std::span<const double> elasticParams,
    std::span<const double> plasticParams,
    EigenSupport::RefMatXd output) const = 0;
  virtual void computeElasticGradient(
    std::span<const double> x,
    std::span<const double> elasticParams,
    std::span<const double> plasticParams,
    EigenSupport::RefVecXd output) const = 0;
  virtual void computePlasticGradient(
    std::span<const double> x,
    std::span<const double> elasticParams,
    std::span<const double> plasticParams,
    EigenSupport::RefVecXd output) const = 0;
  virtual void computeElasticVJP(
    std::span<const double> x,
    std::span<const double> elasticParams,
    std::span<const double> plasticParams,
    std::span<const double> displacementAdjoint,
    EigenSupport::RefVecXd output) const = 0;
  virtual void computePlasticVJP(
    std::span<const double> x,
    std::span<const double> elasticParams,
    std::span<const double> plasticParams,
    std::span<const double> displacementAdjoint,
    EigenSupport::RefVecXd output) const = 0;

  virtual int computeVonMisesStress(
    std::span<const double>, std::span<const double>,
    std::span<const double>, std::span<double>) const
  {
    throw UnsupportedDeformationDiagnosticError(
      "Von Mises stress is not implemented by this deformation element.");
  }
  virtual int computeMaxStrain(
    std::span<const double>, std::span<const double>,
    std::span<const double>, std::span<double>) const
  {
    throw UnsupportedDeformationDiagnosticError(
      "Maximum strain is not implemented by this deformation element.");
  }

  virtual int getNumElasticParameters() const = 0;
  virtual int getNumPlasticParameters() const = 0;
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
