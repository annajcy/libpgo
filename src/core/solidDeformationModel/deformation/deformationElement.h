/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#pragma once

#include "EigenSupport.h"

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
  // Evaluate energy and displacement gradient in one pass so the element
  // prepares its geometry/material state only once.
  virtual double computeEnergyGradient(
    std::span<const double> x,
    std::span<const double> elasticParams,
    std::span<const double> plasticParams,
    EigenSupport::RefVecXd displacementGradient) const
  {
    const double energy = computeEnergy(x, elasticParams, plasticParams);
    computeDisplacementGradient(
      x, elasticParams, plasticParams, displacementGradient);
    return energy;
  }
  // Evaluate energy, displacement gradient and Hessian in one pass so the
  // element prepares its geometry/material state only once. The default
  // implementation falls back to three separate evaluations; concrete
  // elements override it to reuse the prepared state.
  virtual double computeEnergyGradientHessian(
    std::span<const double> x,
    std::span<const double> elasticParams,
    std::span<const double> plasticParams,
    EigenSupport::RefVecXd displacementGradient,
    EigenSupport::RefMatXd displacementHessian) const
  {
    const double energy = computeEnergy(x, elasticParams, plasticParams);
    computeDisplacementGradient(
      x, elasticParams, plasticParams, displacementGradient);
    computeDisplacementHessian(
      x, elasticParams, plasticParams, displacementHessian);
    return energy;
  }
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

  // advanced routines
  virtual int getNumMaterialLocations() const { return 1; }
};
}  // namespace SolidDeformationModel
}  // namespace pgo
