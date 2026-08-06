/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#pragma once

#include "material/plastic/plasticModel.h"
#include "EigenSupport.h"

#include <stdexcept>

namespace pgo
{
namespace SolidDeformationModel
{
class PlasticModel3DDeformationGradient : public PlasticModel
{
public:
  PlasticModel3DDeformationGradient() = default;
  virtual ~PlasticModel3DDeformationGradient() {}

  // Every concrete 3D model must state its parameter dimension explicitly.
  // All evaluator hooks below throw unless the model implements them.
  virtual int getNumParameters() const override = 0;
  virtual EigenSupport::M3d computeA(std::span<const double> param) const;
  virtual EigenSupport::M3d computeAInv(std::span<const double> param) const;
  virtual double compute_detA(std::span<const double> param) const;
  // The output dimensions match getNumParameters().
  virtual void compute_ddetA_da(
    std::span<const double> param, EigenSupport::RefVecXd ddetA_da) const;
  virtual EigenSupport::M3d compute_dAInv_da(std::span<const double> param, int pi) const;

  virtual EigenSupport::M3d defaultFp() const;
  virtual void projectParam(std::span<double> param, double zeroThreshold) const;
  virtual void toParam(const EigenSupport::M3d &Fp, std::span<double> param) const;

  virtual EigenSupport::M3d computeR(std::span<const double> param) const;
};

inline EigenSupport::M3d PlasticModel3DDeformationGradient::computeA(std::span<const double>) const
{
  throw std::logic_error(
    "PlasticModel3DDeformationGradient::computeA is not implemented.");
}

inline EigenSupport::M3d PlasticModel3DDeformationGradient::computeAInv(std::span<const double>) const
{
  throw std::logic_error(
    "PlasticModel3DDeformationGradient::computeAInv is not implemented.");
}

inline double PlasticModel3DDeformationGradient::compute_detA(std::span<const double>) const
{
  throw std::logic_error(
    "PlasticModel3DDeformationGradient::compute_detA is not implemented.");
}

inline void PlasticModel3DDeformationGradient::compute_ddetA_da(
  std::span<const double>, EigenSupport::RefVecXd) const
{
  throw std::logic_error(
    "PlasticModel3DDeformationGradient::compute_ddetA_da is not implemented.");
}

inline EigenSupport::M3d PlasticModel3DDeformationGradient::compute_dAInv_da(std::span<const double>, int) const
{
  throw std::logic_error(
    "PlasticModel3DDeformationGradient::compute_dAInv_da is not implemented.");
}

inline EigenSupport::M3d PlasticModel3DDeformationGradient::defaultFp() const
{
  throw std::logic_error(
    "PlasticModel3DDeformationGradient::defaultFp is not implemented.");
}

inline void PlasticModel3DDeformationGradient::projectParam(std::span<double>, double) const
{
  throw std::logic_error(
    "PlasticModel3DDeformationGradient::projectParam is not implemented.");
}

inline void PlasticModel3DDeformationGradient::toParam(const EigenSupport::M3d &, std::span<double>) const
{
  throw std::logic_error(
    "PlasticModel3DDeformationGradient::toParam is not implemented.");
}

inline EigenSupport::M3d PlasticModel3DDeformationGradient::computeR(std::span<const double>) const
{
  throw std::logic_error(
    "PlasticModel3DDeformationGradient::computeR is not implemented.");
}
}  // namespace SolidDeformationModel
}  // namespace pgo
