/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#pragma once

#include "material/plastic/plasticModel.h"

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
  virtual void computeA(const double *param, double A[9]) const;
  virtual void computeAInv(const double *param, double AInv[9]) const;
  virtual double compute_detA(const double *param) const;
  // here ddetA_da (and d2detA_da2)'s dimension is matched with param.
  // For example, if param is \in R^3, then ddetA_da is \in R^3 as well.
  // And d2detA_da2 is \in R^3 x R^3
  virtual void compute_ddetA_da(const double *param, double *ddetaA_da, int leadingDim) const;
  virtual void compute_d2detA_da2(const double *param, double *d2detA_da2, int leadingDim) const;
  virtual void compute_dAInv_da(const double *param, int pi, double ret[9]) const;
  virtual void compute_d2AInv_da2(const double *param, int pi, int pj, double ret[9]) const;

  virtual void defaultFp(double *Fp) const;
  virtual void projectParam(double *param, double zeroThreshold) const;
  virtual void toParam(const double *Fp, double *param) const;

  virtual void computeR(const double *param, double R[9]) const;
  virtual void compute_dparamfull_dparamsub(const double *param, const double *basis, int numHandles, double *dpf_dps) const;
  virtual bool has2OrderDeriv() const { return false; }
  virtual void compute_d2paramfull_dparamsub2(const double *param, const double *basis, int numHandles, int pi, double *d2a_dz2) const;

  virtual void compute_paramfull(double *param) const;

  void defaultParams(double *param) const override = 0;
};

inline void PlasticModel3DDeformationGradient::computeA(const double *, double[9]) const
{
  throw std::logic_error(
    "PlasticModel3DDeformationGradient::computeA is not implemented.");
}

inline void PlasticModel3DDeformationGradient::computeAInv(const double *, double[9]) const
{
  throw std::logic_error(
    "PlasticModel3DDeformationGradient::computeAInv is not implemented.");
}

inline double PlasticModel3DDeformationGradient::compute_detA(const double *) const
{
  throw std::logic_error(
    "PlasticModel3DDeformationGradient::compute_detA is not implemented.");
}

inline void PlasticModel3DDeformationGradient::compute_ddetA_da(const double *, double *, int) const
{
  throw std::logic_error(
    "PlasticModel3DDeformationGradient::compute_ddetA_da is not implemented.");
}

inline void PlasticModel3DDeformationGradient::compute_d2detA_da2(const double *, double *, int) const
{
  throw std::logic_error(
    "PlasticModel3DDeformationGradient::compute_d2detA_da2 is not implemented.");
}

inline void PlasticModel3DDeformationGradient::compute_dAInv_da(const double *, int, double[9]) const
{
  throw std::logic_error(
    "PlasticModel3DDeformationGradient::compute_dAInv_da is not implemented.");
}

inline void PlasticModel3DDeformationGradient::compute_d2AInv_da2(const double *, int, int, double[9]) const
{
  throw std::logic_error(
    "PlasticModel3DDeformationGradient::compute_d2AInv_da2 is not implemented.");
}

inline void PlasticModel3DDeformationGradient::defaultFp(double *) const
{
  throw std::logic_error(
    "PlasticModel3DDeformationGradient::defaultFp is not implemented.");
}

inline void PlasticModel3DDeformationGradient::projectParam(double *, double) const
{
  throw std::logic_error(
    "PlasticModel3DDeformationGradient::projectParam is not implemented.");
}

inline void PlasticModel3DDeformationGradient::toParam(const double *, double *) const
{
  throw std::logic_error(
    "PlasticModel3DDeformationGradient::toParam is not implemented.");
}

inline void PlasticModel3DDeformationGradient::computeR(const double *, double[9]) const
{
  throw std::logic_error(
    "PlasticModel3DDeformationGradient::computeR is not implemented.");
}

inline void PlasticModel3DDeformationGradient::compute_dparamfull_dparamsub(const double *, const double *, int, double *) const
{
  throw std::logic_error(
    "PlasticModel3DDeformationGradient::compute_dparamfull_dparamsub is not implemented.");
}

inline void PlasticModel3DDeformationGradient::compute_d2paramfull_dparamsub2(const double *, const double *, int, int, double *) const
{
  throw std::logic_error(
    "PlasticModel3DDeformationGradient::compute_d2paramfull_dparamsub2 is not implemented.");
}

inline void PlasticModel3DDeformationGradient::compute_paramfull(double *) const
{
  throw std::logic_error(
    "PlasticModel3DDeformationGradient::compute_paramfull is not implemented.");
}
}  // namespace SolidDeformationModel
}  // namespace pgo
