/*
author: Bohan Wang
copyright to USC
*/

#pragma once

#include "EigenSupport.h"
#include "constraints/constraintFunctions.h"

namespace pgo
{
namespace ES = EigenSupport;

namespace NonlinearOptimization
{
class NaturalCubicSpline2DWithParameterDerivatives
{
public:
  explicit NaturalCubicSpline2DWithParameterDerivatives(
    EigenSupport::ConstRefVecXd xValues);
  NaturalCubicSpline2DWithParameterDerivatives(
    EigenSupport::ConstRefVecXd xValues,
    EigenSupport::ConstRefVecXd yValues);

  double y(double x) const;
  double y(double x, EigenSupport::ConstRefVecXd yValues) const;
  double dy_dx(double x) const;
  double dy_dx(double x, EigenSupport::ConstRefVecXd yValues) const;
  double d2y_dx2(double x) const;
  double d2y_dx2(double x, EigenSupport::ConstRefVecXd yValues) const;

  void dy_dparam(double x, EigenSupport::RefVecXd dparam) const;
  void d2y_dparam2(double x, EigenSupport::RefMatXd dparam) const;
  void d2y_dparam_dx(double x, EigenSupport::RefVecXd dparam) const;

  double dy_dparam(double x, int i) const;
  double d2y_dparam2(double x, int i, int j) const;
  double d2y_dparam_dx(double x, int i) const;

protected:
  void initialize(EigenSupport::ConstRefVecXd xValues,
    EigenSupport::ConstRefVecXd yValues, bool hasYValues);
  void buildSystem();
  void validateYValues(EigenSupport::ConstRefVecXd yValues) const;

  int n;
  EigenSupport::VXd xNodeValue, yNodeValue;
  EigenSupport::MXd A, AInv;
  EigenSupport::VXd rhs;
};

class NaturalCubicSpline2DAsNonlinearConstraints : public ConstraintFunctions
{
public:
  NaturalCubicSpline2DAsNonlinearConstraints(int numPoints, int xDOFOffset, int yDOFOffset, int paramDOFOffset, int nAll, double y0Prime, double ynPrime);
  virtual ~NaturalCubicSpline2DAsNonlinearConstraints() {}

  virtual void func(EigenSupport::ConstRefVecXd x, EigenSupport::RefVecXd g) const override;
  virtual void jacobian(EigenSupport::ConstRefVecXd x, EigenSupport::SpMatD &jac) const override;
  virtual void hessianInPlace(EigenSupport::ConstRefVecXd x, EigenSupport::ConstRefVecXd lambda, EigenSupport::SpMatD &hess) const override;

  virtual bool isLinear() const override { return false; }
  virtual bool isQuadratic() const override { return false; }
  virtual bool hasHessianVector() const override { return false; }

  void solveParams(EigenSupport::RefVecXd x) const;

protected:
  inline double getxi(EigenSupport::ConstRefVecXd x, int i) const { return x[xDOFOffset + i]; }
  inline double getyi(EigenSupport::ConstRefVecXd x, int i) const { return x[yDOFOffset + i]; }
  inline EigenSupport::V4d getParami(EigenSupport::ConstRefVecXd x, int i) const { return x.segment<4>(paramDOFOffset + i * 4); }

  inline double compute_fi(const EigenSupport::V4d &param, double x) const { return param[0] * x * x * x + param[1] * x * x + param[2] * x + param[3]; }
  inline double compute_fi_1stDeriv(const EigenSupport::V4d &param, double x) const { return 3 * param[0] * x * x + 2 * param[1] * x + param[2]; }
  inline double compute_fi_2ndDeriv(const EigenSupport::V4d &param, double x) const { return 6 * param[0] * x + 2 * param[1]; }

  int numPoints;
  int xDOFOffset;
  int yDOFOffset;
  int paramDOFOffset;
  int nAll;

  double endDeriv[2];
};
}  // namespace NonlinearOptimization
}  // namespace pgo
