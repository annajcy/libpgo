#include "elastic/elasticModel1DCubicSpline.h"

#include "naturalCubicSplineDerivatives.h"
#include "EigenSupport.h"

#include "pgoLogging.h"

#include <memory>

namespace pgo::SolidDeformationModel
{
namespace ES = pgo::EigenSupport;

class ElasticModel1DCubicSplineImpl
{
public:
  std::shared_ptr<NonlinearOptimization::NaturalCubicSpline2DWithParameterDerivatives> spline;
  ES::VXd xValues;
  double c;
};
}  // namespace pgo::SolidDeformationModel

using namespace pgo::SolidDeformationModel;

ElasticModel1DCubicSpline::ElasticModel1DCubicSpline(double coeff, int nPoints, double xLeft, double xRight)
{
  impl_ = std::make_unique<ElasticModel1DCubicSplineImpl>();
  impl_->xValues.resize(nPoints);

  int nGaps = nPoints - 1;
  double detlaX = (xRight - xLeft) / nGaps;

  for (int i = 0; i < nPoints; i++)
    impl_->xValues[i] = xLeft + i * detlaX;

  impl_->spline = std::make_shared<NonlinearOptimization::NaturalCubicSpline2DWithParameterDerivatives>(nPoints, impl_->xValues.data());
  impl_->c = coeff;
}

ElasticModel1DCubicSpline::~ElasticModel1DCubicSpline() = default;

double ElasticModel1DCubicSpline::compute_psi(const double *param, const double F[9],
  const double[], const double[], const double S[]) const
{
  double x = F[0] - F[1];
  return impl_->spline->y(x, param) * impl_->c;
}

void ElasticModel1DCubicSpline::compute_P(const double *param, const double F[9],
  const double[], const double[], const double[], double P[]) const
{
  double x = F[0] - F[1];
  P[0] = impl_->spline->dy_dx(x, param) * impl_->c;
}

void ElasticModel1DCubicSpline::compute_dPdF(const double *param, const double F[9],
  const double[], const double[], const double[], double dPdF[]) const
{
  double x = F[0] - F[1];
  dPdF[0] = impl_->spline->d2y_dx2(x, param) * impl_->c;
}

int ElasticModel1DCubicSpline::getNumParameters() const
{
  return static_cast<int>(impl_->xValues.size());
}

double ElasticModel1DCubicSpline::compute_dpsi_dparam(const double *param, int i, const double F[],
  const double[], const double[], const double[]) const
{
  double x = F[0] - F[1];
  return impl_->spline->dy_dparam(x, param, i) * impl_->c;
}

double
ElasticModel1DCubicSpline::compute_d2psi_dparam2(const double *param, int i, int j,
  const double F[], const double[], const double[], const double[]) const
{
  double x = F[0] - F[1];
  return impl_->spline->d2y_dparam2(x, param, i, j) * impl_->c;
}

void ElasticModel1DCubicSpline::compute_dP_dparam(const double *param, int i, const double F[],
  const double[], const double[], const double[], double *ret) const
{
  double x = F[0] - F[1];
  ret[0] = impl_->spline->d2y_dparam_dx(x, param, i) * impl_->c;
}
