

#include "deformation/materialMaxStepPolynomialUtils.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>

namespace pgo::SolidDeformationModel
{
namespace ES = pgo::EigenSupport;

namespace
{
constexpr double kBoundaryEvalTol = 1e-12;

void insertCriticalPointIfInUnitInterval(std::array<double, 4> &cuts, int &count, double value)
{
  if (value > 0.0 && value < 1.0) {
    cuts[count++] = value;
  }
}

void appendCriticalPointsInUnitInterval(
  const BasicAlgorithms::CubicPolynomial &poly, std::array<double, 4> &cuts, int &count)
{
  const double qa = 3.0 * poly.c3;
  const double qb = 2.0 * poly.c2;
  const double qc = poly.c1;

  if (qa == 0.0) {
    if (qb != 0.0) {
      insertCriticalPointIfInUnitInterval(cuts, count, -qc / qb);
    }
    return;
  }

  const double disc = qb * qb - 4.0 * qa * qc;
  if (disc < 0.0) {
    return;
  }

  const double sqrtDisc = std::sqrt(std::max(0.0, disc));
  const double denom = 2.0 * qa;
  insertCriticalPointIfInUnitInterval(cuts, count, (-qb - sqrtDisc) / denom);
  insertCriticalPointIfInUnitInterval(cuts, count, (-qb + sqrtDisc) / denom);
}
}

double detFromColumns(const ES::V3d &c0, const ES::V3d &c1, const ES::V3d &c2)
{
  ES::M3d M;
  M.col(0) = c0;
  M.col(1) = c1;
  M.col(2) = c2;
  return M.determinant();
}

BasicAlgorithms::CubicPolynomial buildDeterminantCubicFromAffineMatrixPath(const double AData[9], const double BData[9], double eps)
{
  const Eigen::Map<const ES::M3d> A(AData);
  const Eigen::Map<const ES::M3d> B(BData);

  const ES::V3d a0 = A.col(0);
  const ES::V3d a1 = A.col(1);
  const ES::V3d a2 = A.col(2);
  const ES::V3d b0 = B.col(0);
  const ES::V3d b1 = B.col(1);
  const ES::V3d b2 = B.col(2);

  BasicAlgorithms::CubicPolynomial poly;
  poly.c0 = detFromColumns(a0, a1, a2) - eps;
  poly.c1 = detFromColumns(b0, a1, a2) + detFromColumns(a0, b1, a2) + detFromColumns(a0, a1, b2);
  poly.c2 = detFromColumns(b0, b1, a2) + detFromColumns(b0, a1, b2) + detFromColumns(a0, b1, b2);
  poly.c3 = detFromColumns(b0, b1, b2);
  return poly;
}

double applyMaterialMaxStepSafetyClamp(double alpha)
{
  if (alpha >= 1.0) {
    return 1.0;
  }

  return std::max(kMaterialMaxStepMinClamp, std::min(1.0, alpha * kMaterialMaxStepInteriorSafety));
}

ConservativeFeasibleAlphaResult findConservativeFeasibleAlpha(const BasicAlgorithms::CubicPolynomial &poly, double eps)
{
  ConservativeFeasibleAlphaResult result;
  result.phi0 = poly.eval(0.0) + eps;
  if (result.phi0 <= eps) {
    result.alpha = kMaterialMaxStepMinClamp;
    result.illegalInitialState = true;
    return result;
  }

  std::array<double, 4> cuts{};
  int cutCount = 0;
  cuts[cutCount++] = 0.0;
  appendCriticalPointsInUnitInterval(poly, cuts, cutCount);
  cuts[cutCount++] = 1.0;

  std::sort(cuts.begin(), cuts.begin() + cutCount);
  const auto uniqueEnd = std::unique(cuts.begin(), cuts.begin() + cutCount, [](double lhs, double rhs) {
    return std::abs(lhs - rhs) <= kBoundaryEvalTol;
  });
  cutCount = static_cast<int>(std::distance(cuts.begin(), uniqueEnd));

  for (int i = 1; i < cutCount; i++) {
    const double left = cuts[i - 1];
    const double right = cuts[i];
    const double gr = poly.eval(right);

    if (gr > kBoundaryEvalTol) {
      continue;
    }

    if (std::abs(gr) <= kBoundaryEvalTol) {
      result.alpha = applyMaterialMaxStepSafetyClamp(right);
      return result;
    }

    result.alpha = applyMaterialMaxStepSafetyClamp(BasicAlgorithms::findFirstBoundaryRootByBisection(poly, left, right));
    return result;
  }

  result.alpha = 1.0;
  return result;
}
}
