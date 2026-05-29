#include "gaussLegendreHexQuadrature.h"

#include <cmath>

namespace pgo
{
namespace SolidDeformationModel
{

void GaussLegendreHexQuadrature2::point(int i, double xi[3]) const
{
  constexpr double offset = 0.5 / 1.7320508075688772;  // 0.5 / sqrt(3)
  constexpr double gp[2] = { 0.5 - offset, 0.5 + offset };

  // ia is outer loop, ig is inner loop (matching legacy convention)
  const int ia = i / 4;
  const int ib = (i / 2) % 2;
  const int ig = i % 2;

  xi[0] = gp[ia];
  xi[1] = gp[ib];
  xi[2] = gp[ig];
}

double GaussLegendreHexQuadrature2::weight(int) const
{
  return 0.125;  // 1/8
}

}  // namespace SolidDeformationModel
}  // namespace pgo
