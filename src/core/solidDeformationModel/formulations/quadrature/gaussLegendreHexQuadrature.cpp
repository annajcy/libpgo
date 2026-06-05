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

namespace
{
// 4-point Gauss-Legendre mapped from [-1,1] to [0,1]: x = 0.5 + 0.5*node, w = 0.5*node_weight.
// Nodes +/-0.3399810435848563, +/-0.8611363115940526; weights 0.6521451548625461, 0.3478548451374538.
constexpr double kGp4[4] = {
  0.5 - 0.5 * 0.8611363115940526,
  0.5 - 0.5 * 0.3399810435848563,
  0.5 + 0.5 * 0.3399810435848563,
  0.5 + 0.5 * 0.8611363115940526,
};
constexpr double kGw4[4] = {
  0.5 * 0.3478548451374538,
  0.5 * 0.6521451548625461,
  0.5 * 0.6521451548625461,
  0.5 * 0.3478548451374538,
};
}  // namespace

void GaussLegendreHexQuadrature4::point(int i, double xi[3]) const
{
  const int ia = i / 16;
  const int ib = (i / 4) % 4;
  const int ig = i % 4;
  xi[0] = kGp4[ia];
  xi[1] = kGp4[ib];
  xi[2] = kGp4[ig];
}

double GaussLegendreHexQuadrature4::weight(int i) const
{
  const int ia = i / 16;
  const int ib = (i / 4) % 4;
  const int ig = i % 4;
  return kGw4[ia] * kGw4[ib] * kGw4[ig];
}

}  // namespace SolidDeformationModel
}  // namespace pgo
