#include "cubicLinearShapeFunction.h"

#include <stdexcept>

namespace pgo
{
namespace ES = pgo::EigenSupport;
namespace SolidDeformationModel
{

namespace
{
// Vertex coordinates in parametric (alpha, beta, gamma) space.
const ES::V8i kVertexAlpha = (ES::V8i() << 0, 1, 1, 0, 0, 1, 1, 0).finished();
const ES::V8i kVertexBeta = (ES::V8i() << 0, 0, 1, 1, 0, 0, 1, 1).finished();
const ES::V8i kVertexGamma = (ES::V8i() << 0, 0, 0, 0, 1, 1, 1, 1).finished();
}  // namespace

ES::V8d CubicLinearShapeFunction::compute_N(double alpha, double beta, double gamma) const
{
  ES::V8d N;
  for (int i = 0; i < 8; i++) {
    const double a = kVertexAlpha[i] ? alpha : (1.0 - alpha);
    const double b = kVertexBeta[i] ? beta : (1.0 - beta);
    const double g = kVertexGamma[i] ? gamma : (1.0 - gamma);
    N[i] = a * b * g;
  }
  return N;
}

ES::M3x8d CubicLinearShapeFunction::compute_dN_dxi(double alpha, double beta, double gamma) const
{
  ES::M3x8d dN_dxi;
  // Column-major 3x8: dN_dxi(deriv, node) = dN_dxi[deriv + 3 * node]
  for (int i = 0; i < 8; i++) {
    const double alphaFactor = kVertexAlpha[i] ? alpha : (1.0 - alpha);
    const double betaFactor = kVertexBeta[i] ? beta : (1.0 - beta);
    const double gammaFactor = kVertexGamma[i] ? gamma : (1.0 - gamma);

    dN_dxi(0, i) = (kVertexAlpha[i] ? 1.0 : -1.0) * betaFactor * gammaFactor;
    dN_dxi(1, i) = (kVertexBeta[i] ? 1.0 : -1.0) * alphaFactor * gammaFactor;
    dN_dxi(2, i) = (kVertexGamma[i] ? 1.0 : -1.0) * alphaFactor * betaFactor;
  }
  return dN_dxi;
}

void CubicLinearShapeFunction::compute_N(double alpha, double beta, double gamma,
  ES::RefVecXd N_out) const
{
  if (N_out.size() != kNumNodes)
    throw std::invalid_argument("CubicLinearShapeFunction::compute_N output has the wrong size.");
  N_out = compute_N(alpha, beta, gamma);
}

void CubicLinearShapeFunction::compute_dN_dxi(double alpha, double beta, double gamma,
  ES::RefMatXd dN_dxi) const
{
  if (dN_dxi.rows() != 3 || dN_dxi.cols() != kNumNodes)
    throw std::invalid_argument("CubicLinearShapeFunction::compute_dN_dxi output has the wrong shape.");
  dN_dxi = compute_dN_dxi(alpha, beta, gamma);
}

ES::V3d CubicLinearShapeFunction::nodeCoords(int node) const
{
  ES::V3d xi = ES::V3d::Zero();
  if (node >= 0 && node < 8) {
    xi << static_cast<double>(kVertexAlpha[node]),
      static_cast<double>(kVertexBeta[node]),
      static_cast<double>(kVertexGamma[node]);
  }
  return xi;
}

}  // namespace SolidDeformationModel
}  // namespace pgo
