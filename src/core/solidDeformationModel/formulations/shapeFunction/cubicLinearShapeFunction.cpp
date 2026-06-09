#include "cubicLinearShapeFunction.h"

namespace pgo
{
namespace SolidDeformationModel
{

namespace
{
// Vertex coordinates in parametric (alpha, beta, gamma) space.
const int kVertexAlpha[8] = { 0, 1, 1, 0, 0, 1, 1, 0 };
const int kVertexBeta[8] = { 0, 0, 1, 1, 0, 0, 1, 1 };
const int kVertexGamma[8] = { 0, 0, 0, 0, 1, 1, 1, 1 };
}  // namespace

void CubicLinearShapeFunction::N(double alpha, double beta, double gamma, double N_out[]) const
{
  for (int i = 0; i < 8; i++) {
    const double a = kVertexAlpha[i] ? alpha : (1.0 - alpha);
    const double b = kVertexBeta[i] ? beta : (1.0 - beta);
    const double g = kVertexGamma[i] ? gamma : (1.0 - gamma);
    N_out[i] = a * b * g;
  }
}

void CubicLinearShapeFunction::dN_dxi(double alpha, double beta, double gamma, double dN_dxi[]) const
{
  // Column-major 3x8: dN_dxi(deriv, node) = dN_dxi[deriv + 3 * node]
  for (int i = 0; i < 8; i++) {
    const double alphaFactor = kVertexAlpha[i] ? alpha : (1.0 - alpha);
    const double betaFactor = kVertexBeta[i] ? beta : (1.0 - beta);
    const double gammaFactor = kVertexGamma[i] ? gamma : (1.0 - gamma);

    dN_dxi[0 + 3 * i] = (kVertexAlpha[i] ? 1.0 : -1.0) * betaFactor * gammaFactor;
    dN_dxi[1 + 3 * i] = (kVertexBeta[i] ? 1.0 : -1.0) * alphaFactor * gammaFactor;
    dN_dxi[2 + 3 * i] = (kVertexGamma[i] ? 1.0 : -1.0) * alphaFactor * betaFactor;
  }
}

void CubicLinearShapeFunction::nodeCoords(int node, double xi[3]) const
{
  if (node >= 0 && node < 8) {
    xi[0] = static_cast<double>(kVertexAlpha[node]);
    xi[1] = static_cast<double>(kVertexBeta[node]);
    xi[2] = static_cast<double>(kVertexGamma[node]);
  }
}

}  // namespace SolidDeformationModel
}  // namespace pgo
