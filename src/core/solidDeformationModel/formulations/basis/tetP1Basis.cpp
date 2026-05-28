#include "tetP1Basis.h"

namespace pgo
{
namespace SolidDeformationModel
{

void TetP1Basis::N(double xi, double eta, double zeta, double N[4])
{
  (void)xi;
  (void)eta;
  (void)zeta;
  N[0] = 1.0 - xi - eta - zeta;
  N[1] = xi;
  N[2] = eta;
  N[3] = zeta;
}

void TetP1Basis::dN_dxi(double xi, double eta, double zeta, double dN_dxi[12])
{
  (void)xi;
  (void)eta;
  (void)zeta;
  // Column-major 3x4: dN_dxi(deriv, node) = dN_dxi[deriv + 3 * node]
  //
  // Node 0: dN0/dxi = (-1, -1, -1)
  dN_dxi[0] = -1.0;   // d/dxi
  dN_dxi[1] = -1.0;   // d/deta
  dN_dxi[2] = -1.0;   // d/dzeta
  // Node 1: dN1/dxi = (1, 0, 0)
  dN_dxi[3] = 1.0;
  dN_dxi[4] = 0.0;
  dN_dxi[5] = 0.0;
  // Node 2: dN2/dxi = (0, 1, 0)
  dN_dxi[6] = 0.0;
  dN_dxi[7] = 1.0;
  dN_dxi[8] = 0.0;
  // Node 3: dN3/dxi = (0, 0, 1)
  dN_dxi[9] = 0.0;
  dN_dxi[10] = 0.0;
  dN_dxi[11] = 1.0;
}

void TetP1Basis::nodeCoords(int node, double xi[3])
{
  switch (node) {
  case 0: xi[0] = 0.0; xi[1] = 0.0; xi[2] = 0.0; break;
  case 1: xi[0] = 1.0; xi[1] = 0.0; xi[2] = 0.0; break;
  case 2: xi[0] = 0.0; xi[1] = 1.0; xi[2] = 0.0; break;
  case 3: xi[0] = 0.0; xi[1] = 0.0; xi[2] = 1.0; break;
  default: xi[0] = xi[1] = xi[2] = 0.0; break;
  }
}

}  // namespace SolidDeformationModel
}  // namespace pgo
