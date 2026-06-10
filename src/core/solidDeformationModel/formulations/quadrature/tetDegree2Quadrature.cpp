#include "tetDegree2Quadrature.h"

namespace pgo
{
namespace SolidDeformationModel
{
namespace
{
// Barycentric (a, b, b, b) permutations with a + 3b = 1.
constexpr double kA = 0.5854101966249685;
constexpr double kB = 0.1381966011250105;
}  // namespace

void TetDegree2Quadrature::point(int i, double xi[3]) const
{
  xi[0] = kB;
  xi[1] = kB;
  xi[2] = kB;
  if (i < 3) {
    xi[i] = kA;
  }
  // i == 3: (b, b, b); the implicit 4th barycentric coordinate is a.
}

double TetDegree2Quadrature::weight(int /*i*/) const
{
  return 1.0 / 24.0;
}

}  // namespace SolidDeformationModel
}  // namespace pgo
