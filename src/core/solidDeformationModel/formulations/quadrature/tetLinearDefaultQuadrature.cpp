#include "tetLinearDefaultQuadrature.h"

namespace pgo
{
namespace SolidDeformationModel
{

void TetLinearDefaultQuadrature::point(int, double xi[3]) const
{
  xi[0] = 0.25;
  xi[1] = 0.25;
  xi[2] = 0.25;
}

double TetLinearDefaultQuadrature::weight(int) const
{
  return 1.0 / 6.0;
}

}  // namespace SolidDeformationModel
}  // namespace pgo
