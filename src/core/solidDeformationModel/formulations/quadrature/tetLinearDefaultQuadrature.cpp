#include "tetLinearDefaultQuadrature.h"

namespace pgo
{
namespace SolidDeformationModel
{

EigenSupport::V3d TetLinearDefaultQuadrature::point(int) const
{
  return EigenSupport::V3d::Constant(0.25);
}

double TetLinearDefaultQuadrature::weight(int) const
{
  return 1.0 / 6.0;
}

}  // namespace SolidDeformationModel
}  // namespace pgo
