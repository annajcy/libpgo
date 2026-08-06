/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#include "material/plastic/plasticModel3DConstant.h"

#include "EigenSupport.h"

namespace ES = pgo::EigenSupport;
using namespace pgo::SolidDeformationModel;

PlasticModel3DConstant::PlasticModel3DConstant(const ES::M3d &Fp_):
  PlasticModel3DDeformationGradient(),
  Fp(Fp_),
  FpInv(Fp_.fullPivLu().inverse()),
  detFp(Fp_.determinant())
{
}

ES::M3d PlasticModel3DConstant::computeA(std::span<const double>) const
{
  return Fp;
}

ES::M3d PlasticModel3DConstant::computeAInv(std::span<const double>) const
{
  return FpInv;
}

ES::M3d PlasticModel3DConstant::defaultFp() const
{
  return Fp;
}

ES::M3d PlasticModel3DConstant::computeR(std::span<const double>) const
{
  return ES::M3d::Identity();
}


#include <stdexcept>

namespace pgo::SolidDeformationModel {
std::unique_ptr<PlasticModel> VolumetricPlasticity0Definition::createModel(std::span<const double> values, const MaterialFrame &) const
{
  requireFixedChannels(values, 0, id());
  return std::make_unique<PlasticModel3DConstant>(ES::M3d::Identity());
}
}  // namespace pgo::SolidDeformationModel
