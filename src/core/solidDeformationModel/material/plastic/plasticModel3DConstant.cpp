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
MaterialChannelSchema VolumetricPlasticity0Definition::optimizableChannelSchema() const { return {}; }
std::unique_ptr<PlasticModel> VolumetricPlasticity0Definition::createModel(std::span<const double> values, const MaterialFrame &) const
{
  if (!values.empty()) throw std::invalid_argument("volumetric_dof0 has no fixed channels");
  return std::make_unique<PlasticModel3DConstant>(ES::M3d::Identity());
}
}  // namespace pgo::SolidDeformationModel
