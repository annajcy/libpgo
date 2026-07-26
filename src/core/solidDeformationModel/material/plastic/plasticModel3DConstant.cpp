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


#include "simulation/simulationMesh.h"
#include <algorithm>
#include <initializer_list>
#include <stdexcept>

namespace pgo::SolidDeformationModel {
namespace {
void expectSize(std::span<double> output, std::size_t expected) {
  if (output.size() != expected) throw std::invalid_argument("plastic config default parameter buffer has the wrong size");
}
}
std::span<const std::string_view> VolumetricPlasticity0Config::parameterChannelNames() const { return {}; }
void VolumetricPlasticity0Config::initializeDefaultElementChannels(const SimulationMesh &, int, std::span<double> output) const { expectSize(output, 0); }
std::unique_ptr<PlasticModel> VolumetricPlasticity0Config::createModel(const SimulationMesh &, int, const MaterialFrame &) const
{
  return std::make_unique<PlasticModel3DConstant>(ES::M3d::Identity());
}
}  // namespace pgo::SolidDeformationModel
