/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#include "material/plastic/plasticModel3D6DOF.h"
#include <array>

#include "EigenSupport.h"
#include "determinantDerivatives.h"

namespace ES = pgo::EigenSupport;
using namespace pgo::SolidDeformationModel;
using namespace pgo::NonlinearOptimization;

namespace
{
const std::array<ES::M3d, 6> &symmetricBasis()
{
  static const std::array<ES::M3d, 6> basis = [] {
    std::array<ES::M3d, 6> result;
    for (ES::M3d &entry : result)
      entry.setZero();
    result[0](0, 0) = 1.0;
    result[1](0, 1) = result[1](1, 0) = 1.0;
    result[2](0, 2) = result[2](2, 0) = 1.0;
    result[3](1, 1) = 1.0;
    result[4](1, 2) = result[4](2, 1) = 1.0;
    result[5](2, 2) = 1.0;
    return result;
  }();
  return basis;
}
}  // namespace

PlasticModel3D6DOF::PlasticModel3D6DOF():
  PlasticModel3DDeformationGradient()
{
  R = ES::M3d::Identity();
  RT = R.transpose();
}

PlasticModel3D6DOF::PlasticModel3D6DOF(const ES::M3d &referenceToMaterial):
  PlasticModel3DDeformationGradient(),
  R(referenceToMaterial),
  RT(referenceToMaterial.transpose())
{
}

inline ES::M3d getS(std::span<const double> param)
{
  ES::M3d S;
  S << param[0], param[1], param[2],
    param[1], param[3], param[4],
    param[2], param[4], param[5];

  return S;
}

ES::M3d PlasticModel3D6DOF::computeA(std::span<const double> param) const
{
  ES::M3d S = getS(param);
  return RT * S * R;
}

ES::M3d PlasticModel3D6DOF::computeAInv(std::span<const double> param) const
{
  ES::M3d S = getS(param);
  ES::M3d SInv = S.fullPivLu().inverse();
  return RT * SInv * R;
}

double PlasticModel3D6DOF::compute_detA(std::span<const double> param) const
{
  ES::M3d S = getS(param);
  return S.determinant();
}

void PlasticModel3D6DOF::compute_ddetA_da(
  std::span<const double> param, ES::RefVecXd ddetA_da) const
{
  Determinant::Dim3::ddetA_dA_sym(param.data(), ddetA_da.data());
}

void PlasticModel3D6DOF::compute_d2detA_da2(
  std::span<const double> param, ES::RefMatXd d2detA_da2) const
{
  ES::M6d deriv;
  Determinant::Dim3::d2detA_dA2_sym(param.data(), deriv.data());

  d2detA_da2 = deriv;
}

ES::M3d PlasticModel3D6DOF::compute_dAInv_da(std::span<const double> param, int pi) const
{
  ES::M3d S = getS(param);

  const auto &ei = symmetricBasis();

  ES::M3d SInv = S.fullPivLu().inverse();
  ES::M3d Z = -SInv * ei[pi] * SInv;
  return RT * Z * R;
}

ES::M3d PlasticModel3D6DOF::compute_d2AInv_da2(std::span<const double> param, int pi, int pj) const
{
  ES::M3d S = getS(param);

  const auto &ei = symmetricBasis();

  ES::M3d SInv = S.fullPivLu().inverse();

  ES::M3d Z = SInv * ei[pi] * SInv * ei[pj] * SInv +
    SInv * ei[pj] * SInv * ei[pi] * SInv;

  return RT * Z * R;
}

void PlasticModel3D6DOF::projectParam(std::span<double> param, double zeroThreshold) const
{
  const ES::M3d Fp = computeA(param);

  Eigen::SelfAdjointEigenSolver<ES::M3d> eig(Fp);
  ES::M3d R = eig.eigenvectors();
  ES::V3d S = eig.eigenvalues();
  ES::V3d Struncate = S.cwiseMax(ES::V3d::Constant(zeroThreshold));

  ES::M3d FpPrime = R * Struncate.asDiagonal() * R.transpose();

  param[0] = FpPrime(0, 0);
  param[1] = FpPrime(0, 1);
  param[2] = FpPrime(0, 2);

  param[3] = FpPrime(1, 1);
  param[4] = FpPrime(1, 2);

  param[5] = FpPrime(2, 2);
}

ES::M3d PlasticModel3D6DOF::computeR(std::span<const double> param) const
{
  const ES::M3d Fp = computeA(param);

  Eigen::SelfAdjointEigenSolver<ES::M3d> eig(Fp);
  ES::M3d R = eig.eigenvectors();

  return R;
}


#include <stdexcept>

namespace pgo::SolidDeformationModel {
MaterialChannelSchema VolumetricPlasticity6Definition::optimizableChannelSchema() const { static constexpr std::array<std::string_view, 6> names{"Fxx", "Fxy", "Fxz", "Fyy", "Fyz", "Fzz"}; return MaterialChannelSchema(names); }
std::unique_ptr<PlasticModel> VolumetricPlasticity6Definition::createModel(std::span<const double> values, const MaterialFrame &) const
{
  if (!values.empty()) throw std::invalid_argument("volumetric_dof6 has no fixed channels");
  return std::make_unique<PlasticModel3D6DOF>();
}
}  // namespace pgo::SolidDeformationModel
