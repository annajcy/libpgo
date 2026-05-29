#pragma once

#include "../../deformationModel.h"
#include "../kernels/deformationGradientKernel.h"

#include "EigenSupport.h"

#include <array>
#include <vector>

namespace pgo
{
namespace SolidDeformationModel
{

// Per-element cache data for DeformationGradientElementModel<Kernel>.
// Holds current positions, plastic state, and per-quadrature-point kinematics.

template<class Kernel>
struct DeformationGradientElementModelCacheData : public DeformationModelCacheData
{
  static constexpr int numQuadPts = Kernel::numQuadPts;
  static constexpr int localDofs = Kernel::localDofs;

  using M3xN = typename Kernel::M3xN;
  using M9xNDOF = typename Kernel::M9xNDOF;

  int numPlasticParams = 0;
  int numElasticParams = 0;

  // Current positions.
  M3xN x;

  // Plastic state.
  ES::M3d Fp = ES::M3d::Identity();
  ES::M3d FpInv = ES::M3d::Identity();
  double detFp = 1.0;

  ES::VXd plasticParam;
  ES::VXd ddetA_da;
  ES::MXd d2detA_da2;
  std::vector<ES::M3d> dAInv_dai;
  std::vector<ES::M3d> d2AInv_dai_daj;

  // Per-quadrature-point data.
  std::array<ES::M3d, numQuadPts> Fref;
  std::array<ES::M3d, numQuadPts> Fe;
  std::array<ES::M3d, numQuadPts> U, V;
  std::array<ES::V3d, numQuadPts> S;
  std::array<M9xNDOF, numQuadPts> dFdx;
  std::array<M3xN, numQuadPts> Bm;

  ES::VXd materialParam;

  DeformationGradientElementModelCacheData(int np, int ne):
    numPlasticParams(np),
    numElasticParams(ne),
    plasticParam(ES::VXd::Zero(np)),
    ddetA_da(ES::VXd::Zero(np)),
    d2detA_da2(ES::MXd::Zero(np, np)),
    dAInv_dai(np, ES::M3d::Zero()),
    d2AInv_dai_daj(np * np, ES::M3d::Zero()),
    materialParam(ES::VXd::Zero(ne))
  {}

  ES::M3d &d2AInv(int i, int j) { return d2AInv_dai_daj[i * numPlasticParams + j]; }
  const ES::M3d &d2AInv(int i, int j) const { return d2AInv_dai_daj[i * numPlasticParams + j]; }
};

}  // namespace SolidDeformationModel
}  // namespace pgo
