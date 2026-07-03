/*
author: Bohan Wang
copyright to USC, MIT
*/

#pragma once

#include "potentialEnergyAligningMeshConnectivity.h"

namespace pgo
{
namespace ConstraintPotentialEnergies
{
class MultipleVertexPulling : public PotentialEnergyAligningMeshConnectivity
{
public:
  // Owning by-value ctor.  Koff is the Hessian sparsity template (taken
  // from SimulationMesh); every input is copied/moved into owned storage.
  MultipleVertexPulling(
    EigenSupport::SpMatD Koff,
    EigenSupport::VXd restPositionsAll,
    std::vector<int> vertexIndices,
    EigenSupport::VXd targetPositions,
    double coeff = 1.0,
    bool isDisplacement = true);

  virtual double func(EigenSupport::ConstRefVecXd u) const override;
  virtual void gradient(EigenSupport::ConstRefVecXd u, EigenSupport::RefVecXd grad) const override;
  virtual void hessianInPlace(EigenSupport::ConstRefVecXd, EigenSupport::SpMatD &hess) const override;

  virtual NonlinearOptimization::EnergyStateKind stateKind() const override
  {
    return isDisplacement_
      ? NonlinearOptimization::EnergyStateKind::Displacement
      : NonlinearOptimization::EnergyStateKind::Generic;
  }

  void setCoeff(double v) { coeffAll_ = v; }
  void setTargetPositions(EigenSupport::VXd tgt);

  void printErrorInfo(EigenSupport::ConstRefVecXd u) const;

protected:
  typedef Eigen::Matrix<EigenSupport::IDX, 3, 3> M3i;
  std::vector<M3i, Eigen::aligned_allocator<M3i>> KIndices;

  EigenSupport::VXd tgtp_, restpAll_;
  std::vector<int> vertexIndices_;

  double coeffAll_ = 1.0;
  EigenSupport::VXd coeffs_, masks_;
  bool isDisplacement_ = true;
};
}  // namespace ConstraintPotentialEnergies
}  // namespace pgo
