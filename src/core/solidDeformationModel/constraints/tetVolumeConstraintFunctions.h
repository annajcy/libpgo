/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#pragma once

#include "constraints/constraintFunctions.h"
#include "simulation/simulationMesh.h"

#include <tbb/spin_mutex.h>

#include <memory>
#include <functional>
#include <optional>
#include <span>
#include <vector>

class TetMesh;

namespace pgo
{
namespace SolidDeformationModel
{
class TetVolumeConstraintFunctions : public NonlinearOptimization::ConstraintFunctions
{
public:
  TetVolumeConstraintFunctions(const SimulationMesh &tetMesh, int nAll,
    std::optional<std::reference_wrapper<const EigenSupport::VXd>> restPosition = std::nullopt,
    std::optional<std::reference_wrapper<const EigenSupport::M3Xd>> DmInv = std::nullopt);
  virtual ~TetVolumeConstraintFunctions();

  void setDmInv(const EigenSupport::M3Xd &DmInv_);
  void setElementFlags(std::span<const int> flags);

  virtual void func(EigenSupport::ConstRefVecXd x, EigenSupport::RefVecXd g) const override;
  virtual void jacobian(EigenSupport::ConstRefVecXd x, EigenSupport::SpMatD &jac) const override;
  virtual void hessianInPlace(EigenSupport::ConstRefVecXd x, EigenSupport::ConstRefVecXd lambda, EigenSupport::SpMatD &hess) const override;

  virtual bool isLinear() const override { return false; }
  virtual bool isQuadratic() const override { return false; }
  virtual bool hasHessianVector() const override { return false; }

protected:
  const SimulationMesh &tetMesh;
  std::optional<std::reference_wrapper<const EigenSupport::VXd>> restPosition;

  EigenSupport::M3Xd DmInv;
  EigenSupport::EigenArray<EigenSupport::M9x12d> dFdx;

  typedef Eigen::Matrix<EigenSupport::IDX, 4, 1> JacIndex;
  typedef Eigen::Matrix<EigenSupport::IDX, 12, 12> HessIndex;

  EigenSupport::EigenArray<JacIndex> jacobianIndices;
  EigenSupport::EigenArray<HessIndex> hessIndices;
  std::vector<int> elementFlags;

  mutable std::vector<tbb::spin_mutex> hessLocks;

  int nele;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
