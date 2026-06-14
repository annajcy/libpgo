/*
  Shared surface DOF mapping for contact energies.
*/

#pragma once

#include "EigenDef.h"

#include <vector>

namespace pgo
{
namespace Contact
{

class SurfaceDofMap
{
public:
  SurfaceDofMap(
    const EigenSupport::MXd &surfaceRestVertices,
    const EigenSupport::SpMatD &surfaceFromSimulationDispMap);

  int numSimulationDofs() const { return static_cast<int>(simulationDofs_.size()); }
  int numSurfaceDofs() const { return static_cast<int>(surfaceRestPositions_.size()); }
  const std::vector<int> &simulationDofs() const { return simulationDofs_; }

  EigenSupport::VXd surfaceDisplacements(EigenSupport::ConstRefVecXd simulationDisplacements) const;
  EigenSupport::VXd surfacePositions(EigenSupport::ConstRefVecXd simulationDisplacements) const;
  EigenSupport::VXd pullbackGradient(EigenSupport::ConstRefVecXd surfaceGradient) const;
  void pullbackHessian(const EigenSupport::SpMatD &surfaceHessian, EigenSupport::SpMatD &simulationHessian) const;

private:
  void validateSimulationDisplacementSize(EigenSupport::ConstRefVecXd simulationDisplacements) const;
  void validateSurfaceVectorSize(EigenSupport::ConstRefVecXd surfaceVector) const;

  EigenSupport::VXd surfaceRestPositions_;
  EigenSupport::SpMatD surfaceFromSimulationDispMap_;
  std::vector<int> simulationDofs_;
};

}  // namespace Contact
}  // namespace pgo
