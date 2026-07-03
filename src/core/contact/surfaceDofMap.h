/*
  Shared surface DOF mapping for contact energies.
*/

#pragma once

#include "EigenDef.h"

#include <cstdint>
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
  struct RowNnzStats
  {
    std::uint64_t min = 0;
    std::uint64_t max = 0;
    std::uint64_t total = 0;
    std::uint64_t nonzeroRows = 0;
  };

  struct SurfaceMapRowEntry
  {
    Eigen::Index simulationCol = 0;
    double weight = 0.0;
  };

  struct SimulationMapRowEntry
  {
    Eigen::Index surfaceRow = 0;
    double weight = 0.0;
  };

  void validateSimulationDisplacementSize(EigenSupport::ConstRefVecXd simulationDisplacements) const;
  void validateSurfaceVectorSize(EigenSupport::ConstRefVecXd surfaceVector) const;
  void buildSurfaceFromSimulationDispMapRows();
  void parallelSurfaceMapVectorMultiply(
    EigenSupport::ConstRefVecXd simulationDisplacements,
    EigenSupport::RefVecXd surfaceDisplacements) const;
  void parallelTransposeMapVectorMultiply(
    EigenSupport::ConstRefVecXd surfaceVector,
    EigenSupport::RefVecXd simulationVector) const;
  void parallelSurfaceHessianMapMultiply(
    const EigenSupport::SpMatD &surfaceHessian,
    EigenSupport::SpMatD &tmp) const;
  void parallelTransposeMapMultiply(
    const EigenSupport::SpMatD &tmp,
    EigenSupport::SpMatD &simulationHessian) const;
  static RowNnzStats computeRowNnzStats(const EigenSupport::SpMatD &matrix);

  EigenSupport::VXd surfaceRestPositions_;
  EigenSupport::SpMatD surfaceFromSimulationDispMap_;
  RowNnzStats surfaceFromSimulationDispMapRowNnzStats_;
  std::vector<std::vector<SurfaceMapRowEntry>> surfaceFromSimulationDispMapRows_;
  std::vector<std::vector<SimulationMapRowEntry>> simulationToSurfaceDispMapRows_;
  std::vector<int> simulationDofs_;
};

}  // namespace Contact
}  // namespace pgo
