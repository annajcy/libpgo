/*
copyright to Bohan Wang
*/

#pragma once

#include "statefulContactEnergy.h"
#include "surfaceDofMap.h"

#include <limits>
#include <vector>

namespace pgo
{
namespace Contact
{
namespace Floor
{

enum class FloorAxis : int
{
  INVALID = -1,
  X = 0,
  Y = 1,
  Z = 2
};

// Which side of the floor plane the surface is constrained to. The plane sits
// at floorHeight along floorAxis; the penalty activates once a vertex crosses to
// the forbidden side.
enum class FloorSide
{
  KEEP_ABOVE,  // surface must stay at or above floorHeight (floor / ground)
  KEEP_BELOW,  // surface must stay at or below floorHeight (ceiling)
};

struct FloorPenaltyParameters
{
  FloorAxis floorAxis = FloorAxis::INVALID;
  FloorSide floorSide = FloorSide::KEEP_ABOVE;
  double floorHeight = std::numeric_limits<double>::quiet_NaN();
  double floorKappa = std::numeric_limits<double>::quiet_NaN();
};

class FloorContactEnergy : public StatefulContactEnergy
{
public:
  FloorContactEnergy(
    const EigenSupport::MXd &surfaceRestVertices,
    const EigenSupport::SpMatD &surfaceFromSimulationDispMap,
    const FloorPenaltyParameters &params);

  virtual double func(EigenSupport::ConstRefVecXd simulationDisplacements) const override;
  virtual void gradient(
    EigenSupport::ConstRefVecXd simulationDisplacements,
    EigenSupport::RefVecXd simulationGradient) const override;
  virtual void hessian(
    EigenSupport::ConstRefVecXd simulationDisplacements,
    EigenSupport::SpMatD &simulationHessian) const override;
  virtual void hessianInPlace(
    EigenSupport::ConstRefVecXd simulationDisplacements,
    EigenSupport::SpMatD &simulationHessian) const override;
  virtual void hessianAlloc(EigenSupport::SpMatD &simulationHessian) const override;
  virtual double funcGradient(
    EigenSupport::ConstRefVecXd simulationDisplacements,
    EigenSupport::RefVecXd simulationGradient) const override;
  virtual double funcGradientHessian(
    EigenSupport::ConstRefVecXd simulationDisplacements,
    EigenSupport::RefVecXd simulationGradient,
    EigenSupport::SpMatD &simulationHessian) const override;
  virtual void gradientHessian(
    EigenSupport::ConstRefVecXd simulationDisplacements,
    EigenSupport::RefVecXd simulationGradient,
    EigenSupport::SpMatD &simulationHessian) const override;
  virtual void getDOFs(std::vector<int> &dofs) const override;
  virtual int getNumDOFs() const override;
  virtual int isHessianTopologyFixed() const override { return 0; }

  void setFloorHeight(double h);
  double floorHeight() const;
  virtual ContactModelKind contactModelKind() const override { return ContactModelKind::Floor; }

private:
  double computeSurfaceEnergy(EigenSupport::ConstRefVecXd surfacePositions) const;
  void computeSurfaceGradient(
    EigenSupport::ConstRefVecXd surfacePositions,
    EigenSupport::RefVecXd surfaceGradient) const;
  void computeSurfaceHessian(
    EigenSupport::ConstRefVecXd surfacePositions,
    EigenSupport::SpMatD &surfaceHessian) const;
  void computeSurfaceAll(
    EigenSupport::ConstRefVecXd surfacePositions,
    double &surfaceEnergy,
    EigenSupport::RefVecXd surfaceGradient,
    EigenSupport::SpMatD &surfaceHessian) const;

  SurfaceDofMap dofMap_;
  FloorPenaltyParameters params_;
};

}  // namespace Floor
}  // namespace Contact
}  // namespace pgo
