/*
copyright to Bohan Wang
*/

#pragma once

#include "mappedSurfacePotentialEnergy.h"

#include <limits>

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

class FloorContactEnergy : public IPC::MappedSurfacePotentialEnergy
{
public:
  FloorContactEnergy(
    const EigenSupport::MXd &surfaceRestVertices,
    const EigenSupport::SpMatD &surfaceFromSimulationDispMap,
    const FloorPenaltyParameters &params);

  void setFloorHeight(double h);
  double floorHeight() const;
  virtual ContactModelKind contactModelKind() const override { return ContactModelKind::Floor; }

protected:
  virtual double computeSurfaceEnergy(EigenSupport::ConstRefVecXd surfacePositions) const override;
  virtual void computeSurfaceGradient(
    EigenSupport::ConstRefVecXd surfacePositions,
    EigenSupport::RefVecXd surfaceGradient) const override;
  virtual void computeSurfaceHessian(
    EigenSupport::ConstRefVecXd surfacePositions,
    EigenSupport::SpMatD &surfaceHessian) const override;

private:
  FloorPenaltyParameters params_;
};

}  // namespace Floor
}  // namespace Contact
}  // namespace pgo
