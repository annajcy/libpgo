#pragma once

#include "floor/floorContactEnergy.h"
#include "setup/setup.h"

#include <vector>

namespace pgo
{
class ConfigFileJSON;
}

namespace pgo::RunIPCSim
{
struct ParsedFloorConfig
{
  Contact::Floor::FloorPenaltyParameters params;
  IpcFloorMotionState motionState;
};

const char *floorAxisToString(Contact::Floor::FloorAxis axis);
const char *floorSideToString(Contact::Floor::FloorSide side);
std::vector<ParsedFloorConfig> parseFloorsConfig(const pgo::ConfigFileJSON &jconfig);
}  // namespace pgo::RunIPCSim
