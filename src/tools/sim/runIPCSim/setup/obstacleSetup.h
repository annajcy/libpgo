#pragma once

#include "ipc/external/obstacleSurface.h"

#include <memory>
#include <string>
#include <vector>

namespace pgo
{
class ConfigFileJSON;
}

namespace pgo::RunIPCSim
{
std::vector<std::unique_ptr<Contact::IPC::ObstacleSurface>> parseExternalObjects(
  const pgo::ConfigFileJSON &jconfig, double scale, std::vector<bool> *outStaticFlags = nullptr);
}  // namespace pgo::RunIPCSim
