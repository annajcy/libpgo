#include "contact/contactBackend.h"

#include "dynamicStepOptions.h"
#include "floor/floorContactEnergy.h"
#include "ipc/ipcContactEnergy.h"
#include "energySet.h"
#include "app/config.h"
#include "app/logging.h"
#include "app/session.h"
#include "setup/setup.h"

namespace pgo::RunIPCSim
{
class IpcContactBackend final : public RunIPCSimContactBackend
{
public:
  ContactBackendKind kind() const override { return ContactBackendKind::Ipc; }
  std::string description() const override { return "ipc"; }

  void initializeAfterRestart(const RunIPCSimRuntimeConfig &, IpcSimulationContext &, RunIPCSimSession &) override {}

  void beginFrame(int frame, const RunIPCSimRuntimeConfig &, IpcSimulationContext &context, RunIPCSimSession &) override
  {
    for (std::size_t fi = 0; fi < context.floorPotentialEnergies.size(); ++fi)
      context.floorPotentialEnergies[fi]->setFloorHeight(floorHeightAtFrame(context.floorMotionStates[fi], frame));
  }

  void addForces(int frame, const RunIPCSimRuntimeConfig &runtimeConfig,
    IpcSimulationContext &context, RunIPCSimSession &session) override
  {
    const double tCurr = static_cast<double>(frame) * runtimeConfig.timestep;
    context.collisionHandler->setMovingObstacleTime(tCurr + runtimeConfig.timestep);
    session.transientContactModels.push_back({context.collisionHandler, 0.0, 0.0});
    for (const auto &forceModel : context.extraGeneralImplicitForceModels)
      session.transientContactModels.push_back({forceModel, 0.0, 0.0});
  }

  void afterStep(int, const RunIPCSimRuntimeConfig &, IpcSimulationContext &, RunIPCSimSession &) override {}

  void addStaticEnergies(const RunIPCSimRuntimeConfig &runtimeConfig,
    IpcSimulationContext &context, std::vector<NonlinearOptimization::EnergySet::Term> &terms) override
  {
    const int finalFrame = runtimeConfig.numSimSteps > 0 ? runtimeConfig.numSimSteps - 1 : 0;
    for (std::size_t fi = 0; fi < context.floorPotentialEnergies.size(); ++fi)
      context.floorPotentialEnergies[fi]->setFloorHeight(floorHeightAtFrame(context.floorMotionStates[fi], finalFrame));

    const double staticObstacleTime = runtimeConfig.timestep * static_cast<double>(finalFrame);
    context.collisionHandler->setMovingObstacleTime(staticObstacleTime);
    terms.push_back({context.collisionHandler, 1.0});

    for (const auto &forceModel : context.extraGeneralImplicitForceModels)
      terms.push_back({forceModel, 1.0});
  }

  void logSummary(const IpcSimulationContext &, const RunIPCSimSession &session) const override
  {
    logRunIPCSimMaxStepSummary(session.lastDiagnostics);
  }
};

std::shared_ptr<RunIPCSimContactBackend> makeIpcContactBackend()
{
  return std::make_shared<IpcContactBackend>();
}
}  // namespace pgo::RunIPCSim
