#include "app/logging.h"

#include "solver/common/solveDiagnostics.h"
#include "pgoLogging.h"
#include "app/app.h"
#include "app/config.h"
#include "app/output.h"
#include "cli/cliLogging.h"
#include "scopedProfileSection.h"

#include <vector>

namespace pgo::RunIPCSim
{
namespace
{
void logProfileSummary()
{
  auto logger = pgo::Logging::lgr();
  if (!logger || !logger->should_log(spdlog::level::info))
    return;

  const std::vector<pgo::Profiling::ProfileStat> stats = pgo::Profiling::snapshotProfileStatistics();
  SPDLOG_LOGGER_INFO(logger, "runIPCSim profiling summary:");
  for (const pgo::Profiling::ProfileStat &stat : stats) {
    SPDLOG_LOGGER_INFO(logger,
      "profile name={} callCount={} totalSeconds={} maxSeconds={}",
      stat.name, stat.callCount, stat.totalSeconds, stat.maxSeconds);
  }

  const std::vector<pgo::Profiling::ProfileCounterStat> counterStats = pgo::Profiling::snapshotProfileCounterStatistics();
  for (const pgo::Profiling::ProfileCounterStat &stat : counterStats) {
    SPDLOG_LOGGER_INFO(logger,
      "profileCounter name={} sampleCount={} total={} max={}",
      stat.name, stat.sampleCount, stat.total, stat.max);
  }
}
}  // namespace

RunIPCSimRunScope::RunIPCSimRunScope(const pgo::ConfigFileJSON &config,
  const RunIPCSimRuntimeConfig &runtimeConfig,
  const RunIPCSimOptions &options,
  const RunIPCSimOutput &output):
  profilingEnabled_(runtimeConfig.enableProfiling)
{
  if (options.enableCliLog)
    logRedirect_ = std::make_unique<RunSim::ScopedRunSimCliLogRedirect>(output.logPath().string());

  pgo::Logging::init(nullptr, RunSim::resolveConfiguredLogLevel(config));

  if (profilingEnabled_) {
    pgo::Profiling::setProfilingEnabled(true);
    pgo::Profiling::resetProfileStatistics();
  }
}

RunIPCSimRunScope::~RunIPCSimRunScope()
{
  if (profilingEnabled_) {
    pgo::Profiling::setProfilingEnabled(false);
    pgo::Profiling::resetProfileStatistics();
  }
}

void RunIPCSimRunScope::logProfileSummaryIfEnabled() const
{
  if (profilingEnabled_)
    logProfileSummary();
}

void logRunIPCSimMaxStepSummary(
  const pgo::NonlinearOptimization::SolveDiagnostics &summary)
{
  auto logger = pgo::Logging::lgr();
  if (!logger)
    return;

  if (logger->should_log(spdlog::level::info)) {
    SPDLOG_LOGGER_INFO(logger,
      "runIPCSim max-step summary: materialClampCount={} contactClampCount={} minMaterialFeasibleAlphaThisSolve={} minContactFeasibleAlphaThisSolve={} minFeasibleAlphaThisSolve={} minLineSearchAlphaThisSolve={} minEffectiveAlphaThisSolve={}",
      summary.clampCounts[static_cast<int>(pgo::StepSource::Material)],
      summary.clampCounts[static_cast<int>(pgo::StepSource::Contact)],
      summary.minSourceFeasibleAlpha[static_cast<int>(pgo::StepSource::Material)],
      summary.minSourceFeasibleAlpha[static_cast<int>(pgo::StepSource::Contact)],
      summary.minFeasibleAlpha,
      summary.minLineSearchAlpha,
      summary.minEffectiveAlpha);
  }
}
}  // namespace pgo::RunIPCSim
