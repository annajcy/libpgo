#pragma once

#include <memory>

namespace pgo
{
class ConfigFileJSON;

namespace NonlinearOptimization
{
struct SolveDiagnostics;
}

namespace RunSim
{
class ScopedRunSimCliLogRedirect;
}
}  // namespace pgo

namespace pgo::RunIPCSim
{
struct RunIPCSimOptions;
struct RunIPCSimRuntimeConfig;
class RunIPCSimOutput;

class RunIPCSimRunScope
{
public:
  RunIPCSimRunScope(const pgo::ConfigFileJSON &config,
    const RunIPCSimRuntimeConfig &runtimeConfig,
    const RunIPCSimOptions &options,
    const RunIPCSimOutput &output);
  ~RunIPCSimRunScope();

  RunIPCSimRunScope(const RunIPCSimRunScope &) = delete;
  RunIPCSimRunScope &operator=(const RunIPCSimRunScope &) = delete;

  void logProfileSummaryIfEnabled() const;

private:
  bool profilingEnabled_ = false;
  std::unique_ptr<RunSim::ScopedRunSimCliLogRedirect> logRedirect_;
};

void logRunIPCSimMaxStepSummary(
  const pgo::NonlinearOptimization::SolveDiagnostics &diagnostics);
}  // namespace pgo::RunIPCSim
