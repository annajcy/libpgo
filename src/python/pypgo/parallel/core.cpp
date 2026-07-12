#include "core.h"

#include "parallelism/parallelRuntime.h"

namespace pgo
{
namespace
{

nanobind::dict runtimeInfoToDict(const parallel::RuntimeInfo &info)
{
  nanobind::dict dict;
  dict["initialized"] = info.initialized;
  dict["using_default_concurrency"] = info.usingDefaultConcurrency;
  if (info.maxConcurrency.has_value())
    dict["max_concurrency"] = *info.maxConcurrency;
  else
    dict["max_concurrency"] = nanobind::none();
  dict["default_concurrency"] = info.defaultConcurrency;
  dict["effective_tbb_max_allowed_parallelism"] = info.effectiveTbbMaxAllowedParallelism;
  dict["tbb_worker_ceiling"] = info.tbbWorkerCeiling;
  dict["current_worker_participants"] = info.currentWorkerParticipants;
  dict["current_external_participants"] = info.currentExternalParticipants;
  dict["current_total_participants"] = info.currentTotalParticipants;
  dict["peak_total_participants"] = info.peakTotalParticipants;
  dict["participant_pressure_observed"] = info.participantPressureObserved;
  return dict;
}

}  // namespace

int parallelDefaultConcurrency()
{
  return parallel::defaultConcurrency();
}

nanobind::dict parallelInitialize(std::optional<int> maxConcurrency)
{
  parallel::RuntimeOptions options;
  options.maxTbbConcurrency = maxConcurrency;
  return runtimeInfoToDict(parallel::initializeRuntime(options).info());
}

nanobind::dict parallelRuntimeInfo()
{
  return runtimeInfoToDict(parallel::runtimeInfo());
}

}  // namespace pgo
