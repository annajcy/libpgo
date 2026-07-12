#pragma once

#include "parallelExecutor.h"
#include "parallelOptions.h"

#include <memory>

namespace pgo::parallel
{

namespace detail
{
struct RuntimeState;
}

class ParallelRuntime
{
public:
  ParallelRuntime(const ParallelRuntime &) = delete;
  ParallelRuntime &operator=(const ParallelRuntime &) = delete;

  ParallelExecutor createExecutor(ExecutorOptions options = {});
  ParallelExecutor defaultExecutor() const;
  RuntimeInfo info() const;

private:
  ParallelRuntime(int resolvedMaxConcurrency, bool usingDefaultConcurrency);
  ~ParallelRuntime() = default;

  std::shared_ptr<detail::RuntimeState> state_;
  ParallelExecutor defaultExecutor_;

  friend ParallelRuntime &initializeRuntime(RuntimeOptions options);
};

ParallelRuntime &initializeRuntime(RuntimeOptions options = {});
ParallelRuntime &runtime();

// These diagnostics do not initialize the pgo runtime.
int defaultConcurrency();
RuntimeInfo runtimeInfo();

}  // namespace pgo::parallel
