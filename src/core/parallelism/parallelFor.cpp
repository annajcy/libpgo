#include "parallelism/parallelFor.h"

#include "parallelInternal.h"

#include <memory>
#include <stdexcept>

#ifdef PGO_PARALLELISM_HAS_TBB
#  include <tbb/blocked_range.h>
#  include <tbb/parallel_for.h>
#endif

#ifdef PGO_PARALLELISM_HAS_MKL
#  include <mkl.h>
#endif

#ifdef PGO_PARALLELISM_HAS_ACCELERATE_THREADING
#  include <Accelerate/Accelerate.h>
#endif

namespace pgo::parallel
{
namespace
{

[[maybe_unused]] void runSerial(int begin, int end, const detail::ChunkBody &body)
{
  if (begin < end)
    body(begin, end);
}

template<class Fn>
void runNestedKernelPolicy(const Options &options, Fn &&fn)
{
  if (options.nestedKernelPolicy == NestedKernelPolicy::Suppress) {
    class ScopedNestedKernelSuppressor
    {
    public:
      ScopedNestedKernelSuppressor()
      {
#ifdef PGO_PARALLELISM_HAS_MKL
        previousMKLThreads_ = mkl_set_num_threads_local(1);
        shouldRestoreMKL_ = true;
#endif
#ifdef PGO_PARALLELISM_HAS_ACCELERATE_THREADING
        previousAccelerateThreading_ = static_cast<int>(BLASGetThreading());
        shouldRestoreAccelerate_ = BLASSetThreading(BLAS_THREADING_SINGLE_THREADED) == 0;
#endif
      }

      ~ScopedNestedKernelSuppressor()
      {
#ifdef PGO_PARALLELISM_HAS_ACCELERATE_THREADING
        if (shouldRestoreAccelerate_)
          BLASSetThreading(static_cast<BLAS_THREADING>(previousAccelerateThreading_));
#endif
#ifdef PGO_PARALLELISM_HAS_MKL
        if (shouldRestoreMKL_)
          mkl_set_num_threads_local(previousMKLThreads_);
#endif
      }

    private:
      [[maybe_unused]] int previousMKLThreads_ = 0;
      [[maybe_unused]] int previousAccelerateThreading_ = 0;
      [[maybe_unused]] bool shouldRestoreMKL_ = false;
      [[maybe_unused]] bool shouldRestoreAccelerate_ = false;
    } nestedKernelSuppressor;

    fn();
    return;
  }

  fn();
}

thread_local const detail::ExecutorState *currentExecutor = nullptr;

class CurrentExecutorScope
{
public:
  explicit CurrentExecutorScope(const detail::ExecutorState *executor): previous_(currentExecutor)
  {
    currentExecutor = executor;
  }

  ~CurrentExecutorScope()
  {
    currentExecutor = previous_;
  }

private:
  const detail::ExecutorState *previous_;
};

#ifdef PGO_PARALLELISM_HAS_TBB
void runTBB(const detail::ExecutorState *executor, int begin, int end,
  const Options &options, const detail::ChunkBody &body)
{
  const int grainSize = options.grainSize > 0 ? options.grainSize : 1;
  tbb::parallel_for(tbb::blocked_range<int>(begin, end, grainSize),
    [&](const tbb::blocked_range<int> &range) {
      CurrentExecutorScope scope(executor);
      runNestedKernelPolicy(options, [&] {
        body(range.begin(), range.end());
      });
    });
}
#endif

void runInCurrentExecutor(const detail::ExecutorState &executor, int begin, int end,
  const Options &options, const detail::ChunkBody &body)
{
#ifdef PGO_PARALLELISM_HAS_TBB
  runTBB(&executor, begin, end, options, body);
#else
  CurrentExecutorScope scope(&executor);
  runNestedKernelPolicy(options, [&] {
    runSerial(begin, end, body);
  });
#endif
}

void runWithExecutor(const std::shared_ptr<detail::ExecutorState> &executor, int begin, int end,
  const Options &options, const detail::ChunkBody &body)
{
  if (currentExecutor != nullptr) {
    if (currentExecutor != executor.get())
      throw std::logic_error("Cannot switch ParallelExecutor inside an active pgo parallel region.");
    if (begin < end)
      runInCurrentExecutor(*executor, begin, end, options, body);
    return;
  }

  if (begin >= end)
    return;

  detail::executeInArena(executor, [&](const detail::ExecutorState *enteredExecutor) {
    CurrentExecutorScope scope(enteredExecutor);
#ifdef PGO_PARALLELISM_HAS_TBB
    runTBB(enteredExecutor, begin, end, options, body);
#else
    runNestedKernelPolicy(options, [&] {
      runSerial(begin, end, body);
    });
#endif
  });
}

}  // namespace

namespace detail
{

void parallelForChunks(int begin, int end, const Options &options, const ChunkBody &body)
{
  if (currentExecutor != nullptr) {
    if (begin < end)
      runInCurrentExecutor(*currentExecutor, begin, end, options, body);
    return;
  }

  if (begin >= end)
    return;

  const ParallelExecutor executor = runtime().defaultExecutor();
  runWithExecutor(executorState(executor), begin, end, options, body);
}

void parallelForChunks(
  const ParallelExecutor &executor, int begin, int end, const Options &options, const ChunkBody &body)
{
  const std::shared_ptr<ExecutorState> state = executorState(executor);
  if (!state)
    throw std::invalid_argument("ParallelExecutor is empty.");
  runWithExecutor(state, begin, end, options, body);
}

}  // namespace detail
}  // namespace pgo::parallel
