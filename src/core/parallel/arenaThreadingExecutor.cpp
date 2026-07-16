#include "parallel/arenaThreadingExecutor.h"

#include <stdexcept>

#include <tbb/task_scheduler_observer.h>

namespace pgo::parallel
{
namespace
{

int validateMaxConcurrency(int value)
{
  if (value <= 0)
    throw std::invalid_argument("Arena max concurrency must be positive.");
  return value;
}

int validateReservedSlots(int reservedSlots, int maxConcurrency)
{
  if (reservedSlots < 0 || reservedSlots > maxConcurrency)
    throw std::invalid_argument("Arena reserved slots must be between zero and max concurrency.");
  return reservedSlots;
}

ThreadingPolicy validatePolicy(ThreadingPolicy policy)
{
  detail::validateCompleteThreadingPolicy(policy);
  return policy;
}

}  // namespace

class ArenaThreadingExecutor::Observer final : public tbb::task_scheduler_observer
{
public:
  Observer(tbb::task_arena &arena, const ThreadingPolicy &policy): tbb::task_scheduler_observer(arena), policy_(policy)
  {
    observe(true);
  }

  ~Observer() override
  {
    observe(false);
  }

private:
  void on_scheduler_entry(bool) override
  {
    detail::applyThreadingPolicyNoexcept(policy_);
  }

  const ThreadingPolicy &policy_;
};

ArenaThreadingExecutor::ArenaThreadingExecutor(int maxConcurrency, ThreadingPolicy policy,
  int reservedSlots, tbb::task_arena::priority priority): arena_(validateMaxConcurrency(maxConcurrency),
                                                            validateReservedSlots(reservedSlots, maxConcurrency), priority),
                                                          policy_(validatePolicy(std::move(policy))),
                                                          observer_(std::make_unique<Observer>(arena_, policy_))
{
}

ArenaThreadingExecutor::~ArenaThreadingExecutor() noexcept = default;

int ArenaThreadingExecutor::maxConcurrency() const noexcept
{
  return arena_.max_concurrency();
}

const ThreadingPolicy &ArenaThreadingExecutor::threadingPolicy() const noexcept
{
  return policy_;
}

}  // namespace pgo::parallel
