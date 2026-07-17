#pragma once

#include <functional>
#include <memory>

namespace pgo::parallel
{
class ArenaThreadingExecutor;
}

namespace pgo::NonlinearOptimization
{

// Immutable phase-to-executor mapping for Newton solves. A null policy on the
// optimizer preserves the caller's current execution context; a concrete policy
// always owns both phase executors so evaluation and linear-solver work cannot
// accidentally inherit different unspecified defaults.
class NewtonThreadingPolicy final
{
public:
  NewtonThreadingPolicy(
    std::shared_ptr<parallel::ArenaThreadingExecutor> evaluationExecutor,
    std::shared_ptr<parallel::ArenaThreadingExecutor> linearSolverExecutor);

  void executeEvaluation(const std::function<void()> &fn) const;
  void executeLinearSolver(const std::function<void()> &fn) const;

private:
  std::shared_ptr<parallel::ArenaThreadingExecutor> evaluationExecutor_;
  std::shared_ptr<parallel::ArenaThreadingExecutor> linearSolverExecutor_;
};

}  // namespace pgo::NonlinearOptimization
