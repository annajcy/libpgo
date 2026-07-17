#include "solver/newton/newtonThreadingPolicy.h"

#include "parallel/arenaThreadingExecutor.h"

#include <stdexcept>
#include <utility>

namespace pgo::NonlinearOptimization
{

NewtonThreadingPolicy::NewtonThreadingPolicy(
  std::shared_ptr<parallel::ArenaThreadingExecutor> evaluationExecutor,
  std::shared_ptr<parallel::ArenaThreadingExecutor> linearSolverExecutor):
  evaluationExecutor_(std::move(evaluationExecutor)),
  linearSolverExecutor_(std::move(linearSolverExecutor))
{
  if (!evaluationExecutor_)
    throw std::invalid_argument("Newton evaluation executor must not be null");
  if (!linearSolverExecutor_)
    throw std::invalid_argument("Newton linear-solver executor must not be null");
}

void NewtonThreadingPolicy::executeEvaluation(const std::function<void()> &fn) const
{
  evaluationExecutor_->execute(fn);
}

void NewtonThreadingPolicy::executeLinearSolver(const std::function<void()> &fn) const
{
  linearSolverExecutor_->execute(fn);
}

}  // namespace pgo::NonlinearOptimization
