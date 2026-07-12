#pragma once

#include "parallelOptions.h"

#include <memory>
#include <optional>

namespace pgo::parallel
{

class ParallelExecutor;
class ParallelRuntime;

namespace detail
{
struct ExecutorState;
const std::shared_ptr<ExecutorState> &executorState(const ParallelExecutor &executor);
}  // namespace detail

class ParallelExecutor
{
public:
  ParallelExecutor(const ParallelExecutor &) noexcept = default;
  ParallelExecutor(ParallelExecutor &&) noexcept = default;
  ParallelExecutor &operator=(const ParallelExecutor &) noexcept = default;
  ParallelExecutor &operator=(ParallelExecutor &&) noexcept = default;

  std::optional<int> maxConcurrency() const noexcept;

private:
  ParallelExecutor() = default;
  explicit ParallelExecutor(std::shared_ptr<detail::ExecutorState> state);

  std::shared_ptr<detail::ExecutorState> state_;

  friend class ParallelRuntime;
  friend const std::shared_ptr<detail::ExecutorState> &detail::executorState(const ParallelExecutor &executor);
};

}  // namespace pgo::parallel
