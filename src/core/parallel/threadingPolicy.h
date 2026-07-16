#pragma once

#include <optional>

namespace pgo::parallel
{

enum class AccelerateThreading
{
  single,
  multi,
};

// A cross-platform bundle of persistent OS-thread-local backend settings.
// A missing field leaves that backend unchanged when setThreadingPolicy() is called.
struct ThreadingPolicy
{
  // 0 resets oneMKL's local override to its global setting; positive values establish a local
  // thread budget. Negative values are invalid on every platform.
  std::optional<int> mklLocalThreadBudget;
  std::optional<AccelerateThreading> accelerate;
};

// Persistently applies the active backend fields to the calling OS thread. No previous state is
// saved or restored.
void setThreadingPolicy(const ThreadingPolicy &policy);

namespace detail
{

void validateThreadingPolicy(const ThreadingPolicy &policy);
void validateCompleteThreadingPolicy(const ThreadingPolicy &policy);
void applyThreadingPolicyNoexcept(const ThreadingPolicy &policy) noexcept;

}  // namespace detail
}  // namespace pgo::parallel
