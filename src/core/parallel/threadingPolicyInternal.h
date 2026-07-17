#pragma once

#include "parallel/threadingPolicy.h"

namespace pgo::parallel::detail
{

// Opaque, cross-platform snapshot of the active BLAS backend's calling-thread state.
// The integer preserves the backend's native value without exposing vendor headers here.
struct ThreadingPolicySnapshot
{
  int activeBackendValue = 0;
  bool hasActiveBackendValue = false;
};

void validateThreadingPolicy(const ThreadingPolicy &policy);
void validateCompleteThreadingPolicy(const ThreadingPolicy &policy);
void applyThreadingPolicyNoexcept(const ThreadingPolicy &policy) noexcept;
ThreadingPolicySnapshot exchangeThreadingPolicyNoexcept(const ThreadingPolicy &policy) noexcept;
void restoreThreadingPolicyNoexcept(const ThreadingPolicySnapshot &snapshot) noexcept;

}  // namespace pgo::parallel::detail
