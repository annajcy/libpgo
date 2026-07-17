#pragma once

#include <cstddef>

namespace pgo::parallel::detail
{

// Diagnostics for tests and controlled maintenance. These are deliberately not part of the
// public parallel API; production callers should use drainRetiredArenaThreadingExecutorStates().
std::size_t collectRetiredArenaThreadingExecutorStates() noexcept;
std::size_t retiredArenaThreadingExecutorStateCount() noexcept;

}  // namespace pgo::parallel::detail
