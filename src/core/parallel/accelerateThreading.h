#pragma once

#include <utility>

namespace pgo::parallel::detail
{

// Accelerate stores its BLAS/LAPACK threading mode in OS-thread-local state. Managed callbacks use
// this sticky operation so a TBB worker remains safe for later nested BLAS calls.
void setAccelerateSingleThreading();

class ScopedAccelerateMultiThreading
{
public:
  ScopedAccelerateMultiThreading();
  ~ScopedAccelerateMultiThreading() noexcept;

  ScopedAccelerateMultiThreading(const ScopedAccelerateMultiThreading &) = delete;
  ScopedAccelerateMultiThreading &operator=(const ScopedAccelerateMultiThreading &) = delete;

private:
#if defined(__APPLE__)
  unsigned int previous_ = 0;
  bool changed_ = false;
#endif
};

}  // namespace pgo::parallel::detail

namespace pgo::parallel::experimental
{

template<class Fn>
decltype(auto) withMultiThreadedAccelerate(Fn &&fn)
{
  detail::ScopedAccelerateMultiThreading scope;
  return std::forward<Fn>(fn)();
}

}  // namespace pgo::parallel::experimental
