#include "parallel/accelerateThreading.h"

#if defined(__APPLE__)
#  include <Accelerate/Accelerate.h>

#  include <cassert>
#  include <stdexcept>
#endif

namespace pgo::parallel::detail
{

void setAccelerateSingleThreading()
{
#if defined(__APPLE__)
  if (BLASSetThreading(BLAS_THREADING_SINGLE_THREADED) != 0) {
    throw std::runtime_error("Accelerate does not support single-threaded BLAS/LAPACK mode.");
  }
#endif
}

ScopedAccelerateMultiThreading::ScopedAccelerateMultiThreading()
{
#if defined(__APPLE__)
  const BLAS_THREADING previous = BLASGetThreading();
  previous_ = static_cast<unsigned int>(previous);
  changed_ = previous != BLAS_THREADING_MULTI_THREADED;
  if (changed_ && BLASSetThreading(BLAS_THREADING_MULTI_THREADED) != 0) {
    changed_ = false;
    throw std::runtime_error("Accelerate does not support multi-threaded BLAS/LAPACK mode.");
  }
#endif
}

ScopedAccelerateMultiThreading::~ScopedAccelerateMultiThreading() noexcept
{
#if defined(__APPLE__)
  if (changed_) {
    const int result = BLASSetThreading(static_cast<BLAS_THREADING>(previous_));
    assert(result == 0);
    (void)result;
  }
#endif
}

}  // namespace pgo::parallel::detail
