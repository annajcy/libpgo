#include "parallel/threadingPolicy.h"

#include <exception>
#include <stdexcept>

#if defined(PGO_HAS_MKL)
#  include <mkl.h>
#elif defined(__APPLE__)
#  include <Accelerate/Accelerate.h>
#endif

namespace pgo::parallel
{
namespace
{

void applyActiveBackendPolicy(const ThreadingPolicy &policy)
{
#if defined(PGO_HAS_MKL)
  if (policy.mklLocalThreadBudget.has_value())
    (void)mkl_set_num_threads_local(*policy.mklLocalThreadBudget);
#elif defined(__APPLE__)
  if (policy.accelerate.has_value()) {
    const BLAS_THREADING mode = *policy.accelerate == AccelerateThreading::single ? BLAS_THREADING_SINGLE_THREADED : BLAS_THREADING_MULTI_THREADED;
    if (BLASSetThreading(mode) != 0)
      throw std::runtime_error("Accelerate rejected the requested BLAS/LAPACK threading mode.");
  }
#else
  (void)policy;
#endif
}

}  // namespace

namespace detail
{

void validateThreadingPolicy(const ThreadingPolicy &policy)
{
  if (policy.mklLocalThreadBudget.has_value() && *policy.mklLocalThreadBudget < 0)
    throw std::invalid_argument("MKL local thread budget must be non-negative.");

  if (policy.accelerate.has_value()) {
    switch (*policy.accelerate) {
    case AccelerateThreading::single:
    case AccelerateThreading::multi:
      break;
    default:
      throw std::invalid_argument("Invalid Accelerate threading mode.");
    }
  }
}

void validateCompleteThreadingPolicy(const ThreadingPolicy &policy)
{
  validateThreadingPolicy(policy);
#if defined(PGO_HAS_MKL)
  if (!policy.mklLocalThreadBudget.has_value())
    throw std::invalid_argument("An MKL local thread budget is required for an arena executor.");
#elif defined(__APPLE__)
  if (!policy.accelerate.has_value())
    throw std::invalid_argument("An Accelerate threading mode is required for an arena executor.");
#endif
}

void applyThreadingPolicyNoexcept(const ThreadingPolicy &policy) noexcept
{
  try {
    applyActiveBackendPolicy(policy);
  }
  catch (...) {
    std::terminate();
  }
}

}  // namespace detail

void setThreadingPolicy(const ThreadingPolicy &policy)
{
  detail::validateThreadingPolicy(policy);
  applyActiveBackendPolicy(policy);
}

}  // namespace pgo::parallel
