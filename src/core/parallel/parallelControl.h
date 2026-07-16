#pragma once

#include <tbb/global_control.h>

namespace pgo::parallel
{

class GlobalTbbControl final
{
public:
  explicit GlobalTbbControl(int maxAllowedParallelism);
  ~GlobalTbbControl() noexcept = default;

  GlobalTbbControl(const GlobalTbbControl &) = delete;
  GlobalTbbControl &operator=(const GlobalTbbControl &) = delete;
  GlobalTbbControl(GlobalTbbControl &&) = delete;
  GlobalTbbControl &operator=(GlobalTbbControl &&) = delete;

private:
  tbb::global_control control_;
};

int resolveNonExpandingTbbConcurrency(int requestedConcurrency);

}  // namespace pgo::parallel
