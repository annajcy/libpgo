#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <climits>
#include <thread>

#ifdef __linux__
#  include <dirent.h>
#endif

#ifdef __APPLE__
#  include <mach/mach.h>
#endif

namespace pgo::benchmark_helpers
{

inline void updateMaximum(std::atomic<int> &target, int value) noexcept
{
  int observed = target.load(std::memory_order_relaxed);
  while (value > observed &&
    !target.compare_exchange_weak(observed, value, std::memory_order_relaxed)) {
  }
}

inline void updateMinimum(std::atomic<int> &target, int value) noexcept
{
  int observed = target.load(std::memory_order_relaxed);
  while (value < observed &&
    !target.compare_exchange_weak(observed, value, std::memory_order_relaxed)) {
  }
}

inline int observedMinimum(const std::atomic<int> &value) noexcept
{
  const int observed = value.load(std::memory_order_relaxed);
  return observed == INT_MAX ? 0 : observed;
}

inline int currentProcessThreadCount()
{
#ifdef __linux__
  DIR *dir = opendir("/proc/self/task");
  if (!dir)
    return 0;

  int count = 0;
  while (dirent *entry = readdir(dir)) {
    if (entry->d_name[0] != '.')
      ++count;
  }
  closedir(dir);
  return count;
#elif defined(__APPLE__)
  thread_act_array_t threads = nullptr;
  mach_msg_type_number_t count = 0;
  if (task_threads(mach_task_self(), &threads, &count) != KERN_SUCCESS)
    return 0;

  vm_deallocate(mach_task_self(), reinterpret_cast<vm_address_t>(threads), count * sizeof(thread_t));
  return static_cast<int>(count);
#else
  return 0;
#endif
}

class ThreadSampler
{
public:
  void start()
  {
    baseline_ = currentProcessThreadCount();
    peak_.store(baseline_, std::memory_order_relaxed);
    done_.store(false, std::memory_order_release);
    sampler_ = std::thread([this] {
      while (!done_.load(std::memory_order_acquire)) {
        const int count = currentProcessThreadCount();
        int observed = peak_.load(std::memory_order_relaxed);
        while (count > observed &&
          !peak_.compare_exchange_weak(observed, count, std::memory_order_relaxed)) {
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    });
  }

  int stop()
  {
    done_.store(true, std::memory_order_release);
    if (sampler_.joinable())
      sampler_.join();
    return peak_.load(std::memory_order_relaxed);
  }

  int baseline() const { return baseline_; }

private:
  std::atomic<bool> done_ = false;
  std::atomic<int> peak_ = 0;
  int baseline_ = 0;
  std::thread sampler_;
};

inline int adjustedExtraThreads(int baselineThreads, int peakThreads)
{
  constexpr int samplerThreads = 1;
  return std::max(0, peakThreads - baselineThreads - samplerThreads);
}

}  // namespace pgo::benchmark_helpers
