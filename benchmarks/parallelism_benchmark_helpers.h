#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <memory>
#include <thread>
#include <vector>

#include <tbb/task_arena.h>
#include <tbb/task_scheduler_observer.h>

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

class CurrentArenaThreadObserver final
{
public:
  CurrentArenaThreadObserver():
    arena_(tbb::attach{}), observer_(arena_, current_, peak_)
  {
    observer_.observe(true);
  }

  ~CurrentArenaThreadObserver()
  {
    observer_.observe(false);
  }

  int peak() const noexcept { return peak_.load(std::memory_order_relaxed); }

private:
  class Observer final : public tbb::task_scheduler_observer
  {
  public:
    Observer(tbb::task_arena &arena, std::atomic<int> &current, std::atomic<int> &peak):
      tbb::task_scheduler_observer(arena), current_(current), peak_(peak)
    {
    }

  private:
    void on_scheduler_entry(bool) override
    {
      const int current = current_.fetch_add(1, std::memory_order_relaxed) + 1;
      updateMaximum(peak_, current);
    }

    void on_scheduler_exit(bool) override
    {
      current_.fetch_sub(1, std::memory_order_relaxed);
    }

    std::atomic<int> &current_;
    std::atomic<int> &peak_;
  };

  tbb::task_arena arena_;
  std::atomic<int> current_{ 0 };
  std::atomic<int> peak_{ 0 };
  Observer observer_;
};

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

struct MatrixSet
{
  explicit MatrixSet(int matrixN)
  {
    reset(matrixN);
  }

  void reset(int matrixN)
  {
    n = matrixN;
    const std::size_t size = static_cast<std::size_t>(n) * static_cast<std::size_t>(n);
    a.resize(size);
    b.resize(size);
    c.resize(size);

    for (std::size_t i = 0; i < size; ++i) {
      a[i] = 1.0 + static_cast<double>(i % 17) * 0.001;
      b[i] = 0.5 + static_cast<double>(i % 29) * 0.002;
      c[i] = 0.0;
    }
  }

  double centerValue() const
  {
    return c[static_cast<std::size_t>(n / 2) * static_cast<std::size_t>(n) + static_cast<std::size_t>(n / 2)];
  }

  int n = 0;
  std::vector<double> a;
  std::vector<double> b;
  std::vector<double> c;
};

class MatrixPool
{
public:
  MatrixPool(int numSlots, int matrixN)
  {
    slots_.reserve(static_cast<std::size_t>(numSlots));
    for (int i = 0; i < numSlots; ++i)
      slots_.push_back(std::make_unique<MatrixSet>(matrixN));
  }

  MatrixSet &current()
  {
    int index = tbb::this_task_arena::current_thread_index();
    if (index < 0)
      index = 0;
    return *slots_[static_cast<std::size_t>(index) % slots_.size()];
  }

private:
  std::vector<std::unique_ptr<MatrixSet>> slots_;
};

inline int adjustedExtraThreads(int baselineThreads, int peakThreads)
{
  constexpr int samplerThreads = 1;
  return std::max(0, peakThreads - baselineThreads - samplerThreads);
}

}  // namespace pgo::benchmark_helpers
