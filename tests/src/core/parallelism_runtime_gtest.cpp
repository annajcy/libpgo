#include <gtest/gtest.h>

#include "parallelism/parallelFor.h"

#include <atomic>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

namespace P = pgo::parallel;

static_assert(!std::is_destructible_v<P::ParallelRuntime>);

namespace
{

enum class RuntimeScenario
{
  Query,
  Explicit,
  ConcurrentSame,
  ConcurrentQuery,
  ConcurrentConflict,
  Automatic,
  Invalid,
};

int runRuntimeScenario(RuntimeScenario scenario)
{
  try {
    switch (scenario) {
    case RuntimeScenario::Query: {
      if (P::defaultConcurrency() < 1)
        return 1;
      const P::RuntimeInfo first = P::runtimeInfo();
      if (first.initialized || first.maxConcurrency.has_value() ||
        first.currentTotalParticipants != 0 || P::runtimeInfo().initialized)
        return 2;
      return 0;
    }
    case RuntimeScenario::Explicit: {
      P::ParallelRuntime *first = &P::initializeRuntime({ .maxTbbConcurrency = 2 });
      P::ParallelRuntime *second = &P::initializeRuntime({ .maxTbbConcurrency = 2 });
      if (first != second || first->info().usingDefaultConcurrency ||
        first->info().maxConcurrency != 2)
        return 1;
      try {
        P::initializeRuntime({ .maxTbbConcurrency = 3 });
        return 2;
      }
      catch (const std::logic_error &error) {
        const std::string message = error.what();
        return message.find("2") != std::string::npos &&
            message.find("3") != std::string::npos ?
          0 :
          3;
      }
    }
    case RuntimeScenario::ConcurrentSame: {
      constexpr int numThreads = 24;
      std::vector<P::ParallelRuntime *> results(numThreads, nullptr);
      std::atomic<bool> failed = false;
      std::vector<std::thread> threads;
      for (int i = 0; i < numThreads; ++i) {
        threads.emplace_back([&, i] {
          try {
            results[static_cast<std::size_t>(i)] =
              &P::initializeRuntime({ .maxTbbConcurrency = 3 });
          }
          catch (...) {
            failed.store(true, std::memory_order_relaxed);
          }
        });
      }
      for (auto &thread : threads)
        thread.join();
      if (failed.load(std::memory_order_relaxed) || results.front() == nullptr)
        return 1;
      for (P::ParallelRuntime *result : results) {
        if (result != results.front())
          return 2;
      }
      return P::runtimeInfo().maxConcurrency == 3 ? 0 : 3;
    }
    case RuntimeScenario::ConcurrentQuery: {
      std::atomic<bool> start = false;
      std::atomic<bool> done = false;
      std::atomic<bool> invalidSnapshot = false;
      std::vector<std::thread> readers;
      for (int i = 0; i < 12; ++i) {
        readers.emplace_back([&] {
          while (!start.load(std::memory_order_acquire))
            std::this_thread::yield();
          while (!done.load(std::memory_order_acquire)) {
            const P::RuntimeInfo info = P::runtimeInfo();
            const bool validUninitialized = !info.initialized && !info.maxConcurrency.has_value() &&
              info.currentTotalParticipants == 0;
            const bool validInitialized = info.initialized && info.maxConcurrency == 3;
            if (!validUninitialized && !validInitialized)
              invalidSnapshot.store(true, std::memory_order_relaxed);
          }
        });
      }
      start.store(true, std::memory_order_release);
      P::initializeRuntime({ .maxTbbConcurrency = 3 });
      done.store(true, std::memory_order_release);
      for (auto &reader : readers)
        reader.join();
      return invalidSnapshot.load(std::memory_order_relaxed) ? 1 : 0;
    }
    case RuntimeScenario::ConcurrentConflict: {
      constexpr int numThreads = 20;
      std::atomic<int> successes = 0;
      std::atomic<int> conflicts = 0;
      std::vector<std::thread> threads;
      for (int i = 0; i < numThreads; ++i) {
        threads.emplace_back([&, requested = 2 + (i % 2)] {
          try {
            P::initializeRuntime({ .maxTbbConcurrency = requested });
            successes.fetch_add(1, std::memory_order_relaxed);
          }
          catch (const std::logic_error &) {
            conflicts.fetch_add(1, std::memory_order_relaxed);
          }
        });
      }
      for (auto &thread : threads)
        thread.join();
      const auto value = P::runtimeInfo().maxConcurrency;
      return successes.load(std::memory_order_relaxed) == numThreads / 2 &&
          conflicts.load(std::memory_order_relaxed) == numThreads / 2 &&
          (value == 2 || value == 3) ?
        0 :
        1;
    }
    case RuntimeScenario::Automatic: {
      const int candidate = P::defaultConcurrency();
      P::ParallelRuntime *automatic = &P::runtime();
      P::ParallelRuntime *explicitSame =
        &P::initializeRuntime({ .maxTbbConcurrency = candidate });
      return automatic == explicitSame && automatic->info().usingDefaultConcurrency ? 0 : 1;
    }
    case RuntimeScenario::Invalid: {
      try {
        P::initializeRuntime({ .maxTbbConcurrency = 0 });
        return 1;
      }
      catch (const std::invalid_argument &) {
      }
      auto &runtime = P::initializeRuntime({ .maxTbbConcurrency = 2 });
      try {
        runtime.createExecutor({ .maxConcurrency = 0 });
        return 2;
      }
      catch (const std::invalid_argument &) {
      }
      runtime.createExecutor({ .maxConcurrency = 7 });
      return 0;
    }
    }
  }
  catch (...) {
    return 100;
  }
  return 101;
}

void expectIsolated(RuntimeScenario scenario)
{
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  EXPECT_EXIT(std::_Exit(runRuntimeScenario(scenario)), testing::ExitedWithCode(0), "");
}

}  // namespace

TEST(ParallelRuntimeTest, QueriesDoNotInitializeRuntime)
{
  expectIsolated(RuntimeScenario::Query);
}

TEST(ParallelRuntimeTest, ExplicitInitializationIsIdempotentAndRejectsConflict)
{
  expectIsolated(RuntimeScenario::Explicit);
}

TEST(ParallelRuntimeTest, ConcurrentSameValueInitializationPublishesOneRuntime)
{
  expectIsolated(RuntimeScenario::ConcurrentSame);
}

TEST(ParallelRuntimeTest, ConcurrentQueriesSeeOnlyCompleteSnapshots)
{
  expectIsolated(RuntimeScenario::ConcurrentQuery);
}

TEST(ParallelRuntimeTest, ConcurrentConflictingInitializationHasOneWinner)
{
  expectIsolated(RuntimeScenario::ConcurrentConflict);
}

TEST(ParallelRuntimeTest, AutomaticAndExplicitResolvedValueAreIdempotent)
{
  expectIsolated(RuntimeScenario::Automatic);
}

TEST(ParallelRuntimeTest, InvalidConcurrencyIsRejected)
{
  expectIsolated(RuntimeScenario::Invalid);
}
