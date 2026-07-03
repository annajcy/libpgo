#include <gtest/gtest.h>

#include "processMemory.h"
#include "scopedProfileSection.h"
#include "threadRuntime.h"

#include <algorithm>
#include <cstddef>
#include <chrono>
#include <cstdint>
#include <string_view>
#include <thread>
#include <vector>

namespace
{
using pgo::Profiling::ProfileStat;
using pgo::Profiling::ProfileCounterStat;
using pgo::Profiling::ProcessMemoryUsage;
using pgo::Profiling::ScopedProfileSection;
using pgo::Profiling::ScopedThreadRuntimePhase;

const ProfileStat *findChild(const ProfileStat &parent, std::string_view name)
{
  const auto it = std::find_if(parent.children.begin(), parent.children.end(),
    [name](const ProfileStat &stat) { return stat.name == name; });
  return it == parent.children.end() ? nullptr : &(*it);
}

const ProfileCounterStat *findCounterStat(const std::vector<ProfileCounterStat> &stats, std::string_view name)
{
  const auto it = std::find_if(stats.begin(), stats.end(),
    [name](const ProfileCounterStat &stat) { return stat.name == name; });
  return it == stats.end() ? nullptr : &(*it);
}

class ScopedProfileSectionGTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    pgo::Profiling::setProfilingEnabled(false);
    pgo::Profiling::resetProfileStatistics();
  }

  void TearDown() override
  {
    pgo::Profiling::setProfilingEnabled(false);
    pgo::Profiling::resetProfileStatistics();
  }
};
}  // namespace

TEST(ProcessMemoryGTest, ReportsResidentMemoryOnSupportedDesktopPlatforms)
{
  std::vector<std::byte> buffer(1024 * 1024);
  for (std::size_t i = 0; i < buffer.size(); i += 4096)
    buffer[i] = std::byte{ 1 };

  const ProcessMemoryUsage usage = pgo::Profiling::processMemoryUsage();

#if defined(__APPLE__) || defined(__linux__) || defined(_WIN32)
  EXPECT_GT(usage.residentBytes, std::uint64_t{ 0 });
  EXPECT_GT(usage.peakResidentBytes, std::uint64_t{ 0 });
  EXPECT_GE(usage.peakResidentBytes, usage.residentBytes);
#else
  EXPECT_EQ(usage.residentBytes, std::uint64_t{ 0 });
  EXPECT_EQ(usage.peakResidentBytes, std::uint64_t{ 0 });
#endif
}

TEST_F(ScopedProfileSectionGTest, DisabledProfilingLeavesNoRecords)
{
  {
    ScopedProfileSection scope("profiling.disabled");
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  pgo::Profiling::recordProfileCounter("profiling.disabled.counter", 3);
  pgo::Profiling::recordThreadRuntimeSample("profiling.disabled.thread");

  const auto root = pgo::Profiling::snapshotProfileStatistics();
  EXPECT_EQ(root.name, "root");
  EXPECT_EQ(root.callCount, 0u);
  EXPECT_TRUE(root.children.empty());
  EXPECT_TRUE(pgo::Profiling::snapshotProfileCounterStatistics().empty());
}

TEST_F(ScopedProfileSectionGTest, RepeatedSectionsAggregateByName)
{
  pgo::Profiling::setProfilingEnabled(true);

  {
    ScopedProfileSection scope("profiling.repeat");
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  {
    ScopedProfileSection scope("profiling.repeat");
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  const auto root = pgo::Profiling::snapshotProfileStatistics();
  ASSERT_EQ(root.children.size(), 1u);
  EXPECT_EQ(root.children.front().name, "profiling.repeat");
  EXPECT_EQ(root.children.front().localName, "profiling.repeat");
  EXPECT_EQ(root.children.front().path, "profiling.repeat");
  EXPECT_EQ(root.children.front().callCount, 2u);
  EXPECT_GT(root.children.front().totalSeconds, 0.0);
  EXPECT_GT(root.children.front().maxSeconds, 0.0);
}

TEST_F(ScopedProfileSectionGTest, DistinctSectionsRemainIndependentAndSorted)
{
  pgo::Profiling::setProfilingEnabled(true);

  {
    ScopedProfileSection scope("profiling.zeta");
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  {
    ScopedProfileSection scope("profiling.alpha");
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  const auto root = pgo::Profiling::snapshotProfileStatistics();
  ASSERT_EQ(root.children.size(), 2u);
  EXPECT_EQ(root.children[0].name, "profiling.alpha");
  EXPECT_EQ(root.children[1].name, "profiling.zeta");
  EXPECT_EQ(root.children[0].callCount, 1u);
  EXPECT_EQ(root.children[1].callCount, 1u);
}

TEST_F(ScopedProfileSectionGTest, NestedSectionsAreTrackedAsTree)
{
  pgo::Profiling::setProfilingEnabled(true);

  {
    ScopedProfileSection outer("profiling.outer");
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    {
      ScopedProfileSection innerSameName("profiling.outer");
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    {
      ScopedProfileSection innerDifferentName("profiling.inner");
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  const auto root = pgo::Profiling::snapshotProfileStatistics();
  const ProfileStat *outer = findChild(root, "profiling.outer");
  ASSERT_NE(outer, nullptr);
  const ProfileStat *innerSameName = findChild(*outer, "profiling.outer.profiling.outer");
  const ProfileStat *inner = findChild(*outer, "profiling.outer.profiling.inner");
  ASSERT_NE(outer, nullptr);
  ASSERT_NE(innerSameName, nullptr);
  ASSERT_NE(inner, nullptr);
  EXPECT_EQ(outer->callCount, 1u);
  EXPECT_EQ(innerSameName->localName, "profiling.outer");
  EXPECT_EQ(innerSameName->callCount, 1u);
  EXPECT_EQ(inner->callCount, 1u);
  EXPECT_GE(outer->totalSeconds, outer->childrenSeconds);
  EXPECT_GE(outer->totalSeconds, outer->selfSeconds);
  EXPECT_GT(outer->childrenSeconds, 0.0);
  EXPECT_GT(outer->selfSeconds, 0.0);
  EXPECT_NEAR(outer->totalSeconds, outer->selfSeconds + outer->childrenSeconds, 1e-9);
}

TEST_F(ScopedProfileSectionGTest, ResetClearsCollectedStatistics)
{
  pgo::Profiling::setProfilingEnabled(true);

  {
    ScopedProfileSection scope("profiling.reset");
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  pgo::Profiling::recordProfileCounter("profiling.reset.counter", 7);

  ASSERT_FALSE(pgo::Profiling::snapshotProfileStatistics().children.empty());
  ASSERT_FALSE(pgo::Profiling::snapshotProfileCounterStatistics().empty());
  pgo::Profiling::resetProfileStatistics();
  EXPECT_TRUE(pgo::Profiling::snapshotProfileStatistics().children.empty());
  EXPECT_TRUE(pgo::Profiling::snapshotProfileCounterStatistics().empty());
}

TEST_F(ScopedProfileSectionGTest, CountersAggregateByNameAndSort)
{
  pgo::Profiling::setProfilingEnabled(true);

  pgo::Profiling::recordProfileCounter("profiling.counter.zeta", 4);
  pgo::Profiling::recordProfileCounter("profiling.counter.alpha", 2);
  pgo::Profiling::recordProfileCounter("profiling.counter.zeta", 9);

  const auto stats = pgo::Profiling::snapshotProfileCounterStatistics();
  ASSERT_EQ(stats.size(), 2u);
  EXPECT_EQ(stats[0].name, "profiling.counter.alpha");
  EXPECT_EQ(stats[1].name, "profiling.counter.zeta");

  const ProfileCounterStat *alpha = findCounterStat(stats, "profiling.counter.alpha");
  const ProfileCounterStat *zeta = findCounterStat(stats, "profiling.counter.zeta");
  ASSERT_NE(alpha, nullptr);
  ASSERT_NE(zeta, nullptr);
  EXPECT_EQ(alpha->sampleCount, 1u);
  EXPECT_EQ(alpha->total, 2u);
  EXPECT_EQ(alpha->max, 2u);
  EXPECT_DOUBLE_EQ(alpha->avg, 2.0);
  EXPECT_EQ(zeta->sampleCount, 2u);
  EXPECT_EQ(zeta->total, 13u);
  EXPECT_EQ(zeta->max, 9u);
  EXPECT_DOUBLE_EQ(zeta->avg, 6.5);
}

TEST_F(ScopedProfileSectionGTest, MemoryCheckpointRecordsProfileCounters)
{
  pgo::Profiling::setProfilingEnabled(true);

  pgo::Profiling::recordProcessMemoryProfileCounters("profiling.memory");

  const auto stats = pgo::Profiling::snapshotProfileCounterStatistics();
  const ProfileCounterStat *resident = findCounterStat(stats, "profiling.memory.resident_bytes");
  const ProfileCounterStat *peakResident = findCounterStat(stats, "profiling.memory.peak_resident_bytes");

  ASSERT_NE(resident, nullptr);
  ASSERT_NE(peakResident, nullptr);
  EXPECT_EQ(resident->sampleCount, 1u);
  EXPECT_EQ(peakResident->sampleCount, 1u);
  EXPECT_DOUBLE_EQ(resident->avg, static_cast<double>(resident->max));
  EXPECT_DOUBLE_EQ(peakResident->avg, static_cast<double>(peakResident->max));
#if defined(__APPLE__) || defined(__linux__) || defined(_WIN32)
  EXPECT_GT(resident->max, std::uint64_t{ 0 });
  EXPECT_GT(peakResident->max, std::uint64_t{ 0 });
#else
  EXPECT_EQ(resident->max, std::uint64_t{ 0 });
  EXPECT_EQ(peakResident->max, std::uint64_t{ 0 });
#endif
}

TEST(ThreadRuntimeGTest, ReportsProcessThreadCountOnLinux)
{
  const auto threadCount = pgo::Profiling::currentProcessThreadCount();

#if defined(__linux__)
  ASSERT_TRUE(threadCount.has_value());
  EXPECT_GT(*threadCount, std::uint64_t{ 0 });
#else
  EXPECT_FALSE(threadCount.has_value());
#endif
}

TEST_F(ScopedProfileSectionGTest, ThreadRuntimeSamplesAggregateAsCounters)
{
  if (!pgo::Profiling::currentProcessThreadCount().has_value())
    GTEST_SKIP() << "Process thread count is not available on this platform.";

  pgo::Profiling::setProfilingEnabled(true);

  pgo::Profiling::recordThreadRuntimeSample("profiling.thread.sample");
  {
    ScopedThreadRuntimePhase phase("profiling.thread.phase");
  }

  const auto stats = pgo::Profiling::snapshotProfileCounterStatistics();
  const ProfileCounterStat *sample = findCounterStat(stats, "thread_runtime.profiling.thread.sample.nlwp");
  const ProfileCounterStat *phase = findCounterStat(stats, "thread_runtime.profiling.thread.phase.nlwp");
  ASSERT_NE(sample, nullptr);
  ASSERT_NE(phase, nullptr);
  EXPECT_EQ(sample->sampleCount, 1u);
  EXPECT_GT(sample->max, std::uint64_t{ 0 });
  EXPECT_EQ(phase->sampleCount, 2u);
  EXPECT_GT(phase->max, std::uint64_t{ 0 });
}
