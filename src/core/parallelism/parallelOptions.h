#pragma once

#include <optional>

namespace pgo::parallel {

enum class Backend {
  Auto,
  Serial,
  TBB,
  OpenMP,
};

struct Options {
  int numThreads = 0;
  Backend backend = Backend::Auto;
  int grainSize = 0;
};

bool isBackendAvailable(Backend backend);

void setDefaultNumThreads(std::optional<int> numThreads);
std::optional<int> defaultNumThreads();

class ScopedThreadLimit {
public:
  explicit ScopedThreadLimit(std::optional<int> numThreads);
  ~ScopedThreadLimit();

  ScopedThreadLimit(const ScopedThreadLimit &) = delete;
  ScopedThreadLimit &operator=(const ScopedThreadLimit &) = delete;

private:
  std::optional<int> previousNumThreads_;
};

}  // namespace pgo::parallel
