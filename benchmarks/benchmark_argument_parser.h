#pragma once

#include <climits>
#include <cmath>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <string_view>

namespace pgo::benchmark_helpers
{

inline int parseNonnegativeInteger(std::string_view value, std::string_view option)
{
  char *end = nullptr;
  const long parsed = std::strtol(value.data(), &end, 10);
  if (end == value.data() || *end != '\0' || parsed < 0 || parsed > INT_MAX)
    throw std::invalid_argument(std::string(option) + " must be a nonnegative integer.");
  return static_cast<int>(parsed);
}

inline int parsePositiveInteger(std::string_view value, std::string_view option)
{
  const int parsed = parseNonnegativeInteger(value, option);
  if (parsed == 0)
    throw std::invalid_argument(std::string(option) + " must be positive.");
  return parsed;
}

inline double parseNonnegativeDouble(std::string_view value, std::string_view option)
{
  char *end = nullptr;
  const double parsed = std::strtod(value.data(), &end);
  if (end == value.data() || *end != '\0' || !std::isfinite(parsed) || parsed < 0.0)
    throw std::invalid_argument(std::string(option) + " must be a finite nonnegative number.");
  return parsed;
}

inline std::string_view requireValue(int argc, char **argv, std::string_view prefix)
{
  for (int index = 1; index < argc; ++index) {
    const std::string_view argument(argv[index]);
    if (argument.starts_with(prefix))
      return argument.substr(prefix.size());
  }
  throw std::invalid_argument("Missing required option " + std::string(prefix));
}

}  // namespace pgo::benchmark_helpers
