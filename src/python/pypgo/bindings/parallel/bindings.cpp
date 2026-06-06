#include <nanobind/nanobind.h>

#include "parallelism/parallelOptions.h"

#include <optional>

namespace nb = nanobind;

namespace
{

nb::object getNumThreads()
{
  const auto numThreads = pgo::parallel::defaultNumThreads();
  if (!numThreads.has_value())
    return nb::none();
  return nb::int_(*numThreads);
}

void resetNumThreads()
{
  pgo::parallel::setDefaultNumThreads(std::nullopt);
}

void setNumThreads(int numThreads)
{
  if (numThreads <= 0)
    throw nb::value_error("num_threads must be a positive integer or None");
  pgo::parallel::setDefaultNumThreads(numThreads);
}

}  // namespace

void init_parallel_bindings(nb::module_ &m)
{
  m.def("_parallel_get_num_threads", &getNumThreads);
  m.def("_parallel_set_num_threads", &setNumThreads, nb::arg("num_threads"));
  m.def("_parallel_reset_num_threads", &resetNumThreads);
}
