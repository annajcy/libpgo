#include <nanobind/nanobind.h>
#include <nanobind/stl/optional.h>

#include "parallel/arenaThreadingExecutor.h"
#include "parallel/parallelControl.h"
#include "parallel/threadingPolicy.h"

#include <exception>
#include <utility>

namespace nb = nanobind;

namespace
{

class PyArenaThreadingExecutor
{
public:
  PyArenaThreadingExecutor(int maxConcurrency, int reservedSlots,
    std::optional<int> mklLocalThreadBudget,
    std::optional<pgo::parallel::AccelerateThreading> accelerate): executor_(maxConcurrency,
                                                                     {
                                                                       .mklLocalThreadBudget = mklLocalThreadBudget,
                                                                       .accelerate = accelerate,
                                                                     },
                                                                     reservedSlots)
  {
  }

  nb::object execute(const nb::callable &fn)
  {
    PyObject *result = nullptr;
    std::exception_ptr error;
    {
      nb::gil_scoped_release release;
      executor_.execute([&] {
        nb::gil_scoped_acquire acquire;
        try {
          nb::object value = fn();
          result = value.release().ptr();
        }
        catch (...) {
          error = std::current_exception();
        }
      });
    }

    if (error)
      std::rethrow_exception(error);
    return nb::steal<nb::object>(result);
  }

private:
  pgo::parallel::ArenaThreadingExecutor executor_;
};

}  // namespace

void init_parallel_bindings(nb::module_ &m)
{
  using pgo::parallel::AccelerateThreading;
  using pgo::parallel::GlobalTbbControl;

  nb::enum_<AccelerateThreading>(m, "_AccelerateThreading")
    .value("SINGLE", AccelerateThreading::single)
    .value("MULTI", AccelerateThreading::multi);

  nb::class_<GlobalTbbControl>(m, "_GlobalTbbControl")
    .def(nb::init<int>(), nb::arg("max_allowed_parallelism"));

  nb::class_<PyArenaThreadingExecutor>(m, "_ArenaThreadingExecutor")
    .def(nb::init<int, int, std::optional<int>, std::optional<AccelerateThreading>>(),
      nb::arg("max_concurrency"), nb::arg("reserved_slots") = 1,
      nb::arg("mkl_local_thread_budget") = 1,
      nb::arg("accelerate") = AccelerateThreading::single)
    .def("execute", &PyArenaThreadingExecutor::execute, nb::arg("fn"));

  m.def("_parallel_set_threading_policy", [](std::optional<int> mklLocalThreadBudget, std::optional<AccelerateThreading> accelerate) { pgo::parallel::setThreadingPolicy({
                                                                                                                                         .mklLocalThreadBudget = mklLocalThreadBudget,
                                                                                                                                         .accelerate = accelerate,
                                                                                                                                       }); }, nb::arg("mkl_local_thread_budget") = nb::none(), nb::arg("accelerate") = nb::none());
}
