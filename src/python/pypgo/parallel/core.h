#pragma once

#include <nanobind/nanobind.h>

#include "parallel/arenaThreadingExecutor.h"
#include "parallel/threadingPolicy.h"

#include <exception>
#include <memory>
#include <optional>
#include <utility>

namespace nb = nanobind;

class PyArenaThreadingExecutor
{
public:
  PyArenaThreadingExecutor(int maxConcurrency, int reservedSlots,
    std::optional<int> mklLocalThreadBudget,
    std::optional<pgo::parallel::AccelerateThreading> accelerate):
    executor_(std::make_shared<pgo::parallel::ArenaThreadingExecutor>(
      maxConcurrency,
      pgo::parallel::ThreadingPolicy{
        .mklLocalThreadBudget = mklLocalThreadBudget,
        .accelerate = accelerate,
      },
      reservedSlots))
  {
  }

  nb::object execute(const nb::callable &fn)
  {
    PyObject *result = nullptr;
    std::exception_ptr error;
    {
      nb::gil_scoped_release release;
      executor_->execute([&] {
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

  const std::shared_ptr<pgo::parallel::ArenaThreadingExecutor> &handle() const
  {
    return executor_;
  }

private:
  std::shared_ptr<pgo::parallel::ArenaThreadingExecutor> executor_;
};
