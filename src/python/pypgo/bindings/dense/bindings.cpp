#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

#include <chrono>
#include <thread>

#include "eigen_numpy.h"

namespace nb = nanobind;

namespace
{
using DoubleArray = nb::ndarray<nb::numpy, const double>;

double vector_sum(DoubleArray values)
{
  auto mapped = pgo::python::ndarrayToVectorMapXd(values);
  return mapped.sum();
}

nb::ndarray<nb::numpy, double> vector_roundtrip(DoubleArray values)
{
  return pgo::python::vectorXdToNdarray(pgo::python::ndarrayToVectorXd(values));
}

double matrix_sum(DoubleArray values)
{
  auto mapped = pgo::python::ndarrayToRowMajorMatrixMapXd(values);
  return mapped.sum();
}

nb::ndarray<nb::numpy, double> matrix_roundtrip(DoubleArray values)
{
  return pgo::python::matrixXdToNdarray(pgo::python::ndarrayToMatrixXd(values));
}

double fixed_3x3_trace(DoubleArray values)
{
  return pgo::python::ndarrayToFixedMatrix<3, 3>(values).trace();
}

void gil_released_sleep_ms(int milliseconds)
{
  if (milliseconds < 0) {
    throw nb::value_error("milliseconds must be non-negative");
  }

  {
    nb::gil_scoped_release release;
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
  }
}
}  // namespace

void init_dense_bindings(nb::module_ &m)
{
  m.def("_test_vector_sum", &vector_sum);
  m.def("_test_vector_roundtrip", &vector_roundtrip);
  m.def("_test_matrix_sum", &matrix_sum);
  m.def("_test_matrix_roundtrip", &matrix_roundtrip);
  m.def("_test_fixed_3x3_trace", &fixed_3x3_trace);
  m.def("_test_gil_released_sleep_ms", &gil_released_sleep_ms);
}
