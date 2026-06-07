#include "core.h"

#include "eigen_numpy.h"

#include <chrono>
#include <thread>

namespace pgo
{

double vector_sum(DenseDoubleArray values)
{
  auto mapped = pgo::python::ndarrayToVectorMapXd(values);
  return mapped.sum();
}

nanobind::ndarray<nanobind::numpy, double> vector_roundtrip(DenseDoubleArray values)
{
  return pgo::python::vectorXdToNdarray(pgo::python::ndarrayToVectorXd(values));
}

double matrix_sum(DenseDoubleArray values)
{
  auto mapped = pgo::python::ndarrayToRowMajorMatrixMapXd(values);
  return mapped.sum();
}

nanobind::ndarray<nanobind::numpy, double> matrix_roundtrip(DenseDoubleArray values)
{
  return pgo::python::matrixXdToNdarray(pgo::python::ndarrayToMatrixXd(values));
}

double fixed_3x3_trace(DenseDoubleArray values)
{
  return pgo::python::ndarrayToFixedMatrix<3, 3>(values).trace();
}

void gil_released_sleep_ms(int milliseconds)
{
  if (milliseconds < 0) {
    throw nanobind::value_error("milliseconds must be non-negative");
  }

  {
    nanobind::gil_scoped_release release;
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
  }
}

}  // namespace pgo
