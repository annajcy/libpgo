#pragma once

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

namespace pgo
{

using DenseDoubleArray = nanobind::ndarray<nanobind::numpy, const double>;

double vector_sum(DenseDoubleArray values);
nanobind::ndarray<nanobind::numpy, double> vector_roundtrip(DenseDoubleArray values);
double matrix_sum(DenseDoubleArray values);
nanobind::ndarray<nanobind::numpy, double> matrix_roundtrip(DenseDoubleArray values);
double fixed_3x3_trace(DenseDoubleArray values);
void gil_released_sleep_ms(int milliseconds);

}  // namespace pgo
