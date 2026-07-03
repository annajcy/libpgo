#pragma once

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

#include <Eigen/Dense>
#include <array>
#include <algorithm>
#include <memory>
#include <vector>

namespace pgo::python
{
namespace nb = nanobind;

using ConstVectorMapXd = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>;
using ConstMatrixMapXd = Eigen::Map<
  const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>,
  Eigen::Unaligned>;

inline void requireFloat64(const nb::ndarray<nb::numpy, const double> &array)
{
  if (array.dtype() != nb::dtype<double>()) {
    throw nb::value_error("expected dtype float64");
  }
}

inline ConstVectorMapXd ndarrayToVectorMapXd(nb::ndarray<nb::numpy, const double> array)
{
  requireFloat64(array);
  if (array.ndim() != 1) {
    throw nb::value_error("expected shape (n,)");
  }
  if (array.stride(0) != 1) {
    throw nb::value_error("expected C-contiguous vector input");
  }
  return ConstVectorMapXd(array.data(), static_cast<Eigen::Index>(array.shape(0)));
}

inline ConstMatrixMapXd ndarrayToRowMajorMatrixMapXd(nb::ndarray<nb::numpy, const double> array)
{
  requireFloat64(array);
  if (array.ndim() != 2) {
    throw nb::value_error("expected shape (m, n)");
  }
  if (array.stride(1) != 1 || array.stride(0) != static_cast<int64_t>(array.shape(1))) {
    throw nb::value_error("expected C-contiguous row-major matrix input");
  }
  return ConstMatrixMapXd(array.data(),
    static_cast<Eigen::Index>(array.shape(0)),
    static_cast<Eigen::Index>(array.shape(1)));
}

inline Eigen::VectorXd ndarrayToVectorXd(nb::ndarray<nb::numpy, const double> array)
{
  requireFloat64(array);
  if (array.ndim() != 1) {
    throw nb::value_error("expected shape (n,)");
  }

  Eigen::VectorXd out(static_cast<Eigen::Index>(array.shape(0)));
  for (size_t i = 0; i < array.shape(0); ++i) {
    out(static_cast<Eigen::Index>(i)) = array.data()[static_cast<int64_t>(i) * array.stride(0)];
  }
  return out;
}

inline Eigen::MatrixXd ndarrayToMatrixXd(nb::ndarray<nb::numpy, const double> array)
{
  requireFloat64(array);
  if (array.ndim() != 2) {
    throw nb::value_error("expected shape (m, n)");
  }

  Eigen::MatrixXd out(
    static_cast<Eigen::Index>(array.shape(0)),
    static_cast<Eigen::Index>(array.shape(1)));
  for (size_t r = 0; r < array.shape(0); ++r) {
    for (size_t c = 0; c < array.shape(1); ++c) {
      out(static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(c)) =
        array.data()[static_cast<int64_t>(r) * array.stride(0) +
                     static_cast<int64_t>(c) * array.stride(1)];
    }
  }
  return out;
}

template<int Rows, int Cols>
Eigen::Matrix<double, Rows, Cols> ndarrayToFixedMatrix(
  nb::ndarray<nb::numpy, const double> array)
{
  requireFloat64(array);
  if (array.ndim() != 2 ||
      array.shape(0) != static_cast<size_t>(Rows) ||
      array.shape(1) != static_cast<size_t>(Cols)) {
    throw nb::value_error("expected shape (Rows, Cols)");
  }

  auto dynamic = ndarrayToMatrixXd(array);
  return dynamic.template block<Rows, Cols>(0, 0);
}

inline nb::ndarray<nb::numpy, double> vectorXdToNdarray(Eigen::VectorXd values)
{
  auto storage = new std::vector<double>(
    values.data(), values.data() + values.size());
  nb::capsule owner(storage, [](void *p) noexcept {
    delete static_cast<std::vector<double> *>(p);
  });
  return nb::ndarray<nb::numpy, double>(
    storage->data(),
    { storage->size() },
    owner);
}

inline nb::ndarray<nb::numpy, double> matrixXdToNdarray(Eigen::MatrixXd values)
{
  auto storage = new std::vector<double>(
    static_cast<size_t>(values.rows()) * static_cast<size_t>(values.cols()));
  for (Eigen::Index r = 0; r < values.rows(); ++r) {
    for (Eigen::Index c = 0; c < values.cols(); ++c) {
      (*storage)[static_cast<size_t>(r * values.cols() + c)] = values(r, c);
    }
  }

  auto shape = std::make_shared<std::array<size_t, 2>>(
    std::array<size_t, 2>{ static_cast<size_t>(values.rows()), static_cast<size_t>(values.cols()) });
  struct MatrixOwner {
    std::vector<double> *storage;
    std::shared_ptr<std::array<size_t, 2>> shape;
  };
  auto ownerData = new MatrixOwner{ storage, shape };
  nb::capsule owner(ownerData, [](void *p) noexcept {
    auto *data = static_cast<MatrixOwner *>(p);
    delete data->storage;
    delete data;
  });
  return nb::ndarray<nb::numpy, double>(
    storage->data(),
    shape->size(),
    shape->data(),
    owner);
}

}  // namespace pgo::python
