#pragma once

#include "EigenDef.h"
#include "eigen_numpy.h"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

#include <map>
#include <optional>
#include <stdexcept>
#include <tuple>
#include <vector>

class PySparseMatrix {
public:
    PySparseMatrix(int rows, int cols,
        const std::vector<int>& rowIndices,
        const std::vector<int>& colIndices,
        const std::vector<double>& values):
        rows_(rows), cols_(cols)
    {
        initializeFromCOO(rows, cols, rowIndices, colIndices, values);
    }

    explicit PySparseMatrix(pgo::EigenSupport::SpMatD matrix):
        rows_(static_cast<int>(matrix.rows())), cols_(static_cast<int>(matrix.cols()))
    {
        matrix.makeCompressed();
        for (int outer = 0; outer < matrix.outerSize(); ++outer) {
            for (pgo::EigenSupport::SpMatD::InnerIterator it(matrix, outer); it; ++it) {
                rowsIndex_.push_back(static_cast<int>(it.row()));
                colsIndex_.push_back(static_cast<int>(it.col()));
                values_.push_back(it.value());
            }
        }
    }

    int rows() const { return rows_; }
    int cols() const { return cols_; }
    int nnz() const { return static_cast<int>(values_.size()); }

    std::tuple<std::vector<int>, std::vector<int>, std::vector<double>> toCOO() const
    {
        return { rowsIndex_, colsIndex_, values_ };
    }

    std::vector<double> toDense() const
    {
        std::vector<double> dense(static_cast<size_t>(rows_) * static_cast<size_t>(cols_), 0.0);
        for (size_t k = 0; k < values_.size(); ++k) {
            dense[static_cast<size_t>(rowsIndex_[k]) * static_cast<size_t>(cols_) + static_cast<size_t>(colsIndex_[k])]
              += values_[k];
        }
        return dense;
    }

    // Matrix-vector product: y = A * x
    nanobind::ndarray<nanobind::numpy, double> matvec(
      nanobind::ndarray<nanobind::numpy, const double> x) const
    {
        auto xMap = pgo::python::ndarrayToVectorMapXd(x);
        pgo::EigenSupport::VXd y = eigenMatrix() * xMap;
        return pgo::python::vectorXdToNdarray(std::move(y));
    }

    // Matrix-matrix product: C = A * B
    nanobind::ndarray<nanobind::numpy, double> matmat(
      nanobind::ndarray<nanobind::numpy, const double> B) const
    {
        pgo::python::requireFloat64(B);
        if (B.ndim() != 2) {
            throw std::runtime_error("matmat: B must be a 2-D array");
        }
        auto bMap = pgo::python::ndarrayToMatrixXd(B);
        Eigen::MatrixXd C = eigenMatrix() * bMap;
        return pgo::python::matrixXdToNdarray(std::move(C));
    }

    // Named factory
    static PySparseMatrix create(
      int rows,
      int cols,
      const std::vector<int>& rowIndices,
      const std::vector<int>& colIndices,
      const std::vector<double>& values)
    {
        return PySparseMatrix(rows, cols, rowIndices, colIndices, values);
    }

    // Lazily build and cache the Eigen sparse matrix from the stored COO data.
    // The matrix is immutable after construction, so caching is safe.
    const pgo::EigenSupport::SpMatD& eigenMatrix() const
    {
        if (!eigenCache_) {
            std::vector<Eigen::Triplet<double>> triplets;
            triplets.reserve(values_.size());
            for (size_t k = 0; k < values_.size(); ++k) {
                triplets.emplace_back(rowsIndex_[k], colsIndex_[k], values_[k]);
            }
            pgo::EigenSupport::SpMatD m(rows_, cols_);
            m.setFromTriplets(triplets.begin(), triplets.end());
            m.makeCompressed();
            eigenCache_ = std::move(m);
        }
        return *eigenCache_;
    }

private:
    void initializeFromCOO(
        int rows,
        int cols,
        const std::vector<int>& rowIndices,
        const std::vector<int>& colIndices,
        const std::vector<double>& values)
    {
        if (rows < 0 || cols < 0) {
            throw std::runtime_error("SparseMatrix dimensions must be non-negative");
        }
        if (rowIndices.size() != colIndices.size() || rowIndices.size() != values.size()) {
            throw std::runtime_error("SparseMatrix COO arrays must have matching lengths");
        }

        std::map<std::pair<int, int>, double> merged;
        for (size_t i = 0; i < values.size(); ++i) {
            int r = rowIndices[i];
            int c = colIndices[i];
            if (r < 0 || r >= rows || c < 0 || c >= cols) {
                throw std::runtime_error("SparseMatrix COO index out of bounds");
            }
            merged[{ r, c }] += values[i];
        }

        rowsIndex_.reserve(merged.size());
        colsIndex_.reserve(merged.size());
        values_.reserve(merged.size());
        for (const auto& [index, value] : merged) {
            if (value == 0.0) {
                continue;
            }
            rowsIndex_.push_back(index.first);
            colsIndex_.push_back(index.second);
            values_.push_back(value);
        }
    }

private:
    int rows_ = 0;
    int cols_ = 0;
    std::vector<int> rowsIndex_;
    std::vector<int> colsIndex_;
    std::vector<double> values_;
    mutable std::optional<pgo::EigenSupport::SpMatD> eigenCache_;
};
