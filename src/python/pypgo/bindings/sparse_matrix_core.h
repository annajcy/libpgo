#pragma once

#include "EigenDef.h"

#include <map>
#include <stdexcept>
#include <tuple>
#include <vector>

class SparseMatrixCore {
public:
    SparseMatrixCore(int rows, int cols,
        const std::vector<int>& rowIndices,
        const std::vector<int>& colIndices,
        const std::vector<double>& values):
        rows_(rows), cols_(cols)
    {
        initializeFromCOO(rows, cols, rowIndices, colIndices, values);
    }

    explicit SparseMatrixCore(pgo::EigenSupport::SpMatD matrix):
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
};
