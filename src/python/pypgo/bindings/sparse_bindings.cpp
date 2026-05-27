#include <nanobind/nanobind.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>

#include <vector>

#include "sparse_matrix_core.h"

namespace nb = nanobind;

SparseMatrixCore create_sparse_matrix(
    int rows,
    int cols,
    const std::vector<int>& rowIndices,
    const std::vector<int>& colIndices,
    const std::vector<double>& values)
{
    return SparseMatrixCore(rows, cols, rowIndices, colIndices, values);
}

void init_sparse_bindings(nb::module_& m)
{
    nb::class_<SparseMatrixCore>(m, "SparseMatrixCore")
        .def("rows", &SparseMatrixCore::rows)
        .def("cols", &SparseMatrixCore::cols)
        .def("nnz", &SparseMatrixCore::nnz)
        .def("to_coo", &SparseMatrixCore::toCOO);

    m.def("create_sparse_matrix", &create_sparse_matrix);
}
