#include <nanobind/nanobind.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>

#include <vector>

#include "core.h"

namespace nb = nanobind;

PySparseMatrix create_sparse_matrix(
    int rows,
    int cols,
    const std::vector<int>& rowIndices,
    const std::vector<int>& colIndices,
    const std::vector<double>& values)
{
    return PySparseMatrix(rows, cols, rowIndices, colIndices, values);
}

void init_sparse_bindings(nb::module_& m)
{
    nb::class_<PySparseMatrix>(m, "PySparseMatrix")
        .def("rows", &PySparseMatrix::rows)
        .def("cols", &PySparseMatrix::cols)
        .def("nnz", &PySparseMatrix::nnz)
        .def("to_coo", &PySparseMatrix::toCOO)
        .def("to_dense", &PySparseMatrix::toDense);

    m.def("create_sparse_matrix", &create_sparse_matrix);
}
