#include <nanobind/nanobind.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>

#include "core.h"

namespace nb = nanobind;

void init_sparse_bindings(nb::module_& m)
{
    nb::class_<PySparseMatrix>(m, "PySparseMatrix")
        .def("rows", &PySparseMatrix::rows)
        .def("cols", &PySparseMatrix::cols)
        .def("nnz", &PySparseMatrix::nnz)
        .def("to_coo", &PySparseMatrix::toCOO)
        .def("to_dense", &PySparseMatrix::toDense)
        .def("matvec", &PySparseMatrix::matvec)
        .def("matmat", &PySparseMatrix::matmat);

    m.def("create_sparse_matrix", &PySparseMatrix::create);
}
