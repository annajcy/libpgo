#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>

#include <vector>

#include "core.h"
#include "eigen_numpy.h"

#include "EigenSupport.h"

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
        .def("to_dense", &PySparseMatrix::toDense)
        .def("matvec", [](const PySparseMatrix& A, nb::ndarray<nb::numpy, const double> x) {
            auto xMap = pgo::python::ndarrayToVectorMapXd(x);
            Eigen::VectorXd b(A.rows());
            pgo::EigenSupport::mv(A.eigenMatrix(), xMap, b);
            return pgo::python::vectorXdToNdarray(std::move(b));
        })
        .def("matmat", [](const PySparseMatrix& A, nb::ndarray<nb::numpy, const double> B) {
            Eigen::MatrixXd Bm = pgo::python::ndarrayToMatrixXd(B);
            Eigen::MatrixXd C(A.rows(), Bm.cols());
            pgo::EigenSupport::mm(A.eigenMatrix(), Bm, C);
            return pgo::python::matrixXdToNdarray(std::move(C));
        });

    m.def("create_sparse_matrix", &create_sparse_matrix);
}
