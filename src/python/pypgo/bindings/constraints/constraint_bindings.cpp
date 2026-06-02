#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>

#include "constraint_core.h"
#include "constraints/constraintSet.h"
#include "constraints/linearConstraintFunctions.h"
#include "sparse_matrix_core.h"

#include <memory>
#include <stdexcept>
#include <vector>

namespace nb = nanobind;
using namespace pgo;

namespace
{
EigenSupport::SpMatD sparseMatrixToEigen(const PySparseMatrix &A)
{
  auto coo = A.toCOO();
  const auto &rowIndices = std::get<0>(coo);
  const auto &colIndices = std::get<1>(coo);
  const auto &values = std::get<2>(coo);

  std::vector<EigenSupport::TripletD> triplets;
  triplets.reserve(values.size());
  for (size_t i = 0; i < values.size(); ++i) {
    triplets.emplace_back(rowIndices[i], colIndices[i], values[i]);
  }

  EigenSupport::SpMatD matrix(A.rows(), A.cols());
  matrix.setFromTriplets(triplets.begin(), triplets.end());
  return matrix;
}

std::shared_ptr<PyConstraintFunctions> createLinearConstraint(const PySparseMatrix &A, nb::ndarray<nb::numpy, const double> offset)
{
  auto offsetVec = python::ndarrayToVectorXd(offset);
  auto constraints = std::make_shared<NonlinearOptimization::LinearConstraintFunctions>(
    sparseMatrixToEigen(A), std::move(offsetVec));
  return std::make_shared<PyConstraintFunctions>(std::move(constraints));
}

std::shared_ptr<PyConstraintFunctions> createConstraintSet(nb::list terms)
{
  std::vector<NonlinearOptimization::ConstraintSet::Term> cppTerms;
  cppTerms.reserve(nb::len(terms));

  int numDofs = -1;
  for (size_t i = 0; i < nb::len(terms); ++i) {
    auto handle = nb::cast<std::shared_ptr<PyConstraintFunctions>>(terms[i]);
    if (numDofs < 0) {
      numDofs = handle->numDofs();
    } else if (handle->numDofs() != numDofs) {
      throw nb::value_error("All ConstraintFunctionSet terms must have the same num_dofs");
    }
    cppTerms.push_back({ handle->handle_ });
  }

  if (cppTerms.empty()) {
    throw nb::value_error("ConstraintFunctionSet requires at least one term");
  }

  auto set = std::make_shared<NonlinearOptimization::ConstraintSet>(numDofs, std::move(cppTerms));
  return std::make_shared<PyConstraintFunctions>(std::move(set));
}
}  // namespace

void init_constraint_bindings(nb::module_ &m)
{
  nb::class_<PyConstraintFunctions>(m, "ConstraintFunctions")
    .def("__repr__", &PyConstraintFunctions::repr)
    .def_prop_ro("num_dofs", &PyConstraintFunctions::numDofs)
    .def_prop_ro("num_constraints", &PyConstraintFunctions::numConstraints)
    .def_prop_ro("is_linear", &PyConstraintFunctions::isLinear)
    .def("value", &PyConstraintFunctions::value, nb::arg("x"))
    .def("jacobian", &PyConstraintFunctions::jacobian, nb::arg("x"))
    .def("hessian", &PyConstraintFunctions::hessian, nb::arg("x"), nb::arg("multipliers"));

  m.def("_create_linear_constraint", &createLinearConstraint,
    nb::arg("A"), nb::arg("offset"));

  m.def("_create_constraint_function_set", &createConstraintSet,
    nb::arg("terms"));
}
