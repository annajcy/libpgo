#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>

#include "core.h"

namespace nb = nanobind;

void init_constraint_bindings(nb::module_ &m)
{
  nb::class_<PyConstraintFunctions>(m, "PyConstraintFunctions")
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
