#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>

#include "dynamic/core.h"

namespace nb = nanobind;

void init_simulation_bindings(nb::module_ &m)
{
  nb::class_<PyDynamicStepper>(m, "PyDynamicStepper");

  nb::class_<PyBackwardEulerDynamicStepper, PyDynamicStepper>(m, "PyBackwardEulerDynamicStepper")
    .def(nb::init<>());

  nb::class_<PyTRBDF2DynamicStepper, PyDynamicStepper>(m, "PyTRBDF2DynamicStepper")
    .def(nb::init<double>(), nb::arg("gamma") = 0.5)
    .def_prop_ro("gamma", &PyTRBDF2DynamicStepper::gamma);

  nb::class_<PyDynamicSimulation>(m, "PyDynamicSimulation")
    .def(nb::init<int, std::vector<int>, std::vector<int>, std::vector<double>,
           std::shared_ptr<PyPotentialEnergy>, double, double,
           nb::ndarray<nb::numpy, const double>, nb::ndarray<nb::numpy, const double>, nb::ndarray<nb::numpy, const double>,
           double, std::shared_ptr<PyDynamicStepper>, std::vector<int>>(),
      nb::arg("num_dofs"),
      nb::arg("mass_rows"), nb::arg("mass_cols"), nb::arg("mass_vals"),
      nb::arg("energy").none(),
      nb::arg("mass_damping"), nb::arg("stiffness_damping"),
      nb::arg("displacement"), nb::arg("velocity"), nb::arg("acceleration"),
      nb::arg("timestep"), nb::arg("integrator"), nb::arg("fixed_dofs"))
    .def("step", &PyDynamicSimulation::step,
      nb::arg("external_force"), nb::arg("fixed_values"), nb::arg("has_fixed_values"),
      nb::arg("optimizer"))
    .def_prop_ro("displacement", &PyDynamicSimulation::displacement, nb::rv_policy::move)
    .def_prop_ro("velocity", &PyDynamicSimulation::velocity, nb::rv_policy::move)
    .def_prop_ro("acceleration", &PyDynamicSimulation::acceleration, nb::rv_policy::move)
    .def_prop_ro("timestep_id", &PyDynamicSimulation::timestepId)
    .def_prop_ro("time", &PyDynamicSimulation::time)
    .def_prop_ro("num_dofs", &PyDynamicSimulation::numDofs);
}
