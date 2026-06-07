#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include "dynamic/core.h"

namespace nb = nanobind;

void init_simulation_bindings(nb::module_ &m)
{
  nb::class_<PyDynamicSimulation>(m, "PyDynamicSimulation")
    .def(nb::init<int, std::vector<int>, std::vector<int>, std::vector<double>,
           std::shared_ptr<PyPotentialEnergy>, double, double,
           nb::ndarray<nb::numpy, const double>, nb::ndarray<nb::numpy, const double>, nb::ndarray<nb::numpy, const double>,
           double, const std::string &, std::vector<int>, double>(),
      nb::arg("num_dofs"),
      nb::arg("mass_rows"), nb::arg("mass_cols"), nb::arg("mass_vals"),
      nb::arg("energy").none(),
      nb::arg("mass_damping"), nb::arg("stiffness_damping"),
      nb::arg("displacement"), nb::arg("velocity"), nb::arg("acceleration"),
      nb::arg("timestep"), nb::arg("integrator"), nb::arg("fixed_dofs"),
      nb::arg("gamma"))
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
