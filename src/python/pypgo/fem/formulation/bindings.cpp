#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/shared_ptr.h>

#include "core.h"

namespace nb = nanobind;
using namespace pgo;

void init_formulation_bindings(nb::module_ &m)
{
    nb::class_<PyFormulation>(m, "PyFormulation")
        .def_prop_ro("name", &PyFormulation::name)
        .def("nodes_per_element", &PyFormulation::nodesPerElement)
        .def("local_dofs", &PyFormulation::localDofs);

    nb::class_<PyVolumetricFormulation, PyFormulation>(m, "PyVolumetricFormulation");

    nb::class_<PyShellFormulation, PyFormulation>(m, "PyShellFormulation");

    m.def("make_tet_linear", &make_tet_linear);
    m.def("make_cubic_linear", &make_cubic_linear);
    m.def("make_cubic_tricubic_hermite", &make_cubic_tricubic_hermite);
    m.def("make_koiter_shell", &make_koiter_shell);

    m.def("compute_formulation_mass_matrix", &compute_formulation_mass_matrix,
        nb::arg("sim_mesh"), nb::arg("formulation"), nb::arg("mass_field"));
    m.def("compute_formulation_body_force", &compute_formulation_body_force,
        nb::arg("sim_mesh"), nb::arg("formulation"), nb::arg("acceleration"), nb::arg("mass_field"));
    m.def("compute_formulation_surface_embedding_matrix", &compute_formulation_surface_embedding_matrix,
        nb::arg("volume_mesh"), nb::arg("formulation"), nb::arg("surface_vertices_flat"));

    m.def("compute_shell_formulation_mass_matrix", &compute_shell_formulation_mass_matrix,
        nb::arg("sim_mesh"), nb::arg("formulation"), nb::arg("mass_field"));
    m.def("compute_shell_formulation_body_force", &compute_shell_formulation_body_force,
        nb::arg("sim_mesh"), nb::arg("formulation"), nb::arg("acceleration"), nb::arg("mass_field"));
}
