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

    m.def("make_tet_p1", &make_tet_p1);
    m.def("make_linear_cubic", &make_linear_cubic);
    m.def("make_tricubic_hermite", &make_tricubic_hermite);
    m.def("make_koiter_shell", &make_koiter_shell);

    m.def("compute_formulation_mass_matrix", &compute_formulation_mass_matrix,
        nb::arg("volume_mesh"), nb::arg("formulation"));
    m.def("compute_formulation_body_force", &compute_formulation_body_force,
        nb::arg("volume_mesh"), nb::arg("formulation"), nb::arg("acceleration"));
    m.def("compute_formulation_surface_embedding_matrix", &compute_formulation_surface_embedding_matrix,
        nb::arg("volume_mesh"), nb::arg("formulation"), nb::arg("surface_vertices_flat"));
}
