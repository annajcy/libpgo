#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/shared_ptr.h>

#include "formulation_core.h"
#include "../mesh/volume_core.h"
#include "../sparse/core.h"

namespace nb = nanobind;
using namespace pgo;

namespace
{

// --- formulation-aware dynamics operators (accept persistent objects) ---------

PySparseMatrix compute_formulation_mass_matrix(
    const PyVolumeMesh &volumeMesh,
    const PyVolumetricFormulation &formulation)
{
    pgo::EigenSupport::SpMatD M;
    {
        nb::gil_scoped_release release;
        M = formulation.volumetric().buildMassMatrix(*volumeMesh.getVM());
    }
    return PySparseMatrix(std::move(M));
}

std::vector<double> compute_formulation_body_force(
    const PyVolumeMesh &volumeMesh,
    const PyVolumetricFormulation &formulation,
    const std::vector<double> &acceleration)
{
    if (acceleration.size() != 3) {
        throw std::invalid_argument("acceleration must contain exactly 3 values");
    }
    pgo::EigenSupport::V3d a(acceleration[0], acceleration[1], acceleration[2]);
    pgo::EigenSupport::VXd f;
    {
        nb::gil_scoped_release release;
        f = formulation.volumetric().buildBodyForce(*volumeMesh.getVM(), a);
    }
    return std::vector<double>(f.data(), f.data() + f.size());
}

PySparseMatrix compute_formulation_surface_embedding_matrix(
    const PyVolumeMesh &volumeMesh,
    const PyVolumetricFormulation &formulation,
    const std::vector<double> &surfaceVerticesFlat)
{
    if (surfaceVerticesFlat.size() % 3 != 0) {
        throw std::invalid_argument("surface vertices must be a flat 3*m vector");
    }
    const int numVertices = static_cast<int>(surfaceVerticesFlat.size() / 3);
    pgo::EigenSupport::MXd surfaceVertices(numVertices, 3);
    for (int i = 0; i < numVertices; i++) {
        surfaceVertices(i, 0) = surfaceVerticesFlat[static_cast<size_t>(i) * 3 + 0];
        surfaceVertices(i, 1) = surfaceVerticesFlat[static_cast<size_t>(i) * 3 + 1];
        surfaceVertices(i, 2) = surfaceVerticesFlat[static_cast<size_t>(i) * 3 + 2];
    }

    pgo::EigenSupport::SpMatD W;
    {
        nb::gil_scoped_release release;
        W = formulation.volumetric().buildSurfaceEmbeddingMatrix(
            *volumeMesh.getVM(), surfaceVertices);
    }
    return PySparseMatrix(std::move(W));
}

}  // namespace

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
