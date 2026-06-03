#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/pair.h>

namespace nb = nanobind;

// Forward declarations of bind functions
void init_mesh_geo_bindings(nb::module_ &m);
void init_mesh_bindings(nb::module_ &m);
void init_sparse_bindings(nb::module_ &m);
void init_dense_bindings(nb::module_ &m);
void init_energy_bindings(nb::module_ &m);
void init_constraint_bindings(nb::module_ &m);
void init_solver_bindings(nb::module_ &m);
void init_animation_bindings(nb::module_ &m);
void init_implicit_bindings(nb::module_ &m);
void init_parallel_bindings(nb::module_ &m);
void init_simulation_bindings(nb::module_ &m);

NB_MODULE(_core, m) {
    m.def("build_info", []() {
        nb::dict info;
        info["module"] = "pypgo._core";
        info["binding"] = "nanobind";
        info["mesh_geo"] = true;
        return info;
    });

    init_mesh_geo_bindings(m);
    init_mesh_bindings(m);
    init_sparse_bindings(m);
    init_dense_bindings(m);
    init_energy_bindings(m);
    init_constraint_bindings(m);
    init_solver_bindings(m);
    init_animation_bindings(m);
    init_implicit_bindings(m);
    init_parallel_bindings(m);
    init_simulation_bindings(m);
}
