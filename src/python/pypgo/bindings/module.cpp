#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/pair.h>

namespace nb = nanobind;

// Forward declarations of bind functions
void init_mesh_geo_bindings(nb::module_ &m);
void init_mesh_bindings(nb::module_ &m);

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
}
