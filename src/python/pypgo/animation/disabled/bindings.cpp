#include <nanobind/nanobind.h>

#include "core.h"

namespace nb = nanobind;

void init_animation_bindings(nb::module_ &m)
{
    m.def("has_animation_io", &pgo::has_animation_io);
    m.def("has_stress_vdb_export", &pgo::has_stress_vdb_export);
    m.def("dump_abc", &pgo::dump_abc_unavailable,
        nb::arg("filename"), nb::arg("name"),
        nb::arg("rest_positions"), nb::arg("displacements"), nb::arg("triangles"));
}
