#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <stdexcept>

namespace nb = nanobind;

void init_animation_bindings(nb::module_ &m)
{
    m.def("has_animation_io", []() { return false; });
    m.def("dump_abc",
        [](const nb::object &, const nb::object &, const nb::object &,
           const nb::object &, const nb::object &) {
            throw std::runtime_error("Animation IO (Alembic) is not available in this build.");
        },
        nb::arg("filename"), nb::arg("name"),
        nb::arg("rest_positions"), nb::arg("displacements"), nb::arg("triangles"));
}
