#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>

#include "core.h"

namespace nb = nanobind;
using namespace pgo;

void init_mass_bindings(nb::module_ &m)
{
    nb::class_<PyVolumeMassField>(m, "PyVolumeMassField");

    m.def("make_constant_volume_density", &make_constant_volume_density,
        nb::arg("density"));
    m.def("make_elementwise_volume_density", &make_elementwise_volume_density,
        nb::arg("densities"));
}
