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

    nb::class_<PyShellMassField>(m, "PyShellMassField");
    m.def("make_constant_shell_areal_density", &make_constant_shell_areal_density,
        nb::arg("areal_density"));
    m.def("make_shell_density_thickness_constant", &make_shell_density_thickness_constant,
        nb::arg("density"), nb::arg("thickness"));
    m.def("make_shell_density_thickness_elementwise", &make_shell_density_thickness_elementwise,
        nb::arg("density"), nb::arg("thickness"));

    m.def("make_shell_density_elastic_thickness", &make_shell_density_elastic_thickness,
        nb::arg("density"), nb::arg("parameter_field"), nb::arg("channel"));
}
