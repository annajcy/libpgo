#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>

#include "core.h"

namespace nb = nanobind;
using namespace pgo;

void init_mass_bindings(nb::module_ &m)
{
    nb::class_<PyVolumeDensity>(m, "PyVolumeDensity");

    m.def("make_constant_volume_density", &make_constant_volume_density,
        nb::arg("density"));
    m.def("make_elementwise_volume_density", &make_elementwise_volume_density,
        nb::arg("densities"));

    nb::class_<PyShellArealDensity>(m, "PyShellArealDensity");
    m.def("make_constant_shell_areal_density", &make_constant_shell_areal_density,
        nb::arg("areal_density"));
    m.def("make_shell_areal_density_elementwise", &make_shell_areal_density_elementwise,
        nb::arg("areal_densities"));
    m.def("make_shell_areal_density_from_density_thickness",
        nb::overload_cast<double, double>(&make_shell_areal_density_from_density_thickness),
        nb::arg("density"), nb::arg("thickness"));
    m.def("make_shell_areal_density_from_density_thickness",
        nb::overload_cast<double, const std::vector<double> &>(
            &make_shell_areal_density_from_density_thickness),
        nb::arg("density"), nb::arg("thickness"));
}
