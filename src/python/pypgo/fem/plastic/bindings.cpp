#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/shared_ptr.h>

#include "core.h"

namespace nb = nanobind;
using namespace pgo;

void init_plastic_model_bindings(nb::module_ &m)
{
    nb::class_<PyPlasticModel>(m, "PyPlasticModel")
        .def_prop_ro("name", &PyPlasticModel::name)
        .def_prop_ro("dofs", &PyPlasticModel::dofs);

    m.def("make_volumetric_plasticity", &make_volumetric_plasticity,
        nb::arg("dofs") = 6);
    m.def("make_shell_plasticity", &make_shell_plasticity,
        nb::arg("dofs") = 1);
}
