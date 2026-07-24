#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/shared_ptr.h>

#include "core.h"

namespace nb = nanobind;
using namespace pgo;

void init_plastic_model_bindings(nb::module_ &m)
{
    nb::class_<PyPlasticModelConfig>(m, "PyPlasticModelConfig")
        .def_prop_ro("name", &PyPlasticModelConfig::name)
        .def_prop_ro("dofs", &PyPlasticModelConfig::dofs);
    nb::class_<PyVolumetricPlasticity0Config, PyPlasticModelConfig>(m, "PyVolumetricPlasticity0Config").def(nb::init<>());
    nb::class_<PyVolumetricPlasticity3Config, PyPlasticModelConfig>(m, "PyVolumetricPlasticity3Config").def(nb::init<>());
    nb::class_<PyVolumetricPlasticity6Config, PyPlasticModelConfig>(m, "PyVolumetricPlasticity6Config").def(nb::init<>());
    nb::class_<PyShellPlasticity0Config, PyPlasticModelConfig>(m, "PyShellPlasticity0Config").def(nb::init<>());
    nb::class_<PyShellPlasticity1Config, PyPlasticModelConfig>(m, "PyShellPlasticity1Config").def(nb::init<>());
}
