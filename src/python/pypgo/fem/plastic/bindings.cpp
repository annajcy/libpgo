#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>

#include "core.h"

namespace nb = nanobind;
using namespace pgo;

void init_plastic_model_bindings(nb::module_ &m)
{
    nb::class_<PyPlasticModelDefinition>(m, "PyPlasticModelDefinition")
        .def_prop_ro("name", &PyPlasticModelDefinition::name)
        .def_prop_ro("dofs", &PyPlasticModelDefinition::dofs)
        .def_prop_ro("num_fixed_channels", &PyPlasticModelDefinition::numFixedChannels)
        .def_prop_ro("num_optimizable_channels", &PyPlasticModelDefinition::numOptimizableChannels);
    nb::class_<PyVolumetricPlasticity0Definition, PyPlasticModelDefinition>(m, "PyVolumetricPlasticity0Definition").def(nb::init<>());
    nb::class_<PyVolumetricPlasticity3Definition, PyPlasticModelDefinition>(m, "PyVolumetricPlasticity3Definition").def(nb::init<>());
    nb::class_<PyVolumetricPlasticity6Definition, PyPlasticModelDefinition>(m, "PyVolumetricPlasticity6Definition").def(nb::init<>());
    nb::class_<PyShellPlasticity0Definition, PyPlasticModelDefinition>(m, "PyShellPlasticity0Definition").def(nb::init<>());
    nb::class_<PyShellPlasticity1Definition, PyPlasticModelDefinition>(m, "PyShellPlasticity1Definition").def(nb::init<>());
}
