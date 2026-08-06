#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>

#include "core.h"

namespace nb = nanobind;
using namespace pgo;

namespace
{
template<class PyDefinition>
void bindDefaultPlasticDefinition(nb::module_ &m, const char *pythonName)
{
    nb::class_<PyDefinition, PyPlasticModelDefinition>(m, pythonName).def(nb::init<>());
}
}  // namespace

void init_plastic_model_bindings(nb::module_ &m)
{
    nb::class_<PyPlasticModelDefinition>(m, "PyPlasticModelDefinition")
        .def_prop_ro("name", &PyPlasticModelDefinition::name)
        .def_prop_ro("dofs", &PyPlasticModelDefinition::dofs)
        .def_prop_ro("num_fixed_channels", &PyPlasticModelDefinition::numFixedChannels)
        .def_prop_ro("num_optimizable_channels", &PyPlasticModelDefinition::numOptimizableChannels);
    bindDefaultPlasticDefinition<PyVolumetricPlasticity0Definition>(m, "PyVolumetricPlasticity0Definition");
    bindDefaultPlasticDefinition<PyVolumetricPlasticity3Definition>(m, "PyVolumetricPlasticity3Definition");
    bindDefaultPlasticDefinition<PyVolumetricPlasticity6Definition>(m, "PyVolumetricPlasticity6Definition");
    bindDefaultPlasticDefinition<PyShellPlasticity0Definition>(m, "PyShellPlasticity0Definition");
    bindDefaultPlasticDefinition<PyShellPlasticity1Definition>(m, "PyShellPlasticity1Definition");
}
