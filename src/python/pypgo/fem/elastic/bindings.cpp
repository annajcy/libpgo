#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>

#include "core.h"
#include "../../simulation/core.h"

namespace nb = nanobind;
using namespace pgo;

namespace
{
template<class PyDefinition>
void bindDefaultElasticDefinition(nb::module_ &m, const char *pythonName)
{
    nb::class_<PyDefinition, PyElasticModelDefinition>(m, pythonName).def(nb::init<>());
}
}  // namespace

void init_elastic_model_bindings(nb::module_ &m)
{
    nb::class_<PyElasticModelDefinition>(m, "PyElasticModelDefinition")
        .def_prop_ro("name", &PyElasticModelDefinition::name)
        .def_prop_ro("num_fixed_channels", &PyElasticModelDefinition::numFixedChannels)
        .def_prop_ro("num_optimizable_channels", &PyElasticModelDefinition::numOptimizableChannels);

    bindDefaultElasticDefinition<PyStableNeoDefinition>(m, "PyStableNeoDefinition");
    bindDefaultElasticDefinition<PyNeoHookeanDefinition>(m, "PyNeoHookeanDefinition");
    bindDefaultElasticDefinition<PyStVKDefinition>(m, "PyStVKDefinition");
    bindDefaultElasticDefinition<PyStVKVolumeDefinition>(m, "PyStVKVolumeDefinition");
    bindDefaultElasticDefinition<PyLinearElasticDefinition>(m, "PyLinearElasticDefinition");
    bindDefaultElasticDefinition<PyMooneyRivlinDefinition>(m, "PyMooneyRivlinDefinition");
    bindDefaultElasticDefinition<PyKoiterStVKDefinition>(m, "PyKoiterStVKDefinition");
    nb::class_<PySystematicPokingDefinition, PyElasticModelDefinition>(
        m, "PySystematicPokingDefinition")
        .def(nb::init<
            const std::vector<double> &,
            int,
            const std::vector<double> &,
            int>(),
            nb::arg("stretch_knots"),
            nb::arg("stretch_rest_knot_index"),
            nb::arg("volume_knots"),
            nb::arg("volume_rest_knot_index"));
}
