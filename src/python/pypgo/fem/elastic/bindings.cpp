#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>

#include "core.h"
#include "../../simulation/core.h"

namespace nb = nanobind;
using namespace pgo;

void init_elastic_model_bindings(nb::module_ &m)
{
    nb::class_<PyElasticModelDefinition>(m, "PyElasticModelDefinition")
        .def_prop_ro("name", &PyElasticModelDefinition::name)
        .def_prop_ro("fixed_channel_names", &PyElasticModelDefinition::fixedChannelNames)
        .def_prop_ro("optimizable_channel_names", &PyElasticModelDefinition::optimizableChannelNames)
        .def_prop_ro("frame_requirement", &PyElasticModelDefinition::frameRequirement);

    nb::class_<PyStableNeoDefinition, PyElasticModelDefinition>(m, "PyStableNeoDefinition").def(nb::init<>());
    nb::class_<PyStVKDefinition, PyElasticModelDefinition>(m, "PyStVKDefinition").def(nb::init<>());
    nb::class_<PyStVKVolumeDefinition, PyElasticModelDefinition>(m, "PyStVKVolumeDefinition").def(nb::init<>());
    nb::class_<PyLinearElasticDefinition, PyElasticModelDefinition>(m, "PyLinearElasticDefinition").def(nb::init<>());
    nb::class_<PyMooneyRivlinDefinition, PyElasticModelDefinition>(m, "PyMooneyRivlinDefinition").def(nb::init<>());
    nb::class_<PyKoiterStVKDefinition, PyElasticModelDefinition>(m, "PyKoiterStVKDefinition").def(nb::init<>());
}
