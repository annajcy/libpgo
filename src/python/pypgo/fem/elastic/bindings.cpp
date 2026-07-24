#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/shared_ptr.h>

#include "core.h"
#include "../../simulation/core.h"

namespace nb = nanobind;
using namespace pgo;

void init_elastic_model_bindings(nb::module_ &m)
{
    nb::class_<PyElasticModelConfig>(m, "PyElasticModelConfig")
        .def_prop_ro("name", &PyElasticModelConfig::name)
        .def("num_channels",
            static_cast<int (PyElasticModelConfig::*)(const PySimulationMesh &) const>(&PyElasticModelConfig::numChannels));

    nb::class_<PyStableNeoConfig, PyElasticModelConfig>(m, "PyStableNeoConfig").def(nb::init<>());
    nb::class_<PyStVKConfig, PyElasticModelConfig>(m, "PyStVKConfig").def(nb::init<>());
    nb::class_<PyStVKVolumeConfig, PyElasticModelConfig>(m, "PyStVKVolumeConfig").def(nb::init<>());
    nb::class_<PyLinearElasticConfig, PyElasticModelConfig>(m, "PyLinearElasticConfig").def(nb::init<>());
    nb::class_<PyMooneyRivlinConfig, PyElasticModelConfig>(m, "PyMooneyRivlinConfig").def(nb::init<>());
    nb::class_<PyKoiterStVKConfig, PyElasticModelConfig>(m, "PyKoiterStVKConfig").def(nb::init<>());
}
