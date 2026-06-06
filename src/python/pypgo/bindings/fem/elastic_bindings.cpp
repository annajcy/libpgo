#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/shared_ptr.h>

#include "elastic_core.h"
#include "../simulation/core.h"

namespace nb = nanobind;
using namespace pgo;

void init_elastic_model_bindings(nb::module_ &m)
{
    nb::class_<PyElasticModel>(m, "PyElasticModel")
        .def_prop_ro("name", &PyElasticModel::name)
        .def("num_channels", [](const PyElasticModel &self, const PySimulationMesh &mesh) {
            return self.numChannels(mesh.mesh());
        });

    m.def("make_stable_neo", &make_stable_neo);
    m.def("make_stvk", &make_stvk);
    m.def("make_stvk_vol", &make_stvk_vol);
    m.def("make_linear_elastic", &make_linear_elastic);
    m.def("make_mooney_rivlin", &make_mooney_rivlin);
    m.def("make_koiter_stvk", &make_koiter_stvk);
}
