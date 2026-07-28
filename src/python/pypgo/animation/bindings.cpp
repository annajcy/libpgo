#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include "core.h"

namespace nb = nanobind;

void init_animation_bindings(nb::module_ &m)
{
    m.def("has_animation_io", &pgo::has_animation_io);

    m.def("dump_abc", &pgo::dump_abc,
        nb::arg("filename"), nb::arg("name"),
        nb::arg("rest_positions"), nb::arg("displacements"),
        nb::arg("triangles"), nb::arg("fps") = 24.0);

    nb::class_<pgo::PyAnimationLoader>(m, "PyAnimationLoader")
        .def(nb::init<>())
        .def("load", &pgo::PyAnimationLoader::load, nb::arg("filename"))
        .def("save_abc", &pgo::PyAnimationLoader::saveABC, nb::arg("prefix"));

    m.def("has_stress_vdb_export", &pgo::has_stress_vdb_export);

#if defined(PYPGO_HAS_STRESS_VDB)
    nb::class_<pgo::PyStressFieldVDBExporter>(m, "PyStressFieldVDBExporter")
        .def(nb::init<>())
        .def("load_tet_mesh", &pgo::PyStressFieldVDBExporter::loadTetMesh, nb::arg("veg_path"))
        .def("load_deformation_sequence", &pgo::PyStressFieldVDBExporter::loadDeformationSequence,
            nb::arg("folder"), nb::arg("pattern"), nb::arg("frame_start"), nb::arg("frame_end"))
        .def("load_von_mises_sequence", &pgo::PyStressFieldVDBExporter::loadVonMisesSequence,
            nb::arg("folder"), nb::arg("pattern"), nb::arg("frame_start"), nb::arg("frame_end"))
        .def("export_animation_vdb", &pgo::PyStressFieldVDBExporter::exportAnimationVDB,
            nb::arg("output_dir"), nb::arg("prefix"), nb::arg("voxel_size") = 0.0)
        .def("num_frames", &pgo::PyStressFieldVDBExporter::numFrames);
#endif
}
