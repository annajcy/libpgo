#include "abcWriter.h"

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#if defined(PYPGO_HAS_STRESS_VDB)
#  include "stressFieldVDBExporter.h"
#endif

namespace nb = nanobind;

void init_animation_bindings(nb::module_ &m)
{
    m.def("has_animation_io", []() { return true; });

    // dump_abc(filename, name, rest_positions, displacements, triangles)
    // rest_positions : flat float list, length 3*n_verts
    // displacements  : list of n_frames flat float lists, each 3*n_verts
    // triangles      : list of n_tri [i,j,k] lists
    m.def("dump_abc",
        [](const std::string &filename,
           const std::string &name,
           const std::vector<float> &rest_positions,
           const std::vector<std::vector<float>> &displacements,
           const std::vector<std::vector<int>> &triangles) {
            nb::gil_scoped_release release;
            pgo::AnimationIO::dumpABC(
                filename.c_str(), name.c_str(),
                rest_positions, displacements, triangles);
        },
        nb::arg("filename"), nb::arg("name"),
        nb::arg("rest_positions"), nb::arg("displacements"), nb::arg("triangles"));

#if defined(PYPGO_HAS_STRESS_VDB)
    m.def("has_stress_vdb_export", []() { return true; });

    nb::class_<pgo::AnimationIO::StressFieldVDBExporter>(m, "PyStressFieldVDBExporter")
        .def(nb::init<>())
        .def("load_tet_mesh",
            [](pgo::AnimationIO::StressFieldVDBExporter &self, const std::string &veg_path) {
                nb::gil_scoped_release release;
                return self.loadTetMesh(veg_path.c_str());
            },
            nb::arg("veg_path"))
        .def("load_deformation_sequence",
            [](pgo::AnimationIO::StressFieldVDBExporter &self,
               const std::string &folder, const std::string &pattern,
               int frame_start, int frame_end) {
                nb::gil_scoped_release release;
                return self.loadDeformationSequence(
                    folder.c_str(), pattern.c_str(), frame_start, frame_end);
            },
            nb::arg("folder"), nb::arg("pattern"),
            nb::arg("frame_start"), nb::arg("frame_end"))
        .def("load_von_mises_sequence",
            [](pgo::AnimationIO::StressFieldVDBExporter &self,
               const std::string &folder, const std::string &pattern,
               int frame_start, int frame_end) {
                nb::gil_scoped_release release;
                return self.loadVonMisesSequence(
                    folder.c_str(), pattern.c_str(), frame_start, frame_end);
            },
            nb::arg("folder"), nb::arg("pattern"),
            nb::arg("frame_start"), nb::arg("frame_end"))
        .def("export_animation_vdb",
            [](const pgo::AnimationIO::StressFieldVDBExporter &self,
               const std::string &output_dir, const std::string &prefix,
               double voxel_size) {
                nb::gil_scoped_release release;
                return self.exportAnimationVDB(
                    output_dir.c_str(), prefix.c_str(), voxel_size);
            },
            nb::arg("output_dir"), nb::arg("prefix"), nb::arg("voxel_size") = 0.0)
        .def("num_frames", &pgo::AnimationIO::StressFieldVDBExporter::numFrames);
#else
    m.def("has_stress_vdb_export", []() { return false; });
#endif
}
