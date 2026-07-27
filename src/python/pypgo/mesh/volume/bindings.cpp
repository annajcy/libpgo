#include <nanobind/nanobind.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>

#include "core.h"

namespace nb = nanobind;
using namespace pgo;

void init_volume_mesh_bindings(nb::module_ &m)
{
    nb::enum_<PyVolumeMesh::MeshType>(m, "MeshType")
        .value("Tet", PyVolumeMesh::MeshType::Tet)
        .value("Cubic", PyVolumeMesh::MeshType::Cubic);

    nb::class_<PyVolumeMesh>(m, "PyVolumeMesh")
        .def("mesh_type", &PyVolumeMesh::meshType)
        .def("num_vertices", &PyVolumeMesh::numVertices)
        .def("num_elements", &PyVolumeMesh::numElements)
        .def("export_geometry", &export_geometry)
        .def("export_material", &export_material)
        .def("export_material_payload", &export_material_payload);

    nb::class_<PyBarycentricEmbedding>(m, "PyBarycentricEmbedding")
        .def(nb::init<const std::vector<double>&, const PyVolumeMesh&>())
        .def("num_target_locations", &PyBarycentricEmbedding::numTargetLocations)
        .def("num_volume_vertices", &PyBarycentricEmbedding::numVolumeVertices)
        .def("num_element_vertices", &PyBarycentricEmbedding::numElementVertices)
        .def("embedding_indices_flat", &PyBarycentricEmbedding::embeddingIndicesFlat)
        .def("embedding_weights_flat", &PyBarycentricEmbedding::embeddingWeightsFlat)
        .def("embedding_elements", &PyBarycentricEmbedding::embeddingElements)
        .def("interpolation_matrix", &PyBarycentricEmbedding::interpolationMatrix)
        .def("interpolation_matrix_coo", &PyBarycentricEmbedding::interpolationMatrixCOO)
        .def("deform", &PyBarycentricEmbedding::deform);

    nb::class_<PySimulationMesh>(m, "PySimulationMesh")
        .def("mesh_type", &PySimulationMesh::meshType)
        .def("num_vertices", &PySimulationMesh::numVertices)
        .def("num_elements", &PySimulationMesh::numElements)
        .def("num_element_vertices", &PySimulationMesh::numElementVertices);
    nb::class_<PySimulationImportResult>(m, "PySimulationImportResult")
        .def_prop_ro("mesh", &PySimulationImportResult::mesh)
        .def_prop_ro(
          "material_catalog",
          &PySimulationImportResult::materialCatalog);

    nb::class_<PyImportedMaterialRecord>(m, "PyImportedMaterialRecord")
        .def(nb::init<std::string, std::string, nb::dict>(),
            nb::arg("name"), nb::arg("family"), nb::arg("properties"))
        .def_prop_ro("name", &PyImportedMaterialRecord::name)
        .def_prop_ro("family", &PyImportedMaterialRecord::family)
        .def_prop_ro("properties", &PyImportedMaterialRecord::properties);
    nb::class_<PyImportedElementSet>(m, "PyImportedElementSet")
        .def(nb::init<std::string, std::vector<int>>(),
            nb::arg("name"), nb::arg("elements"))
        .def_prop_ro("name", &PyImportedElementSet::name)
        .def_prop_ro("elements", &PyImportedElementSet::elements);
    nb::class_<PyImportedMaterialRegion>(m, "PyImportedMaterialRegion")
        .def(nb::init<int, int>(), nb::arg("material_index"), nb::arg("set_index"))
        .def_prop_ro("material_index", &PyImportedMaterialRegion::materialIndex)
        .def_prop_ro("set_index", &PyImportedMaterialRegion::setIndex);
    nb::class_<PyNamedMaterialInputField>(m, "PyNamedMaterialInputField")
        .def(nb::init<std::string, std::vector<std::string>,
                      std::vector<std::vector<double>>, std::vector<int>>(),
            nb::arg("name"), nb::arg("channel_names"), nb::arg("value_rows"),
            nb::arg("element_to_row"))
        .def_prop_ro("name", &PyNamedMaterialInputField::name)
        .def_prop_ro("channel_names", &PyNamedMaterialInputField::channelNames)
        .def_prop_ro("value_rows", &PyNamedMaterialInputField::valueRows)
        .def_prop_ro("element_to_row", &PyNamedMaterialInputField::elementToRow);
    nb::class_<PyImportedMaterialCatalog>(m, "PyImportedMaterialCatalog")
        .def(nb::init<int, std::vector<PyImportedMaterialRecord>,
                      std::vector<PyImportedElementSet>,
                      std::vector<PyImportedMaterialRegion>>(),
            nb::arg("num_elements"), nb::arg("materials"), nb::arg("sets"),
            nb::arg("regions"))
        .def_prop_ro("num_elements", &PyImportedMaterialCatalog::numElements)
        .def_prop_ro("materials", &PyImportedMaterialCatalog::materials)
        .def_prop_ro("sets", &PyImportedMaterialCatalog::sets)
        .def_prop_ro("regions", &PyImportedMaterialCatalog::regions)
        .def_prop_ro("element_material_indices", &PyImportedMaterialCatalog::elementMaterialIndices);
    nb::class_<PyNamedMaterialInputData>(m, "PyNamedMaterialInputData")
        .def(nb::init<int, std::vector<PyNamedMaterialInputField>>(),
            nb::arg("num_elements"), nb::arg("fields"))
        .def_prop_ro("num_elements", &PyNamedMaterialInputData::numElements)
        .def_prop_ro("fields", &PyNamedMaterialInputData::fields);

    nb::class_<PyVegENuMaterialPayload>(m, "PyVegENuMaterialPayload")
        .def_rw("name", &PyVegENuMaterialPayload::name)
        .def_rw("density", &PyVegENuMaterialPayload::density)
        .def_rw("E", &PyVegENuMaterialPayload::E)
        .def_rw("nu", &PyVegENuMaterialPayload::nu);

    nb::class_<PyVegMooneyRivlinMaterialPayload>(m, "PyVegMooneyRivlinMaterialPayload")
        .def_rw("name", &PyVegMooneyRivlinMaterialPayload::name)
        .def_rw("density", &PyVegMooneyRivlinMaterialPayload::density)
        .def_rw("mu01", &PyVegMooneyRivlinMaterialPayload::mu01)
        .def_rw("mu10", &PyVegMooneyRivlinMaterialPayload::mu10)
        .def_rw("v1", &PyVegMooneyRivlinMaterialPayload::v1);

    nb::class_<PyVegOrthotropicMaterialPayload>(m, "PyVegOrthotropicMaterialPayload")
        .def_rw("name", &PyVegOrthotropicMaterialPayload::name)
        .def_rw("density", &PyVegOrthotropicMaterialPayload::density)
        .def_rw("E1", &PyVegOrthotropicMaterialPayload::E1)
        .def_rw("E2", &PyVegOrthotropicMaterialPayload::E2)
        .def_rw("E3", &PyVegOrthotropicMaterialPayload::E3)
        .def_rw("nu12", &PyVegOrthotropicMaterialPayload::nu12)
        .def_rw("nu23", &PyVegOrthotropicMaterialPayload::nu23)
        .def_rw("nu31", &PyVegOrthotropicMaterialPayload::nu31)
        .def_rw("G12", &PyVegOrthotropicMaterialPayload::G12)
        .def_rw("G23", &PyVegOrthotropicMaterialPayload::G23)
        .def_rw("G31", &PyVegOrthotropicMaterialPayload::G31)
        .def_rw("R", &PyVegOrthotropicMaterialPayload::R);

    nb::class_<PyVegPayload>(m, "PyVegPayload")
        .def_prop_ro("mesh_data", &vegPayloadMeshData)
        .def_prop_ro("materials", &vegPayloadMaterials)
        .def_prop_ro("sets", &vegPayloadSets)
        .def_prop_ro("regions", &vegPayloadRegions);

    m.def("_create_enu_material_payload", &create_enu_material_payload);
    m.def("_create_mooney_rivlin_material_payload", &create_mooney_rivlin_material_payload);
    m.def("_create_orthotropic_material_payload", &create_orthotropic_material_payload);
    m.def("create_volume_mesh", &create_volume_mesh);
    m.def("_create_veg_payload", &create_veg_payload,
        nb::arg("mesh_data"), nb::arg("materials"),
        nb::arg("sets"), nb::arg("regions"));
    m.def("_create_volume_mesh_from_veg_payload",
        &create_volume_mesh_from_veg_payload, nb::arg("payload"));
    m.def("load_volume_mesh", &load_volume_mesh);
    m.def("save_volume_mesh", &save_volume_mesh);
    m.def("_read_veg_payload", &read_veg);
    m.def("read_msh", &read_msh);
    m.def("_write_veg_payload", &write_veg,
        nb::arg("path"), nb::arg("payload"));
    m.def("extract_surface_mesh", &extract_surface_mesh, nb::arg("volume_mesh"), nb::arg("triangulate") = true);
    m.def("extract_veg_payload_from_volume_mesh", &extract_veg_payload_from_volume_mesh, nb::arg("volume_mesh"));
    m.def("_import_simulation_mesh_from_volume", &import_simulation_mesh_from_volume);
    m.def("_create_shell_simulation_mesh", &create_shell_simulation_mesh,
        nb::arg("surface_data"));
    m.def("compute_mass_matrix", &compute_mass_matrix,
        nb::arg("volume_mesh"), nb::arg("inflate3dim") = true);
}
