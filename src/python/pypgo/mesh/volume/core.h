#pragma once

#include "barycentricCoordinates.h"
#include "common.h"
#include "../geo/core.h"
#include "meshData.h"
#include "volumetricMesh.h"
#include "tetMesh.h"
#include "cubicMesh.h"
#include "../../simulation/core.h"
#include "../../sparse/core.h"

#include <nanobind/nanobind.h>

#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace pgo
{

namespace nb = nanobind;

class PyVolumeMesh {
public:
    enum class MeshType { Tet, Cubic };

    PyVolumeMesh(std::unique_ptr<VolumetricMeshes::TetMesh> tetMesh)
        : type_(MeshType::Tet), tetMesh_(std::move(tetMesh)) {}

    PyVolumeMesh(std::unique_ptr<VolumetricMeshes::CubicMesh> cubicMesh)
        : type_(MeshType::Cubic), cubicMesh_(std::move(cubicMesh)) {}

    explicit PyVolumeMesh(
        std::unique_ptr<VolumetricMeshes::VolumetricMesh> volumeMesh);

    MeshType meshType() const { return type_; }

    int numVertices() const { return getVM()->getNumVertices(); }
    int numElements() const { return getVM()->getNumElements(); }

    const VolumetricMeshes::VolumetricMesh* getVM() const {
        return type_ == MeshType::Tet
            ? static_cast<const VolumetricMeshes::VolumetricMesh*>(tetMesh_.get())
            : static_cast<const VolumetricMeshes::VolumetricMesh*>(cubicMesh_.get());
    }

private:
    MeshType type_;
    std::unique_ptr<VolumetricMeshes::TetMesh> tetMesh_;
    std::unique_ptr<VolumetricMeshes::CubicMesh> cubicMesh_;
};

struct PyVegENuMaterialPayload {
    std::string name;
    double density;
    double E;
    double nu;
};

struct PyVegMooneyRivlinMaterialPayload {
    std::string name;
    double density;
    double mu01;
    double mu10;
    double v1;
};

struct PyVegOrthotropicMaterialPayload {
    std::string name;
    double density;
    double E1;
    double E2;
    double E3;
    double nu12;
    double nu23;
    double nu31;
    double G12;
    double G23;
    double G31;
    std::vector<double> R;
};

struct PyVegPayload {
    VolumetricMeshes::VegMeshData meshData;
    std::vector<VolumetricMeshes::VegMaterialPayload> materials;
    std::vector<std::pair<std::string, std::vector<int>>> sets;
    std::vector<std::pair<int, int>> regions;
};

class PyBarycentricEmbedding {
public:
    PyBarycentricEmbedding(const std::vector<double>& targetLocationsFlat, const PyVolumeMesh& volumeMesh);

    PySparseMatrix interpolationMatrix() const;
    std::tuple<std::vector<int>, std::vector<int>, std::vector<double>> interpolationMatrixCOO() const;
    int numElementVertices() const;
    std::vector<int> embeddingIndicesFlat() const;
    std::vector<double> embeddingWeightsFlat() const;
    std::vector<int> embeddingElements() const;
    std::vector<double> deform(const std::vector<double>& volumeDispFlat) const;
    int numTargetLocations() const { return numTargetLocations_; }
    int numVolumeVertices() const { return numVolumeVertices_; }

private:
    std::unique_ptr<InterpolationCoordinates::BarycentricCoordinates> coords_;
    int numTargetLocations_ = 0;
    int numVolumeVertices_ = 0;
};

PySparseMatrix compute_mass_matrix(const PyVolumeMesh& volumeMesh, bool inflate3Dim);

std::shared_ptr<PyVolumeMesh> create_volume_mesh(const nb::object& meshDataObj, const PyMaterialSpec& mat);
std::shared_ptr<PyVolumeMesh> load_volume_mesh(const std::string& path);
void save_volume_mesh(const std::string& path, const PyVolumeMesh& vm);
nb::object export_geometry(const PyVolumeMesh& vm);
PyMaterialSpec export_material(const PyVolumeMesh& vm);
nb::object export_material_payload(const PyVolumeMesh& vm);
std::shared_ptr<PyVegPayload> extract_veg_payload_from_volume_mesh(const PyVolumeMesh& vm);
std::shared_ptr<PyVegPayload> create_veg_payload(
    const nb::object& meshDataObj,
    const std::vector<nb::object>& materialPayloads,
    const std::vector<std::pair<std::string, std::vector<int>>>& setPayloads,
    const std::vector<std::pair<int, int>>& regionPayloads);
std::shared_ptr<PyVolumeMesh> create_volume_mesh_from_veg_payload(
    const PyVegPayload& payload);

std::shared_ptr<PyVegPayload> read_veg(const std::string& path);
PyTetMeshData read_msh(const std::string& path);

void write_veg(const std::string& path, const PyVegPayload& payload);

PyTriMeshData extract_surface_mesh(const PyVolumeMesh& vm, bool triangulate);
std::shared_ptr<PySimulationMesh> create_volume_simulation_mesh(
    const PyVolumeMesh& vm);
std::shared_ptr<PySimulationMesh> create_shell_simulation_mesh(
    const PyTriMeshData& surfaceData);

PyVegENuMaterialPayload create_enu_material_payload(
    const std::string& name, double density, double E, double nu);
PyVegMooneyRivlinMaterialPayload create_mooney_rivlin_material_payload(
    const std::string& name, double density, double mu01, double mu10, double v1);
PyVegOrthotropicMaterialPayload create_orthotropic_material_payload(
    const std::string& name,
    double density,
    double E1,
    double E2,
    double E3,
    double nu12,
    double nu23,
    double nu31,
    double G12,
    double G23,
    double G31,
    const std::vector<double>& R);

nb::object vegPayloadMeshData(const PyVegPayload& self);
nb::list vegPayloadMaterials(const PyVegPayload& self);
std::vector<std::pair<std::string, std::vector<int>>> vegPayloadSets(const PyVegPayload& self);
std::vector<std::pair<int, int>> vegPayloadRegions(const PyVegPayload& self);

}  // namespace pgo
