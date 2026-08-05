#include "core.h"

#include "generateMassMatrix.h"
#include "generateSurfaceMesh.h"
#if defined(PGO_HAS_GMSH)
#  include "loadMshFile.h"
#endif
#include "triMeshGeo.h"
#include "vegFile.h"
#include "volumetricMeshENuMaterial.h"
#include "volumetricMeshMooneyRivlinMaterial.h"
#include "volumetricMeshOrthotropicMaterial.h"

#include "simulation/simulationMesh.h"

#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

namespace pgo
{
namespace
{
using VM = VolumetricMeshes::VolumetricMesh;

template<class T>
nb::list makePythonList(const std::vector<T>& values)
{
    nb::list result;
    for (const auto& value : values) {
        result.append(value);
    }
    return result;
}

nb::object materialPayloadFromMaterial(const VM::Material* material) {
    if (auto* enu = VolumetricMeshes::downcastENuMaterial(material)) {
        return nb::cast(PyVegENuMaterialPayload{
            enu->getName(), enu->getDensity(), enu->getE(), enu->getNu() });
    }

    auto* mutableMaterial = const_cast<VM::Material*>(material);
    if (auto* mooney = VolumetricMeshes::downcastMooneyRivlinMaterial(mutableMaterial)) {
        return nb::cast(PyVegMooneyRivlinMaterialPayload{
            mooney->getName(), mooney->getDensity(), mooney->getmu01(), mooney->getmu10(), mooney->getv1() });
    }
    if (auto* orthotropic = VolumetricMeshes::downcastOrthotropicMaterial(mutableMaterial)) {
        std::vector<double> R(9);
        orthotropic->getR(R.data());
        return nb::cast(PyVegOrthotropicMaterialPayload{
            orthotropic->getName(), orthotropic->getDensity(),
            orthotropic->getE1(), orthotropic->getE2(), orthotropic->getE3(),
            orthotropic->getNu12(), orthotropic->getNu23(), orthotropic->getNu31(),
            orthotropic->getG12(), orthotropic->getG23(), orthotropic->getG31(),
            std::move(R) });
    }
    throw std::runtime_error("unsupported material type in volume mesh");
}

nb::object materialPayloadFromVegPayload(const VolumetricMeshes::VegMaterialPayload& payload)
{
    return std::visit([](const auto& material) -> nb::object {
        using T = std::decay_t<decltype(material)>;
        if constexpr (std::is_same_v<T, VolumetricMeshes::VegENuMaterialPayload>) {
            return nb::cast(PyVegENuMaterialPayload{
                material.name, material.density, material.E, material.nu });
        }
        else if constexpr (std::is_same_v<T, VolumetricMeshes::VegMooneyRivlinMaterialPayload>) {
            return nb::cast(PyVegMooneyRivlinMaterialPayload{
                material.name, material.density, material.mu01, material.mu10, material.v1 });
        }
        else {
            return nb::cast(PyVegOrthotropicMaterialPayload{
                material.name, material.density,
                material.E1, material.E2, material.E3,
                material.nu12, material.nu23, material.nu31,
                material.G12, material.G23, material.G31,
                std::vector<double>(material.R.begin(), material.R.end()) });
        }
    }, payload);
}

nb::object meshDataFromVegPayload(const VolumetricMeshes::VegMeshData& meshData)
{
    return std::visit([](const auto& data) -> nb::object {
        using T = std::decay_t<decltype(data)>;
        if constexpr (std::is_same_v<T, Mesh::MeshData<4>>) {
            return nb::cast(PyTetMeshData(data));
        }
        else {
            return nb::cast(PyCubicMeshData(data));
        }
    }, meshData);
}

VolumetricMeshes::VegMaterialPayload vegPayloadFromMaterialObject(const nb::object& obj) {
    if (nb::isinstance<PyVegENuMaterialPayload>(obj)) {
        auto p = nb::cast<PyVegENuMaterialPayload>(obj);
        return VolumetricMeshes::VegENuMaterialPayload{ p.name, p.density, p.E, p.nu };
    }
    if (nb::isinstance<PyVegMooneyRivlinMaterialPayload>(obj)) {
        auto p = nb::cast<PyVegMooneyRivlinMaterialPayload>(obj);
        return VolumetricMeshes::VegMooneyRivlinMaterialPayload{ p.name, p.density, p.mu01, p.mu10, p.v1 };
    }
    if (nb::isinstance<PyVegOrthotropicMaterialPayload>(obj)) {
        auto p = nb::cast<PyVegOrthotropicMaterialPayload>(obj);
        if (p.R.size() != 9) {
            throw std::runtime_error("Orthotropic material R must contain 9 row-major values");
        }
        VolumetricMeshes::VegOrthotropicMaterialPayload payload;
        payload.name = p.name;
        payload.density = p.density;
        payload.E1 = p.E1;
        payload.E2 = p.E2;
        payload.E3 = p.E3;
        payload.nu12 = p.nu12;
        payload.nu23 = p.nu23;
        payload.nu31 = p.nu31;
        payload.G12 = p.G12;
        payload.G23 = p.G23;
        payload.G31 = p.G31;
        std::copy(p.R.begin(), p.R.end(), payload.R.begin());
        return payload;
    }
    throw std::runtime_error("unsupported material payload type");
}

std::shared_ptr<PyVegPayload> makePyVegPayload(VolumetricMeshes::VegFilePayload payload)
{
    auto result = std::make_shared<PyVegPayload>();
    result->meshData = std::move(payload.meshData);
    result->materials = std::move(payload.materials);
    for (const auto& set : payload.sets) {
        result->sets.emplace_back(set.name, set.elements);
    }
    for (const auto& region : payload.regions) {
        result->regions.emplace_back(region.materialIndex, region.setIndex);
    }
    return result;
}

VolumetricMeshes::VegFilePayload makeVegFilePayload(
    const nb::object& meshDataObj,
    const std::vector<nb::object>& materialPayloads,
    const std::vector<std::pair<std::string, std::vector<int>>>& setPayloads,
    const std::vector<std::pair<int, int>>& regionPayloads)
{
    VolumetricMeshes::VegFilePayload payload;
    if (nb::isinstance<PyTetMeshData>(meshDataObj)) {
        payload.meshData = nb::cast<const PyTetMeshData&>(meshDataObj).core();
    }
    else if (nb::isinstance<PyCubicMeshData>(meshDataObj)) {
        payload.meshData = nb::cast<const PyCubicMeshData&>(meshDataObj).core();
    }
    else {
        throw nb::type_error("VegFile mesh_data must be TetMeshData or CubicMeshData");
    }

    payload.materials.reserve(materialPayloads.size());
    for (const auto& material : materialPayloads)
        payload.materials.push_back(vegPayloadFromMaterialObject(material));
    payload.sets.reserve(setPayloads.size());
    for (const auto& [name, elements] : setPayloads)
        payload.sets.push_back(VolumetricMeshes::VegSetPayload{ name, elements });
    payload.regions.reserve(regionPayloads.size());
    for (const auto& [materialIndex, setIndex] : regionPayloads)
        payload.regions.push_back(
            VolumetricMeshes::VegRegionPayload{ materialIndex, setIndex });
    return payload;
}

VolumetricMeshes::VegFilePayload makeVegFilePayload(const PyVegPayload& source)
{
    VolumetricMeshes::VegFilePayload payload;
    payload.meshData = source.meshData;
    payload.materials = source.materials;
    payload.sets.reserve(source.sets.size());
    for (const auto& [name, elements] : source.sets)
        payload.sets.push_back(VolumetricMeshes::VegSetPayload{ name, elements });
    payload.regions.reserve(source.regions.size());
    for (const auto& [materialIndex, setIndex] : source.regions)
        payload.regions.push_back(
            VolumetricMeshes::VegRegionPayload{ materialIndex, setIndex });
    return payload;
}

}  // namespace

PyVolumeMesh::PyVolumeMesh(
    std::unique_ptr<VolumetricMeshes::VolumetricMesh> volumeMesh):
    type_(MeshType::Tet)
{
    if (!volumeMesh)
        throw std::invalid_argument("PyVolumeMesh requires a non-null mesh.");
    if (volumeMesh->getElementType() == VolumetricMeshes::VolumetricMesh::TET) {
        auto* tet = dynamic_cast<VolumetricMeshes::TetMesh*>(volumeMesh.get());
        if (!tet)
            throw std::runtime_error("Tet volume mesh has an unexpected dynamic type.");
        volumeMesh.release();
        tetMesh_.reset(tet);
        return;
    }
    if (volumeMesh->getElementType() == VolumetricMeshes::VolumetricMesh::CUBIC) {
        type_ = MeshType::Cubic;
        auto* cubic = dynamic_cast<VolumetricMeshes::CubicMesh*>(volumeMesh.get());
        if (!cubic)
            throw std::runtime_error("Cubic volume mesh has an unexpected dynamic type.");
        volumeMesh.release();
        cubicMesh_.reset(cubic);
        return;
    }
    throw std::invalid_argument("PyVolumeMesh supports only tet and cubic meshes.");
}

PyBarycentricEmbedding::PyBarycentricEmbedding(const std::vector<double>& targetLocationsFlat, const PyVolumeMesh& volumeMesh)
    : numTargetLocations_(static_cast<int>(targetLocationsFlat.size() / 3)),
      numVolumeVertices_(volumeMesh.numVertices())
{
    if (targetLocationsFlat.size() % 3 != 0) {
        throw std::runtime_error("target locations must be a flat 3*m vector");
    }
    {
        nb::gil_scoped_release release;
        coords_ = std::make_unique<InterpolationCoordinates::BarycentricCoordinates>(
            numTargetLocations_, targetLocationsFlat.data(), volumeMesh.getVM());
    }
}

PySparseMatrix PyBarycentricEmbedding::interpolationMatrix() const
{
    EigenSupport::SpMatD matrix;
    {
        nb::gil_scoped_release release;
        matrix = coords_->generateInterpolationMatrix();
    }
    return PySparseMatrix(std::move(matrix));
}

std::tuple<std::vector<int>, std::vector<int>, std::vector<double>> PyBarycentricEmbedding::interpolationMatrixCOO() const
{
    return interpolationMatrix().toCOO();
}

int PyBarycentricEmbedding::numElementVertices() const { return coords_->getNumElementVertices(); }

std::vector<int> PyBarycentricEmbedding::embeddingIndicesFlat() const
{
    const auto& indices = coords_->getEmbeddingVertexIndices();
    return std::vector<int>(indices.begin(), indices.end());
}

std::vector<double> PyBarycentricEmbedding::embeddingWeightsFlat() const
{
    const auto& weights = coords_->getEmbeddingWeights();
    return std::vector<double>(weights.begin(), weights.end());
}

std::vector<int> PyBarycentricEmbedding::embeddingElements() const
{
    const auto& elements = coords_->getElements();
    return std::vector<int>(elements.begin(), elements.end());
}

std::vector<double> PyBarycentricEmbedding::deform(const std::vector<double>& volumeDispFlat) const
{
    if (static_cast<int>(volumeDispFlat.size()) != numVolumeVertices_ * 3) {
        throw std::runtime_error("volume displacement length must equal 3 * volume mesh vertices");
    }
    std::vector<double> surfaceDisp(static_cast<size_t>(numTargetLocations_) * 3);
    {
        nb::gil_scoped_release release;
        coords_->deform(volumeDispFlat.data(), surfaceDisp.data());
    }
    return surfaceDisp;
}

PySparseMatrix compute_mass_matrix(const PyVolumeMesh& volumeMesh, bool inflate3Dim)
{
    EigenSupport::SpMatD M;
    {
        nb::gil_scoped_release release;
        VolumetricMeshes::GenerateMassMatrix::computeMassMatrix(volumeMesh.getVM(), M, inflate3Dim);
    }
    return PySparseMatrix(std::move(M));
}

std::shared_ptr<PyVolumeMesh> create_volume_mesh(const nb::object& meshDataObj, const PyMaterialSpec& mat) {
    if (nb::isinstance<PyTetMeshData>(meshDataObj)) {
        const auto& meshData = nb::cast<const PyTetMeshData&>(meshDataObj);
        std::unique_ptr<VolumetricMeshes::TetMesh> tetMesh;
        {
            nb::gil_scoped_release release;
            tetMesh = std::make_unique<VolumetricMeshes::TetMesh>(meshData.core(), mat.E(), mat.nu(), mat.density());
        }
        return std::make_shared<PyVolumeMesh>(std::move(tetMesh));
    } else if (nb::isinstance<PyCubicMeshData>(meshDataObj)) {
        const auto& meshData = nb::cast<const PyCubicMeshData&>(meshDataObj);
        std::unique_ptr<VolumetricMeshes::CubicMesh> cubicMesh;
        {
            nb::gil_scoped_release release;
            cubicMesh = std::make_unique<VolumetricMeshes::CubicMesh>(meshData.core(), mat.E(), mat.nu(), mat.density());
        }
        return std::make_shared<PyVolumeMesh>(std::move(cubicMesh));
    } else {
        throw std::runtime_error("Unsupported element mesh type for create_volume_mesh");
    }
}

std::shared_ptr<PyVolumeMesh> load_volume_mesh(const std::string& path) {
    auto type = VolumetricMeshes::VolumetricMesh::getElementType(path.c_str());
    if (type == VolumetricMeshes::VolumetricMesh::TET) {
        std::unique_ptr<VolumetricMeshes::TetMesh> mesh;
        {
            nb::gil_scoped_release release;
            mesh = std::make_unique<VolumetricMeshes::TetMesh>(path.c_str());
        }
        return std::make_shared<PyVolumeMesh>(std::move(mesh));
    } else if (type == VolumetricMeshes::VolumetricMesh::CUBIC) {
        std::unique_ptr<VolumetricMeshes::CubicMesh> mesh;
        {
            nb::gil_scoped_release release;
            mesh = std::make_unique<VolumetricMeshes::CubicMesh>(path.c_str());
        }
        return std::make_shared<PyVolumeMesh>(std::move(mesh));
    } else {
        throw std::runtime_error("Unsupported or invalid volumetric mesh type in file: " + path);
    }
}

void save_volume_mesh(const std::string& path, const PyVolumeMesh& vm) {
    int result = 0;
    {
        nb::gil_scoped_release release;
        result = vm.getVM()->saveToAscii(path.c_str());
    }
    if (result != 0) {
        throw std::runtime_error("Failed to save volume mesh to " + path);
    }
}

nb::object export_geometry(const PyVolumeMesh& vm) {
    const auto* vMesh = vm.getVM();
    std::vector<Vec3d> vertices;
    std::vector<int> elements;
    vMesh->exportMeshGeometry(vertices, elements);

    if (vm.meshType() == PyVolumeMesh::MeshType::Tet) {
        auto meshData = Mesh::MeshData<4>::fromFlatElements(std::move(vertices), std::move(elements));
        return nb::cast(PyTetMeshData(std::move(meshData)));
    } else {
        auto meshData = Mesh::MeshData<8>::fromFlatElements(std::move(vertices), std::move(elements));
        return nb::cast(PyCubicMeshData(std::move(meshData)));
    }
}

PyMaterialSpec export_material(const PyVolumeMesh& vm) {
    const auto* vMesh = vm.getVM();
    if (vMesh->getNumMaterials() > 0) {
        auto* enumMat = VolumetricMeshes::downcastENuMaterial(vMesh->getMaterial(0));
        if (enumMat) {
            return PyMaterialSpec(enumMat->getE(), enumMat->getNu(), enumMat->getDensity());
        }
        return PyMaterialSpec(1e9, 0.45, vMesh->getMaterial(0)->getDensity());
    }
    return PyMaterialSpec();
}

nb::object export_material_payload(const PyVolumeMesh& vm) {
    const auto* vMesh = vm.getVM();
    if (vMesh->getNumMaterials() == 0) {
        return nb::cast(PyVegENuMaterialPayload{ "defaultMaterial", 1000.0, 1e9, 0.45 });
    }
    return materialPayloadFromMaterial(vMesh->getMaterial(0));
}

std::shared_ptr<PyVegPayload> extract_veg_payload_from_volume_mesh(const PyVolumeMesh& vm) {
    VolumetricMeshes::VegFilePayload payload;
    {
        nb::gil_scoped_release release;
        payload = vm.getVM()->toVegFilePayload();
    }

    return makePyVegPayload(std::move(payload));
}

std::shared_ptr<PyVegPayload> create_veg_payload(
    const nb::object& meshDataObj,
    const std::vector<nb::object>& materialPayloads,
    const std::vector<std::pair<std::string, std::vector<int>>>& setPayloads,
    const std::vector<std::pair<int, int>>& regionPayloads)
{
    return makePyVegPayload(makeVegFilePayload(
        meshDataObj, materialPayloads, setPayloads, regionPayloads));
}

std::shared_ptr<PyVolumeMesh> create_volume_mesh_from_veg_payload(
    const PyVegPayload& payload)
{
    std::unique_ptr<VolumetricMeshes::VolumetricMesh> mesh;
    {
        nb::gil_scoped_release release;
        mesh = VolumetricMeshes::VolumetricMesh::fromVegFilePayload(
            makeVegFilePayload(payload));
    }
    return std::make_shared<PyVolumeMesh>(std::move(mesh));
}

std::shared_ptr<PyVegPayload> read_veg(const std::string& path) {
    return makePyVegPayload(VolumetricMeshes::readVegFile(path));
}

PyTetMeshData read_msh(const std::string& path) {
#if !defined(PGO_HAS_GMSH)
    (void)path;
    throw std::runtime_error("Gmsh .msh reader is not available in this build.");
#else
    std::vector<Vec3d> vertices;
    std::vector<int> elements;
    {
        nb::gil_scoped_release release;
        auto tetMesh = VolumetricMeshes::loadMshFile(path.c_str());

        // Extract geometry as Vec3d + Vec4i, then flatten to int for MeshData<4>
        std::vector<Vec4i> tetElements;
        tetMesh.exportMeshGeometry(vertices, tetElements);

        elements.reserve(tetElements.size() * 4);
        for (const auto& tet : tetElements) {
            elements.push_back(tet[0]);
            elements.push_back(tet[1]);
            elements.push_back(tet[2]);
            elements.push_back(tet[3]);
        }
    }
    return PyTetMeshData(
        Mesh::MeshData<4>::fromFlatElements(std::move(vertices), std::move(elements)));
#endif
}

void write_veg(const std::string& path, const PyVegPayload& source)
{
    {
        nb::gil_scoped_release release;
        VolumetricMeshes::writeVegFile(path, makeVegFilePayload(source));
    }
}

PyTriMeshData extract_surface_mesh(const PyVolumeMesh& vm, bool triangulate)
{
    std::vector<Vec3d> vertices;
    std::vector<std::vector<int>> faces;
    {
        nb::gil_scoped_release release;
        VolumetricMeshes::GenerateSurfaceMesh::computeMesh(vm.getVM(), vertices, faces, triangulate, false);
    }

    std::vector<int> triangles;
    triangles.reserve(faces.size() * 3);
    for (const auto& face : faces) {
        if (face.size() != 3) {
            throw std::runtime_error("extract_surface_mesh expected triangle faces; pass triangulate=true for cubic meshes");
        }
        triangles.insert(triangles.end(), face.begin(), face.end());
    }
    return PyTriMeshData(Mesh::MeshData<3>::fromFlatElements(std::move(vertices), std::move(triangles)));
}

std::shared_ptr<PySimulationMesh> create_volume_simulation_mesh(
    const PyVolumeMesh& vm)
{
    const auto* volume = vm.getVM();
    std::shared_ptr<SolidDeformationModel::SimulationMesh> mesh;
    {
        nb::gil_scoped_release release;
        if (auto* tetMesh = dynamic_cast<const VolumetricMeshes::TetMesh*>(volume)) {
            mesh = SolidDeformationModel::loadTetMesh(*tetMesh);
        }
        else if (auto* cubicMesh = dynamic_cast<const VolumetricMeshes::CubicMesh*>(volume)) {
            mesh = SolidDeformationModel::loadCubicMesh(*cubicMesh);
        }
        else {
            throw std::runtime_error(
                "Unsupported volume mesh type for simulation");
        }
    }
    return std::make_shared<PySimulationMesh>(std::move(mesh));
}

std::shared_ptr<PySimulationMesh> create_shell_simulation_mesh(
    const PyTriMeshData& surfaceData)
{
    Mesh::TriMeshGeo surface(surfaceData.core());
    std::shared_ptr<SolidDeformationModel::SimulationMesh> mesh;
    {
        nb::gil_scoped_release release;
        mesh = SolidDeformationModel::loadShellMesh(surface);
    }
    return std::make_shared<PySimulationMesh>(std::move(mesh));
}

PyVegENuMaterialPayload create_enu_material_payload(
    const std::string& name, double density, double E, double nu)
{
    return { name, density, E, nu };
}

PyVegMooneyRivlinMaterialPayload create_mooney_rivlin_material_payload(
    const std::string& name, double density, double mu01, double mu10, double v1)
{
    return { name, density, mu01, mu10, v1 };
}

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
    const std::vector<double>& R)
{
    if (R.size() != 9) {
        throw std::runtime_error("Orthotropic material R must contain 9 row-major values");
    }
    return { name, density, E1, E2, E3, nu12, nu23, nu31, G12, G23, G31, R };
}

nb::object vegPayloadMeshData(const PyVegPayload& self) { return meshDataFromVegPayload(self.meshData); }
nb::list vegPayloadMaterials(const PyVegPayload& self) {
    nb::list materials;
    for (const auto& material : self.materials) {
        materials.append(materialPayloadFromVegPayload(material));
    }
    return materials;
}
std::vector<std::pair<std::string, std::vector<int>>> vegPayloadSets(const PyVegPayload& self) { return self.sets; }
std::vector<std::pair<int, int>> vegPayloadRegions(const PyVegPayload& self) { return self.regions; }

}  // namespace pgo
