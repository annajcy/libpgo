#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/shared_ptr.h>
#include <algorithm>
#include <memory>
#include <set>
#include <stdexcept>
#include <vector>

#include "meshData.h"
#include "triMeshGeo.h"
#include "volumetricMesh.h"
#include "tetMesh.h"
#include "cubicMesh.h"
#include "generateSurfaceMesh.h"
#include "volumetricMeshENuMaterial.h"
#include "volumetricMeshMooneyRivlinMaterial.h"
#include "volumetricMeshOrthotropicMaterial.h"
#include "vegFile.h"
#include "common.h"
#include "barycentricCoordinates.h"
#include "generateMassMatrix.h"
#include "simulationMesh.h"
#include "formulations/formulation.h"
#include "formulations/formulationDynamics.h"
#include "sparse_matrix_core.h"
#include "simulation_mesh_core.h"
#include "vegFile.h"

namespace nb = nanobind;
using namespace pgo;

namespace
{
using VM = VolumetricMeshes::VolumetricMesh;

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
    nb::object meshData;
    nb::list materials;
    std::vector<std::pair<std::string, std::vector<int>>> sets;
    std::vector<std::pair<int, int>> regions;
};

std::vector<double> flattenPositions(const std::vector<Vec3d>& positions) {
    std::vector<double> flat(positions.size() * 3);
    for (size_t i = 0; i < positions.size(); ++i) {
        flat[i * 3 + 0] = positions[i][0];
        flat[i * 3 + 1] = positions[i][1];
        flat[i * 3 + 2] = positions[i][2];
    }
    return flat;
}

template<int K>
std::pair<std::vector<double>, std::vector<int>> flattenMeshData(const Mesh::MeshData<K>& data) {
    return { flattenPositions(data.positions()), data.elementsFlat() };
}

std::unique_ptr<VM::Material> makeMaterial(const nb::object& obj) {
    if (nb::isinstance<PyVegENuMaterialPayload>(obj)) {
        auto p = nb::cast<PyVegENuMaterialPayload>(obj);
        return std::make_unique<VM::ENuMaterial>(p.name, p.density, p.E, p.nu);
    }
    if (nb::isinstance<PyVegMooneyRivlinMaterialPayload>(obj)) {
        auto p = nb::cast<PyVegMooneyRivlinMaterialPayload>(obj);
        return std::make_unique<VM::MooneyRivlinMaterial>(p.name, p.density, p.mu01, p.mu10, p.v1);
    }
    if (nb::isinstance<PyVegOrthotropicMaterialPayload>(obj)) {
        auto p = nb::cast<PyVegOrthotropicMaterialPayload>(obj);
        if (p.R.size() != 9) {
            throw std::runtime_error("Orthotropic material R must contain 9 row-major values");
        }
        return std::make_unique<VM::OrthotropicMaterial>(
            p.name, p.density,
            p.E1, p.E2, p.E3,
            p.nu12, p.nu23, p.nu31,
            p.G12, p.G23, p.G31,
            p.R.data());
    }
    throw std::runtime_error("unsupported material payload type");
}

std::vector<VM::Set> makeSets(const std::vector<std::pair<std::string, std::vector<int>>>& payloads) {
    std::vector<VM::Set> sets;
    sets.reserve(payloads.size());
    for (const auto& [name, elements] : payloads) {
        sets.emplace_back(name, std::set<int>(elements.begin(), elements.end()));
    }
    return sets;
}

std::vector<VM::Region> makeRegions(const std::vector<std::pair<int, int>>& payloads) {
    std::vector<VM::Region> regions;
    regions.reserve(payloads.size());
    for (const auto& [materialIndex, setIndex] : payloads) {
        regions.emplace_back(materialIndex, setIndex);
    }
    return regions;
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

}  // namespace

class PyVolumeMesh {
public:
    enum class MeshType { Tet, Cubic };

    PyVolumeMesh(std::unique_ptr<VolumetricMeshes::TetMesh> tetMesh)
        : type_(MeshType::Tet), tetMesh_(std::move(tetMesh)) {}

    PyVolumeMesh(std::unique_ptr<VolumetricMeshes::CubicMesh> cubicMesh)
        : type_(MeshType::Cubic), cubicMesh_(std::move(cubicMesh)) {}

    MeshType meshType() const { return type_; }

    int numVertices() const {
        return getVM()->getNumVertices();
    }

    int numElements() const {
        return getVM()->getNumElements();
    }

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

PySparseMatrix compute_mass_matrix(const PyVolumeMesh& volumeMesh, bool inflate3Dim)
{
    pgo::EigenSupport::SpMatD M;
    {
        nb::gil_scoped_release release;
        VolumetricMeshes::GenerateMassMatrix::computeMassMatrix(volumeMesh.getVM(), M, inflate3Dim);
    }
    return PySparseMatrix(std::move(M));
}

std::unique_ptr<SolidDeformationModel::Formulation> make_volume_formulation(
    const std::string& formulationName)
{
    if (formulationName == "tet_p1") {
        return std::make_unique<SolidDeformationModel::P1TetFormulation>();
    }
    if (formulationName == "hex_trilinear") {
        return std::make_unique<SolidDeformationModel::LinearCubicFormulation>();
    }
    if (formulationName == "hex_tricubic_hermite") {
        return std::make_unique<SolidDeformationModel::TricubicHermiteFormulation>();
    }
    throw std::invalid_argument(
        "Unknown volume formulation '" + formulationName +
        "'. Expected 'tet_p1', 'hex_trilinear', or 'hex_tricubic_hermite'.");
}

SolidDeformationModel::HermiteBoundaryPolicy parse_hermite_boundary_policy(
    const std::string& policy)
{
    if (policy == "value") {
        return SolidDeformationModel::HermiteBoundaryPolicy::Value;
    }
    if (policy == "first") {
        return SolidDeformationModel::HermiteBoundaryPolicy::First;
    }
    if (policy == "all") {
        return SolidDeformationModel::HermiteBoundaryPolicy::All;
    }
    throw std::invalid_argument("Hermite boundary policy must be 'value', 'first', or 'all'");
}

PySparseMatrix compute_formulation_mass_matrix(
    const PyVolumeMesh& volumeMesh,
    const std::string& formulationName)
{
    auto formulation = make_volume_formulation(formulationName);
    pgo::EigenSupport::SpMatD M;
    {
        nb::gil_scoped_release release;
        M = SolidDeformationModel::buildFormulationMassMatrix(
            *volumeMesh.getVM(), *formulation);
    }
    return PySparseMatrix(std::move(M));
}

std::vector<double> compute_formulation_body_force(
    const PyVolumeMesh& volumeMesh,
    const std::string& formulationName,
    const std::vector<double>& acceleration)
{
    if (acceleration.size() != 3) {
        throw std::invalid_argument("acceleration must contain exactly 3 values");
    }
    auto formulation = make_volume_formulation(formulationName);
    pgo::EigenSupport::V3d a(acceleration[0], acceleration[1], acceleration[2]);
    pgo::EigenSupport::VXd f;
    {
        nb::gil_scoped_release release;
        f = SolidDeformationModel::buildFormulationBodyForce(
            *volumeMesh.getVM(), *formulation, a);
    }
    return std::vector<double>(f.data(), f.data() + f.size());
}

PySparseMatrix compute_formulation_surface_embedding_matrix(
    const PyVolumeMesh& volumeMesh,
    const std::string& formulationName,
    const std::vector<double>& surfaceVerticesFlat)
{
    if (surfaceVerticesFlat.size() % 3 != 0) {
        throw std::invalid_argument("surface vertices must be a flat 3*m vector");
    }
    const int numVertices = static_cast<int>(surfaceVerticesFlat.size() / 3);
    pgo::EigenSupport::MXd surfaceVertices(numVertices, 3);
    for (int i = 0; i < numVertices; i++) {
        surfaceVertices(i, 0) = surfaceVerticesFlat[static_cast<size_t>(i) * 3 + 0];
        surfaceVertices(i, 1) = surfaceVerticesFlat[static_cast<size_t>(i) * 3 + 1];
        surfaceVertices(i, 2) = surfaceVerticesFlat[static_cast<size_t>(i) * 3 + 2];
    }

    auto formulation = make_volume_formulation(formulationName);
    pgo::EigenSupport::SpMatD W;
    {
        nb::gil_scoped_release release;
        W = SolidDeformationModel::buildFormulationSurfaceEmbeddingMatrix(
            *volumeMesh.getVM(), *formulation, surfaceVertices);
    }
    return PySparseMatrix(std::move(W));
}

std::vector<int> hermite_vertex_dofs(
    const std::vector<int>& vertexIds,
    const std::string& policy)
{
    return SolidDeformationModel::hermiteVertexDofs(
        vertexIds, parse_hermite_boundary_policy(policy));
}

std::vector<int> hermite_face_dofs(
    const PyVolumeMesh& volumeMesh,
    int axis,
    bool maxSide,
    const std::string& policy)
{
    return SolidDeformationModel::hermiteFaceDofs(
        *volumeMesh.getVM(), axis, maxSide, parse_hermite_boundary_policy(policy));
}

class PyBarycentricEmbedding {
public:
    PyBarycentricEmbedding(const std::vector<double>& targetLocationsFlat, const PyVolumeMesh& volumeMesh)
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

    PySparseMatrix interpolationMatrix() const
    {
        pgo::EigenSupport::SpMatD matrix;
        {
            nb::gil_scoped_release release;
            matrix = coords_->generateInterpolationMatrix();
        }
        return PySparseMatrix(std::move(matrix));
    }

    std::tuple<std::vector<int>, std::vector<int>, std::vector<double>> interpolationMatrixCOO() const
    {
        return interpolationMatrix().toCOO();
    }

    int numElementVertices() const { return coords_->getNumElementVertices(); }

    std::vector<int> embeddingIndicesFlat() const
    {
        const auto& indices = coords_->getEmbeddingVertexIndices();
        return std::vector<int>(indices.begin(), indices.end());
    }

    std::vector<double> embeddingWeightsFlat() const
    {
        const auto& weights = coords_->getEmbeddingWeights();
        return std::vector<double>(weights.begin(), weights.end());
    }

    std::vector<int> embeddingElements() const
    {
        const auto& elements = coords_->getElements();
        return std::vector<int>(elements.begin(), elements.end());
    }

    std::vector<double> deform(const std::vector<double>& volumeDispFlat) const
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

    int numTargetLocations() const { return numTargetLocations_; }
    int numVolumeVertices() const { return numVolumeVertices_; }

private:
    std::unique_ptr<InterpolationCoordinates::BarycentricCoordinates> coords_;
    int numTargetLocations_ = 0;
    int numVolumeVertices_ = 0;
};

// --- create from MeshData + MaterialSpec ---

std::shared_ptr<PyVolumeMesh> create_volume_mesh(const nb::object& meshDataObj, const PyMaterialSpec& mat) {
    if (nb::isinstance<Mesh::MeshData<4>>(meshDataObj)) {
        const auto& meshData = nb::cast<const Mesh::MeshData<4>&>(meshDataObj);
        std::unique_ptr<VolumetricMeshes::TetMesh> tetMesh;
        {
            nb::gil_scoped_release release;
            tetMesh = std::make_unique<VolumetricMeshes::TetMesh>(meshData, mat.E(), mat.nu(), mat.density());
        }
        return std::make_shared<PyVolumeMesh>(std::move(tetMesh));
    } else if (nb::isinstance<Mesh::MeshData<8>>(meshDataObj)) {
        const auto& meshData = nb::cast<const Mesh::MeshData<8>&>(meshDataObj);
        std::unique_ptr<VolumetricMeshes::CubicMesh> cubicMesh;
        {
            nb::gil_scoped_release release;
            cubicMesh = std::make_unique<VolumetricMeshes::CubicMesh>(meshData, mat.E(), mat.nu(), mat.density());
        }
        return std::make_shared<PyVolumeMesh>(std::move(cubicMesh));
    } else {
        throw std::runtime_error("Unsupported element mesh type for create_volume_mesh");
    }
}

// --- load directly from .veg file (zero redundant construction) ---

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

// --- save to .veg file ---

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

// --- lazy export: geometry ---

nb::object export_geometry(const PyVolumeMesh& vm) {
    const auto* vMesh = vm.getVM();
    std::vector<Vec3d> vertices;
    std::vector<int> elements;
    vMesh->exportMeshGeometry(vertices, elements);

    if (vm.meshType() == PyVolumeMesh::MeshType::Tet) {
        auto meshData = Mesh::MeshData<4>::fromFlatElements(std::move(vertices), std::move(elements));
        return nb::cast(std::move(meshData));
    } else {
        auto meshData = Mesh::MeshData<8>::fromFlatElements(std::move(vertices), std::move(elements));
        return nb::cast(std::move(meshData));
    }
}

// --- lazy export: material ---

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

PyVegPayload extract_veg_payload_from_volume_mesh(const PyVolumeMesh& vm) {
    VolumetricMeshes::VegFilePayload payload;
    {
        nb::gil_scoped_release release;
        payload = vm.getVM()->toVegFilePayload();
    }

    PyVegPayload result;
    result.meshData = std::visit([](auto&& meshData) {
        return nb::cast(std::forward<decltype(meshData)>(meshData));
    }, std::move(payload.meshData));

    for (const auto& material : payload.materials) {
        result.materials.append(materialPayloadFromVegPayload(material));
    }
    for (const auto& set : payload.sets) {
        result.sets.emplace_back(set.name, set.elements);
    }
    for (const auto& region : payload.regions) {
        result.regions.emplace_back(region.materialIndex, region.setIndex);
    }
    return result;
}

std::shared_ptr<PyVolumeMesh> create_volume_mesh_multi(
    const nb::object& meshDataObj,
    const std::vector<nb::object>& materialPayloads,
    const std::vector<std::pair<std::string, std::vector<int>>>& setPayloads,
    const std::vector<std::pair<int, int>>& regionPayloads)
{
    std::vector<std::unique_ptr<VM::Material>> materials;
    materials.reserve(materialPayloads.size());
    std::vector<const VM::Material*> materialPtrs;
    materialPtrs.reserve(materialPayloads.size());
    for (const auto& payload : materialPayloads) {
        materials.push_back(makeMaterial(payload));
        materialPtrs.push_back(materials.back().get());
    }

    auto sets = makeSets(setPayloads);
    auto regions = makeRegions(regionPayloads);

    if (nb::isinstance<Mesh::MeshData<4>>(meshDataObj)) {
        const auto& data = nb::cast<const Mesh::MeshData<4>&>(meshDataObj);
        auto [vertices, elements] = flattenMeshData(data);
        std::unique_ptr<VolumetricMeshes::TetMesh> tetMesh;
        {
            nb::gil_scoped_release release;
            tetMesh = std::make_unique<VolumetricMeshes::TetMesh>(
                static_cast<int>(data.numVertices()), vertices.data(),
                static_cast<int>(data.numElements()), elements.data(),
                static_cast<int>(materials.size()), materialPtrs.data(),
                static_cast<int>(sets.size()), sets.data(),
                static_cast<int>(regions.size()), regions.data());
        }
        return std::make_shared<PyVolumeMesh>(std::move(tetMesh));
    }

    if (nb::isinstance<Mesh::MeshData<8>>(meshDataObj)) {
        const auto& data = nb::cast<const Mesh::MeshData<8>&>(meshDataObj);
        auto [vertices, elements] = flattenMeshData(data);
        std::unique_ptr<VolumetricMeshes::CubicMesh> cubicMesh;
        {
            nb::gil_scoped_release release;
            cubicMesh = std::make_unique<VolumetricMeshes::CubicMesh>(
                static_cast<int>(data.numVertices()), vertices.data(),
                static_cast<int>(data.numElements()), elements.data(),
                static_cast<int>(materials.size()), materialPtrs.data(),
                static_cast<int>(sets.size()), sets.data(),
                static_cast<int>(regions.size()), regions.data());
        }
        return std::make_shared<PyVolumeMesh>(std::move(cubicMesh));
    }

    throw std::runtime_error("Unsupported element mesh type for create_volume_mesh_multi");
}

PyVegPayload read_veg(const std::string& path) {
    VolumetricMeshes::VegFilePayload payload;
    {
        nb::gil_scoped_release release;
        payload = VolumetricMeshes::readVegFile(path);
    }

    PyVegPayload result;
    result.meshData = std::visit([](auto&& meshData) {
        return nb::cast(std::forward<decltype(meshData)>(meshData));
    }, std::move(payload.meshData));

    for (const auto& material : payload.materials) {
        result.materials.append(materialPayloadFromVegPayload(material));
    }
    for (const auto& set : payload.sets) {
        result.sets.emplace_back(set.name, set.elements);
    }
    for (const auto& region : payload.regions) {
        result.regions.emplace_back(region.materialIndex, region.setIndex);
    }
    return result;
}

void write_veg(
    const std::string& path,
    const nb::object& meshDataObj,
    const std::vector<nb::object>& materialPayloads,
    const std::vector<std::pair<std::string, std::vector<int>>>& setPayloads,
    const std::vector<std::pair<int, int>>& regionPayloads)
{
    VolumetricMeshes::VegFilePayload payload;
    if (nb::isinstance<Mesh::MeshData<4>>(meshDataObj)) {
        payload.meshData = nb::cast<Mesh::MeshData<4>>(meshDataObj);
    }
    else if (nb::isinstance<Mesh::MeshData<8>>(meshDataObj)) {
        payload.meshData = nb::cast<Mesh::MeshData<8>>(meshDataObj);
    }
    else {
        throw std::runtime_error("Unsupported element mesh type for write_veg");
    }

    payload.materials.reserve(materialPayloads.size());
    for (const auto& material : materialPayloads) {
        payload.materials.push_back(vegPayloadFromMaterialObject(material));
    }
    payload.sets.reserve(setPayloads.size());
    for (const auto& [name, elements] : setPayloads) {
        payload.sets.push_back(VolumetricMeshes::VegSetPayload{ name, elements });
    }
    payload.regions.reserve(regionPayloads.size());
    for (const auto& [materialIndex, setIndex] : regionPayloads) {
        payload.regions.push_back(VolumetricMeshes::VegRegionPayload{ materialIndex, setIndex });
    }

    {
        nb::gil_scoped_release release;
        VolumetricMeshes::writeVegFile(path, payload);
    }
}

Mesh::MeshData<3> extract_surface_mesh(const PyVolumeMesh& vm, bool triangulate)
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
    return Mesh::MeshData<3>::fromFlatElements(std::move(vertices), std::move(triangles));
}

std::shared_ptr<PySimulationMesh> create_simulation_mesh_from_volume(const PyVolumeMesh& vm)
{
    const auto* volume = vm.getVM();
    for (int element = 0; element < volume->getNumElements(); ++element) {
        if (VolumetricMeshes::downcastENuMaterial(volume->getElementMaterial(element)) == nullptr) {
            throw std::runtime_error(
                "SimulationMesh.create_volumetric currently supports only ENuMaterial; element " +
                std::to_string(element) + " uses a non-ENu volume material");
        }
    }

    std::unique_ptr<SolidDeformationModel::SimulationMesh> simMesh;
    {
        nb::gil_scoped_release release;
        if (auto* tetMesh = dynamic_cast<const VolumetricMeshes::TetMesh*>(volume)) {
            simMesh = SolidDeformationModel::loadTetMesh(tetMesh);
        }
        else if (auto* cubicMesh = dynamic_cast<const VolumetricMeshes::CubicMesh*>(volume)) {
            simMesh = SolidDeformationModel::loadCubicMesh(cubicMesh);
        }
        else {
            throw std::runtime_error("Unsupported volume mesh type for SimulationMesh.create_volumetric");
        }
    }
    return std::make_shared<PySimulationMesh>(std::move(simMesh));
}

std::shared_ptr<PySimulationMesh> create_simulation_mesh_from_shell(
    const Mesh::MeshData<3>& surfaceData,
    double thickness,
    double E,
    double nu)
{
    Mesh::TriMeshGeo surface(surfaceData);
    SolidDeformationModel::SimulationMeshENuhMaterial material(E, nu, thickness);
    std::unique_ptr<SolidDeformationModel::SimulationMesh> simMesh;
    {
        nb::gil_scoped_release release;
        simMesh = SolidDeformationModel::loadShellMesh(surface, &material);
    }
    return std::make_shared<PySimulationMesh>(std::move(simMesh));
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

void init_mesh_bindings(nb::module_ &m) {
    nb::enum_<PyVolumeMesh::MeshType>(m, "MeshType")
        .value("Tet", PyVolumeMesh::MeshType::Tet)
        .value("Cubic", PyVolumeMesh::MeshType::Cubic);

    nb::class_<PyVolumeMesh>(m, "PyVolumeMesh")
        .def("mesh_type", &PyVolumeMesh::meshType)
        .def("num_vertices", &PyVolumeMesh::numVertices)
        .def("num_elements", &PyVolumeMesh::numElements)
        .def("export_geometry", [](const PyVolumeMesh& self) {
            return export_geometry(self);
        })
        .def("export_material", [](const PyVolumeMesh& self) {
            return export_material(self);
        })
        .def("export_material_payload", [](const PyVolumeMesh& self) {
            return export_material_payload(self);
        });

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
        .def_prop_ro("mesh_data", [](const PyVegPayload& self) { return self.meshData; })
        .def_prop_ro("materials", [](const PyVegPayload& self) { return self.materials; })
        .def_prop_ro("sets", [](const PyVegPayload& self) { return self.sets; })
        .def_prop_ro("regions", [](const PyVegPayload& self) { return self.regions; });

    m.def("create_enu_material_payload", &create_enu_material_payload);
    m.def("create_mooney_rivlin_material_payload", &create_mooney_rivlin_material_payload);
    m.def("create_orthotropic_material_payload", &create_orthotropic_material_payload);
    m.def("create_volume_mesh", &create_volume_mesh);
    m.def("create_volume_mesh_multi", &create_volume_mesh_multi);
    m.def("load_volume_mesh", &load_volume_mesh);
    m.def("save_volume_mesh", &save_volume_mesh);
    m.def("read_veg", &read_veg);
    m.def("write_veg", &write_veg);
    m.def("extract_surface_mesh", &extract_surface_mesh, nb::arg("volume_mesh"), nb::arg("triangulate") = true);
    m.def("extract_veg_payload_from_volume_mesh", &extract_veg_payload_from_volume_mesh, nb::arg("volume_mesh"));
    m.def("create_simulation_mesh_from_volume", &create_simulation_mesh_from_volume);
    m.def("create_simulation_mesh_from_shell", &create_simulation_mesh_from_shell,
        nb::arg("surface_data"), nb::arg("thickness"), nb::arg("E"), nb::arg("nu"));
    m.def("compute_mass_matrix", &compute_mass_matrix,
        nb::arg("volume_mesh"), nb::arg("inflate3dim") = true);
    m.def("compute_formulation_mass_matrix", &compute_formulation_mass_matrix,
        nb::arg("volume_mesh"), nb::arg("formulation"));
    m.def("compute_formulation_body_force", &compute_formulation_body_force,
        nb::arg("volume_mesh"), nb::arg("formulation"), nb::arg("acceleration"));
    m.def("compute_formulation_surface_embedding_matrix", &compute_formulation_surface_embedding_matrix,
        nb::arg("volume_mesh"), nb::arg("formulation"), nb::arg("surface_vertices_flat"));
    m.def("hermite_vertex_dofs", &hermite_vertex_dofs,
        nb::arg("vertex_ids"), nb::arg("policy") = "value");
    m.def("hermite_face_dofs", &hermite_face_dofs,
        nb::arg("volume_mesh"), nb::arg("axis"), nb::arg("max_side"), nb::arg("policy") = "all");
}
