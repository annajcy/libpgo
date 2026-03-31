#include "internal/volumetric_mesh_data.h"

#include <stdexcept>
#include <utility>

namespace pgo::VolumetricMeshes::internal {

VolumetricMeshData::VolumetricMeshData(int num_element_vertices, std::vector<Vec3d> vertices, std::vector<int> elements)
    : m_num_element_vertices(num_element_vertices),
      m_vertices(std::move(vertices)),
      m_elements(std::move(elements)) {
    validate_basic_invariants();
}

int VolumetricMeshData::num_vertices() const {
    return static_cast<int>(m_vertices.size());
}

int VolumetricMeshData::num_elements() const {
    if (m_num_element_vertices == 0) {
        return 0;
    }
    return static_cast<int>(m_elements.size()) / m_num_element_vertices;
}

int VolumetricMeshData::num_element_vertices() const {
    return m_num_element_vertices;
}

std::span<Vec3d> VolumetricMeshData::vertices() {
    return m_vertices;
}

std::span<const Vec3d> VolumetricMeshData::vertices() const {
    return m_vertices;
}

std::span<int> VolumetricMeshData::elements() {
    return m_elements;
}

std::span<const int> VolumetricMeshData::elements() const {
    return m_elements;
}

Vec3d& VolumetricMeshData::vertex(int index) {
    return m_vertices[static_cast<size_t>(index)];
}

const Vec3d& VolumetricMeshData::vertex(int index) const {
    return m_vertices[static_cast<size_t>(index)];
}

void VolumetricMeshData::set_vertex(int index, const Vec3d& position) {
    m_vertices[static_cast<size_t>(index)] = position;
}

int VolumetricMeshData::vertex_index(int element, int vertex) const {
    return m_elements[static_cast<size_t>(element * m_num_element_vertices + vertex)];
}

std::span<int> VolumetricMeshData::vertex_indices(int element) {
    return std::span<int>(m_elements.data() + element * m_num_element_vertices, m_num_element_vertices);
}

std::span<const int> VolumetricMeshData::vertex_indices(int element) const {
    return std::span<const int>(m_elements.data() + element * m_num_element_vertices, m_num_element_vertices);
}

void VolumetricMeshData::assign(std::vector<Vec3d> vertices, std::vector<int> elements) {
    m_vertices = std::move(vertices);
    m_elements = std::move(elements);
    validate_basic_invariants();
}

void VolumetricMeshData::resize_vertices(int num_vertices) {
    if (num_vertices < 0) {
        throw std::invalid_argument("num_vertices must be non-negative");
    }
    m_vertices.resize(static_cast<size_t>(num_vertices));
}

void VolumetricMeshData::resize_elements_for_num_elements(int num_elements) {
    if (num_elements < 0) {
        throw std::invalid_argument("num_elements must be non-negative");
    }
    m_elements.resize(static_cast<size_t>(num_elements * m_num_element_vertices));
}

void VolumetricMeshData::renumber_vertices(std::span<const int> permutation) {
    std::vector<Vec3d> new_vertices(m_vertices.size());
    for (int index = 0; index < num_vertices(); ++index) {
        new_vertices[static_cast<size_t>(permutation[static_cast<size_t>(index)])] =
            m_vertices[static_cast<size_t>(index)];
    }
    m_vertices = std::move(new_vertices);

    for (int& vertex_id : m_elements) {
        vertex_id = permutation[static_cast<size_t>(vertex_id)];
    }
}

void VolumetricMeshData::validate_basic_invariants() const {
    if (m_num_element_vertices < 0) {
        throw std::invalid_argument("num_element_vertices must be non-negative");
    }
    if (m_num_element_vertices == 0 && !m_elements.empty()) {
        throw std::invalid_argument("elements require a positive num_element_vertices");
    }
    if (m_num_element_vertices > 0 && m_elements.size() % static_cast<size_t>(m_num_element_vertices) != 0) {
        throw std::invalid_argument("elements size must be divisible by num_element_vertices");
    }
    for (int vertex_id : m_elements) {
        if (vertex_id < 0 || vertex_id >= num_vertices()) {
            throw std::out_of_range("element vertex index out of range");
        }
    }
}

}  // namespace pgo::VolumetricMeshes::internal
