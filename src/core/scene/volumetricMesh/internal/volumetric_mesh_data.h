#pragma once

#include "meshLinearAlgebra.h"

#include <span>
#include <vector>

namespace pgo::VolumetricMeshes::internal {

class VolumetricMeshData {
public:
    VolumetricMeshData() = default;
    VolumetricMeshData(int num_element_vertices, std::vector<Vec3d> vertices, std::vector<int> elements);

    int num_vertices() const;
    int num_elements() const;
    int num_element_vertices() const;

    std::span<Vec3d>       vertices();
    std::span<const Vec3d> vertices() const;
    std::span<int>         elements();
    std::span<const int>   elements() const;

    Vec3d&       vertex(int index);
    const Vec3d& vertex(int index) const;
    void         set_vertex(int index, const Vec3d& position);

    int                     vertex_index(int element, int vertex) const;
    std::span<int>          vertex_indices(int element);
    std::span<const int>    vertex_indices(int element) const;

    void assign(std::vector<Vec3d> vertices, std::vector<int> elements);
    void resize_vertices(int num_vertices);
    void resize_elements_for_num_elements(int num_elements);
    void renumber_vertices(std::span<const int> permutation);
    void validate_basic_invariants() const;

private:
    int                m_num_element_vertices = 0;
    std::vector<Vec3d> m_vertices;
    std::vector<int>   m_elements;
};

}  // namespace pgo::VolumetricMeshes::internal
