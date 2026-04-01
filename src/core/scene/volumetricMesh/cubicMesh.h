/*************************************************************************
 *                                                                       *
 * Vega FEM Simulation Library Version 4.0                               *
 *                                                                       *
 * "volumetricMesh" library , Copyright (C) 2007 CMU, 2009 MIT, 2018 USC *
 * All rights reserved.                                                  *
 *                                                                       *
 * Code author: Jernej Barbic                                            *
 * http://www.jernejbarbic.com/vega                                      *
 *                                                                       *
 * Research: Jernej Barbic, Hongyi Xu, Yijing Li,                        *
 *           Danyong Zhao, Bohan Wang,                                   *
 *           Fun Shing Sin, Daniel Schroeder,                            *
 *           Doug L. James, Jovan Popovic                                *
 *                                                                       *
 * Funding: National Science Foundation, Link Foundation,                *
 *          Singapore-MIT GAMBIT Game Lab,                               *
 *          Zumberge Research and Innovation Fund at USC,                *
 *          Sloan Foundation, Okawa Foundation,                          *
 *          USC Annenberg Foundation                                     *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of the BSD-style license that is            *
 * included with this library in the file LICENSE.txt                    *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the file     *
 * LICENSE.TXT for more details.                                         *
 *                                                                       *
 *************************************************************************/

/*

  This class is a container for a cubic ("voxel") volumetric 3D mesh.
  See also volumetricMesh.h.

  All cubes are of equal size, aligned with coordinate system axes, and
  follow a grid pattern. They are typically obtained by voxelizing a
  given input triangle geometry.

  To generate a CubicMesh from an input triangle mesh (optionally flood-filling
  interior chambers), you can use the "Large Modal Deformation Factory"
  application ( http://www.jernejbarbic.com/vega ) .

*/

// vtx order in CUBIC:
//
//     3 - - - 2
//    /|      /|
//   7 - - - 6 |       y
//   | |     | |       |
//   | 0 - - | 1       |_ _ _x
//   |/      |/       /
//   4 - - - 5       z

#pragma once

#include "boundingBox.h"
#include "meshLinearAlgebra.h"
#include "materials/material_access.h"
#include "ops/mesh_common_ops.h"
#include "storage/mesh_storage.h"
#include "types/element_types.h"
#include "types/element_set.h"
#include "types/material_region.h"
#include "types/mesh_constants.h"
#include "types/mesh_enums.h"

#include <filesystem>
#include <memory>
#include <set>
#include <span>
#include <vector>
// see also volumetricMesh.h for a description of the routines

namespace pgo {
namespace VolumetricMeshes {
namespace io::detail {
struct LoadedMeshData;
}
class CubicMesh;
namespace ops {
void compute_alpha_beta_gamma(const CubicMesh& mesh, int element, Vec3d pos, double* alpha, double* beta,
                              double* gamma);
}
namespace algorithms {
void subdivide_cubic_mesh(CubicMesh& mesh);
}

class CubicMesh {
public:
    // loads the mesh from a file
    // ASCII: .veg text input formut, see documentation and the provided examples
    // BINARY: .vegb binary input format
    CubicMesh(const std::filesystem::path& filename,
              FileFormatType               fileFormat = FileFormatType::ByExtension,
              int                          verbose    = 1);

    CubicMesh(std::span<const std::byte> binaryStream);

    // constructs a mesh from the given vertices and elements, with a single region and material
    // "vertices" is double-precision array of length 3 x numVertices
    // "elements" is an integer array of length 8 x numElements
    CubicMesh(std::span<const Vec3d> vertices, std::span<const CubicElement> elements, double E = E_default,
              double nu = nu_default, double density = density_default);

    // constructs a mesh from the given vertices and elements,
    // with an arbitrary number of sets, regions and materials
    // "vertices" is double-precision array of length 3 x numVertices
    // "elements" is an integer array of length 8 x numElements
    // "materials", "sets" and "regions" will be copied internally (deep copy), so they
    // can be released after calling this constructor
    CubicMesh(std::span<const Vec3d> vertices, std::span<const CubicElement> elements,
              std::vector<MaterialRecord> materials, std::vector<ElementSet> sets,
              std::vector<MaterialRegion> regions);

    // constructs a voxel mesh with the given voxels, as a subset of a regular 3D grid
    // 'voxels' gives the grid indices (3 per voxel) of the voxels that are to be included in the mesh
    // 'voxels' has length 3 x numVoxels
    static std::unique_ptr<CubicMesh> createFromUniformGrid(int resolution, std::span<const int> voxels,
                                                            double E = E_default, double nu = nu_default,
                                                            double density = density_default);

    // creates a mesh consisting of the specified element subset of the given CubicMesh
    CubicMesh(const CubicMesh& mesh, std::span<const int> elements, std::map<int, int>* vertexMap = nullptr);

    CubicMesh(const CubicMesh& CubicMesh);
    std::unique_ptr<CubicMesh> clone() const;
    ~CubicMesh();

    int getNumVertices() const { return m_storage.geometry().num_vertices(); }
    Vec3d& getVertex(int i) { return m_storage.geometry().vertex(i); }
    const Vec3d& getVertex(int i) const { return m_storage.geometry().vertex(i); }
    Vec3d& getVertex(int element, int vertex) { return m_storage.geometry().vertex(m_storage.geometry().vertex_index(element, vertex)); }
    const Vec3d& getVertex(int element, int vertex) const {
        return m_storage.geometry().vertex(m_storage.geometry().vertex_index(element, vertex));
    }
    void getElementVertices(int element, Vec3d* element_vertices) const {
        for (int i = 0; i < getNumElementVertices(); i++) {
            element_vertices[i] = getVertex(element, i);
        }
    }
    int getVertexIndex(int element, int vertex) const { return m_storage.geometry().vertex_index(element, vertex); }
    std::span<const int> getVertexIndices(int element) const { return m_storage.geometry().vertex_indices(element); }
    std::span<Vec3d> getVertices() { return m_storage.geometry().vertices(); }
    std::span<const Vec3d> getVertices() const { return m_storage.geometry().vertices(); }
    int getNumElements() const { return m_storage.geometry().num_elements(); }
    int getNumElementVertices() const { return m_storage.geometry().num_element_vertices(); }
    std::span<const int> getElements() const { return m_storage.geometry().elements(); }
    void setVertex(int i, const Vec3d& pos) {
        m_storage.geometry().set_vertex(i, pos);
        sync_storage_from_legacy_state_for_transition();
    }

    int getNumMaterials() const { return ops::common::num_materials(*this); }
    const MaterialRecord& getMaterial(int i) const { return ops::common::material(*this, i); }
    MaterialRecord& getMaterial(int i) { return ops::common::material(*this, i); }
    const MaterialRecord& getElementMaterial(int el) const { return ops::common::element_material(*this, el); }
    MaterialRecord& getElementMaterial(int el) { return ops::common::element_material(*this, el); }
    int getNumSets() const { return ops::common::num_sets(*this); }
    const ElementSet& getSet(int i) const { return ops::common::set(*this, i); }
    int getNumRegions() const { return ops::common::num_regions(*this); }
    const MaterialRegion& getRegion(int i) const { return ops::common::region(*this, i); }
    void setMaterial(int i, const MaterialRecord& material) { ops::common::set_material(*this, i, material); }
    void setSingleMaterial(double E, double nu, double density) { ops::common::set_single_material(*this, E, nu, density); }
    void addMaterial(const MaterialRecord& material, const ElementSet& new_set, bool remove_empty_sets,
                     bool remove_empty_materials) {
        ops::common::add_material(*this, material, new_set, remove_empty_sets, remove_empty_materials);
    }
    void assignMaterialsToElements(int verbose) { ops::common::assign_materials_to_elements(*this, verbose); }
    void propagateRegionsToElements() { ops::common::propagate_regions_to_elements(*this); }
    double getElementDensity(int el) const { return ops::common::element_density(*this, el); }

    // === misc queries ===

    static ElementType  elementType() { return ElementType::Cubic; }
    ElementType getElementType() const { return elementType(); }

    inline double getCubeSize() const { return cubeSize; }

    Vec3d getElementCenter(int el) const { return ops::common::element_center(*this, el); }
    double getVolume() const { return ops::common::volume(*this); }
    double getElementVolume(int el) const;
    void getVertexVolumes(double* vertex_volumes) const { ops::common::vertex_volumes(*this, vertex_volumes); }
    void computeElementMassMatrix(int el, double* mass_matrix) const;
    void getElementInertiaTensor(int el, Mat3d& inertia_tensor) const;
    double getMass() const { return ops::common::mass(*this); }
    void getInertiaParameters(double& mass, Vec3d& center_of_mass, Mat3d& inertia_tensor) const {
        ops::common::inertia_parameters(*this, mass, center_of_mass, inertia_tensor);
    }
    void getMeshGeometricParameters(Vec3d& centroid, double* radius) const {
        ops::common::mesh_geometric_parameters(*this, centroid, radius);
    }
    Mesh::BoundingBox getBoundingBox() const { return ops::common::bounding_box(*this); }
    void getVerticesInElements(const std::vector<int>& elements, std::vector<int>& vertices) const {
        ops::common::vertices_in_elements(*this, elements, vertices);
    }
    void getElementsTouchingVertices(const std::vector<int>& vertices, std::vector<int>& elements) const {
        ops::common::elements_touching_vertices(*this, vertices, elements);
    }
    void getElementsWithOnlyVertices(const std::vector<int>& vertices, std::vector<int>& elements) const {
        ops::common::elements_with_only_vertices(*this, vertices, elements);
    }
    void getVertexNeighborhood(const std::vector<int>& vertices, std::vector<int>& neighborhood) const {
        ops::common::vertex_neighborhood(*this, vertices, neighborhood);
    }
    int getClosestVertex(Vec3d pos) const { return ops::common::closest_vertex(*this, pos); }
    int getClosestElement(const Vec3d& pos) const { return internal::mesh_queries::get_closest_element(*this, pos); }
    int getContainingElement(Vec3d pos) const { return ops::common::containing_element(*this, pos); }
    void computeGravity(double* gravity_force, double g = 9.81, bool add_force = false) const {
        ops::common::compute_gravity(*this, gravity_force, g, add_force);
    }
    void applyDeformation(const double* u) { ops::common::apply_deformation(*this, u); }
    void applyLinearTransformation(double* pos, double* R) { ops::common::apply_linear_transformation(*this, pos, R); }
    void renumberVertices(const std::vector<int>& permutation) { ops::common::renumber_vertices(*this, permutation); }

    bool containsVertex(int element, Vec3d pos) const;  // true if given element contain given position, false otherwise

    // edge queries
    int getNumElementEdges() const;
    void getElementEdges(int el, int* edge_buffer) const;

    // subdivides the cube mesh
    void subdivide();

    // === interpolation ===

    void computeBarycentricWeights(int el, const Vec3d& pos, double* weights) const;

    int interpolateData(double* volumetricMeshVertexData, int numLocations, int r, double* interpolationLocations,
                        double* destMatrix, double zeroThreshold = -1.0) const;

    // computes approximation to the normal correction for the given deformations
    // vertexData size must a matrix of size 3 * nElements x r
    // destMatrix must be a vector of length 3 * numLocations x r
    // staticNormals must be a vector of length 3 * numLocations
    // returns the number of vertices that were not contained inside any element
    // vertices more than distanceThreshold away from any element vertex are assigned zero data
    int normalCorrection(double* vertexData, int numLocations, int r, double* interpolationLocations,
                         double* staticNormals, double* normalCorrection, double zeroThreshold = -1.0) const;

    int interpolateGradient(const double* U, int numFields, Vec3d pos, double* grad) const {
        return ops::common::interpolate_gradient(*this, U, numFields, pos, grad);
    }
    void interpolateGradient(int element, const double* U, int numFields, Vec3d pos, double* grad) const;

    // advanced, to ensure computeBarycentricWeights, containsVertex, generateInterpolationWeights,
    // generateContainingElements work even when elements are cubes, transformed via a general linear transformation
    // parallelepiped=1 : the elements are cubes transformed via a linear transformation (i.e., they are
    // parallelepipeds) parallelepiped=0 : (default) the elements are axis-aligned cubes
    void setParallelepipedMode(int parallelepipedMode);
    internal::VolumetricMeshData& geometry_data() { return m_storage.geometry(); }
    const internal::VolumetricMeshData& geometry_data() const { return m_storage.geometry(); }
    internal::MaterialCatalog& material_catalog() { return m_storage.material_catalog(); }
    const internal::MaterialCatalog& material_catalog() const { return m_storage.material_catalog(); }
    void sync_storage_from_legacy_state_for_transition();
    void reset_material_catalog(std::vector<MaterialRecord> materials, std::vector<ElementSet> sets,
                                std::vector<MaterialRegion> regions, int verbose) {
        ops::common::reset_material_catalog(*this, std::move(materials), std::move(sets), std::move(regions), verbose);
    }

private:
    double cubeSize;
    double invCubeSize;
    explicit CubicMesh(int numElementVertices);
    void assignFromData(io::detail::LoadedMeshData data, int verbose = 0);
    void SetInverseCubeSize();
    int  parallelepipedMode;  // normally this is 0; in advanced usage, it can be 1 (see above)

    // computes the normalized location of "pos" inside el
    // when inside the element, one has 0 <= alpha <= 1, 0 <= beta <= 1, 0 <= gamma <= 1
    void computeAlphaBetaGamma(int el, Vec3d pos, double* alpha, double* beta, double* gamma) const;

    friend void ops::compute_alpha_beta_gamma(const CubicMesh& mesh, int element, Vec3d pos, double* alpha,
                                              double* beta, double* gamma);
    friend void algorithms::subdivide_cubic_mesh(CubicMesh& mesh);
    void set_storage(storage::MeshStorage storage);

    storage::MeshStorage m_storage;
};

}  // namespace VolumetricMeshes
}  // namespace pgo
