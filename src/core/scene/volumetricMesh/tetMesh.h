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
  This class is a container for a tetrahedral volumetric 3D mesh. See
  also volumetricMesh.h. The tetrahedra can take arbitrary shapes (not
  limited to only a few shapes).
*/

#pragma once

#include "boundingBox.h"
#include "meshLinearAlgebra.h"
#include "materials/material_access.h"
#include "ops/mesh_common_ops.h"
#include "storage/mesh_storage.h"
#include "types/element_set.h"
#include "types/material_region.h"
#include "types/mesh_constants.h"
#include "types/mesh_enums.h"

#include <filesystem>
#include <memory>
#include <set>
#include <span>
#include <vector>

namespace pgo {
namespace Mesh {
class TetMeshGeo;
}

namespace VolumetricMeshes {
namespace io::detail {
struct LoadedMeshData;
}

class TetMesh {
public:
    // loads the mesh from a file
    // ASCII: .veg text input format, see documentation and the provided examples
    // BINARY: .vegb binary input format
    TetMesh(const std::filesystem::path& filename,
            FileFormatType               fileFormat = FileFormatType::ByExtension,
            int                          verbose    = 1);

    TetMesh(std::span<const std::byte> binaryStream);

    // constructs a tet mesh with only four vertices and one tet
    TetMesh(const Vec3d& p0, const Vec3d& p1, const Vec3d& p2, const Vec3d& p3);

    // constructs a tet mesh from the given vertices and elements,
    // with a single region and material ("E, nu" material)
    // "vertices" is double-precision array of length 3 x numVertices .
    // "elements" is an integer array of length 4 x numElements
    TetMesh(std::span<const Vec3d> vertices, std::span<const Vec4i> elements, double E = E_default,
            double nu = nu_default, double density = density_default);

    // constructs a tet mesh from the given vertices and elements,
    // with an arbitrary number of sets, regions and materials
    // "vertices" is double-precision array of length 3 x numVertices
    // "elements" is an integer array of length 4 x numElements
    // "materials", "sets" and "regions" will be copied internally (deep copy), so you
    // can release them after calling this constructor
    TetMesh(std::span<const Vec3d> vertices, std::span<const Vec4i> elements,
            std::vector<MaterialRecord> materials, std::vector<ElementSet> sets,
            std::vector<MaterialRegion> regions);

    // loads a file of a "special" (not .veg) type
    // currently one such special format is supported:
    // specialFileType=0:
    //   the ".ele" and ".node" format, used by TetGen,
    //   "filename" is the basename, e.g., passing "mesh" will load the mesh from "mesh.ele" and "mesh.node"
    // specialFileType=1:
    //   the ".msh" format, used by Gmsh
    // default material parameters will be used
    TetMesh(const std::filesystem::path& filename, int specialFileType, int verbose);

    // creates a mesh consisting of the specified element subset of the given TetMesh
    TetMesh(const TetMesh& mesh, std::span<const int> elements, std::map<int, int>* vertexMap = nullptr);

    TetMesh(const TetMesh& tetMesh);
    std::unique_ptr<TetMesh> clone() const;
    ~TetMesh();

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

    static ElementType  elementType() { return ElementType::Tet; }
    ElementType getElementType() const { return elementType(); }

    static double getSignedTetVolume(const Vec3d& a, const Vec3d& b, const Vec3d& c, const Vec3d& d);
    static double getTetVolume(const Vec3d& a, const Vec3d& b, const Vec3d& c, const Vec3d& d);
    static double getTetDeterminant(const Vec3d& a, const Vec3d& b, const Vec3d& c, const Vec3d& d);
    inline double getTetDeterminant(int el) const {
        return getTetDeterminant(getVertex(el, 0), getVertex(el, 1), getVertex(el, 2), getVertex(el, 3));
    }

    Vec3d getElementCenter(int el) const { return ops::common::element_center(*this, el); }
    double getVolume() const { return ops::common::volume(*this); }
    double getElementVolume(int el) const;
    void getVertexVolumes(double* vertex_volumes) const { ops::common::vertex_volumes(*this, vertex_volumes); }
    void getElementInertiaTensor(int el, Mat3d& inertia_tensor) const;
    void computeElementMassMatrix(int element, double* mass_matrix) const;
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

    bool containsVertex(int element, Vec3d pos) const;  // true if given element contain given position, false otherwise
    int getClosestElement(const Vec3d& pos) const;
    int getContainingElement(Vec3d pos) const { return ops::common::containing_element(*this, pos); }
    void computeGravity(double* gravity_force, double g = 9.81, bool add_force = false) const {
        ops::common::compute_gravity(*this, gravity_force, g, add_force);
    }
    void applyDeformation(const double* u) { ops::common::apply_deformation(*this, u); }
    void applyLinearTransformation(double* pos, double* R) { ops::common::apply_linear_transformation(*this, pos, R); }
    void renumberVertices(const std::vector<int>& permutation) { ops::common::renumber_vertices(*this, permutation); }

    // edge queries
    int getNumElementEdges() const;
    void getElementEdges(int el, int* edge_buffer) const;

    // === interpolation ===

    static void  computeBarycentricWeights(const Vec3d tetVertexPos[4], const Vec3d& pos, double weights[4]);
    static void  computeBarycentricWeights(const Vec3d& tetVtxPos0, const Vec3d& tetVtxPos1, const Vec3d& tetVtxPos2,
                                           const Vec3d& tetVtxPos3, const Vec3d& pos, double weights[4]);
    void computeBarycentricWeights(int el, const Vec3d& pos, double* weights) const;

    // note here the gradient is col-major
    void computeGradient(int element, const double* U, int numFields, double* grad)
        const;  // for tet meshes, gradient is constant inside each tet, hence no need to specify position
    int interpolateGradient(const double* U, int numFields, Vec3d pos, double* grad) const {
        return ops::common::interpolate_gradient(*this, U, numFields, pos, grad);
    }
    void interpolateGradient(int element, const double* U, int numFields, Vec3d pos, double* grad)
        const;  // conforms to the old virtual function shape, "pos" does not affect the computation

    // === misc ===

    void orient();  // orients the tets (re-orders vertices within each tet), so that each tet has positive orientation:
                    // ((v1 - v0) x (v2 - v0)) dot (v3 - v0) >= 0

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
    explicit TetMesh(int numElementVertices);
    void assignFromData(io::detail::LoadedMeshData data, int verbose = 0);

    void set_storage(storage::MeshStorage storage);

    storage::MeshStorage m_storage;
};
}  // namespace VolumetricMeshes
}  // namespace pgo
