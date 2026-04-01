/*************************************************************************
 *                                                                       *
 * Vega FEM Simulation Library Version 4.0                               *
 *                                                                       *
 * "volumetricMesh" library , Copyright (C) 2007 CMU, 2009 MIT, 2018 USC *
 * All rights reserved.                                                  *
 *                                                                       *
 * Code authors: Jernej Barbic, Yijing Li                                *
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

#pragma once

/*
  This abstract class can store a generic volumetric 3D mesh.
  It stores the mesh geometric information (elements and vertices),
  and also the material parameters of each individual mesh element
  (Young's modulus, Poisson ratio, mass density). This is done by
  organizing elements with the same material parameters into a "region".
  The class supports several geometric queries and interpolation to
  an embedded triangle mesh ("Free-Form Deformation").  It also
  supports exporting the mesh to an .ele or .node format (the format
  used by the Stellar and TetGen mesh generation packages).  Derived classes
  are the TetMesh (general tetrahedral meshes), and CubicMesh
  (axis-aligned cubic "voxel" meshes). See description in tetMesh.h and cubicMesh.h.

  All quantities are 0-indexed, except the input mesh files where the
  elements and vertices are 1-indexed (same as in TetGen and Stellar).
*/

#include "meshLinearAlgebra.h"
#include "boundingBox.h"
#include "internal/volumetric_mesh_data.h"
#include "materials/material_access.h"
#include "types/element_set.h"
#include "types/element_types.h"
#include "types/material_region.h"

#include <cstddef>
#include <cstdio>
#include <filesystem>
#include <istream>
#include <memory>
#include <ostream>
#include <span>
#include <vector>
#include <set>
#include <string>
#include <map>

namespace pgo {
namespace VolumetricMeshes {
class VolumetricMesh;
namespace internal {
class MaterialCatalog;
class MeshMutation;
}
namespace editing {
void subset_in_place(VolumetricMesh& mesh, const std::set<int>& subsetElements, int removeIsolatedVertices,
                     std::map<int, int>* old2NewVertexIDMap);
void remove_isolated_vertices(VolumetricMesh& mesh, std::map<int, int>* old2NewVertexIDMap);
}

class VolumetricMesh {
public:
    // Note: This class is abstract and cannot be instantiated; use the constructors in the derived classes (TetMesh,
    // CubicMesh) to initialize a mesh, or use the load routine in volumetricMeshLoader.h

    // copy constructor, destructor
    VolumetricMesh(const VolumetricMesh& volumetricMesh);
    virtual std::unique_ptr<VolumetricMesh> clone() const = 0;
    virtual ~VolumetricMesh();

    // === vertex and element access ===

    enum class ElementType { Invalid, Tet, Cubic };
    // ASCII is the text .veg format, BINARY is the binary .vegb format,
    // BY_EXT: determine the file format based on filename
    enum class FileFormatType { Ascii, Binary, ByExtension, Unknown };
    virtual ElementType getElementType() const = 0;  // calls the derived class to identify itself

    inline int          getNumVertices() const { return m_geometry.num_vertices(); }
    inline Vec3d&       getVertex(int i) { return m_geometry.vertex(i); }
    inline const Vec3d& getVertex(int i) const { return m_geometry.vertex(i); }
    inline Vec3d&       getVertex(int element, int vertex) {
        return m_geometry.vertex(m_geometry.vertex_index(element, vertex));
    }
    inline const Vec3d& getVertex(int element, int vertex) const {
        return m_geometry.vertex(m_geometry.vertex_index(element, vertex));
    }
    inline void getElementVertices(int element, Vec3d* elementVertices)
        const;  // elementVertices should have size of getNumElementVertices() pre-allocated
    inline int getVertexIndex(int element, int vertex) const { return m_geometry.vertex_index(element, vertex); }
    inline std::span<const int> getVertexIndices(int element) const { return m_geometry.vertex_indices(element); }
    inline std::span<Vec3d> getVertices() { return m_geometry.vertices(); }
    inline std::span<const Vec3d> getVertices() const { return m_geometry.vertices(); }
    inline int          getNumElements() const { return m_geometry.num_elements(); }
    inline int          getNumElementVertices() const { return m_geometry.num_element_vertices(); }
    inline std::span<const int> getElements() const { return m_geometry.elements(); }
    void                renumberVertices(
                       const std::vector<int>& permutation);  // renumbers the vertices using the provided permutation
    inline void setVertex(int i, const Vec3d& pos) { m_geometry.set_vertex(i, pos); }  // set the position of a vertex

    // === materials access ===

    int             getNumMaterials() const;
    const MaterialRecord& getMaterial(int i) const;
    const MaterialRecord& getElementMaterial(int el) const;

    int        getNumSets() const;
    const ElementSet& getSet(int i) const;

    int           getNumRegions() const;
    const MaterialRegion& getRegion(int i) const;

    // === materials editing ===
    MaterialRecord& getMaterial(int i);
    MaterialRecord& getElementMaterial(int el);
    void      setMaterial(int i, const MaterialRecord& material);  // sets i-th material to "material"
    void      setSingleMaterial(double E, double nu,
                                double density);  // erases all materials and creates a single material for the entire mesh
    void addMaterial(const MaterialRecord& material, const ElementSet& newSet, bool removeEmptySets,
                     bool removeEmptyMaterials);

    // mass density of an element
    double getElementDensity(int el) const;
    // computes the mass matrix of a single element
    // note: to compute the mass matrix for the entire mesh, use algorithms/generate_mass_matrix.h
    virtual void computeElementMassMatrix(
        int element, double* massMatrix) const = 0;  // massMatrix is numElementVertices_ x numElementVertices_

    // === geometric queries and transformations ===

    Vec3d getElementCenter(int el) const;

    // center of mass and inertia tensor
    double         getVolume() const;
    virtual double getElementVolume(int el) const = 0;
    void           getVertexVolumes(double* vertexVolumes) const;  // compute the volume "belonging" to each vertex
    virtual void   getElementInertiaTensor(int el, Mat3d& inertiaTensor)
        const = 0;  // returns the inertia tensor of a single element, around its center of mass, with unit density
    double getMass() const;  // compute the total mass of the mesh, using the mass density material information
    // get mass, center of mass and inertia tensor (around center of mass) for the entire mesh
    void getInertiaParameters(double& mass, Vec3d& centerOfMass, Mat3d& inertiaTensor) const;

    // centroid is the geometric center of all vertices; radius is the tightest fitting sphere centered at the centroid
    void              getMeshGeometricParameters(Vec3d& centroid, double* radius) const;
    Mesh::BoundingBox getBoundingBox() const;

    // mesh 1-neighborhood queries
    // input elements don't need to be sorted or deduplicated, output vertices are sorted and deduplicated
    void getVerticesInElements(const std::vector<int>& elements, std::vector<int>& vertices) const;
    // input vertices should be sorted, output elements are sorted and deduplicated
    void getElementsTouchingVertices(const std::vector<int>& vertices, std::vector<int>& elements) const;
    // get elements that are only made from vertices from input vector vertices
    // input vertices should be sorted, output elements are sorted and deduplicated
    void getElementsWithOnlyVertices(const std::vector<int>& vertices, std::vector<int>& elements) const;
    void getVertexNeighborhood(const std::vector<int>& vertices, std::vector<int>& neighborhood)
        const;  // get 1-ring of neighboring vtx, including input vtx

    // proximity queries
    virtual int getClosestElement(
        const Vec3d& pos) const;  // finds the closest element to the given position (using linear scan); distance to a
                                  // element is defined as distance to its center
    int getClosestVertex(Vec3d pos) const;      // finds the closest vertex to the given position (using linear scan)
    int getContainingElement(Vec3d pos) const;  // finds the element that containts the given position (using linear
                                                // scan); if such element does not exist, -1 is returned
    virtual bool containsVertex(int   element,
                                Vec3d pos) const = 0;  // true if given element contain given position, false otherwise

    // computes the gravity vector (different forces on different mesh vertices due to potentially varying mass
    // densities) gravityForce must be a pre-allocated vector of length 3xnumVertices()
    void computeGravity(double* gravityForce, double g = 9.81, bool addForce = false) const;

    // edge queries
    virtual int  getNumElementEdges() const = 0;
    virtual void getElementEdges(
        int el, int* edgeBuffer) const = 0;  // edgeBuffer must be pre-allocated, of size 2 x numElementEdges()

    // (permanently) applies the deformation to the vertices of the mesh
    void applyDeformation(const double* u);
    void applyLinearTransformation(
        double* pos, double* R);  // transforms every vertex as X |--> pos + R * X (R must be given row-major)

    // computes barycentric weights of the given position with respect to the given element
    virtual void computeBarycentricWeights(int element, const Vec3d& pos, double* weights) const = 0;

    // computes the gradient of a 3D vector field (specified at the volumetric mesh vertices), at the location "pos"
    // "numFields" fields can be interpolated simultaneously; each is given as one column of the U matrix
    // U is a 3numVertices x numFields matrix; stored column-major
    // output: grad is 9 x numFields matrix, stored column-major; each column gives the gradient (3x3 matrix), stored
    // row-major format return: 0 if pos inside the mesh, 1 otherwise
    int interpolateGradient(const double* U, int numFields, Vec3d pos, double* grad) const;
    // in this version, the element containing the "pos" must be known, and prescribed directly
    virtual void interpolateGradient(int element, const double* U, int numFields, Vec3d pos, double* grad) const = 0;

    // default values for materials
    constexpr static double E_default       = 1e9;
    constexpr static double nu_default      = 0.45;
    constexpr static double density_default = 1000.0;

    // generate the "allElements" set, which is the default set in VolumetricMesh that includes all the elements
    static ElementSet            generateAllElementsSet(int numElements);
    constexpr static const char* allElementsSetName = "allElements";

protected:
    internal::VolumetricMeshData& geometry_data() { return m_geometry; }
    const internal::VolumetricMeshData& geometry_data() const { return m_geometry; }
    internal::MaterialCatalog& material_catalog();
    const internal::MaterialCatalog& material_catalog() const;

    internal::VolumetricMeshData               m_geometry;
    std::unique_ptr<internal::MaterialCatalog> m_material_catalog;

    explicit VolumetricMesh(int numElementVertices_);
    void propagateRegionsToElements();
    // constructs a mesh from the given vertices and elements,
    // with a single region and material ("E, nu" material)
    // "vertices" is double-precision array of length 3 x numVertices
    // "elements" is an integer array of length numElements x numElementVertices
    VolumetricMesh(std::span<const Vec3d> vertices, int numElementVertices, std::span<const int> elements,
                   double E = E_default, double nu = nu_default, double density = density_default);

    // constructs a mesh from the given vertices and elements,
    // with an arbitrary number of sets, regions and materials
    // "vertices" is double-precision array of length 3 x numVertices
    // "elements" is an integer array of length numElements x numElementVertices
    // "materials", "sets" and "regions" will be copied internally (deep copy), so they
    // can be released after calling this constructor
    VolumetricMesh(std::span<const Vec3d> vertices, int numElementVertices, std::span<const int> elements,
                   std::vector<MaterialRecord> materials, std::vector<ElementSet> sets,
                   std::vector<MaterialRegion> regions);

    // creates a submesh consisting of the specified elements of the given mesh
    // if vertexMap is non-null, it also returns a renaming datastructure: vertexMap[big mesh vertex] is the vertex
    // index in the subset mesh
    VolumetricMesh(const VolumetricMesh& mesh, std::span<const int> elements, std::map<int, int>* vertexMap = nullptr);
    void assignMaterialsToElements(int verbose);
    void reset_material_catalog(std::vector<MaterialRecord> materials, std::vector<ElementSet> sets,
                                std::vector<MaterialRegion> regions, int verbose);

private:
    friend class VolumetricMeshExtensions;
    friend class VolumetricMeshLoader;
    friend class internal::MeshMutation;
};

inline void VolumetricMesh::getElementVertices(int element, Vec3d* elementVertices) const {
    for (int i = 0; i < getNumElementVertices(); i++)
        elementVertices[i] = getVertex(element, i);
}

}  // namespace VolumetricMeshes
}  // namespace pgo
