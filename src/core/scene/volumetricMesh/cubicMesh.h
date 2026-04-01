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

#include "volumetricMeshTypes.h"
#include "volumetricMesh.h"

#include <filesystem>
#include <memory>
#include <span>
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

class CubicMesh : public VolumetricMesh {
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
              std::vector<std::unique_ptr<Material>> materials, std::vector<Set> sets,
              std::vector<Region> regions);

    // constructs a voxel mesh with the given voxels, as a subset of a regular 3D grid
    // 'voxels' gives the grid indices (3 per voxel) of the voxels that are to be included in the mesh
    // 'voxels' has length 3 x numVoxels
    static std::unique_ptr<CubicMesh> createFromUniformGrid(int resolution, std::span<const int> voxels,
                                                            double E = E_default, double nu = nu_default,
                                                            double density = density_default);

    // creates a mesh consisting of the specified element subset of the given CubicMesh
    CubicMesh(const CubicMesh& mesh, std::span<const int> elements, std::map<int, int>* vertexMap = nullptr);

    CubicMesh(const CubicMesh& CubicMesh);
    virtual std::unique_ptr<VolumetricMesh> clone() const override;
    virtual ~CubicMesh();

    // === misc queries ===

    static VolumetricMesh::ElementType  elementType() { return ElementType::Cubic; }
    virtual VolumetricMesh::ElementType getElementType() const override { return elementType(); }

    inline double getCubeSize() const { return cubeSize; }

    virtual double getElementVolume(int el) const override;
    virtual void   computeElementMassMatrix(int el, double* massMatrix) const override;
    virtual void   getElementInertiaTensor(int el, Mat3d& inertiaTensor) const override;

    virtual bool containsVertex(
        int element, Vec3d pos) const override;  // true if given element contain given position, false otherwise

    // edge queries
    virtual int  getNumElementEdges() const override;
    virtual void getElementEdges(int el, int* edgeBuffer) const override;

    // subdivides the cube mesh
    void subdivide();

    // === interpolation ===

    virtual void computeBarycentricWeights(int el, const Vec3d& pos, double* weights) const override;

    int interpolateData(double* volumetricMeshVertexData, int numLocations, int r, double* interpolationLocations,
                        double* destMatrix, double zeroThreshold = -1.0) const;

    // computes approximation to the normal correction for the given deformations
    // vertexData size must a matrix of size 3 * nElements x r
    // destMatrix must be a vector of length 3 * numLocations x r
    // staticNormals must be a vector of length 3 * numLocations
    // returns the number of vertices that were not contained inside any element
    // vertices more than distanceThreshold away from any element vertex are assigned zero data
    int normalCorrection(double* vertexData, int numLocations, int r, double* interpolationLocations,
                         double* staticNormals, double* normalCorrection, double zeroThreshold = -1.0)
        const;  // note: this routine could be promoted to volumetricMesh.h

    virtual void interpolateGradient(int element, const double* U, int numFields, Vec3d pos,
                                     double* grad) const override;

    // advanced, to ensure computeBarycentricWeights, containsVertex, generateInterpolationWeights,
    // generateContainingElements work even when elements are cubes, transformed via a general linear transformation
    // parallelepiped=1 : the elements are cubes transformed via a linear transformation (i.e., they are
    // parallelepipeds) parallelepiped=0 : (default) the elements are axis-aligned cubes
    void setParallelepipedMode(int parallelepipedMode);

protected:
    double cubeSize;
    double invCubeSize;
    CubicMesh(int numElementVertices) : VolumetricMesh(numElementVertices) {}
    void assignFromData(io::detail::LoadedMeshData data, int verbose = 0);
    void SetInverseCubeSize();
    int  parallelepipedMode;  // normally this is 0; in advanced usage, it can be 1 (see above)

    // computes the normalized location of "pos" inside el
    // when inside the element, one has 0 <= alpha <= 1, 0 <= beta <= 1, 0 <= gamma <= 1
    void computeAlphaBetaGamma(int el, Vec3d pos, double* alpha, double* beta, double* gamma) const;

    friend class VolumetricMeshExtensions;
    friend void ops::compute_alpha_beta_gamma(const CubicMesh& mesh, int element, Vec3d pos, double* alpha,
                                              double* beta, double* gamma);
    friend void algorithms::subdivide_cubic_mesh(CubicMesh& mesh);
};

}  // namespace VolumetricMeshes
}  // namespace pgo
