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

#include "cubicMesh.h"
#include "algorithms/cubic_mesh_interpolation.h"
#include "algorithms/cubic_mesh_normal_correction.h"
#include "algorithms/cubic_mesh_refinement.h"
#include "io/mesh_loaders.h"
#include "io/mesh_io_types.h"
#include "internal/material_catalog.h"
#include "internal/mesh_mutation.h"
#include "ops/mesh_construction.h"
#include "ops/cubic_mesh_ops.h"
#include "traits/mesh_traits.h"

#include "triple.h"
#include "pgoLogging.h"
#include "EigenSupport.h"

#include <set>
#include <map>
#include <vector>
#include <algorithm>

namespace pgo {
namespace VolumetricMeshes {

using CubicMeshTraits = traits::mesh_traits<CubicMesh>;

namespace {
std::vector<int> flattenCubicElements(std::span<const CubicElement> elements) {
    std::vector<int> flat(elements.size() * 8);
    for (size_t ei = 0; ei < elements.size(); ++ei) {
        for (int vi = 0; vi < 8; ++vi) {
            flat[ei * 8 + vi] = elements[ei][vi];
        }
    }
    return flat;
}
}  // namespace

CubicMesh::CubicMesh(const std::filesystem::path& filename, FileFormatType fileFormat, int verbose)
    : VolumetricMesh(8), parallelepipedMode(0) {
    assignFromData(io::detail::load_cubic_data(filename, fileFormat, verbose), verbose);
}

CubicMesh::CubicMesh(std::span<const std::byte> binaryStream)
    : VolumetricMesh(8), parallelepipedMode(0) {
    assignFromData(io::detail::load_cubic_data(binaryStream), 0);
}

CubicMesh::CubicMesh(std::span<const Vec3d> vertices, std::span<const CubicElement> elements, double E, double nu,
                     double density)
    : VolumetricMesh(8), parallelepipedMode(0) {
    const std::vector<int> flat_elements = flattenCubicElements(elements);
    set_storage(ops::make_mesh_storage(vertices, 8, flat_elements, E, nu, density));
}

CubicMesh::CubicMesh(std::span<const Vec3d> vertices, std::span<const CubicElement> elements,
                     std::vector<MaterialRecord> materials, std::vector<ElementSet> sets,
                     std::vector<MaterialRegion> regions)
    : VolumetricMesh(8),
      parallelepipedMode(0) {
    const std::vector<int> flat_elements = flattenCubicElements(elements);
    set_storage(ops::make_mesh_storage(vertices, 8, flat_elements, std::move(materials), std::move(sets),
                                       std::move(regions)));
}

CubicMesh::CubicMesh(const CubicMesh& source)
    : VolumetricMesh(source),
      cubeSize(source.cubeSize),
      invCubeSize(source.invCubeSize),
      parallelepipedMode(source.parallelepipedMode),
      m_storage(source.m_storage) {}

std::unique_ptr<VolumetricMesh> CubicMesh::clone() const {
    return std::make_unique<CubicMesh>(*this);
}

void CubicMesh::SetInverseCubeSize() {
    if (cubeSize > 0)
        invCubeSize = 1.0 / cubeSize;
    else
        invCubeSize = 0;
}

std::unique_ptr<CubicMesh> CubicMesh::createFromUniformGrid(int resolution, std::span<const int> voxels, double E,
                                                            double nu, double density) {
    int numElementVertices = 8;
    const int numVoxels = static_cast<int>(voxels.size() / 3);

    // create the indices of all vertices
    typedef triple<int, int, int> tripleIndex;
    std::set<tripleIndex>         vertexSet;
    int                           vtxI[8] = {0, 1, 1, 0, 0, 1, 1, 0};
    int                           vtxJ[8] = {0, 0, 1, 1, 0, 0, 1, 1};
    int                           vtxK[8] = {0, 0, 0, 0, 1, 1, 1, 1};
    for (int vox = 0; vox < numVoxels; vox++) {
        int i = voxels[3 * vox + 0];
        int j = voxels[3 * vox + 1];
        int k = voxels[3 * vox + 2];

        for (int corner = 0; corner < numElementVertices; corner++) {
            tripleIndex triIndex(i + vtxI[corner], j + vtxJ[corner], k + vtxK[corner]);
            vertexSet.insert(triIndex);
        }
    }

    int                        numVertices = (int)vertexSet.size();
    std::vector<Vec3d>         vertices(numVertices);
    int                        count = 0;
    std::map<tripleIndex, int> vertexMap;
    for (std::set<tripleIndex>::iterator iter = vertexSet.begin(); iter != vertexSet.end(); iter++) {
        int i                   = iter->first;
        int j                   = iter->second;
        int k                   = iter->third;
        vertices[count]         = Vec3d(-0.5 + 1.0 * i / resolution, -0.5 + 1.0 * j / resolution,
                                -0.5 + 1.0 * k / resolution);
        vertexMap.insert(std::make_pair(tripleIndex(i, j, k), count));
        // printf("%d %d %d: %d\n", i,j,k, count);
        count++;
    }

    int numElements = numVoxels;
    std::vector<CubicElement> elements(numElements);
    for (int vox = 0; vox < numElements; vox++) {
        int i = voxels[3 * vox + 0];
        int j = voxels[3 * vox + 1];
        int k = voxels[3 * vox + 2];
        for (int corner = 0; corner < numElementVertices; corner++) {
            int I = i + vtxI[corner];
            int J = j + vtxJ[corner];
            int K = k + vtxK[corner];
            // printf("I=%d J=%d K=%d\n", I, J, K);

            // find I, J, K
            int vtxIndex = vertexMap[tripleIndex(I, J, K)];
            // printf("vtxIndex = %d\n", vtxIndex);
            elements[vox][corner] = vtxIndex;
        }
    }

    return std::make_unique<CubicMesh>(vertices, elements, E, nu, density);
}

CubicMesh::CubicMesh(const CubicMesh& cubeMesh, std::span<const int> elements, std::map<int, int>* vertexMap_)
    : VolumetricMesh(cubeMesh, elements, vertexMap_) {
    sync_storage_from_legacy_state_for_transition();
    cubeSize = cubeMesh.getCubeSize();
    SetInverseCubeSize();
}

CubicMesh::~CubicMesh() {}

void CubicMesh::assignFromData(io::detail::LoadedMeshData data, int) {
    if (data.element_type != elementType()) {
        printf("Error: mesh is not a cubic mesh.\n");
        throw 11;
    }
    if (data.geometry.num_element_vertices() != 8) {
        printf("Error: cubic mesh data has %d vertices per element.\n", data.geometry.num_element_vertices());
        throw 12;
    }

    set_storage(ops::make_mesh_storage(std::move(data)));

    if (getNumElements() > 0)
        cubeSize = (getVertex(0, 1) - getVertex(0, 0)).norm();
    else
        cubeSize = 0.0;
    SetInverseCubeSize();
}

void CubicMesh::sync_storage_from_legacy_state_for_transition() {
    m_storage = storage::MeshStorage(geometry_data(), material_catalog());
    m_storage.validate_invariants();
}

void CubicMesh::set_storage(storage::MeshStorage storage) {
    storage.validate_invariants();
    m_storage = std::move(storage);
    ops::assign_common_storage(*this, m_storage);

    if (getNumElements() > 0) {
        cubeSize = (getVertex(0, 1) - getVertex(0, 0)).norm();
    } else {
        cubeSize = 0.0;
    }
    SetInverseCubeSize();
}

bool CubicMesh::containsVertex(int element, Vec3d pos) const {
    return CubicMeshTraits::contains_vertex(*this, element, pos);
}

// interpolates given cubic mesh vertex 3D data to the destination locations
// vertexData size must a vector of length 3 * nElements
// destMatrix must be a vector of length 3 * numLocations
// if a given location does not belong to any element, extrapolation to the nearest element will be used
// vertices more than distanceThreshold away from any element vertex are assigned zero data
int CubicMesh::interpolateData(double* vertexData, int numInterpolationLocations, int r, double* interpolationLocations,
                               double* destMatrix, double distanceThreshold) const {
    return algorithms::interpolate_cubic_mesh_data(*this, vertexData, numInterpolationLocations, r,
                                                   interpolationLocations, destMatrix, distanceThreshold);
}

// computes approximation to the normal correction for the given deformation
// vertexData size must a vector of length 3 * nElements
// normalCorrection (output) must be a vector of length 3 * numLocations
// staticNormals must be a vector of length 3 * numLocations
// vertices more than distanceThreshold away from any element vertex are assigned zero data
int CubicMesh::normalCorrection(double* vertexData, int numInterpolationLocations, int r,
                                double* interpolationLocations, double* staticNormals, double* normalCorrection,
                                double distanceThreshold) const {
    return algorithms::cubic_mesh_normal_correction(*this, vertexData, numInterpolationLocations, r,
                                                    interpolationLocations, staticNormals, normalCorrection,
                                                    distanceThreshold);
}

void CubicMesh::interpolateGradient(int element, const double* U, int numFields, Vec3d pos, double* grad) const {
    CubicMeshTraits::interpolate_gradient(*this, element, U, numFields, pos, grad);
}

void CubicMesh::computeAlphaBetaGamma(int el, Vec3d pos, double* alpha, double* beta, double* gamma) const {
    ops::compute_alpha_beta_gamma(*this, el, pos, alpha, beta, gamma);
}

void CubicMesh::computeBarycentricWeights(int el, const Vec3d& pos, double* weights) const {
    CubicMeshTraits::compute_barycentric_weights(*this, el, pos, weights);
}

double CubicMesh::getElementVolume(int el) const {
    return CubicMeshTraits::element_volume(*this, el);
}

void CubicMesh::getElementInertiaTensor(int el, Mat3d& inertiaTensor) const {
    CubicMeshTraits::element_inertia_tensor(*this, el, inertiaTensor);
}

int CubicMesh::getNumElementEdges() const {
    return CubicMeshTraits::num_element_edges(*this);
}

void CubicMesh::getElementEdges(int el, int* edgeBuffer) const {
    CubicMeshTraits::fill_element_edges(*this, el, edgeBuffer);
}

void CubicMesh::computeElementMassMatrix(int el, double* massMatrix) const {
    CubicMeshTraits::compute_element_mass_matrix(*this, el, massMatrix);
}

void CubicMesh::subdivide() {
    algorithms::subdivide_cubic_mesh(*this);
    sync_storage_from_legacy_state_for_transition();
}

void CubicMesh::setParallelepipedMode(int parallelepipedMode_) {
    parallelepipedMode = parallelepipedMode_;
}

}  // namespace VolumetricMeshes
}  // namespace pgo
