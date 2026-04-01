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
#include "io/mesh_loaders.h"
#include "io/mesh_io_types.h"
#include "internal/material_catalog.h"
#include "internal/mesh_mutation.h"
#include "ops/cubic_mesh_ops.h"

#include "triple.h"
#include "pgoLogging.h"
#include "EigenSupport.h"

#include <cfloat>
#include <cstring>
#include <set>
#include <map>
#include <vector>
#include <algorithm>

namespace pgo {
namespace VolumetricMeshes {

namespace ES = EigenSupport;

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
    : VolumetricMesh(vertices, 8, flattenCubicElements(elements), E, nu, density), parallelepipedMode(0) {
    if (!elements.empty())
        cubeSize = (getVertex(0, 1) - getVertex(0, 0)).norm();
    else
        cubeSize = 0.0;

    SetInverseCubeSize();
}

CubicMesh::CubicMesh(std::span<const Vec3d> vertices, std::span<const CubicElement> elements,
                     std::vector<std::unique_ptr<Material>> materials, std::vector<Set> sets,
                     std::vector<Region> regions)
    : VolumetricMesh(vertices, 8, flattenCubicElements(elements), std::move(materials), std::move(sets),
                     std::move(regions)),
      parallelepipedMode(0) {
    if (!elements.empty())
        cubeSize = (getVertex(0, 1) - getVertex(0, 0)).norm();
    else
        cubeSize = 0.0;

    SetInverseCubeSize();
}

CubicMesh::CubicMesh(const CubicMesh& source)
    : VolumetricMesh(source),
      cubeSize(source.cubeSize),
      invCubeSize(source.invCubeSize),
      parallelepipedMode(source.parallelepipedMode) {}

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

    geometry_data() = std::move(data.geometry);
    material_catalog() = std::move(data.material_catalog);
    material_catalog().validate_against_num_elements(getNumElements());

    if (getNumElements() > 0)
        cubeSize = (getVertex(0, 1) - getVertex(0, 0)).norm();
    else
        cubeSize = 0.0;
    SetInverseCubeSize();
}

bool CubicMesh::containsVertex(int element, Vec3d pos) const {
    return ops::contains_vertex(*this, element, pos);
}

// interpolates given cubic mesh vertex 3D data to the destination locations
// vertexData size must a vector of length 3 * nElements
// destMatrix must be a vector of length 3 * numLocations
// if a given location does not belong to any element, extrapolation to the nearest element will be used
// vertices more than distanceThreshold away from any element vertex are assigned zero data
int CubicMesh::interpolateData(double* vertexData, int numInterpolationLocations, int r, double* interpolationLocations,
                               double* destMatrix, double distanceThreshold) const {
    assert(getNumElementVertices() == 8);
    const int num_vertices = getNumVertices();

    int numExternalVertices = 0;

    for (int i = 0; i < numInterpolationLocations; i++)  // over all interpolation locations
    {
        if (i % 100 == 0) {
            printf("%d ", i);
            fflush(nullptr);
        }

        Vec3d pos = Vec3d(interpolationLocations[3 * i + 0], interpolationLocations[3 * i + 1],
                          interpolationLocations[3 * i + 2]);

        // find element containing pos
        int element = getClosestElement(pos);

        if (!containsVertex(element, pos))
            numExternalVertices++;

        if (distanceThreshold > 0) {
            // check whether vertex is close enough to the cube mesh
            double minDistance  = DBL_MAX;
            int    assignedZero = 0;
            for (int ii = 0; ii < getNumElementVertices(); ii++) {
                const Vec3d& vpos = getVertex(element, ii);
                if ((vpos - pos).norm() < minDistance) {
                    minDistance = (vpos - pos).norm();
                }
            }

            if (minDistance > distanceThreshold) {
                // assign zero data
                for (int j = 0; j < r; j++) {
                    destMatrix[ES::ELT(3 * numInterpolationLocations, 3 * i + 0, j)] = 0;
                    destMatrix[ES::ELT(3 * numInterpolationLocations, 3 * i + 1, j)] = 0;
                    destMatrix[ES::ELT(3 * numInterpolationLocations, 3 * i + 2, j)] = 0;
                }
                assignedZero++;
                continue;
            }
        }

        // compute barycentric coordinates
        Vec3d  w     = pos - getVertex(element, 0);
        double alpha = w[0] * invCubeSize;
        double beta  = w[1] * invCubeSize;
        double gamma = w[2] * invCubeSize;

        double f000 = (1 - alpha) * (1 - beta) * (1 - gamma);
        double f100 = (alpha) * (1 - beta) * (1 - gamma);
        double f110 = (alpha) * (beta) * (1 - gamma);
        double f010 = (1 - alpha) * (beta) * (1 - gamma);

        double f001 = (1 - alpha) * (1 - beta) * (gamma);
        double f101 = (alpha) * (1 - beta) * (gamma);
        double f111 = (alpha) * (beta) * (gamma);
        double f011 = (1 - alpha) * (beta) * (gamma);

        int v000 = getVertexIndex(element, 0);
        int v100 = getVertexIndex(element, 1);
        int v110 = getVertexIndex(element, 2);
        int v010 = getVertexIndex(element, 3);
        int v001 = getVertexIndex(element, 4);
        int v101 = getVertexIndex(element, 5);
        int v111 = getVertexIndex(element, 6);
        int v011 = getVertexIndex(element, 7);

        for (int j = 0; j < r; j++) {
            Vec3d data000 = Vec3d(vertexData[ES::ELT(3 * num_vertices, 3 * v000 + 0, j)],
                                  vertexData[ES::ELT(3 * num_vertices, 3 * v000 + 1, j)],
                                  vertexData[ES::ELT(3 * num_vertices, 3 * v000 + 2, j)]);
            Vec3d data100 = Vec3d(vertexData[ES::ELT(3 * num_vertices, 3 * v100 + 0, j)],
                                  vertexData[ES::ELT(3 * num_vertices, 3 * v100 + 1, j)],
                                  vertexData[ES::ELT(3 * num_vertices, 3 * v100 + 2, j)]);
            Vec3d data110 = Vec3d(vertexData[ES::ELT(3 * num_vertices, 3 * v110 + 0, j)],
                                  vertexData[ES::ELT(3 * num_vertices, 3 * v110 + 1, j)],
                                  vertexData[ES::ELT(3 * num_vertices, 3 * v110 + 2, j)]);
            Vec3d data010 = Vec3d(vertexData[ES::ELT(3 * num_vertices, 3 * v010 + 0, j)],
                                  vertexData[ES::ELT(3 * num_vertices, 3 * v010 + 1, j)],
                                  vertexData[ES::ELT(3 * num_vertices, 3 * v010 + 2, j)]);

            Vec3d data001 = Vec3d(vertexData[ES::ELT(3 * num_vertices, 3 * v001 + 0, j)],
                                  vertexData[ES::ELT(3 * num_vertices, 3 * v001 + 1, j)],
                                  vertexData[ES::ELT(3 * num_vertices, 3 * v001 + 2, j)]);
            Vec3d data101 = Vec3d(vertexData[ES::ELT(3 * num_vertices, 3 * v101 + 0, j)],
                                  vertexData[ES::ELT(3 * num_vertices, 3 * v101 + 1, j)],
                                  vertexData[ES::ELT(3 * num_vertices, 3 * v101 + 2, j)]);
            Vec3d data111 = Vec3d(vertexData[ES::ELT(3 * num_vertices, 3 * v111 + 0, j)],
                                  vertexData[ES::ELT(3 * num_vertices, 3 * v111 + 1, j)],
                                  vertexData[ES::ELT(3 * num_vertices, 3 * v111 + 2, j)]);
            Vec3d data011 = Vec3d(vertexData[ES::ELT(3 * num_vertices, 3 * v011 + 0, j)],
                                  vertexData[ES::ELT(3 * num_vertices, 3 * v011 + 1, j)],
                                  vertexData[ES::ELT(3 * num_vertices, 3 * v011 + 2, j)]);

            Vec3d interpolatedData = f000 * data000 + f100 * data100 + f110 * data110 + f010 * data010 +
                                     f001 * data001 + f101 * data101 + f111 * data111 + f011 * data011;

            destMatrix[ES::ELT(3 * numInterpolationLocations, 3 * i + 0, j)] = interpolatedData[0];
            destMatrix[ES::ELT(3 * numInterpolationLocations, 3 * i + 1, j)] = interpolatedData[1];
            destMatrix[ES::ELT(3 * numInterpolationLocations, 3 * i + 2, j)] = interpolatedData[2];
        }
    }

    printf("\n");

    return numExternalVertices;
}

// computes approximation to the normal correction for the given deformation
// vertexData size must a vector of length 3 * nElements
// normalCorrection (output) must be a vector of length 3 * numLocations
// staticNormals must be a vector of length 3 * numLocations
// vertices more than distanceThreshold away from any element vertex are assigned zero data
int CubicMesh::normalCorrection(double* vertexData, int numInterpolationLocations, int r,
                                double* interpolationLocations, double* staticNormals, double* normalCorrection,
                                double distanceThreshold) const {
    assert(getNumElementVertices() == 8);
    const int num_vertices = getNumVertices();

    int numExternalVertices = 0;

    for (int i = 0; i < numInterpolationLocations; i++)  // over all interpolation locations
    {
        if (i % 100 == 0) {
            printf("%d ", i);
            fflush(nullptr);
        }

        Vec3d pos = Vec3d(interpolationLocations[3 * i + 0], interpolationLocations[3 * i + 1],
                          interpolationLocations[3 * i + 2]);

        // find element containing pos
        int element = getClosestElement(pos);

        if (!containsVertex(element, pos))
            numExternalVertices++;

        if (distanceThreshold > 0) {
            // check whether vertex is close enough to the cube mesh
            double minDistance  = DBL_MAX;
            int    assignedZero = 0;
            for (int ii = 0; ii < getNumElementVertices(); ii++) {
                const Vec3d& vpos = getVertex(element, ii);
                if ((vpos - pos).norm() < minDistance) {
                    minDistance = (vpos - pos).norm();
                }
            }

            if (minDistance > distanceThreshold) {
                // assign zero data
                for (int j = 0; j < r; j++) {
                    normalCorrection[ES::ELT(3 * numInterpolationLocations, 3 * i + 0, j)] = 0;
                    normalCorrection[ES::ELT(3 * numInterpolationLocations, 3 * i + 1, j)] = 0;
                    normalCorrection[ES::ELT(3 * numInterpolationLocations, 3 * i + 2, j)] = 0;
                }
                assignedZero++;

                continue;
            }
        }

        // compute barycentric coordinates
        Vec3d  w     = pos - getVertex(element, 0);
        double alpha = w[0] * invCubeSize;
        double beta  = w[1] * invCubeSize;
        double gamma = w[2] * invCubeSize;

        // double f000 = (1-alpha)*(1-beta)*(1-gamma);
        // double f100 = (alpha)*(1-beta)*(1-gamma);
        // double f110 = (alpha)*(beta)*(1-gamma);
        // double f010 = (1-alpha)*(beta)*(1-gamma);

        // double f001 = (1-alpha)*(1-beta)*(gamma);
        // double f101 = (alpha)*(1-beta)*(gamma);
        // double f111 = (alpha)*(beta)*(gamma);
        // double f011 = (1-alpha)*(beta)*(gamma);

        int v000 = getVertexIndex(element, 0);
        int v100 = getVertexIndex(element, 1);
        int v110 = getVertexIndex(element, 2);
        int v010 = getVertexIndex(element, 3);
        int v001 = getVertexIndex(element, 4);
        int v101 = getVertexIndex(element, 5);
        int v111 = getVertexIndex(element, 6);
        int v011 = getVertexIndex(element, 7);

        Vec3d gradf000(invCubeSize * -(1 - beta) * (1 - gamma), invCubeSize * -(1 - alpha) * (1 - gamma),
                       invCubeSize * -(1 - alpha) * (1 - beta));
        Vec3d gradf100(invCubeSize * (1 - beta) * (1 - gamma), invCubeSize * -alpha * (1 - gamma),
                       invCubeSize * -alpha * (1 - beta));
        Vec3d gradf110(invCubeSize * beta * (1 - gamma), invCubeSize * alpha * (1 - gamma),
                       invCubeSize * -alpha * beta);
        Vec3d gradf010(invCubeSize * -beta * (1 - gamma), invCubeSize * (1 - alpha) * (1 - gamma),
                       invCubeSize * (1 - alpha) * -beta);

        Vec3d gradf001(invCubeSize * -(1 - beta) * gamma, invCubeSize * -(1 - alpha) * gamma,
                       invCubeSize * (1 - alpha) * (1 - beta));
        Vec3d gradf101(invCubeSize * (1 - beta) * gamma, invCubeSize * -alpha * gamma,
                       invCubeSize * alpha * (1 - beta));
        Vec3d gradf111(invCubeSize * beta * gamma, invCubeSize * alpha * gamma, invCubeSize * alpha * beta);
        Vec3d gradf011(invCubeSize * -beta * gamma, invCubeSize * (1 - alpha) * gamma,
                       invCubeSize * (1 - alpha) * beta);

        Vec3d normal = Vec3d(staticNormals[3 * i + 0], staticNormals[3 * i + 1], staticNormals[3 * i + 2]);

        for (int j = 0; j < r; j++) {
            Vec3d u000 = Vec3d(vertexData[ES::ELT(3 * num_vertices, 3 * v000 + 0, j)],
                               vertexData[ES::ELT(3 * num_vertices, 3 * v000 + 1, j)],
                               vertexData[ES::ELT(3 * num_vertices, 3 * v000 + 2, j)]);
            Vec3d u100 = Vec3d(vertexData[ES::ELT(3 * num_vertices, 3 * v100 + 0, j)],
                               vertexData[ES::ELT(3 * num_vertices, 3 * v100 + 1, j)],
                               vertexData[ES::ELT(3 * num_vertices, 3 * v100 + 2, j)]);
            Vec3d u110 = Vec3d(vertexData[ES::ELT(3 * num_vertices, 3 * v110 + 0, j)],
                               vertexData[ES::ELT(3 * num_vertices, 3 * v110 + 1, j)],
                               vertexData[ES::ELT(3 * num_vertices, 3 * v110 + 2, j)]);
            Vec3d u010 = Vec3d(vertexData[ES::ELT(3 * num_vertices, 3 * v010 + 0, j)],
                               vertexData[ES::ELT(3 * num_vertices, 3 * v010 + 1, j)],
                               vertexData[ES::ELT(3 * num_vertices, 3 * v010 + 2, j)]);

            Vec3d u001 = Vec3d(vertexData[ES::ELT(3 * num_vertices, 3 * v001 + 0, j)],
                               vertexData[ES::ELT(3 * num_vertices, 3 * v001 + 1, j)],
                               vertexData[ES::ELT(3 * num_vertices, 3 * v001 + 2, j)]);
            Vec3d u101 = Vec3d(vertexData[ES::ELT(3 * num_vertices, 3 * v101 + 0, j)],
                               vertexData[ES::ELT(3 * num_vertices, 3 * v101 + 1, j)],
                               vertexData[ES::ELT(3 * num_vertices, 3 * v101 + 2, j)]);
            Vec3d u111 = Vec3d(vertexData[ES::ELT(3 * num_vertices, 3 * v111 + 0, j)],
                               vertexData[ES::ELT(3 * num_vertices, 3 * v111 + 1, j)],
                               vertexData[ES::ELT(3 * num_vertices, 3 * v111 + 2, j)]);
            Vec3d u011 = Vec3d(vertexData[ES::ELT(3 * num_vertices, 3 * v011 + 0, j)],
                               vertexData[ES::ELT(3 * num_vertices, 3 * v011 + 1, j)],
                               vertexData[ES::ELT(3 * num_vertices, 3 * v011 + 2, j)]);

            Vec3d coef(0, 0, 0);
            coef += gradf000.dot(normal) * u000;
            coef += gradf100.dot(normal) * u100;
            coef += gradf110.dot(normal) * u110;
            coef += gradf010.dot(normal) * u010;
            coef += gradf001.dot(normal) * u001;
            coef += gradf101.dot(normal) * u101;
            coef += gradf111.dot(normal) * u111;
            coef += gradf011.dot(normal) * u011;

            normalCorrection[ES::ELT(3 * numInterpolationLocations, 3 * i + 0, j)] = coef[0];
            normalCorrection[ES::ELT(3 * numInterpolationLocations, 3 * i + 1, j)] = coef[1];
            normalCorrection[ES::ELT(3 * numInterpolationLocations, 3 * i + 2, j)] = coef[2];
        }
    }

    printf("\n");

    return numExternalVertices;
}

void CubicMesh::interpolateGradient(int element, const double* U, int numFields, Vec3d pos, double* grad) const {
    ops::interpolate_gradient(*this, element, U, numFields, pos, grad);
}

void CubicMesh::computeAlphaBetaGamma(int el, Vec3d pos, double* alpha, double* beta, double* gamma) const {
    ops::compute_alpha_beta_gamma(*this, el, pos, alpha, beta, gamma);
}

void CubicMesh::computeBarycentricWeights(int el, const Vec3d& pos, double* weights) const {
    ops::compute_barycentric_weights(*this, el, pos, weights);
}

double CubicMesh::getElementVolume(int el) const {
    return ops::element_volume(*this, el);
}

void CubicMesh::getElementInertiaTensor(int el, Mat3d& inertiaTensor) const {
    ops::element_inertia_tensor(*this, el, inertiaTensor);
}

int CubicMesh::getNumElementEdges() const {
    return ops::num_element_edges(*this);
}

void CubicMesh::getElementEdges(int el, int* edgeBuffer) const {
    ops::fill_element_edges(*this, el, edgeBuffer);
}

void CubicMesh::computeElementMassMatrix(int el, double* massMatrix) const {
    ops::compute_element_mass_matrix(*this, el, massMatrix);
}

void CubicMesh::subdivide() {
    const int        num_elements = getNumElements();
    int              numNewElements = 8 * num_elements;
    std::vector<int> newElements(numNewElements * 8);

    int parentMask[8][3] = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}, {0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1}};

    int mask[8][8][3];
    for (int el = 0; el < 8; el++)
        for (int vtx = 0; vtx < 8; vtx++)
            for (int dim = 0; dim < 3; dim++) {
                mask[el][vtx][dim] = parentMask[el][dim] + parentMask[vtx][dim];
                // printf("%d\n", mask[el][vtx][dim]);
            }

    std::vector<Vec3d> newVertices;
    for (int el = 0; el < num_elements; el++) {
        const Vec3d& v0 = getVertex(el, 0);

        // create the 8 children cubes
        for (int child = 0; child < 8; child++) {
            int childVtx[8];
            for (int vtx = 0; vtx < 8; vtx++) {
                Vec3d pos = v0 + 0.5 * cubeSize * Vec3d(mask[child][vtx][0], mask[child][vtx][1], mask[child][vtx][2]);
                // printf("%G %G %G\n", pos[0], pos[1], pos[2]);
                //  search for vertex
                int found = -1;
                for (int i = 0; i < (int)newVertices.size(); i++) {
                    if ((pos - newVertices[i]).norm() < 0.25 * cubeSize) {
                        found = i;
                        break;
                    }
                }

                if (found == -1) {
                    // new vertex
                    newVertices.push_back(pos);
                    found = (int)newVertices.size() - 1;
                }

                childVtx[vtx]                           = found;
                newElements[(8 * el + child) * 8 + vtx] = childVtx[vtx];
            }
        }
    }

    cubeSize *= 0.5;

    internal::MeshMutation::replace_geometry(*this,
                                             internal::VolumetricMeshData(8, std::move(newVertices), std::move(newElements)));
    internal::MeshMutation::material_catalog(*this).expand_elements(8);
}

void CubicMesh::setParallelepipedMode(int parallelepipedMode_) {
    parallelepipedMode = parallelepipedMode_;
}

}  // namespace VolumetricMeshes
}  // namespace pgo
