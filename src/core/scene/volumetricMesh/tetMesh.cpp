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

#include "tetMesh.h"
#include "io/detail/gmsh_reader.h"
#include "io/detail/tetgen_reader.h"
#include "io/mesh_loaders.h"
#include "io/mesh_io_types.h"
#include "ops/mesh_construction.h"
#include "ops/tet_mesh_ops.h"

namespace pgo {
namespace VolumetricMeshes {

namespace {
std::vector<int> flattenTetElements(std::span<const Vec4i> elements) {
    std::vector<int> flat(elements.size() * 4);
    for (size_t ei = 0; ei < elements.size(); ++ei) {
        for (int vi = 0; vi < 4; ++vi) {
            flat[ei * 4 + vi] = elements[ei][vi];
        }
    }
    return flat;
}
}  // namespace

TetMesh::TetMesh(const std::filesystem::path& filename, FileFormatType fileFormat, int verbose)
    : VolumetricMesh(4) {
    assignFromData(io::detail::load_tet_data(filename, fileFormat, verbose), verbose);
}

TetMesh::TetMesh(std::span<const std::byte> binaryStream) : VolumetricMesh(4) {
    assignFromData(io::detail::load_tet_data(binaryStream), 0);
}

TetMesh::TetMesh(const std::filesystem::path& filename, int specialFileType, int verbose) : VolumetricMesh(4) {
    if (specialFileType == 0) {
        assignFromData(io::detail::read_tetgen_mesh(filename, verbose), verbose);
        return;
    }

#if defined(PGO_HAS_GMSH)
    if (specialFileType == 1) {
        assignFromData(io::detail::read_gmsh_mesh(filename, verbose), verbose);
        return;
    }
#endif

    printf("Unknown special file type %d requested.\n", specialFileType);
    throw 1;
}

TetMesh::TetMesh(const Vec3d& p0, const Vec3d& p1, const Vec3d& p2, const Vec3d& p3) : VolumetricMesh(4) {
    geometry_data() = internal::VolumetricMeshData(4, {p0, p1, p2, p3}, {0, 1, 2, 3});
    setSingleMaterial(E_default, nu_default, density_default);
}

TetMesh::TetMesh(std::span<const Vec3d> vertices_, std::span<const Vec4i> elements_, double E, double nu,
                 double density)
    : VolumetricMesh(vertices_, 4, flattenTetElements(elements_), E, nu, density) {}

TetMesh::TetMesh(std::span<const Vec3d> vertices_, std::span<const Vec4i> elements_,
                 std::vector<MaterialRecord> materials_, std::vector<ElementSet> sets_,
                 std::vector<MaterialRegion> regions_)
    : VolumetricMesh(vertices_, 4, flattenTetElements(elements_), std::move(materials_), std::move(sets_),
                     std::move(regions_)) {}

TetMesh::TetMesh(const TetMesh& source) : VolumetricMesh(source) {}

std::unique_ptr<VolumetricMesh> TetMesh::clone() const {
    return std::make_unique<TetMesh>(*this);
}

TetMesh::TetMesh(const TetMesh& tetMesh, std::span<const int> elements_, std::map<int, int>* vertexMap_)
    : VolumetricMesh(tetMesh, elements_, vertexMap_) {}

TetMesh::~TetMesh() {}

void TetMesh::assignFromData(io::detail::LoadedMeshData data, int) {
    if (data.element_type != elementType()) {
        printf("Error: mesh is not a tet mesh.\n");
        throw 11;
    }
    if (data.geometry.num_element_vertices() != 4) {
        printf("Error: tet mesh data has %d vertices per element.\n", data.geometry.num_element_vertices());
        throw 12;
    }

    ops::assign_common_loaded_data(*this, std::move(data));
}

void TetMesh::computeElementMassMatrix(int el, double* massMatrix) const {
    ops::compute_element_mass_matrix(*this, el, massMatrix);
}

double TetMesh::getSignedTetVolume(const Vec3d& a, const Vec3d& b, const Vec3d& c, const Vec3d& d) {
    return ops::signed_tet_volume(a, b, c, d);
}

double TetMesh::getTetVolume(const Vec3d& a, const Vec3d& b, const Vec3d& c, const Vec3d& d) {
    return ops::tet_volume(a, b, c, d);
}

double TetMesh::getElementVolume(int el) const {
    return ops::element_volume(*this, el);
}

void TetMesh::getElementInertiaTensor(int el, Mat3d& inertiaTensor) const {
    ops::element_inertia_tensor(*this, el, inertiaTensor);
}

bool TetMesh::containsVertex(int el, Vec3d pos) const  // true if given element contain given position, false otherwise
{
    return ops::contains_vertex(*this, el, pos);
}

void TetMesh::computeBarycentricWeights(const Vec3d tetVtxPos[4], const Vec3d& pos, double weights[4]) {
    ops::compute_barycentric_weights(std::span<const Vec3d, 4>(tetVtxPos, 4), pos, weights);
}

void TetMesh::computeBarycentricWeights(const Vec3d& tetVtxPos0, const Vec3d& tetVtxPos1, const Vec3d& tetVtxPos2,
                                        const Vec3d& tetVtxPos3, const Vec3d& pos, double weights[4]) {
    ops::compute_barycentric_weights(tetVtxPos0, tetVtxPos1, tetVtxPos2, tetVtxPos3, pos, weights);
}

void TetMesh::computeBarycentricWeights(int el, const Vec3d& pos, double* weights) const {
    ops::compute_barycentric_weights(*this, el, pos, weights);
}

double TetMesh::getTetDeterminant(const Vec3d& a, const Vec3d& b, const Vec3d& c, const Vec3d& d) {
    return ops::tet_determinant(a, b, c, d);
}

void TetMesh::interpolateGradient(int element, const double* U, int numFields, Vec3d pos, double* grad) const {
    computeGradient(element, U, numFields, grad);
}

void TetMesh::computeGradient(int element, const double* U, int numFields, double* grad) const {
    ops::compute_gradient(*this, element, U, numFields, grad);
}

int TetMesh::getNumElementEdges() const {
    return ops::num_element_edges(*this);
}

void TetMesh::getElementEdges(int el, int* edgeBuffer) const {
    ops::fill_element_edges(*this, el, edgeBuffer);
}

void TetMesh::orient() {
    for (int el = 0; el < getNumElements(); el++) {
        const double det =
            ops::tet_determinant(getVertex(el, 0), getVertex(el, 1), getVertex(el, 2), getVertex(el, 3));

        if (det < 0) {
            std::span<int> element_vertices = geometry_data().vertex_indices(el);
            std::swap(element_vertices[2], element_vertices[3]);
        }
    }
}

int TetMesh::getClosestElement(const Vec3d& pos) const {
    return ops::closest_element(*this, pos);
}
}  // namespace VolumetricMeshes
}  // namespace pgo
