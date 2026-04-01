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

#include "algorithms/generate_surface_mesh.h"

#include "concepts/mesh_concepts.h"
#include "cubicMesh.h"
#include "ops/element_face_ops.h"
#include "pgoLogging.h"
#include "tetMesh.h"

#include <cstdlib>
#include <iostream>
#include <unordered_map>

namespace {

pgo::Mesh::OTriKey reversed_face(const pgo::Mesh::OTriKey& face) {
    return face.getReversedTriKey();
}

pgo::Mesh::ORectKey reversed_face(const pgo::Mesh::ORectKey& face) {
    return face.getReversedRectKey();
}

template <typename FaceContainer, typename FaceKey = typename FaceContainer::value_type>
void accumulate_surface_faces(const FaceContainer& element_faces, bool all_element_faces,
                              std::unordered_map<FaceKey, int>& surface_faces,
                              std::vector<std::vector<int>>& faces, bool triangulate) {
    for (const FaceKey& face : element_faces) {
        if (all_element_faces) {
            pgo::VolumetricMeshes::ops::append_face(face, triangulate, faces);
            continue;
        }

        auto it = surface_faces.find(face);
        if (it != surface_faces.end()) {
            it->second++;
            continue;
        }

        const FaceKey reversed = reversed_face(face);
        it = surface_faces.find(reversed);
        if (it != surface_faces.end()) {
            it->second--;
            continue;
        }

        surface_faces.emplace(face, 1);
    }
}

template <typename FaceKey>
void emit_surface_faces(std::unordered_map<FaceKey, int>& surface_faces, std::vector<std::vector<int>>& faces,
                        bool triangulate) {
    for (const auto& [face, count] : surface_faces) {
        if (count == 0) {
            continue;
        }

        FaceKey oriented_face = face;
        if (count < 0) {
            oriented_face = reversed_face(face);
        }

        for (int i = 0; i < std::abs(count); ++i) {
            pgo::VolumetricMeshes::ops::append_face(oriented_face, triangulate, faces);
        }
    }
}

template <pgo::VolumetricMeshes::concepts::TetMeshLike MeshT>
void compute_surface_mesh_impl(const MeshT& mesh, std::vector<std::vector<int>>& faces, bool all_element_faces) {
    std::unordered_map<pgo::Mesh::OTriKey, int> surface_faces;
    for (int element = 0; element < mesh.getNumElements(); ++element) {
        const auto element_faces = pgo::VolumetricMeshes::ops::tet_element_faces(mesh, element);
        accumulate_surface_faces(element_faces, all_element_faces, surface_faces, faces, false);
    }

    if (!all_element_faces) {
        emit_surface_faces(surface_faces, faces, false);
    }
}

template <pgo::VolumetricMeshes::concepts::CubicMeshLike MeshT>
void compute_surface_mesh_impl(const MeshT& mesh, std::vector<std::vector<int>>& faces, bool triangulate,
                               bool all_element_faces) {
    std::unordered_map<pgo::Mesh::ORectKey, int> surface_faces;
    for (int element = 0; element < mesh.getNumElements(); ++element) {
        const auto element_faces = pgo::VolumetricMeshes::ops::cubic_element_faces(mesh, element);
        accumulate_surface_faces(element_faces, all_element_faces, surface_faces, faces, triangulate);
    }

    if (!all_element_faces) {
        emit_surface_faces(surface_faces, faces, triangulate);
    }
}

template <pgo::VolumetricMeshes::concepts::VolumetricMeshLike MeshT>
void compute_surface_mesh_impl(const MeshT& mesh, std::vector<pgo::EigenSupport::V3d>& vertices,
                               std::vector<std::vector<int>>& faces, bool triangulate, bool all_element_faces) {
    vertices.clear();
    faces.clear();

    for (int vertex = 0; vertex < mesh.getNumVertices(); ++vertex) {
        vertices.emplace_back(mesh.getVertex(vertex));
    }

    if constexpr (pgo::VolumetricMeshes::concepts::TetMeshLike<MeshT>) {
        compute_surface_mesh_impl(mesh, faces, all_element_faces);
    } else {
        compute_surface_mesh_impl(mesh, faces, triangulate, all_element_faces);
    }
}

}  // namespace

void pgo::VolumetricMeshes::GenerateSurfaceMesh::computeMesh(AnyMeshRef mesh,
                                                             std::vector<EigenSupport::V3d>& vertices,
                                                             std::vector<std::vector<int>>& faces, bool triangulate,
                                                             bool allElementFaces) {
    std::visit(
        [&](const auto& mesh_ref) {
            using MeshT = std::remove_cvref_t<decltype(mesh_ref.get())>;
            bool effective_triangulate = triangulate;
            if constexpr (concepts::TetMeshLike<MeshT>) {
                effective_triangulate = false;
            }
            compute_surface_mesh_impl(mesh_ref.get(), vertices, faces, effective_triangulate, allElementFaces);
        },
        mesh);
}
