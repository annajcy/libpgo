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

#include "generateSurfaceMesh.h"
#include "cubicMesh.h"
#include "ops/element_face_ops.h"
#include "tetMesh.h"
#include "pgoLogging.h"

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

}  // namespace

// the main routine
void pgo::VolumetricMeshes::GenerateSurfaceMesh::computeMesh(const VolumetricMesh*           volumetricMesh,
                                                             std::vector<EigenSupport::V3d>& vertices,
                                                             std::vector<std::vector<int>>& faces, bool triangulate,
                                                             bool allElementFaces) {
    const int faceDegree = ops::face_degree(*volumetricMesh);
    if (faceDegree == 3) {
        triangulate = false;
    }

    if (faceDegree == 0) {
        printf("Error: unsupported volumetricMesh type encountered.\n");
        return;
    }

    // create an empty surface volumetricMesh
    vertices.clear();
    faces.clear();

    // add all vertices
    for (int i = 0; i < volumetricMesh->getNumVertices(); i++)
        vertices.emplace_back(volumetricMesh->getVertex(i));

    // build unique list of all surface faces

    if (volumetricMesh->getElementType() == VolumetricMesh::ElementType::Tet) {
        const TetMesh* tetMesh = dynamic_cast<const TetMesh*>(volumetricMesh);
        PGO_ALOG(tetMesh != nullptr);

        std::unordered_map<Mesh::OTriKey, int> surfaceFaces;
        for (int i = 0; i < volumetricMesh->getNumElements(); i++) {
            const auto element_faces = ops::tet_element_faces(*tetMesh, i);
            accumulate_surface_faces(element_faces, allElementFaces, surfaceFaces, faces, false);
        }

        if (allElementFaces == false) {
            emit_surface_faces(surfaceFaces, faces, false);
        }
    } else if (volumetricMesh->getElementType() == VolumetricMesh::ElementType::Cubic) {
        const CubicMesh* cubicMesh = dynamic_cast<const CubicMesh*>(volumetricMesh);
        PGO_ALOG(cubicMesh != nullptr);

        std::unordered_map<Mesh::ORectKey, int> surfaceFaces;
        for (int i = 0; i < volumetricMesh->getNumElements(); i++) {
            const auto element_faces = ops::cubic_element_faces(*cubicMesh, i);
            accumulate_surface_faces(element_faces, allElementFaces, surfaceFaces, faces, triangulate);
        }
        if (allElementFaces == false) {
            emit_surface_faces(surfaceFaces, faces, triangulate);
        }
    } else {
        std::cerr << "Error: unknown VolumetricMesh element type in GenerateSurfaceMesh" << std::endl;
        return;
    }
}
