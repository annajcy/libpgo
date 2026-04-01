/*************************************************************************
 *                                                                       *
 * Vega FEM Simulation Library Version 4.0                               *
 *                                                                       *
 * "interpolationCoordinates" library , Copyright (C) 2018 USC           *
 * All rights reserved.                                                  *
 *                                                                       *
 * Code authors: Yijing Li, Jernej Barbic                                *
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

#include "barycentricCoordinates.h"
#include "algorithms/mesh_interpolation.h"
#include "io/interpolation_weight_io.h"
#include "pgoLogging.h"
#include "boundingVolumeTree.h"
#include "EigenSupport.h"
#include "tetMesh.h"
#include "cubicMesh.h"

#include <cassert>
#include <iostream>
#include <fstream>
#include <sstream>
#include <queue>
#include <climits>
#include <atomic>

#include <tbb/parallel_for.h>

using namespace pgo;
using namespace pgo::InterpolationCoordinates;
using namespace pgo::VolumetricMeshes;

namespace ES = pgo::EigenSupport;

BarycentricCoordinates::BarycentricCoordinates(int numLocations_, const double* locations,
                                               VolumetricMeshes::AnyMeshRef mesh) {
    PGO_ALOG(locations != nullptr);
    initializeInterpolationWeights(numLocations_, locations, mesh);
}

void BarycentricCoordinates::initializeInterpolationWeights(int numLocations_, const double* locations,
                                                            VolumetricMeshes::AnyMeshRef mesh_ref) {
    std::visit(
        [&](const auto& typed_mesh_ref) {
            const auto& mesh = typed_mesh_ref.get();

            numCageVertices = mesh.getNumVertices();
            numLocations = numLocations_;
            numElementVertices = mesh.getNumElementVertices();

            indices.resize(numElementVertices * numLocations);
            weights.resize(numElementVertices * numLocations);
            elements.resize(numLocations);

            std::atomic<int> numExternalVertices(0);

            Mesh::BoundingBoxBVTree bvTree;
            std::vector<Mesh::BoundingBox> elementBBs(mesh.getNumElements());
            for (int ei = 0; ei < mesh.getNumElements(); ei++) {
                const auto vertex_indices = mesh.getVertexIndices(ei);
                BasicAlgorithms::ArrayRef<int> index_ref(mesh.getNumElementVertices(), vertex_indices.data());
                elementBBs[static_cast<size_t>(ei)] = Mesh::BoundingBox(mesh.getVertices().data(), index_ref);
            }
            bvTree.buildByInertiaPartition(elementBBs);

            tbb::parallel_for(0, numLocations, [&](int i) {
                ES::V3d pos = ES::Mp<const ES::V3d>(locations + 3 * i);
                thread_local std::vector<int> closest_bb_ids;

                closest_bb_ids.clear();
                bvTree.getClosestBoundingBoxes(elementBBs, pos, closest_bb_ids);
                PGO_ALOG(closest_bb_ids.size() > 0);

                int target_element_id = -1;
                for (int ele_id : closest_bb_ids) {
                    if (mesh.containsVertex(ele_id, pos)) {
                        target_element_id = ele_id;
                        break;
                    }
                }

                if (target_element_id < 0) {
                    double closest_distance2 = DBL_MAX;
                    for (int ele_id : closest_bb_ids) {
                        Vec3d center = mesh.getElementCenter(ele_id);
                        const double dist2 = (pos - center).squaredNorm();
                        if (dist2 < closest_distance2) {
                            closest_distance2 = dist2;
                            target_element_id = ele_id;
                        }
                    }
                    numExternalVertices++;
                }

                elements[static_cast<size_t>(i)] = target_element_id;

                std::memcpy(indices.data() + i * numElementVertices, mesh.getVertexIndices(target_element_id).data(),
                            sizeof(int) * numElementVertices);

                mesh.computeBarycentricWeights(target_element_id, pos, weights.data() + i * numElementVertices);
            });
        },
        mesh_ref);
}

BarycentricCoordinates::BarycentricCoordinates(int numLocations_, int numElementVertices_, const int* indices_,
                                               const double* weights_, const int* elementIndices_) {
    numLocations       = numLocations_;
    numElementVertices = numElementVertices_;
    indices.resize(numElementVertices * numLocations);
    weights.resize(numElementVertices * numLocations);
    memcpy(&indices[0], indices_, sizeof(int) * indices.size());
    memcpy(&weights[0], weights_, sizeof(double) * weights.size());
    elements.resize(numLocations, -1);
    if (elementIndices_)
        elements.assign(elementIndices_, elementIndices_ + numLocations);
}

BarycentricCoordinates::BarycentricCoordinates(const std::vector<Vec4i>& tetVtxIndices,
                                               const std::vector<Vec4d>& tetWeights, const int* elementIndices)
    : BarycentricCoordinates(std::min(tetVtxIndices.size(), tetWeights.size()), 4, (const int*)tetVtxIndices.data(),
                             (const double*)tetWeights.data(), elementIndices) {
    PGO_ALOG(tetVtxIndices.size() == tetWeights.size());
}

void BarycentricCoordinates::deform(const double* verticesDisp, double* locationDisp) const {
    interpolation::apply(verticesDisp, locationDisp, numLocations, numElementVertices, &indices[0], &weights[0]);
}

int BarycentricCoordinates::saveInterpolationWeights(const std::string& filename) const {
    InterpolationWeights interpolation;
    interpolation.numElementVertices = numElementVertices;
    interpolation.indices            = indices;
    interpolation.weights            = weights;
    interpolation.elements           = elements;
    int ret = interpolation::save_weights(filename, interpolation);
    std::cout << (ret == 0 ? "Saved" : "Failed to save");
    std::cout << " interpolation weights (numLocations: " << numLocations
              << ", numElementVertices: " << numElementVertices << ") to " << filename << "." << std::endl;
    return ret;
}

ES::SpMatD BarycentricCoordinates::generateInterpolationMatrix() const {
    PGO_ALOG(numCageVertices > 0);

    return ES::createWeightMatrix(numLocations * 3, numCageVertices * 3, numLocations, numElementVertices, nullptr,
                                  getEmbeddingVertexIndices().data(), getEmbeddingWeights().data(), 3);
}

#define READ_ONE_PAIR                                                                                              \
    do {                                                                                                           \
        int    index = 0;                                                                                          \
        double w     = 0;                                                                                          \
        ss >> index;                                                                                               \
        ss >> w;                                                                                                   \
        if (ss.fail()) {                                                                                           \
            std::cerr << index << " " << w << std::endl;                                                           \
            std::cerr << "Error: incorrect interp file format at \"" << buffer << "\" in interp file " << filename \
                      << "." << std::endl;                                                                         \
            throw 1;                                                                                               \
        }                                                                                                          \
        if (index < 0) {                                                                                           \
            std::cerr << "Error: invalid index at \"" << buffer << "\" in interp file " << filename << "."         \
                      << std::endl;                                                                                \
            throw 1;                                                                                               \
        }                                                                                                          \
        indices.push_back(index);                                                                                  \
        weights.push_back(w);                                                                                      \
    } while (0)

BarycentricCoordinates::BarycentricCoordinates(const std::string& filename) {
    std::ifstream fin(filename.c_str(), std::ios::binary);
    if (!fin) {
        std::cerr << "Error: cannot open interp file " << filename << "." << std::endl;
        throw 1;
    }
    fin >> std::ws;

    int               count = 0;
    std::string       buffer;
    std::stringstream ss;
    numElementVertices = INT_MAX;
    while (!fin.eof()) {
        getline(fin, buffer);
        fin >> std::ws;
        if (buffer.size() == 0)
            continue;

        //    istringstream ss(buffer);
        ss.clear();
        ss.str(buffer);
        ss.seekg(0);
        int c = 0;
        ss >> c;
        //    cout << "At line: " << buffer << endl;
        if (c != count) {
            std::cerr << "Warning: wrong line index at \"" << buffer << "\" in interp file " << filename << "."
                      << std::endl;
        }
        if (numElementVertices == INT_MAX) {
            // determine #element vertices
            ss >> std::ws;
            //      cout << "First line " << endl;
            numElementVertices = 0;
            while (!ss.eof()) {
                //        cout << "RRR" << endl;
                READ_ONE_PAIR;
                numElementVertices++;
                //        cout << "numElementVertices now: " << numElementVertices << endl;
                ss >> std::ws;
            }
        } else {
            for (int i = 0; i < numElementVertices; i++) {
                READ_ONE_PAIR;
            }
        }
        count++;
    }
    numLocations = count;

    elements.resize(numLocations, -1);  // no information for elements read from file

    std::cout << "Loaded " << numLocations << " locations from file " << filename
              << " with numElementVertices = " << numElementVertices << "." << std::endl;
    PGO_ALOG((int)indices.size() == numLocations * numElementVertices);
    PGO_ALOG((int)weights.size() == numLocations * numElementVertices);
}
