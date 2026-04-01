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
  Computes the mass matrix for the given volumetric mesh.
  See also volumetricMesh.h .
*/

#pragma once

#include "EigenDef.h"
#include "concepts/mesh_concepts.h"

#include <cstring>
#include <vector>

#include <tbb/parallel_for.h>
#include <tbb/spin_mutex.h>

namespace pgo {
namespace VolumetricMeshes {

namespace GenerateMassMatrix {

namespace detail {

template <class MeshT>
void compute_mass_matrix_impl(const MeshT& mesh, EigenSupport::SpMatD& mass_matrix, bool inflate3_dim,
                              const double* element_weight) {
    const int n                    = mesh.getNumVertices();
    const int num_element_vertices = mesh.getNumElementVertices();

    if (inflate3_dim) {
        mass_matrix.resize(3 * n, 3 * n);
    } else {
        mass_matrix.resize(n, n);
    }

    std::vector<EigenSupport::TripletD> entries;

    if (inflate3_dim) {
        entries.resize(num_element_vertices * num_element_vertices * 3 * mesh.getNumElements());
    } else {
        entries.resize(num_element_vertices * num_element_vertices * mesh.getNumElements());
    }

    tbb::parallel_for(0, mesh.getNumElements(), [&](int el) {
        thread_local EigenSupport::MXd element_mass;

        if (element_mass.rows() == 0) {
            element_mass.resize(num_element_vertices, num_element_vertices);
        }

        mesh.computeElementMassMatrix(el, element_mass.data());
        for (int i = 0; i < num_element_vertices; i++) {
            const int vtxi = mesh.getVertexIndex(el, i);
            for (int j = 0; j < num_element_vertices; j++) {
                const int vtxj = mesh.getVertexIndex(el, j);
                double w       = 1.0;
                if (element_weight) {
                    w = element_weight[el];
                }

                const double entry = element_mass(i, j) * w;
                if (!inflate3_dim) {
                    entries[el * num_element_vertices * num_element_vertices + i * num_element_vertices + j] =
                        EigenSupport::TripletD(vtxi, vtxj, entry);
                } else {
                    for (int d = 0; d < 3; d++) {
                        entries[(el * num_element_vertices * num_element_vertices + i * num_element_vertices + j) * 3 + d] =
                            EigenSupport::TripletD(vtxi * 3 + d, vtxj * 3 + d, entry);
                    }
                }
            }
        }
    });

    mass_matrix.setFromTriplets(entries.begin(), entries.end());
}

template <class MeshT>
void compute_vertex_masses_impl(const MeshT& mesh, double* masses, bool inflate3_dim) {
    const int n                    = mesh.getNumVertices();
    const int num_element_vertices = mesh.getNumElementVertices();
    std::memset(masses, 0, sizeof(double) * n * (inflate3_dim ? 3 : 1));

    std::vector<tbb::spin_mutex> vertex_locks(n);
    tbb::parallel_for(0, mesh.getNumElements(), [&](int el) {
        thread_local EigenSupport::MXd element_mass;

        if (element_mass.rows() == 0) {
            element_mass.resize(num_element_vertices, num_element_vertices);
        }

        mesh.computeElementMassMatrix(el, element_mass.data());
        for (int i = 0; i < num_element_vertices; i++) {
            const int vtxi = mesh.getVertexIndex(el, i);
            double vtx_mass = 0.0;
            for (int j = 0; j < num_element_vertices; j++) {
                vtx_mass += element_mass(i, j);
            }

            double* mass_buffer_ptr = inflate3_dim ? &masses[3 * vtxi] : &masses[vtxi];

            vertex_locks[vtxi].lock();
            *mass_buffer_ptr += vtx_mass;
            vertex_locks[vtxi].unlock();
        }
    });

    if (inflate3_dim) {
        for (int i = 0; i < n; i++) {
            masses[3 * i + 1] = masses[3 * i + 2] = masses[3 * i];
        }
    }
}

template <class MeshT>
void compute_vertex_masses_by_averaging_neighboring_elements_impl(const MeshT& mesh, double* masses,
                                                                  bool inflate3_dim) {
    const int n                    = mesh.getNumVertices();
    const int num_element_vertices = mesh.getNumElementVertices();
    const double inv_num_ele_vtx   = 1.0 / num_element_vertices;
    std::memset(masses, 0, sizeof(double) * n * (inflate3_dim ? 3 : 1));

    std::vector<tbb::spin_mutex> vertex_locks(n);
    tbb::parallel_for(0, mesh.getNumElements(), [&](int el) {
        const double vtx_mass = mesh.getElementVolume(el) * mesh.getElementDensity(el) * inv_num_ele_vtx;
        for (int i = 0; i < num_element_vertices; i++) {
            const int vtxi = mesh.getVertexIndex(el, i);

            double* mass_buffer_ptr = inflate3_dim ? &masses[3 * vtxi] : &masses[vtxi];
            vertex_locks[vtxi].lock();
            *mass_buffer_ptr += vtx_mass;
            vertex_locks[vtxi].unlock();
        }
    });

    if (inflate3_dim) {
        for (int i = 0; i < n; i++) {
            masses[3 * i + 1] = masses[3 * i + 2] = masses[3 * i];
        }
    }
}

}  // namespace detail

template <concepts::VolumetricMeshLike MeshT>
void computeMassMatrix(const MeshT& mesh, EigenSupport::SpMatD& massMatrix, bool inflate3Dim = false,
                       const double* elementWeight = nullptr) {
    detail::compute_mass_matrix_impl(mesh, massMatrix, inflate3Dim, elementWeight);
}

template <concepts::VolumetricMeshLike MeshT>
void computeVertexMasses(const MeshT& mesh, double* masses, bool inflate3Dim = false) {
    detail::compute_vertex_masses_impl(mesh, masses, inflate3Dim);
}

template <concepts::VolumetricMeshLike MeshT>
void computeVertexMassesByAveragingNeighboringElements(const MeshT& mesh, double* masses, bool inflate3Dim = false) {
    detail::compute_vertex_masses_by_averaging_neighboring_elements_impl(mesh, masses, inflate3Dim);
}

}  // namespace GenerateMassMatrix

}  // namespace VolumetricMeshes
}  // namespace pgo
