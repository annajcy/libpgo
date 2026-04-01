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

#include "algorithms/generate_mass_matrix.h"
#include "volumetricMesh.h"

namespace pgo::VolumetricMeshes::GenerateMassMatrix {

void computeMassMatrix(const VolumetricMesh* volumetricMesh, EigenSupport::SpMatD& massMatrix, bool inflate3Dim,
                       const double* elementWeight) {
    detail::compute_mass_matrix_impl(*volumetricMesh, massMatrix, inflate3Dim, elementWeight);
}

void computeVertexMasses(const VolumetricMesh* volumetricMesh, double* masses, bool inflate3Dim) {
    detail::compute_vertex_masses_impl(*volumetricMesh, masses, inflate3Dim);
}

void computeVertexMassesByAveragingNeighboringElements(const VolumetricMesh* volumetricMesh, double* masses,
                                                       bool inflate3Dim) {
    detail::compute_vertex_masses_by_averaging_neighboring_elements_impl(*volumetricMesh, masses, inflate3Dim);
}

}  // namespace pgo::VolumetricMeshes::GenerateMassMatrix
