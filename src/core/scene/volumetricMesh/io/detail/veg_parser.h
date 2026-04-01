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

#pragma once

#include <cstdio>
#include <string>
#include <vector>

namespace pgo::VolumetricMeshes::io::detail {

class VolumetricMeshParser {
public:
    VolumetricMeshParser(const char* include_token = nullptr);
    ~VolumetricMeshParser();

    int open(const char* filename);

    char* getNextLine(char* s, int num_retained_spaces = 0, int remove_whitespace = 1);

    void rewindToStart();
    void close();

    static void upperCase(char* s);
    static void removeWhitespace(char* s, int num_retained_spaces = 0);
    static void beautifyLine(char* s, int num_retained_spaces, int remove_whitespace = 1);

protected:
    FILE*              fin = nullptr;
    std::vector<FILE*> fileStack;

    std::string directoryName;

    char includeToken[96];
    int  includeTokenLength;
};

}  // namespace pgo::VolumetricMeshes::io::detail
