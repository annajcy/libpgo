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

#include "io/detail/veg_parser.h"

#include "fileIO.h"

#include <cstdio>
#include <cctype>
#include <cstring>

namespace pgo::VolumetricMeshes::io::detail {

VolumetricMeshParser::VolumetricMeshParser(const char* include_token) {
    while (!fileStack.empty()) {
        fileStack.pop_back();
    }

    if (include_token == nullptr) {
        includeTokenLength = 9;
        strcpy(includeToken, "*INCLUDE ");
    } else {
        includeTokenLength = strlen(include_token);
        strcpy(includeToken, include_token);
    }
}

int VolumetricMeshParser::open(const char* filename) {
    directoryName = BasicAlgorithms::getPathDirectoryName(filename);

    fin = fopen(filename, "r");
    if (!fin) {
        return 1;
    }

    fileStack.push_back(fin);
    return 0;
}

VolumetricMeshParser::~VolumetricMeshParser() {
    close();
}

void VolumetricMeshParser::rewindToStart() {
    if (fileStack.empty()) {
        return;
    }

    while (fileStack.size() > 1) {
        fclose(fileStack.back());
        fileStack.pop_back();
    }

    fin = fileStack[0];
    rewind(fin);
}

void VolumetricMeshParser::close() {
    while (!fileStack.empty()) {
        fclose(fileStack.back());
        fileStack.pop_back();
    }
}

void VolumetricMeshParser::beautifyLine(char* s, int num_retained_spaces, int remove_whitespace) {
    if (remove_whitespace) {
        removeWhitespace(s, num_retained_spaces);
    }

    char* pos = s;
    while (*pos != '\0') {
        pos++;
    }

    if (pos != s) {
        for (pos--; pos != s && std::isspace(*pos); *pos = '\0') {
        }
    }
}

char* VolumetricMeshParser::getNextLine(char* s, int num_retained_spaces, int remove_whitespace) {
    char* code;
    do {
        while (((code = fgets(s, 1023, fin)) == nullptr) && (fileStack.size() > 1)) {
            fclose(fin);
            fileStack.pop_back();
            fin = fileStack.back();
        }

        if (code == nullptr) {
            return nullptr;
        }
    } while ((s[0] == '#') || (s[0] == 13) || (s[0] == 10));

    beautifyLine(s, num_retained_spaces, remove_whitespace);

    while (strncmp(s, includeToken, includeTokenLength) == 0) {
        std::string new_file = &(s[includeTokenLength]);
        std::string new_file_complete_name = directoryName + "/" + new_file;
        FILE* new_fin = fopen(new_file_complete_name.data(), "r");

        if (!new_fin) {
            printf("Error: couldn't open include file %s.\n", new_file_complete_name.c_str());
            close();
            throw -1;
        }

        if ((code = fgets(s, 1023, new_fin)) != nullptr) {
            beautifyLine(s, num_retained_spaces, remove_whitespace);
            fileStack.push_back(new_fin);
            fin = new_fin;
        } else {
            fclose(new_fin);
            printf("Warning: include file is empty.\n");
            code = getNextLine(s, num_retained_spaces);
            beautifyLine(s, num_retained_spaces, remove_whitespace);
            return code;
        }
    }

    return code;
}

void VolumetricMeshParser::upperCase(char* s) {
    char case_difference = 'A' - 'a';
    for (unsigned int i = 0; i < strlen(s); i++) {
        if ((s[i] >= 'a') && (s[i] <= 'z')) {
            s[i] += case_difference;
        }
    }
}

void VolumetricMeshParser::removeWhitespace(char* s, int num_retained_spaces) {
    char* p = s;
    while (*p != 0) {
        while (1) {
            bool erase_character = (*p == ' ');
            for (int i = 1; i <= num_retained_spaces; i++) {
                if (*(p + i) == 0) {
                    erase_character = false;
                    break;
                }
                erase_character =
                    erase_character && ((p == s) || (*(p + i) == ' '));
            }

            if (!erase_character) {
                break;
            }

            char* q = p;
            while (*q != 0) {
                *q = *(q + 1);
                q++;
            }
        }
        p++;
    }
}

}  // namespace pgo::VolumetricMeshes::io::detail
