#pragma once

#include "types/element_set.h"

namespace pgo::VolumetricMeshes {

inline constexpr double E_default       = 1e9;
inline constexpr double nu_default      = 0.45;
inline constexpr double density_default = 1000.0;

inline constexpr const char* allElementsSetName = "allElements";

inline ElementSet generateAllElementsSet(int numElements) {
    ElementSet set(allElementsSetName);
    for (int element = 0; element < numElements; ++element) {
        set.insert(element);
    }
    return set;
}

}  // namespace pgo::VolumetricMeshes
