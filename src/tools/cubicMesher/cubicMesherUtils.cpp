#include "cubicMesherUtils.h"

#include <cstdint>
#include <limits>

namespace cubic_mesher {

bool computeNumVoxels(int resolution, int& numVoxels) {
    if (resolution <= 0) {
        return false;
    }

    const int64_t n   = static_cast<int64_t>(resolution);
    const int64_t num = n * n * n;
    if (num > static_cast<int64_t>(std::numeric_limits<int>::max())) {
        return false;
    }

    numVoxels = static_cast<int>(num);
    return true;
}

}  // namespace cubic_mesher
