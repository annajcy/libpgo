#include "types/material_region.h"

#include <gtest/gtest.h>

namespace {

TEST(MaterialRegionTest, StoresSetAndMaterialIndices) {
    pgo::VolumetricMeshes::MaterialRegion region(2, 5);

    EXPECT_EQ(region.getMaterialIndex(), 2);
    EXPECT_EQ(region.getSetIndex(), 5);

    region.setMaterialIndex(7);
    region.setSetIndex(9);

    EXPECT_EQ(region.getMaterialIndex(), 7);
    EXPECT_EQ(region.getSetIndex(), 9);
}

}  // namespace
