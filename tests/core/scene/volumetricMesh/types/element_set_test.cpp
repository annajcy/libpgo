#include "types/element_set.h"

#include <gtest/gtest.h>

#include <set>

namespace {

TEST(ElementSetTest, TracksMembershipAndCopyBehavior) {
    pgo::VolumetricMeshes::ElementSet set("selected", std::set<int>{1, 3});

    EXPECT_EQ(set.getName(), "selected");
    EXPECT_EQ(set.getNumElements(), 2);
    EXPECT_TRUE(set.isMember(1));
    EXPECT_TRUE(set.isMember(3));
    EXPECT_FALSE(set.isMember(2));

    pgo::VolumetricMeshes::ElementSet copied = set;
    copied.insert(5);

    EXPECT_EQ(copied.getNumElements(), 3);
    EXPECT_TRUE(copied.isMember(5));
    EXPECT_FALSE(set.isMember(5));

    copied.clear();
    EXPECT_EQ(copied.getNumElements(), 0);
}

}  // namespace
