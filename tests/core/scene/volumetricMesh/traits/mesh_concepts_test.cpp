#include "concepts/mesh_concepts.h"

#include "cubicMesh.h"
#include "tetMesh.h"

#include <gtest/gtest.h>

namespace {

using pgo::VolumetricMeshes::CubicMesh;
using pgo::VolumetricMeshes::TetMesh;
using pgo::VolumetricMeshes::concepts::CubicMeshLike;
using pgo::VolumetricMeshes::concepts::TetMeshLike;
using pgo::VolumetricMeshes::concepts::VolumetricMeshLike;

static_assert(VolumetricMeshLike<TetMesh>);
static_assert(VolumetricMeshLike<CubicMesh>);
static_assert(TetMeshLike<TetMesh>);
static_assert(!TetMeshLike<CubicMesh>);
static_assert(CubicMeshLike<CubicMesh>);
static_assert(!CubicMeshLike<TetMesh>);

TEST(MeshConceptsTest, ConceptChecksCompileForSupportedMeshTypes) {
    SUCCEED();
}

}  // namespace
