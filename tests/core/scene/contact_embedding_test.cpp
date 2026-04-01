#include "barycentricCoordinates.h"
#include "cubicMesh.h"
#include "tetMesh.h"
#include "triangleMeshExternalContactHandler.h"
#include "triangleMeshSelfContactHandler.h"
#include "triMeshGeo.h"
#include "pgoLogging.h"

#include <gtest/gtest.h>

#include <filesystem>
#include <vector>

namespace pgo::Contact::test {

namespace {

std::filesystem::path repoRootPath() {
    return std::filesystem::absolute(std::filesystem::path(__FILE__))
        .parent_path()
        .parent_path()
        .parent_path()
        .parent_path();
}

EigenSupport::VXd makeSurfaceRestPositions(const Mesh::TriMeshGeo& surfaceMesh) {
    EigenSupport::VXd surfaceRestPositions(surfaceMesh.numVertices() * 3);
    for (int vi = 0; vi < surfaceMesh.numVertices(); ++vi) {
        surfaceRestPositions.segment<3>(vi * 3) = surfaceMesh.pos(vi);
    }

    return surfaceRestPositions;
}

double maxAbsDenseDifference(const EigenSupport::SpMatD& lhs, const EigenSupport::SpMatD& rhs) {
    EXPECT_EQ(lhs.rows(), rhs.rows());
    EXPECT_EQ(lhs.cols(), rhs.cols());

    const Eigen::MatrixXd lhsDense(lhs);
    const Eigen::MatrixXd rhsDense(rhs);
    return (lhsDense - rhsDense).cwiseAbs().maxCoeff();
}

template <class MeshType>
InterpolationCoordinates::BarycentricCoordinates makeBarycentricCoordinates(const Mesh::TriMeshGeo& surfaceMesh,
                                                                            const MeshType& mesh) {
    const EigenSupport::VXd surfaceRestPositions = makeSurfaceRestPositions(surfaceMesh);
    return {surfaceMesh.numVertices(), surfaceRestPositions.data(), &mesh};
}

template <class MeshType>
void expectExternalEmbeddingMatchesBarycentric(const std::filesystem::path& meshPath,
                                               const std::filesystem::path& surfacePath) {
    Logging::init();

    MeshType         mesh(meshPath.string().c_str());
    Mesh::TriMeshGeo surfaceMesh;
    ASSERT_TRUE(surfaceMesh.load(surfacePath.string()));

    auto bc = makeBarycentricCoordinates(surfaceMesh, mesh);
    const EigenSupport::SpMatD expected = bc.generateInterpolationMatrix();

    const std::vector<Mesh::TriMeshRef> externalSurfaces;
    Contact::TriangleMeshExternalContactHandler handler(surfaceMesh.positions(), surfaceMesh.triangles(),
                                                        mesh.getNumVertices() * 3, externalSurfaces, 1,
                                                        &bc.getEmbeddingVertexIndices(), &bc.getEmbeddingWeights());

    const EigenSupport::SpMatD& actual = handler.getSampleEmbeddingMatrix();
    ASSERT_EQ(actual.rows(), expected.rows());
    ASSERT_EQ(actual.cols(), expected.cols());
    EXPECT_NEAR(maxAbsDenseDifference(actual, expected), 0.0, 1e-12);
}

void expectSelfEmbeddingMatchesBarycentric(const std::filesystem::path& meshPath,
                                           const std::filesystem::path& surfacePath) {
    Logging::init();

    VolumetricMeshes::CubicMesh mesh(meshPath.string().c_str());
    Mesh::TriMeshGeo            surfaceMesh;
    ASSERT_TRUE(surfaceMesh.load(surfacePath.string()));

    auto bc = makeBarycentricCoordinates(surfaceMesh, mesh);
    const EigenSupport::SpMatD expected = bc.generateInterpolationMatrix();

    Contact::TriangleMeshSelfContactHandler handler(surfaceMesh.positions(), surfaceMesh.triangles(),
                                                    mesh.getNumVertices() * 3, 1, &bc.getEmbeddingVertexIndices(),
                                                    &bc.getEmbeddingWeights());

    const EigenSupport::SpMatD& actual = handler.getSampleEmbeddingMatrix();
    ASSERT_EQ(actual.rows(), expected.rows());
    ASSERT_EQ(actual.cols(), expected.cols());
    EXPECT_NEAR(maxAbsDenseDifference(actual, expected), 0.0, 1e-12);
}

}  // namespace

TEST(ContactEmbeddingTest, CubicExternalMatchesBarycentricEmbedding) {
    const std::filesystem::path repoRoot = repoRootPath();
    expectExternalEmbeddingMatchesBarycentric<VolumetricMeshes::CubicMesh>(
        repoRoot / "examples" / "cubic-box" / "cubic-box.veg",
        repoRoot / "examples" / "cubic-box" / "cubic-box.obj");
}

TEST(ContactEmbeddingTest, CubicSelfMatchesBarycentricEmbedding) {
    const std::filesystem::path repoRoot = repoRootPath();
    expectSelfEmbeddingMatchesBarycentric(repoRoot / "examples" / "cubic-box" / "cubic-box.veg",
                                          repoRoot / "examples" / "cubic-box" / "cubic-box.obj");
}

TEST(ContactEmbeddingTest, TetExternalMatchesBarycentricEmbedding) {
    const std::filesystem::path repoRoot = repoRootPath();
    expectExternalEmbeddingMatchesBarycentric<VolumetricMeshes::TetMesh>(
        repoRoot / "examples" / "box" / "box.veg", repoRoot / "examples" / "box" / "box.obj");
}

TEST(ContactEmbeddingTest, RejectsMalformedEmbeddingLayout) {
    Logging::init();

    const std::filesystem::path repoRoot = repoRootPath();
    VolumetricMeshes::CubicMesh mesh((repoRoot / "examples" / "cubic-box" / "cubic-box.veg").string().c_str());
    Mesh::TriMeshGeo            surfaceMesh;
    ASSERT_TRUE(surfaceMesh.load((repoRoot / "examples" / "cubic-box" / "cubic-box.obj").string()));

    auto bc = makeBarycentricCoordinates(surfaceMesh, mesh);

    std::vector<int>    badIndices = bc.getEmbeddingVertexIndices();
    std::vector<double> weights    = bc.getEmbeddingWeights();
    badIndices.pop_back();

    const std::vector<Mesh::TriMeshRef> externalSurfaces;
    EXPECT_THROW((void)Contact::TriangleMeshExternalContactHandler(surfaceMesh.positions(), surfaceMesh.triangles(),
                                                                   mesh.getNumVertices() * 3, externalSurfaces, 1,
                                                                   &badIndices, &weights),
                 std::runtime_error);
}

}  // namespace pgo::Contact::test
