#include "pgo_c.h"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <vector>

namespace test {

namespace {
std::array<double, 12> unitTetVertices() {
    return {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
}

std::array<int, 4> unitTetIndices() {
    return {0, 1, 2, 3};
}
}  // namespace

TEST(PgoCApiTest, CreateTetMeshGeoAndReadBackBuffers) {
    pgo_init();

    auto vertices = unitTetVertices();
    auto tets     = unitTetIndices();

    pgoTetMeshGeoStructHandle tetmesh = pgo_create_tetmeshgeo(4, vertices.data(), 1, tets.data());
    ASSERT_NE(tetmesh, nullptr);

    EXPECT_EQ(pgo_tetmeshgeo_get_num_vertices(tetmesh), 4);
    EXPECT_EQ(pgo_tetmeshgeo_get_num_tets(tetmesh), 1);

    std::array<double, 12> outVertices{};
    pgo_tetmeshgeo_get_vertices(tetmesh, outVertices.data());
    for (size_t i = 0; i < outVertices.size(); i++) {
        EXPECT_NEAR(outVertices[i], vertices[i], 1e-12);
    }

    std::array<int, 4> outTets{};
    pgo_tetmeshgeo_get_tets(tetmesh, outTets.data());
    EXPECT_EQ(outTets, tets);

    pgo_destroy_tetmeshgeo(tetmesh);
}

TEST(PgoCApiTest, TetLaplacianMatrixCreationAndSparseExport) {
    pgo_init();

    auto vertices = unitTetVertices();
    auto tets     = unitTetIndices();

    pgoTetMeshGeoStructHandle tetmesh = pgo_create_tetmeshgeo(4, vertices.data(), 1, tets.data());
    ASSERT_NE(tetmesh, nullptr);

    pgoSparseMatrixStructHandle L = pgo_create_tet_laplacian_matrix(tetmesh, 0, 1, 0);
    ASSERT_NE(L, nullptr);

    const int64_t nnz = pgo_sparse_matrix_get_num_entries(L);
    EXPECT_GT(nnz, 0);

    std::vector<int>    rows(static_cast<size_t>(nnz));
    std::vector<int>    cols(static_cast<size_t>(nnz));
    std::vector<double> values(static_cast<size_t>(nnz));

    pgo_sparse_matrix_get_row_indices(L, rows.data());
    pgo_sparse_matrix_get_col_indices(L, cols.data());
    pgo_sparse_matrix_get_values(L, values.data());

    bool allFinite     = true;
    bool hasValidIndex = false;
    for (size_t i = 0; i < values.size(); i++) {
        allFinite     = allFinite && std::isfinite(values[i]);
        hasValidIndex = hasValidIndex || (rows[i] >= 0 && cols[i] >= 0);
    }
    EXPECT_TRUE(allFinite);
    EXPECT_TRUE(hasValidIndex);

    pgo_destroy_sparse_matrix(L);
    pgo_destroy_tetmeshgeo(tetmesh);
}

}  // namespace test