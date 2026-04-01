#include "cubicMeshDeformationModel.h"

#include "cubicMesh.h"
#include "elasticModelStableNeoHookeanMaterial.h"
#include "finiteDifference.h"
#include "plasticModel3DConstant.h"
#include "pgoLogging.h"
#include "EigenSupport.h"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <memory>

namespace pgo::SolidDeformationModel::test {
namespace {

using pgo::NonlinearOptimization::FiniteDifference;
namespace ES = pgo::EigenSupport;

std::unique_ptr<VolumetricMeshes::CubicMesh> makeSingleCubicMesh(double E = 2345.0, double nu = 0.31,
                                                                 double density = 77.0) {
    std::array<int, 3> voxels = {0, 0, 0};
    return std::unique_ptr<VolumetricMeshes::CubicMesh>(
        VolumetricMeshes::CubicMesh::createFromUniformGrid(1, 1, voxels.data(), E, nu, density));
}

ES::V24d extractRestPositions(const VolumetricMeshes::CubicMesh& cubicMesh) {
    ES::V24d restPositions;
    for (int vi = 0; vi < 8; vi++) {
        const Vec3d& position = cubicMesh.getVertex(0, vi);
        restPositions.segment<3>(vi * 3) = ES::V3d(position[0], position[1], position[2]);
    }
    return restPositions;
}

void computeSvd(const ES::M3d& F, ES::M3d& U, ES::M3d& V, ES::V3d& S) {
    Eigen::JacobiSVD<ES::M3d, Eigen::NoQRPreconditioner> svd(F, Eigen::ComputeFullU | Eigen::ComputeFullV);
    U = svd.matrixU();
    V = svd.matrixV();
    S = svd.singularValues();

    if (U.determinant() < 0.0) {
        U.col(2) *= -1.0;
        S(2) *= -1.0;
    }
    if (V.determinant() < 0.0) {
        V.col(2) *= -1.0;
        S(2) *= -1.0;
    }
}

}  // namespace

TEST(CubicMeshDeformationModelTest, AffineDeformationMatchesMaterialEnergyDensityTimesVolume) {
    Logging::init();

    std::unique_ptr<VolumetricMeshes::CubicMesh> cubicMesh = makeSingleCubicMesh();
    const ES::V24d restPositions = extractRestPositions(*cubicMesh);

    ElasticModelStableNeoHookeanMaterial elasticModel(23.0, 41.0);
    const double identityFp[9] = {
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0,
    };
    PlasticModel3DConstant plasticModel(identityFp);
    CubicMeshDeformationModel model(restPositions.data(), &elasticModel, &plasticModel);

    ES::M3d affineF;
    affineF << 1.10, 0.05, -0.02,
               0.03, 0.95, 0.04,
              -0.01, 0.02, 1.07;
    const ES::V3d translation(0.2, -0.1, 0.15);

    ES::V24d x;
    for (int vi = 0; vi < 8; vi++) {
        x.segment<3>(vi * 3) = affineF * restPositions.segment<3>(vi * 3) + translation;
    }

    std::unique_ptr<DeformationModelCacheData> cache(model.allocateCacheData());
    model.prepareData(x.data(), nullptr, nullptr, cache.get());

    ES::M3d U, V;
    ES::V3d S;
    computeSvd(affineF, U, V, S);
    const double expectedEnergy =
        elasticModel.compute_psi(nullptr, affineF.data(), U.data(), V.data(), S.data()) *
        cubicMesh->getElementVolume(0);

    EXPECT_EQ(model.getNumVertices(), 8);
    EXPECT_EQ(model.getNumDOFs(), 24);
    EXPECT_EQ(model.getNumMaterialLocations(), 8);
    EXPECT_NEAR(model.computeEnergy(cache.get()), expectedEnergy, 1e-10);
}

TEST(CubicMeshDeformationModelTest, FiniteDifferenceMatchesGradientAndHessian) {
    Logging::init();

    std::unique_ptr<VolumetricMeshes::CubicMesh> cubicMesh = makeSingleCubicMesh();
    const ES::V24d restPositions = extractRestPositions(*cubicMesh);

    ElasticModelStableNeoHookeanMaterial elasticModel(17.0, 33.0);
    const double identityFp[9] = {
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0,
    };
    PlasticModel3DConstant plasticModel(identityFp);
    CubicMeshDeformationModel model(restPositions.data(), &elasticModel, &plasticModel);

    ES::V24d x = restPositions;
    const std::array<double, 24> offsets = {
        0.03, -0.01, 0.02,  -0.02, 0.01, -0.015, 0.01, 0.02,  -0.025, -0.01, -0.02, 0.015,
        0.02, -0.015, 0.01, -0.025, 0.015, 0.02, 0.015, -0.01, -0.02, 0.01,  0.025, -0.01,
    };
    for (int i = 0; i < 24; i++) {
        x[i] += offsets[i];
    }

    std::unique_ptr<DeformationModelCacheData> cache(model.allocateCacheData());
    FiniteDifference fd(FiniteDifference::M_FIVE_POINT, 1e-7);

    double gradError = 0.0;
    double hessError = 0.0;
    FiniteDifference::EvalFunc eval = [&](const double* xIn, double* E, double* grad, double* hess) {
        model.prepareData(xIn, nullptr, nullptr, cache.get());

        if (E) {
            *E = model.computeEnergy(cache.get());
        }

        if (grad) {
            model.compute_dE_dx(cache.get(), grad);
        }

        if (hess) {
            model.compute_d2E_dx2(cache.get(), hess);
        }
    };

    fd.testEnergy(eval, 24, true, true, -1.0, x.data(), &gradError, &hessError);

    EXPECT_LT(gradError, 1e-6);
    EXPECT_LT(hessError, 2e-5);
}

}  // namespace pgo::SolidDeformationModel::test
