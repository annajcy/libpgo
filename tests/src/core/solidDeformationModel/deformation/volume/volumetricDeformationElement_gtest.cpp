#include "gtest/gtest.h"

#include "material/elastic/elasticModelStableNeoHookeanMaterial.h"
#include "material/plastic/plasticModel3DConstant.h"
#include "material/plastic/plasticModel3D3DOF.h"

#include "formulations/shapeFunction/tetLinearShapeFunction.h"
#include "formulations/shapeFunction/cubicLinearShapeFunction.h"
#include "formulations/quadrature/tetLinearDefaultQuadrature.h"
#include "formulations/quadrature/gaussLegendreHexQuadrature.h"
#include "deformation/volume/volumetricElementMapping.h"
#include "deformation/volume/volumetricDeformationElement.h"
#include "EigenSupport.h"

#include <memory>
#include <cmath>
#include <span>

namespace ES = pgo::EigenSupport;
using namespace pgo::SolidDeformationModel;

namespace
{
template<typename Derived>
std::span<const double> constSpan(const Eigen::MatrixBase<Derived> &values)
{
  return std::span<const double>(values.derived().data(),
    static_cast<size_t>(values.size()));
}

template<typename Derived>
std::span<double> mutableSpan(Eigen::MatrixBase<Derived> &values)
{
  return std::span<double>(values.derived().data(),
    static_cast<size_t>(values.size()));
}

const double restTet[12] = {
  0.0,
  0.0,
  0.0,
  2.0,
  0.0,
  0.0,
  0.0,
  3.0,
  0.0,
  0.0,
  0.0,
  4.0,
};

const double restHex[24] = {
  0.0,
  0.0,
  0.0,
  1.5,
  0.0,
  0.0,
  1.5,
  2.0,
  0.0,
  0.0,
  2.0,
  0.0,
  0.0,
  0.0,
  3.0,
  1.5,
  0.0,
  3.0,
  1.5,
  2.0,
  3.0,
  0.0,
  2.0,
  3.0,
};
}  // namespace

// ============================================================
// Tet: energy is finite at rest and gradient FD check
// ============================================================

TEST(VolumetricDeformationElementGTest, TetEnergyFiniteAtRest)
{
  auto elasticModel = std::make_unique<ElasticModelStableNeoHookeanMaterial>(1200.0, 1800.0);
  auto plasticModel = std::make_unique<PlasticModel3DConstant>(ES::M3d::Identity());
  VolumetricElementMapping mapping(restTet, TetLinearShapeFunction{}, TetLinearDefaultQuadrature{});
  VolumetricDeformationElement model(std::move(mapping), std::move(elasticModel), std::move(plasticModel));

  ES::V12d xVec;
  for (int i = 0; i < 12; i++)
    xVec[i] = restTet[i];

  double energy = model.computeEnergy(
    constSpan(xVec), std::span<const double>{}, std::span<const double>{});
  EXPECT_TRUE(std::isfinite(energy));

  ES::V12d grad;
  model.computeDisplacementGradient(
    constSpan(xVec), {}, {}, grad);
  for (int i = 0; i < 12; i++)
    EXPECT_TRUE(std::isfinite(grad[i]));

  ES::M12d hess;
  model.computeDisplacementHessian(
    constSpan(xVec), {}, {}, hess);
  for (int i = 0; i < 144; i++)
    EXPECT_TRUE(std::isfinite(hess.data()[i]));
}

TEST(VolumetricDeformationElementGTest, ParameterizedModelRejectsMissingParameters)
{
  auto elasticModel =
    std::make_unique<ElasticModelStableNeoHookeanMaterial>(1200.0, 1800.0);
  auto plasticModel =
    std::make_unique<PlasticModel3D3DOF>(ES::M3d::Identity());
  VolumetricElementMapping mapping(
    restTet, TetLinearShapeFunction{}, TetLinearDefaultQuadrature{});
  VolumetricDeformationElement model(
    std::move(mapping), std::move(elasticModel), std::move(plasticModel));

  ES::V12d xVec;
  for (int i = 0; i < 12; i++)
    xVec[i] = restTet[i];
  EXPECT_THROW(
    model.computeEnergy(constSpan(xVec), {}, {}),
    std::invalid_argument);

  const ES::V3d plasticParams = ES::V3d::Ones();
  EXPECT_NO_THROW(
    model.computeEnergy(constSpan(xVec), {}, constSpan(plasticParams)));
}

// ============================================================
// Hex: energy is finite at rest and gradient FD check
// ============================================================

TEST(VolumetricDeformationElementGTest, HexEnergyFiniteAtRest)
{
  auto elasticModel = std::make_unique<ElasticModelStableNeoHookeanMaterial>(1200.0, 1800.0);
  auto plasticModel = std::make_unique<PlasticModel3DConstant>(ES::M3d::Identity());
  VolumetricElementMapping mapping(restHex, CubicLinearShapeFunction{}, GaussLegendreHexQuadrature2{});
  VolumetricDeformationElement model(std::move(mapping), std::move(elasticModel), std::move(plasticModel));

  ES::V24d xVec;
  for (int i = 0; i < 24; i++)
    xVec[i] = restHex[i];

  double energy = model.computeEnergy(constSpan(xVec), {}, {});
  EXPECT_TRUE(std::isfinite(energy));

  ES::V24d grad;
  model.computeDisplacementGradient(constSpan(xVec), {}, {}, grad);
  for (int i = 0; i < 24; i++)
    EXPECT_TRUE(std::isfinite(grad[i]));

  ES::M24d hess;
  model.computeDisplacementHessian(constSpan(xVec), {}, {}, hess);
  for (int i = 0; i < 576; i++)
    EXPECT_TRUE(std::isfinite(hess.data()[i]));
}

// ============================================================
// Tet: gradient matches finite difference of energy
// ============================================================

TEST(VolumetricDeformationElementGTest, TetGradientMatchesFD)
{
  auto elasticModel = std::make_unique<ElasticModelStableNeoHookeanMaterial>(1200.0, 1800.0);
  auto plasticModel = std::make_unique<PlasticModel3DConstant>(ES::M3d::Identity());
  VolumetricElementMapping mapping(restTet, TetLinearShapeFunction{}, TetLinearDefaultQuadrature{});
  VolumetricDeformationElement model(std::move(mapping), std::move(elasticModel), std::move(plasticModel));

  ES::V12d xVec;
  for (int i = 0; i < 12; i++)
    xVec[i] = restTet[i] + 0.01 * std::sin(0.7 * static_cast<double>(i));

  ES::V12d grad;
  model.computeDisplacementGradient(constSpan(xVec), {}, {}, grad);

  const double eps = 1e-6;
  for (int i = 0; i < 12; i++) {
    ES::V12d xp = xVec, xm = xVec;
    xp[i] += eps;
    xm[i] -= eps;

    double ep = model.computeEnergy(constSpan(xp), {}, {});
    double em = model.computeEnergy(constSpan(xm), {}, {});

    double fd = (ep - em) / (2.0 * eps);
    EXPECT_NEAR(grad[i], fd, 1e-5) << "grad[" << i << "]";
  }
}

// ============================================================
// Hex: gradient matches finite difference of energy
// ============================================================

TEST(VolumetricDeformationElementGTest, HexGradientMatchesFD)
{
  auto elasticModel = std::make_unique<ElasticModelStableNeoHookeanMaterial>(1200.0, 1800.0);
  auto plasticModel = std::make_unique<PlasticModel3DConstant>(ES::M3d::Identity());
  VolumetricElementMapping mapping(restHex, CubicLinearShapeFunction{}, GaussLegendreHexQuadrature2{});
  VolumetricDeformationElement model(std::move(mapping), std::move(elasticModel), std::move(plasticModel));

  ES::V24d xVec;
  for (int i = 0; i < 24; i++)
    xVec[i] = restHex[i] + 0.01 * std::sin(0.7 * static_cast<double>(i));

  ES::V24d grad;
  model.computeDisplacementGradient(constSpan(xVec), {}, {}, grad);

  const double eps = 1e-6;
  for (int i = 0; i < 24; i++) {
    ES::V24d xp = xVec, xm = xVec;
    xp[i] += eps;
    xm[i] -= eps;

    double ep = model.computeEnergy(constSpan(xp), {}, {});
    double em = model.computeEnergy(constSpan(xm), {}, {});

    double fd = (ep - em) / (2.0 * eps);
    EXPECT_NEAR(grad[i], fd, 1e-5) << "grad[" << i << "]";
  }
}
