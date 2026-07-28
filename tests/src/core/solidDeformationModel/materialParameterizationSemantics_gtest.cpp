#include <gtest/gtest.h>

#include "energy/deformationModelEnergy.h"
#include "energy/elasticMaterialEnergy.h"
#include "formulations/formulation/formulations.h"
#include "material/parameterization/materialChannelMapping.h"
#include "material/projection/materialInputProjection.h"
#include "material/parameterization/materialParameterization.h"
#include "material/runtime/optimizableParameterRef.h"
#include "material/runtime/optimizableParameters.h"
#include "material/elastic/elasticModel3DDeformationGradient.h"
#include "material/plastic/plasticModel3DConstant.h"
#include "materialTestUtils.h"
#include "simulation/simulationMesh.h"

#include <array>
#include <cmath>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace
{
namespace ES = pgo::EigenSupport;
using namespace pgo::SolidDeformationModel;

/// Target semantic contract:
///   input  z[0]     is named "logE" by OptimizableParameterField;
///   output theta[0] is named "E" by ElasticModelDefinition.
class LogYoungsModulusEvaluator final : public DifferentiableMaterialChannelMapping
{
public:
  int numInputs() const override { return 1; }
  int numChannels() const override { return 1; }
  bool isAffine() const override { return false; }

  void evaluate(
    int, int, std::span<const double> inputs,
    std::span<double> channels) const override
  {
    if (inputs.size() != 1 || channels.size() != 1)
      throw std::invalid_argument(
        "LogYoungsModulusEvaluator expects one parameter and one channel.");
    channels[0] = std::exp(inputs[0]);
  }

  void evaluateJacobian(
    int, int, std::span<const double> inputs,
    ES::RefMatXd jacobian) const override
  {
    if (inputs.size() != 1 || jacobian.rows() != 1 || jacobian.cols() != 1)
      throw std::invalid_argument(
        "LogYoungsModulusEvaluator Jacobian shape mismatch.");
    jacobian(0, 0) = std::exp(inputs[0]);
  }

  void evaluateHessians(
    int, int, std::span<const double> inputs,
    std::span<ES::MXd> channelHessians) const override
  {
    if (inputs.size() != 1 || channelHessians.size() != 1 ||
      channelHessians[0].rows() != 1 || channelHessians[0].cols() != 1)
      throw std::invalid_argument(
        "LogYoungsModulusEvaluator Hessian shape mismatch.");
    channelHessians[0](0, 0) = std::exp(inputs[0]);
  }
};

/// A deliberately small constitutive model whose sole physical parameter is
/// Young's modulus E. It makes the material-parameter derivative chain easy to
/// isolate while still exercising the real deformation assembler.
class YoungsModulusTestModel final : public ElasticModel3DDeformationGradient
{
public:
  int getNumParameters() const override { return 1; }

  double compute_psi(
    std::span<const double> parameters,
    const SpectralState &state) const override
  {
    return 0.5 * parameters[0] *
      (state.F - ES::M3d::Identity()).squaredNorm();
  }

  ES::M3d compute_P(
    std::span<const double> parameters,
    const SpectralState &state) const override
  {
    return parameters[0] * (state.F - ES::M3d::Identity());
  }

  ES::M9d compute_dPdF(
    std::span<const double> parameters,
    const SpectralState &) const override
  {
    return parameters[0] * ES::M9d::Identity();
  }

  void compute_dpsi_dparams(
    std::span<const double> parameters,
    const SpectralState &state,
    ES::RefVecXd derivative) const override
  {
    if (parameters.size() != 1 || derivative.size() != 1)
      throw std::invalid_argument(
        "YoungsModulusTestModel parameter-gradient dimensions.");
    derivative[0] =
      0.5 * (state.F - ES::M3d::Identity()).squaredNorm();
  }

  void compute_d2psi_dparams2(
    std::span<const double> parameters,
    const SpectralState &,
    ES::RefMatXd derivative) const override
  {
    if (parameters.size() != 1 ||
      derivative.rows() != 1 || derivative.cols() != 1)
      throw std::invalid_argument(
        "YoungsModulusTestModel parameter-Hessian dimensions.");
    derivative.setZero();
  }

  void compute_dP_dparams(
    std::span<const double> parameters,
    const SpectralState &state,
    ES::RefMatXd derivative) const override
  {
    if (parameters.size() != 1 ||
      derivative.rows() != 9 || derivative.cols() != 1)
      throw std::invalid_argument(
        "YoungsModulusTestModel P-Jacobian dimensions.");
    const ES::M3d dP = state.F - ES::M3d::Identity();
    derivative.col(0) = Eigen::Map<const ES::V9d>(dP.data());
  }

  void compute_d2PdF2(
    std::span<const double>, const SpectralState &,
    ES::M81x9d &derivative) const override
  {
    derivative.setZero();
  }

  ES::M3d compute_d2Pdparam2(
    std::span<const double>, int parameter0, int parameter1,
    const SpectralState &) const override
  {
    if (parameter0 != 0 || parameter1 != 0)
      throw std::out_of_range("YoungsModulusTestModel parameter index.");
    return ES::M3d::Zero();
  }

  ES::M9d compute_d2PdFdparam(
    std::span<const double>, int parameter,
    const SpectralState &) const override
  {
    if (parameter != 0)
      throw std::out_of_range("YoungsModulusTestModel parameter index.");
    return ES::M9d::Identity();
  }
};

class YoungsModulusTestDefinition final : public ElasticModelDefinition
{
public:
  std::string_view id() const override { return "test_youngs_modulus"; }

  MaterialChannelSchema fixedChannelSchema() const override
  {
    return {};
  }

  MaterialChannelSchema optimizableChannelSchema() const override
  {
    static constexpr std::array<std::string_view, 1> names{ "E" };
    return MaterialChannelSchema(names);
  }

  std::unique_ptr<ElasticModel> createModel(
    std::span<const double> fixed,
    const MaterialFrame &) const override
  {
    if (!fixed.empty())
      throw std::invalid_argument(
        "YoungsModulusTestDefinition has no fixed channels.");
    return std::make_unique<YoungsModulusTestModel>();
  }
};

std::shared_ptr<const SimulationMesh> makeOneElementMesh()
{
  static constexpr std::array<double, 24> vertices{
    0,
    0,
    0,
    1,
    0,
    0,
    1,
    1,
    0,
    0,
    1,
    0,
    0,
    0,
    1,
    1,
    0,
    1,
    1,
    1,
    1,
    0,
    1,
    1,
  };
  static constexpr std::array<int, 8> elementVertices{
    0, 1, 2, 3, 4, 5, 6, 7
  };
  return std::shared_ptr<const SimulationMesh>(new SimulationMesh(
    8, std::span<const double>(vertices),
    1, 8, std::span<const int>(elementVertices),
    SimulationMeshType::CUBIC));
}

std::shared_ptr<const MaterialParameterization> makeLogEParameterization()
{
  constexpr int numElements = 1;
  auto elasticDefinition =
    std::make_shared<const YoungsModulusTestDefinition>();
  auto plasticDefinition =
    std::make_shared<const VolumetricPlasticity0Definition>();

  auto elasticOptimizable = std::make_shared<const OptimizableParameterField>(
    ParameterInputSchema(std::vector<std::string>{ "logE" }),
    std::make_shared<const ConstantParameterLayout>(numElements, 1),
    std::make_shared<const LogYoungsModulusEvaluator>());
  auto plasticOptimizable = std::make_shared<const OptimizableParameterField>(
    ParameterInputSchema{},
    std::make_shared<const ElementwiseParameterLayout>(numElements, 0),
    std::make_shared<const IdentityMaterialChannelMapping>(0));

  return std::make_shared<const MaterialParameterization>(
    ElasticParameterization(
      std::move(elasticDefinition),
      TestUtils::emptyFixedField(numElements),
      std::move(elasticOptimizable)),
    PlasticParameterization(
      std::move(plasticDefinition),
      TestUtils::emptyFixedField(numElements),
      std::move(plasticOptimizable)));
}

TEST(LogYoungsModulusEvaluator, ValueJacobianAndHessianMatchFiniteDifferences)
{
  LogYoungsModulusEvaluator evaluator;
  const std::array<double, 1> input{ std::log(1200.0) };
  std::array<double, 1> output{};
  ES::MXd jacobian(1, 1);
  std::vector<ES::MXd> outputHessians(1, ES::MXd(1, 1));

  evaluator.evaluate(0, 0, input, output);
  evaluator.evaluateJacobian(0, 0, input, jacobian);
  evaluator.evaluateHessians(0, 0, input, outputHessians);

  EXPECT_NEAR(output[0], 1200.0, 1e-12);

  constexpr double step = 1e-5;
  const std::array<double, 1> plusInput{ input[0] + step };
  const std::array<double, 1> minusInput{ input[0] - step };
  std::array<double, 1> plusOutput{}, minusOutput{};
  ES::MXd plusJacobian(1, 1), minusJacobian(1, 1);
  evaluator.evaluate(0, 0, plusInput, plusOutput);
  evaluator.evaluate(0, 0, minusInput, minusOutput);
  evaluator.evaluateJacobian(0, 0, plusInput, plusJacobian);
  evaluator.evaluateJacobian(0, 0, minusInput, minusJacobian);

  const double finiteDifferenceJacobian =
    (plusOutput[0] - minusOutput[0]) / (2.0 * step);
  const double finiteDifferenceHessian =
    (plusJacobian(0, 0) - minusJacobian(0, 0)) / (2.0 * step);
  EXPECT_NEAR(jacobian(0, 0), finiteDifferenceJacobian, 1e-7);
  EXPECT_NEAR(
    outputHessians[0](0, 0), finiteDifferenceHessian, 1e-7);
}

TEST(MaterialParameterizationSemanticContract, PhysicalChannelRefUsesMappingDerivatives)
{
  const auto parameterization = makeLogEParameterization();
  const double logE = std::log(1200.0);
  OptimizableParameters parameters(
    parameterization->elastic().optimizableField(),
    parameterization->plastic().optimizableField(),
    ES::VXd::Constant(1, logE), ES::VXd{});
  const auto state = parameters.snapshot().view();
  const OptimizableMaterialChannelRef channel(
    parameterization->elastic(), "E");

  OptimizableParameterEvaluationScratch scratch;
  EXPECT_NEAR(channel.value(0, 0, state, scratch), 1200.0, 1e-12);

  ES::VXd derivative(1);
  channel.localDerivative(0, 0, state, scratch, derivative);
  EXPECT_NEAR(derivative[0], 1200.0, 1e-12);

  ES::MXd hessian(1, 1);
  channel.localHessian(0, 0, state, hessian);
  EXPECT_NEAR(hessian(0, 0), 1200.0, 1e-12);
}

TEST(MaterialParameterizationSemanticContract, AllowsDistinctParametersAndChannels)
{
  const auto parameterization = makeLogEParameterization();
  ASSERT_NE(parameterization, nullptr);

  const auto inputNames =
    parameterization->elastic().optimizableField()->inputSchema().parameterNames();
  const auto outputSchema =
    parameterization->elastic().optimizableChannelSchema();
  const auto outputNames = outputSchema.channelNames();
  ASSERT_EQ(inputNames.size(), 1);
  ASSERT_EQ(outputNames.size(), 1);
  EXPECT_EQ(inputNames[0], "logE");
  EXPECT_EQ(outputNames[0], "E");
  std::string channelName = "E";
  const OptimizableMaterialChannelRef channel(
    parameterization->elastic(), channelName);
  channelName = "mutated";
  EXPECT_EQ(channel.name(), "E");
  EXPECT_EQ(channel.channelIndex(), 0);
}

TEST(MaterialParameterizationSemanticContract, NamedProjectionInitializesParametersNotChannels)
{
  const auto mesh = makeOneElementMesh();
  const auto parameterization = makeLogEParameterization();
  const auto &field = *parameterization->elastic().optimizableField();

  const auto physicalAsset = TestUtils::makeAsset(
    mesh, { "E" }, { 1200.0 });
  EXPECT_THROW(
    projectImportedMaterialInputs(
      physicalAsset->materialCatalog(),
      field.inputSchema(), field.layout()),
    std::invalid_argument);

  const double logE = std::log(1200.0);
  const auto inputAsset = TestUtils::makeAsset(
    mesh, { "logE" }, { logE });
  const ES::VXd values = projectImportedMaterialInputs(
    inputAsset->materialCatalog(), field.inputSchema(), field.layout());
  ASSERT_EQ(values.size(), 1);
  EXPECT_NEAR(values[0], logE, 1e-12);
}

TEST(MaterialParameterizationSemanticContract, NamedFieldProjectionComposesWithSuppliedInitialValues)
{
  const auto mesh = makeOneElementMesh();
  ES::MXd rows(1, 1);
  rows(0, 0) = 1200.0;
  const NamedMaterialInputData inputs(
    1, {NamedMaterialInputField({"E"}, std::move(rows), {0}, "manual")});
  const ConstantParameterLayout layout(1, 1);

  MaterialParameterData data;
  data.elastic.fixedValues = projectNamedMaterialInputs(
    inputs, ParameterInputSchema({ "E" }), layout);
  data.elastic.initialOptimizableValues = ES::VXd::Constant(
    1, std::log(1200.0));

  ASSERT_EQ(data.elastic.fixedValues.size(), 1);
  EXPECT_DOUBLE_EQ(data.elastic.fixedValues[0], 1200.0);
  EXPECT_NEAR(
    data.elastic.initialOptimizableValues[0], std::log(1200.0), 1e-12);
}

TEST(MaterialParameterizationSemanticContract, LogEGradientAndHessianMatchFiniteDifferences)
{
  const auto mesh = makeOneElementMesh();
  const auto parameterization = makeLogEParameterization();
  const double logE = std::log(1200.0);
  const auto inputAsset = TestUtils::makeAsset(
    mesh, { "logE" }, { logE });
  MaterialParameterData projected;
  const auto &field = *parameterization->elastic().optimizableField();
  projected.elastic.fixedValues = ES::VXd{};
  projected.elastic.initialOptimizableValues =
    projectImportedMaterialInputs(
      inputAsset->materialCatalog(), field.inputSchema(), field.layout());
  projected.plastic.fixedValues = ES::VXd{};
  projected.plastic.initialOptimizableValues = ES::VXd{};
  parameterization->validate(projected);
  auto data = std::make_shared<const MaterialParameterData>(
    std::move(projected));

  auto assignment = std::make_shared<const MaterialAssignment>(
    mesh, parameterization, std::move(data),
    std::make_shared<const GlobalAxesMaterialFrameField>(1));
  CubicLinearFormulation formulation;
  DeformationModelOptions options;
  options.projectHessianPSD = false;
  options.enableMaterialMaxStep = false;
  const auto deformationEnergy =
    std::make_shared<DeformationModelEnergy>(
      std::move(assignment), formulation, options);

  ES::VXd displacement = ES::VXd::Zero(deformationEnergy->getNumDOFs());
  for (int i = 0; i < displacement.size(); ++i)
    displacement[i] = 0.01 * std::sin(0.7 * i + 0.2);
  ElasticMaterialEnergy elasticEnergy(deformationEnergy, displacement);

  ES::VXd input(1);
  input[0] = logE;
  ES::VXd gradient(1);
  elasticEnergy.gradient(input, gradient);
  ES::SpMatD hessian;
  elasticEnergy.hessianAlloc(hessian);
  elasticEnergy.hessianInPlace(input, hessian);
  const ES::MXd denseHessian(hessian);

  constexpr double step = 1e-5;
  ES::VXd plus = input;
  ES::VXd minus = input;
  plus[0] += step;
  minus[0] -= step;
  const double finiteDifferenceGradient =
    (elasticEnergy.func(plus) - elasticEnergy.func(minus)) /
    (2.0 * step);

  ES::VXd plusGradient(1), minusGradient(1);
  elasticEnergy.gradient(plus, plusGradient);
  elasticEnergy.gradient(minus, minusGradient);
  const double finiteDifferenceHessian =
    (plusGradient[0] - minusGradient[0]) / (2.0 * step);

  EXPECT_NEAR(
    gradient[0], finiteDifferenceGradient,
    1e-7 * std::max(1.0, std::abs(finiteDifferenceGradient)));
  ASSERT_EQ(denseHessian.rows(), 1);
  ASSERT_EQ(denseHessian.cols(), 1);
  EXPECT_NEAR(
    denseHessian(0, 0), finiteDifferenceHessian,
    1e-7 * std::max(1.0, std::abs(finiteDifferenceHessian)));
}

}  // namespace
