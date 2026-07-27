#pragma once

#include "deformation/deformationModel.h"
#include "formulations/dof/dofLayout.h"
#include "EigenSupport.h"

#include <cstddef>
#include <memory>
#include <span>
#include <vector>

namespace pgo
{
namespace SolidDeformationModel
{

struct DeformationModelAssemblerElementWorkspace
{
  // One workspace follows each element. An outer element task may be
  // suspended while nested parallel work runs, so this storage must not be
  // tied to a worker thread.
  EigenSupport::VXd localPosition;
  EigenSupport::VXd localDirection;
  EigenSupport::VXd localGradient;
  EigenSupport::VXd elasticParamValues;
  EigenSupport::VXd plasticParamValues;
  EigenSupport::VXd localParamValues;
  EigenSupport::VXd rawParamGradient;
  EigenSupport::VXd localParamGradient;
  EigenSupport::MXd localParamHessian;
  EigenSupport::MXd paramWorkMatrix;
  EigenSupport::MXd localMixedMatrix;
  EigenSupport::MXd paramDerivativeData;
  EigenSupport::MXd paramDerivativeData2;
  std::vector<EigenSupport::MXd> plasticParamEvaluatorHessians;
  std::vector<EigenSupport::MXd> elasticParamEvaluatorHessians;
  std::vector<double> localMatrixData;
  std::vector<double> materialLocationValues;
  std::vector<DofGroup> groups;
  double energy = 0.0;

  DeformationModelAssemblerElementWorkspace(int localDofs,
    int maxMaterialLocations, int maxMaterialParams, int maxLocalParams,
    const DeformationModel &model);

  std::span<EigenSupport::MXd> preparePlasticParamEvaluatorHessians(
    int numChannels, int numLocalParameters);
  std::span<EigenSupport::MXd> prepareElasticParamEvaluatorHessians(
    int numChannels, int numLocalParameters);

  DeformationModelEvaluator &evaluator() { return *evaluator_; }
  const DeformationModelEvaluator &evaluator() const { return *evaluator_; }

private:
  std::unique_ptr<DeformationModelEvaluator> evaluator_;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
