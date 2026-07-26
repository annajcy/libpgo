#pragma once

#include "deformation/deformationModel.h"
#include "formulations/dof/dofLayout.h"
#include "EigenSupport.h"

#include <cstddef>
#include <functional>
#include <memory>
#include <span>
#include <vector>

namespace pgo
{
namespace SolidDeformationModel
{

class DeformationModelAssemblerCacheData
{
public:
  // One eagerly allocated workspace per element. An outer element task may be
  // suspended while nested parallel work runs, so this storage must follow the
  // element rather than the worker thread executing it.
  struct ElementScratch
  {
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
    std::vector<EigenSupport::MXd> plasticParamMappingHessians;
    std::vector<EigenSupport::MXd> elasticParamMappingHessians;
    std::vector<double> localMatrixData;
    std::vector<double> materialLocationValues;
    std::vector<DofGroup> groups;
    double energy = 0.0;

    ElementScratch(int localDofs, int maxMaterialLocations,
      int maxMaterialParams, int maxLocalParams,
      const DeformationModel &model);

    std::span<EigenSupport::MXd> preparePlasticParamMappingHessians(
      int numChannels, int numLocalDofs);
    std::span<EigenSupport::MXd> prepareElasticParamMappingHessians(
      int numChannels, int numLocalDofs);

    DeformationModel::CacheData &cacheData() { return *cacheData_; }
    const DeformationModel::CacheData &cacheData() const { return *cacheData_; }

  private:
    std::unique_ptr<DeformationModel::CacheData> cacheData_;
  };

  DeformationModelAssemblerCacheData(int localDofs, int maxMaterialLocations,
    int maxMaterialParams, int maxLocalParams,
    std::span<const std::reference_wrapper<const DeformationModel>> models);

  ElementScratch &elementScratch(int ele) { return elementScratch_[ele]; }
  const ElementScratch &elementScratch(int ele) const { return elementScratch_[ele]; }
  std::size_t numElementScratch() const { return elementScratch_.size(); }

private:
  std::vector<ElementScratch> elementScratch_;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
