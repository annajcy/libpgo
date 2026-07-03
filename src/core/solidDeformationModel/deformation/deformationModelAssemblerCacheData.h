#pragma once

#include "deformation/deformationModel.h"
#include "formulations/dof/dofLayout.h"
#include "EigenSupport.h"

#include <tbb/enumerable_thread_specific.h>
#include <tbb/partitioner.h>

#include <memory>
#include <vector>

namespace pgo
{
namespace SolidDeformationModel
{

class DeformationModelAssemblerCacheData
{
public:
  struct ThreadScratch
  {
    EigenSupport::VXd localPosition;
    EigenSupport::VXd localDirection;
    EigenSupport::VXd localGradient;
    EigenSupport::VXd elasticParamValues;
    EigenSupport::VXd plasticParamValues;
    EigenSupport::VXd rawParamGradient;
    EigenSupport::VXd localParamGradient;
    EigenSupport::MXd localParamHessian;
    EigenSupport::MXd paramWorkMatrix;
    EigenSupport::MXd localMixedMatrix;
    std::vector<double> paramDerivativeData;
    std::vector<double> paramDerivativeData2;
    std::vector<double> localMatrixData;
    std::vector<double> materialLocationValues;
    std::vector<DofGroup> groups;
    std::vector<int> globalDofIndices;
    double energy = 0.0;

    ThreadScratch() = default;
    ThreadScratch(int localDofs, int maxMaterialLocations,
      int maxMaterialParams, int maxLocalParams);

    DeformationModel::CacheData *cacheFor(const DeformationModel &model);
    size_t numReusableCaches() const { return reusableCacheData.size(); }

  private:
    std::vector<std::unique_ptr<DeformationModel::CacheData>> reusableCacheData;
  };

  DeformationModelAssemblerCacheData(int localDofs, int maxMaterialLocations,
    int maxMaterialParams, int maxLocalParams);

  ThreadScratch &scratchForCurrentThread();
  tbb::enumerable_thread_specific<ThreadScratch> &threadScratch() { return *threadScratch_; }
  const tbb::enumerable_thread_specific<ThreadScratch> &threadScratch() const { return *threadScratch_; }

  tbb::affinity_partitioner partitioners[5];

private:
  std::unique_ptr<tbb::enumerable_thread_specific<ThreadScratch>> threadScratch_;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
