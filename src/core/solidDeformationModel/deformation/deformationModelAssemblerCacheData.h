#pragma once

#include "deformation/deformationModel.h"
#include "EigenSupport.h"

#include <tbb/partitioner.h>
#include <tbb/task_arena.h>

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
    std::vector<int> globalDofIndices;
    double energy = 0.0;

    ThreadScratch() = default;
    ThreadScratch(int localDofs, int maxMaterialLocations,
      int maxMaterialParams, int maxLocalParams);
  };

  DeformationModelAssemblerCacheData(int localDofs, int maxMaterialLocations,
    int maxMaterialParams, int maxLocalParams);

  ThreadScratch &scratchForCurrentThread();
  std::vector<ThreadScratch> &threadScratch() { return threadScratch_; }
  const std::vector<ThreadScratch> &threadScratch() const { return threadScratch_; }

  tbb::affinity_partitioner partitioners[5];
  std::vector<std::unique_ptr<DeformationModel::CacheData>> elementCacheData;

private:
  std::vector<ThreadScratch> threadScratch_;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
