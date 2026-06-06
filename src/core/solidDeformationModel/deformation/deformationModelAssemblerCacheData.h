#pragma once

#include "deformation/deformationModel.h"
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
  tbb::enumerable_thread_specific<double> energyLocalBuffer;
  tbb::enumerable_thread_specific<EigenSupport::MXd> upperRightBlockTLS, lowerRightBlockTLS;
  tbb::enumerable_thread_specific<EigenSupport::VXd> gradBlockTLS;

  tbb::affinity_partitioner partitioners[5];
  std::vector<std::unique_ptr<DeformationModel::CacheData>> elementCacheData;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
