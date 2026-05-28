#pragma once

#include "EigenDef.h"

#include <vector>

namespace pgo
{
namespace SolidDeformationModel
{

// Abstract interface for DOF gather/scatter/sparsity policy.
// Task 6 will complete the assembler migration to use this interface.
// Task 2 only declares it so FormulationTraits can reference DofLayout types.

// Note: DynamicIndexMatrix is defined here (instead of the assembler header)
// so DofLayout can declare it without a circular dependency.
using DynamicIndexMatrix = Eigen::Matrix<std::ptrdiff_t, Eigen::Dynamic, Eigen::Dynamic>;

class DofLayout
{
public:
  virtual ~DofLayout() = default;

  virtual int numGlobalDofs() const = 0;
  virtual int numLocalDofs(int ele) const = 0;

  virtual void gather(int ele, const double *global, double *local) const = 0;
  virtual void scatterAddGradient(int ele, const double *local, double *global) const = 0;

  virtual void addHessianSparsity(int ele,
    std::vector<EigenSupport::TripletD> &entries) const = 0;

  virtual void buildLocalToGlobalMatrixIndices(int ele,
    const EigenSupport::SpMatD &KTemplate,
    DynamicIndexMatrix &indices) const = 0;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
