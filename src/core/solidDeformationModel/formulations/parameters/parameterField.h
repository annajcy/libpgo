#pragma once

#include "EigenSupport.h"

#include <stdexcept>

namespace pgo
{
namespace SolidDeformationModel
{

enum class ParameterFieldKind
{
  CONSTANT,
  QUADRATURE_POINT,     // future
  NODAL_INTERPOLATED,   // future
  EXTERNAL_PROCEDURAL,  // future
};

struct ParameterSample
{
  EigenSupport::VXd value;
  EigenSupport::MXd dValueDLocal;

  void resize(int numChannels, int numLocalDofs)
  {
    value.resize(numChannels);
    dValueDLocal.resize(numChannels, numLocalDofs);
  }
};

class ParameterField
{
public:
  virtual ~ParameterField() = default;

  virtual ParameterFieldKind kind() const = 0;
  virtual int numChannels() const = 0;
  virtual int numLocalDofs() const = 0;

  virtual void sample(int ele, int quadratureId, ParameterSample &out) const = 0;

  virtual void setGlobalData(const double *data) = 0;

  bool getExposeAsOptimizationVariable() const { return exposeAsOptimizationVariable_; }
  void setExposeAsOptimizationVariable(bool val);

protected:
  bool exposeAsOptimizationVariable_ = false;
};

inline void ParameterField::setExposeAsOptimizationVariable(bool val)
{
  if (val) {
    throw std::invalid_argument(
      "exposeAsOptimizationVariable is not supported in this milestone.");
  }
  exposeAsOptimizationVariable_ = val;
}

class OptimizableField : public ParameterField
{
public:
  struct ParameterDofLayout
  {
    virtual ~ParameterDofLayout() = default;
    virtual int numGlobalDofs() const = 0;
    virtual int numLocalDofs() const = 0;
    virtual void gather(int ele, const double *global, double *local) const = 0;
  };

  virtual const ParameterDofLayout *dofLayout() const = 0;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
