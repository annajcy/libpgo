#pragma once

#include "EigenSupport.h"

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

class ParameterField
{
public:
  virtual ~ParameterField() = default;

  virtual ParameterFieldKind kind() const = 0;
  virtual int numChannels() const = 0;
  virtual int numLocalDofs() const = 0;

  virtual void computeValue(int ele, int quadratureId, double *out) const = 0;

  virtual void setGlobalData(const double *data) = 0;
};

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
  virtual void computeDerivative(int ele, int quadratureId, double *derivOut) const = 0;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
