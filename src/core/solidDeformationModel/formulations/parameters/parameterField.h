#pragma once

#include "EigenSupport.h"

#include <string>
#include <vector>

namespace pgo
{
namespace SolidDeformationModel
{

enum class ParameterFieldKind
{
  CONSTANT,
  ELEMENTWISE,
  QUADRATURE_POINT,     // future
  NODAL_INTERPOLATED,   // future
  EXTERNAL_PROCEDURAL,  // future
};

enum class ParameterDomain
{
  ELASTIC,
  PLASTIC,
};

struct ParameterFieldSpec
{
  ParameterDomain domain = ParameterDomain::ELASTIC;
  std::string modelId;
  int numChannels = 0;
  std::vector<std::string> channelNames;
};

class ParameterField
{
public:
  virtual ~ParameterField() = default;

  virtual const ParameterFieldSpec &spec() const = 0;
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
    virtual bool matchesParameterShape(int numChannels, int numElements) const = 0;
    virtual int globalDof(int ele, int localDof) const = 0;
    virtual void gather(int ele, const double *global, double *local) const = 0;
  };

  virtual const ParameterDofLayout *dofLayout() const = 0;
  virtual const double *globalData() const = 0;
  virtual int numValueRows() const = 0;
  virtual void computeDerivative(int ele, int quadratureId, double *derivOut) const = 0;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
