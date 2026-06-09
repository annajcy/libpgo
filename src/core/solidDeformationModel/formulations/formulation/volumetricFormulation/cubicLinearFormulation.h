#pragma once

#include "cubicFormulation.h"

namespace pgo
{
namespace SolidDeformationModel
{

class CubicLinearFormulation : public CubicFormulation
{
public:
  CubicLinearFormulation();
  std::string_view getName() const override;
  int getNodesPerElement() const override;
  int getLocalDofs() const override;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
