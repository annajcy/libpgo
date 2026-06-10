#pragma once

#include "tetFormulation.h"

namespace pgo
{
namespace SolidDeformationModel
{

class TetLinearFormulation : public TetFormulation
{
public:
  TetLinearFormulation();
  std::string_view getName() const override;
  int getNodesPerElement() const override;
  int getLocalDofs() const override;
  const Quadrature &massQuadrature() const override;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
