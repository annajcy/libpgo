#pragma once

#include "shellFormulation.h"

namespace pgo
{
namespace SolidDeformationModel
{

class KoiterShellFormulation : public ShellFormulation
{
public:
  std::string_view getName() const override;
  int numBasisFunctionsPerElement() const override;
  int getLocalDofs() const override;

  std::unique_ptr<ShellElementMapping> createElementMapping(
    const EigenSupport::V18d &restX, const std::array<bool, 6> &hasVtx) const override;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
