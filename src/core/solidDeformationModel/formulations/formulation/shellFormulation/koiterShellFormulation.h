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
  int getNodesPerElement() const override;
  int getLocalDofs() const override;

  std::unique_ptr<ShellElementMapping> createElementMapping(
    const double restX[18], const bool hasVtx[6]) const override;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
