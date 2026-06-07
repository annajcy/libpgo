#include "rayleighDampingAssembly.h"

#include "EigenSupport.h"

namespace pgo
{
namespace Simulation
{
namespace ES = pgo::EigenSupport;

EigenSupport::SpMatD assembleRayleighDamping(
  const std::vector<ImplicitModelTerm> &terms,
  const EigenSupport::SpMatD &mass,
  EigenSupport::ConstRefVecXd state)
{
  const int n = (int)mass.rows();

  // Accumulate into triplets; the structural pattern (union of the mass pattern
  // and the contributing terms' Hessian patterns) is state-independent, which
  // lets the stage residual reuse a single Hessian template across steps.
  std::vector<ES::TripletD> entries;

  double totalMassDamping = 0.0;
  for (const ImplicitModelTerm &term : terms)
    totalMassDamping += term.massDamping;

  if (totalMassDamping != 0.0) {
    entries.reserve(entries.size() + (std::size_t)mass.nonZeros());
    for (ES::IDX outeri = 0; outeri < mass.outerSize(); ++outeri) {
      for (ES::SpMatD::InnerIterator it(mass, outeri); it; ++it) {
        entries.emplace_back(
          (ES::SpMatD::StorageIndex)it.row(),
          (ES::SpMatD::StorageIndex)it.col(),
          totalMassDamping * it.value());
      }
    }
  }

  for (const ImplicitModelTerm &term : terms) {
    if (!(term.stiffnessDamping > 0.0))
      continue;
    if (!term.energy->isHessianTopologyFixed())
      continue;

    std::vector<int> dofs;
    term.energy->getDOFs(dofs);

    ES::SpMatD K;
    term.energy->hessianAlloc(K);
    if (K.nonZeros() == 0)
      continue;

    ES::VXd xlocal((Eigen::Index)dofs.size());
    for (int j = 0; j < (int)dofs.size(); j++)
      xlocal[j] = state[dofs[j]];

    term.energy->hessianInPlace(xlocal, K);

    entries.reserve(entries.size() + (std::size_t)K.nonZeros());
    for (ES::IDX outeri = 0; outeri < K.outerSize(); ++outeri) {
      for (ES::SpMatD::InnerIterator it(K, outeri); it; ++it) {
        entries.emplace_back(
          (ES::SpMatD::StorageIndex)dofs[it.row()],
          (ES::SpMatD::StorageIndex)dofs[it.col()],
          term.stiffnessDamping * it.value());
      }
    }
  }

  ES::SpMatD D(n, n);
  D.setFromTriplets(entries.begin(), entries.end());
  return D;
}

}  // namespace Simulation
}  // namespace pgo
