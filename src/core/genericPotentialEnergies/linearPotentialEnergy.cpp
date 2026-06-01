/*
author: Bohan Wang
copyright to USC, MIT
*/

#include "linearPotentialEnergy.h"
#include "pgoLogging.h"

#include <numeric>

using namespace pgo::PredefinedPotentialEnergies;

LinearPotentialEnergy::LinearPotentialEnergy(EigenSupport::VXd b):
  b_(std::move(b))
{
  allDOFs.assign(b_.size(), 0);
  std::iota(allDOFs.begin(), allDOFs.end(), 0);
}

void LinearPotentialEnergy::setDOFs(const std::vector<int> &dofs)
{
  PGO_ALOG((int)dofs.size() == (int)b_.size());
  allDOFs = dofs;
}