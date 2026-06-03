/*
author: Bohan Wang
copyright to USC, MIT
*/

#include "laplacianProblem.h"

#include "energySet.h"
#include "quadraticPotentialEnergy.h"
#include "solver/ipopt/IpoptOptimizer.h"
#include "solver/knitro/KnitroOptimizer.h"
#include "solver/service/optimizationProblem.h"
#include "solver/service/optimizationResult.h"

#include <algorithm>
#include <iostream>
#include <numeric>
#include <stdexcept>

using namespace pgo;
using namespace pgo::PredefinedPotentialEnergies;

namespace ES = pgo::EigenSupport;

LaplacianProblem::LaplacianProblem(const EigenSupport::SpMatD &L, int biLaplacian, const std::vector<int> &fixedDOFs,
  const EigenSupport::VXd &fixedValues, double variableLow, double variableHi)
{
  if (biLaplacian) {
    // sys = L.transpose() * L;
    ES::mm(L, L, sys, 1);
  }
  else
    sys = L;

  xlow.resize(sys.rows());
  xhi.resize(sys.rows());

  setVariableConstraints(fixedDOFs, fixedValues, variableLow, variableHi);
}

void LaplacianProblem::setVariableConstraints(const std::vector<int> &fixedDOFs, const EigenSupport::VXd &fixedValues, double variableLow, double variableHi)
{
  for (Eigen::Index i = 0; i < sys.rows(); i++) {
    xlow[i] = variableLow;
    xhi[i] = variableHi;
  }

  for (int dof : fixedDOFs) {
    xlow[dof] = fixedValues[dof];
    xhi[dof] = fixedValues[dof];
  }

  xinit = fixedValues;
}

void LaplacianProblem::solve(EigenSupport::RefVecXd xfinal, int numIter, double eps, int verbose)
{
  (void)numIter;
  (void)eps;
  (void)verbose;

  std::shared_ptr<QuadraticPotentialEnergy> energy = std::make_shared<QuadraticPotentialEnergy>(sys);

  auto energyAll = std::make_shared<NonlinearOptimization::EnergySet>(sys.rows(),
      std::vector<NonlinearOptimization::EnergySet::Term>{{energy, 1.0}});

  xfinal = xinit;
  NonlinearOptimization::Optimization::OptimizationProblem problem;
  problem.objective = energyAll;
  problem.variableBounds.lower = xlow;
  problem.variableBounds.upper = xhi;

  NonlinearOptimization::Optimization::OptimizationResult optimizationResult;
#if defined(PGO_HAS_KNITRO)
  NonlinearOptimization::Optimization::KnitroOptimizer::Options options;
  options.maxIterations = numIter;
  options.optimalityTolerance = eps;
  options.verbose = std::max(verbose, 0);
  NonlinearOptimization::Optimization::KnitroOptimizer optimizer(options);
  optimizationResult = optimizer.solve(problem, xfinal);
#elif defined(PGO_HAS_IPOPT)
  NonlinearOptimization::Optimization::IpoptOptimizer::Options options;
  options.maxIterations = numIter;
  options.tolerance = eps;
  options.printLevel = std::max(verbose, 0);
  NonlinearOptimization::Optimization::IpoptOptimizer optimizer(options);
  optimizationResult = optimizer.solve(problem, xfinal);
#else
  throw std::runtime_error("LaplacianProblem requires Knitro or Ipopt for bounded optimization");
#endif

  xfinal = optimizationResult.x;
  NonlinearOptimization::SolverResult result = optimizationResult.solver;
  std::cout << "Solver result: " << NonlinearOptimization::formatSolverResultSummary(result) << std::endl;
}
