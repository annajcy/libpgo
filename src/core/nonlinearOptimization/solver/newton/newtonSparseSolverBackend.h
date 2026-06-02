#pragma once

#include "EigenSupport.h"

#include <memory>

namespace pgo::NonlinearOptimization
{

enum class NewtonSparseSolverKind
{
  Auto,
  EigenSimplicialLDLT,
  MKLPardiso,
  OrigPardiso
};

struct NewtonSparseSolverOptions
{
  NewtonSparseSolverKind kind = NewtonSparseSolverKind::Auto;
};

class NewtonSparseSolverBackend
{
public:
  virtual ~NewtonSparseSolverBackend() = default;

  virtual void analyze(const EigenSupport::SpMatD &A) = 0;
  virtual bool factorize(const EigenSupport::SpMatD &A) = 0;
  virtual bool solve(const EigenSupport::SpMatD &A, double *x, double *rhs) = 0;
  virtual const char *name() const = 0;
};

std::unique_ptr<NewtonSparseSolverBackend> createNewtonSparseSolverBackend(
  const NewtonSparseSolverOptions &options,
  const EigenSupport::SpMatD &A);

}  // namespace pgo::NonlinearOptimization
