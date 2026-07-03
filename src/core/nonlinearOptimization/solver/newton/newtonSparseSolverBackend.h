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

// A sparse-solver selector is an immutable, directly-constructible handle that
// carries one backend kind and its parameters.  ``build(A)`` returns the actual
// backend (which needs the sparse pattern to initialise Pardiso wrappers).
// Stored as ``shared_ptr<const>`` in optimizer options — the same handle can be
// reused across solves.

class NewtonSparseSolverSelector
{
public:
  virtual ~NewtonSparseSolverSelector() = default;

  virtual std::unique_ptr<NewtonSparseSolverBackend> build(
    const EigenSupport::SpMatD &A) const = 0;
};

class AutoSparseSolverSelector final : public NewtonSparseSolverSelector
{
public:
  AutoSparseSolverSelector() = default;
  std::unique_ptr<NewtonSparseSolverBackend> build(const EigenSupport::SpMatD &A) const override;
};

class EigenLDLTSparseSolverSelector final : public NewtonSparseSolverSelector
{
public:
  EigenLDLTSparseSolverSelector() = default;
  std::unique_ptr<NewtonSparseSolverBackend> build(const EigenSupport::SpMatD &A) const override;
};

class MKLPardisoSparseSolverSelector final : public NewtonSparseSolverSelector
{
public:
  MKLPardisoSparseSolverSelector() = default;
  std::unique_ptr<NewtonSparseSolverBackend> build(const EigenSupport::SpMatD &A) const override;
};

class OrigPardisoSparseSolverSelector final : public NewtonSparseSolverSelector
{
public:
  OrigPardisoSparseSolverSelector() = default;
  std::unique_ptr<NewtonSparseSolverBackend> build(const EigenSupport::SpMatD &A) const override;
};

}  // namespace pgo::NonlinearOptimization
