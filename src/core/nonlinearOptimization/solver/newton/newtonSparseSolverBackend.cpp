#include "solver/newton/newtonSparseSolverBackend.h"

#if defined(PGO_HAS_MKL) && !defined(PGO_HAS_ORIG_PARDISO)
#  include "EigenMKLPardisoSupport.h"
#endif

#if defined(PGO_HAS_ORIG_PARDISO)
#  include "EigenOrigPardisoSupport.h"
#endif

#include <Eigen/Sparse>

#include <stdexcept>
#include <utility>

namespace pgo::NonlinearOptimization
{
namespace
{

class EigenSimplicialLDLTBackend final : public NewtonSparseSolverBackend
{
public:
  void analyze(const EigenSupport::SpMatD &A) override
  {
    n_ = A.rows();
    solver_.analyzePattern(A);
  }

  bool factorize(const EigenSupport::SpMatD &A) override
  {
    solver_.factorize(A);
    return solver_.info() == Eigen::Success;
  }

  bool solve(const EigenSupport::SpMatD &, double *x, double *rhs) override
  {
    Eigen::Map<EigenSupport::VXd>(x, n_).noalias() =
      solver_.solve(Eigen::Map<const EigenSupport::VXd>(rhs, n_));
    return solver_.info() == Eigen::Success;
  }

  const char *name() const override { return "EigenSimplicialLDLT"; }

private:
  Eigen::Index n_ = 0;
  Eigen::SimplicialLDLT<EigenSupport::SpMatD> solver_;
};

#if defined(PGO_HAS_MKL) && !defined(PGO_HAS_ORIG_PARDISO)
class MKLPardisoBackend final : public NewtonSparseSolverBackend
{
public:
  explicit MKLPardisoBackend(const EigenSupport::SpMatD &A):
    solver_(A, EigenSupport::EigenMKLPardisoSupport::MatrixType::REAL_SYM_INDEFINITE,
      EigenSupport::EigenMKLPardisoSupport::ReorderingType::NESTED_DISSECTION,
      0, 0, 0, 0, 0, 0)
  {
  }

  void analyze(const EigenSupport::SpMatD &A) override { solver_.analyze(A); }
  bool factorize(const EigenSupport::SpMatD &A) override { return solver_.factorize(A) == 0; }
  bool solve(const EigenSupport::SpMatD &A, double *x, double *rhs) override { return solver_.solve(A, x, rhs, 1) == 0; }
  const char *name() const override { return "MKLPardiso"; }

private:
  EigenSupport::EigenMKLPardisoSupport solver_;
};
#endif

#if defined(PGO_HAS_ORIG_PARDISO)
class OrigPardisoBackend final : public NewtonSparseSolverBackend
{
public:
  explicit OrigPardisoBackend(const EigenSupport::SpMatD &A):
    solver_(A, EigenSupport::EigenOrigPardisoSupport::MatrixType::REAL_SYM_INDEFINITE,
      EigenSupport::EigenOrigPardisoSupport::ReorderingType::NESTED_DISSECTION_4,
      0, 0, 0, 0, 0, 0)
  {
  }

  void analyze(const EigenSupport::SpMatD &A) override { solver_.analyze(A); }
  bool factorize(const EigenSupport::SpMatD &A) override { return solver_.factorize(A) == 0; }
  bool solve(const EigenSupport::SpMatD &A, double *x, double *rhs) override { return solver_.solve(A, x, rhs, 1) == 0; }
  const char *name() const override { return "OrigPardiso"; }

private:
  EigenSupport::EigenOrigPardisoSupport solver_;
};
#endif

std::unique_ptr<NewtonSparseSolverBackend> createEigenBackend(const EigenSupport::SpMatD &A)
{
  auto backend = std::make_unique<EigenSimplicialLDLTBackend>();
  backend->analyze(A);
  return backend;
}

}  // namespace

std::unique_ptr<NewtonSparseSolverBackend> createNewtonSparseSolverBackend(
  const NewtonSparseSolverOptions &options,
  const EigenSupport::SpMatD &A)
{
  switch (options.kind) {
    case NewtonSparseSolverKind::Auto:
#if defined(PGO_HAS_MKL) && !defined(PGO_HAS_ORIG_PARDISO)
      {
        auto backend = std::make_unique<MKLPardisoBackend>(A);
        backend->analyze(A);
        return backend;
      }
#else
      return createEigenBackend(A);
#endif

    case NewtonSparseSolverKind::EigenSimplicialLDLT:
      return createEigenBackend(A);

    case NewtonSparseSolverKind::MKLPardiso:
#if defined(PGO_HAS_MKL) && !defined(PGO_HAS_ORIG_PARDISO)
      {
        auto backend = std::make_unique<MKLPardisoBackend>(A);
        backend->analyze(A);
        return backend;
      }
#else
      throw std::invalid_argument("MKL Pardiso sparse solver backend is not available in this build");
#endif

    case NewtonSparseSolverKind::OrigPardiso:
#if defined(PGO_HAS_ORIG_PARDISO)
      {
        auto backend = std::make_unique<OrigPardisoBackend>(A);
        backend->analyze(A);
        return backend;
      }
#else
      throw std::invalid_argument("Orig Pardiso sparse solver backend is not available in this build");
#endif
  }

  throw std::invalid_argument("Unknown Newton sparse solver backend");
}

// ── Selector ``build`` — thin wrappers around the existing factory ─────────

std::unique_ptr<NewtonSparseSolverBackend> AutoSparseSolverSelector::build(const EigenSupport::SpMatD &A) const
{
#if defined(PGO_HAS_MKL) && !defined(PGO_HAS_ORIG_PARDISO)
  // MKL PARDISO: best performance, included with conda MKL
  auto backend = std::make_unique<MKLPardisoBackend>(A);
  backend->analyze(A);
  return backend;
#else
  // Eigen SimplicialLDLT: always available fallback
  auto backend = std::make_unique<EigenSimplicialLDLTBackend>();
  backend->analyze(A);
  return backend;
#endif
}

std::unique_ptr<NewtonSparseSolverBackend> EigenLDLTSparseSolverSelector::build(const EigenSupport::SpMatD &A) const
{
  return createEigenBackend(A);
}

std::unique_ptr<NewtonSparseSolverBackend> MKLPardisoSparseSolverSelector::build(const EigenSupport::SpMatD &A) const
{
#if defined(PGO_HAS_MKL) && !defined(PGO_HAS_ORIG_PARDISO)
  auto backend = std::make_unique<MKLPardisoBackend>(A);
  backend->analyze(A);
  return backend;
#else
  throw std::invalid_argument("MKL Pardiso sparse solver backend is not available in this build");
#endif
}

std::unique_ptr<NewtonSparseSolverBackend> OrigPardisoSparseSolverSelector::build(const EigenSupport::SpMatD &A) const
{
#if defined(PGO_HAS_ORIG_PARDISO)
  auto backend = std::make_unique<OrigPardisoBackend>(A);
  backend->analyze(A);
  return backend;
#else
  throw std::invalid_argument("Orig Pardiso sparse solver backend is not available in this build");
#endif
}

}  // namespace pgo::NonlinearOptimization
