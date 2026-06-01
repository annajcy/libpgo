#pragma once

#include "potentialEnergy.h"
#include "lineSearchAwareEnergy.h"

#include <memory>
#include <vector>

namespace pgo
{
namespace NonlinearOptimization
{
class EnergySetBuffer;

class EnergySet : public PotentialEnergy, public LineSearchAwareEnergy
{
public:
  struct Term
  {
    std::shared_ptr<const PotentialEnergy> energy;
    double weight = 1.0;
  };

  EnergySet(int numDofs, std::vector<Term> terms);
  EnergySet(int numDofs, std::vector<Term> terms, std::shared_ptr<EnergySetBuffer> buffer);

  virtual ~EnergySet();

  int numTerms() const { return static_cast<int>(terms_.size()); }
  const Term &term(int i) const { return terms_[i]; }
  void setWeight(int i, double w);

  virtual double func(EigenSupport::ConstRefVecXd x) const override;
  virtual void gradient(EigenSupport::ConstRefVecXd x, EigenSupport::RefVecXd grad) const override;
  virtual void hessianInPlace(EigenSupport::ConstRefVecXd x, EigenSupport::SpMatD &hess) const override;
  virtual double func_grad_hessian(
    EigenSupport::ConstRefVecXd x,
    EigenSupport::RefVecXd grad,
    EigenSupport::SpMatD &hess) const override;
  virtual void hessianVector(EigenSupport::ConstRefVecXd x, EigenSupport::ConstRefVecXd vec, EigenSupport::RefVecXd hessVec) const override;
  virtual void hessianAlloc(EigenSupport::SpMatD &hess) const override { hess = hessianAll; }
  virtual void gradient_hessian(EigenSupport::ConstRefVecXd x, EigenSupport::RefVecXd grad, EigenSupport::SpMatD &hess) const override;

  virtual void getDOFs(std::vector<int> &dofs) const override { dofs = allDOFs; }
  virtual int getNumDOFs() const override { return nAll; }

  const EigenSupport::SpMatD &getHessianTemplate() const { return hessianAll; }
  int getNNZHessian() const { return static_cast<int>(hessianAll.nonZeros()); }

  virtual int isQuadratic() const override { return isQuadraticEnergy; }
  virtual int hasHessianVector() const override { return hasHessianVectorProduct; }
  virtual int isHessianTopologyFixed() const override;
  virtual void hessian(EigenSupport::ConstRefVecXd x, EigenSupport::SpMatD &hess) const override;

  virtual EnergyStateKind stateKind() const override;

  virtual MaxStepResult computeMaxStepLimit(EigenSupport::ConstRefVecXd x, EigenSupport::ConstRefVecXd dx) const override;
  virtual void beginLineSearch(EigenSupport::ConstRefVecXd x, EigenSupport::ConstRefVecXd dx) const override;
  virtual void endLineSearch() const override;

  void printEnergy(EigenSupport::ConstRefVecXd x) const;

private:
  void init_();

protected:
  void mapx(EigenSupport::ConstRefVecXd x, const std::vector<int> &dofs, EigenSupport::RefVecXd xlocal) const;

  std::vector<Term> terms_;
  std::vector<PotentialEnergy_const_p> potentialEnergies;
  std::vector<EigenSupport::SpMatI, Eigen::aligned_allocator<EigenSupport::SpMatI>> hessianMatrixMappings;
  std::vector<std::vector<int>> energyDOFs;
  EigenSupport::SpMatD hessianAll;

  std::vector<double> energyCoeffs;
  std::vector<int> allDOFs;
  int nAll;
  int isQuadraticEnergy = 0;
  int hasHessianVectorProduct = 0;

  std::shared_ptr<EnergySetBuffer> buffer_;
};

typedef std::shared_ptr<EnergySet> EnergySet_p;
typedef std::shared_ptr<const EnergySet> EnergySet_const_p;
}  // namespace NonlinearOptimization
}  // namespace pgo
