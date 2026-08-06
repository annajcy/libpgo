#pragma once

#include "energy/deformationEnergyOperator.h"

namespace pgo::SolidDeformationModel
{

/// Displacement potential obtained by fixing one immutable material state.
class DeformationPotentialEnergy final : public NonlinearOptimization::PotentialEnergy
{
public:
  DeformationPotentialEnergy(
    std::shared_ptr<DeformationEnergyOperator> energyOperator,
    MaterialState materialState);

  const DeformationEnergyOperator &energyOperator() const { return *energyOperator_; }
  DeformationEnergyOperator &energyOperator() { return *energyOperator_; }
  const MaterialState &materialState() const { return materialState_; }
  const DeformationModelAssembler &assembler() const
  {
    return energyOperator_->assembler();
  }
  const EigenSupport::VXd &getRestDofs() const
  {
    return energyOperator_->getRestDofs();
  }
  const EigenSupport::VXd &getVertexRestPositions() const
  {
    return energyOperator_->getVertexRestPositions();
  }
  void computePlasticGradient(EigenSupport::ConstRefVecXd u, EigenSupport::RefVecXd g) const
  {
    energyOperator_->computePlasticGradient(u, materialState_.view(), g);
  }
  void computeElasticGradient(EigenSupport::ConstRefVecXd u, EigenSupport::RefVecXd g) const
  {
    energyOperator_->computeElasticGradient(u, materialState_.view(), g);
  }
  void computeVonMisesStresses(EigenSupport::ConstRefVecXd u, EigenSupport::RefVecXd out) const
  {
    energyOperator_->computeVonMisesStresses(u, materialState_.view(), out);
  }
  void computeMaxStrains(EigenSupport::ConstRefVecXd u, EigenSupport::RefVecXd out) const
  {
    energyOperator_->computeMaxStrains(u, materialState_.view(), out);
  }

  double func(EigenSupport::ConstRefVecXd x) const override;
  void gradient(
    EigenSupport::ConstRefVecXd x,
    EigenSupport::RefVecXd grad) const override;
  void hessianInPlace(
    EigenSupport::ConstRefVecXd x,
    EigenSupport::SpMatD &hess) const override;
  void hessianAlloc(EigenSupport::SpMatD &hess) const override;
  // Combined hot-path evaluations: the underlying operator prepares each
  // element's state once for all requested quantities.
  double funcGradient(
    EigenSupport::ConstRefVecXd x,
    EigenSupport::RefVecXd grad) const override;
  double funcGradientHessian(
    EigenSupport::ConstRefVecXd x,
    EigenSupport::RefVecXd grad,
    EigenSupport::SpMatD &hess) const override;
  void gradientHessian(
    EigenSupport::ConstRefVecXd x,
    EigenSupport::RefVecXd grad,
    EigenSupport::SpMatD &hess) const override;
  void getDOFs(std::vector<int> &dofs) const override;
  int getNumDOFs() const override;
  NonlinearOptimization::EnergyStateKind stateKind() const override
  {
    return NonlinearOptimization::EnergyStateKind::Displacement;
  }

private:
  std::shared_ptr<DeformationEnergyOperator> energyOperator_;
  MaterialState materialState_;
};

}  // namespace pgo::SolidDeformationModel
