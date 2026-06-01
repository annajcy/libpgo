#pragma once

#include "potentialEnergy.h"

#include <vector>

namespace pgo::NonlinearOptimization
{

double evaluateValue(const PotentialEnergy &energy, EigenSupport::ConstRefVecXd x);

EigenSupport::VXd evaluateGradient(const PotentialEnergy &energy, EigenSupport::ConstRefVecXd x);

EigenSupport::SpMatD evaluateHessian(const PotentialEnergy &energy, EigenSupport::ConstRefVecXd x);

MaxStepResult evaluateMaxStep(const PotentialEnergy &energy, EigenSupport::ConstRefVecXd x, EigenSupport::ConstRefVecXd dx);

std::vector<int> dofsOf(const PotentialEnergy &energy);

}  // namespace pgo::NonlinearOptimization
