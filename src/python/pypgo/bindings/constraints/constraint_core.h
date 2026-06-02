#pragma once

#include "constraints/constraintEvaluation.h"
#include "constraints/constraintFunctions.h"
#include "eigen_numpy.h"
#include "sparse_matrix_core.h"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

#include <cstdint>
#include <memory>
#include <string>

class PyConstraintFunctions
{
public:
  explicit PyConstraintFunctions(std::shared_ptr<const pgo::NonlinearOptimization::ConstraintFunctions> constraints):
    handle_(std::move(constraints))
  {
  }

  int numDofs() const { return handle_->getNumDOFs(); }
  int numConstraints() const { return handle_->getNumConstraints(); }
  bool isLinear() const { return handle_->isLinear(); }

  nanobind::ndarray<nanobind::numpy, double> value(nanobind::ndarray<nanobind::numpy, const double> x) const
  {
    auto xMap = pgo::python::ndarrayToVectorMapXd(x);
    pgo::EigenSupport::VXd values;
    {
      nanobind::gil_scoped_release release;
      values = pgo::NonlinearOptimization::evaluateConstraintValues(*handle_, xMap);
    }
    return pgo::python::vectorXdToNdarray(std::move(values));
  }

  PySparseMatrix jacobian(nanobind::ndarray<nanobind::numpy, const double> x) const
  {
    auto xMap = pgo::python::ndarrayToVectorMapXd(x);
    pgo::EigenSupport::SpMatD jac;
    {
      nanobind::gil_scoped_release release;
      jac = pgo::NonlinearOptimization::evaluateConstraintJacobian(*handle_, xMap);
    }
    return PySparseMatrix(std::move(jac));
  }

  PySparseMatrix hessian(
    nanobind::ndarray<nanobind::numpy, const double> x,
    nanobind::ndarray<nanobind::numpy, const double> multipliers) const
  {
    auto xMap = pgo::python::ndarrayToVectorMapXd(x);
    auto multiplierMap = pgo::python::ndarrayToVectorMapXd(multipliers);
    pgo::EigenSupport::SpMatD hess;
    {
      nanobind::gil_scoped_release release;
      hess = pgo::NonlinearOptimization::evaluateConstraintHessian(*handle_, xMap, multiplierMap);
    }
    return PySparseMatrix(std::move(hess));
  }

  std::string repr() const
  {
    return "ConstraintFunctions(" + std::to_string(numConstraints()) + " constraints, " +
      std::to_string(numDofs()) + " DOFs)";
  }

  std::shared_ptr<const pgo::NonlinearOptimization::ConstraintFunctions> handle_;
};
