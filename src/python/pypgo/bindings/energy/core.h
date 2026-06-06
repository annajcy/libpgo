#pragma once

#include "eigen_numpy.h"
#include "evaluation.h"
#include "potentialEnergy.h"
#include "solver/common/solveDiagnostics.h"
#include "../sparse/core.h"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

class PyPotentialEnergy
{
public:
  explicit PyPotentialEnergy(std::shared_ptr<const pgo::NonlinearOptimization::PotentialEnergy> energy):
    handle_(std::move(energy))
  {
  }

  int numDofs() const { return handle_->getNumDOFs(); }

  nanobind::ndarray<nanobind::numpy, std::int64_t> dofs() const
  {
    auto d = pgo::NonlinearOptimization::dofsOf(*handle_);
    auto storage = new std::vector<std::int64_t>(d.begin(), d.end());
    nanobind::capsule owner(storage, [](void *p) noexcept {
      delete static_cast<std::vector<std::int64_t> *>(p);
    });
    return nanobind::ndarray<nanobind::numpy, std::int64_t>(
      storage->data(),
      { storage->size() },
      owner);
  }

  std::string stateKind() const
  {
    switch (handle_->stateKind()) {
      case pgo::NonlinearOptimization::EnergyStateKind::Displacement:
        return "displacement";
      default:
        return "generic";
    }
  }

  double value(nanobind::ndarray<nanobind::numpy, const double> x) const
  {
    auto xMap = pgo::python::ndarrayToVectorMapXd(x);
    double result;
    {
      nanobind::gil_scoped_release release;
      result = pgo::NonlinearOptimization::evaluateValue(*handle_, xMap);
    }
    return result;
  }

  nanobind::ndarray<nanobind::numpy, double> gradient(nanobind::ndarray<nanobind::numpy, const double> x) const
  {
    auto xMap = pgo::python::ndarrayToVectorMapXd(x);
    pgo::EigenSupport::VXd grad;
    {
      nanobind::gil_scoped_release release;
      grad = pgo::NonlinearOptimization::evaluateGradient(*handle_, xMap);
    }
    return pgo::python::vectorXdToNdarray(std::move(grad));
  }

  PySparseMatrix hessian(nanobind::ndarray<nanobind::numpy, const double> x) const
  {
    auto xMap = pgo::python::ndarrayToVectorMapXd(x);
    pgo::EigenSupport::SpMatD H;
    {
      nanobind::gil_scoped_release release;
      H = pgo::NonlinearOptimization::evaluateHessian(*handle_, xMap);
    }
    return PySparseMatrix(std::move(H));
  }

  pgo::NonlinearOptimization::StepConstraint maxStep(
    nanobind::ndarray<nanobind::numpy, const double> x,
    nanobind::ndarray<nanobind::numpy, const double> dx) const
  {
    auto xMap = pgo::python::ndarrayToVectorMapXd(x);
    auto dxMap = pgo::python::ndarrayToVectorMapXd(dx);
    pgo::NonlinearOptimization::StepConstraint result;
    {
      nanobind::gil_scoped_release release;
      result = pgo::NonlinearOptimization::evaluateMaxStep(*handle_, xMap, dxMap);
    }
    return result;
  }

  nanobind::ndarray<nanobind::numpy, double> zeroState() const
  {
    int n = handle_->getNumDOFs();
    auto data = new std::vector<double>(static_cast<size_t>(n), 0.0);
    nanobind::capsule owner(data, [](void *p) noexcept {
      delete static_cast<std::vector<double> *>(p);
    });
    return nanobind::ndarray<nanobind::numpy, double>(
      data->data(),
      { data->size() },
      owner);
  }

  std::string repr() const
  {
    return "PotentialEnergy(" + std::to_string(handle_->getNumDOFs()) + " DOFs)";
  }

  std::shared_ptr<const pgo::NonlinearOptimization::PotentialEnergy> handle_;
};
