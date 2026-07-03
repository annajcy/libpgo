#pragma once
#include "solver/newton/newtonTerminationPolicy.h"  // for NewtonIterationContext
#include <memory>
namespace pgo::NonlinearOptimization {

class NewtonDampingPolicy {
public:
  virtual ~NewtonDampingPolicy() = default;
  virtual double dampingForIteration(const NewtonIterationContext &ctx) const = 0;
};

class NoDampingPolicy final : public NewtonDampingPolicy {
public:
  double dampingForIteration(const NewtonIterationContext &) const override;
};

class FixedDampingPolicy final : public NewtonDampingPolicy {
public:
  struct Params { double dampingScale = 1.0; };
  explicit FixedDampingPolicy(Params p = Params{1.0}) : params_(p) {}
  double dampingForIteration(const NewtonIterationContext &ctx) const override;
private:
  Params params_;
};

}  // namespace pgo::NonlinearOptimization
