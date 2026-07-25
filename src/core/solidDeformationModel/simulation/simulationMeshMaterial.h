#pragma once

#include <memory>
#include <cmath>
#include <stdexcept>

namespace pgo
{
namespace SolidDeformationModel
{

/// Immutable material input values used to populate evaluator data.
class SimulationMeshENuMaterial
{
public:
  SimulationMeshENuMaterial() = default;
  SimulationMeshENuMaterial(double E_, double nu_, double J_ = 10000): E(E_), nu(nu_), J(J_) {}

  double getMuLame() const { return E / (2 * (1 + nu)); }
  double getLambdaLame() const { return (nu * E) / ((1 + nu) * (1 - 2 * nu)); }
  double getE() const { return E; }
  double getNu() const { return nu; }
  double getCompressionRatio() const { return J; }

private:
  double E = 6e3, nu = 0.4, J = 10000;
};

class SimulationMeshENuhMaterial : public SimulationMeshENuMaterial
{
public:
  SimulationMeshENuhMaterial() = default;
  SimulationMeshENuhMaterial(double E_, double nu_, double h_, double J_ = 10000):
    SimulationMeshENuMaterial(E_, nu_, J_), h(h_) {}
  double geth() const { return h; }
private:
  double h = 1e-4;
};

class SimulationMeshHillMaterial
{
public:
  SimulationMeshHillMaterial() = default;
  SimulationMeshHillMaterial(double E_act_, double gamma_, double lo_): E_act(E_act_), gamma(gamma_), lo(lo_) {}
  double getEact() const { return E_act; }
  double getGamma() const { return gamma; }
  double getLo() const { return lo; }
private:
  double E_act = 0.1e6, gamma = 1, lo = 0.6;
};

class SimulationMeshMooneyRivlinMaterial
{
public:
  SimulationMeshMooneyRivlinMaterial(double mu01_, double mu10_, double v1_):
    mu01Value(mu01_), mu10Value(mu10_), v1Value(v1_)
  {
    if (!std::isfinite(mu01Value) || !std::isfinite(mu10Value) || !std::isfinite(v1Value))
      throw std::invalid_argument("Mooney-Rivlin parameters must be finite");
    if (mu01Value + mu10Value <= 0.0)
      throw std::invalid_argument("Mooney-Rivlin requires mu01 + mu10 > 0");
    if (v1Value <= 0.0)
      throw std::invalid_argument("Mooney-Rivlin requires v1 > 0");
  }

  double mu01() const { return mu01Value; }
  double mu10() const { return mu10Value; }
  double v1() const { return v1Value; }

private:
  double mu01Value;
  double mu10Value;
  double v1Value;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
