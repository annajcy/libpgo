/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#pragma once

#include "material/elastic/elasticModel.h"
#include "deformation/hessianProjection.h"
#include "EigenSupport.h"

#include <span>
#include <stdexcept>

namespace pgo
{
namespace SolidDeformationModel
{

// Immutable spectral data for one deformation-gradient evaluation.  The
// matrices use Eigen's normal column-major convention; `stretches(i)` is the
// principal stretch associated with column i of U/V.
struct SpectralState
{
  EigenSupport::M3d F = EigenSupport::M3d::Identity();
  EigenSupport::M3d U = EigenSupport::M3d::Identity();
  EigenSupport::M3d V = EigenSupport::M3d::Identity();
  EigenSupport::V3d stretches = EigenSupport::V3d::Ones();
};

class ElasticModel3DDeformationGradient : public ElasticModel
{
public:
  ElasticModel3DDeformationGradient() {}
  virtual ~ElasticModel3DDeformationGradient() {}

  // The deformation-gradient API is intentionally Eigen-native.  A single
  // SpectralState keeps F, its SVD factors, and the principal stretches
  // together, so callers cannot accidentally mix data from different SVDs.
  virtual double compute_psi(std::span<const double> param,
    const SpectralState &state) const = 0;
  virtual EigenSupport::M3d compute_P(std::span<const double> param,
    const SpectralState &state) const = 0;
  virtual EigenSupport::M9d compute_dPdF(std::span<const double> param,
    const SpectralState &state) const = 0;
  virtual EigenSupport::M9d compute_dPdF_psd(std::span<const double> param,
    const SpectralState &state) const
  {
    return projectSymmetricPSD(compute_dPdF(param, state));
  }

  // Every concrete 3D model must state its parameter dimension explicitly.
  // Parameter derivative hooks below throw unless the model implements them.
  int getNumParameters() const override = 0;

  // compute the 1st order derivative with respect to the i-th parameter
  virtual double compute_dpsi_dparam(std::span<const double> param, int i,
    const SpectralState &state) const;
  // compute the 2nd order derivative with respect to the (i-th, j-th) parameter
  virtual double compute_d2psi_dparam2(std::span<const double> param, int i, int j,
    const SpectralState &state) const;
  // compute the 2nd order derivative with respect to the i-th parameter and F
  virtual EigenSupport::M3d compute_dP_dparam(std::span<const double> param, int i,
    const SpectralState &state) const;

  // M81x9d row (j * 9 + i), column k contains d3psi/(dFi dFj dFk).
  virtual void compute_d2PdF2(std::span<const double> param,
    const SpectralState &state, EigenSupport::M81x9d &d2PdF2) const;
  virtual EigenSupport::M3d compute_d2Pdparam2(std::span<const double> param, int i, int j,
    const SpectralState &state) const;
  virtual EigenSupport::M9d compute_d2PdFdparam(std::span<const double> param, int i,
    const SpectralState &state) const;

  bool Has3rdOrderDerivative() const { return has3rdOrderDerivative; }

protected:
  bool has3rdOrderDerivative = false;
};

inline double ElasticModel3DDeformationGradient::compute_dpsi_dparam(
  std::span<const double>, int, const SpectralState &) const
{
  throw std::logic_error(
    "ElasticModel3DDeformationGradient::compute_dpsi_dparam is not implemented.");
}

inline double ElasticModel3DDeformationGradient::compute_d2psi_dparam2(
  std::span<const double>, int, int, const SpectralState &) const
{
  throw std::logic_error(
    "ElasticModel3DDeformationGradient::compute_d2psi_dparam2 is not implemented.");
}

inline EigenSupport::M3d ElasticModel3DDeformationGradient::compute_dP_dparam(
  std::span<const double>, int, const SpectralState &) const
{
  throw std::logic_error(
    "ElasticModel3DDeformationGradient::compute_dP_dparam is not implemented.");
}

inline void ElasticModel3DDeformationGradient::compute_d2PdF2(
  std::span<const double>, const SpectralState &, EigenSupport::M81x9d &) const
{
  throw std::logic_error(
    "ElasticModel3DDeformationGradient::compute_d2PdF2 is not implemented.");
}

inline EigenSupport::M3d ElasticModel3DDeformationGradient::compute_d2Pdparam2(
  std::span<const double>, int, int, const SpectralState &) const
{
  throw std::logic_error(
    "ElasticModel3DDeformationGradient::compute_d2Pdparam2 is not implemented.");
}

inline EigenSupport::M9d ElasticModel3DDeformationGradient::compute_d2PdFdparam(
  std::span<const double>, int, const SpectralState &) const
{
  throw std::logic_error(
    "ElasticModel3DDeformationGradient::compute_d2PdFdparam is not implemented.");
}

}  // namespace SolidDeformationModel
}  // namespace pgo
