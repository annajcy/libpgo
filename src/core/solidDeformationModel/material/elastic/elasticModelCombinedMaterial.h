/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#pragma once

#include "material/elastic/elasticModelDefinition.h"

#include "material/elastic/elasticModel3DDeformationGradient.h"

#include <array>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

namespace pgo
{
namespace SolidDeformationModel
{
template<int count>
class ElasticModelCombinedMaterial : public ElasticModel3DDeformationGradient
{
  static_assert(
    count > 0,
    "ElasticModelCombinedMaterial requires a positive compile-time material count; use ElasticModelDynamicCombinedMaterial for a runtime count.");

public:
  template<typename... T>
  explicit ElasticModelCombinedMaterial(T&&... mats);
  ~ElasticModelCombinedMaterial() override = default;

  virtual double compute_psi(std::span<const double> param,
    const SpectralState &state) const override;
  virtual EigenSupport::M3d compute_P(std::span<const double> param,
    const SpectralState &state) const override;
  virtual EigenSupport::M9d compute_dPdF(std::span<const double> param,
    const SpectralState &state) const override;

  virtual int getNumParameters() const override { return numTotalParameters; }
  void compute_dpsi_dparams(std::span<const double> param,
    const SpectralState &state,
    EigenSupport::RefVecXd dpsiDparam) const override;
  void compute_d2psi_dparams2(std::span<const double> param,
    const SpectralState &state,
    EigenSupport::RefMatXd d2psiDparam2) const override;
  void compute_dP_dparams(std::span<const double> param,
    const SpectralState &state,
    EigenSupport::RefMatXd dP_dparam) const override;

  virtual void compute_d2PdF2(std::span<const double> param,
    const SpectralState &state, EigenSupport::M81x9d &d2PdF2) const override;
  virtual EigenSupport::M3d compute_d2Pdparam2(std::span<const double> param, int i, int j,
    const SpectralState &state) const override;
  virtual EigenSupport::M9d compute_d2PdFdparam(std::span<const double> param, int i,
    const SpectralState &state) const override;

  const ElasticModel3DDeformationGradient *getMaterial(int id) const { return materials[id]; }

protected:
  std::array<std::unique_ptr<ElasticModel3DDeformationGradient>, count> owned_;
  std::array<const ElasticModel3DDeformationGradient *, count> materials;
  std::array<int, count + 1> parameterOffsets;
  int numTotalParameters;
};

template<int count>
template<typename... T>
inline ElasticModelCombinedMaterial<count>::ElasticModelCombinedMaterial(T&&... mats)
{
  static_assert(sizeof...(T) == count, "Number of args must match count");

  int i = 0;
  (..., (owned_[i] = std::unique_ptr<ElasticModel3DDeformationGradient>(std::forward<T>(mats)),
         materials[i] = owned_[i].get(),
         ++i));

  int offset = 0;
  for (int j = 0; j < count; j++) {
    parameterOffsets[j] = offset;
    offset += materials[j]->getNumParameters();
  }
  parameterOffsets[count] = offset;
  numTotalParameters = offset;

  has3rdOrderDerivative = true;
  for (int j = 0; j < count; j++) {
    if (!materials[j]->Has3rdOrderDerivative()) {
      has3rdOrderDerivative = false;
      break;
    }
  }
}

template<int count>
inline double ElasticModelCombinedMaterial<count>::compute_psi(std::span<const double> param,
  const SpectralState &state) const
{
  double energy = 0;
  for (int i = 0; i < count; i++) {
    energy += materials[i]->compute_psi(param.subspan(parameterOffsets[i], materials[i]->getNumParameters()), state);
  }

  return energy;
}

template<int count>
inline EigenSupport::M3d ElasticModelCombinedMaterial<count>::compute_P(std::span<const double> param,
  const SpectralState &state) const
{
  EigenSupport::M3d P = EigenSupport::M3d::Zero();
  for (int i = 0; i < count; i++) {
    P += materials[i]->compute_P(param.subspan(parameterOffsets[i], materials[i]->getNumParameters()), state);
  }
  return P;
}

template<int count>
inline EigenSupport::M9d ElasticModelCombinedMaterial<count>::compute_dPdF(std::span<const double> param,
  const SpectralState &state) const
{
  EigenSupport::M9d dPdFOut = EigenSupport::M9d::Zero();
  for (int i = 0; i < count; i++) {
    dPdFOut += materials[i]->compute_dPdF(param.subspan(parameterOffsets[i], materials[i]->getNumParameters()), state);
  }
  return dPdFOut;
}

template<int count>
inline void ElasticModelCombinedMaterial<count>::compute_dpsi_dparams(
  std::span<const double> param,
  const SpectralState &state,
  EigenSupport::RefVecXd dpsiDparam) const
{
  if (param.size() != static_cast<std::size_t>(numTotalParameters) ||
    dpsiDparam.size() != numTotalParameters)
    throw std::invalid_argument(
      "ElasticModelCombinedMaterial parameter-gradient dimensions do not match.");

  for (int mi = 0; mi < count; ++mi) {
    const int childCount = materials[mi]->getNumParameters();
    if (childCount == 0)
      continue;
    materials[mi]->compute_dpsi_dparams(
      param.subspan(parameterOffsets[mi], childCount),
      state,
      dpsiDparam.segment(parameterOffsets[mi], childCount));
  }
}

template<int count>
inline void ElasticModelCombinedMaterial<count>::compute_d2psi_dparams2(
  std::span<const double> param,
  const SpectralState &state,
  EigenSupport::RefMatXd d2psiDparam2) const
{
  if (param.size() != static_cast<std::size_t>(numTotalParameters) ||
    d2psiDparam2.rows() != numTotalParameters ||
    d2psiDparam2.cols() != numTotalParameters)
    throw std::invalid_argument(
      "ElasticModelCombinedMaterial parameter-Hessian dimensions do not match.");

  d2psiDparam2.setZero();
  for (int mi = 0; mi < count; ++mi) {
    const int childCount = materials[mi]->getNumParameters();
    if (childCount == 0)
      continue;
    materials[mi]->compute_d2psi_dparams2(
      param.subspan(parameterOffsets[mi], childCount),
      state,
      d2psiDparam2.block(
        parameterOffsets[mi], parameterOffsets[mi],
        childCount, childCount));
  }
}

template<int count>
inline void ElasticModelCombinedMaterial<count>::compute_dP_dparams(
  std::span<const double> param,
  const SpectralState &state,
  EigenSupport::RefMatXd dP_dparam) const
{
  if (param.size() != static_cast<std::size_t>(numTotalParameters) ||
    dP_dparam.rows() != 9 ||
    dP_dparam.cols() != numTotalParameters)
    throw std::invalid_argument(
      "ElasticModelCombinedMaterial P-Jacobian dimensions do not match.");

  for (int mi = 0; mi < count; ++mi) {
    const int childCount = materials[mi]->getNumParameters();
    if (childCount == 0)
      continue;
    materials[mi]->compute_dP_dparams(
      param.subspan(parameterOffsets[mi], childCount),
      state,
      dP_dparam.block(0, parameterOffsets[mi], 9, childCount));
  }
}

template<int count>
inline void ElasticModelCombinedMaterial<count>::compute_d2PdF2(std::span<const double> param,
  const SpectralState &state, EigenSupport::M81x9d &d2PdF2) const
{
  d2PdF2.setZero();
  for (int i = 0; i < count; i++) {
    EigenSupport::M81x9d temp;
    materials[i]->compute_d2PdF2(param.subspan(parameterOffsets[i], materials[i]->getNumParameters()), state, temp);
    d2PdF2 += temp;
  }
}

template<int count>
inline EigenSupport::M3d ElasticModelCombinedMaterial<count>::compute_d2Pdparam2(std::span<const double> param, int i, int j,
  const SpectralState &state) const
{
  int mi = 0;
  for (; mi < count; mi++) {
    if (i >= parameterOffsets[mi] && i < parameterOffsets[mi + 1])
      break;
  }

  EigenSupport::M3d d2p_dparam2 = EigenSupport::M3d::Zero();

  if (mi >= count)
    throw std::out_of_range("ElasticModelCombinedMaterial parameter index is out of range.");

  int mj = 0;
  for (; mj < count; mj++) {
    if (j >= parameterOffsets[mj] && j < parameterOffsets[mj + 1])
      break;
  }

  if (mj >= count)
    throw std::out_of_range("ElasticModelCombinedMaterial parameter index is out of range.");

  if (mi != mj)
    return d2p_dparam2;

  return materials[mi]->compute_d2Pdparam2(param.subspan(parameterOffsets[mi], materials[mi]->getNumParameters()), i - parameterOffsets[mi], j - parameterOffsets[mi],
    state);
}

template<int count>
inline EigenSupport::M9d ElasticModelCombinedMaterial<count>::compute_d2PdFdparam(std::span<const double> param, int i,
  const SpectralState &state) const
{
  int mi = 0;
  for (; mi < count; mi++) {
    if (i >= parameterOffsets[mi] && i < parameterOffsets[mi + 1])
      break;
  }

  if (mi >= count)
    throw std::out_of_range("ElasticModelCombinedMaterial parameter index is out of range.");

  return materials[mi]->compute_d2PdFdparam(param.subspan(parameterOffsets[mi], materials[mi]->getNumParameters()), i - parameterOffsets[mi], state);
}

class ElasticModelDynamicCombinedMaterial
  : public ElasticModel3DDeformationGradient
{
public:
  explicit ElasticModelDynamicCombinedMaterial(
    std::vector<std::unique_ptr<ElasticModel3DDeformationGradient>> mats);
  ~ElasticModelDynamicCombinedMaterial() override = default;

  virtual double compute_psi(std::span<const double> param,
    const SpectralState &state) const override;
  virtual EigenSupport::M3d compute_P(std::span<const double> param,
    const SpectralState &state) const override;
  virtual EigenSupport::M9d compute_dPdF(std::span<const double> param,
    const SpectralState &state) const override;

  virtual int getNumParameters() const override { return numTotalParameters; }
  void compute_dpsi_dparams(std::span<const double> param,
    const SpectralState &state,
    EigenSupport::RefVecXd dpsiDparam) const override;
  void compute_d2psi_dparams2(std::span<const double> param,
    const SpectralState &state,
    EigenSupport::RefMatXd d2psiDparam2) const override;
  void compute_dP_dparams(std::span<const double> param,
    const SpectralState &state,
    EigenSupport::RefMatXd dP_dparam) const override;

  virtual void compute_d2PdF2(std::span<const double> param,
    const SpectralState &state, EigenSupport::M81x9d &d2PdF2) const override;
  virtual EigenSupport::M3d compute_d2Pdparam2(std::span<const double> param, int i, int j,
    const SpectralState &state) const override;
  virtual EigenSupport::M9d compute_d2PdFdparam(std::span<const double> param, int i,
    const SpectralState &state) const override;

protected:
  std::vector<std::unique_ptr<ElasticModel3DDeformationGradient>> owned_;
  std::vector<const ElasticModel3DDeformationGradient *> materials;
  int count;

  std::vector<int> parameterOffsets;
  int numTotalParameters;
};

inline ElasticModelDynamicCombinedMaterial::
  ElasticModelDynamicCombinedMaterial(
  std::vector<std::unique_ptr<ElasticModel3DDeformationGradient>> mats)
  : count(static_cast<int>(mats.size()))
{
  owned_ = std::move(mats);
  materials.resize(count);
  parameterOffsets.resize(count + 1);

  int offset = 0;
  for (int i = 0; i < count; i++) {
    materials[i] = owned_[i].get();
    parameterOffsets[i] = offset;
    offset += materials[i]->getNumParameters();
  }
  parameterOffsets[count] = offset;
  numTotalParameters = offset;

  has3rdOrderDerivative = true;
  for (int i = 0; i < count; i++) {
    if (!materials[i]->Has3rdOrderDerivative()) {
      has3rdOrderDerivative = false;
      break;
    }
  }
}

inline double ElasticModelDynamicCombinedMaterial::compute_psi(std::span<const double> param,
  const SpectralState &state) const
{
  double energy = 0;
  for (int i = 0; i < count; i++) {
    energy += materials[i]->compute_psi(param.subspan(parameterOffsets[i], materials[i]->getNumParameters()), state);
  }

  return energy;
}

inline EigenSupport::M3d ElasticModelDynamicCombinedMaterial::compute_P(std::span<const double> param,
  const SpectralState &state) const
{
  EigenSupport::M3d P = EigenSupport::M3d::Zero();
  for (int i = 0; i < count; i++) {
    P += materials[i]->compute_P(param.subspan(parameterOffsets[i], materials[i]->getNumParameters()), state);
  }
  return P;
}

inline EigenSupport::M9d ElasticModelDynamicCombinedMaterial::compute_dPdF(std::span<const double> param,
  const SpectralState &state) const
{
  EigenSupport::M9d dPdFOut = EigenSupport::M9d::Zero();
  for (int i = 0; i < count; i++) {
    dPdFOut += materials[i]->compute_dPdF(param.subspan(parameterOffsets[i], materials[i]->getNumParameters()), state);
  }
  return dPdFOut;
}

inline void ElasticModelDynamicCombinedMaterial::compute_dpsi_dparams(
  std::span<const double> param,
  const SpectralState &state,
  EigenSupport::RefVecXd dpsiDparam) const
{
  if (param.size() != static_cast<std::size_t>(numTotalParameters) ||
    dpsiDparam.size() != numTotalParameters)
    throw std::invalid_argument(
      "ElasticModelDynamicCombinedMaterial parameter-gradient dimensions do not match.");

  for (int mi = 0; mi < count; ++mi) {
    const int childCount = materials[mi]->getNumParameters();
    if (childCount == 0)
      continue;
    materials[mi]->compute_dpsi_dparams(
      param.subspan(parameterOffsets[mi], childCount),
      state,
      dpsiDparam.segment(parameterOffsets[mi], childCount));
  }
}

inline void ElasticModelDynamicCombinedMaterial::compute_d2psi_dparams2(
  std::span<const double> param,
  const SpectralState &state,
  EigenSupport::RefMatXd d2psiDparam2) const
{
  if (param.size() != static_cast<std::size_t>(numTotalParameters) ||
    d2psiDparam2.rows() != numTotalParameters ||
    d2psiDparam2.cols() != numTotalParameters)
    throw std::invalid_argument(
      "ElasticModelDynamicCombinedMaterial parameter-Hessian dimensions do not match.");

  d2psiDparam2.setZero();
  for (int mi = 0; mi < count; ++mi) {
    const int childCount = materials[mi]->getNumParameters();
    if (childCount == 0)
      continue;
    materials[mi]->compute_d2psi_dparams2(
      param.subspan(parameterOffsets[mi], childCount),
      state,
      d2psiDparam2.block(
        parameterOffsets[mi], parameterOffsets[mi],
        childCount, childCount));
  }
}

inline void ElasticModelDynamicCombinedMaterial::compute_dP_dparams(
  std::span<const double> param,
  const SpectralState &state,
  EigenSupport::RefMatXd dP_dparam) const
{
  if (param.size() != static_cast<std::size_t>(numTotalParameters) ||
    dP_dparam.rows() != 9 ||
    dP_dparam.cols() != numTotalParameters)
    throw std::invalid_argument(
      "ElasticModelDynamicCombinedMaterial P-Jacobian dimensions do not match.");

  for (int mi = 0; mi < count; ++mi) {
    const int childCount = materials[mi]->getNumParameters();
    if (childCount == 0)
      continue;
    materials[mi]->compute_dP_dparams(
      param.subspan(parameterOffsets[mi], childCount),
      state,
      dP_dparam.block(0, parameterOffsets[mi], 9, childCount));
  }
}

inline void ElasticModelDynamicCombinedMaterial::compute_d2PdF2(std::span<const double> param,
  const SpectralState &state, EigenSupport::M81x9d &d2PdF2) const
{
  d2PdF2.setZero();
  for (int i = 0; i < count; i++) {
    EigenSupport::M81x9d temp;
    materials[i]->compute_d2PdF2(param.subspan(parameterOffsets[i], materials[i]->getNumParameters()), state, temp);
    d2PdF2 += temp;
  }
}

inline EigenSupport::M3d ElasticModelDynamicCombinedMaterial::compute_d2Pdparam2(std::span<const double> param, int i, int j,
  const SpectralState &state) const
{
  int mi = 0;
  for (; mi < count; mi++) {
    if (i >= parameterOffsets[mi] && i < parameterOffsets[mi + 1])
      break;
  }

  EigenSupport::M3d d2p_dparam2 = EigenSupport::M3d::Zero();

  if (mi >= count)
    throw std::out_of_range("ElasticModelDynamicCombinedMaterial parameter index is out of range.");

  int mj = 0;
  for (; mj < count; mj++) {
    if (j >= parameterOffsets[mj] && j < parameterOffsets[mj + 1])
      break;
  }

  if (mj >= count)
    throw std::out_of_range("ElasticModelDynamicCombinedMaterial parameter index is out of range.");

  if (mi != mj)
    return d2p_dparam2;

  return materials[mi]->compute_d2Pdparam2(param.subspan(parameterOffsets[mi], materials[mi]->getNumParameters()), i - parameterOffsets[mi], j - parameterOffsets[mi],
    state);
}

inline EigenSupport::M9d ElasticModelDynamicCombinedMaterial::compute_d2PdFdparam(std::span<const double> param, int i,
  const SpectralState &state) const
{
  int mi = 0;
  for (; mi < count; mi++) {
    if (i >= parameterOffsets[mi] && i < parameterOffsets[mi + 1])
      break;
  }

  if (mi >= count)
    throw std::out_of_range("ElasticModelDynamicCombinedMaterial parameter index is out of range.");

  return materials[mi]->compute_d2PdFdparam(param.subspan(parameterOffsets[mi], materials[mi]->getNumParameters()), i - parameterOffsets[mi], state);
}

class StVKVolumeDefinition final : public ElasticModelDefinition
{
public:
  std::string_view id() const override { return "stvk_vol"; }
  int numOptimizableChannels() const override { return 0; }
  int numFixedChannels() const override { return 3; }
  std::unique_ptr<ElasticModel> createModel(std::span<const double>, const MaterialFrame &) const override;
private:
};
}  // namespace SolidDeformationModel
}  // namespace pgo
