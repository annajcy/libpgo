#pragma once

#include <memory>
#include <stdexcept>
#include <typeindex>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

namespace pgo
{
namespace SolidDeformationModel
{

/// A read-only element field.  Repeated shared handles are stored once in a
/// palette and referenced by element index; equal-but-distinct values are not
/// compared or merged.
template<class T>
class ElementField
{
public:
  static ElementField uniform(int numElements, T value)
  {
    if (numElements < 0)
      throw std::invalid_argument("element field size must be non-negative");
    ElementField result;
    result.palette_.emplace_back(std::make_shared<const T>(std::move(value)));
    result.elementToPalette_.assign(static_cast<std::size_t>(numElements), 0);
    return result;
  }

  static ElementField fromValues(std::vector<T> values)
  {
    std::vector<std::shared_ptr<const T>> handles;
    handles.reserve(values.size());
    for (auto &value : values)
      handles.emplace_back(std::make_shared<const T>(std::move(value)));
    return fromShared(static_cast<int>(handles.size()), std::move(handles));
  }

  /// Construct from a palette and per-element palette indices.  This is the
  /// lossless path used by mesh loaders when material slots are shared.
  static ElementField fromPalette(
    std::vector<std::shared_ptr<const T>> palette,
    std::vector<int> elementToPalette)
  {
    if (palette.empty() && !elementToPalette.empty())
      throw std::invalid_argument("element field palette cannot be empty");
    for (const auto &value : palette)
      if (!value)
        throw std::invalid_argument("element field palette cannot contain null handles");
    for (int index : elementToPalette)
      if (index < 0 || index >= static_cast<int>(palette.size()))
        throw std::invalid_argument("element field palette index out of range");
    ElementField result;
    result.palette_ = std::move(palette);
    result.elementToPalette_ = std::move(elementToPalette);
    return result;
  }

  /// Construct from a shared handle for each element.  Repeated pointer
  /// identities are compressed into one palette entry.
  static ElementField fromShared(
    int numElements, std::vector<std::shared_ptr<const T>> elementValues)
  {
    if (numElements < 0 || static_cast<int>(elementValues.size()) != numElements)
      throw std::invalid_argument("element field value count must match element count");
    ElementField result;
    std::unordered_map<const T *, int> indices;
    result.elementToPalette_.reserve(elementValues.size());
    for (const auto &value : elementValues) {
      if (!value)
        throw std::invalid_argument("element field cannot contain null handles");
      const auto *key = value.get();
      auto [it, inserted] = indices.emplace(key, static_cast<int>(result.palette_.size()));
      if (inserted)
        result.palette_.emplace_back(value);
      result.elementToPalette_.push_back(it->second);
    }
    return result;
  }

  int size() const { return static_cast<int>(elementToPalette_.size()); }

  const T &at(int element) const
  {
    if (element < 0 || element >= size())
      throw std::out_of_range("element field element index out of range");
    return *palette_[elementToPalette_[element]];
  }

private:
  std::vector<std::shared_ptr<const T>> palette_;
  std::vector<int> elementToPalette_;
};

/// Type-erased collection of immutable element fields.  The exact C++ type
/// is the only key, so a field can never be selected by a string or enum tag.
class ElementFieldStore
{
public:
  ElementFieldStore() = default;
  ElementFieldStore(ElementFieldStore &&) noexcept = default;
  ElementFieldStore &operator=(ElementFieldStore &&) noexcept = default;
  ElementFieldStore(const ElementFieldStore &) = delete;
  ElementFieldStore &operator=(const ElementFieldStore &) = delete;

  template<class T>
  void add(ElementField<T> field)
  {
    const std::type_index key(typeid(T));
    if (fields_.count(key))
      throw std::invalid_argument("element field store already contains this exact type");
    if (numElements_ >= 0 && field.size() != numElements_)
      throw std::invalid_argument("all element fields must have the same element count");
    if (numElements_ < 0)
      numElements_ = field.size();
    fields_.emplace(key, std::make_unique<Holder<T>>(std::move(field)));
  }

  template<class T>
  const ElementField<T> &require() const
  {
    const auto it = fields_.find(std::type_index(typeid(T)));
    if (it == fields_.end())
      throw std::invalid_argument("required element field type is missing");
    return static_cast<const Holder<T> &>(*it->second).field;
  }

  template<class T>
  bool contains() const
  {
    return fields_.count(std::type_index(typeid(T))) != 0;
  }

  void validateSize(int numElements) const
  {
    if (numElements_ >= 0 && numElements_ != numElements)
      throw std::invalid_argument("element field store size must match the mesh element count");
  }

private:
  struct Concept
  {
    virtual ~Concept() = default;
    virtual int size() const = 0;
  };
  template<class T>
  struct Holder final : Concept
  {
    explicit Holder(ElementField<T> value): field(std::move(value)) {}
    int size() const override { return field.size(); }
    ElementField<T> field;
  };

  std::unordered_map<std::type_index, std::unique_ptr<Concept>> fields_;
  int numElements_ = -1;
};

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
  SimulationMeshMooneyRivlinMaterial(int N, int M, const double *Cpq, const double *D_):
    N(N), M(M), C((N + 1) * (N + 1)), D(M)
  {
    for (int p = 0; p <= N; p++)
      for (int q = 0; q <= N; q++)
        C[q * (N + 1) + p] = Cpq[q * (N + 1) + p];
    for (int i = 0; i < M; i++) D[i] = D_[i];
  }
  SimulationMeshMooneyRivlinMaterial(int N, int M, double E, double nu):
    N(N), M(M), C((N + 1) * (N + 1), 0.0), D(M, 0.0)
  {
    const double bulkModulus = E / (3 * (1 - 2 * nu));
    const double shearModulus = E / (2 * (1 + nu));
    getC(1, 0) = shearModulus * 0.5;
    getD(0) = 2.0 / bulkModulus;
  }
  double getC(int p, int q) const { return C[q * (N + 1) + p]; }
  double getD(int i) const { return D[i]; }
  const double *getC() const { return C.data(); }
  const double *getD() const { return D.data(); }
  int getM() const { return M; }
  int getN() const { return N; }
protected:
  double &getC(int p, int q) { return C[q * (N + 1) + p]; }
  double &getD(int i) { return D[i]; }
  int N, M;
  std::vector<double> C, D;
};

class SimulationMeshMooneyRivlinhMaterial : public SimulationMeshMooneyRivlinMaterial
{
public:
  SimulationMeshMooneyRivlinhMaterial(int N_, int M_, const double *Cpq_, const double *D_, double h_):
    SimulationMeshMooneyRivlinMaterial(N_, M_, Cpq_, D_), h(h_) {}
  SimulationMeshMooneyRivlinhMaterial(int N_, int M_, double E_, double nu_, double h_):
    SimulationMeshMooneyRivlinMaterial(N_, M_, E_, nu_), h(h_) {}
  double geth() const { return h; }
private:
  double h = 1e-4;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
