#pragma once

#include <memory>
#include <stdexcept>
#include <typeindex>
#include <unordered_map>
#include <utility>
#include <vector>

namespace pgo::SolidDeformationModel
{

template<class T>
class ElementField
{
public:
  static ElementField uniform(int numElements, T value)
  {
    if (numElements < 0) throw std::invalid_argument("element field size must be non-negative");
    ElementField result;
    result.palette_.emplace_back(std::make_shared<const T>(std::move(value)));
    result.elementToPalette_.assign(static_cast<std::size_t>(numElements), 0);
    return result;
  }

  static ElementField fromValues(std::vector<T> values)
  {
    std::vector<std::shared_ptr<const T>> handles;
    handles.reserve(values.size());
    for (auto &value : values) handles.emplace_back(std::make_shared<const T>(std::move(value)));
    return fromShared(static_cast<int>(handles.size()), std::move(handles));
  }

  static ElementField fromPalette(
    std::vector<std::shared_ptr<const T>> palette,
    std::vector<int> elementToPalette)
  {
    if (palette.empty() && !elementToPalette.empty()) throw std::invalid_argument("element field palette cannot be empty");
    for (const auto &value : palette)
      if (!value) throw std::invalid_argument("element field palette cannot contain null handles");
    for (int index : elementToPalette)
      if (index < 0 || index >= static_cast<int>(palette.size())) throw std::invalid_argument("element field palette index out of range");
    ElementField result;
    result.palette_ = std::move(palette);
    result.elementToPalette_ = std::move(elementToPalette);
    return result;
  }

  static ElementField fromShared(int numElements, std::vector<std::shared_ptr<const T>> elementValues)
  {
    if (numElements < 0 || static_cast<int>(elementValues.size()) != numElements)
      throw std::invalid_argument("element field value count must match element count");
    ElementField result;
    std::unordered_map<const T *, int> indices;
    result.elementToPalette_.reserve(elementValues.size());
    for (const auto &value : elementValues) {
      if (!value) throw std::invalid_argument("element field cannot contain null handles");
      const auto *key = value.get();
      auto [it, inserted] = indices.emplace(key, static_cast<int>(result.palette_.size()));
      if (inserted) result.palette_.emplace_back(value);
      result.elementToPalette_.push_back(it->second);
    }
    return result;
  }

  int size() const { return static_cast<int>(elementToPalette_.size()); }
  const T &at(int element) const
  {
    if (element < 0 || element >= size()) throw std::out_of_range("element field element index out of range");
    return *palette_[elementToPalette_[element]];
  }

private:
  std::vector<std::shared_ptr<const T>> palette_;
  std::vector<int> elementToPalette_;
};

class ElementFieldStore
{
public:
  ElementFieldStore() = default;
  ElementFieldStore(ElementFieldStore &&) noexcept = default;
  ElementFieldStore &operator=(ElementFieldStore &&) noexcept = default;
  ElementFieldStore(const ElementFieldStore &) = delete;
  ElementFieldStore &operator=(const ElementFieldStore &) = delete;

  template<class T> void add(ElementField<T> field)
  {
    const std::type_index key(typeid(T));
    if (fields_.count(key)) throw std::invalid_argument("element field store already contains this exact type");
    if (numElements_ >= 0 && field.size() != numElements_) throw std::invalid_argument("all element fields must have the same element count");
    if (numElements_ < 0) numElements_ = field.size();
    fields_.emplace(key, std::make_unique<Holder<T>>(std::move(field)));
  }

  template<class T> const ElementField<T> &require() const
  {
    const auto it = fields_.find(std::type_index(typeid(T)));
    if (it == fields_.end()) throw std::invalid_argument("required element field type is missing");
    return static_cast<const Holder<T> &>(*it->second).field;
  }
  template<class T> bool contains() const { return fields_.count(std::type_index(typeid(T))) != 0; }
  void validateSize(int numElements) const
  {
    if (numElements_ >= 0 && numElements_ != numElements) throw std::invalid_argument("element field store size must match the mesh element count");
  }

private:
  struct Concept { virtual ~Concept() = default; };
  template<class T> struct Holder final : Concept
  {
    explicit Holder(ElementField<T> value): field(std::move(value)) {}
    ElementField<T> field;
  };
  std::unordered_map<std::type_index, std::unique_ptr<Concept>> fields_;
  int numElements_ = -1;
};

}  // namespace pgo::SolidDeformationModel
