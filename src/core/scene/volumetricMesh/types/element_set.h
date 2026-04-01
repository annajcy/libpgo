#pragma once

#include <set>
#include <string>

namespace pgo::VolumetricMeshes {

class ElementSet {
public:
    ElementSet() = default;
    explicit ElementSet(const std::string& name);
    ElementSet(const ElementSet& set);
    ElementSet(const std::string& name, const std::set<int>& elements);

    ElementSet& operator=(const ElementSet& set);

    std::string getName() const { return name; }
    int getNumElements() const { return static_cast<int>(elements.size()); }
    void getElements(std::set<int>& output_elements) const { output_elements = elements; }
    const std::set<int>& getElements() const { return elements; }
    bool isMember(int element) const;

    std::set<int>& getElements() { return elements; }
    void insert(int element);
    void clear();

private:
    std::string name;
    std::set<int> elements;
};

inline ElementSet::ElementSet(const std::string& name_) {
    name = name_;
}

inline ElementSet::ElementSet(const ElementSet& set) {
    elements = set.elements;
    name = set.getName();
}

inline ElementSet::ElementSet(const std::string& name_, const std::set<int>& elements_)
    : name(name_), elements(elements_) {}

inline ElementSet& ElementSet::operator=(const ElementSet& set) {
    elements = set.elements;
    name = set.getName();
    return *this;
}

inline bool ElementSet::isMember(int element) const {
    return elements.find(element) != elements.end();
}

inline void ElementSet::insert(int element) {
    elements.insert(element);
}

inline void ElementSet::clear() {
    elements.clear();
}

}  // namespace pgo::VolumetricMeshes
