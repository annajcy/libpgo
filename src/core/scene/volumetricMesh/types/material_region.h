#pragma once

namespace pgo::VolumetricMeshes {

class MaterialRegion {
public:
    MaterialRegion() = default;
    MaterialRegion(int material_index, int set_index);

    int getMaterialIndex() const { return materialIndex; }
    int getSetIndex() const { return setIndex; }
    void setMaterialIndex(int index) { materialIndex = index; }
    void setSetIndex(int index) { setIndex = index; }

private:
    int setIndex = -1;
    int materialIndex = -1;
};

inline MaterialRegion::MaterialRegion(int material_index, int set_index)
    : setIndex(set_index), materialIndex(material_index) {}

}  // namespace pgo::VolumetricMeshes
