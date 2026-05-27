#include "vegFile.h"

#include "cubicMesh.h"
#include "tetMesh.h"
#include "volumetricMeshENuMaterial.h"
#include "volumetricMeshMooneyRivlinMaterial.h"
#include "volumetricMeshOrthotropicMaterial.h"
#include "volumetricMeshParser.h"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>

namespace pgo::VolumetricMeshes
{
namespace
{
using VM = VolumetricMesh;

std::string trim(const std::string &text)
{
  const auto begin = text.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos)
    return {};
  const auto end = text.find_last_not_of(" \t\r\n");
  return text.substr(begin, end - begin + 1);
}

bool startsWith(const std::string &text, const char *prefix)
{
  return text.rfind(prefix, 0) == 0;
}

std::vector<std::string> readAsciiLines(const std::filesystem::path &path)
{
  VolumetricMeshParser parser;
  if (parser.open(path.string().c_str()) != 0)
    throw std::runtime_error("Failed to open veg file: " + path.string());

  std::vector<std::string> lines;
  char lineBuffer[1024];
  try {
    while (parser.getNextLine(lineBuffer, 0, 0) != nullptr)
      lines.emplace_back(lineBuffer);
  }
  catch (...) {
    parser.close();
    throw;
  }
  parser.close();
  return lines;
}

std::vector<double> parseDoubles(std::string text)
{
  for (char &ch : text) {
    if (ch == ',' || ch == '\t')
      ch = ' ';
  }
  std::istringstream in(text);
  std::vector<double> values;
  double value;
  while (in >> value)
    values.push_back(value);
  return values;
}

std::vector<int> parseInts(std::string text)
{
  for (char &ch : text) {
    if (ch == ',' || ch == '\t')
      ch = ' ';
  }
  std::istringstream in(text);
  std::vector<int> values;
  int value;
  while (in >> value)
    values.push_back(value);
  return values;
}

std::string compact(std::string text)
{
  text.erase(std::remove_if(text.begin(), text.end(), [](unsigned char ch) {
    return std::isspace(ch) != 0;
  }), text.end());
  return text;
}

VegMaterialPayload parseMaterial(const std::string &name, const std::string &spec, const std::filesystem::path &path)
{
  const std::string trimmedSpec = trim(spec);
  const auto comma = trimmedSpec.find(',');
  if (comma == std::string::npos)
    throw std::runtime_error("Malformed material in " + path.string() + ": " + spec);

  const std::string type = compact(trimmedSpec.substr(0, comma));
  const std::vector<double> values = parseDoubles(trimmedSpec.substr(comma + 1));

  if (type == "ENU") {
    if (values.size() < 3)
      throw std::runtime_error("ENU material requires density, E, and nu in " + path.string());
    return VegENuMaterialPayload{ name, values[0], values[1], values[2] };
  }

  if (startsWith(type, "MOONEYRIVLIN")) {
    if (values.size() < 4)
      throw std::runtime_error("Mooney-Rivlin material requires density, mu01, mu10, and v1 in " + path.string());
    return VegMooneyRivlinMaterialPayload{ name, values[0], values[1], values[2], values[3] };
  }

  if (startsWith(type, "ORTHOTROPIC")) {
    if (values.size() < 10)
      throw std::runtime_error("Orthotropic material requires at least density, E1/E2/E3, nu12/nu23/nu31, and G12/G23/G31 in " + path.string());
    VegOrthotropicMaterialPayload payload;
    payload.name = name;
    payload.density = values[0];
    payload.E1 = values[1];
    payload.E2 = values[2];
    payload.E3 = values[3];
    payload.nu12 = values[4];
    payload.nu23 = values[5];
    payload.nu31 = values[6];
    payload.G12 = values[7];
    payload.G23 = values[8];
    payload.G31 = values[9];
    if (values.size() >= 19) {
      for (int i = 0; i < 9; ++i)
        payload.R[static_cast<size_t>(i)] = values[static_cast<size_t>(10 + i)];
    }
    return payload;
  }

  throw std::runtime_error("Unsupported material type in " + path.string() + ": " + type);
}

void assignFallbackRegions(VegFilePayload &payload, int numElements)
{
  if (payload.sets.empty()) {
    VegSetPayload all;
    all.name = "allElements";
    all.elements.resize(static_cast<size_t>(numElements));
    for (int i = 0; i < numElements; ++i)
      all.elements[static_cast<size_t>(i)] = i;
    payload.sets.push_back(std::move(all));
  }

  std::vector<char> assigned(static_cast<size_t>(numElements), 0);
  for (const VegRegionPayload &region : payload.regions) {
    if (region.materialIndex < 0 || region.materialIndex >= static_cast<int>(payload.materials.size()))
      throw std::runtime_error("Veg region references an out-of-range material");
    if (region.setIndex < 0 || region.setIndex >= static_cast<int>(payload.sets.size()))
      throw std::runtime_error("Veg region references an out-of-range set");
    for (int element : payload.sets[static_cast<size_t>(region.setIndex)].elements) {
      if (element < 0 || element >= numElements)
        throw std::runtime_error("Veg set references an out-of-range element");
      assigned[static_cast<size_t>(element)] = 1;
    }
  }

  std::vector<int> unassigned;
  for (int i = 0; i < numElements; ++i) {
    if (!assigned[static_cast<size_t>(i)])
      unassigned.push_back(i);
  }
  if (unassigned.empty())
    return;

  if (payload.materials.empty()) {
    payload.materials.push_back(VegENuMaterialPayload{
      "defaultMaterial", VM::density_default, VM::E_default, VM::nu_default });
  }

  const int setIndex = static_cast<int>(payload.sets.size());
  payload.sets.push_back(VegSetPayload{ "unassignedSet", std::move(unassigned) });
  payload.regions.push_back(VegRegionPayload{
    static_cast<int>(payload.materials.size()) - 1, setIndex });
}

VegFilePayload readAsciiVegFile(const std::filesystem::path &path)
{
  const std::vector<std::string> lines = readAsciiLines(path);
  VegFilePayload payload;
  std::vector<Vec3d> vertices;
  std::vector<int> elements;
  int elementWidth = 0;
  int parsedNumElements = 0;
  int oneIndexedVertices = 1;
  int oneIndexedElements = 1;
  std::map<std::string, int> materialMap;
  std::map<std::string, int> setMap;

  size_t i = 0;
  while (i < lines.size()) {
    const std::string line = trim(lines[i]);

    if (startsWith(line, "*VERTICES")) {
      if (++i >= lines.size())
        throw std::runtime_error("Missing *VERTICES header in " + path.string());
      const std::vector<int> header = parseInts(lines[i++]);
      if (header.empty() || header[0] < 0)
        throw std::runtime_error("Invalid vertex count in " + path.string());
      vertices.reserve(static_cast<size_t>(header[0]));
      for (int row = 0; row < header[0] && i < lines.size(); ++row, ++i) {
        const std::vector<double> values = parseDoubles(lines[i]);
        if (values.size() < 4)
          throw std::runtime_error("Malformed vertex line in " + path.string() + ": " + lines[i]);
        if (static_cast<int>(values[0]) == 0)
          oneIndexedVertices = 0;
        vertices.emplace_back(values[1], values[2], values[3]);
      }
      continue;
    }

    if (startsWith(line, "*ELEMENTS")) {
      if (++i >= lines.size())
        throw std::runtime_error("Missing element type in " + path.string());
      const std::string type = compact(lines[i++]);
      if (type == "TET")
        elementWidth = 4;
      else if (type == "CUBIC")
        elementWidth = 8;
      else
        throw std::runtime_error("Unsupported element type in " + path.string() + ": " + type);

      if (i >= lines.size())
        throw std::runtime_error("Missing element count in " + path.string());
      const std::vector<int> header = parseInts(lines[i++]);
      if (header.empty() || header[0] < 0)
        throw std::runtime_error("Invalid element count in " + path.string());
      parsedNumElements = header[0];
      elements.reserve(static_cast<size_t>(header[0] * elementWidth));
      for (int row = 0; row < header[0] && i < lines.size(); ++row, ++i) {
        const std::vector<int> values = parseInts(lines[i]);
        if (static_cast<int>(values.size()) < elementWidth + 1)
          throw std::runtime_error("Malformed element line in " + path.string() + ": " + lines[i]);
        if (values[0] == 0)
          oneIndexedElements = 0;
        for (int local = 0; local < elementWidth; ++local)
          elements.push_back(values[static_cast<size_t>(local + 1)] - oneIndexedVertices);
      }
      if (payload.sets.empty()) {
        VegSetPayload all;
        all.name = "allElements";
        all.elements.resize(static_cast<size_t>(parsedNumElements));
        for (int element = 0; element < parsedNumElements; ++element)
          all.elements[static_cast<size_t>(element)] = element;
        setMap[all.name] = 0;
        payload.sets.push_back(std::move(all));
      }
      continue;
    }

    if (startsWith(line, "*MATERIAL")) {
      const std::string name = compact(line.substr(9));
      if (++i >= lines.size())
        throw std::runtime_error("Missing material payload in " + path.string());
      materialMap[name] = static_cast<int>(payload.materials.size());
      payload.materials.push_back(parseMaterial(name, lines[i++], path));
      continue;
    }

    if (startsWith(line, "*SET")) {
      VegSetPayload set;
      set.name = compact(line.substr(4));
      while (++i < lines.size()) {
        if (startsWith(trim(lines[i]), "*"))
          break;
        for (int element : parseInts(lines[i]))
          set.elements.push_back(element - oneIndexedElements);
      }
      std::sort(set.elements.begin(), set.elements.end());
      set.elements.erase(std::unique(set.elements.begin(), set.elements.end()), set.elements.end());
      setMap[set.name] = static_cast<int>(payload.sets.size());
      payload.sets.push_back(std::move(set));
      continue;
    }

    if (startsWith(line, "*REGION")) {
      if (++i >= lines.size())
        throw std::runtime_error("Missing region payload in " + path.string());
      std::string spec = compact(lines[i++]);
      const auto comma = spec.find(',');
      if (comma == std::string::npos)
        throw std::runtime_error("Malformed region line in " + path.string() + ": " + spec);
      const std::string setName = spec.substr(0, comma);
      const std::string materialName = spec.substr(comma + 1);
      auto setIt = setMap.find(setName);
      auto materialIt = materialMap.find(materialName);
      if (setIt == setMap.end())
        throw std::runtime_error("Region references unknown set in " + path.string() + ": " + setName);
      if (materialIt == materialMap.end())
        throw std::runtime_error("Region references unknown material in " + path.string() + ": " + materialName);
      payload.regions.push_back(VegRegionPayload{ materialIt->second, setIt->second });
      continue;
    }

    ++i;
  }

  if (elementWidth == 4)
    payload.meshData = Mesh::MeshData<4>::fromFlatElements(std::move(vertices), std::move(elements));
  else if (elementWidth == 8)
    payload.meshData = Mesh::MeshData<8>::fromFlatElements(std::move(vertices), std::move(elements));
  else
    throw std::runtime_error("No *ELEMENTS section found in " + path.string());

  assignFallbackRegions(payload, parsedNumElements);
  return payload;
}

std::unique_ptr<VM::Material> makeMaterial(const VegMaterialPayload &payload)
{
  return std::visit([](const auto &material) -> std::unique_ptr<VM::Material> {
    using T = std::decay_t<decltype(material)>;
    if constexpr (std::is_same_v<T, VegENuMaterialPayload>) {
      return std::make_unique<VM::ENuMaterial>(material.name, material.density, material.E, material.nu);
    }
    else if constexpr (std::is_same_v<T, VegMooneyRivlinMaterialPayload>) {
      return std::make_unique<VM::MooneyRivlinMaterial>(
        material.name, material.density, material.mu01, material.mu10, material.v1);
    }
    else {
      return std::make_unique<VM::OrthotropicMaterial>(
        material.name, material.density,
        material.E1, material.E2, material.E3,
        material.nu12, material.nu23, material.nu31,
        material.G12, material.G23, material.G31,
        const_cast<double *>(material.R.data()));
    }
  }, payload);
}

std::vector<VM::Set> makeSets(const std::vector<VegSetPayload> &payloads)
{
  std::vector<VM::Set> sets;
  sets.reserve(payloads.size());
  for (const VegSetPayload &payload : payloads)
    sets.emplace_back(payload.name, std::set<int>(payload.elements.begin(), payload.elements.end()));
  return sets;
}

std::vector<VM::Region> makeRegions(const std::vector<VegRegionPayload> &payloads)
{
  std::vector<VM::Region> regions;
  regions.reserve(payloads.size());
  for (const VegRegionPayload &payload : payloads)
    regions.emplace_back(payload.materialIndex, payload.setIndex);
  return regions;
}

template<class T>
void readBinary(FILE *file, T &value, const std::filesystem::path &path, const char *field)
{
  if (fread(&value, sizeof(T), 1, file) != 1)
    throw std::runtime_error("Failed to read binary veg field " + std::string(field) + " from " + path.string());
}

template<class T>
void readBinaryArray(FILE *file, T *values, size_t count, const std::filesystem::path &path, const char *field)
{
  if (count > 0 && fread(values, sizeof(T), count, file) != count)
    throw std::runtime_error("Failed to read binary veg field " + std::string(field) + " from " + path.string());
}

std::string readBinaryString(FILE *file, const std::filesystem::path &path, const char *field)
{
  int length = 0;
  readBinary(file, length, path, field);
  if (length < 0)
    throw std::runtime_error("Negative string length in binary veg field " + std::string(field));
  std::string value(static_cast<size_t>(length), '\0');
  readBinaryArray(file, value.data(), static_cast<size_t>(length), path, field);
  return value;
}

VegFilePayload readBinaryVegFile(const std::filesystem::path &path)
{
  FILE *file = fopen(path.string().c_str(), "rb");
  if (file == nullptr)
    throw std::runtime_error("Failed to open binary veg file: " + path.string());

  try {
    double version = 0.0;
    readBinary(file, version, path, "version");

    int elementType = 0;
    readBinary(file, elementType, path, "elementType");
    int elementWidth = 0;
    if (elementType == VM::TET)
      elementWidth = 4;
    else if (elementType == VM::CUBIC)
      elementWidth = 8;
    else
      throw std::runtime_error("Unsupported binary veg element type in " + path.string());

    int numVertices = 0;
    readBinary(file, numVertices, path, "numVertices");
    if (numVertices < 0)
      throw std::runtime_error("Negative vertex count in " + path.string());
    std::vector<Vec3d> vertices(static_cast<size_t>(numVertices));
    readBinaryArray(
      file, reinterpret_cast<double *>(vertices.data()),
      static_cast<size_t>(numVertices) * 3, path, "vertices");

    int numElements = 0;
    readBinary(file, numElements, path, "numElements");
    if (numElements < 0)
      throw std::runtime_error("Negative element count in " + path.string());
    int fileElementWidth = 0;
    readBinary(file, fileElementWidth, path, "numElementVertices");
    if (fileElementWidth != elementWidth)
      throw std::runtime_error("Binary veg element width does not match element type in " + path.string());
    std::vector<int> elements(static_cast<size_t>(numElements * elementWidth));
    readBinaryArray(file, elements.data(), elements.size(), path, "elements");

    VegFilePayload payload;
    if (elementWidth == 4)
      payload.meshData = Mesh::MeshData<4>::fromFlatElements(std::move(vertices), std::move(elements));
    else
      payload.meshData = Mesh::MeshData<8>::fromFlatElements(std::move(vertices), std::move(elements));

    int numMaterials = 0;
    readBinary(file, numMaterials, path, "numMaterials");
    if (numMaterials < 0)
      throw std::runtime_error("Negative material count in " + path.string());
    for (int materialIndex = 0; materialIndex < numMaterials; ++materialIndex) {
      const std::string name = readBinaryString(file, path, "materialName");
      int materialType = 0;
      readBinary(file, materialType, path, "materialType");
      if (materialType == VM::Material::ENU) {
        double properties[VM::Material::ENU_NUM_PROPERTIES];
        readBinaryArray(file, properties, VM::Material::ENU_NUM_PROPERTIES, path, "enuMaterial");
        payload.materials.push_back(VegENuMaterialPayload{
          name,
          properties[VM::Material::ENU_DENSITY],
          properties[VM::Material::ENU_E],
          properties[VM::Material::ENU_NU],
        });
      }
      else if (materialType == VM::Material::MOONEYRIVLIN) {
        double properties[VM::Material::MOONEYRIVLIN_NUM_PROPERTIES];
        readBinaryArray(file, properties, VM::Material::MOONEYRIVLIN_NUM_PROPERTIES, path, "mooneyRivlinMaterial");
        payload.materials.push_back(VegMooneyRivlinMaterialPayload{
          name,
          properties[VM::Material::MOONEYRIVLIN_DENSITY],
          properties[VM::Material::MOONEYRIVLIN_MU01],
          properties[VM::Material::MOONEYRIVLIN_MU10],
          properties[VM::Material::MOONEYRIVLIN_V1],
        });
      }
      else if (materialType == VM::Material::ORTHOTROPIC) {
        double properties[VM::Material::ORTHOTROPIC_NUM_PROPERTIES];
        double R[9];
        readBinaryArray(file, properties, VM::Material::ORTHOTROPIC_NUM_PROPERTIES, path, "orthotropicMaterial");
        readBinaryArray(file, R, 9, path, "orthotropicR");
        VegOrthotropicMaterialPayload material;
        material.name = name;
        material.density = properties[VM::Material::ORTHOTROPIC_DENSITY];
        material.E1 = properties[VM::Material::ORTHOTROPIC_E1];
        material.E2 = properties[VM::Material::ORTHOTROPIC_E2];
        material.E3 = properties[VM::Material::ORTHOTROPIC_E3];
        material.nu12 = properties[VM::Material::ORTHOTROPIC_NU12];
        material.nu23 = properties[VM::Material::ORTHOTROPIC_NU23];
        material.nu31 = properties[VM::Material::ORTHOTROPIC_NU31];
        material.G12 = properties[VM::Material::ORTHOTROPIC_G12];
        material.G23 = properties[VM::Material::ORTHOTROPIC_G23];
        material.G31 = properties[VM::Material::ORTHOTROPIC_G31];
        std::copy(R, R + 9, material.R.begin());
        payload.materials.push_back(material);
      }
      else {
        throw std::runtime_error("Unsupported material type in binary veg file: " + path.string());
      }
    }

    int numSets = 0;
    readBinary(file, numSets, path, "numSets");
    if (numSets <= 0)
      throw std::runtime_error("Binary veg file must contain at least the allElements set: " + path.string());
    VegSetPayload all;
    all.name = "allElements";
    all.elements.resize(static_cast<size_t>(numElements));
    for (int element = 0; element < numElements; ++element)
      all.elements[static_cast<size_t>(element)] = element;
    payload.sets.push_back(std::move(all));
    for (int setIndex = 1; setIndex < numSets; ++setIndex) {
      VegSetPayload set;
      set.name = readBinaryString(file, path, "setName");
      int cardinality = 0;
      readBinary(file, cardinality, path, "setCardinality");
      if (cardinality < 0)
        throw std::runtime_error("Negative set cardinality in " + path.string());
      set.elements.resize(static_cast<size_t>(cardinality));
      readBinaryArray(file, set.elements.data(), set.elements.size(), path, "setElements");
      payload.sets.push_back(std::move(set));
    }

    int numRegions = 0;
    readBinary(file, numRegions, path, "numRegions");
    if (numRegions < 0)
      throw std::runtime_error("Negative region count in " + path.string());
    for (int regionIndex = 0; regionIndex < numRegions; ++regionIndex) {
      VegRegionPayload region;
      readBinary(file, region.materialIndex, path, "regionMaterialIndex");
      readBinary(file, region.setIndex, path, "regionSetIndex");
      payload.regions.push_back(region);
    }

    assignFallbackRegions(payload, numElements);
    fclose(file);
    return payload;
  }
  catch (...) {
    fclose(file);
    throw;
  }
}
}  // namespace

VegFilePayload readVegFile(const std::filesystem::path &path)
{
  const VM::fileFormatType fileType = VM::getFileFormatTypeByExt(path.string().c_str());
  if (fileType == VM::BINARY)
    return readBinaryVegFile(path);
  return readAsciiVegFile(path);
}

void writeVegFile(const std::filesystem::path &path, const VegFilePayload &payload)
{
  std::vector<std::unique_ptr<VM::Material>> materials;
  std::vector<const VM::Material *> materialPtrs;
  materials.reserve(payload.materials.size());
  materialPtrs.reserve(payload.materials.size());
  for (const VegMaterialPayload &material : payload.materials) {
    materials.push_back(makeMaterial(material));
    materialPtrs.push_back(materials.back().get());
  }
  std::vector<VM::Set> sets = makeSets(payload.sets);
  std::vector<VM::Region> regions = makeRegions(payload.regions);

  const int result = std::visit([&](const auto &meshData) {
    auto flat = meshData.elementsFlat();
    std::vector<double> vertices;
    vertices.reserve(meshData.positions().size() * 3);
    for (const Vec3d &v : meshData.positions()) {
      vertices.push_back(v[0]);
      vertices.push_back(v[1]);
      vertices.push_back(v[2]);
    }

    using T = std::decay_t<decltype(meshData)>;
    if constexpr (std::is_same_v<T, Mesh::MeshData<4>>) {
      TetMesh mesh(
        static_cast<int>(meshData.numVertices()), vertices.data(),
        static_cast<int>(meshData.numElements()), flat.data(),
        static_cast<int>(materials.size()), materialPtrs.data(),
        static_cast<int>(sets.size()), sets.data(),
        static_cast<int>(regions.size()), regions.data());
      return mesh.saveToAscii(path.string().c_str());
    }
    else {
      CubicMesh mesh(
        static_cast<int>(meshData.numVertices()), vertices.data(),
        static_cast<int>(meshData.numElements()), flat.data(),
        static_cast<int>(materials.size()), materialPtrs.data(),
        static_cast<int>(sets.size()), sets.data(),
        static_cast<int>(regions.size()), regions.data());
      return mesh.saveToAscii(path.string().c_str());
    }
  }, payload.meshData);

  if (result != 0)
    throw std::runtime_error("Failed to write veg file: " + path.string());
}

}  // namespace pgo::VolumetricMeshes
