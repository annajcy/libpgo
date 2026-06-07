#include "core.h"

#include "abcWriter.h"

#include <nanobind/nanobind.h>

namespace pgo
{

bool has_animation_io()
{
  return true;
}

void dump_abc(const std::string &filename,
  const std::string &name,
  const std::vector<float> &restPositions,
  const std::vector<std::vector<float>> &displacements,
  const std::vector<std::vector<int>> &triangles)
{
  nanobind::gil_scoped_release release;
  AnimationIO::dumpABC(filename.c_str(), name.c_str(), restPositions, displacements, triangles);
}

int PyAnimationLoader::load(const std::string &filename)
{
  nanobind::gil_scoped_release release;
  return loader_.load(filename.c_str());
}

int PyAnimationLoader::saveABC(const std::string &prefix)
{
  nanobind::gil_scoped_release release;
  return loader_.saveABC(prefix.c_str());
}

bool has_stress_vdb_export()
{
#if defined(PYPGO_HAS_STRESS_VDB)
  return true;
#else
  return false;
#endif
}

#if defined(PYPGO_HAS_STRESS_VDB)
int PyStressFieldVDBExporter::loadTetMesh(const std::string &vegPath)
{
  nanobind::gil_scoped_release release;
  return exporter_.loadTetMesh(vegPath.c_str());
}

int PyStressFieldVDBExporter::loadDeformationSequence(
  const std::string &folder, const std::string &pattern, int frameStart, int frameEnd)
{
  nanobind::gil_scoped_release release;
  return exporter_.loadDeformationSequence(folder.c_str(), pattern.c_str(), frameStart, frameEnd);
}

int PyStressFieldVDBExporter::loadVonMisesSequence(
  const std::string &folder, const std::string &pattern, int frameStart, int frameEnd)
{
  nanobind::gil_scoped_release release;
  return exporter_.loadVonMisesSequence(folder.c_str(), pattern.c_str(), frameStart, frameEnd);
}

int PyStressFieldVDBExporter::exportAnimationVDB(
  const std::string &outputDir, const std::string &prefix, double voxelSize) const
{
  nanobind::gil_scoped_release release;
  return exporter_.exportAnimationVDB(outputDir.c_str(), prefix.c_str(), voxelSize);
}

int PyStressFieldVDBExporter::numFrames() const
{
  return exporter_.numFrames();
}
#endif

}  // namespace pgo
