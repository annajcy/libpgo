#pragma once

#include "animationLoader.h"

#if defined(PYPGO_HAS_STRESS_VDB)
#  include "stressFieldVDBExporter.h"
#endif

#include <string>
#include <vector>

namespace pgo
{

bool has_animation_io();
void dump_abc(const std::string &filename,
  const std::string &name,
  const std::vector<float> &restPositions,
  const std::vector<std::vector<float>> &displacements,
  const std::vector<std::vector<int>> &triangles);

class PyAnimationLoader
{
public:
  int load(const std::string &filename);
  int saveABC(const std::string &prefix);

private:
  AnimationIO::AnimationLoader loader_;
};

bool has_stress_vdb_export();

#if defined(PYPGO_HAS_STRESS_VDB)
class PyStressFieldVDBExporter
{
public:
  int loadTetMesh(const std::string &vegPath);
  int loadDeformationSequence(const std::string &folder, const std::string &pattern, int frameStart, int frameEnd);
  int loadVonMisesSequence(const std::string &folder, const std::string &pattern, int frameStart, int frameEnd);
  int exportAnimationVDB(const std::string &outputDir, const std::string &prefix, double voxelSize) const;
  int numFrames() const;

private:
  AnimationIO::StressFieldVDBExporter exporter_;
};
#endif

}  // namespace pgo
