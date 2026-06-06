#pragma once

#include "deformationModelManager.h"
#include "simulationMesh.h"

#include <memory>
#include <string>

namespace pgo
{

// Python-facing elastic model wrapper.  Holds a DeformationModelElasticMaterial
// enum (the C++ "elastic model" is just a factory discriminator, not a polymorphic
// object), plus cached per-element parameter channel count.
class PyElasticModel
{
public:
  explicit PyElasticModel(SolidDeformationModel::DeformationModelElasticMaterial type);

  std::string name() const;
  SolidDeformationModel::DeformationModelElasticMaterial type() const { return type_; }

  // Number of parameter channels per element for the given mesh.
  int numChannels(const SolidDeformationModel::SimulationMesh &mesh) const;

private:
  SolidDeformationModel::DeformationModelElasticMaterial type_;
};

// --- factory functions ---

std::shared_ptr<PyElasticModel> make_stable_neo();
std::shared_ptr<PyElasticModel> make_stvk();
std::shared_ptr<PyElasticModel> make_stvk_vol();
std::shared_ptr<PyElasticModel> make_linear_elastic();
std::shared_ptr<PyElasticModel> make_mooney_rivlin();
std::shared_ptr<PyElasticModel> make_koiter_stvk();

}  // namespace pgo
