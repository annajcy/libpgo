#include "core.h"

#include "elastic/elasticModelFactory.h"
#include "../../simulation/core.h"

namespace pgo
{
namespace SD = SolidDeformationModel;

PyElasticModel::PyElasticModel(SD::DeformationModelElasticMaterial type)
  : type_(type)
{
}

std::string PyElasticModel::name() const
{
  return SD::ElasticModelFactory::modelId(type_);
}

int PyElasticModel::numChannels(const SD::SimulationMesh &mesh) const
{
  return SD::ElasticModelFactory::parameterSpec(mesh, type_).numChannels;
}

int PyElasticModel::numChannels(const PySimulationMesh &mesh) const
{
  return numChannels(mesh.mesh());
}

std::shared_ptr<PyElasticModel> make_stable_neo()
{
  return std::make_shared<PyElasticModel>(SD::DeformationModelElasticMaterial::STABLE_NEO);
}

std::shared_ptr<PyElasticModel> make_stvk()
{
  return std::make_shared<PyElasticModel>(SD::DeformationModelElasticMaterial::STVK);
}

std::shared_ptr<PyElasticModel> make_stvk_vol()
{
  return std::make_shared<PyElasticModel>(SD::DeformationModelElasticMaterial::STVK_VOL);
}

std::shared_ptr<PyElasticModel> make_linear_elastic()
{
  return std::make_shared<PyElasticModel>(SD::DeformationModelElasticMaterial::LINEAR);
}

std::shared_ptr<PyElasticModel> make_mooney_rivlin()
{
  return std::make_shared<PyElasticModel>(SD::DeformationModelElasticMaterial::MOONEY_RIVLIN);
}

std::shared_ptr<PyElasticModel> make_koiter_stvk()
{
  return std::make_shared<PyElasticModel>(SD::DeformationModelElasticMaterial::KOITER_STVK);
}

}  // namespace pgo
