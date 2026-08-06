#pragma once

#include "material/modelDefinition.h"

namespace pgo::SolidDeformationModel
{

class ElasticModel;

/// Immutable, shareable definition of one elastic constitutive model.
///
/// The definition owns the physical channel contract and constructs the
/// per-element constitutive model from fixed channels and the material frame.
using ElasticModelDefinition = ModelDefinition<ElasticModel>;

}  // namespace pgo::SolidDeformationModel
