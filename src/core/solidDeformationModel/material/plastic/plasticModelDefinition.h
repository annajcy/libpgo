#pragma once

#include "material/modelDefinition.h"

namespace pgo::SolidDeformationModel
{

class PlasticModel;

/// Immutable, shareable definition of one plastic constitutive model.
///
/// The definition owns the physical channel contract and constructs the
/// per-element constitutive model from fixed channels and the material frame.
using PlasticModelDefinition = ModelDefinition<PlasticModel>;

}  // namespace pgo::SolidDeformationModel
