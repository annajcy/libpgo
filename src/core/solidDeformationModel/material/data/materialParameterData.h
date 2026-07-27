#pragma once

#include "EigenSupport.h"

namespace pgo::SolidDeformationModel
{

/// Numeric values produced by a one-shot projection.  This type deliberately
/// contains no model, schema, layout, mapping or mesh handles.
struct MaterialParameterBlockData
{
  EigenSupport::VXd fixedValues;
  EigenSupport::VXd initialOptimizableValues;
};

struct MaterialParameterData
{
  MaterialParameterBlockData elastic;
  MaterialParameterBlockData plastic;
};

}  // namespace pgo::SolidDeformationModel
