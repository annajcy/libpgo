#pragma once

namespace pgo
{
namespace SolidDeformationModel
{

// Formulation tag objects — one per implemented or planned FEM formulation.
// Topology is expressed by the factory function name (makeTetDeformationModel etc.),
// not by a common FormulationType enum.

// Linear tetrahedron, 4 nodes, vertex-centered 3-DOF.
struct TetP1 {};

// Trilinear hexahedron, 8 nodes, vertex-centered 3-DOF.
struct HexTrilinear {};

// Koiter thin-shell (existing KoiterDeformationModel path).
struct ShellKoiter {};

}  // namespace SolidDeformationModel
}  // namespace pgo
