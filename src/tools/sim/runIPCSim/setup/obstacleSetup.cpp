#include "setup/obstacleSetup.h"

#include "configFileJSON.h"
#include "setup/setupCommon.h"
#include "triMeshGeo.h"

#include <array>
#include <memory>
#include <stdexcept>

namespace pgo::RunIPCSim
{
namespace ES = pgo::EigenSupport;

std::vector<std::unique_ptr<Contact::IPC::ObstacleSurface>> parseExternalObjects(
  const pgo::ConfigFileJSON &jconfig, double scale,
  std::vector<bool> *outStaticFlags)
{
  std::vector<std::unique_ptr<Contact::IPC::ObstacleSurface>> obstacles;
  if (outStaticFlags)
    outStaticFlags->clear();

  if (!jconfig.exist("external-objects"))
    return obstacles;

  const auto &extObjs = jconfig.handle()["external-objects"];
  if (!extObjs.is_array())
    throwConfigError("`external-objects` must be a JSON array.");

  obstacles.reserve(extObjs.size());
  if (outStaticFlags)
    outStaticFlags->reserve(extObjs.size());

  for (std::size_t oi = 0; oi < extObjs.size(); ++oi) {
    const auto &objJson = extObjs.at(oi);
    if (!objJson.is_object())
      throwConfigError("Each `external-objects[]` entry must be a JSON object.");

    if (!objJson.contains("filename"))
      throwConfigError("Missing required field `external-objects[].filename`.");
    if (!objJson.contains("movement"))
      throwConfigError("Missing required field `external-objects[].movement`.");

    const std::string filename = jconfig.resolvePath(objJson["filename"].get<std::string>());
    const std::array<double, 3> movementArr = objJson["movement"].get<std::array<double, 3>>();
    const double objScale = objJson.contains("scale") ? objJson["scale"].get<double>() : scale;

    // Load obstacle mesh
    pgo::Mesh::TriMeshGeo obsMesh;
    if (!obsMesh.load(filename))
      throw std::runtime_error("Failed to load external object mesh: " + filename);
    for (int vi = 0; vi < obsMesh.numVertices(); ++vi)
      obsMesh.pos(vi) *= objScale;

    ES::MXd V(obsMesh.numVertices(), 3);
    ES::MXi F(obsMesh.numTriangles(), 3);
    for (int vi = 0; vi < obsMesh.numVertices(); ++vi)
      V.row(vi) = obsMesh.pos(vi).transpose();
    for (int fi = 0; fi < obsMesh.numTriangles(); ++fi)
      F.row(fi) = obsMesh.tri(fi).transpose();

    ES::V3d velocity(movementArr[0], movementArr[1], movementArr[2]);
    if (velocity.isZero()) {
      obstacles.push_back(std::make_unique<Contact::IPC::StaticObstacleSurface>(
        std::move(V), std::move(F)));
    }
    else {
      obstacles.push_back(std::make_unique<Contact::IPC::LinearMovingObstacleSurface>(
        std::move(V), std::move(F), velocity));
    }
    if (outStaticFlags)
      outStaticFlags->push_back(velocity.isZero());
  }

  return obstacles;
}
}  // namespace pgo::RunIPCSim
