#include "runSimCore.h"

#include "fileService.h"
#include "basicIO.h"
#include "generateTetMeshMatrix.h"
#include "tetMesh.h"
#include "triMeshGeo.h"
#include "geometryQuery.h"
#include "simulationMesh.h"
#include "deformationModelManager.h"
#include "tetMeshDeformationModel.h"
#include "plasticModel3DDeformationGradient.h"
#include "deformationModelAssembler.h"
#include "deformationModelEnergy.h"
#include "multiVertexPullingSoftConstraints.h"
#include "implicitBackwardEulerTimeIntegrator.h"
#include "TRBDF2TimeIntegrator.h"
#include "generateMassMatrix.h"
#include "barycentricCoordinates.h"
#include "triangleMeshExternalContactHandler.h"
#include "pointPenetrationEnergy.h"
#include "triangleMeshSelfContactHandler.h"
#include "pointTrianglePairCouplingEnergyWithCollision.h"
#include "linearPotentialEnergy.h"
#include "NewtonRaphsonSolver.h"
#include "pgoLogging.h"

#include <tbb/global_control.h>

#include <algorithm>
#include <fmt/format.h>
#include <iostream>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <thread>
#include <vector>

namespace pgo {
namespace api {

RunSimConfig parseRunSimConfig(const ConfigFileJSON& jconfig, const std::string& configFilePath) {
    using namespace EigenSupport;
    namespace ES = EigenSupport;

    if (configFilePath.empty()) {
        throw std::runtime_error("Config file path is empty.");
    }
    const FileService::ConfigPathResolver configPathResolver(configFilePath);

    RunSimConfig config;

    const bool hasTetMesh   = jconfig.exist("tet-mesh");
    const bool hasCubicMesh = jconfig.exist("cubic-mesh");

    if (hasTetMesh && hasCubicMesh) {
        throw std::runtime_error("Config cannot contain both 'tet-mesh' and 'cubic-mesh'.");
    }

    if (!hasTetMesh && !hasCubicMesh) {
        throw std::runtime_error("Config must contain either 'tet-mesh' or 'cubic-mesh'.");
    }

    if (hasTetMesh) {
        config.tetMeshFilename = configPathResolver.resolve(jconfig.getString("tet-mesh", 1));
        if (config.tetMeshFilename.empty()) {
            throw std::runtime_error("'tet-mesh' resolves to an empty path.");
        }
        config.volumetricMeshType = VolumetricMeshInputType::TET;
    } else {
        config.cubicMeshFilename = configPathResolver.resolve(jconfig.getString("cubic-mesh", 1));
        if (config.cubicMeshFilename.empty()) {
            throw std::runtime_error("'cubic-mesh' resolves to an empty path.");
        }
        config.volumetricMeshType = VolumetricMeshInputType::CUBIC;
    }

    if (jconfig.exist("volumetric-mesh-type")) {
        const std::string hintedType = jconfig.getString("volumetric-mesh-type", 1);

        VolumetricMeshInputType hintedMeshType;
        if (hintedType == "tet") {
            hintedMeshType = VolumetricMeshInputType::TET;
        } else if (hintedType == "cubic") {
            hintedMeshType = VolumetricMeshInputType::CUBIC;
        } else {
            throw std::runtime_error(fmt::format("Unsupported volumetric-mesh-type: {}", hintedType));
        }

        if (hintedMeshType != config.volumetricMeshType) {
            throw std::runtime_error(
                "volumetric-mesh-type does not match the volumetric mesh filename key.");
        }
    }

    config.surfaceMeshFilename = configPathResolver.resolve(jconfig.getString("surface-mesh", 1));
    auto readVec3              = [&](const char* key) {
        auto values = jconfig.getValue<std::array<double, 3>>(key, 1);
        return ES::V3d(values[0], values[1], values[2]);
    };

    config.extAcc               = readVec3("g");
    config.initialVel           = readVec3("init-vel");
    config.initialDisp          = readVec3("init-disp");
    config.scale                = jconfig.getDouble("scale", 1);
    config.timestep             = jconfig.getDouble("timestep", 1);
    config.contactStiffness     = jconfig.getDouble("contact-stiffness", 1);
    config.contactSamples       = jconfig.getInt("contact-sample", 1);
    config.contactFrictionCoeff = jconfig.getDouble("contact-friction-coeff", 1);
    config.contactVelEps        = jconfig.getDouble("contact-vel-eps", 1);
    config.solverEps            = jconfig.getDouble("solver-eps", 1);
    config.solverMaxIter        = jconfig.getInt("solver-max-iter", 1);
    config.dampingParams        = jconfig.getValue<std::array<double, 2>>("damping-params", 1);

    std::string material = jconfig.getString("elastic-material");
    if (material == "stable-neo") {
        config.elasticMaterial = SolidDeformationModel::DeformationModelElasticMaterial::STABLE_NEO;
    } else if (material == "stvk-vol") {
        config.elasticMaterial = SolidDeformationModel::DeformationModelElasticMaterial::STVK_VOL;
    } else {
        throw std::runtime_error(fmt::format("Unsupported elastic material: {}", material));
    }

    config.numSimSteps  = jconfig.getInt("num-timestep", 1);
    config.dumpInterval = jconfig.getInt("dump-interval", 1);
    config.simType      = jconfig.getString("sim-type");
    config.outputFolder = configPathResolver.resolve(jconfig.getString("output", 1));
    config.deterministicMode = jconfig.handle().value("deterministic", false);

    if (jconfig.exist("fixed-vertices")) {
        for (const auto& fv : jconfig.handle()["fixed-vertices"]) {
            FixedVertexAttachment attachment;
            attachment.filename = configPathResolver.resolve(fv["filename"].get<std::string>());
            attachment.movement = fv["movement"].get<std::array<double, 3>>();
            attachment.coeff    = fv["coeff"].get<double>();
            config.fixedVertices.push_back(attachment);
        }
    }

    if (jconfig.exist("external-objects")) {
        for (const auto& obj : jconfig.handle()["external-objects"]) {
            ExternalObject ext;
            ext.filename = configPathResolver.resolve(obj["filename"].get<std::string>());
            ext.movement = obj["movement"].get<std::array<double, 3>>();
            config.externalObjects.push_back(ext);
        }
    }

    return config;
}

int runSimFromConfig(const RunSimConfig& config) {
    using namespace EigenSupport;
    namespace ES = EigenSupport;

    if (config.volumetricMeshType == VolumetricMeshInputType::CUBIC) {
        throw std::runtime_error(
            "Cubic mesh input parsing is enabled, but runSim cubic execution is not wired yet. "
            "Please complete Lecture 03 Step 6.");
    }

    const int defaultParallelism =
        std::min(64, static_cast<int>(std::max(1u, std::thread::hardware_concurrency())));
    const int effectiveParallelism = config.deterministicMode ? 1 : defaultParallelism;

    tbb::global_control threadLimit(tbb::global_control::max_allowed_parallelism, effectiveParallelism);
    tbb::global_control stackSizeLimit(tbb::global_control::thread_stack_size, 16 * 1024 * 1024);

    if (config.deterministicMode) {
        SPDLOG_LOGGER_INFO(Logging::lgr(), "Deterministic mode enabled; forcing single-threaded execution.");
    }

    VolumetricMeshes::TetMesh tetMesh(config.tetMeshFilename.c_str());
    for (int vi = 0; vi < tetMesh.getNumVertices(); vi++) {
        tetMesh.setVertex(vi, tetMesh.getVertex(vi) * config.scale);
    }

    Mesh::TriMeshGeo surfaceMesh;
    if (surfaceMesh.load(config.surfaceMeshFilename) != true)
        return 1;

    for (int vi = 0; vi < surfaceMesh.numVertices(); vi++) {
        surfaceMesh.pos(vi) *= config.scale;
    }

    int     surfn  = surfaceMesh.numVertices();
    int     surfn3 = surfn * 3;
    ES::VXd surfaceRestPositions(surfn3);
    for (int vi = 0; vi < surfn; vi++) {
        surfaceRestPositions.segment<3>(vi * 3) = surfaceMesh.pos(vi);
    }

    InterpolationCoordinates::BarycentricCoordinates bc(surfaceMesh.numVertices(), surfaceRestPositions.data(),
                                                        &tetMesh);
    ES::SpMatD                                       W = bc.generateInterpolationMatrix();

    std::shared_ptr<SolidDeformationModel::SimulationMesh> simMesh(SolidDeformationModel::loadTetMesh(&tetMesh));
    std::shared_ptr<SolidDeformationModel::DeformationModelManager> dmm =
        std::make_shared<SolidDeformationModel::DeformationModelManager>();

    dmm->setMesh(simMesh.get(), nullptr, nullptr);
    dmm->init(pgo::SolidDeformationModel::DeformationModelPlasticMaterial::VOLUMETRIC_DOF6, config.elasticMaterial, 1);

    std::vector<double>                                               elementWeights(simMesh->getNumElements(), 1.0);
    std::shared_ptr<SolidDeformationModel::DeformationModelAssembler> assembler =
        std::make_shared<SolidDeformationModel::DeformationModelAssembler>(dmm, elementWeights.data());

    int n    = simMesh->getNumVertices();
    int n3   = n * 3;
    int nele = simMesh->getNumElements();

    ES::VXd plasticity(nele * 6);
    ES::M3d I = ES::M3d::Identity();
    for (int ei = 0; ei < nele; ei++) {
        const SolidDeformationModel::PlasticModel3DDeformationGradient* pm =
            dynamic_cast<const SolidDeformationModel::PlasticModel3DDeformationGradient*>(
                dmm->getDeformationModel(ei)->getPlasticModel());
        if (!pm) {
            SPDLOG_LOGGER_ERROR(Logging::lgr(), "Plastic model is not of type PlasticModel3DDeformationGradient.");
            return 1;
        }
        pm->toParam(I.data(), plasticity.data() + ei * dmm->getNumPlasticParameters());
    }

    ES::VXd restPosition(n3);
    for (int vi = 0; vi < n; vi++) {
        double p[3];
        simMesh->getVertex(vi, p);
        restPosition.segment<3>(vi * 3) = ES::V3d(p[0], p[1], p[2]);
    }

    std::shared_ptr<SolidDeformationModel::DeformationModelEnergy> elasticEnergy =
        std::make_shared<SolidDeformationModel::DeformationModelEnergy>(assembler, &restPosition, 0);
    elasticEnergy->setPlasticParams(plasticity);

    ES::VXd zero(n3);
    zero.setZero();

    ES::SpMatD K;
    elasticEnergy->createHessian(K);
    elasticEnergy->hessian(zero, K);

    std::vector<std::shared_ptr<ConstraintPotentialEnergies::MultipleVertexPulling>> pullingEnergies;
    std::vector<ES::VXd>                                                             pullingTargets, pullingTargetRests;
    Mesh::TriMeshGeo                                                                 tempMesh;
    for (const auto& fv : config.fixedVertices) {
        std::vector<int> fixedVertices;
        if (BasicIO::read1DText(fv.filename.c_str(), std::back_inserter(fixedVertices)) != 0) {
            return 1;
        }
        std::sort(fixedVertices.begin(), fixedVertices.end());

        ES::VXd tgtVertexPositions(fixedVertices.size() * 3);
        ES::VXd tgtVertexRests(fixedVertices.size() * 3);
        for (int vi = 0; vi < (int)fixedVertices.size(); vi++) {
            ES::V3d movement(fv.movement[0], fv.movement[1], fv.movement[2]);
            tgtVertexPositions.segment<3>(vi * 3) = restPosition.segment<3>(fixedVertices[vi] * 3) + movement;
            tgtVertexRests.segment<3>(vi * 3)     = restPosition.segment<3>(fixedVertices[vi] * 3);
            tempMesh.addPos(tgtVertexPositions.segment<3>(vi * 3));
        }

        auto pullingEnergy = std::make_shared<ConstraintPotentialEnergies::MultipleVertexPulling>(
            K, restPosition.data(), (int)fixedVertices.size(), fixedVertices.data(), tgtVertexPositions.data(), nullptr,
            1);
        pullingEnergy->setCoeff(fv.coeff);
        pullingEnergies.push_back(pullingEnergy);
        pullingTargets.push_back(tgtVertexPositions);
        pullingTargetRests.push_back(tgtVertexRests);
    }

    tempMesh.save("fv.obj");

    std::vector<std::string> kinematicObjectFilenames;
    std::vector<ES::V3d>     kinematicObjectMovements;
    for (const auto& obj : config.externalObjects) {
        kinematicObjectFilenames.push_back(obj.filename);
        kinematicObjectMovements.emplace_back(obj.movement[0], obj.movement[1], obj.movement[2]);
    }

    ES::SpMatD M;
    VolumetricMeshes::GenerateMassMatrix::computeMassMatrix(&tetMesh, M, true);

    ES::VXd g(n3);
    for (int vi = 0; vi < n; vi++) {
        g.segment<3>(vi * 3) = config.extAcc;
    }

    ES::VXd fext(n3);
    ES::mv(M, g, fext);

    if (config.simType == "dynamic") {
        std::vector<Mesh::TriMeshGeo> kinematicObjects;
        for (const auto& filename : kinematicObjectFilenames) {
            kinematicObjects.emplace_back();
            if (kinematicObjects.back().load(filename) != true) {
                return 1;
            }

            for (int vi = 0; vi < kinematicObjects.back().numVertices(); vi++) {
                kinematicObjects.back().pos(vi) *= config.scale;
            }
        }

        std::vector<Mesh::TriMeshRef> kinematicObjectsRef;
        for (int i = 0; i < (int)kinematicObjects.size(); i++) {
            kinematicObjectsRef.emplace_back(kinematicObjects[i]);
        }

        std::shared_ptr<Contact::TriangleMeshExternalContactHandler> externalContactHandler;
        std::shared_ptr<Contact::TriangleMeshSelfContactHandler>     selfCD;
        if (kinematicObjectsRef.size() && config.contactStiffness > 0) {
            externalContactHandler = std::make_shared<Contact::TriangleMeshExternalContactHandler>(
                surfaceMesh.positions(), surfaceMesh.triangles(), n3, kinematicObjectsRef, config.contactSamples,
                &bc.getEmbeddingVertexIndices(), &bc.getEmbeddingWeights());
        }

        if (config.contactStiffness > 0) {
            selfCD = std::make_shared<Contact::TriangleMeshSelfContactHandler>(
                surfaceMesh.positions(), surfaceMesh.triangles(), n3, config.contactSamples,
                &bc.getEmbeddingVertexIndices(), &bc.getEmbeddingWeights());
        }

        std::shared_ptr<Simulation::ImplicitBackwardEulerTimeIntegrator> intg =
            std::make_shared<Simulation::ImplicitBackwardEulerTimeIntegrator>(M, elasticEnergy, config.dampingParams[0],
                                                                              config.dampingParams[1], config.timestep,
                                                                              config.solverMaxIter, config.solverEps);

#if defined(PGO_HAS_KNITRO)
        intg->setSolverOption(Simulation::TimeIntegratorSolverOption::SO_KNITRO);
        intg->setSolverConfigFile("config.opt");
#endif

        for (auto pullingEnergy : pullingEnergies)
            intg->addImplicitForceModel(pullingEnergy, 0, 0);

        intg->setExternalForce(fext.data());

        ES::VXd x = restPosition, u(n3);
        ES::VXd uvel(n3), uacc(n3), usurf(surfn3);

        u.setZero();
        uvel.setZero();
        uacc.setZero();
        usurf.setZero();

        for (int i = 0; i < n; i++) {
            uvel.segment<3>(i * 3) = config.initialVel;
            u.segment<3>(i * 3)    = config.initialDisp;
        }

        if (!std::filesystem::exists(config.outputFolder)) {
            std::filesystem::create_directories(config.outputFolder);
        }

        int frameStart = 0;
        if (config.restartEnabled) {
            for (int framei = config.numSimSteps - 1; framei >= 0; framei--) {
                std::string filename = fmt::format("{}/deform{:04d}.u", config.outputFolder, framei);
                if (!std::filesystem::exists(filename)) {
                    continue;
                }

                ES::MXd uMat(n3, 3);
                if (ES::readMatrix(filename.c_str(), uMat) == 0) {
                    frameStart     = framei;
                    u.noalias()    = uMat.col(0);
                    uvel.noalias() = uMat.col(1);
                    uacc.noalias() = uMat.col(2);
                    std::cout << "Restarting from frame " << framei << std::endl;
                    break;
                }
            }

            if (config.restartPause) {
                std::cout << frameStart << std::endl;
                std::cin.get();
            }
        }

        ES::mv(W, u, usurf);

        for (int framei = frameStart + 1; framei < config.numSimSteps; framei++) {
            intg->clearGeneralImplicitForceModel();

            double ratio = (double)framei / (config.numSimSteps - 1);
            for (size_t pi = 0; pi < pullingEnergies.size(); pi++) {
                ES::VXd restTgt = pullingTargetRests[pi];
                ES::VXd curTgt  = restTgt * (1 - ratio) + pullingTargets[pi] * ratio;
                pullingEnergies[pi]->setTargetPos(curTgt.data());

                std::cout << "Frame " << framei << ", attachment " << pi << " target: " << curTgt.transpose().head(3)
                          << std::endl;
            }

            std::shared_ptr<Contact::PointPenetrationEnergy> extContactEnergy;
            Contact::PointPenetrationEnergyBuffer*           extContactBuffer = nullptr;
            if (externalContactHandler) {
                externalContactHandler->execute(usurf.data());

                if (externalContactHandler->getNumCollidingSamples()) {
                    extContactEnergy = externalContactHandler->buildContactEnergy();
                    extContactBuffer = extContactEnergy->allocateBuffer();

                    auto posFunc = [&restPosition](const EigenSupport::V3d& u, EigenSupport::V3d& p, int dofStart) {
                        p = u + restPosition.segment<3>(dofStart);
                    };

                    auto lastPosFunc = [&restPosition, &u](const EigenSupport::V3d& x, EigenSupport::V3d& p,
                                                           int dofStart) {
                        p = u.segment<3>(dofStart) + restPosition.segment<3>(dofStart);
                    };

                    extContactEnergy->setComputePosFunction(posFunc);
                    extContactEnergy->setBuffer(extContactBuffer);
                    extContactEnergy->setCoeff(config.contactStiffness);

                    extContactEnergy->setFrictionCoeff(config.contactFrictionCoeff);
                    extContactEnergy->setComputeLastPosFunction(lastPosFunc);
                    extContactEnergy->setVelEps(config.contactVelEps);
                    extContactEnergy->setTimestep(config.timestep);

                    intg->addGeneralImplicitForceModel(extContactEnergy, 0, 0);
                }
            }

            std::shared_ptr<Contact::PointTrianglePairCouplingEnergyWithCollision> selfContactEnergy;
            Contact::PointTrianglePairCouplingEnergyWithCollisionBuffer*           selfContactEnergyBuf = nullptr;

            if (selfCD) {
                selfCD->execute(usurf.data());

                if (selfCD->getCollidingTrianglePair().size() > 0) {
                    selfCD->handleContactDCD(0, 100);

                    selfContactEnergy = selfCD->buildContactEnergy();
                    selfContactEnergy->setToPosFunction([&restPosition](const ES::V3d& x, ES::V3d& p, int offset) {
                        p = x + restPosition.segment<3>(offset);
                    });

                    selfContactEnergy->setToLastPosFunction(
                        [&restPosition, &u](const ES::V3d& x, ES::V3d& p, int offset) {
                            p = restPosition.segment<3>(offset) + u.segment<3>(offset);
                        });

                    selfContactEnergyBuf = selfContactEnergy->allocateBuffer();
                    selfContactEnergy->setBuffer(selfContactEnergyBuf);
                    selfContactEnergy->setCoeff(config.contactStiffness);
                    selfContactEnergy->computeClosestPosition(u.data());

                    selfContactEnergy->setFrictionCoeff(config.contactFrictionCoeff);
                    selfContactEnergy->setTimestep(config.timestep);
                    selfContactEnergy->setVelEps(config.contactVelEps);

                    intg->addGeneralImplicitForceModel(selfContactEnergy, 0, 0);
                }
            }

            intg->setqState(u, uvel, uacc);

            intg->doTimestep(1, 2, 1);

            intg->getq(u);
            intg->getqvel(uvel);
            intg->getqacc(uacc);

            if (extContactBuffer && extContactEnergy) {
                extContactEnergy->freeBuffer(extContactBuffer);
            }

            if (selfContactEnergyBuf && selfContactEnergy) {
                selfContactEnergy->freeBuffer(selfContactEnergyBuf);
            }

            ES::mv(W, u, usurf);

            if (framei % config.dumpInterval == 0) {
                ES::VXd psurf = surfaceRestPositions + usurf;

                Mesh::TriMeshGeo mesh = surfaceMesh;
                for (int vi = 0; vi < mesh.numVertices(); vi++) {
                    mesh.pos(vi) = psurf.segment<3>(vi * 3) / config.scale;
                }
                mesh.save(fmt::format("{}/ret{:04d}.obj", config.outputFolder, framei / config.dumpInterval));
            }

            for (size_t eobji = 0; eobji < kinematicObjects.size(); eobji++) {
                ES::V3d movement = kinematicObjectMovements[eobji] / (config.numSimSteps - 1);
                for (int vi = 0; vi < kinematicObjects[eobji].numVertices(); vi++) {
                    kinematicObjects[eobji].pos(vi) += movement;
                }
                externalContactHandler->updateExternalSurface(eobji, kinematicObjectsRef[eobji]);
            }

            ES::MXd uMat(n3, 3);
            uMat.col(0) = u;
            uMat.col(1) = uvel;
            uMat.col(2) = uacc;

            ES::writeMatrix(fmt::format("{}/deform{:04d}.u", config.outputFolder, framei).c_str(), uMat);
        }
    } else if (config.simType == "static") {
        std::shared_ptr<PredefinedPotentialEnergies::LinearPotentialEnergy> externalForcesEnergy =
            std::make_shared<PredefinedPotentialEnergies::LinearPotentialEnergy>(fext);

        std::shared_ptr<NonlinearOptimization::PotentialEnergies> energyAll =
            std::make_shared<NonlinearOptimization::PotentialEnergies>(n3);
        energyAll->addPotentialEnergy(elasticEnergy);
        for (auto eng : pullingEnergies)
            energyAll->addPotentialEnergy(eng, 1.0);
        energyAll->addPotentialEnergy(externalForcesEnergy, -1.0);
        energyAll->init();

        NonlinearOptimization::NewtonRaphsonSolver::SolverParam solverParam;

        ES::VXd u(n3);
        u.setZero();

        energyAll->printEnergy(u);

        NonlinearOptimization::NewtonRaphsonSolver solver(u.data(), solverParam, energyAll, std::vector<int>(),
                                                          nullptr);
        solver.solve(u.data(), config.solverMaxIter, config.solverEps, 2);

        ES::VXd x = restPosition + u;

        ES::VXd xsurf(surfn3);
        ES::mv(W, x, xsurf);

        Mesh::TriMeshGeo meshOut = surfaceMesh;
        for (int vi = 0; vi < meshOut.numVertices(); vi++) {
            meshOut.pos(vi) = xsurf.segment<3>(vi * 3);
        }
        meshOut.save(config.outputFolder);
    }

    return 0;
}

}  // namespace api
}  // namespace pgo
