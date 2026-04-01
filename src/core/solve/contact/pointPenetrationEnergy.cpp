#include "pointPenetrationEnergy.h"

#include "pgoLogging.h"
#include "EigenSupport.h"
#include "createTriMesh.h"

#include <tbb/enumerable_thread_specific.h>
#include <tbb/spin_mutex.h>
#include <tbb/parallel_for.h>
#include <tbb/concurrent_vector.h>

#include <numeric>
#include <atomic>
#include <iomanip>
#include <stdexcept>

using namespace pgo;
using namespace pgo::Contact;

namespace ES = pgo::EigenSupport;

namespace pgo::Contact {
struct PointPenetrationEnergyBuffer {
public:
    tbb::enumerable_thread_specific<double> energyTLS;
    std::vector<tbb::spin_mutex>            entryLocks;
};

}  // namespace pgo::Contact

namespace {

[[noreturn]] void throwPseudoNormalLookupFailure(const char* source, int objectID, const Mesh::TriMeshGeo& mesh,
                                                 int triID, int feature, int edgeVtx0, int edgeVtx1) {
    if (triID < 0 || triID >= mesh.numTriangles()) {
        SPDLOG_LOGGER_CRITICAL(Logging::lgr(),
                               "Pseudo-normal lookup failed in {}: object={}, triID={}, feature={}, invalid triangle "
                               "index for mesh with {} triangles",
                               source, objectID, triID, feature, mesh.numTriangles());
    } else {
        const ES::V3i tri = mesh.tri(triID);
        SPDLOG_LOGGER_CRITICAL(Logging::lgr(),
                               "Pseudo-normal lookup failed in {}: object={}, triID={}, feature={}, vtxIDs=({}, {}, "
                               "{}), edge=({}, {})",
                               source, objectID, triID, feature, tri[0], tri[1], tri[2], edgeVtx0, edgeVtx1);
    }

    throw std::runtime_error("Pseudo-normal lookup failed. Simulation aborted.");
}

const ES::V3d& getPseudoNormalOrThrow(const Mesh::TriMeshPseudoNormal& meshNormals, const Mesh::TriMeshGeo& mesh,
                                      int objectID, int triID, int feature, const char* source) {
    if (triID < 0 || triID >= mesh.numTriangles()) {
        throwPseudoNormalLookupFailure(source, objectID, mesh, triID, feature, -1, -1);
    }

    const ES::V3i tri = mesh.tri(triID);
    switch (feature) {
        case 0:
        case 1:
        case 2:
        case 6:
            return meshNormals.getPseudoNormal(mesh.triangles().data(), triID, feature);
        case 3:
            if (!meshNormals.hasEdgeNormal(tri[0], tri[1])) {
                throwPseudoNormalLookupFailure(source, objectID, mesh, triID, feature, tri[0], tri[1]);
            }
            return meshNormals.edgeNormal(tri[0], tri[1]);
        case 4:
            if (!meshNormals.hasEdgeNormal(tri[1], tri[2])) {
                throwPseudoNormalLookupFailure(source, objectID, mesh, triID, feature, tri[1], tri[2]);
            }
            return meshNormals.edgeNormal(tri[1], tri[2]);
        case 5:
            if (!meshNormals.hasEdgeNormal(tri[2], tri[0])) {
                throwPseudoNormalLookupFailure(source, objectID, mesh, triID, feature, tri[2], tri[0]);
            }
            return meshNormals.edgeNormal(tri[2], tri[0]);
        default:
            throwPseudoNormalLookupFailure(source, objectID, mesh, triID, feature, -1, -1);
    }
}

}  // namespace

PointPenetrationEnergy::PointPenetrationEnergy(int np, int na, const double* coeffs, const double* tgtPos,
                                               const double* nrms, const std::vector<std::vector<int>>& bIdx,
                                               const std::vector<std::vector<double>>& bWeights, double bcCoeffs,
                                               int useN, int checkPene, const std::vector<Mesh::TriMeshGeo>* ems,
                                               const std::vector<Mesh::TriMeshBVTree>*       embvs,
                                               const std::vector<Mesh::TriMeshPseudoNormal>* empn)
    : nAll(na),
      numPoints(np),
      constraintCoeffs(coeffs),
      constraintTargetPositions(tgtPos),
      constraintNormals(nrms),
      barycentricIdx(bIdx),
      barycentricWeights(bWeights),
      coeffAll(bcCoeffs),
      useNormal(useN),
      checkPenetration(checkPene),
      externalMeshes(ems),
      externalMeshBVTrees(embvs),
      externalMeshNormals(empn) {
    std::vector<ES::TripletD> entries;
    for (int pi = 0; pi < numPoints; pi++) {
        for (int vi = 0; vi < (int)barycentricIdx[pi].size(); vi++) {
            int vidi = barycentricIdx[pi][vi];

            for (int vj = 0; vj < (int)barycentricIdx[pi].size(); vj++) {
                int vidj = barycentricIdx[pi][vj];

                for (int dofi = 0; dofi < 3; dofi++) {
                    for (int dofj = 0; dofj < 3; dofj++) {
                        int globalRow = vidi * 3 + dofi;
                        int globalCol = vidj * 3 + dofj;

                        entries.emplace_back(globalRow, globalCol, 1.0);
                    }
                }
            }
        }
    }

    hessianConstant.resize(nAll, nAll);
    hessianConstant.setFromTriplets(entries.begin(), entries.end());
    ES::buildEntryMap(hessianConstant, hessianEntryMap);

    allDOFs.resize(nAll);
    std::iota(allDOFs.begin(), allDOFs.end(), 0);
}

PointPenetrationEnergyBuffer* PointPenetrationEnergy::allocateBuffer() const {
    PointPenetrationEnergyBuffer* buf = new PointPenetrationEnergyBuffer;
    buf->entryLocks                   = std::vector<tbb::spin_mutex>(nAll);

    return buf;
}

void PointPenetrationEnergy::freeBuffer(PointPenetrationEnergyBuffer* buf) const {
    delete buf;
}

double PointPenetrationEnergy::frictionPotential(const ES::V3d& x, const ES::V3d& xlast, const ES::V3d& n, double eps,
                                                 double timestep) const {
    // compute velocity v = (x_t+1 - x_t) / h
    // (v0 - vbc) - (v0 - vbc).n * n
    // w xdiff - n^T(w xdiff)n
    ES::V3d xdiff = x - xlast;
    ES::M3d InnT  = ES::M3d::Identity() - ES::tensorProduct(n, n);
    ES::V3d r     = InnT * xdiff;

    // compute energy
    double d = r.norm();
    double k = eps * timestep;

    if (d < k)
        return -(d * d * d) / (3.0 * k * k) + (d * d) / k + k / 3;
    // when velmag == timestep * eps
    // the function value is timestep * eps
    // the derivative is 1
    else
        return d;
}

void PointPenetrationEnergy::frictionPotentialGradient(const ES::V3d& x, const ES::V3d& xlast, const ES::V3d& n,
                                                       double eps, double timestep, ES::V3d& grad) const {
    // compute volecity v = (x_t+1 - x_t) / h
    ES::V3d xdiff = x - xlast;
    ES::M3d InnT  = ES::M3d::Identity() - ES::tensorProduct(n, n);
    ES::V3d r     = InnT * xdiff;

    // compute energy
    double d  = r.norm();
    double k  = eps * timestep;
    double c1 = 0;

    if (d < k) {
        c1 = (-d / (k * k) + 2 / k);
    } else {
        c1 = 1.0 / d;
    }

    ES::V3d grad_r = c1 * r;
    if (grad_r.norm() > 1.1) {
        std::cerr << "grad_r > 1" << std::endl;
        std::cerr << "grad_r: " << std::setprecision(16) << grad_r[0] << ',' << grad_r[1] << ',' << grad_r[2] << '\n'
                  << "|grad_r|: " << grad_r.norm() << '\n'
                  << eps << ',' << timestep << '\n'
                  << c1 << '\n'
                  << "r: " << r[0] << ',' << r[1] << ',' << r[2] << '\n'
                  << "|r|: " << r.norm() << std::endl;

        exit(1);
    }

    grad = InnT * grad_r;
}

void PointPenetrationEnergy::frictionPotentialHessian(const ES::V3d& x, const ES::V3d& xlast, const ES::V3d& n,
                                                      double eps, double timestep, ES::M3d& hess) const {
    ES::V3d xdiff = x - xlast;
    ES::M3d InnT  = ES::M3d::Identity() - ES::tensorProduct(n, n);
    ES::V3d r     = InnT * xdiff;

    double d = r.norm();
    double k = eps * timestep;

    ES::M3d hess_r;
    ES::V3d dddr   = r / d;
    ES::M3d d2ddr2 = ES::M3d::Identity() / d - (1.0 / (d * d * d)) * (r * r.transpose());
    if (dddr.hasNaN()) {
        dddr.setZero();
    }

    if (d2ddr2.hasNaN()) {
        d2ddr2.setZero();
    }

    if (d < k) {
        double dEdd   = -d * d / (k * k) + 2.0 * d / k;
        double d2Edd2 = -2.0 * d / (k * k) + 2.0 / k;
        hess_r        = d2Edd2 * (dddr * dddr.transpose()) + dEdd * d2ddr2;
    } else {
        double dEdd   = 1.0;
        double d2Edd2 = 0.0;
        hess_r        = d2Edd2 * (dddr * dddr.transpose()) + dEdd * d2ddr2;
    }

    if (1) {
        Eigen::SelfAdjointEigenSolver<ES::M3d> eig(hess_r);
        ES::V3d                                eigv = eig.eigenvalues().cwiseMax(0.0);
        ES::M3d                                eigb = eig.eigenvectors();
        hess_r                                      = eigb * eigv.asDiagonal() * eigb.transpose();
    }

    hess = InnT * hess_r * InnT;
}

// E = (n^T (sum w_i (restp + u) - tgtp))^2 * 0.5
double PointPenetrationEnergy::func(ES::ConstRefVecXd u) const {
    for (auto& val : buf->energyTLS) {
        val = 0.0;
    }

    int deepPenetration = 0;

    tbb::parallel_for(
        0, numPoints,
        [&](int ci) {
            if (std::abs(constraintCoeffs[ci]) < 1e-9)
                return;

            ES::V3d p;
            p.setZero();
            for (int vi = 0; vi < (int)barycentricIdx[ci].size(); vi++) {
                int     vid = barycentricIdx[ci][vi];
                ES::V3d pp;
                posFunc(u.segment<3>(vid * 3), pp, vid * 3);

                p += pp * barycentricWeights[ci][vi];
            }

            double& energyLocal = buf->energyTLS.local();

            if (useNormal) {
                ES::V3d n, p0;
                if (!resolveContactPlane(p, ci, p0, n))
                    return;

                double proj = (p - p0).dot(n);
                energyLocal += proj * proj * 0.5 * constraintCoeffs[ci];

                if (std::abs(proj) > 2e-3)
                    deepPenetration = 1;

                if (frictionCoeff > 0 && frictionContactForceMag > 0) {
                    ES::V3d fnow     = n * proj * constraintCoeffs[ci];
                    double  f_normal = fnow.norm();

                    ES::V3d plast;
                    plast.setZero();
                    for (int vi = 0; vi < (int)barycentricIdx[ci].size(); vi++) {
                        int     vid = barycentricIdx[ci][vi];
                        ES::V3d pp;
                        lastPosFunc(u.segment<3>(vid * 3), pp, vid * 3);

                        plast += pp * barycentricWeights[ci][vi];
                    }

                    double frictionEnergy = frictionPotential(p, plast, n, eps, timestep);
                    frictionEnergy *= frictionContactForceMag * frictionCoeff * f_normal;
                    energyLocal += frictionEnergy;
                }
            } else {
                ES::V3d p0 = ES::Mp<const ES::V3d>(constraintTargetPositions + ci * 3);
                energyLocal += (p - p0).squaredNorm() * 0.5 * constraintCoeffs[ci];
            }
        },
        tbb::static_partitioner());

    double energyAll = std::accumulate(buf->energyTLS.begin(), buf->energyTLS.end(), 0.0) * coeffAll;

    // if (deepPenetration)
    //   return 1e20;

    return energyAll;
}

// dE = (n^T (sum w_i (restp + u) - tgtp)) d((n^T (sum w_i (restp + u) - tgtp)))
// dE = (sum w_i (restp + u) - tgtp) d(sum w_i (restp + u) - tgtp)
void PointPenetrationEnergy::gradient(ES::ConstRefVecXd u, ES::RefVecXd grad) const {
    grad.setZero();

    tbb::concurrent_vector<ES::V6d> ffrics;

    tbb::parallel_for(
        0, numPoints,
        [&](int ci) {
            if (std::abs(constraintCoeffs[ci]) < 1e-9)
                return;

            ES::V3d p;
            p.setZero();
            for (int vi = 0; vi < (int)barycentricIdx[ci].size(); vi++) {
                int     vid = barycentricIdx[ci][vi];
                ES::V3d pp;
                posFunc(u.segment<3>(vid * 3), pp, vid * 3);

                p += pp * barycentricWeights[ci][vi];
            }

            if (useNormal) {
                ES::V3d n, p0;
                if (!resolveContactPlane(p, ci, p0, n))
                    return;

                ES::V3d diff       = p - p0;
                double  proj       = diff.dot(n);
                ES::V3d gradSample = n * proj * constraintCoeffs[ci];

                if (frictionCoeff > 0 && frictionContactForceMag > 0) {
                    double f_normal = gradSample.norm();

                    ES::V3d plast;
                    plast.setZero();
                    for (int vi = 0; vi < (int)barycentricIdx[ci].size(); vi++) {
                        int     vid = barycentricIdx[ci][vi];
                        ES::V3d pp;
                        lastPosFunc(u.segment<3>(vid * 3), pp, vid * 3);

                        plast += pp * barycentricWeights[ci][vi];
                    }

                    ES::V3d gradFric;
                    frictionPotentialGradient(p, plast, n, eps, timestep, gradFric);
                    gradFric *= frictionContactForceMag * frictionCoeff * f_normal;

                    // ES::V3d gradFric2;
                    // double frictionEnergy = frictionPotential(p, plast, n, eps, timestep);
                    // gradFric2 = frictionEnergy * frictionContactForceMag * frictionCoeff * n *
                    // constraintCoeffs[ci]; gradSample += gradFric + gradFric2;
                    gradSample += gradFric;

                    if (saveDebugInfo) {
                        ES::V6d vis;
                        vis.head<3>() = plast;
                        vis.tail<3>() = gradFric;
                        ffrics.emplace_back(vis);
                    }
                }

                for (int vi = 0; vi < (int)barycentricIdx[ci].size(); vi++) {
                    int     vid       = barycentricIdx[ci][vi];
                    double  w         = barycentricWeights[ci][vi];
                    ES::V3d gradLocal = gradSample * w;

                    buf->entryLocks[vid].lock();

                    grad.segment<3>(vid * 3) += gradLocal;

                    buf->entryLocks[vid].unlock();
                }
            } else {
                ES::V3d p0 = ES::Mp<const ES::V3d>(constraintTargetPositions + ci * 3);
                ES::V3d diff = p - p0;
                for (int vi = 0; vi < (int)barycentricIdx[ci].size(); vi++) {
                    int     vid       = barycentricIdx[ci][vi];
                    double  w         = barycentricWeights[ci][vi];
                    ES::V3d gradLocal = diff * w * constraintCoeffs[ci];

                    buf->entryLocks[vid].lock();

                    grad.segment<3>(vid * 3) += gradLocal;

                    buf->entryLocks[vid].unlock();
                }
            }
        },
        tbb::static_partitioner());

    grad *= coeffAll;

    if (saveDebugInfo) {
        Mesh::TriMeshGeo fmesh;
        for (const auto& viz : ffrics) {
            ES::V3d          dir = viz.tail<3>().normalized();
            Mesh::TriMeshGeo tri = Mesh::createSingleTriangleMesh(viz.head<3>(), viz.head<3>() - dir,
                                                                  viz.head<3>() - dir + ES::V3d::Constant(1e-5));
            fmesh.addMesh(tri);
        }
        fmesh.save("ff.obj");
    }
}

// use n
// E = (n^T (sum w_i (restp + u) - tgtp)) ((n^T (sum w_i (restp + u) - tgtp)))
// W = [ w1I w2I w3I w4I]
// d2E = W^T (nnT) W

// not use n
// E = (sum w_i (restp + u) - tgtp) (sum w_i (restp + u) - tgtp)
// W = [ w1I w2I w3I w4I]
// d2E = W^T W

void PointPenetrationEnergy::computeHessian() {
    memset(hessianConstant.valuePtr(), 0, sizeof(double) * hessianConstant.nonZeros());

    tbb::parallel_for(
        0, numPoints,
        [&](int ci) {
            if (std::abs(constraintCoeffs[ci]) < 1e-9)
                return;

            ES::V3d n   = ES::Mp<const ES::V3d>(constraintNormals + ci * 3);
            ES::M3d nnT = ES::tensorProduct(n, n) * constraintCoeffs[ci];

            if (useNormal == 0) {
                nnT = ES::M3d::Identity() * constraintCoeffs[ci];
            }

            for (int vi = 0; vi < (int)barycentricIdx[ci].size(); vi++) {
                double wi = barycentricWeights[ci][vi];

                for (int vj = 0; vj < (int)barycentricIdx[ci].size(); vj++) {
                    double wj = barycentricWeights[ci][vj];

                    ES::M3d hLocal = nnT * wi * wj;

                    for (int dofi = 0; dofi < 3; dofi++) {
                        for (int dofj = 0; dofj < 3; dofj++) {
                            int grow = barycentricIdx[ci][vi] * 3 + dofi;
                            int gcol = barycentricIdx[ci][vj] * 3 + dofj;

                            auto it = hessianEntryMap.find(std::make_pair(grow, gcol));
                            PGO_ALOG(it != hessianEntryMap.end());

                            std::ptrdiff_t offset = it->second;

                            buf->entryLocks[grow].lock();

                            hessianConstant.valuePtr()[offset] += hLocal(dofi, dofj);

                            buf->entryLocks[grow].unlock();
                        }
                    }
                }
            }
        },
        tbb::static_partitioner());
}

void PointPenetrationEnergy::hessian(ES::ConstRefVecXd u, ES::SpMatD& hess) const {
    if (checkPenetration) {
        std::memset(hess.valuePtr(), 0, sizeof(double) * hess.nonZeros());

        tbb::parallel_for(
            0, numPoints,
            [&](int ci) {
                // for (int ci = 0; ci < numPoints; ci ++) {
                if (std::abs(constraintCoeffs[ci]) < 1e-9)
                    return;

                ES::V3d p;
                p.setZero();
                for (int vi = 0; vi < (int)barycentricIdx[ci].size(); vi++) {
                    int     vid = barycentricIdx[ci][vi];
                    ES::V3d pp;
                    posFunc(u.segment<3>(vid * 3), pp, vid * 3);

                    p += pp * barycentricWeights[ci][vi];
                }

                ES::V3d n, p0;
                if (!resolveContactPlane(p, ci, p0, n))
                    return;

                ES::M3d nnT;
                if (useNormal == 0) {
                    nnT = ES::M3d::Identity() * constraintCoeffs[ci];
                } else {
                    nnT = ES::tensorProduct(n, n) * constraintCoeffs[ci];
                }

                if (frictionCoeff > 0 && frictionContactForceMag > 0) {
                    double  proj     = (p - p0).dot(n);
                    ES::V3d fnow     = n * proj * constraintCoeffs[ci];
                    double  f_normal = fnow.norm();

                    ES::V3d plast;
                    plast.setZero();
                    for (int vi = 0; vi < (int)barycentricIdx[ci].size(); vi++) {
                        int     vid = barycentricIdx[ci][vi];
                        ES::V3d pp;
                        lastPosFunc(u.segment<3>(vid * 3), pp, vid * 3);

                        plast += pp * barycentricWeights[ci][vi];
                    }

                    ES::M3d hessFric;
                    frictionPotentialHessian(p, plast, n, eps, timestep, hessFric);
                    hessFric *= frictionContactForceMag * frictionCoeff * f_normal;

                    // ES::V3d gradFric;
                    // frictionPotentialGradient(p, plast, n, eps, timestep, gradFric);

                    // ES::M3d hessFric2 = ES::tensorProduct(gradFric, n) * frictionContactForceMag * frictionCoeff*
                    // constraintCoeffs[ci]; ES::M3d hessFric3 = ES::tensorProduct(n, gradFric) *
                    // frictionContactForceMag * frictionCoeff* constraintCoeffs[ci];

                    nnT += hessFric;  // + hessFric2 + hessFric3;
                }

                for (int vi = 0; vi < (int)barycentricIdx[ci].size(); vi++) {
                    double wi = barycentricWeights[ci][vi];

                    for (int vj = 0; vj < (int)barycentricIdx[ci].size(); vj++) {
                        double wj = barycentricWeights[ci][vj];

                        ES::M3d hLocal = nnT * wi * wj;

                        for (int dofi = 0; dofi < 3; dofi++) {
                            for (int dofj = 0; dofj < 3; dofj++) {
                                int grow = barycentricIdx[ci][vi] * 3 + dofi;
                                int gcol = barycentricIdx[ci][vj] * 3 + dofj;

                                auto it = hessianEntryMap.find(std::make_pair(grow, gcol));
                                PGO_ALOG(it != hessianEntryMap.end());

                                std::ptrdiff_t offset = it->second;

                                buf->entryLocks[grow].lock();

                                hess.valuePtr()[offset] += hLocal(dofi, dofj);

                                buf->entryLocks[grow].unlock();
                            }
                        }
                    }
                }
            },
            tbb::static_partitioner());
    } else {
        memcpy(hess.valuePtr(), hessianConstant.valuePtr(), sizeof(double) * hess.nonZeros());
    }

    hess *= coeffAll;
}

bool PointPenetrationEnergy::isInside(const ES::V3d& p, const ES::V3d p0, const ES::V3d n) const {
    return (p - p0).dot(n) <= 0;
}

bool PointPenetrationEnergy::queryRuntimeContactPlane(const ES::V3d& p, ES::V3d& p0, ES::V3d& n) const {
    if (!(externalMeshes && externalMeshBVTrees && externalMeshNormals)) {
        return false;
    }

    double bestDist2 = DBL_MAX;
    bool   found     = false;

    thread_local std::vector<std::tuple<double, double, int>> nodeStack;

    for (int oi = 0; oi < static_cast<int>(externalMeshes->size()); oi++) {
        const auto& mesh       = (*externalMeshes)[oi];
        const auto& meshBVTree = (*externalMeshBVTrees)[oi];
        const auto& meshNormal = (*externalMeshNormals)[oi];

        nodeStack.clear();
        auto ret = meshBVTree.closestTriangleQuery(mesh, p, nodeStack);
        if (ret.triID < 0) {
            continue;
        }

        const ES::V3d& tgtNormal =
            getPseudoNormalOrThrow(meshNormal, mesh, oi, ret.triID, ret.feature, "external-contact-runtime");
        if ((p - ret.closestPosition).dot(tgtNormal) > 0) {
            continue;
        }

        if (!found || ret.dist2 < bestDist2) {
            found     = true;
            bestDist2 = ret.dist2;
            p0        = ret.closestPosition;
            n         = tgtNormal;
        }
    }

    return found;
}

bool PointPenetrationEnergy::resolveContactPlane(const ES::V3d& p, int ci, ES::V3d& p0, ES::V3d& n) const {
    if (externalMeshes && externalMeshBVTrees && externalMeshNormals) {
        return queryRuntimeContactPlane(p, p0, n);
    }

    n  = ES::Mp<const ES::V3d>(constraintNormals + ci * 3);
    p0 = ES::Mp<const ES::V3d>(constraintTargetPositions + ci * 3);

    if (checkPenetration == 0) {
        return true;
    }

    return isInside(p, p0, n);
}
