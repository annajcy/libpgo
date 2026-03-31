/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#pragma once

#include <memory>
#include <vector>

namespace pgo {
namespace SolidDeformationModel {

class SimulationMeshMaterial {
public:
    SimulationMeshMaterial() {}
    virtual ~SimulationMeshMaterial() {}

    virtual std::unique_ptr<SimulationMeshMaterial> clone() const = 0;
};

class SimulationMeshENuMaterial : public SimulationMeshMaterial {
public:
    SimulationMeshENuMaterial() {}
    SimulationMeshENuMaterial(double E_, double nu_, double J_ = 10000) : E(E_), nu(nu_), J(J_) {}
    virtual ~SimulationMeshENuMaterial() {}

    void setE(double E_) { E = E_; }
    void setNu(double nu_) { nu = nu_; }
    void setJ(double J_) { J = J_; }

    double getMuLame() const { return E / (2 * (1 + nu)); }
    double getLambdaLame() const { return (nu * E) / ((1 + nu) * (1 - 2 * nu)); }

    double getE() const { return E; }
    double getNu() const { return nu; }
    double getCompressionRatio() const { return J; }

    virtual std::unique_ptr<SimulationMeshMaterial> clone() const override {
        return std::make_unique<SimulationMeshENuMaterial>(E, nu, J);
    }

protected:
    double E = 6e3, nu = 0.4;
    double J = 10000;
};

class SimulationMeshENuhMaterial : public SimulationMeshENuMaterial {
public:
    SimulationMeshENuhMaterial() {}
    SimulationMeshENuhMaterial(double E_, double nu_, double h_, double J_ = 10000)
        : SimulationMeshENuMaterial(E_, nu_, J_), h(h_) {}

    virtual ~SimulationMeshENuhMaterial() {}

    void   seth(double h_) { h = h_; }
    double geth() const { return h; }

    virtual std::unique_ptr<SimulationMeshMaterial> clone() const override {
        return std::make_unique<SimulationMeshENuhMaterial>(E, nu, h, J);
    }

protected:
    double h = 1e-4;
};

class SimulationMeshHillMaterial : public SimulationMeshMaterial {
public:
    SimulationMeshHillMaterial() {}
    SimulationMeshHillMaterial(double E_act_, double gamma_, double lo_) : E_act(E_act_), gamma(gamma_), lo(lo_) {}
    virtual ~SimulationMeshHillMaterial() {}

    void setEact(double E_) { E_act = E_; }
    void setGamma(double gamma_) { gamma = gamma_; }
    void setLo(double lo_) { lo = lo_; }

    double getEact() const { return E_act; }
    double getGamma() const { return gamma; }
    double getLo() const { return lo; }

    virtual std::unique_ptr<SimulationMeshMaterial> clone() const override {
        return std::make_unique<SimulationMeshHillMaterial>(E_act, gamma, lo);
    }

protected:
    double E_act = 0.1e6, gamma = 1, lo = 0.6;
};

class SimulationMeshMooneyRivlinMaterial : public SimulationMeshMaterial {
public:
    SimulationMeshMooneyRivlinMaterial(int N, int M, const double* Cpq, const double* D_) {
        this->N = N;
        this->M = M;
        C.assign((N + 1) * (N + 1), 0.0);
        D.assign(M, 0.0);

        for (int p = 0; p <= N; p++) {
            for (int q = 0; q <= N; q++) {
                getC(p, q) = Cpq[q * (N + 1) + p];
            }
        }

        for (int i = 0; i < M; i++) {
            getD(i) = D_[i];
        }
    }

    SimulationMeshMooneyRivlinMaterial(int N, int M, double E, double nu) {
        this->N = N;
        this->M = M;
        C.assign((N + 1) * (N + 1), 0.0);
        D.assign(M, 0.0);

        for (int p = 0; p <= N; p++) {
            for (int q = 0; q <= N; q++) {
                getC(p, q) = 0;
            }
        }

        for (int i = 0; i < M; i++) {
            getD(i) = 0;
        }

        double bulkModulus  = E / (3 * (1 - 2 * nu));
        double shearModulus = E / (2 * (1 + nu));

        getC(1, 0) = shearModulus * 0.5;
        getD(0)    = 2.0 / bulkModulus;
    }

    virtual ~SimulationMeshMooneyRivlinMaterial() {}

    void setC(int p, int q, double val) { getC(p, q) = val; }

    void setD(int i, double val) { getD(i) = val; }

    double& getC(int p, int q) { return C[q * (N + 1) + p]; }

    const double& getC(int p, int q) const { return C[q * (N + 1) + p]; }

    double& getD(int i) { return D[i]; }

    const double& getD(int i) const { return D[i]; }

    const double* getC() const { return C.data(); }
    const double* getD() const { return D.data(); }

    int getM() const { return M; }
    int getN() const { return N; }

    virtual std::unique_ptr<SimulationMeshMaterial> clone() const override {
        return std::make_unique<SimulationMeshMooneyRivlinMaterial>(N, M, C.data(), D.data());
    }

protected:
    int                 M, N;
    std::vector<double> C, D;
};

class SimulationMeshMooneyRivlinhMaterial : public SimulationMeshMooneyRivlinMaterial {
public:
    SimulationMeshMooneyRivlinhMaterial(int N_, int M_, const double* Cpq_, const double* D_, double h_)
        : SimulationMeshMooneyRivlinMaterial(N_, M_, Cpq_, D_), h(h_) {}
    SimulationMeshMooneyRivlinhMaterial(int N_, int M_, double E_, double nu_, double h_)
        : SimulationMeshMooneyRivlinMaterial(N_, M_, E_, nu_), h(h_) {}
    virtual ~SimulationMeshMooneyRivlinhMaterial() {}

    void   seth(double h_) { h = h_; }
    double geth() const { return h; }

    virtual std::unique_ptr<SimulationMeshMaterial> clone() const override {
        return std::make_unique<SimulationMeshMooneyRivlinhMaterial>(N, M, C.data(), D.data(), h);
    }

protected:
    double h = 1e-4;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
