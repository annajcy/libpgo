#pragma once

class MaterialSpecCore {
public:
    MaterialSpecCore(double E = 1e9, double nu = 0.45, double density = 1000.0)
        : E_(E), nu_(nu), density_(density) {}

    double E() const { return E_; }
    double nu() const { return nu_; }
    double density() const { return density_; }

    void setE(double E) { E_ = E; }
    void setNu(double nu) { nu_ = nu; }
    void setDensity(double density) { density_ = density; }

private:
    double E_;
    double nu_;
    double density_;
};
