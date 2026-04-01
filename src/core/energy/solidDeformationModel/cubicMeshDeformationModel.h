/*
author: OpenAI Codex
copyright to USC,MIT,NUS
*/

#pragma once

#include "deformationModel.h"

namespace pgo {
namespace SolidDeformationModel {
class CubicMeshDeformationModelInternal;

class CubicMeshDeformationModel : public DeformationModel {
public:
    CubicMeshDeformationModel(const double restPositions[24], ElasticModel* elasticModel, PlasticModel* plasticModel);
    virtual ~CubicMeshDeformationModel() override;

    using DeformationModel::CacheData;

    virtual CacheData* allocateCacheData() const override;
    virtual void       freeCacheData(CacheData* data) const override;
    virtual void       prepareData(const double* x, const double* param, const double* materialParam,
                                   CacheData* cacheData) const override;

    virtual void vonMisesStress(const CacheData* cacheDataBase, int& nPt, double* stresses) const override;
    virtual void maxStrain(const CacheData* cacheDataBase, int& nPt, double* stresses) const override;

    virtual double computeEnergy(const CacheData* cacheData) const override;
    virtual void   compute_dE_dx(const CacheData* cacheData, double* grad) const override;
    virtual void   compute_d2E_dx2(const CacheData* cacheData, double* hess) const override;

    virtual void compute_dE_da(const CacheData* cacheData, double* grad) const override;
    virtual void compute_d2E_da2(const CacheData* cacheData, double* hess) const override;
    virtual void compute_d2E_dxda(const CacheData* cacheData, double* hess) const override;

    virtual void compute_dE_db(const CacheData* cacheData, double* grad) const override;
    virtual void compute_d2E_db2(const CacheData* cacheData, double* hess) const override;
    virtual void compute_d2E_dxdb(const CacheData* cacheData, double* hess) const override;

    virtual void compute_d2E_dadb(const CacheData* cacheData, double* hess) const override;

    virtual int getNumVertices() const override { return 8; }
    virtual int getNumDOFs() const override { return 24; }
    virtual int getNumMaterialLocations() const override { return 8; }

protected:
    CubicMeshDeformationModelInternal* ind;
};
}  // namespace SolidDeformationModel
}  // namespace pgo
