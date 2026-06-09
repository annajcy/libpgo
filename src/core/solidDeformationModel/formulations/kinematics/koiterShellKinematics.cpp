#include "koiterShellKinematics.h"

namespace pgo
{
namespace SolidDeformationModel
{

KoiterShellKinematics::KoiterShellKinematics(const double restX[18], const bool hasVtx[6])
{
  for (int i = 0; i < 6; i++) {
    restX_[i] = ES::V3d(restX[3 * i], restX[3 * i + 1], restX[3 * i + 2]);
    hasVtx_[i] = hasVtx[i];
  }
  restI_ = compute_a_and_derivatives(restX_, nullptr, nullptr);
  restII_ = compute_b_and_derivatives(restX_, nullptr, nullptr);
  restArea_ = 0.5 * (restX_[1] - restX_[0]).cross(restX_[2] - restX_[0]).norm();
}

ES::M2d KoiterShellKinematics::compute_a_and_derivatives(
  const ES::V3d x[3],
  Eigen::Matrix<double, 4, 9> *da_dx,
  ES::M9d ahess[4]) const
{
  ES::V3d e1 = x[1] - x[0];
  ES::V3d e2 = x[2] - x[0];
  ES::M2d a;
  a(0, 0) = e1.squaredNorm();
  a(0, 1) = e1.dot(e2);
  a(1, 0) = a(0, 1);
  a(1, 1) = e2.squaredNorm();

  if (da_dx) {
    da_dx->setZero();
    da_dx->block<1, 3>(0, 0) = -2.0 * e1.transpose();
    da_dx->block<1, 3>(0, 3) = 2.0 * e1.transpose();

    Eigen::RowVector3d da01_dv0 = (-e1 - e2).transpose();
    da_dx->block<1, 3>(1, 0) = da01_dv0;
    da_dx->block<1, 3>(1, 3) = e2.transpose();
    da_dx->block<1, 3>(1, 6) = e1.transpose();
    da_dx->row(2) = da_dx->row(1);
    da_dx->block<1, 3>(3, 0) = -2.0 * e2.transpose();
    da_dx->block<1, 3>(3, 6) = 2.0 * e2.transpose();
  }

  if (ahess) {
    for (int i = 0; i < 4; ++i) {
      ahess[i].setZero();
    }

    ES::M3d I3 = ES::M3d::Identity();
    ahess[0].block<3, 3>(0, 0) = 2.0 * I3;
    ahess[0].block<3, 3>(0, 3) = -2.0 * I3;
    ahess[0].block<3, 3>(3, 0) = -2.0 * I3;
    ahess[0].block<3, 3>(3, 3) = 2.0 * I3;

    ahess[1].block<3, 3>(0, 0) = 2.0 * I3;
    ahess[1].block<3, 3>(0, 3) = -I3;
    ahess[1].block<3, 3>(0, 6) = -I3;
    ahess[1].block<3, 3>(3, 0) = -I3;
    ahess[1].block<3, 3>(3, 6) = I3;
    ahess[1].block<3, 3>(6, 0) = -I3;
    ahess[1].block<3, 3>(6, 3) = I3;

    ahess[2] = ahess[1];

    ahess[3].block<3, 3>(0, 0) = 2.0 * I3;
    ahess[3].block<3, 3>(0, 6) = -2.0 * I3;
    ahess[3].block<3, 3>(6, 0) = -2.0 * I3;
    ahess[3].block<3, 3>(6, 6) = 2.0 * I3;
  }
  return a;
}

ES::M2d KoiterShellKinematics::compute_b_and_derivatives(
  const ES::V3d x[6],
  Eigen::Matrix<double, 4, 18> *db_dx,
  ES::M18d bhess[4]) const
{
  if (db_dx) {
    db_dx->setZero();
  }
  if (bhess) {
    for (int i = 0; i < 4; i++) {
      bhess[i].setZero();
    }
  }
  Eigen::Matrix<double, 3, 18> IIderiv;
  ES::M18d IIhess[3];

  ES::V3d II = secondFundamentalFormEntries(x,
    db_dx ? &IIderiv : nullptr,
    bhess ? IIhess : nullptr);

  ES::M2d result;
  result << II[0] + II[1], II[0], II[0], II[0] + II[2];

  if (db_dx) {
    db_dx->row(0) += IIderiv.row(0);
    db_dx->row(0) += IIderiv.row(1);

    db_dx->row(1) += IIderiv.row(0);
    db_dx->row(2) += IIderiv.row(0);

    db_dx->row(3) += IIderiv.row(0);
    db_dx->row(3) += IIderiv.row(2);
  }

  if (bhess) {
    bhess[0] += IIhess[0];
    bhess[0] += IIhess[1];

    bhess[1] += IIhess[0];
    bhess[2] += IIhess[0];

    bhess[3] += IIhess[0];
    bhess[3] += IIhess[2];
  }

  return result;
}

ES::V3d KoiterShellKinematics::secondFundamentalFormEntries(
  const ES::V3d x[6],
  Eigen::Matrix<double, 3, 18> *derivative,
  ES::M18d hessian[3]) const
{
  if (derivative)
    derivative->setZero();

  if (hessian) {
    for (int i = 0; i < 3; i++)
      hessian[i].setZero();
  }
  ES::V3d II;

  ES::V3d oppNormals[3];
  Eigen::Matrix<double, 3, 9> dn[3];
  ES::M9d hn[3][3];

  Eigen::Matrix<double, 3, 9> dcn;
  ES::M9d hcn[3];
  ES::V3d cNormal = faceNormal(x[0], x[1], x[2],
    (derivative || hessian) ? &dcn : nullptr,
    hessian ? hcn : nullptr);

  for (int i = 0; i < 3; i++) {
    if (hasVtx_[oppVtx[i]] == 0) {
      oppNormals[i].setZero();
      dn[i].setZero();
      for (int j = 0; j < 3; j++)
        hn[i][j].setZero();
    }
    else {
      ES::V3d x0 = x[oppVtx[i]];
      ES::V3d x1 = x[(i + 2) % 3];
      ES::V3d x2 = x[(i + 1) % 3];

      oppNormals[i] = faceNormal(x0, x1, x2,
        (derivative || hessian) ? &dn[i] : nullptr,
        hessian ? hn[i] : nullptr);
    }
  }

  ES::V3d qs[3];
  ES::V3d mvec[3];
  ES::V3d qvec[3];
  double mnorms[3];
  for (int i = 0; i < 3; i++) {
    qs[i] = x[i];
    mvec[i] = oppNormals[i] + cNormal;
    mnorms[i] = mvec[i].norm();
  }

  for (int i = 0; i < 3; i++) {
    int ip1 = (i + 1) % 3;
    int ip2 = (i + 2) % 3;
    qvec[i] = qs[ip1] + qs[ip2] - 2.0 * qs[i];
  }

  for (int i = 0; i < 3; i++) {
    int ip1 = (i + 1) % 3;
    int ip2 = (i + 2) % 3;
    II[i] = (qs[ip1] + qs[ip2] - 2.0 * qs[i]).dot(oppNormals[i]) / mnorms[i];

    if (derivative) {
      derivative->block<1, 3>(i, 3 * i) += -2.0 * oppNormals[i].transpose() / mnorms[i];
      derivative->block<1, 3>(i, 3 * ip1) += 1.0 * oppNormals[i].transpose() / mnorms[i];
      derivative->block<1, 3>(i, 3 * ip2) += 1.0 * oppNormals[i].transpose() / mnorms[i];

      derivative->block<1, 3>(i, 9 + 3 * (oppVtx[i] - 3)) += qvec[i].transpose() / mnorms[i] * dn[i].block<3, 3>(0, 0);
      derivative->block<1, 3>(i, 3 * ip2) += qvec[i].transpose() / mnorms[i] * dn[i].block<3, 3>(0, 3);
      derivative->block<1, 3>(i, 3 * ip1) += qvec[i].transpose() / mnorms[i] * dn[i].block<3, 3>(0, 6);

      derivative->block<1, 3>(i, 9 + 3 * (oppVtx[i] - 3)) += -qvec[i].dot(oppNormals[i]) / mnorms[i] / mnorms[i] / mnorms[i] * mvec[i].transpose() * dn[i].block<3, 3>(0, 0);
      derivative->block<1, 3>(i, 3 * ip2) += -qvec[i].dot(oppNormals[i]) / mnorms[i] / mnorms[i] / mnorms[i] * mvec[i].transpose() * dn[i].block<3, 3>(0, 3);
      derivative->block<1, 3>(i, 3 * ip1) += -qvec[i].dot(oppNormals[i]) / mnorms[i] / mnorms[i] / mnorms[i] * mvec[i].transpose() * dn[i].block<3, 3>(0, 6);

      derivative->block<1, 3>(i, 0) += -qvec[i].dot(oppNormals[i]) / mnorms[i] / mnorms[i] / mnorms[i] * mvec[i].transpose() * dcn.block<3, 3>(0, 0);
      derivative->block<1, 3>(i, 3) += -qvec[i].dot(oppNormals[i]) / mnorms[i] / mnorms[i] / mnorms[i] * mvec[i].transpose() * dcn.block<3, 3>(0, 3);
      derivative->block<1, 3>(i, 6) += -qvec[i].dot(oppNormals[i]) / mnorms[i] / mnorms[i] / mnorms[i] * mvec[i].transpose() * dcn.block<3, 3>(0, 6);
    }

    if (hessian) {
      int ip1 = (i + 1) % 3;
      int ip2 = (i + 2) % 3;

      int miidx[3];
      miidx[0] = 9 + 3 * (oppVtx[i] - 3);
      miidx[1] = 3 * ip2;
      miidx[2] = 3 * ip1;

      ES::M3d dnij[3];
      for (int j = 0; j < 3; j++)
        dnij[j] = dn[i].block<3, 3>(0, 3 * j);

      for (int j = 0; j < 3; j++) {
        hessian[i].block<3, 3>(miidx[j], 3 * ip1) += (1.0 / mnorms[i]) * dnij[j].transpose();
        hessian[i].block<3, 3>(miidx[j], 3 * ip2) += (1.0 / mnorms[i]) * dnij[j].transpose();
        hessian[i].block<3, 3>(miidx[j], 3 * i) += (-2.0 / mnorms[i]) * dnij[j].transpose();

        ES::V3d dnijTm = dnij[j].transpose() * mvec[i];
        ES::V3d dcnjTm = (dcn.block<3, 3>(0, 3 * j).transpose() * mvec[i]);
        ES::M3d term3 = dnijTm * oppNormals[i].transpose();

        hessian[i].block<3, 3>(miidx[j], 3 * ip1) += (-1.0 / mnorms[i] / mnorms[i] / mnorms[i]) * term3;
        hessian[i].block<3, 3>(miidx[j], 3 * ip2) += (-1.0 / mnorms[i] / mnorms[i] / mnorms[i]) * term3;
        hessian[i].block<3, 3>(miidx[j], 3 * i) += (2.0 / mnorms[i] / mnorms[i] / mnorms[i]) * term3;

        ES::M3d term4 = dcnjTm * oppNormals[i].transpose();

        hessian[i].block<3, 3>(3 * j, 3 * ip1) += (-1.0 / mnorms[i] / mnorms[i] / mnorms[i]) * term4;
        hessian[i].block<3, 3>(3 * j, 3 * ip2) += (-1.0 / mnorms[i] / mnorms[i] / mnorms[i]) * term4;
        hessian[i].block<3, 3>(3 * j, 3 * i) += (2.0 / mnorms[i] / mnorms[i] / mnorms[i]) * term4;

        hessian[i].block<3, 3>(3 * ip1, miidx[j]) += (1.0 / mnorms[i]) * dnij[j];
        hessian[i].block<3, 3>(3 * ip2, miidx[j]) += (1.0 / mnorms[i]) * dnij[j];
        hessian[i].block<3, 3>(3 * i, miidx[j]) += (-2.0 / mnorms[i]) * dnij[j];

        for (int k = 0; k < 3; k++) {
          hessian[i].block<3, 3>(miidx[j], miidx[k]) += (-1.0 / mnorms[i] / mnorms[i] / mnorms[i]) * (dnij[j].transpose() * mvec[i]) * (qvec[i].transpose() * dnij[k]);
          hessian[i].block<3, 3>(3 * j, miidx[k]) += (-1.0 / mnorms[i] / mnorms[i] / mnorms[i]) * (dcn.block<3, 3>(0, 3 * j).transpose() * mvec[i]) * (qvec[i].transpose() * dnij[k]);
        }

        ES::M3d term1 = oppNormals[i] * dnijTm.transpose();
        hessian[i].block<3, 3>(3 * ip1, miidx[j]) += (-1.0 / mnorms[i] / mnorms[i] / mnorms[i]) * term1;
        hessian[i].block<3, 3>(3 * ip2, miidx[j]) += (-1.0 / mnorms[i] / mnorms[i] / mnorms[i]) * term1;
        hessian[i].block<3, 3>(3 * i, miidx[j]) += (2.0 / mnorms[i] / mnorms[i] / mnorms[i]) * term1;

        ES::M3d term2 = oppNormals[i] * dcnjTm.transpose();
        hessian[i].block<3, 3>(3 * ip1, 3 * j) += (-1.0 / mnorms[i] / mnorms[i] / mnorms[i]) * term2;
        hessian[i].block<3, 3>(3 * ip2, 3 * j) += (-1.0 / mnorms[i] / mnorms[i] / mnorms[i]) * term2;
        hessian[i].block<3, 3>(3 * i, 3 * j) += (2.0 / mnorms[i] / mnorms[i] / mnorms[i]) * term2;

        ES::V3d dnijTq = dnij[j].transpose() * qvec[i];

        for (int k = 0; k < 3; k++) {
          hessian[i].block<3, 3>(miidx[j], miidx[k]) += (-1.0 / mnorms[i] / mnorms[i] / mnorms[i]) * dnijTq * (mvec[i].transpose() * dnij[k]);
          hessian[i].block<3, 3>(miidx[j], 3 * k) += (-1.0 / mnorms[i] / mnorms[i] / mnorms[i]) * dnijTq * (mvec[i].transpose() * dcn.block<3, 3>(0, 3 * k));
        }

        double qdoto = qvec[i].dot(oppNormals[i]);

        for (int k = 0; k < 3; k++) {
          hessian[i].block<3, 3>(miidx[j], miidx[k]) += (-1.0 / mnorms[i] / mnorms[i] / mnorms[i]) * qdoto * dnij[j].transpose() * dnij[k];
          hessian[i].block<3, 3>(miidx[j], 3 * k) += (-1.0 / mnorms[i] / mnorms[i] / mnorms[i]) * qdoto * dnij[j].transpose() * dcn.block<3, 3>(0, 3 * k);
          hessian[i].block<3, 3>(3 * j, miidx[k]) += (-1.0 / mnorms[i] / mnorms[i] / mnorms[i]) * qdoto * dcn.block<3, 3>(0, 3 * j).transpose() * dnij[k];
          hessian[i].block<3, 3>(3 * j, 3 * k) += (-1.0 / mnorms[i] / mnorms[i] / mnorms[i]) * qdoto * dcn.block<3, 3>(0, 3 * j).transpose() * dcn.block<3, 3>(0, 3 * k);

          hessian[i].block<3, 3>(miidx[j], miidx[k]) += (3.0 / mnorms[i] / mnorms[i] / mnorms[i] / mnorms[i] / mnorms[i]) * qdoto * dnijTm * (mvec[i].transpose() * dnij[k]);
          hessian[i].block<3, 3>(miidx[j], 3 * k) += (3.0 / mnorms[i] / mnorms[i] / mnorms[i] / mnorms[i] / mnorms[i]) * qdoto * dnijTm * (mvec[i].transpose() * dcn.block<3, 3>(0, 3 * k));
          hessian[i].block<3, 3>(3 * j, miidx[k]) += (3.0 / mnorms[i] / mnorms[i] / mnorms[i] / mnorms[i] / mnorms[i]) * qdoto * dcnjTm * (mvec[i].transpose() * dnij[k]);
          hessian[i].block<3, 3>(3 * j, 3 * k) += (3.0 / mnorms[i] / mnorms[i] / mnorms[i] / mnorms[i] / mnorms[i]) * qdoto * dcnjTm * (mvec[i].transpose() * dcn.block<3, 3>(0, 3 * k));

          for (int l = 0; l < 3; l++) {
            hessian[i].block<3, 3>(miidx[j], miidx[k]) += (-1.0 / mnorms[i] / mnorms[i] / mnorms[i]) * qdoto * mvec[i][l] * hn[i][l].block<3, 3>(3 * j, 3 * k);
            hessian[i].block<3, 3>(3 * j, 3 * k) += (-1.0 / mnorms[i] / mnorms[i] / mnorms[i]) * qdoto * mvec[i][l] * hcn[l].block<3, 3>(3 * j, 3 * k);
            hessian[i].block<3, 3>(miidx[j], miidx[k]) += (1.0 / mnorms[i]) * qvec[i][l] * hn[i][l].block<3, 3>(3 * j, 3 * k);
          }
        }
      }
    }
  }

  return II;
}

ES::M3d KoiterShellKinematics::crossMatrix(const Eigen::Vector3d &v)
{
  ES::M3d ret;
  ret << 0, -v[2], v[1],
    v[2], 0, -v[0],
    -v[1], v[0], 0;
  return ret;
}

ES::V3d KoiterShellKinematics::faceNormal(
  const ES::V3d x0, const ES::V3d x1, const ES::V3d x2,
  Eigen::Matrix<double, 3, 9> *derivative,
  ES::M9d hessian[3]) const
{
  if (derivative)
    derivative->setZero();

  if (hessian) {
    for (int i = 0; i < 3; i++)
      hessian[i].setZero();
  }

  ES::V3d n = (x1 - x0).cross(x2 - x0);

  if (derivative) {
    derivative->block(0, 0, 3, 3) += crossMatrix(x2 - x1);
    derivative->block(0, 3, 3, 3) += crossMatrix(x0 - x2);
    derivative->block(0, 6, 3, 3) += crossMatrix(x1 - x0);
  }

  if (hessian) {
    for (int j = 0; j < 3; j++) {
      Eigen::Vector3d ej(0, 0, 0);
      ej[j] = 1.0;
      ES::M3d ejc = crossMatrix(ej);
      hessian[j].block(0, 3, 3, 3) -= ejc;
      hessian[j].block(0, 6, 3, 3) += ejc;
      hessian[j].block(3, 6, 3, 3) -= ejc;
      hessian[j].block(3, 0, 3, 3) += ejc;
      hessian[j].block(6, 0, 3, 3) -= ejc;
      hessian[j].block(6, 3, 3, 3) += ejc;
    }
  }

  return n;
}

}  // namespace SolidDeformationModel
}  // namespace pgo
