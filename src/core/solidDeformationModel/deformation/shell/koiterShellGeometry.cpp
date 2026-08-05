#include "koiterShellDeformationElement.h"

namespace pgo
{
namespace SolidDeformationModel
{

KoiterShellDeformationElement::FirstFundamentalFormResult KoiterShellDeformationElement::computeFirstFundamentalFormImpl(
  const APositions &x, bool computeDerivative, bool computeHessian) const
{
  ES::V3d e1 = x[1] - x[0];
  ES::V3d e2 = x[2] - x[0];
  FirstFundamentalFormResult result;
  result.value(0, 0) = e1.squaredNorm();
  result.value(0, 1) = e1.dot(e2);
  result.value(1, 0) = result.value(0, 1);
  result.value(1, 1) = e2.squaredNorm();

  if (computeDerivative) {
    result.derivative.setZero();
    result.derivative.block<1, 3>(0, 0) = -2.0 * e1.transpose();
    result.derivative.block<1, 3>(0, 3) = 2.0 * e1.transpose();

    Eigen::RowVector3d da01_dv0 = (-e1 - e2).transpose();
    result.derivative.block<1, 3>(1, 0) = da01_dv0;
    result.derivative.block<1, 3>(1, 3) = e2.transpose();
    result.derivative.block<1, 3>(1, 6) = e1.transpose();
    result.derivative.row(2) = result.derivative.row(1);
    result.derivative.block<1, 3>(3, 0) = -2.0 * e2.transpose();
    result.derivative.block<1, 3>(3, 6) = 2.0 * e2.transpose();
  }

  if (computeHessian) {
    for (int i = 0; i < 4; ++i) {
      result.hessian[i].setZero();
    }

    ES::M3d I3 = ES::M3d::Identity();
    result.hessian[0].block<3, 3>(0, 0) = 2.0 * I3;
    result.hessian[0].block<3, 3>(0, 3) = -2.0 * I3;
    result.hessian[0].block<3, 3>(3, 0) = -2.0 * I3;
    result.hessian[0].block<3, 3>(3, 3) = 2.0 * I3;

    result.hessian[1].block<3, 3>(0, 0) = 2.0 * I3;
    result.hessian[1].block<3, 3>(0, 3) = -I3;
    result.hessian[1].block<3, 3>(0, 6) = -I3;
    result.hessian[1].block<3, 3>(3, 0) = -I3;
    result.hessian[1].block<3, 3>(3, 6) = I3;
    result.hessian[1].block<3, 3>(6, 0) = -I3;
    result.hessian[1].block<3, 3>(6, 3) = I3;

    result.hessian[2] = result.hessian[1];

    result.hessian[3].block<3, 3>(0, 0) = 2.0 * I3;
    result.hessian[3].block<3, 3>(0, 6) = -2.0 * I3;
    result.hessian[3].block<3, 3>(6, 0) = -2.0 * I3;
    result.hessian[3].block<3, 3>(6, 6) = 2.0 * I3;
  }
  return result;
}

KoiterShellDeformationElement::SecondFundamentalFormMatrixResult KoiterShellDeformationElement::computeSecondFundamentalFormImpl(
  const BPositions &x, bool computeDerivative, bool computeHessian) const
{
  SecondFundamentalFormMatrixResult result;
  if (computeDerivative) {
    result.derivative.setZero();
  }
  if (computeHessian) {
    for (int i = 0; i < 4; i++) {
      result.hessian[i].setZero();
    }
  }

  const SecondFundamentalFormEntriesResult II = secondFundamentalFormEntries(
    x, computeDerivative, computeHessian);

  result.value << II.value[0] + II.value[1], II.value[0], II.value[0], II.value[0] + II.value[2];

  if (computeDerivative) {
    result.derivative.row(0) += II.derivative.row(0);
    result.derivative.row(0) += II.derivative.row(1);

    result.derivative.row(1) += II.derivative.row(0);
    result.derivative.row(2) += II.derivative.row(0);

    result.derivative.row(3) += II.derivative.row(0);
    result.derivative.row(3) += II.derivative.row(2);
  }

  if (computeHessian) {
    result.hessian[0] += II.hessian[0];
    result.hessian[0] += II.hessian[1];

    result.hessian[1] += II.hessian[0];
    result.hessian[2] += II.hessian[0];

    result.hessian[3] += II.hessian[0];
    result.hessian[3] += II.hessian[2];
  }

  return result;
}

ES::M2d KoiterShellDeformationElement::computeFirstFundamentalForm(const APositions &x) const
{
  return computeFirstFundamentalFormImpl(x, false, false).value;
}

ES::M4x9d KoiterShellDeformationElement::computeFirstFundamentalFormDerivative(const APositions &x) const
{
  return computeFirstFundamentalFormImpl(x, true, false).derivative;
}

ES::M9x36d KoiterShellDeformationElement::computeFirstFundamentalFormHessian(const APositions &x) const
{
  const FirstFundamentalFormResult result = computeFirstFundamentalFormImpl(x, false, true);
  ES::M9x36d hessian;
  hessian.setZero();
  for (int i = 0; i < 4; ++i)
    hessian.block<9, 9>(0, 9 * i) = result.hessian[i];
  return hessian;
}

ES::M2d KoiterShellDeformationElement::computeSecondFundamentalForm(const BPositions &x) const
{
  return computeSecondFundamentalFormImpl(x, false, false).value;
}

ES::M4x18d KoiterShellDeformationElement::computeSecondFundamentalFormDerivative(const BPositions &x) const
{
  return computeSecondFundamentalFormImpl(x, true, false).derivative;
}

ES::M18x72d KoiterShellDeformationElement::computeSecondFundamentalFormHessian(const BPositions &x) const
{
  const SecondFundamentalFormMatrixResult result = computeSecondFundamentalFormImpl(x, false, true);
  ES::M18x72d hessian;
  hessian.setZero();
  for (int i = 0; i < 4; ++i)
    hessian.block<18, 18>(0, 18 * i) = result.hessian[i];
  return hessian;
}

KoiterShellDeformationElement::SecondFundamentalFormEntriesResult
KoiterShellDeformationElement::secondFundamentalFormEntries(
  const BPositions &x, bool computeDerivative, bool computeHessian) const
{
  SecondFundamentalFormEntriesResult result;
  result.value.setZero();
  if (computeDerivative)
    result.derivative.setZero();
  if (computeHessian) {
    for (int i = 0; i < 3; i++)
      result.hessian[i].setZero();
  }

  std::array<ES::V3d, 3> oppNormals;
  std::array<Eigen::Matrix<double, 3, 9>, 3> dn;
  std::array<std::array<ES::M9d, 3>, 3> hn;

  const FaceNormalResult centerNormal = faceNormal(
    x[0], x[1], x[2], computeDerivative || computeHessian, computeHessian);
  const ES::V3d &cNormal = centerNormal.value;
  const ES::M3x9d &dcn = centerNormal.derivative;
  const std::array<ES::M9d, 3> &hcn = centerNormal.hessian;

  for (int i = 0; i < 3; i++) {
    if (hasVertex_[oppVtx[i]] == 0) {
      oppNormals[i].setZero();
      dn[i].setZero();
      for (int j = 0; j < 3; j++)
        hn[i][j].setZero();
    }
    else {
      ES::V3d x0 = x[oppVtx[i]];
      ES::V3d x1 = x[(i + 2) % 3];
      ES::V3d x2 = x[(i + 1) % 3];

      const FaceNormalResult oppositeNormal = faceNormal(
        x0, x1, x2, computeDerivative || computeHessian, computeHessian);
      oppNormals[i] = oppositeNormal.value;
      if (computeDerivative || computeHessian)
        dn[i] = oppositeNormal.derivative;
      if (computeHessian)
        hn[i] = oppositeNormal.hessian;
    }
  }

  std::array<ES::V3d, 3> qs;
  std::array<ES::V3d, 3> mvec;
  std::array<ES::V3d, 3> qvec;
  std::array<double, 3> mnorms;
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
    result.value[i] = (qs[ip1] + qs[ip2] - 2.0 * qs[i]).dot(oppNormals[i]) / mnorms[i];

    if (computeDerivative) {
      result.derivative.block<1, 3>(i, 3 * i) += -2.0 * oppNormals[i].transpose() / mnorms[i];
      result.derivative.block<1, 3>(i, 3 * ip1) += 1.0 * oppNormals[i].transpose() / mnorms[i];
      result.derivative.block<1, 3>(i, 3 * ip2) += 1.0 * oppNormals[i].transpose() / mnorms[i];

      result.derivative.block<1, 3>(i, 9 + 3 * (oppVtx[i] - 3)) += qvec[i].transpose() / mnorms[i] * dn[i].block<3, 3>(0, 0);
      result.derivative.block<1, 3>(i, 3 * ip2) += qvec[i].transpose() / mnorms[i] * dn[i].block<3, 3>(0, 3);
      result.derivative.block<1, 3>(i, 3 * ip1) += qvec[i].transpose() / mnorms[i] * dn[i].block<3, 3>(0, 6);

      result.derivative.block<1, 3>(i, 9 + 3 * (oppVtx[i] - 3)) += -qvec[i].dot(oppNormals[i]) / mnorms[i] / mnorms[i] / mnorms[i] * mvec[i].transpose() * dn[i].block<3, 3>(0, 0);
      result.derivative.block<1, 3>(i, 3 * ip2) += -qvec[i].dot(oppNormals[i]) / mnorms[i] / mnorms[i] / mnorms[i] * mvec[i].transpose() * dn[i].block<3, 3>(0, 3);
      result.derivative.block<1, 3>(i, 3 * ip1) += -qvec[i].dot(oppNormals[i]) / mnorms[i] / mnorms[i] / mnorms[i] * mvec[i].transpose() * dn[i].block<3, 3>(0, 6);

      result.derivative.block<1, 3>(i, 0) += -qvec[i].dot(oppNormals[i]) / mnorms[i] / mnorms[i] / mnorms[i] * mvec[i].transpose() * dcn.block<3, 3>(0, 0);
      result.derivative.block<1, 3>(i, 3) += -qvec[i].dot(oppNormals[i]) / mnorms[i] / mnorms[i] / mnorms[i] * mvec[i].transpose() * dcn.block<3, 3>(0, 3);
      result.derivative.block<1, 3>(i, 6) += -qvec[i].dot(oppNormals[i]) / mnorms[i] / mnorms[i] / mnorms[i] * mvec[i].transpose() * dcn.block<3, 3>(0, 6);
    }

    if (computeHessian) {
      int ip1 = (i + 1) % 3;
      int ip2 = (i + 2) % 3;

      std::array<int, 3> miidx;
      miidx[0] = 9 + 3 * (oppVtx[i] - 3);
      miidx[1] = 3 * ip2;
      miidx[2] = 3 * ip1;

      std::array<ES::M3d, 3> dnij;
      for (int j = 0; j < 3; j++)
        dnij[j] = dn[i].block<3, 3>(0, 3 * j);

      for (int j = 0; j < 3; j++) {
        result.hessian[i].block<3, 3>(miidx[j], 3 * ip1) += (1.0 / mnorms[i]) * dnij[j].transpose();
        result.hessian[i].block<3, 3>(miidx[j], 3 * ip2) += (1.0 / mnorms[i]) * dnij[j].transpose();
        result.hessian[i].block<3, 3>(miidx[j], 3 * i) += (-2.0 / mnorms[i]) * dnij[j].transpose();

        ES::V3d dnijTm = dnij[j].transpose() * mvec[i];
        ES::V3d dcnjTm = (dcn.block<3, 3>(0, 3 * j).transpose() * mvec[i]);
        ES::M3d term3 = dnijTm * oppNormals[i].transpose();

        result.hessian[i].block<3, 3>(miidx[j], 3 * ip1) += (-1.0 / mnorms[i] / mnorms[i] / mnorms[i]) * term3;
        result.hessian[i].block<3, 3>(miidx[j], 3 * ip2) += (-1.0 / mnorms[i] / mnorms[i] / mnorms[i]) * term3;
        result.hessian[i].block<3, 3>(miidx[j], 3 * i) += (2.0 / mnorms[i] / mnorms[i] / mnorms[i]) * term3;

        ES::M3d term4 = dcnjTm * oppNormals[i].transpose();

        result.hessian[i].block<3, 3>(3 * j, 3 * ip1) += (-1.0 / mnorms[i] / mnorms[i] / mnorms[i]) * term4;
        result.hessian[i].block<3, 3>(3 * j, 3 * ip2) += (-1.0 / mnorms[i] / mnorms[i] / mnorms[i]) * term4;
        result.hessian[i].block<3, 3>(3 * j, 3 * i) += (2.0 / mnorms[i] / mnorms[i] / mnorms[i]) * term4;

        result.hessian[i].block<3, 3>(3 * ip1, miidx[j]) += (1.0 / mnorms[i]) * dnij[j];
        result.hessian[i].block<3, 3>(3 * ip2, miidx[j]) += (1.0 / mnorms[i]) * dnij[j];
        result.hessian[i].block<3, 3>(3 * i, miidx[j]) += (-2.0 / mnorms[i]) * dnij[j];

        for (int k = 0; k < 3; k++) {
          result.hessian[i].block<3, 3>(miidx[j], miidx[k]) += (-1.0 / mnorms[i] / mnorms[i] / mnorms[i]) * (dnij[j].transpose() * mvec[i]) * (qvec[i].transpose() * dnij[k]);
          result.hessian[i].block<3, 3>(3 * j, miidx[k]) += (-1.0 / mnorms[i] / mnorms[i] / mnorms[i]) * (dcn.block<3, 3>(0, 3 * j).transpose() * mvec[i]) * (qvec[i].transpose() * dnij[k]);
        }

        ES::M3d term1 = oppNormals[i] * dnijTm.transpose();
        result.hessian[i].block<3, 3>(3 * ip1, miidx[j]) += (-1.0 / mnorms[i] / mnorms[i] / mnorms[i]) * term1;
        result.hessian[i].block<3, 3>(3 * ip2, miidx[j]) += (-1.0 / mnorms[i] / mnorms[i] / mnorms[i]) * term1;
        result.hessian[i].block<3, 3>(3 * i, miidx[j]) += (2.0 / mnorms[i] / mnorms[i] / mnorms[i]) * term1;

        ES::M3d term2 = oppNormals[i] * dcnjTm.transpose();
        result.hessian[i].block<3, 3>(3 * ip1, 3 * j) += (-1.0 / mnorms[i] / mnorms[i] / mnorms[i]) * term2;
        result.hessian[i].block<3, 3>(3 * ip2, 3 * j) += (-1.0 / mnorms[i] / mnorms[i] / mnorms[i]) * term2;
        result.hessian[i].block<3, 3>(3 * i, 3 * j) += (2.0 / mnorms[i] / mnorms[i] / mnorms[i]) * term2;

        ES::V3d dnijTq = dnij[j].transpose() * qvec[i];

        for (int k = 0; k < 3; k++) {
          result.hessian[i].block<3, 3>(miidx[j], miidx[k]) += (-1.0 / mnorms[i] / mnorms[i] / mnorms[i]) * dnijTq * (mvec[i].transpose() * dnij[k]);
          result.hessian[i].block<3, 3>(miidx[j], 3 * k) += (-1.0 / mnorms[i] / mnorms[i] / mnorms[i]) * dnijTq * (mvec[i].transpose() * dcn.block<3, 3>(0, 3 * k));
        }

        double qdoto = qvec[i].dot(oppNormals[i]);

        for (int k = 0; k < 3; k++) {
          result.hessian[i].block<3, 3>(miidx[j], miidx[k]) += (-1.0 / mnorms[i] / mnorms[i] / mnorms[i]) * qdoto * dnij[j].transpose() * dnij[k];
          result.hessian[i].block<3, 3>(miidx[j], 3 * k) += (-1.0 / mnorms[i] / mnorms[i] / mnorms[i]) * qdoto * dnij[j].transpose() * dcn.block<3, 3>(0, 3 * k);
          result.hessian[i].block<3, 3>(3 * j, miidx[k]) += (-1.0 / mnorms[i] / mnorms[i] / mnorms[i]) * qdoto * dcn.block<3, 3>(0, 3 * j).transpose() * dnij[k];
          result.hessian[i].block<3, 3>(3 * j, 3 * k) += (-1.0 / mnorms[i] / mnorms[i] / mnorms[i]) * qdoto * dcn.block<3, 3>(0, 3 * j).transpose() * dcn.block<3, 3>(0, 3 * k);

          result.hessian[i].block<3, 3>(miidx[j], miidx[k]) += (3.0 / mnorms[i] / mnorms[i] / mnorms[i] / mnorms[i] / mnorms[i]) * qdoto * dnijTm * (mvec[i].transpose() * dnij[k]);
          result.hessian[i].block<3, 3>(miidx[j], 3 * k) += (3.0 / mnorms[i] / mnorms[i] / mnorms[i] / mnorms[i] / mnorms[i]) * qdoto * dnijTm * (mvec[i].transpose() * dcn.block<3, 3>(0, 3 * k));
          result.hessian[i].block<3, 3>(3 * j, miidx[k]) += (3.0 / mnorms[i] / mnorms[i] / mnorms[i] / mnorms[i] / mnorms[i]) * qdoto * dcnjTm * (mvec[i].transpose() * dnij[k]);
          result.hessian[i].block<3, 3>(3 * j, 3 * k) += (3.0 / mnorms[i] / mnorms[i] / mnorms[i] / mnorms[i] / mnorms[i]) * qdoto * dcnjTm * (mvec[i].transpose() * dcn.block<3, 3>(0, 3 * k));

          for (int l = 0; l < 3; l++) {
            result.hessian[i].block<3, 3>(miidx[j], miidx[k]) += (-1.0 / mnorms[i] / mnorms[i] / mnorms[i]) * qdoto * mvec[i][l] * hn[i][l].block<3, 3>(3 * j, 3 * k);
            result.hessian[i].block<3, 3>(3 * j, 3 * k) += (-1.0 / mnorms[i] / mnorms[i] / mnorms[i]) * qdoto * mvec[i][l] * hcn[l].block<3, 3>(3 * j, 3 * k);
            result.hessian[i].block<3, 3>(miidx[j], miidx[k]) += (1.0 / mnorms[i]) * qvec[i][l] * hn[i][l].block<3, 3>(3 * j, 3 * k);
          }
        }
      }
    }
  }

  return result;
}

ES::M3d KoiterShellDeformationElement::crossMatrix(const Eigen::Vector3d &v)
{
  ES::M3d ret;
  ret << 0, -v[2], v[1],
    v[2], 0, -v[0],
    -v[1], v[0], 0;
  return ret;
}

KoiterShellDeformationElement::FaceNormalResult KoiterShellDeformationElement::faceNormal(
  const ES::V3d &x0, const ES::V3d &x1, const ES::V3d &x2,
  bool computeDerivative, bool computeHessian) const
{
  FaceNormalResult result;
  if (computeDerivative)
    result.derivative.setZero();
  if (computeHessian) {
    for (int i = 0; i < 3; i++)
      result.hessian[i].setZero();
  }

  ES::V3d n = (x1 - x0).cross(x2 - x0);

  if (computeDerivative) {
    result.derivative.block(0, 0, 3, 3) += crossMatrix(x2 - x1);
    result.derivative.block(0, 3, 3, 3) += crossMatrix(x0 - x2);
    result.derivative.block(0, 6, 3, 3) += crossMatrix(x1 - x0);
  }

  if (computeHessian) {
    for (int j = 0; j < 3; j++) {
      Eigen::Vector3d ej(0, 0, 0);
      ej[j] = 1.0;
      ES::M3d ejc = crossMatrix(ej);
      result.hessian[j].block(0, 3, 3, 3) -= ejc;
      result.hessian[j].block(0, 6, 3, 3) += ejc;
      result.hessian[j].block(3, 6, 3, 3) -= ejc;
      result.hessian[j].block(3, 0, 3, 3) += ejc;
      result.hessian[j].block(6, 0, 3, 3) -= ejc;
      result.hessian[j].block(6, 3, 3, 3) += ejc;
    }
  }

  result.value = n;
  return result;
}

}  // namespace SolidDeformationModel
}  // namespace pgo
