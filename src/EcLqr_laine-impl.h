/**
 * @file     EcLqr_laine-impl.h
 * @brief    Functions for solving equality constrained LQR using Laine19's
 * technique
 * @author   Gerry Chen
 * @author   Shuo Yang
 * @author   Yetong Zhang
 */

#pragma once

#include <gtsam/linear/VectorValues.h>
#include <gtsam/inference/Symbol.h>

#include <Eigen/StdVector>

namespace ecLqr {

using gtsam::VectorValues;
using gtsam::Symbol;

template <int N, int M>
VectorValues laineSolFromParams(const EcLqrParams<N, M> &params) {
  gttic_(solve);
  return laineSolFromGains(laineGainsFromParams(params), params);
}

template <int N, int M>
gtsam::VectorValues laineSolFromGains(const Gains<N, M> &gains,
                                      const EcLqrParams<N, M> &params) {
  gttic_(calculate_openloop_forwardpass);
  // forward pass to calculate (open-loop) x/u
  gtsam::VectorValues sol;
  Eigen::Matrix<double, N, 1> x = params.x0;
  Eigen::Matrix<double, M, 1> u;
  for (size_t t = 0; t < params.T; ++t) {
    sol.emplace(Symbol('x', t), x);
    u = gains.first[t] * x + gains.second[t];
    sol.emplace(Symbol('u', t), u);
    x = params.A * x + params.B * u;
  }
  sol.emplace(Symbol('x', params.T), x);
  return sol;
}

template <int N, int M>
Gains<N, M> laineGainsFromParams(const EcLqrParams<N, M> &params) {
  gttic_(calculate_gains);
  using Eigen::Matrix;
  using Eigen::MatrixXd;

  // 1. convenience declarations
  const Matrix<double, N, N> &A = params.A;
  const Matrix<double, N, M> &B = params.B;
  const Matrix<double, N, N> &Q = params.Q;
  const Matrix<double, M, M> &R = params.R;
  const Matrix<double, N, N> &Qf = params.Qf;
  const auto &T = params.T;

  // 2. cost-to-go and constraint-to-go initializations
  Matrix<double, N, N> Vxxt1 = Qf;
  Matrix<double, N, 1> vxlt1 = -Qf * params.xf;
  MatrixXd Hxt1, hlt1;
  auto constr = params.xConstraints.find(T);
  if (constr != params.xConstraints.end()) {
    Hxt1 = constr->second.G;
    hlt1 = constr->second.h;
  } else {
    Hxt1 = MatrixXd(0, N);
    hlt1 = MatrixXd(0, 1);
  }

  // 3. backwards pass to calculate K & k matrices
  Gains<N, M> gains;
  gains.first.reserve(T);
  gains.second.reserve(T);
  // iterate
  for (int t = T - 1; t >= 0; --t) {
    // eq. 12
    Matrix<double, N, 1> mxlt = A.transpose() * vxlt1;
    Matrix<double, M, 1> mult = B.transpose() * vxlt1;
    Matrix<double, N, N> Mxxt = Q + A.transpose() * Vxxt1 * A;
    Matrix<double, M, M> Muut = R + B.transpose() * Vxxt1 * B;
    Matrix<double, M, N> Muxt = B.transpose() * Vxxt1 * A;

    // assemble all constraints that act on this time step
    MatrixXd Nxt, Nut, nlt, C, D, r, G, G0, h;
    auto xuconstr = params.xuConstraints.find(t);
    if (xuconstr != params.xuConstraints.end()) {
      C = xuconstr->second.C; D = xuconstr->second.D; r = xuconstr->second.r;
    } else {
      C = MatrixXd(0, N); D = MatrixXd(0, M); r = MatrixXd(0, 1);
    }
    constr = params.xConstraints.find(t);
    if (constr != params.xConstraints.end()) {
      G = constr->second.G; G0 = MatrixXd::Zero(G.rows(), M); h = constr->second.h;
    } else {
      G = MatrixXd(0, N); G0 = MatrixXd(0, M); h = MatrixXd(0, 1);
    }
    int nrows = C.rows() + G.rows() + Hxt1.rows();

    Nxt = (MatrixXd(nrows, N) << C, G, Hxt1 * A).finished();
    Nut = (MatrixXd(nrows, M) << D, G0, Hxt1 * B).finished();
    nlt = (MatrixXd(nrows, 1) << r, h, hlt1).finished();

    // Compute K / k
    Matrix<double, M, N> &K = gains.first[t];
    Matrix<double, M, 1> &k = gains.second[t];
    if (nrows == 0) {
      // unconstrained case.  Note: svd segfaults when nrows == 0
      const MatrixXd prefix = Muut.inverse();
      K = -( prefix * Muxt );
      k = -( prefix * mult );
    } else {
      // constrained case: equations 17 and 18
      Eigen::JacobiSVD<MatrixXd> svd(Nut,
                                     Eigen::ComputeThinU | Eigen::ComputeFullV);
      const MatrixXd &V = svd.matrixV();
      if (svd.rank() == 0) {
        const MatrixXd &Z = V;
        const MatrixXd tmp = Z.transpose() * Muut * Z;
        const MatrixXd prefix = Z * Muut.inverse() * Z.transpose();
        K = -( prefix * Muxt );
        k = -( prefix * mult );
      } else if (svd.rank() == M) {
        const MatrixXd &P = V;
        const auto &s = svd.singularValues();
        MatrixXd pinv = MatrixXd(M, Nut.rows());
        for (int row = 0; row < M; ++row)
          pinv.row(row) = 1/s[row] * svd.matrixU().col(row).transpose();
        const MatrixXd prefix = P * pinv;
        K = -( prefix * Nxt );
        k = -( prefix * nlt );
      } else {
        const MatrixXd &P = V.leftCols(svd.rank());
        const MatrixXd &Z = V.rightCols(M-svd.rank());
        const MatrixXd tmp = Z.transpose() * Muut * Z;
        const MatrixXd prefixA = Z * tmp.inverse() * Z.transpose();

        const Eigen::VectorXd &s = svd.singularValues();
        MatrixXd pinv = MatrixXd(svd.rank(), Nut.rows());
        for (int row = 0; row < svd.rank(); ++row)
          pinv.row(row) = 1/s[row] * svd.matrixU().col(row);
        const MatrixXd prefixB = P * pinv;

        K = -( prefixB * Nxt + prefixA * Muxt );
        k = -( prefixB * nlt + prefixA * mult);
      }
    }

    // update constraint and cost to go
    Hxt1 = Nxt + Nut * K;
    hlt1 = nlt + Nut * k;
    MatrixXd Vxxt = Mxxt + 2 * K.transpose() * Muxt + K.transpose() * Muut * K;
    Vxxt1 = (Vxxt + Vxxt.transpose()) / 2;
    vxlt1 = mxlt + K.transpose() * mult +
            (Muxt.transpose() + K.transpose() * Muut) * k;

    Eigen::Map<Eigen::RowVectorXd> Vxxt1print(Vxxt1.data(), Vxxt1.size());

    // reduce H/h
    if (hlt1.rows() > 0) {
      gttic_(reduceH);
      // remove redundant terms, the paragraph below equation 21
      MatrixXd cxt1 = (MatrixXd(hlt1.rows(), N+1) << hlt1, Hxt1).finished();
      #define SLOWER_BUT_MORE_ACCURATEx
      #ifdef SLOWER_BUT_MORE_ACCURATE
      // SVD version (slower)
      Eigen::JacobiSVD<MatrixXd> svd2(
          cxt1, Eigen::ComputeThinU | Eigen::ComputeThinV);
      svd2.setThreshold(1e-4);
      if (abs(svd2.singularValues()(0)) < 1e-6) {
        hlt1 = MatrixXd(0, 1);
        Hxt1 = MatrixXd(0, N);
      } else {
        MatrixXd VT = svd2.matrixV().transpose();
        hlt1 = VT.block(0, 0, svd2.rank(), 1);
        Hxt1 = VT.block(0, 1, svd2.rank(), N);
      }
      #else
      // LU decomposition version (faster but less numerically stable)
      Eigen::FullPivLU<MatrixXd> lu_decomp(cxt1.transpose());
      if (lu_decomp.maxPivot() < 1e-6) {
        hlt1 = MatrixXd(0, 1);
        Hxt1 = MatrixXd(0, N);
      } else {
        const MatrixXd &newC = lu_decomp.image(cxt1.transpose());
        hlt1 = newC.topRows(1).transpose();
        Hxt1 = newC.bottomRows(newC.rows()-1).transpose();
      }
      #endif
    }
  }

  return gains;
}

}  // namespace ecLqr
