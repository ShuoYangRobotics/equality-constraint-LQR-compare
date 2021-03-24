/**
 * @file     EcLqrParams.h
 * @brief    Parameters relevant to equality constarined LQR
 * @author   Gerry Chen
 * @author   Shuo Yang
 * @author   Yetong Zhang
 */

#pragma once

#include <gtsam/base/Matrix.h>

#include <map>

namespace ecLqr {

using gtsam::Matrix;
using gtsam::Vector;

template <int N, int M>
struct EcLqrParams {
  typedef Eigen::Matrix<double, N, 1> VectorN;
	size_t T;
	VectorN x0;
	VectorN xf;
  Eigen::Matrix<double, N, N> A;
  Eigen::Matrix<double, N, M> B;
  Eigen::Matrix<double, N, N> Q;
  Eigen::Matrix<double, N, N> Qf;
  Eigen::Matrix<double, M, M> R;

  struct XUConstraint {
    Matrix C;
    Matrix D;
    Vector r;
  };
  std::map<size_t, XUConstraint> xuConstraints;

  struct XConstraint {
    Matrix G;
    Vector h;
  };
  std::map<size_t, XConstraint> xConstraints;
};

template <int N, int M>
using Gains = std::pair<std::vector<Eigen::Matrix<double, M, N> >,
                        std::vector<Eigen::Matrix<double, M, 1> > >;

}  // namespace ecLqr
