/**
 * @file     EcLqr_fg-impl.h
 * @brief    Useful functions for generating factor graphs which represent
 * linear time invariant (LTI) dynamics systems.
 * @author   Gerry Chen
 * @author   Shuo Yang
 * @author   Yetong Zhang
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/inference/Symbol.h>

namespace ecLqr {

using gtsam::noiseModel::Constrained;
using gtsam::noiseModel::Gaussian;
using gtsam::noiseModel::Diagonal;
using gtsam::Symbol;

template <int N, int M>
GaussianFactorGraph GfgFromEcLqr(const EcLqrParams<N, M> &params) {
  GaussianFactorGraph graph;

  // noise models
  auto priorNm = Constrained::All(N);
  auto dynamicsNm = Constrained::All(N);
  // TODO: handle non-diagonal case
  auto qNm = Diagonal::Precisions(params.Q.diagonal());
  auto qfNm = Diagonal::Precisions(params.Qf.diagonal());
  auto rNm = Diagonal::Precisions(params.R.diagonal());

  // useful matrix constants
  auto INxN = Eigen::Matrix<double, N, N>::Identity();
  auto minusINxN = -Eigen::Matrix<double, N, N>::Identity();
  auto IMxM = Eigen::Matrix<double, M, M>::Identity();
  auto Zx = Eigen::Matrix<double, N, 1>::Zero();
  auto Zu = Eigen::Matrix<double, M, 1>::Zero();

  // initial condition
  graph.add(Symbol('x', 0), INxN, params.x0, priorNm);

  // loop
  for (size_t t = 0; t < params.T; ++t) {
    // dynamics: xdot = Ax + Bu
    graph.add(Symbol('x', t), params.A,    //
              Symbol('u', t), params.B,    //
              Symbol('x', t + 1), minusINxN,  //
              Zx, dynamicsNm);
    
    // objectives
    graph.add(Symbol('x', t), INxN, Zx, qNm);
    graph.add(Symbol('u', t), IMxM, Zu, rNm);
  }

  // state constraints
  for (const auto &[t, c] : params.xConstraints)
    graph.add(Symbol('x', t), c.G, -c.h, Constrained::All(c.G.rows()));
  for (const auto &[t, c] : params.xuConstraints)
    graph.add(Symbol('x', t), c.C, Symbol('u', t), c.D, -c.r,
              Constrained::All(c.C.rows()));

  // final objective
  graph.add(Symbol('x', params.T), INxN, params.xf, qfNm);

  return graph;
}

template <int N, int M>
GaussianFactorGraph GfgFromStateSpace(const Eigen::Matrix<double, N, N> &A,
                                      const Eigen::Matrix<double, N, M> &B,
                                      size_t T) {
  GaussianFactorGraph graph;

  auto noiseModel = Constrained::All(N);
  auto minusI = -Eigen::Matrix<double, N, N>::Identity();
  auto Z = Eigen::Matrix<double, N, 1>::Zero();
  for (size_t t = 0; t < T; ++t) {
    graph.add(Symbol('x', t), A,           //
              Symbol('u', t), B,           //
              Symbol('x', t + 1), minusI,  //
              Z, noiseModel);
  }

  // std::cout << "hello" << std::endl;
  return graph;
}

template <int N, int M>
GaussianFactorGraph GfgFromStateSpace(
    const std::vector<Eigen::Matrix<double, N, N>> &As,
    const std::vector<Eigen::Matrix<double, N, M>> &Bs, size_t T) {
  GaussianFactorGraph graph;

  auto noiseModel = Constrained::All(N);
  auto minusI = -Eigen::Matrix<double, N, N>::Identity();
  auto Z = Eigen::Matrix<double, N, 1>::Zero();
  for (size_t t = 0; t < T; ++t) {
    auto A = As[t];
    auto B = Bs[t];
    graph.add(Symbol('x', t), A,           //
              Symbol('u', t), B,           //
              Symbol('x', t + 1), minusI,  //
              Z, noiseModel);
  }

  return graph;
}

}  // namespace ecLqr
