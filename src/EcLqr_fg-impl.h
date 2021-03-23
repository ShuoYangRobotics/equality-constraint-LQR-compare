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
using gtsam::Symbol;

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
