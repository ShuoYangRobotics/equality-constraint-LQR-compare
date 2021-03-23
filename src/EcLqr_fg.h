/**
 * @file     Lti.h
 * @brief    Useful functions for generating factor graphs which represent
 * linear time invariant (LTI) dynamics systems.
 * @author   Gerry Chen
 * @author   Shuo Yang
 * @author   Yetong Zhang
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/linear/GaussianFactorGraph.h>

namespace ecLqr {

using gtsam::GaussianFactorGraph;

/**
 *  Creates a Gaussian Factor Graph (gfg) that represents a finite horizon
 * rollout of the LTI system given by xdot = A.x + B.u, where A is an nxn matrix
 * and B is an nxm matrix.
 * @param A The state transition matrix (F_x in the paper)
 * @param B The input matrix (F_u in the paper)
 * @param T The number of timesteps to use.  This will simulate x for timesteps
 * 0...T and u for timesteps 0...T-1
 */
template <int N, int M>
GaussianFactorGraph GfgFromStateSpace(
    const Eigen::Matrix<double, N, N> &A, const Eigen::Matrix<double, N, M> &B,
    size_t T);

/**
 *  Creates a Gaussian Factor Graph (gfg) that represents a finite horizon
 * rollout of the LTI system given by xdot = A(t).x + B(t).u where A is a vector
 * of nxn matrices of length T and B is a vector of nxm matrices of length
 * T.
 * @param A The state transition matrices (F_x_t in the paper)
 * @param B The input matrices (F_u_t in the paper)
 * @param T The number of timesteps to use.  This will simulate x for timesteps
 * 0...T and u for timesteps 0...T-1
 */
template <int N, int M>
GaussianFactorGraph GfgFromStateSpace(
    const std::vector<Eigen::Matrix<double, N, N>> &A,
    const std::vector<Eigen::Matrix<double, N, M>> &B, size_t T);

}  // namespace ecLqr

#include "EcLqr_fg-impl.h"
