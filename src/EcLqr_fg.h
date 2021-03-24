/**
 * @file     EcLqr_fg.h
 * @brief    Useful functions for generating factor graphs which represent
 * linear time invariant (LTI) dynamics systems.
 * @author   Gerry Chen
 * @author   Shuo Yang
 * @author   Yetong Zhang
 */

#pragma once

#include "EcLqrParams.h"

#include <gtsam/linear/GaussianFactorGraph.h>

namespace ecLqr {

using gtsam::GaussianFactorGraph;
using gtsam::VectorValues;

/**
 * Creates a (linear) factor graph for the equality constrained LQR problem
 * described by params.
 * @param params The parameters of the ecLQR problem
 */
template <int N, int M>
GaussianFactorGraph GfgFromParams(const EcLqrParams<N, M> &params);

/**
 * Returns the open-loop control and state trajectories for a factor graph.
 * @param graph The factor graph
 */
VectorValues fgSolFromGfg(const GaussianFactorGraph &graph);

}  // namespace ecLqr

#include "EcLqr_fg-impl.h"
