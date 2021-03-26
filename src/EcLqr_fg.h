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
#include <gtsam/linear/GaussianBayesNet.h>

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
 * Performs sequential elimination on the factor graph in a backwards
 * elimination ordering
 * @param graph The factor graph
 * @param T     The number of time steps (from params.T)
 * @return  The Bayes Net
 */
gtsam::GaussianBayesNet::shared_ptr BnFromGfg(const GaussianFactorGraph &graph,
                                              size_t T);

/**
 * Returns the open-loop control and state trajectories for a factor graph. Note
 * that this does another full elimination on the factor graph, which is
 * wasteful.  Use the Bayes Net (fgSolFromBn) version instead, if you have
 * already calculated the gains.
 * @param graph The factor graph
 */
inline VectorValues fgSolFromGfg(const GaussianFactorGraph &graph);

/**
 * Returns the open-loop control and state trajectories for a Bayes Net.
 * @param net The Bayes Net
 */
VectorValues fgSolFromBn(const gtsam::GaussianBayesNet::shared_ptr &net);

/**
 * Returns the gain matrices for a factor graph.
 * @param graph The factor graph
 * @param T     The number of time steps (from params.T)
 * @return  The K matrices (MxN) and k vectors (Mx1) stored in std::vectors of
 * length T, and the two vectors are stored in a std::pair.
 */
template <int N, int M>
Gains<N, M> fgGainsFromGfg(const GaussianFactorGraph &graph, size_t T);

/**
 * Returns the gain matrices for a Bayes Net.
 * @param net The Bayes Net
 * @param T     The number of time steps (from params.T)
 * @return  The K matrices (MxN) and k vectors (Mx1) stored in std::vectors of
 * length T, and the two vectors are stored in a std::pair.
 */
template <int N, int M>
Gains<N, M> fgGainsFromBn(const gtsam::GaussianBayesNet::shared_ptr &net,
                          size_t T);

}  // namespace ecLqr

#include "EcLqr_fg-impl.h"
