/**
 * @file     EcLqr_laine.h
 * @brief    Functions for solving equality constrained LQR using Laine19's
 * technique
 * @author   Gerry Chen
 * @author   Shuo Yang
 * @author   Yetong Zhang
 */

#pragma once

#include "EcLqrParams.h"

#include <gtsam/linear/VectorValues.h>

namespace ecLqr {

/**
 * Returns the open-loop control and state trajectories for the equality
 * constrained LQR problem described by params.
 * @param params The parameters of the ecLQR problem
 */
template <int N, int M>
gtsam::VectorValues laineSolFromParams(const EcLqrParams<N, M> &params);

}  // namespace ecLqr

#include "EcLqr_laine-impl.h"
