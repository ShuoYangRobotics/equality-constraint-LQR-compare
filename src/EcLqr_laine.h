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

template <int N, int M>
gtsam::VectorValues laineSolFromEcLqr(const EcLqrParams<N, M> &params);

}  // namespace ecLqr

#include "EcLqr_laine-impl.h"
