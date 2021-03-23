/**
 * @file     Lti.cpp
 * @brief    Useful functions for generating factor graphs which represent
 * linear time invariant (LTI) dynamics systems.
 * @author   Gerry Chen
 * @author   Shuo Yang
 * @author   Yetong Zhang
 */

#include "Lti.h"

#include <gtsam/base/Matrix.h>
#include <gtsam/linear/GaussianFactorGraph.h>

using namespace gtsam;
using namespace std;

namespace ecLqr {

GaussianFactorGraph GfgFromStateSpace(const Matrix &A, const Matrix &B,
                                             size_t T) {
  return GaussianFactorGraph();
}

GaussianFactorGraph GfgFromStateSpace(const std::vector<Matrix> &A,
                                             const std::vector<Matrix> &B,
                                             size_t T) {
  return GaussianFactorGraph();
}

}  // namespace ecLqr
