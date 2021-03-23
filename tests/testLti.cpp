/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testCholesky.cpp
 * @author  Richard Roberts
 * @date    Nov 5, 2010
 */

#include <src/Lti.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/base/Matrix.h>

#include <CppUnitLite/TestHarness.h>

using namespace ecLqr;
using namespace gtsam;
using namespace std;

/* ************************************************************************* */
TEST(Lti, constantABmatrices) {
  Matrix66 A = I_6x6;
  Matrix63 B = (Matrix63() << I_3x3, I_3x3).finished();
  size_t T = 10;
  auto graph = GfgFromStateSpace(A, B, T);
  
  EXPECT(assert_equal(T, graph.size()));

  size_t t = 0;
  for (const auto &factor : graph) {
    KeyVector expected_keys;
    expected_keys.push_back(Symbol('x', t));
    expected_keys.push_back(Symbol('u', t));
    expected_keys.push_back(Symbol('x', t+1));

    EXPECT(expected_keys == factor->keys());

    Matrix expected_jacobian = (Matrix(16, 6) << A.transpose(), B.transpose(),
                                -I_6x6, Matrix16::Zero())
                                   .finished()
                                   .transpose();
    EXPECT(assert_equal(expected_jacobian, factor->augmentedJacobian()));

    ++t;
  }
}

/* ************************************************************************* */
TEST(Lti, nonconstantABmatrices) {
  vector<Matrix66> As;
  vector<Matrix63> Bs;
  size_t T = 10;
  for (size_t t = 0; t < T; ++t) {
    As.push_back(I_6x6);
    Bs.push_back((Matrix63() << I_3x3, I_3x3).finished());
    As[t](0, 1) = t;
  }
  auto graph = GfgFromStateSpace(As, Bs, T);

  EXPECT(assert_equal(T, graph.size()));

  size_t t = 0;
  for (const auto &factor : graph) {
    KeyVector expected_keys;
    expected_keys.push_back(Symbol('x', t));
    expected_keys.push_back(Symbol('u', t));
    expected_keys.push_back(Symbol('x', t+1));

    EXPECT(expected_keys == factor->keys());

    Matrix expected_jacobian = (Matrix(16, 6) << As[t].transpose(),
                                Bs[t].transpose(), -I_6x6, Matrix16::Zero())
                                   .finished()
                                   .transpose();
    EXPECT(assert_equal(expected_jacobian, factor->augmentedJacobian()));

    ++t;
  }
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
