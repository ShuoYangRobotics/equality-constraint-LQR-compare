/**
 * @file    testEcLqr_fg.cpp
 * @author  Gerry Chen
 * @author  Shuo Yang
 * @author  Yetong Zhang
 */

#include "exampleProblems.h"

#include <src/EcLqrParams.h>
#include <src/EcLqr_fg.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/linear/VectorValues.h>

#include <CppUnitLite/TestHarness.h>

#include <fstream>

using namespace ecLqr;
using namespace gtsam;
using namespace std;

/* ************************************************************************* */
TEST(EcLqr, lqr_factorgraph) {
  EcLqrParams<2, 1> params = example::params_simple_2d_system();

  auto graph = GfgFromEcLqr(params);

  auto result = graph.optimize();

  #include "data/simple_2d_system.h"
  for (size_t t = 0; t < params.T; ++t) {
    Vector2 expected_xy = (Vector2() << x[t], y[t]).finished();
    Vector1 expected_u = (Vector1() << u[t]).finished();
    EXPECT(assert_equal(expected_xy, result.at(Symbol('x', t))));
    EXPECT(assert_equal(expected_u, result.at(Symbol('u', t))));
  }
}

/* ************************************************************************* */
TEST(EcLqr, three_by_three) {
  EcLqrParams<3, 3> params =
      example::params_three_by_three_system_state_and_control();

  auto graph = GfgFromEcLqr(params);

  auto result = graph.optimize();

  #include "data/three_by_three_system_state_and_control.h"
  for (size_t t = 0; t < params.T; ++t) {
    Vector3 expected_xyz = (Vector3() << x[t], y[t], z[t]).finished();
    EXPECT(assert_equal(expected_xyz, result.at(Symbol('x', t))));
  }
}

/* ************************************************************************* */
TEST(EcLqr, constantABmatrices) {
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
TEST(EcLqr, nonconstantABmatrices) {
  vector<Matrix66> As;
  vector<Matrix63> Bs;
  size_t T = 10;
  for (size_t t = 0; t < T; ++t) {
    auto I = I_6x6;
    As.push_back(I);
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
