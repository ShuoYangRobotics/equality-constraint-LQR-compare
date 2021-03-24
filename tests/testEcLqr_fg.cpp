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

  auto graph = GfgFromParams(params);
  auto result = fgSolFromGfg(graph);
  auto gains = fgGainsFromGfg<2, 1>(graph, params.T);

  #include "data/simple_2d_system.h"
  for (size_t t = 0; t < params.T; ++t) {
    Vector2 expected_xy = (Vector2() << x[t], y[t]).finished();
    Vector1 expected_u = (Vector1() << u[t]).finished();
    EXPECT(assert_equal(expected_xy, result.at(Symbol('x', t))));
    EXPECT(assert_equal(expected_u, result.at(Symbol('u', t))));
    Vector1 expected_u_2 = -gains.first[t] * expected_xy + gains.second[t];
    EXPECT(assert_equal(expected_u_2, result.at(Symbol('u', t))));
  }
}

/* ************************************************************************* */
TEST(EcLqr, three_by_three) {
  EcLqrParams<3, 3> params =
      example::params_three_by_three_system_state_and_control();

  auto graph = GfgFromParams(params);
  auto result = fgSolFromGfg(graph);
  auto gains = fgGainsFromGfg<3, 3>(graph, params.T);

  #include "data/three_by_three_system_state_and_control.h"
  for (size_t t = 0; t < params.T; ++t) {
    Vector3 expected_xyz = (Vector3() << x[t], y[t], z[t]).finished();
    EXPECT(assert_equal(expected_xyz, result.at(Symbol('x', t))));
    Vector3 expected_u_2 = -gains.first[t] * expected_xyz + gains.second[t];
    EXPECT(assert_equal(expected_u_2, result.at(Symbol('u', t))));
  }
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
