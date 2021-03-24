/**
 * @file    testEcLqr_laine.cpp
 * @author  Gerry Chen
 * @author  Shuo Yang
 * @author  Yetong Zhang
 */

#include "exampleProblems.h"

#include <src/EcLqrParams.h>
#include <src/EcLqr_laine.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/linear/VectorValues.h>

#include <CppUnitLite/TestHarness.h>

#include <iomanip>

using namespace ecLqr;
using namespace gtsam;
using namespace std;

/* ************************************************************************* */
TEST(EcLqr, simple_2d_system) {
  EcLqrParams<2, 1> params = example::params_simple_2d_system();

  cout << setiosflags(ios::fixed) << setprecision(3);
  auto gains = laineGainsFromParams(params);
  auto result = laineSolFromGains(gains, params);
  
  // self consistency check
  auto result2 = laineSolFromParams(params);
  EXPECT(assert_equal(result, result2));

  // ground truth check
  #include "data/simple_2d_system.h"
  for (size_t t = 0; t < params.T; ++t) {
    Vector2 expected_xy = (Vector2() << x[t], y[t]).finished();
    Vector1 expected_u = (Vector1() << u[t]).finished();
    EXPECT(assert_equal(expected_xy, result.at(Symbol('x', t))));
    EXPECT(assert_equal(expected_u, result.at(Symbol('u', t))));
  }

  // debug
  for (size_t t = 0; t < params.T; ++t) {
    auto xy = result.at(Symbol('x', t));
    auto u = result.at(Symbol('u', t));
    cout << x[t] << " : " << xy(0) << '\t'
         << y[t] << " : " << xy(1) << '\t'
         << u[t] << " : " << u(0) << endl;
  }
}

/* ************************************************************************* */
TEST(EcLqr, three_by_three) {
  EcLqrParams<3, 3> params =
      example::params_three_by_three_system_state_and_control();

  cout << setiosflags(ios::fixed) << setprecision(3);
  auto gains = laineGainsFromParams(params);
  auto result = laineSolFromGains(gains, params);

  // self consistency check
  auto result2 = laineSolFromParams(params);
  EXPECT(assert_equal(result, result2));

  // ground truth check
  #include "data/three_by_three_system_state_and_control.h"
  for (size_t t = 0; t < params.T; ++t) {
    Vector3 expected_xyz = (Vector3() << x[t], y[t], z[t]).finished();
    EXPECT(assert_equal(expected_xyz, result.at(Symbol('x', t))));
  }

  // debug
  for (size_t t = 0; t < params.T; ++t) {
    auto xyz = result.at(Symbol('x', t));
    cout << x[t] << " : " << xyz(0) << '\t'
         << y[t] << " : " << xyz(1) << '\t'
         << z[t] << " : " << xyz(2) << endl;
  }
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
