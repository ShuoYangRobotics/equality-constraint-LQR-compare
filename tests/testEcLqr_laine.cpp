/**
 * @file    testEcLqr_laine.cpp
 * @author  Gerry Chen
 * @author  Shuo Yang
 * @author  Yetong Zhang
 */

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
TEST(EcLqr, lqr_laine) {
  double dt = 0.01;
  double Tf = 1;
  size_t T = Tf / dt;

  EcLqrParams<2, 1> params;
  params.T = T;
  params.x0 = Z_2x1;
  params.xf = (Vector2() << 3, 2).finished();
  params.A = (Matrix22() << 1, dt, 0, 1).finished();
  params.B = (Vector2() << 0, dt).finished();
  params.Q = 1e-2 * I_2x2;
  params.R = 1e-3 * I_1x1;
  params.Qf = 500 * I_2x2;
  EcLqrParams<2, 1>::XConstraint xConstraint{
      I_2x2, -(Vector2() << 2, -2).finished()};
  params.xConstraints.emplace(T / 2 - 1, xConstraint);

  cout << setiosflags(ios::fixed) << setprecision(3);
  auto result = laineSolFromEcLqr(params);

  #include "simple_2d_system.h"
  for (size_t t = 0; t < T; ++t) {
    Vector2 expected_xy = (Vector2() << x[t], y[t]).finished();
    Vector1 expected_u = (Vector1() << u[t]).finished();
    EXPECT(assert_equal(expected_xy, result.at(Symbol('x', t))));
    EXPECT(assert_equal(expected_u, result.at(Symbol('u', t))));
  }

  cout << setiosflags(ios::fixed) << setprecision(3);
  for (size_t t = 0; t < T; ++t) {
    cout << x[t] << "\t" << y[t] << " : " << result.at(Symbol('x', t)).transpose() << "\t***\t";
    cout << u[t] << " : " << result.at(Symbol('u', t)) << endl;
  }
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
