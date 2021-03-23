/**
 * @file     simple_2d_system.cpp
 * @brief    Basic equality constrained LQR example with timing
 * @author   Gerry Chen
 * @author   Shuo Yang
 * @author   Yetong Zhang
 */

#include <src/EcLqr_fg.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/base/timing.h>

using namespace ecLqr;
using namespace gtsam;
using namespace std;

int main(int argc, char* argv[]) {
  double dt = 0.01;
  double Tf = 1;
  size_t T = Tf / dt;

  // setup
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
      I_2x2, -(Vector2() << 2, -2).finished(), T / 2};
  params.xConstraints.emplace_back(xConstraint);

  // solve
  {
    for (int i = 0; i < 1000; ++i) {
      gttic_(createGraph);
      auto graph = GfgFromEcLqr(params);
      gttoc_(createGraph);
      gttic_(optimize);
      auto result = graph.optimize();
    }
  }

  // for (size_t t = 0; t < T; ++t) {
  //   auto xy = result.at(symbol('x', t));
  //   cout << t << ":\t" << xy[0] << '\t' << xy[1] << endl;
  // }

  tictoc_finishedIteration_();
  tictoc_print_();
}
