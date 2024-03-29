/**
 * @file     simple_2d_system.cpp
 * @brief    Basic equality constrained LQR example with timing
 * @author   Gerry Chen
 * @author   Shuo Yang
 * @author   Yetong Zhang
 */

#include <src/EcLqr_fg.h>
#include <src/EcLqr_laine.h>
#include <tests/exampleProblems.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/base/timing.h>

using namespace ecLqr;
using namespace gtsam;
using namespace std;

int main(int argc, char* argv[]) {
  // setup
  EcLqrParams<2, 1> params = example::params_simple_2d_system();

  // solve
  {
    for (int i = 0; i < 1000; ++i) {
      gttic_(fg);
      auto graph = GfgFromParams(params);
      // auto gains = fgGainsFromGfg<2, 1>(graph, params.T);
      // auto result = fgSolFromGfg(graph);
      auto bn = BnFromGfg(graph, params.T);
      auto gains = fgGainsFromBn<2, 1>(bn, params.T);
      auto result = fgSolFromBn(bn);
      gttoc_(fg);
      gttic_(laine);
      auto gains2 = laineGainsFromParams(params);
      auto result2 = laineSolFromGains(gains2, params);
      gttoc_(laine);
    }
  }

  // debug print solution
  // for (size_t t = 0; t < T; ++t) {
  //   auto xy = result.at(symbol('x', t));
  //   cout << t << ":\t" << xy[0] << '\t' << xy[1] << endl;
  // }

  tictoc_finishedIteration_();
  tictoc_print_();
}
