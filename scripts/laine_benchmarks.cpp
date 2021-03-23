/**
 * @file     laine_benchmarks.cpp
 * @brief    Timing benchmarking example, with n, m, T, and % constrained
 * parameters to match Laine's Table 1 parameters.  Matrices are randomized.
 * @author   Gerry Chen
 * @author   Shuo Yang
 * @author   Yetong Zhang
 */

#include <src/EcLqrParams.h>
#include <src/EcLqr_fg.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/base/timing.h>

using namespace ecLqr;
using namespace gtsam;
using namespace std;

template <int N>
Eigen::Matrix<double, N, N> getRandSpd() {
  auto tmpV = Eigen::Matrix<double, N, 1>::Random();
  auto tmpV2 = tmpV.cwiseAbs();
  return tmpV2.asDiagonal();
}

int main(int argc, char* argv[]) {
  #define N 40
  #define M 10
  #define ncx 20
  #define ncxu 20
  #define CONSTRAINED_PROPORTION 0.9
  constexpr size_t T = 250;

  for (int i = 0; i < 10; ++i) {
    gttic_(defineProblem);
    EcLqrParams<N, M> params;
    params.T = T;
    params.x0 = Eigen::Matrix<double, N, 1>::Random();
    params.xf = Eigen::Matrix<double, N, 1>::Random();
    params.A = Eigen::Matrix<double, N, N>::Random();
    params.B = Eigen::Matrix<double, N, M>::Random();
    params.Q = getRandSpd<N>();
    params.R = getRandSpd<M>();
    params.Qf = getRandSpd<N>();

    for (size_t t = 0; t < T; ++t) {
      if ((t % 10) >= (10*CONSTRAINED_PROPORTION))
        continue;
      auto C = Eigen::Matrix<double, ncxu, N>::Random();
      auto D = Eigen::Matrix<double, ncxu, M>::Random();
      auto r = Eigen::Matrix<double, ncxu, 1>::Random();
      EcLqrParams<N, M>::XUConstraint xuConstraint{C, D, r, t};
      params.xuConstraints.push_back(xuConstraint);
    }
    gttoc_(defineProblem);

    gttic_(createGraph);
    auto graph = GfgFromEcLqr(params);
    gttoc_(createGraph);
    gttic_(optimize);
    auto result = graph.optimize();
    gttoc_(optimize);
  }

  tictoc_finishedIteration_();
  tictoc_print_();
}
