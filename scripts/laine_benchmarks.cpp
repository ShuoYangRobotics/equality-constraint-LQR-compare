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
#include <src/EcLqr_laine.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/base/timing.h>

#include <chrono> // sanity check on GTSAM's timing functions

using namespace ecLqr;
using namespace gtsam;
using namespace std;
using namespace std::chrono;

// problem parameters
#define ITER 10
#define N 20
#define M 20
#define ncx 10
#define ncxu (M-1)
#define CONSTRAINED_PROPORTION 1
constexpr size_t T = 250;

/// creates a random noise matrix
template <int n>
Eigen::Matrix<double, n, n> getRandSpd() {
  auto tmpV = Eigen::Matrix<double, n, 1>::Random();
  auto tmpV2 = tmpV.cwiseAbs();
  return tmpV2.asDiagonal();
}

/// creates an EcLqrParams object corresponding to problem parameters with
/// randomized matrices
EcLqrParams<N, M> create_params() {
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
    EcLqrParams<N, M>::XUConstraint xuConstraint{C, D, r};
    params.xuConstraints.emplace(t, xuConstraint);
  }

  return params;
}

int main(int argc, char* argv[]) {
  auto start = high_resolution_clock::now();
  auto stop = high_resolution_clock::now();
  auto duration = duration_cast<microseconds>(stop - start);
  size_t fg_total_us = 0, laine_total_us = 0;

  for (int i = 0; i < ITER; ++i) {
    auto params = create_params();

    start = high_resolution_clock::now();
    gttic_(fg);
    auto graph = GfgFromParams(params);
    // auto result = fgSolFromParams(params);
    // auto gains = fgGainsFromGfg<N, M>(graph, params.T);
    auto bn = BnFromGfg(graph, params.T);
    auto gains = fgGainsFromBn<N, M>(bn, params.T);
    gttoc_(fg);
    stop = high_resolution_clock::now();
    duration = duration_cast<microseconds>(stop - start);
    fg_total_us += duration.count();

    start = high_resolution_clock::now();
    gttic_(laine);
    // auto results2 = laineSolFromParams(params);
    auto gains2 = laineGainsFromParams(params);
    gttoc_(laine);
    tictoc_finishedIteration_();
    stop = high_resolution_clock::now();
    duration = duration_cast<microseconds>(stop - start);
    laine_total_us += duration.count();

    assert_equal(laineSolFromGains(gains2, params), fgSolFromBn(bn));
  }

  cout << "GTSAM timing measurements (over " << ITER << " trials):" << endl;
  tictoc_print_();
  cout << endl;
  cout << "Manual timing measurements (sanity check):" << endl;
  cout << "fg time:    " << fg_total_us/1000.0/ITER << "ms per trial" << endl;
  cout << "laine time: " << laine_total_us/1000.0/ITER << "ms per trial" << endl;
}
