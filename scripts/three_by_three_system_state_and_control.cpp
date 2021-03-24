/**
 * @file     three_by_three_system_and_state_control.cpp
 * @brief    Init LQR problem, this is the simple example given by Laine 2019
 * section 3.B
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

#include <chrono>

using namespace ecLqr;
using namespace gtsam;
using namespace std;
using namespace std::chrono;

#define N 3
#define M 3
#define ITER 100

int main(int argc, char* argv[]) {
  // timing setup
  auto start = high_resolution_clock::now();
  auto stop = high_resolution_clock::now();
  auto duration = duration_cast<microseconds>(stop - start);
  size_t fg_total_us = 0, laine_total_us = 0;

  // many trials
  auto params = example::params_three_by_three_system_state_and_control();
  for (int i = 0; i < ITER; ++i) {
    start = high_resolution_clock::now();
    gttic_(fg_total);
    gttic_(fg_createGraph);
    auto graph = GfgFromEcLqr(params);
    gttoc_(fg_createGraph);
    gttic_(fg_optimize);
    auto result = graph.optimize();
    gttoc_(fg_optimize);
    gttoc_(fg_total);
    stop = high_resolution_clock::now();
    duration = duration_cast<microseconds>(stop - start);
    fg_total_us += duration.count();

    start = high_resolution_clock::now();
    gttic_(laine_solve);
    auto result2 = laineSolFromEcLqr(params);
    gttoc_(laine_solve);
    tictoc_finishedIteration_();
    stop = high_resolution_clock::now();
    duration = duration_cast<microseconds>(stop - start);
    laine_total_us += duration.count();

    assert_equal(result, result2);
  }

  cout << "GTSAM timing measurements (over " << ITER << " trials):" << endl;
  tictoc_print_();
  cout << endl;
  cout << "Manual timing measurements (sanity check):" << endl;
  cout << "fg time:    " << fg_total_us/1000.0/ITER << "ms per trial" << endl;
  cout << "laine time: " << laine_total_us/1000.0/ITER << "ms per trial" << endl;
}
