/**
 * @file    dataloader.h
 * @brief   Sets up the same parameters as some example problems from matlab
 * @author  Gerry Chen
 * @author  Shuo Yang
 * @author  Yetong Zhang
 */

#include <src/EcLqrParams.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/timing.h>

namespace example {

using namespace gtsam;  // namespace pollution - whatever

/// creates an EcLqrParams object corresponding to the
/// simple_2d_system example
ecLqr::EcLqrParams<2, 1> params_simple_2d_system() {
  double dt = 0.01;
  double Tf = 1;
  size_t T = Tf / dt;

  ecLqr::EcLqrParams<2, 1> params;
  params.T = T;
  params.x0 = Z_2x1;
  params.xf = (Vector2() << 3, 2).finished();
  params.A = (Matrix22() << 1, dt, 0, 1).finished();
  params.B = (Vector2() << 0, dt).finished();
  params.Q = 1e-2 * I_2x2;
  params.R = 1e-3 * I_1x1;
  params.Qf = 500 * I_2x2;
  ecLqr::EcLqrParams<2, 1>::XConstraint xConstraint{I_2x2,
                                             -(Vector2() << 2, -2).finished()};
  params.xConstraints.emplace(T / 2 - 1, xConstraint);

  return params;
}

/// creates an EcLqrParams object corresponding to the
/// three_by_three_system_state_and_control example
ecLqr::EcLqrParams<3, 3> params_three_by_three_system_state_and_control() {
  gttic_(defineProblem);
  ecLqr::EcLqrParams<3, 3> params;
  params.T = 100;
  params.x0 = Eigen::Matrix<double, 3, 1>::Zero();
  params.xf = (Eigen::Matrix<double, 3, 1>() << 3, 2, 1).finished();
  params.A = Eigen::Matrix<double, 3, 3>::Identity() +
             0.01 * (Eigen::Matrix<double, 3, 3>() <<  //
                     -0.4762, 0.0576, -0.8775,  //
                     -0.1532, -0.9880, 0.0183,  //
                     -0.8659, 0.1432, 0.4793)
                        .finished();
  params.B = 0.01 * (Eigen::Matrix<double, 3, 3>() <<  //
                     -0.6294, -0.4978, -0.5967, //
                     -0.3749, -0.4781, 0.7943,  //
                     -0.6807, 0.7236, 0.1143)
                        .finished();
  params.Q = 1e-2 * Eigen::Matrix<double, 3, 3>::Identity();
  params.R = 1e-3 * Eigen::Matrix<double, 3, 3>::Identity();
  params.Qf = 500 * Eigen::Matrix<double, 3, 3>::Identity();

  {
    auto C = Eigen::Matrix<double, 3, 3>::Identity();
    auto D = Eigen::Matrix<double, 3, 3>::Identity();
    auto r = (Eigen::Matrix<double, 3, 1>() << 10, 20, 30).finished();
    ecLqr::EcLqrParams<3, 3>::XUConstraint xuConstraint{C, D, r};
    params.xuConstraints.emplace(params.T / 2 - 1, xuConstraint);
  }

  return params;
}

}  // namespace example
