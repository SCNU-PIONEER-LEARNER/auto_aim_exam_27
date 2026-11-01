#include "rm_tools/iterated_extended_kalman_filter.hpp"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include <cmath>

TEST(IteratedExtendedKalmanFilter, MatchesLinearKalmanUpdate)
{
  Eigen::VectorXd x0(1);
  x0 << 0.0;
  Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(1, 1);
  rm_tools::IteratedExtendedKalmanFilter filter(x0, P0);

  Eigen::VectorXd z(1);
  z << 1.0;
  Eigen::MatrixXd R(1, 1);
  R << 1.0;

  auto h = [](const Eigen::VectorXd & x) {
    Eigen::VectorXd z_pred(1);
    z_pred << x[0];
    return z_pred;
  };
  auto calculate_H = [](const Eigen::VectorXd &) {
    Eigen::MatrixXd H(1, 1);
    H << 1.0;
    return H;
  };

  filter.update(z, R, h, calculate_H);

  EXPECT_NEAR(filter.X[0], 0.5, 1e-12);
  EXPECT_NEAR(filter.P(0, 0), 0.5, 1e-12);
}

TEST(IteratedExtendedKalmanFilter, RelinearizesDuringNonlinearUpdate)
{
  Eigen::VectorXd x0(1);
  x0 << 1.0;
  Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(1, 1);
  rm_tools::IteratedExtendedKalmanFilter filter(x0, P0);

  Eigen::VectorXd z(1);
  z << 4.0;
  Eigen::MatrixXd R(1, 1);
  R << 0.01;

  auto h = [](const Eigen::VectorXd & x) {
    Eigen::VectorXd z_pred(1);
    z_pred << x[0] * x[0];
    return z_pred;
  };

  int jacobian_calls = 0;
  auto calculate_H = [&jacobian_calls](const Eigen::VectorXd & x) {
    ++jacobian_calls;
    Eigen::MatrixXd H(1, 1);
    H << 2.0 * x[0];
    return H;
  };

  filter.update(z, R, h, calculate_H, 5, 0.0);

  EXPECT_EQ(jacobian_calls, 5);
  EXPECT_LT(std::abs(filter.X[0] * filter.X[0] - 4.0), 0.2);
  EXPECT_TRUE(std::isfinite(filter.P(0, 0)));
}
