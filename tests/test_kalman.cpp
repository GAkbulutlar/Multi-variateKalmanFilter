#include <gtest/gtest.h>
#include <Eigen/Dense>

#include "KalmanFilter.h"

TEST(KalmanFilterStateTransitionTest, PredictUpdatesPositionWithConstantVelocity)
{
    constexpr double dt = 0.1;

    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(4, 4);
    F(0, 2) = dt;
    F(1, 3) = dt;

    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, 4);
    H(0, 0) = 1.0;
    H(1, 1) = 1.0;

    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(4, 4);
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(2, 2);

    Eigen::VectorXd x0(4);
    x0 << 1.0, -2.0, 0.5, -0.25;

    Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(4, 4);

    KalmanFilter filter(F, H, Q, R, x0, P0);
    filter.Predict();

    const Eigen::VectorXd predicted = filter.GetState();

    EXPECT_NEAR(predicted(0), x0(0) + x0(2) * dt, 1e-12);
    EXPECT_NEAR(predicted(1), x0(1) + x0(3) * dt, 1e-12);
    EXPECT_NEAR(predicted(2), x0(2), 1e-12);
    EXPECT_NEAR(predicted(3), x0(3), 1e-12);
}
