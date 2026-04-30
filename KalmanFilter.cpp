/// KalmanFilter.cpp
/// Implementation of the Multivariate Kalman Filter class.

#include "KalmanFilter.h"
#include <sstream>
#include <cmath>

KalmanFilter::KalmanFilter(
    const MatrixXd& F,
    const MatrixXd& H,
    const MatrixXd& Q,
    const MatrixXd& R,
    const VectorXd& x0,
    const MatrixXd& P0
)
    : F_(F), H_(H), Q_(Q), R_(R), state_(x0), covariance_(P0)
{
    // Validate dimensions.
    int n = F_.rows();  // State dimension.
    int m = H_.rows();  // Measurement dimension.

    if (F_.cols() != n) {
        throw std::invalid_argument("F must be square (state transition matrix)");
    }
    if (H_.cols() != n) {
        throw std::invalid_argument("H must have " + std::to_string(n) + " columns (measurement matrix)");
    }
    if (Q_.rows() != n || Q_.cols() != n) {
        throw std::invalid_argument("Q must be " + std::to_string(n) + "x" + std::to_string(n) + " (process noise)");
    }
    if (R_.rows() != m || R_.cols() != m) {
        throw std::invalid_argument("R must be " + std::to_string(m) + "x" + std::to_string(m) + " (measurement noise)");
    }
    if (x0.size() != n) {
        throw std::invalid_argument("x0 must have " + std::to_string(n) + " elements (initial state)");
    }
    if (P0.rows() != n || P0.cols() != n) {
        throw std::invalid_argument("P0 must be " + std::to_string(n) + "x" + std::to_string(n) + " (initial covariance)");
    }
}

void KalmanFilter::Predict(const VectorXd& u)
{
    int n = state_.size();  // State dimension.
    VectorXd control = u;   // Use provided control or zero vector.

    if (control.size() == 0) {
        control = VectorXd::Zero(n);
    }

    if (control.size() != n) {
        throw std::invalid_argument("Control vector dimension mismatch");
    }

    // Predict state: x_{k|k-1} = F * x_{k-1|k-1}
    // (Control input is absorbed into state transition in this model.)
    state_ = F_ * state_;

    // Predict covariance: P_{k|k-1} = F * P_{k-1|k-1} * F^T + Q
    covariance_ = F_ * covariance_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd& z)
{
    int m = z.size();  // Measurement dimension.

    if (m != H_.rows()) {
        throw std::invalid_argument("Measurement dimension mismatch");
    }

    // Innovation / residual: y = z - H * x_{k|k-1}
    VectorXd innovation = z - H_ * state_;

    // Innovation covariance: S = H * P_{k|k-1} * H^T + R
    MatrixXd innovationCovariance = H_ * covariance_ * H_.transpose() + R_;

    // Kalman gain: K = P_{k|k-1} * H^T * S^{-1}
    MatrixXd kalmanGain = covariance_ * H_.transpose() * innovationCovariance.inverse();

    // Update state: x_{k|k} = x_{k|k-1} + K * y
    state_ = state_ + kalmanGain * innovation;

    // Update covariance: P_{k|k} = (I - K * H) * P_{k|k-1}
    int n = state_.size();
    MatrixXd identity = MatrixXd::Identity(n, n);
    covariance_ = (identity - kalmanGain * H_) * covariance_;
}
