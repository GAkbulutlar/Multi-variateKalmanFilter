/// KalmanFilter.h
/// A professional-grade Multivariate Kalman Filter using Eigen 3.
/// Supports arbitrary state dimension; specialized for 4D state (x, y, vx, vy).

#pragma once

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <stdexcept>

/// \class KalmanFilter
/// \brief Implements the Kalman Filter algorithm for linear systems.
/// 
/// State transition: x_{k|k-1} = F * x_{k-1|k-1} + w_k
/// Measurement:     z_k = H * x_k + v_k
/// where w_k ~ N(0, Q) is process noise and v_k ~ N(0, R) is measurement noise.
class KalmanFilter {
public:
    using MatrixXd = Eigen::MatrixXd;  ///< Dynamic Eigen matrix of doubles.
    using VectorXd = Eigen::VectorXd;  ///< Dynamic Eigen vector of doubles.

    /// \struct SimulationState
    /// \brief Encapsulates one time-step sample with ground truth, measurement, and estimate.
    struct SimulationState {
        int step;                  ///< Time index k.
        VectorXd truth;            ///< Ground-truth state vector.
        VectorXd measurement;      ///< Noisy sensor measurement.
        VectorXd estimate;         ///< Kalman posterior estimate.
        MatrixXd covariance;       ///< Posterior covariance matrix P.
    };

    /// Constructor: Initialize the Kalman filter with system parameters.
    /// \param F     State transition matrix (n x n).
    /// \param H     Measurement matrix (m x n): maps n-D state to m-D measurements.
    /// \param Q     Process noise covariance (n x n).
    /// \param R     Measurement noise covariance (m x m).
    /// \param x0    Initial state estimate (n-D vector).
    /// \param P0    Initial estimate covariance (n x n matrix).
    /// \throws std::invalid_argument if dimensions are inconsistent.
    KalmanFilter(
        const MatrixXd& F,
        const MatrixXd& H,
        const MatrixXd& Q,
        const MatrixXd& R,
        const VectorXd& x0,
        const MatrixXd& P0
    );

    /// Destructor
    ~KalmanFilter() = default;

    /// Predict the next state estimate given a control input.
    /// Implements: x_{k|k-1} = F * x_{k-1|k-1}
    ///             P_{k|k-1} = F * P_{k-1|k-1} * F^T + Q
    /// \param u     Control input vector (default: zero vector).
    void Predict(const VectorXd& u = VectorXd());

    /// Update the state estimate with a new measurement.
    /// Implements: innovation y = z - H * x_{k|k-1}
    ///             S = H * P_{k|k-1} * H^T + R (innovation covariance)
    ///             K = P_{k|k-1} * H^T * S^{-1} (Kalman gain)
    ///             x_{k|k} = x_{k|k-1} + K * y
    ///             P_{k|k} = (I - K * H) * P_{k|k-1}
    /// \param z     Measurement vector (m-D).
    /// \throws std::invalid_argument if measurement dimension is incorrect.
    void Update(const VectorXd& z);

    /// Get the current state estimate.
    /// \return The state vector x.
    [[nodiscard]] VectorXd GetState() const { return state_; }

    /// Get the current estimate covariance.
    /// \return The covariance matrix P.
    [[nodiscard]] MatrixXd GetCovariance() const { return covariance_; }

    /// Get the state dimension.
    /// \return Number of state variables.
    [[nodiscard]] int GetStateDimension() const { return static_cast<int>(state_.size()); }

    /// Get the measurement dimension.
    /// \return Number of measurement variables.
    [[nodiscard]] int GetMeasurementDimension() const { return static_cast<int>(H_.rows()); }

private:
    // System model matrices.
    MatrixXd F_;  ///< State transition matrix.
    MatrixXd H_;  ///< Measurement matrix.
    MatrixXd Q_;  ///< Process noise covariance.
    MatrixXd R_;  ///< Measurement noise covariance.

    // State and covariance.
    VectorXd state_;    ///< Current state estimate x.
    MatrixXd covariance_;  ///< Current estimate covariance P.
};
