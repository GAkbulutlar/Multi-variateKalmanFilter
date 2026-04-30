/// main.cpp
/// Main simulation loop for the 4D Multivariate Kalman Filter.
/// Simulates a 2D object moving with constant velocity, with noisy measurements.

#include "KalmanFilter.h"
#include "Utilities.h"
#include <iostream>
#include <random>
#include <iomanip>

using Eigen::MatrixXd;
using Eigen::VectorXd;

int main() {
    try {
        std::cout << "========== 4D Multivariate Kalman Filter Simulation ==========\n\n";

        // --- Model Parameters ---
        // State: [x, y, vx, vy]
        // Measurement: [x, y] (position only; velocity is not directly measured)
        
        constexpr double dt = 0.1;  // Time step [seconds].
        constexpr int numSteps = 200;  // Total simulation time steps.

        // State transition matrix (constant velocity model).
        // x_k = x_{k-1} + vx * dt
        // y_k = y_{k-1} + vy * dt
        // vx_k = vx_{k-1}
        // vy_k = vy_{k-1}
        MatrixXd F = MatrixXd::Identity(4, 4);
        F(0, 2) = dt;  // x += vx * dt
        F(1, 3) = dt;  // y += vy * dt

        // Measurement matrix (position-only observation).
        // z_k = [x_k, y_k]
        MatrixXd H = MatrixXd::Zero(2, 4);
        H(0, 0) = 1.0;  // Measure x.
        H(1, 1) = 1.0;  // Measure y.

        // Process noise covariance (uncertainty in velocity).
        MatrixXd Q = MatrixXd::Identity(4, 4) * 0.01;
        Q(0, 0) = 0.002;  // Small noise in x position.
        Q(1, 1) = 0.002;  // Small noise in y position.
        Q(2, 2) = 0.02;   // Higher noise in vx (acceleration uncertainty).
        Q(3, 3) = 0.02;   // Higher noise in vy.

        // Measurement noise covariance (sensor uncertainty).
        MatrixXd R = MatrixXd::Identity(2, 2) * 0.25;

        // Initial state estimate: [0, 0, 0, 0].
        VectorXd x0 = VectorXd::Zero(4);

        // Initial estimate covariance (high uncertainty).
        MatrixXd P0 = MatrixXd::Identity(4, 4) * 10.0;

        // Construct the Kalman filter.
        KalmanFilter kf(F, H, Q, R, x0, P0);

        std::cout << "System Configuration:\n"
                  << "  State dimension: 4 (x, y, vx, vy)\n"
                  << "  Measurement dimension: 2 (x, y)\n"
                  << "  Time step: " << dt << " s\n"
                  << "  Total steps: " << numSteps << "\n"
                  << "  Simulation duration: " << (numSteps * dt) << " s\n\n";

        // --- Ground Truth and Measurement Trajectories ---
        // Generate synthetic trajectory with smooth motion and random perturbations.
        std::mt19937 rng(42);  // Deterministic seed for repeatability.
        std::normal_distribution<double> processNoise(0.0, std::sqrt(0.01));
        std::normal_distribution<double> measurementNoise(0.0, std::sqrt(0.25));

        std::vector<VectorXd> truthStates;      // Ground truth.
        std::vector<KalmanFilter::SimulationState> results;  // Kalman filter results.

        VectorXd trueState = VectorXd::Zero(4);
        trueState(2) = 0.5;   // Initial velocity Vx = 0.5 m/s.
        trueState(3) = 0.3;   // Initial velocity Vy = 0.3 m/s.

        std::cout << std::setw(5) << "Step"
                  << std::setw(12) << "Truth X"
                  << std::setw(12) << "Truth Y"
                  << std::setw(12) << "Meas X"
                  << std::setw(12) << "Meas Y"
                  << std::setw(12) << "Est X"
                  << std::setw(12) << "Est Y"
                  << std::setw(12) << "Est Vx"
                  << std::setw(12) << "Est Vy\n";
        std::cout << std::string(100, '-') << "\n";

        for (int step = 0; step < numSteps; ++step) {
            // Add small perturbations to velocity (realistic motion).
            if (step % 50 == 0 && step > 0) {
                trueState(2) += 0.1;  // Accelerate Vx periodically.
            }

            // Evolve ground truth with process noise.
            VectorXd nextState = F * trueState;
            for (int i = 0; i < 4; ++i) {
                nextState(i) += processNoise(rng);
            }
            trueState = nextState;

            // Generate noisy measurement (position only).
            VectorXd measurement = H * trueState;
            for (int i = 0; i < 2; ++i) {
                measurement(i) += measurementNoise(rng);
            }

            // Kalman filter predict step.
            kf.Predict();

            // Kalman filter update step.
            kf.Update(measurement);

            // Store results.
            VectorXd estimate = kf.GetState();
            truthStates.push_back(trueState);

            const double dxMeas = trueState(0) - measurement(0);
            const double dyMeas = trueState(1) - measurement(1);
            const double dxKf = trueState(0) - estimate(0);
            const double dyKf = trueState(1) - estimate(1);

            const double measErrorSq = dxMeas * dxMeas + dyMeas * dyMeas;
            const double kfErrorSq = dxKf * dxKf + dyKf * dyKf;

            KalmanFilter::SimulationState simState;
            simState.step = step;
            simState.truth = trueState;
            simState.measurement = measurement;
            simState.estimate = estimate;
            simState.covariance = kf.GetCovariance();
            simState.measErrorSq = measErrorSq;
            simState.kfErrorSq = kfErrorSq;
            results.push_back(simState);

            // Print progress every 20 steps.
            if (step % 20 == 0 || step == numSteps - 1) {
                std::cout << std::setw(5) << step
                          << std::setw(12) << std::fixed << std::setprecision(3) << trueState(0)
                          << std::setw(12) << std::fixed << std::setprecision(3) << trueState(1)
                          << std::setw(12) << std::fixed << std::setprecision(3) << measurement(0)
                          << std::setw(12) << std::fixed << std::setprecision(3) << measurement(1)
                          << std::setw(12) << std::fixed << std::setprecision(3) << estimate(0)
                          << std::setw(12) << std::fixed << std::setprecision(3) << estimate(1)
                          << std::setw(12) << std::fixed << std::setprecision(3) << estimate(2)
                          << std::setw(12) << std::fixed << std::setprecision(3) << estimate(3) << "\n";
            }
        }

        std::cout << "\n";

        // --- Performance Metrics ---
        double sumMeasErrSq = 0.0;
        double sumKfErrSq = 0.0;
        for (const auto& sample : results) {
            sumMeasErrSq += sample.measErrorSq;
            sumKfErrSq += sample.kfErrorSq;
        }

        const double measRMSE = std::sqrt(sumMeasErrSq / static_cast<double>(results.size()));
        const double posRMSE = std::sqrt(sumKfErrSq / static_cast<double>(results.size()));
        std::cout << "Position RMSE (measurement vs truth): " << std::fixed << std::setprecision(4) << measRMSE << " units\n";
        std::cout << "Position RMSE (estimate vs truth): " << std::fixed << std::setprecision(4) << posRMSE << " units\n";

        // Compute velocity RMSE (only for estimate vs truth, since velocity is not measured).
        double velRMSE = 0.0;
        for (std::size_t i = 0; i < truthStates.size(); ++i) {
            double vxErr = results[i].estimate(2) - truthStates[i](2);
            double vyErr = results[i].estimate(3) - truthStates[i](3);
            velRMSE += vxErr * vxErr + vyErr * vyErr;
        }
        velRMSE = std::sqrt(velRMSE / (2.0 * truthStates.size()));
        std::cout << "Velocity RMSE (estimate vs truth): " << std::fixed << std::setprecision(4) << velRMSE << " units/s\n";

        // --- Export Results ---
        std::cout << "\n--- Exporting Results ---\n";
        utils::ExportToCSV("simulation_results.csv", results);
        utils::ExportPositionPlotSVG("position_plot.svg", results);
        utils::ExportVelocityPlotSVG("velocity_plot.svg", results);

        std::cout << "\nSimulation completed successfully!\n";
        std::cout << "Output files:\n"
                  << "  - simulation_results.csv (raw data)\n"
                  << "  - position_plot.svg (X-Y position trajectory)\n"
                  << "  - velocity_plot.svg (velocity components vs time)\n";

        return 0;

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}
