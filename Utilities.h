/// Utilities.h
/// Helper functions for RMSE calculation, CSV export, and SVG visualization.

#pragma once

#include "KalmanFilter.h"
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>

namespace utils {

/// Calculate Root Mean Squared Error between two signal vectors (element-wise).
/// \param reference  Reference (ground truth) signal.
/// \param estimate   Estimated signal.
/// \return RMSE value; NaN if vectors have different lengths or are empty.
inline double ComputeRMSE(
    const std::vector<Eigen::VectorXd>& reference,
    const std::vector<Eigen::VectorXd>& estimate
) {
    if (reference.empty() || estimate.empty() || reference.size() != estimate.size()) {
        return std::numeric_limits<double>::quiet_NaN();
    }

    double sumSquaredError = 0.0;
    int totalElements = 0;

    for (std::size_t i = 0; i < reference.size(); ++i) {
        const Eigen::VectorXd& ref = reference[i];
        const Eigen::VectorXd& est = estimate[i];

        if (ref.size() != est.size()) {
            return std::numeric_limits<double>::quiet_NaN();
        }

        for (int j = 0; j < ref.size(); ++j) {
            double error = est(j) - ref(j);
            sumSquaredError += error * error;
            totalElements++;
        }
    }

    return std::sqrt(sumSquaredError / static_cast<double>(totalElements));
}

/// Export simulation results to a CSV file.
/// \param filePath   Path where CSV will be written.
/// \param samples    Vector of SimulationState samples.
void ExportToCSV(
    const std::string& filePath,
    const std::vector<KalmanFilter::SimulationState>& samples
) {
    std::ofstream file(filePath);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open file for writing: " + filePath);
    }

    // Write header: step, truth_x, truth_y, truth_vx, truth_vy, meas_x, meas_y, est_x, est_y, est_vx, est_vy
    file << "step,truth_x,truth_y,truth_vx,truth_vy,meas_x,meas_y,est_x,est_y,est_vx,est_vy\n";

    // Write data rows.
    for (const auto& sample : samples) {
        file << sample.step << ",";
        
        for (int i = 0; i < sample.truth.size(); ++i) {
            file << std::fixed << std::setprecision(6) << sample.truth(i);
            if (i < sample.truth.size() - 1) file << ",";
        }
        file << ",";

        for (int i = 0; i < sample.measurement.size(); ++i) {
            file << std::fixed << std::setprecision(6) << sample.measurement(i);
            if (i < sample.measurement.size() - 1) file << ",";
        }
        file << ",";

        for (int i = 0; i < sample.estimate.size(); ++i) {
            file << std::fixed << std::setprecision(6) << sample.estimate(i);
            if (i < sample.estimate.size() - 1) file << ",";
        }
        file << "\n";
    }

    file.close();
    std::cout << "CSV exported to: " << filePath << "\n";
}

/// Generate an SVG plot of position vs time (Truth vs Estimate).
/// \param filePath   Path where SVG will be written.
/// \param samples    Vector of SimulationState samples.
void ExportPositionPlotSVG(
    const std::string& filePath,
    const std::vector<KalmanFilter::SimulationState>& samples
) {
    if (samples.empty()) {
        throw std::runtime_error("No samples to plot");
    }

    constexpr double svgW = 1020.0, svgH = 580.0;
    constexpr double marginL = 90.0, marginR = 28.0, marginT = 60.0, marginB = 72.0;
    const double plotW = svgW - marginL - marginR, plotH = svgH - marginT - marginB;

    // Find min/max for both X and Y position.
    double minX = samples[0].truth(0), maxX = samples[0].truth(0);
    double minY = samples[0].truth(1), maxY = samples[0].truth(1);

    for (const auto& s : samples) {
        minX = std::min(minX, std::min(s.truth(0), std::min(s.measurement(0), s.estimate(0))));
        maxX = std::max(maxX, std::max(s.truth(0), std::max(s.measurement(0), s.estimate(0))));
        minY = std::min(minY, std::min(s.truth(1), std::min(s.measurement(1), s.estimate(1))));
        maxY = std::max(maxY, std::max(s.truth(1), std::max(s.measurement(1), s.estimate(1))));
    }

    const double padX = (maxX - minX) * 0.08;
    const double padY = (maxY - minY) * 0.08;
    minX -= padX; maxX += padX;
    minY -= padY; maxY += padY;

    auto toPixelX = [&](double val) { return marginL + ((val - minX) / (maxX - minX)) * plotW; };
    auto toPixelY = [&](double val) { return marginT + (1.0 - (val - minY) / (maxY - minY)) * plotH; };

    // Build point strings.
    std::ostringstream truthPts, measPts, estPts;
    for (const auto& s : samples) {
        truthPts << std::fixed << std::setprecision(2) << toPixelX(s.truth(0)) << ',' << toPixelY(s.truth(1)) << ' ';
        measPts << std::fixed << std::setprecision(2) << toPixelX(s.measurement(0)) << ',' << toPixelY(s.measurement(1)) << ' ';
        estPts << std::fixed << std::setprecision(2) << toPixelX(s.estimate(0)) << ',' << toPixelY(s.estimate(1)) << ' ';
    }

    // Write SVG.
    std::ofstream file(filePath);
    if (!file.is_open()) throw std::runtime_error("Cannot open SVG file for writing: " + filePath);

    file << "<?xml version='1.0' encoding='UTF-8'?>\n"
         << "<svg xmlns='http://www.w3.org/2000/svg' width='" << svgW << "' height='" << svgH << "'>\n"
         << "<rect width='100%' height='100%' fill='white'/>\n"
         << "<text x='" << (svgW / 2.0) << "' y='34' text-anchor='middle' font-size='18' font-weight='bold' fill='#222'>"
         << "Position Trajectory (X-Y Plane)</text>\n"
         << "<rect x='" << marginL << "' y='" << marginT << "' width='" << plotW << "' height='" << plotH
         << "' fill='none' stroke='#ccc' stroke-width='1'/>\n";

    // Grid lines for X.
    for (int i = 0; i <= 8; ++i) {
        double val = minX + (maxX - minX) * (i / 8.0);
        double px = toPixelX(val);
        file << "<line x1='" << std::fixed << std::setprecision(1) << px << "' y1='" << marginT
             << "' x2='" << px << "' y2='" << (marginT + plotH)
             << "' stroke='#e8e8e8' stroke-width='1'/>\n";
        file << "<text x='" << px << "' y='" << (marginT + plotH + 20.0)
             << "' text-anchor='middle' font-size='11' fill='#555'>"
             << std::fixed << std::setprecision(1) << val << "</text>\n";
    }

    // Grid lines for Y.
    for (int i = 0; i <= 8; ++i) {
        double val = minY + (maxY - minY) * (i / 8.0);
        double py = toPixelY(val);
        file << "<line x1='" << marginL << "' y1='" << std::fixed << std::setprecision(1) << py
             << "' x2='" << (marginL + plotW) << "' y2='" << py
             << "' stroke='#e8e8e8' stroke-width='1'/>\n";
        file << "<text x='" << (marginL - 6.0) << "' y='" << py
             << "' text-anchor='end' dominant-baseline='middle' font-size='11' fill='#555'>"
             << std::fixed << std::setprecision(1) << val << "</text>\n";
    }

    // Polylines.
    file << "<polyline points='" << measPts.str() << "' fill='none' stroke='#F5A623' stroke-width='1.4' stroke-dasharray='4,3' opacity='0.7'/>\n"
         << "<polyline points='" << truthPts.str() << "' fill='none' stroke='#D0021B' stroke-width='2.2'/>\n"
         << "<polyline points='" << estPts.str() << "' fill='none' stroke='#2196F3' stroke-width='2.8'/>\n";

    // Axis labels.
    file << "<text x='" << (marginL + plotW / 2.0) << "' y='" << (svgH - 14.0)
         << "' text-anchor='middle' font-size='13' fill='#333'>Position X</text>\n"
         << "<text x='18' y='" << (marginT + plotH / 2.0)
         << "' text-anchor='middle' font-size='13' fill='#333' transform='rotate(-90,18," << (marginT + plotH / 2.0) << ")'>"
         << "Position Y</text>\n";

    // Legend.
    const double legX = marginL + plotW - 170.0, legY = marginT + 12.0;
    file << "<rect x='" << legX << "' y='" << legY << "' width='155' height='85' fill='white' fill-opacity='0.9' stroke='#ccc' rx='4'/>\n"
         << "<line x1='" << (legX + 8.0) << "' y1='" << (legY + 16.0) << "' x2='" << (legX + 38.0) << "' y2='" << (legY + 16.0)
         << "' stroke='#D0021B' stroke-width='2.2'/>\n"
         << "<text x='" << (legX + 44.0) << "' y='" << (legY + 20.0) << "' font-size='12' fill='#333'>Ground truth</text>\n"
         << "<line x1='" << (legX + 8.0) << "' y1='" << (legY + 36.0) << "' x2='" << (legX + 38.0) << "' y2='" << (legY + 36.0)
         << "' stroke='#F5A623' stroke-width='1.4' stroke-dasharray='4,3'/>\n"
         << "<text x='" << (legX + 44.0) << "' y='" << (legY + 40.0) << "' font-size='12' fill='#333'>Measurement</text>\n"
         << "<line x1='" << (legX + 8.0) << "' y1='" << (legY + 56.0) << "' x2='" << (legX + 38.0) << "' y2='" << (legY + 56.0)
         << "' stroke='#2196F3' stroke-width='2.8'/>\n"
         << "<text x='" << (legX + 44.0) << "' y='" << (legY + 60.0) << "' font-size='12' fill='#333'>Kalman estimate</text>\n"
         << "</svg>\n";

    file.close();
    std::cout << "Position plot SVG exported to: " << filePath << "\n";
}

/// Generate an SVG plot of velocity vs time (Estimate only, since velocity is not directly measured).
/// \param filePath   Path where SVG will be written.
/// \param samples    Vector of SimulationState samples.
void ExportVelocityPlotSVG(
    const std::string& filePath,
    const std::vector<KalmanFilter::SimulationState>& samples
) {
    if (samples.empty()) {
        throw std::runtime_error("No samples to plot");
    }

    constexpr double svgW = 1000.0, svgH = 580.0;
    constexpr double marginL = 80.0, marginR = 28.0, marginT = 60.0, marginB = 72.0;
    const double plotW = svgW - marginL - marginR, plotH = svgH - marginT - marginB;

    // Find min/max for velocity components.
    double minVx = samples[0].truth(2), maxVx = samples[0].truth(2);
    double minVy = samples[0].truth(3), maxVy = samples[0].truth(3);

    for (const auto& s : samples) {
        minVx = std::min(minVx, std::min(s.truth(2), s.estimate(2)));
        maxVx = std::max(maxVx, std::max(s.truth(2), s.estimate(2)));
        minVy = std::min(minVy, std::min(s.truth(3), s.estimate(3)));
        maxVy = std::max(maxVy, std::max(s.truth(3), s.estimate(3)));
    }

    const double padVx = (maxVx - minVx) * 0.08;
    const double padVy = (maxVy - minVy) * 0.08;
    minVx -= padVx; maxVx += padVx;
    minVy -= padVy; maxVy += padVy;

    auto toTimeX = [&](int step) { return marginL + (step / static_cast<double>(samples.size() - 1)) * plotW; };
    auto toVxY = [&](double val) { return marginT + (1.0 - (val - minVx) / (maxVx - minVx)) * plotH * 0.48; };
    auto toVyY = [&](double val) { return marginT + plotH * 0.52 + (1.0 - (val - minVy) / (maxVy - minVy)) * plotH * 0.48; };

    // Build point strings for Vx and Vy.
    std::ostringstream truthVxPts, estVxPts, truthVyPts, estVyPts;
    for (const auto& s : samples) {
        double x = toTimeX(s.step);
        truthVxPts << std::fixed << std::setprecision(2) << x << ',' << toVxY(s.truth(2)) << ' ';
        estVxPts << std::fixed << std::setprecision(2) << x << ',' << toVxY(s.estimate(2)) << ' ';
        truthVyPts << std::fixed << std::setprecision(2) << x << ',' << toVyY(s.truth(3)) << ' ';
        estVyPts << std::fixed << std::setprecision(2) << x << ',' << toVyY(s.estimate(3)) << ' ';
    }

    // Write SVG.
    std::ofstream file(filePath);
    if (!file.is_open()) throw std::runtime_error("Cannot open SVG file for writing: " + filePath);

    file << "<?xml version='1.0' encoding='UTF-8'?>\n"
         << "<svg xmlns='http://www.w3.org/2000/svg' width='" << svgW << "' height='" << svgH << "'>\n"
         << "<rect width='100%' height='100%' fill='white'/>\n"
         << "<text x='" << (svgW / 2.0) << "' y='34' text-anchor='middle' font-size='18' font-weight='bold' fill='#222'>"
         << "Velocity Estimates vs Time (Vx and Vy)</text>\n";

    // Dividing line between Vx and Vy subplots.
    file << "<line x1='" << marginL << "' y1='" << (marginT + plotH * 0.5) << "' x2='" << (marginL + plotW)
         << "' y2='" << (marginT + plotH * 0.5) << "' stroke='#ddd' stroke-width='2' stroke-dasharray='6,4'/>\n";

    // Vx subplot border.
    file << "<rect x='" << marginL << "' y='" << marginT << "' width='" << plotW << "' height='" << (plotH * 0.48)
         << "' fill='none' stroke='#ccc' stroke-width='1'/>\n";

    // Vy subplot border.
    file << "<rect x='" << marginL << "' y='" << (marginT + plotH * 0.52) << "' width='" << plotW << "' height='" << (plotH * 0.48)
         << "' fill='none' stroke='#ccc' stroke-width='1'/>\n";

    // Grid and labels for Vx.
    for (int i = 0; i <= 8; ++i) {
        double val = minVx + (maxVx - minVx) * (i / 8.0);
        double py = toVxY(val);
        file << "<line x1='" << marginL << "' y1='" << std::fixed << std::setprecision(1) << py
             << "' x2='" << (marginL + plotW) << "' y2='" << py
             << "' stroke='#f0f0f0' stroke-width='1'/>\n";
        file << "<text x='" << (marginL - 4.0) << "' y='" << py
             << "' text-anchor='end' dominant-baseline='middle' font-size='10' fill='#666'>"
             << std::fixed << std::setprecision(2) << val << "</text>\n";
    }

    // Grid and labels for Vy.
    for (int i = 0; i <= 8; ++i) {
        double val = minVy + (maxVy - minVy) * (i / 8.0);
        double py = toVyY(val);
        file << "<line x1='" << marginL << "' y1='" << std::fixed << std::setprecision(1) << py
             << "' x2='" << (marginL + plotW) << "' y2='" << py
             << "' stroke='#f0f0f0' stroke-width='1'/>\n";
        file << "<text x='" << (marginL - 4.0) << "' y='" << py
             << "' text-anchor='end' dominant-baseline='middle' font-size='10' fill='#666'>"
             << std::fixed << std::setprecision(2) << val << "</text>\n";
    }

    // Time axis labels (shared).
    for (int i = 0; i <= 10; ++i) {
        int step = (i * samples.size()) / 10;
        double px = toTimeX(step);
        file << "<text x='" << std::fixed << std::setprecision(1) << px
             << "' y='" << (marginT + plotH + 20.0) << "' text-anchor='middle' font-size='11' fill='#555'>"
             << step << "</text>\n";
    }

    // Polylines for Vx.
    file << "<polyline points='" << truthVxPts.str() << "' fill='none' stroke='#D0021B' stroke-width='2.0' opacity='0.8'/>\n"
         << "<polyline points='" << estVxPts.str() << "' fill='none' stroke='#2196F3' stroke-width='2.5'/>\n";

    // Polylines for Vy.
    file << "<polyline points='" << truthVyPts.str() << "' fill='none' stroke='#D0021B' stroke-width='2.0' opacity='0.8'/>\n"
         << "<polyline points='" << estVyPts.str() << "' fill='none' stroke='#2196F3' stroke-width='2.5'/>\n";

    // Subplot titles.
    file << "<text x='" << (marginL + plotW / 2.0) << "' y='" << (marginT + 22.0)
         << "' text-anchor='middle' font-size='13' font-weight='bold' fill='#333'>Velocity X (Vx)</text>\n"
         << "<text x='" << (marginL + plotW / 2.0) << "' y='" << (marginT + plotH * 0.52 + 22.0)
         << "' text-anchor='middle' font-size='13' font-weight='bold' fill='#333'>Velocity Y (Vy)</text>\n";

    // Shared x-axis label.
    file << "<text x='" << (marginL + plotW / 2.0) << "' y='" << (svgH - 14.0)
         << "' text-anchor='middle' font-size='13' fill='#333'>Time step k</text>\n";

    // Legend.
    const double legX = marginL + plotW - 140.0, legY = marginT + 12.0;
    file << "<rect x='" << legX << "' y='" << legY << "' width='135' height='65' fill='white' fill-opacity='0.92' stroke='#ccc' rx='4'/>\n"
         << "<line x1='" << (legX + 8.0) << "' y1='" << (legY + 16.0) << "' x2='" << (legX + 38.0) << "' y2='" << (legY + 16.0)
         << "' stroke='#D0021B' stroke-width='2.0'/>\n"
         << "<text x='" << (legX + 44.0) << "' y='" << (legY + 20.0) << "' font-size='11' fill='#333'>Ground truth</text>\n"
         << "<line x1='" << (legX + 8.0) << "' y1='" << (legY + 36.0) << "' x2='" << (legX + 38.0) << "' y2='" << (legY + 36.0)
         << "' stroke='#2196F3' stroke-width='2.5'/>\n"
         << "<text x='" << (legX + 44.0) << "' y='" << (legY + 40.0) << "' font-size='11' fill='#333'>Kalman est.</text>\n"
         << "</svg>\n";

    file.close();
    std::cout << "Velocity plot SVG exported to: " << filePath << "\n";
}

}  // namespace utils
