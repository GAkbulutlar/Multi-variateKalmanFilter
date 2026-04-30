#!/usr/bin/env python3
"""Generate RMSE comparison chart and trajectory animation from simulation_results.csv."""

from pathlib import Path
import csv

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from typing import Dict, List


def load_csv_series(csv_path: Path) -> Dict[str, List[float]]:
    """Load numeric CSV columns into lists keyed by column name."""
    with csv_path.open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        fieldnames = reader.fieldnames or []
        required_cols = {
            "truth_x",
            "truth_y",
            "meas_x",
            "meas_y",
            "est_x",
            "est_y",
            "meas_error_sq",
            "kf_error_sq",
        }
        missing = sorted(required_cols - set(fieldnames))
        if missing:
            raise ValueError(f"CSV is missing required columns: {', '.join(missing)}")

        series: Dict[str, List[float]] = {name: [] for name in required_cols}
        for row in reader:
            for key in required_cols:
                series[key].append(float(row[key]))

    if not series["truth_x"]:
        raise ValueError("CSV has no data rows")

    return series


def create_rmse_bar_chart(series: Dict[str, List[float]], output_path: Path) -> None:
    """Create side-by-side RMSE bar chart from squared error columns."""
    meas_mean = sum(series["meas_error_sq"]) / len(series["meas_error_sq"])
    kf_mean = sum(series["kf_error_sq"]) / len(series["kf_error_sq"])
    rmse_meas = meas_mean ** 0.5
    rmse_kf = kf_mean ** 0.5

    fig, ax = plt.subplots(figsize=(7, 5))
    labels = ["Measurement", "Kalman Estimate"]
    values = [rmse_meas, rmse_kf]
    colors = ["#F5A623", "#2196F3"]

    bars = ax.bar(labels, values, color=colors)
    ax.set_title("Position RMSE Comparison")
    ax.set_ylabel("RMSE [units]")
    ax.grid(axis="y", linestyle="--", alpha=0.4)

    for bar, value in zip(bars, values):
        ax.text(
            bar.get_x() + bar.get_width() / 2,
            value,
            f"{value:.3f}",
            ha="center",
            va="bottom",
            fontsize=10,
        )

    fig.tight_layout()
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def create_trajectory_animation(series: Dict[str, List[float]], output_path: Path) -> None:
    """Create animated trajectory GIF where lines grow frame-by-frame."""
    truth_x = series["truth_x"]
    truth_y = series["truth_y"]
    meas_x = series["meas_x"]
    meas_y = series["meas_y"]
    est_x = series["est_x"]
    est_y = series["est_y"]

    min_x = min(min(truth_x), min(meas_x), min(est_x))
    max_x = max(max(truth_x), max(meas_x), max(est_x))
    min_y = min(min(truth_y), min(meas_y), min(est_y))
    max_y = max(max(truth_y), max(meas_y), max(est_y))

    pad_x = 0.08 * (max_x - min_x if max_x > min_x else 1.0)
    pad_y = 0.08 * (max_y - min_y if max_y > min_y else 1.0)

    fig, ax = plt.subplots(figsize=(8, 6))
    ax.set_xlim(min_x - pad_x, max_x + pad_x)
    ax.set_ylim(min_y - pad_y, max_y + pad_y)
    ax.set_xlabel("Position X")
    ax.set_ylabel("Position Y")
    ax.set_title("Trajectory Animation: Truth vs Measurement vs Kalman")
    ax.grid(True, linestyle="--", alpha=0.35)

    truth_line, = ax.plot([], [], color="#D0021B", linewidth=2.2, label="Ground truth")
    meas_line, = ax.plot([], [], color="#F5A623", linewidth=1.5, linestyle="--", alpha=0.75, label="Measurement")
    est_line, = ax.plot([], [], color="#2196F3", linewidth=2.5, label="Kalman estimate")
    ax.legend(loc="best")

    def update(frame: int):
        idx = frame + 1
        truth_line.set_data(truth_x[:idx], truth_y[:idx])
        meas_line.set_data(meas_x[:idx], meas_y[:idx])
        est_line.set_data(est_x[:idx], est_y[:idx])
        return truth_line, meas_line, est_line

    animation = FuncAnimation(
        fig,
        update,
        frames=len(truth_x),
        interval=40,
        blit=True,
        repeat=True,
    )

    animation.save(output_path, writer="pillow", fps=25)
    plt.close(fig)


def main() -> None:
    root = Path(__file__).resolve().parent
    csv_path = root / "simulation_results.csv"

    if not csv_path.exists():
        raise FileNotFoundError(f"CSV not found: {csv_path}")

    series = load_csv_series(csv_path)

    rmse_chart_path = root / "rmse_comparison.png"
    gif_path = root / "trajectory_animation.gif"

    create_rmse_bar_chart(series, rmse_chart_path)
    create_trajectory_animation(series, gif_path)

    print(f"RMSE chart saved to: {rmse_chart_path}")
    print(f"Trajectory animation saved to: {gif_path}")


if __name__ == "__main__":
    main()
