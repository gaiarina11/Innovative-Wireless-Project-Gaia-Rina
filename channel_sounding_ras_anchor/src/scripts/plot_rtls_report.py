#!/usr/bin/env python3
"""Genera figure statiche per il report RTLS a partire dal CSV host."""

import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib
import numpy as np
from matplotlib.patches import Circle, Ellipse

matplotlib.use("Agg")
import matplotlib.pyplot as plt


ANCHOR_IDS = (0, 1, 2)


def parse_position(spec):
    try:
        values = tuple(float(value) for value in spec.split(","))
    except ValueError as exc:
        raise argparse.ArgumentTypeError("formato richiesto: X,Y") from exc
    if len(values) != 2 or not all(math.isfinite(value) for value in values):
        raise argparse.ArgumentTypeError("formato richiesto: X,Y")
    return values


def load_anchors(path):
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    return {anchor_id: tuple(data["anchors_m"][str(anchor_id)]) for anchor_id in ANCHOR_IDS}


def load_rows(path):
    with Path(path).open(encoding="utf-8", newline="") as handle:
        rows = [row for row in csv.DictReader(handle) if row["status"] == "accepted"]
    rows = [row for row in rows if row["filtered_x_m"] and row["filtered_y_m"]]
    if not rows:
        raise ValueError("il CSV non contiene posizioni accettate")
    return rows


def setup_axes(anchors, title):
    fig, ax = plt.subplots(figsize=(8, 7))
    for anchor_id, point in anchors.items():
        ax.scatter(*point, marker="s", s=150, color="blue", zorder=5)
        ax.annotate(
            f"Anchor {anchor_id}", point, xytext=(0, 16),
            textcoords="offset points", ha="center", fontweight="bold"
        )
    ax.set_title(title)
    ax.set_xlabel("X (meters)")
    ax.set_ylabel("Y (meters)")
    ax.grid(True, alpha=0.55)
    ax.set_aspect("equal", adjustable="box")
    return fig, ax


def include_points(ax, anchors, points, margin=0.25):
    all_points = np.vstack((np.asarray(list(anchors.values()), dtype=float), points))
    minima = np.min(all_points, axis=0) - margin
    maxima = np.max(all_points, axis=0) + margin
    span = np.maximum(maxima - minima, 1.0)
    center = (minima + maxima) / 2.0
    half = max(span) / 2.0
    ax.set_xlim(center[0] - half, center[0] + half)
    ax.set_ylim(center[1] - half, center[1] + half)


def add_confidence_ellipse(ax, points):
    if len(points) < 3:
        return
    covariance = np.cov(points.T)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    order = np.argsort(eigenvalues)[::-1]
    eigenvalues = eigenvalues[order]
    eigenvectors = eigenvectors[:, order]
    angle = math.degrees(math.atan2(eigenvectors[1, 0], eigenvectors[0, 0]))
    # sqrt(chi-square quantile 95%, 2 DoF) = sqrt(5.991)
    scale = math.sqrt(5.991)
    width, height = 2.0 * scale * np.sqrt(np.maximum(eigenvalues, 0.0))
    ax.add_patch(
        Ellipse(
            np.mean(points, axis=0), width, height, angle=angle,
            facecolor="red", edgecolor="darkred", alpha=0.12,
            linewidth=1.5, label="95% confidence ellipse"
        )
    )


def plot_static(rows, anchors, ground_truth, output):
    points = np.asarray(
        [(float(row["filtered_x_m"]), float(row["filtered_y_m"])) for row in rows]
    )
    mean = np.mean(points, axis=0)
    fig, ax = setup_axes(anchors, "RTLS - Static Tag Stability")
    ax.scatter(points[:, 0], points[:, 1], s=24, color="red", alpha=0.30,
               label="Position estimates")
    ax.scatter(*mean, marker="*", s=260, color="darkred", label="Mean position", zorder=7)
    add_confidence_ellipse(ax, points)

    metrics = [f"N = {len(points)}", f"stdX = {np.std(points[:, 0]):.3f} m",
               f"stdY = {np.std(points[:, 1]):.3f} m"]
    extra = [mean]
    if ground_truth is not None:
        truth = np.asarray(ground_truth, dtype=float)
        errors = np.linalg.norm(points - truth, axis=1)
        ax.scatter(*truth, marker="x", s=180, linewidths=3, color="black",
                   label="Ground truth", zorder=8)
        metrics.extend(
            [f"mean error = {np.mean(errors):.3f} m",
             f"P95 = {np.percentile(errors, 95):.3f} m"]
        )
        extra.append(truth)
    ax.text(0.02, 0.98, "\n".join(metrics), transform=ax.transAxes, va="top",
            bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.9})
    include_points(ax, anchors, np.asarray(extra))
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(output, dpi=220, bbox_inches="tight")


def plot_trajectory(rows, anchors, output):
    points = np.asarray(
        [(float(row["filtered_x_m"]), float(row["filtered_y_m"])) for row in rows]
    )
    fig, ax = setup_axes(anchors, "RTLS - Estimated Trajectory")
    ax.plot(points[:, 0], points[:, 1], color="0.55", linewidth=1.2, alpha=0.7)
    colors = np.arange(len(points))
    scatter = ax.scatter(points[:, 0], points[:, 1], c=colors, cmap="viridis",
                         s=30, label="Estimated trajectory", zorder=5)
    ax.scatter(*points[0], color="limegreen", edgecolor="black", s=130,
               label="Start", zorder=7)
    ax.scatter(*points[-1], marker="X", color="red", edgecolor="black", s=150,
               label="End", zorder=7)
    fig.colorbar(scatter, ax=ax, label="Sample time index")
    include_points(ax, anchors, points)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(output, dpi=220, bbox_inches="tight")


def plot_ranging(rows, anchors, output):
    row = rows[-1]
    position = np.asarray((float(row["filtered_x_m"]), float(row["filtered_y_m"])))
    fig, ax = setup_axes(anchors, "RTLS - Trilateration from One Distance Set")
    colors = ("tab:blue", "tab:orange", "tab:green")
    for anchor_id, color in zip(ANCHOR_IDS, colors):
        value = row[f"filtered_distance_a{anchor_id}"]
        if not value:
            continue
        distance = float(value)
        ax.add_patch(
            Circle(anchors[anchor_id], distance, fill=False, linestyle="--",
                   linewidth=1.5, color=color, alpha=0.8,
                   label=f"D{anchor_id} = {distance:.3f} m")
        )
        ax.plot(
            (anchors[anchor_id][0], position[0]),
            (anchors[anchor_id][1], position[1]),
            linestyle=":", color=color, alpha=0.7
        )
    ax.scatter(*position, marker="*", s=300, color="red", label="Estimated tag", zorder=8)
    radius = max(
        [float(row[f"filtered_distance_a{i}"]) for i in ANCHOR_IDS
         if row[f"filtered_distance_a{i}"]] + [0.5]
    )
    include_points(ax, anchors, np.asarray([position]), margin=max(0.25, radius * 0.15))
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(output, dpi=220, bbox_inches="tight")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--kind", choices=("static", "trajectory", "ranging"), required=True)
    parser.add_argument("--ground-truth", type=parse_position, metavar="X,Y")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    if args.kind == "static" and args.ground_truth is None:
        parser.error("--kind static richiede --ground-truth X,Y")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    anchors = load_anchors(args.config)
    rows = load_rows(args.csv)
    if args.kind == "static":
        plot_static(rows, anchors, args.ground_truth, args.output)
    elif args.kind == "trajectory":
        plot_trajectory(rows, anchors, args.output)
    else:
        plot_ranging(rows, anchors, args.output)
    print(f"[REPORT] Grafico salvato: {args.output}")


if __name__ == "__main__":
    main()
