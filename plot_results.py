#!/usr/bin/env python3
"""Generates comparison plots from a completed sweep (see run_experiment.py --sweep).

Reads results/summary.csv + results/sweep/*_tasks.csv + *_ticks.csv and
writes PNGs to results/plots/.
"""
import argparse
import glob
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

REPO_ROOT = os.path.dirname(os.path.abspath(__file__))


def load_sweep(sweep_dir):
    tasks = {}
    ticks = {}
    for path in sorted(glob.glob(os.path.join(sweep_dir, "*_tasks.csv"))):
        label = os.path.basename(path)[: -len("_tasks.csv")]
        df = pd.read_csv(path)
        if not df.empty:
            tasks[label] = df
    for path in sorted(glob.glob(os.path.join(sweep_dir, "*_ticks.csv"))):
        label = os.path.basename(path)[: -len("_ticks.csv")]
        ticks[label] = pd.read_csv(path)
    return tasks, ticks


def config_of(label):
    # label format: "<scheduler>_<offloading>_seed<N>"
    parts = label.rsplit("_seed", 1)
    return parts[0]


def plot_latency_bars(tasks, out_dir):
    rows = []
    for label, df in tasks.items():
        rows.append({"config": config_of(label), "mean_latency_s": df["e2e_latency_s"].mean(),
                     "p95_latency_s": df["e2e_latency_s"].quantile(0.95)})
    agg = pd.DataFrame(rows).groupby("config").agg(["mean", "std"])

    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5))
    for ax, col, title in zip(axes, ["mean_latency_s", "p95_latency_s"],
                               ["Mean end-to-end task latency", "P95 end-to-end task latency"]):
        means = agg[col]["mean"].sort_values()
        stds = agg[col]["std"].reindex(means.index)
        ax.bar(means.index, means.values, yerr=stds.values, capsize=4,
               color=["#4C72B0", "#DD8452", "#55A868", "#C44E52"][: len(means)])
        ax.set_title(title)
        ax.set_ylabel("seconds")
        ax.tick_params(axis="x", rotation=30)
    fig.suptitle("Scheduler x Offloading Policy Comparison (mean +/- std over seeds)")
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "latency_by_config.png"), dpi=150)
    plt.close(fig)


def plot_latency_cdf(tasks, out_dir):
    by_config = {}
    for label, df in tasks.items():
        by_config.setdefault(config_of(label), []).append(df["e2e_latency_s"].values)

    fig, ax = plt.subplots(figsize=(7, 5))
    for config, arrays in sorted(by_config.items()):
        all_lat = np.concatenate(arrays)
        xs = np.sort(all_lat)
        ys = np.arange(1, len(xs) + 1) / len(xs)
        ax.plot(xs, ys, label=config, linewidth=2)
    ax.set_xlabel("End-to-end task latency (s)")
    ax.set_ylabel("CDF")
    ax.set_title("Task latency distribution by configuration")
    ax.legend()
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "latency_cdf.png"), dpi=150)
    plt.close(fig)


def plot_completion_over_time(ticks, out_dir, seed_suffix="_seed1"):
    fig, ax = plt.subplots(figsize=(7, 5))
    for label, df in sorted(ticks.items()):
        if not label.endswith(seed_suffix):
            continue
        ax.plot(df["time_s"], df["cum_completed"], label=config_of(label), linewidth=2)
    ax.set_xlabel("Simulation time (s)")
    ax.set_ylabel("Cumulative tasks completed")
    ax.set_title(f"Task throughput over time ({seed_suffix.lstrip('_')})")
    ax.legend()
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "throughput_over_time.png"), dpi=150)
    plt.close(fig)


def plot_mobility_dynamics(ticks, out_dir, example_label):
    if example_label not in ticks:
        example_label = sorted(ticks.keys())[0]
    df = ticks[example_label]

    fig, axes = plt.subplots(3, 1, figsize=(8, 9), sharex=True)

    axes[0].plot(df["time_s"], df["num_vehicles"], color="#4C72B0")
    axes[0].set_ylabel("Vehicles in sim")
    axes[0].set_title(f"Real SUMO mobility driving the pipeline ({example_label})")

    axes[1].plot(df["time_s"], df["server_100_latency_ms"], label="server 100 (near corner)")
    axes[1].plot(df["time_s"], df["server_101_latency_ms"], label="server 101 (far corner)")
    axes[1].set_ylabel("Link latency (ms)")
    axes[1].legend()

    axes[2].plot(df["time_s"], df["server_100_queue"], label="server 100 queue")
    axes[2].plot(df["time_s"], df["server_101_queue"], label="server 101 queue")
    axes[2].set_ylabel("Queue length (tasks)")
    axes[2].set_xlabel("Simulation time (s)")
    axes[2].legend()

    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "mobility_and_congestion.png"), dpi=150)
    plt.close(fig)


def plot_speed_vs_latency(ticks, out_dir):
    frames = []
    for label, df in ticks.items():
        d = df[["mean_speed_mps", "server_100_latency_ms", "server_101_latency_ms"]].copy()
        frames.append(d)
    all_df = pd.concat(frames, ignore_index=True)
    all_df = all_df[(all_df["server_100_latency_ms"] > 0) | (all_df["server_101_latency_ms"] > 0)]

    fig, ax = plt.subplots(figsize=(7, 5))
    ax.scatter(all_df["mean_speed_mps"], all_df["server_100_latency_ms"], alpha=0.15, s=10,
               label="server 100", color="#4C72B0")
    ax.scatter(all_df["mean_speed_mps"], all_df["server_101_latency_ms"], alpha=0.15, s=10,
               label="server 101", color="#DD8452")
    ax.set_xlabel("Mean vehicle speed in network (m/s)")
    ax.set_ylabel("Link latency (ms)")
    ax.set_title("Traffic speed vs. edge-server link latency\n(real SUMO trajectories, across all runs)")
    ax.legend()
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "speed_vs_latency.png"), dpi=150)
    plt.close(fig)


def plot_dispatch_vs_complete(tasks, out_dir):
    rows = []
    for label, df in tasks.items():
        rows.append({"config": config_of(label), "n_tasks": len(df)})
    agg = pd.DataFrame(rows).groupby("config")["n_tasks"].agg(["mean", "std"]).sort_values("mean")

    fig, ax = plt.subplots(figsize=(7, 5))
    ax.bar(agg.index, agg["mean"], yerr=agg["std"], capsize=4, color="#55A868")
    ax.set_ylabel("Tasks completed (mean over seeds)")
    ax.set_title("Total tasks completed per configuration")
    ax.tick_params(axis="x", rotation=30)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "tasks_completed_by_config.png"), dpi=150)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sweep-dir", default=os.path.join(REPO_ROOT, "results", "sweep"))
    parser.add_argument("--out-dir", default=os.path.join(REPO_ROOT, "results", "plots"))
    args = parser.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    tasks, ticks = load_sweep(args.sweep_dir)
    if not tasks:
        raise SystemExit(f"No *_tasks.csv found in {args.sweep_dir} -- run the sweep first.")

    plot_latency_bars(tasks, args.out_dir)
    plot_latency_cdf(tasks, args.out_dir)
    plot_completion_over_time(ticks, args.out_dir)
    plot_mobility_dynamics(ticks, args.out_dir, example_label="heft_greedy_seed1")
    plot_speed_vs_latency(ticks, args.out_dir)
    plot_dispatch_vs_complete(tasks, args.out_dir)

    print(f"Wrote plots to {args.out_dir}")


if __name__ == "__main__":
    main()
