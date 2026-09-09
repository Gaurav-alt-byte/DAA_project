#!/usr/bin/env python3
"""Runs the IoV edge-scheduling experiment against real SUMO.

Launching SUMO and launching the C++ experiment_driver are kept in this
one script deliberately, so the C++ side (traci_client.h / sumo_bridge)
never has to spawn or manage a child process itself -- it only ever
connects to a TraCI port that's already listening. That keeps the C++
side identical on Windows and Linux.

Usage:
    python3 run_experiment.py --scenario sumo_scenario/grid.sumocfg \
        --scheduler heft --offloading greedy --duration 200 --seed 1

    # Or run the full comparison sweep used for the report plots:
    python3 run_experiment.py --sweep
"""
import argparse
import itertools
import os
import platform
import socket
import subprocess
import sys
import time

REPO_ROOT = os.path.dirname(os.path.abspath(__file__))


def find_free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def find_sumo_binary() -> str:
    # Prefer SUMO_HOME if set (matches the project's existing setup notes).
    sumo_home = os.environ.get("SUMO_HOME")
    if sumo_home:
        candidate = os.path.join(sumo_home, "bin", "sumo.exe" if platform.system() == "Windows" else "sumo")
        if os.path.exists(candidate):
            return candidate
    # Fall back to the `sumo` package installed via `pip install eclipse-sumo`.
    try:
        import sumo as sumo_pkg
        candidate = os.path.join(
            os.path.dirname(sumo_pkg.__file__), "bin",
            "sumo.exe" if platform.system() == "Windows" else "sumo")
        if os.path.exists(candidate):
            return candidate
    except ImportError:
        pass
    # Last resort: whatever's on PATH.
    return "sumo"


def run_one(scenario, scheduler, offloading, duration, step_length, arrival_rate,
            seed, out_prefix, label, driver_bin, sumo_bin, use_mock=False, quiet=False):
    os.makedirs(os.path.dirname(out_prefix) or ".", exist_ok=True)

    if use_mock:
        cmd = [driver_bin, "--mock", "--duration", str(duration), "--scheduler", scheduler,
               "--offloading", offloading, "--arrival-rate", str(arrival_rate),
               "--seed", str(seed), "--out-prefix", out_prefix, "--label", label]
        if not quiet:
            print(f"[{label}] running against mock mobility...")
        subprocess.run(cmd, check=True, cwd=REPO_ROOT)
        return

    port = find_free_port()
    sumo_cmd = [sumo_bin, "-c", scenario, "--remote-port", str(port),
                "--step-length", str(step_length)]
    if not quiet:
        print(f"[{label}] starting SUMO on port {port}: {' '.join(sumo_cmd)}")
    sumo_proc = subprocess.Popen(sumo_cmd, cwd=REPO_ROOT,
                                  stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        # No readiness probe here on purpose: SUMO's TraCI server accepts
        # exactly one client connection, and any probe connect() would
        # itself consume that slot before experiment_driver gets to it.
        # experiment_driver's TraciClient already retries its own
        # connection attempts internally, so we just give the process a
        # brief head start and let it handle the rest.
        time.sleep(0.3)
        driver_cmd = [driver_bin, "--port", str(port), "--duration", str(duration),
                      "--step-length", str(step_length), "--scheduler", scheduler,
                      "--offloading", offloading, "--arrival-rate", str(arrival_rate),
                      "--seed", str(seed), "--out-prefix", out_prefix, "--label", label]
        if not quiet:
            print(f"[{label}] running experiment_driver...")
        subprocess.run(driver_cmd, check=True, cwd=REPO_ROOT)
    finally:
        sumo_proc.terminate()
        try:
            sumo_proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            sumo_proc.kill()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", default="sumo_scenario/grid.sumocfg")
    parser.add_argument("--scheduler", default="heft", choices=["heft", "fifo"])
    parser.add_argument("--offloading", default="greedy", choices=["greedy", "nearest"])
    parser.add_argument("--duration", type=float, default=200.0)
    parser.add_argument("--step-length", type=float, default=0.1)
    parser.add_argument("--arrival-rate", type=float, default=0.5)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--out-prefix", default="results/run")
    parser.add_argument("--label", default=None)
    parser.add_argument("--mock", action="store_true", help="use synthetic mobility instead of SUMO")
    parser.add_argument("--driver-bin", default=os.path.join(REPO_ROOT, "experiment_driver"))
    parser.add_argument("--sumo-bin", default=None)
    parser.add_argument("--sweep", action="store_true",
                         help="run every scheduler x offloading combo x 3 seeds, for the comparison plots")
    parser.add_argument("--sweep-seeds", default="1,2,3")
    parser.add_argument("--sweep-duration", type=float, default=200.0)
    args = parser.parse_args()

    if not os.path.exists(args.driver_bin) and not os.path.exists(args.driver_bin + ".exe"):
        sys.exit(f"experiment_driver not found at {args.driver_bin} -- build it first (see README.md)")

    sumo_bin = args.sumo_bin or find_sumo_binary()

    if args.sweep:
        seeds = [int(s) for s in args.sweep_seeds.split(",")]
        combos = list(itertools.product(["heft", "fifo"], ["greedy", "nearest"], seeds))
        print(f"Running sweep: {len(combos)} configs ({args.sweep_duration}s each)...")
        # Fresh summary each sweep run.
        summary_path = os.path.join(REPO_ROOT, "results", "summary.csv")
        if os.path.exists(summary_path):
            os.remove(summary_path)
        for scheduler, offloading, seed in combos:
            label = f"{scheduler}_{offloading}_seed{seed}"
            out_prefix = os.path.join("results", "sweep", label)
            run_one(args.scenario, scheduler, offloading, args.sweep_duration, args.step_length,
                    args.arrival_rate, seed, out_prefix, label, args.driver_bin, sumo_bin,
                    use_mock=args.mock, quiet=True)
            print(f"  done: {label}")
        print("Sweep complete. See results/summary.csv and results/sweep/*.csv")
        return

    label = args.label or f"{args.scheduler}_{args.offloading}"
    run_one(args.scenario, args.scheduler, args.offloading, args.duration, args.step_length,
            args.arrival_rate, args.seed, args.out_prefix, label, args.driver_bin, sumo_bin,
            use_mock=args.mock)


if __name__ == "__main__":
    main()
