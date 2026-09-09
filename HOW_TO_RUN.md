# How to build and run the real SUMO integration

Tested here on Linux; you're on Windows/MSYS2 per earlier notes, so both
are covered below. The C++ code is platform-agnostic (Winsock vs POSIX
sockets are handled with `#ifdef _WIN32` in `traci_client.h`), only the
build command differs.

## 1. Prerequisites

- **SUMO itself.** Easiest path, same on both platforms:
  ```
  pip install eclipse-sumo
  ```
  This installs real SUMO binaries via the `sumo` Python package —
  `run_experiment.py` finds them automatically. (Alternatively, install
  SUMO normally and set `SUMO_HOME`.)
- **Python 3** with `pandas` and `matplotlib` for the plotting script:
  ```
  pip install pandas matplotlib
  ```
- **A C++17 compiler.** On Windows/MSYS2: `pacman -S mingw-w64-ucrt-x86_64-gcc`
  (or whichever toolchain you're already using for the rest of the
  project).

## 2. Build

From the project root:

**Linux / macOS:**
```
g++ -std=c++17 -O2 -I. -o experiment_driver main.cpp simulation/sumo_integration/sumo_bridge.cpp
```

**Windows (MSYS2 UCRT64 shell):**
```
g++ -std=c++17 -O2 -I. -o experiment_driver.exe main.cpp simulation/sumo_integration/sumo_bridge.cpp -lws2_32
```
(`-lws2_32` links Winsock — required on Windows, harmless-but-unnecessary
flag doesn't exist on Linux, which is why the two commands differ.)

If your project already has a root `CMakeLists.txt` that does
`add_subdirectory(simulation/sumo_integration)`, link your main
executable's target against the `sumo_integration` target instead — its
`CMakeLists.txt` is already updated to build `sumo_bridge.cpp` and pull
in `ws2_32` on Windows automatically.

## 3. Run a single experiment

```
python3 run_experiment.py --scheduler heft --offloading greedy \
    --duration 200 --arrival-rate 1.2 --seed 1 --out-prefix results/run
```

This launches `sumo` headless on a free port with
`sumo_scenario/grid.sumocfg`, connects `experiment_driver` to it over
TraCI, runs the experiment, and cleans SUMO up afterward. Output:
- `results/run_tasks.csv` — one row per completed task (latency, which
  DAG/vehicle it belonged to, etc.)
- `results/run_ticks.csv` — one row per simulation tick (vehicle count,
  per-server queue length and link latency over time)
- a row appended to `results/summary.csv`

Options: `--scheduler {heft,fifo}`, `--offloading {greedy,nearest}`,
`--duration <seconds>`, `--arrival-rate <mean DAGs/sec>`, `--seed <int>`,
`--mock` (skip SUMO, use the synthetic mobility feed instead, for quick
iteration). Run `./experiment_driver --help` for the driver's own flags
if you want to skip the Python wrapper and drive an already-running
`sumo --remote-port N` directly.

## 4. Run the full comparison sweep (what the report plots use)

```
python3 run_experiment.py --sweep --sweep-seeds 1,2,3 --sweep-duration 200 --arrival-rate 1.2
```

Runs all 4 (scheduler x offloading) combinations x 3 seeds = 12 runs,
writing each run's CSVs to `results/sweep/` and one combined
`results/summary.csv`.

## 5. Generate plots

```
python3 plot_results.py
```

Reads `results/sweep/*.csv` and writes PNGs to `results/plots/`:
- `latency_by_config.png` — mean/P95 latency bar charts by configuration
- `latency_cdf.png` — full latency distribution per configuration
- `throughput_over_time.png` — cumulative completed tasks over time
- `mobility_and_congestion.png` — vehicle count, link latency, and queue
  depth over time for one example run (the "proof it's real SUMO" plot)
- `speed_vs_latency.png` — mean network speed vs. link latency, across
  all runs
- `tasks_completed_by_config.png` — total completed tasks per config

## Troubleshooting

- **"Could not connect to SUMO TraCI server"**: something else is
  already using that port, or SUMO crashed on startup — check that
  `sumo_scenario/grid.sumocfg`'s paths resolve relative to wherever you
  run the command from (it expects to be run from the project root).
- **Windows build errors mentioning `socket`/`connect`/`send`**: means
  `-lws2_32` was left off the g++ command.
- Don't add a port-readiness probe of your own before launching
  `experiment_driver` — SUMO's TraCI server accepts exactly one client
  connection, and a probe connection will consume that slot before the
  driver gets to it (this bit us once — see the comment in
  `run_experiment.py::run_one`). The driver's own `TraciClient::connect`
  already retries internally.
