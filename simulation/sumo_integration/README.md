# SUMO / TraCI Simulation & Evaluation Layer

**Owner:** Member 4

## What this module owns

- Driving the SUMO mobility simulation over **TraCI** (SUMO's TCP
  remote-control protocol), via a small hand-rolled C++ client
  (`traci_client.h`) — **not libsumo**. Earlier notes here said libsumo;
  that was the original plan, but linking libsumo's C++ library turned
  out to be a build-environment headache (no prebuilt binary dev
  package, only the Python wheel), so this uses the same wire protocol
  the official Python client uses instead. It needs nothing but the
  `sumo` binary itself — see `TRACI_PROTOCOL_NOTES.md` for the
  byte-verified protocol decode.
- Reading live vehicle positions each simulation step and computing
  vehicle-to-edge-server proximity / link latency
- Feeding that latency data into `offloading/`'s `ServerState`
- Collecting evaluation metrics (per-task end-to-end latency, queue
  depth, throughput) into `results/`

## Interface contract

```
sumo process (via TraCI) --> vehicle positions --> latency model --> ServerState.link_latency_ms
                                                                              |
                                                                              v
                                                                      (Offloading Engine)
```

This module is the one place that talks to SUMO directly — nothing else
in the repo should touch `traci_client.h` or the TraCI socket directly.

## Status

- [x] `IMobilityProvider` interface — shared by the mock feed and the
      real bridge
- [x] `MockMobilityProvider` — synthetic vehicles on circular paths,
      kept around for fast iteration / `--mock` runs without SUMO
- [x] SUMO network/route files (`sumo_scenario/grid.net.xml`,
      `routes.rou.xml`, `grid.sumocfg`) — a 4x4 grid, 200 routed
      vehicles, verified against real `sumo`
- [x] `traci_client.h` — raw-socket TraCI client, byte-verified against
      SUMO 1.27.1 (see `TRACI_PROTOCOL_NOTES.md`)
- [x] `SumoBridge` — real `IMobilityProvider` backed by `traci_client.h`
- [x] Baseline policies for comparison: `FifoScheduler`
      (`src/scheduling/fifo_scheduler.h`), `NearestServerPolicy`
      (`src/offloading/nearest_server_policy.h`)
- [x] `main.cpp` rewritten as a continuous-arrival experiment driver
      (random fork-join DAGs, CLI-selectable scheduler/offloading
      policy, per-task + per-tick CSV output)
- [x] `run_experiment.py` — launches SUMO on a free port, runs the
      driver against it, cleans up; `--sweep` runs the full
      scheduler x offloading x seed comparison used for the plots
- [x] `plot_results.py` — generates the comparison plots into
      `results/plots/`

Nothing left unchecked from the original scope. Two things worth
flagging honestly rather than hiding:

- At the arrival rate used for the report plots (1.2 DAGs/s, 2 edge
  servers), **offloading policy** (greedy load+latency vs. naive
  nearest-server) makes a clear, consistent difference in task latency;
  **scheduler choice** (HEFT-lite vs FIFO dispatch order) does not,
  because with only 2 servers there's rarely real dispatch-order
  contention to resolve. Push `--arrival-rate` higher or drop to 1
  server if you want to see the scheduler matter too.
- The latency model (`SumoBridge::estimateLatencyMs`) is a documented
  analytic model (fixed overhead + distance-scaled term) on top of real
  SUMO trajectories, not a full radio/backhaul simulator — standard
  practice for this kind of evaluation, but worth stating plainly at
  the mid-eval rather than implying it's a measured channel.

## What to present at mid-eval

- Walk through `traci_client.h` + `TRACI_PROTOCOL_NOTES.md`: this is a
  from-scratch protocol implementation, byte-verified against the
  official client — a defensible, explainable design decision, not a
  black box.
- Show `results/plots/mobility_and_congestion.png`: real vehicle count
  ramping up as SUMO trips depart, real fluctuating link latency, real
  (if modest, at this load) queueing — proof the whole pipeline is
  actually driven by SUMO, not synthetic data.
- Show `results/plots/latency_cdf.png` and `latency_by_config.png` for
  the actual comparison result: load-aware offloading beats
  nearest-server across the whole latency distribution, not just on
  average.
- `results/summary.csv` + per-run CSVs in `results/sweep/` as the raw
  backing data.

## Running it

See the top-level `HOW_TO_RUN.md` for the full build + run instructions.

## Setup notes

Requires the `sumo` binary on `PATH`, or `SUMO_HOME` set, or the
`eclipse-sumo` Python wheel installed (`pip install eclipse-sumo`) —
`run_experiment.py` checks all three, in that order of preference via
`SUMO_HOME`. No libsumo linking needed.
