#pragma once
#include <string>
#include "mobility_provider.h"
#include "traci_client.h"

namespace iov::simulation {

// Real SUMO backend. Connects over TraCI (SUMO's TCP remote-control
// protocol — see traci_client.h) to an already-running `sumo` process,
// so it needs no libsumo linking, just the `sumo` binary. Launching
// that process is the experiment runner's job (run_experiment.py), not
// this class's — see README.md for why.
//
// step() advances SUMO by one simulation step and refreshes the cached
// per-vehicle positions/speeds in a single batched round trip.
// vehiclePositions() then just returns that cache (no I/O), matching
// the const contract of IMobilityProvider.
class SumoBridge : public IMobilityProvider {
public:
    // Connects to a `sumo --remote-port <port>` instance already
    // listening on host:port. Throws TraciClientError if it can't
    // connect after retrying.
    SumoBridge(const std::string& host, int port);
    ~SumoBridge() override;

    void step() override;
    std::unordered_map<std::string, VehiclePose> vehiclePositions() const override;

    // Distance-based latency model applied on top of *real* SUMO
    // trajectories: base_ms accounts for fixed processing/propagation
    // overhead, per_meter_ms models distance-dependent radio/backhaul
    // delay. This is a documented analytic model, not a full network
    // simulator — see README.md "Latency model" section for the
    // rationale (this is standard practice in vehicular-edge papers:
    // real mobility + analytic channel model).
    double estimateLatencyMs(const std::string& vehicle_id,
                            double server_x, double server_y) const override;

private:
    TraciClient client_;
    std::unordered_map<std::string, VehiclePose> cache_;

    static constexpr double kBaseLatencyMs = 2.0;
    static constexpr double kPerMeterMs = 0.05;
};

}  // namespace iov::simulation
