#include "sumo_bridge.h"
#include <cmath>

namespace iov::simulation {

SumoBridge::SumoBridge(const std::string& host, int port) {
    client_.connect(host, port);
}

SumoBridge::~SumoBridge() {
    client_.close();
}

void SumoBridge::step() {
    client_.simulationStep();
    auto samples = client_.getAllVehicleData();

    cache_.clear();
    cache_.reserve(samples.size());
    for (const auto& s : samples) {
        cache_[s.id] = VehiclePose{s.id, s.x, s.y, s.speed};
    }
}

std::unordered_map<std::string, VehiclePose> SumoBridge::vehiclePositions() const {
    return cache_;
}

double SumoBridge::estimateLatencyMs(const std::string& vehicle_id,
                                    double server_x, double server_y) const {
    auto it = cache_.find(vehicle_id);
    if (it == cache_.end()) return 1e9;  // vehicle not present -> unreachable
    double dx = it->second.x - server_x;
    double dy = it->second.y - server_y;
    double dist = std::sqrt(dx * dx + dy * dy);
    return kBaseLatencyMs + dist * kPerMeterMs;
}

}  // namespace iov::simulation
