#pragma once
#include <algorithm>
#include <limits>
#include "offloading_policy.h"

namespace iov::offloading {

// Concrete, working policy: score each candidate server as
// (queue_length / cpu_capacity) + latency_weight * link_latency_ms,
// pick the minimum. This is the "jointly consider congestion and
// proximity" rule the objective calls for, in its simplest useful form.
class GreedyLatencyLoadPolicy : public IOffloadingPolicy {
public:
    explicit GreedyLatencyLoadPolicy(double latency_weight = 0.5)
        : latency_weight_(latency_weight) {}

    Assignment decide(const iov::scheduling::Task& task,
                    const std::vector<ServerState>& available_servers) override {
        if (available_servers.empty()) return {task.id, std::nullopt};

        double best_score = std::numeric_limits<double>::max();
        uint64_t best_server = 0;
        for (const auto& s : available_servers) {
            double load_term = s.queue_length / std::max(s.cpu_capacity, 0.001);
            double score = load_term + latency_weight_ * s.link_latency_ms;
            if (score < best_score) {
                best_score = score;
                best_server = s.server_id;
            }
        }
        return {task.id, best_server};
    }

private:
    double latency_weight_;
};

}  // namespace iov::offloading
