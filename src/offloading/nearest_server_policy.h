#pragma once
#include <limits>
#include "offloading_policy.h"

namespace iov::offloading {

// Baseline policy for comparison: always offload to whichever server
// currently has the lowest link latency, ignoring queue congestion
// entirely. This is the naive alternative GreedyLatencyLoadPolicy
// (which jointly considers load and latency) is meant to beat —
// expected to look good at low load and degrade as queues build up on
// whichever server happens to be closest.
class NearestServerPolicy : public IOffloadingPolicy {
public:
    Assignment decide(const iov::scheduling::Task& task,
                       const std::vector<ServerState>& available_servers) override {
        if (available_servers.empty()) return {task.id, std::nullopt};

        double best_latency = std::numeric_limits<double>::max();
        uint64_t best_server = 0;
        for (const auto& s : available_servers) {
            if (s.link_latency_ms < best_latency) {
                best_latency = s.link_latency_ms;
                best_server = s.server_id;
            }
        }
        return {task.id, best_server};
    }
};

}  // namespace iov::offloading
