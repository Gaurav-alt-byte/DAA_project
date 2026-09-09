#pragma once
#include <algorithm>
#include "task_scheduler.h"

namespace iov::scheduling {

// Baseline policy for comparison: dispatch ready tasks in plain
// ascending-id order (i.e. the order they were created in), ignoring
// the DAG's dependency structure entirely beyond "is it ready". This is
// the naive alternative HeftLiteScheduler's critical-path-first
// ordering is meant to beat.
class FifoScheduler : public ITaskScheduler {
public:
    std::vector<uint64_t> orderReadyTasks(
        const std::vector<Task>& /*all_tasks*/,
        const std::vector<uint64_t>& ready_task_ids) override {
        std::vector<uint64_t> ordered = ready_task_ids;
        std::sort(ordered.begin(), ordered.end());
        return ordered;
    }
};

}  // namespace iov::scheduling
