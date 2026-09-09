// Experiment driver for the IoV edge-scheduling pipeline.
//
// Unlike the old main.cpp (a single fixed 5-task DAG run once against a
// mock mobility feed), this continuously spawns random fork-join DAGs
// over the course of the run, each "owned" by a real (or mock) vehicle,
// and records per-task and per-tick metrics to CSV so different
// scheduler/offloading-policy/mobility combinations can be compared.
//
// See simulation/sumo_integration/README.md and run_experiment.py for
// how this gets wired up to a real running SUMO instance.
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <random>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "src/scheduling/task_scheduler.h"
#include "src/scheduling/heft_lite_scheduler.h"
#include "src/scheduling/fifo_scheduler.h"
#include "src/offloading/offloading_policy.h"
#include "src/offloading/greedy_latency_load_policy.h"
#include "src/offloading/nearest_server_policy.h"
#include "src/resource_mgmt/fifo_resource_allocator.h"
#include "simulation/sumo_integration/mobility_provider.h"
#include "simulation/sumo_integration/mobility_mock.h"
#include "simulation/sumo_integration/sumo_bridge.h"

using namespace iov;

namespace {

struct Config {
    std::string host = "127.0.0.1";
    int port = -1;
    bool use_mock = false;
    int mock_num_vehicles = 6;

    std::string scheduler_name = "heft";      // heft | fifo
    std::string offloading_name = "greedy";   // greedy | nearest

    double duration_s = 200.0;
    double step_length_s = 0.1;   // must match the .sumocfg's --step-length
    double arrival_rate_hz = 0.5; // mean new DAGs per second
    double latency_weight = 0.3;
    unsigned seed = 42;

    std::string out_prefix = "results/run";
    std::string label;  // free-text tag written into the summary row
};

Config parseArgs(int argc, char** argv) {
    Config c;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&]() -> std::string {
            if (i + 1 >= argc) throw std::runtime_error("missing value for " + a);
            return argv[++i];
        };
        if (a == "--host") c.host = next();
        else if (a == "--port") c.port = std::stoi(next());
        else if (a == "--mock") c.use_mock = true;
        else if (a == "--num-vehicles") c.mock_num_vehicles = std::stoi(next());
        else if (a == "--scheduler") c.scheduler_name = next();
        else if (a == "--offloading") c.offloading_name = next();
        else if (a == "--duration") c.duration_s = std::stod(next());
        else if (a == "--step-length") c.step_length_s = std::stod(next());
        else if (a == "--arrival-rate") c.arrival_rate_hz = std::stod(next());
        else if (a == "--latency-weight") c.latency_weight = std::stod(next());
        else if (a == "--seed") c.seed = static_cast<unsigned>(std::stoul(next()));
        else if (a == "--out-prefix") c.out_prefix = next();
        else if (a == "--label") c.label = next();
        else if (a == "--help") {
            std::cout <<
                "Usage: experiment_driver [--mock | --port N] [options]\n"
                "  --host H              TraCI host (default 127.0.0.1)\n"
                "  --port N              TraCI port of an already-running `sumo --remote-port N`\n"
                "  --mock                use the synthetic mobility feed instead of SUMO\n"
                "  --num-vehicles N      vehicle count for --mock (default 6)\n"
                "  --scheduler S         heft | fifo (default heft)\n"
                "  --offloading S        greedy | nearest (default greedy)\n"
                "  --duration S          simulated seconds to run (default 200)\n"
                "  --step-length S       seconds per tick, match .sumocfg (default 0.1)\n"
                "  --arrival-rate R      mean new DAGs per second (default 0.5)\n"
                "  --latency-weight W    GreedyLatencyLoadPolicy weight (default 0.3)\n"
                "  --seed N              RNG seed (default 42)\n"
                "  --out-prefix P        output file prefix (default results/run)\n"
                "  --label L             free-text tag stored in the summary row\n";
            std::exit(0);
        } else {
            throw std::runtime_error("unknown argument: " + a);
        }
    }
    if (!c.use_mock && c.port < 0)
        throw std::runtime_error("must pass either --mock or --port <traci-port>");
    return c;
}

// One randomly generated fork-join DAG, tagged with a global id range so
// it can be dispatched through the shared allocator alongside every
// other in-flight DAG.
struct ActiveDag {
    int dag_id;
    std::vector<scheduling::Task> tasks;               // Task.id / dependencies are GLOBAL ids
    std::unordered_map<uint64_t, const scheduling::Task*> by_id;
    std::string owner_vehicle;
    double arrival_time_s;
    std::unordered_set<uint64_t> completed;
    std::unordered_set<uint64_t> dispatched;
    std::unordered_map<uint64_t, double> dispatch_time_s;
    std::unordered_map<uint64_t, double> completion_time_s;
    std::unordered_map<uint64_t, double> last_latency_ms;  // per server_id, last known-good
    std::unique_ptr<scheduling::ITaskScheduler> scheduler;

    bool finished() const { return completed.size() == tasks.size(); }

    // Called once after tasks is fully populated and will not move
    // again, so pointers into it stay valid for the DAG's lifetime.
    void rebuildIndex() {
        by_id.clear();
        for (const auto& t : tasks) by_id[t.id] = &t;
    }
};

std::unique_ptr<scheduling::ITaskScheduler> makeScheduler(const std::string& name) {
    if (name == "heft") return std::make_unique<scheduling::HeftLiteScheduler>();
    if (name == "fifo") return std::make_unique<scheduling::FifoScheduler>();
    throw std::runtime_error("unknown --scheduler: " + name);
}

std::unique_ptr<offloading::IOffloadingPolicy> makeOffloadingPolicy(const std::string& name,
                                                                     double latency_weight) {
    if (name == "greedy")
        return std::make_unique<offloading::GreedyLatencyLoadPolicy>(latency_weight);
    if (name == "nearest") return std::make_unique<offloading::NearestServerPolicy>();
    throw std::runtime_error("unknown --offloading: " + name);
}

// Generates a random fork-join DAG. Each task (other than the root)
// depends on 1-2 earlier tasks in the DAG, so the dependency graph is
// valid by construction. Global ids are allocated from next_global_id.
ActiveDag makeRandomDag(int dag_id, uint64_t& next_global_id, const std::string& owner_vehicle,
                         double arrival_time_s, const std::string& scheduler_name,
                         std::mt19937& rng) {
    std::uniform_int_distribution<int> n_tasks_dist(3, 6);
    std::uniform_real_distribution<double> workload_dist(20.0, 80.0);
    std::uniform_real_distribution<double> data_size_dist(50.0, 300.0);
    std::uniform_int_distribution<int> n_deps_dist(1, 2);

    int n = n_tasks_dist(rng);
    uint64_t base = next_global_id;
    next_global_id += static_cast<uint64_t>(n);

    ActiveDag dag;
    dag.dag_id = dag_id;
    dag.owner_vehicle = owner_vehicle;
    dag.arrival_time_s = arrival_time_s;
    dag.scheduler = makeScheduler(scheduler_name);
    dag.tasks.reserve(n);

    for (int i = 0; i < n; ++i) {
        scheduling::Task t;
        t.id = base + static_cast<uint64_t>(i);
        t.estimated_workload = workload_dist(rng);
        t.data_size = data_size_dist(rng);
        if (i > 0) {
            int n_deps = std::min(n_deps_dist(rng), i);
            std::vector<int> pool(i);
            for (int k = 0; k < i; ++k) pool[k] = k;
            std::shuffle(pool.begin(), pool.end(), rng);
            for (int k = 0; k < n_deps; ++k)
                t.dependencies.push_back(base + static_cast<uint64_t>(pool[k]));
        }
        dag.tasks.push_back(t);
    }
    dag.rebuildIndex();
    return dag;
}

struct ServerInfo {
    resource_mgmt::ServerSpec spec;
    double x, y;
};

}  // namespace

int main(int argc, char** argv) {
    Config cfg;
    try {
        cfg = parseArgs(argc, argv);
    } catch (const std::exception& e) {
        std::cerr << "Argument error: " << e.what() << "\n";
        return 1;
    }

    std::unique_ptr<simulation::IMobilityProvider> mobility;
    if (cfg.use_mock) {
        mobility = std::make_unique<simulation::MockMobilityProvider>(cfg.mock_num_vehicles);
        std::cout << "Using MockMobilityProvider (" << cfg.mock_num_vehicles << " vehicles)\n";
    } else {
        try {
            mobility = std::make_unique<simulation::SumoBridge>(cfg.host, cfg.port);
        } catch (const std::exception& e) {
            std::cerr << "Failed to connect to SUMO at " << cfg.host << ":" << cfg.port
                      << " -- " << e.what() << "\n";
            return 1;
        }
        std::cout << "Connected to SUMO via TraCI at " << cfg.host << ":" << cfg.port << "\n";
    }

    // Two heterogeneous edge servers placed at opposite corners of the
    // 4x4 grid network (0..600 in both x and y for grid.net.xml).
    std::vector<ServerInfo> servers = {
        {{100, 1.0}, 100.0, 100.0},  // slower, near one corner
        {{101, 2.5}, 500.0, 500.0},  // faster, near the opposite corner
    };
    std::vector<resource_mgmt::ServerSpec> specs;
    for (const auto& s : servers) specs.push_back(s.spec);

    resource_mgmt::FifoResourceAllocator allocator(specs);
    auto offloading_policy = makeOffloadingPolicy(cfg.offloading_name, cfg.latency_weight);

    std::mt19937 rng(cfg.seed);
    std::uniform_real_distribution<double> uni01(0.0, 1.0);

    std::vector<ActiveDag> active_dags;
    std::unordered_map<uint64_t, int> task_owner_dag;  // global task id -> owning dag_id
    std::unordered_map<int, size_t> dag_id_to_index;    // dag_id -> current index in active_dags

    uint64_t next_global_id = 0;
    int next_dag_id = 0;
    int total_dags_spawned = 0;
    uint64_t total_tasks_dispatched = 0;
    uint64_t total_tasks_completed = 0;

    std::ofstream tasks_csv(cfg.out_prefix + "_tasks.csv");
    tasks_csv << "task_id,dag_id,owner_vehicle,arrival_time_s,dispatch_time_s,"
                 "completion_time_s,e2e_latency_s,workload\n";

    std::ofstream ticks_csv(cfg.out_prefix + "_ticks.csv");
    ticks_csv << "time_s,num_vehicles,mean_speed_mps,active_dags,"
                 "server_100_queue,server_100_latency_ms,"
                 "server_101_queue,server_101_latency_ms,"
                 "cum_dispatched,cum_completed\n";

    int n_steps = static_cast<int>(cfg.duration_s / cfg.step_length_s);

    for (int step_idx = 0; step_idx < n_steps; ++step_idx) {
        double sim_time_s = step_idx * cfg.step_length_s;

        mobility->step();
        auto vpos = mobility->vehiclePositions();

        // --- maybe spawn a new DAG ---
        double spawn_prob = cfg.arrival_rate_hz * cfg.step_length_s;
        if (!vpos.empty() && uni01(rng) < spawn_prob) {
            std::vector<std::string> ids;
            ids.reserve(vpos.size());
            for (const auto& kv : vpos) ids.push_back(kv.first);
            std::uniform_int_distribution<size_t> pick(0, ids.size() - 1);
            std::string owner = ids[pick(rng)];

            ActiveDag dag = makeRandomDag(next_dag_id, next_global_id, owner, sim_time_s,
                                           cfg.scheduler_name, rng);
            for (const auto& t : dag.tasks) task_owner_dag[t.id] = next_dag_id;
            active_dags.push_back(std::move(dag));
            active_dags.back().rebuildIndex();  // pointers must target the final vector element
            dag_id_to_index[next_dag_id] = active_dags.size() - 1;
            ++next_dag_id;
            ++total_dags_spawned;
        }

        // --- dispatch ready tasks for every active DAG ---
        for (auto& dag : active_dags) {
            std::vector<uint64_t> ready;
            for (const auto& t : dag.tasks) {
                if (dag.dispatched.count(t.id)) continue;
                bool deps_done = true;
                for (auto dep : t.dependencies)
                    if (!dag.completed.count(dep)) { deps_done = false; break; }
                if (deps_done) ready.push_back(t.id);
            }
            if (ready.empty()) continue;

            auto ordered = dag.scheduler->orderReadyTasks(dag.tasks, ready);

            for (auto task_id : ordered) {
                const scheduling::Task& task = *dag.by_id.at(task_id);

                // Refresh per-server link latency from this DAG's owner
                // vehicle's current position before deciding.
                for (const auto& s : servers) {
                    double lat = mobility->estimateLatencyMs(dag.owner_vehicle, s.x, s.y);
                    if (lat >= 1e9 && dag.last_latency_ms.count(s.spec.server_id)) {
                        lat = dag.last_latency_ms[s.spec.server_id];  // vehicle left; use last-known
                    } else if (lat < 1e9) {
                        dag.last_latency_ms[s.spec.server_id] = lat;
                    }
                    allocator.setLinkLatency(s.spec.server_id, lat);
                }

                auto server_state = allocator.currentState();
                auto assignment = offloading_policy->decide(task, server_state);
                if (!assignment.server_id.has_value()) continue;  // no server available this tick

                allocator.admit(*assignment.server_id, {task.id, task.estimated_workload});
                dag.dispatched.insert(task.id);
                dag.dispatch_time_s[task.id] = sim_time_s;
                ++total_tasks_dispatched;
            }
        }

        // --- advance server execution ---
        auto completed_now = allocator.tick(cfg.step_length_s * 1000.0);
        for (auto task_id : completed_now) {
            auto owner_it = task_owner_dag.find(task_id);
            if (owner_it == task_owner_dag.end()) continue;
            auto idx_it = dag_id_to_index.find(owner_it->second);
            if (idx_it == dag_id_to_index.end()) continue;
            ActiveDag& dag = active_dags[idx_it->second];

            dag.completed.insert(task_id);
            dag.completion_time_s[task_id] = sim_time_s;
            ++total_tasks_completed;

            const scheduling::Task& task = *dag.by_id.at(task_id);
            double arrival = dag.arrival_time_s;
            double completion = sim_time_s;
            tasks_csv << task_id << "," << dag.dag_id << "," << dag.owner_vehicle << ","
                      << arrival << "," << dag.dispatch_time_s[task_id] << "," << completion << ","
                      << (completion - arrival) << "," << task.estimated_workload << "\n";
        }

        // --- drop finished DAGs ---
        for (auto it = active_dags.begin(); it != active_dags.end();) {
            if (it->finished()) {
                dag_id_to_index.erase(it->dag_id);
                it = active_dags.erase(it);
                for (size_t i = 0; i < active_dags.size(); ++i)
                    dag_id_to_index[active_dags[i].dag_id] = i;
                for (size_t i = 0; i < active_dags.size(); ++i)
                    active_dags[i].rebuildIndex();  // erase() may have relocated elements
            } else {
                ++it;
            }
        }

        // --- per-tick metrics ---
        double mean_speed = 0.0;
        for (const auto& kv : vpos) mean_speed += kv.second.speed;
        if (!vpos.empty()) mean_speed /= static_cast<double>(vpos.size());

        auto state = allocator.currentState();
        std::unordered_map<uint64_t, offloading::ServerState> state_by_id;
        for (const auto& s : state) state_by_id[s.server_id] = s;

        ticks_csv << sim_time_s << "," << vpos.size() << "," << mean_speed << ","
                  << active_dags.size() << "," << state_by_id[100].queue_length << ","
                  << state_by_id[100].link_latency_ms << "," << state_by_id[101].queue_length << ","
                  << state_by_id[101].link_latency_ms << "," << total_tasks_dispatched << ","
                  << total_tasks_completed << "\n";
    }

    tasks_csv.close();
    ticks_csv.close();

    // --- summary row (appended so a sweep of runs accumulates) ---
    std::string summary_path = "results/summary.csv";
    bool exists = static_cast<bool>(std::ifstream(summary_path));
    std::ofstream summary(summary_path, std::ios::app);
    if (!exists) {
        summary << "label,scheduler,offloading,mobility,seed,duration_s,arrival_rate_hz,"
                   "dags_spawned,tasks_dispatched,tasks_completed,completion_ratio\n";
    }
    double completion_ratio = total_tasks_dispatched > 0
        ? static_cast<double>(total_tasks_completed) / static_cast<double>(total_tasks_dispatched)
        : 0.0;
    summary << (cfg.label.empty() ? (cfg.scheduler_name + "_" + cfg.offloading_name) : cfg.label)
            << "," << cfg.scheduler_name << "," << cfg.offloading_name << ","
            << (cfg.use_mock ? "mock" : "sumo") << "," << cfg.seed << "," << cfg.duration_s << ","
            << cfg.arrival_rate_hz << "," << total_dags_spawned << "," << total_tasks_dispatched
            << "," << total_tasks_completed << "," << completion_ratio << "\n";
    summary.close();

    std::cout << "=== Run complete ===\n"
              << "DAGs spawned:      " << total_dags_spawned << "\n"
              << "Tasks dispatched:  " << total_tasks_dispatched << "\n"
              << "Tasks completed:   " << total_tasks_completed << "\n"
              << "Wrote " << cfg.out_prefix << "_tasks.csv, " << cfg.out_prefix
              << "_ticks.csv, and appended a row to " << summary_path << "\n";

    return 0;
}
