#include <zmq.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "aethersat/sim/spacecraft_sim.hpp"

namespace {

std::atomic<bool> g_running{true};

void handle_signal(int /*sig*/) {
    g_running.store(false);
}

void print_usage(const char* prog) {
    std::cout << "Usage: " << prog << " [options]\n"
              << "  --pub-endpoint <endpoint>  ZMQ PUB telemetry endpoint (default: tcp://*:5555)\n"
              << "  --cmd-endpoint <endpoint>  ZMQ REP telecommand endpoint (default: tcp://*:5556)\n"
              << "  --rate-hz <hz>             Simulation tick rate in Hz (default: 50)\n"
              << "  --time-scale <scale>       Simulation time-scale multiplier (default: 1.0)\n"
              << "  --help                     Show this message\n";
}

}  // namespace

int main(int argc, char* argv[]) {
    std::string pub_endpoint = "tcp://*:5555";
    std::string cmd_endpoint = "tcp://*:5556";
    double rate_hz = 50.0;
    double time_scale = 1.0;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--pub-endpoint" && i + 1 < argc) {
            pub_endpoint = argv[++i];
        } else if (arg == "--cmd-endpoint" && i + 1 < argc) {
            cmd_endpoint = argv[++i];
        } else if (arg == "--rate-hz" && i + 1 < argc) {
            rate_hz = std::stod(argv[++i]);
        } else if (arg == "--time-scale" && i + 1 < argc) {
            time_scale = std::stod(argv[++i]);
        } else if (arg == "--help" || arg == "-h") {
            print_usage(argv[0]);
            return 0;
        } else {
            std::cerr << "Unknown or incomplete argument: " << arg << "\n";
            print_usage(argv[0]);
            return 1;
        }
    }

    if (rate_hz <= 0.0 || time_scale <= 0.0) {
        std::cerr << "--rate-hz and --time-scale must be positive.\n";
        return 1;
    }

    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    try {
        aethersat::sim::ZmqContext ctx;
        aethersat::sim::ZmqSocket pub_sock(ctx, ZMQ_PUB);
        aethersat::sim::ZmqSocket rep_sock(ctx, ZMQ_REP);

        pub_sock.bind(pub_endpoint);
        rep_sock.bind(cmd_endpoint);

        aethersat::sim::SpacecraftSim sim;

        std::cout << "[aethersat_sim] Started | PUB=" << pub_endpoint
                  << " | CMD=" << cmd_endpoint
                  << " | rate=" << rate_hz << " Hz"
                  << " | time_scale=" << time_scale << "x" << std::endl;

        const double wall_dt_s = 1.0 / rate_hz;
        const double sim_dt_s = time_scale / rate_hz;
        const auto period = std::chrono::duration<double>(wall_dt_s);

        std::vector<uint8_t> tc_buf;
        while (g_running.load()) {
            const auto tick_start = std::chrono::steady_clock::now();

            // Drain any pending telecommands without blocking the simulation loop
            while (rep_sock.recv(tc_buf, ZMQ_DONTWAIT)) {
                const auto ack = sim.handle_telecommand(tc_buf);
                rep_sock.send(ack, 0);
            }

            // Advance physics & virtual buses
            sim.step(sim_dt_s);

            // Stream CCSDS Telemetry packets
            pub_sock.send(sim.build_adcs_orbit_tm(), ZMQ_DONTWAIT);
            pub_sock.send(sim.build_eps_health_tm(), ZMQ_DONTWAIT);

            const auto elapsed = std::chrono::steady_clock::now() - tick_start;
            if (elapsed < period) {
                std::this_thread::sleep_for(period - elapsed);
            }
        }

        std::cout << "[aethersat_sim] Shutting down cleanly." << std::endl;
    } catch (const std::exception& ex) {
        std::cerr << "[aethersat_sim] Fatal error: " << ex.what() << std::endl;
        return 1;
    }

    return 0;
}
