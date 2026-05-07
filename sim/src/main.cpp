#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <memory>
#include <string>
#include <cmath>

// Tier 2 graph loader + factory.
#include "../../shared/core/block_factory.hpp"
#include "../../shared/core/graph.hpp"
#include "../../shared/core/graph_loader.hpp"

// Block types referenced after load (for runtime config + telemetry readout).
#include "../../shared/blocks/altitude_hold.hpp"
#include "../../shared/blocks/attitude_controller.hpp"
#include "../../shared/blocks/quadrotor_ekf.hpp"
#include "../../shared/blocks/unit_delay.hpp"
#include "../../shared/data/gnss_origin.hpp"
#include "../../shared/types/state.h"
#include "blocks/sensors_impl.hpp"
#include "blocks/signal_generator.hpp"
#include "quadcopter/dynamics.hpp"
#include "quadcopter/registrations.hpp"

namespace {

// Look up a block by name and dynamic_cast it to the expected concrete
// type. The graph spec promises this name maps to this type — if it
// doesn't, the user has shipped a misconfigured JSON and we abort with
// a clear message rather than dereference a null.
template<typename T>
T& require_block(const shared::Graph& g, const std::string& name) {
    shared::Block* b = g.find(name);
    if (!b) {
        std::cerr << "fatal: block '" << name << "' not found in graph\n";
        std::exit(1);
    }
    T* typed = dynamic_cast<T*>(b);
    if (!typed) {
        std::cerr << "fatal: block '" << name
                  << "' is not of the expected type\n";
        std::exit(1);
    }
    return *typed;
}

std::string read_file(const std::string& path) {
    std::ifstream in(path);
    if (!in) {
        std::cerr << "fatal: cannot open graph spec '" << path << "'\n";
        std::exit(1);
    }
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

} // namespace

int main(int argc, char** argv) {
    using shared::TrueState;
    using shared::NavigationState;
    using shared::Vec3;
    using shared::Quat;
    using shared::AttitudeReference;

    const std::string graph_path = (argc > 1) ? argv[1]
                                              : "configs/quad.json";

    std::cout << "=== Quadrotor Closed-Loop Sim (graph: "
              << graph_path << ") ===\n\n";

    // --- Build factory + load graph -----------------------------------
    shared::BlockFactory factory;
    sim::register_quadrotor_blocks(factory);

    shared::Graph graph;
    shared::load_graph_json(read_file(graph_path), factory, graph);

    // --- Runtime config (state that isn't graph topology) -------------
    // Initial pose: 1 m above ground, hovering. Identity attitude.
    TrueState init;
    init.position         = Vec3(0.0, 0.0, -1.0);
    init.velocity         = Vec3::Zero();
    init.attitude         = Quat::Identity();
    init.angular_velocity = Vec3::Zero();

    auto& dynamics = require_block<sim::quadcopter::QuadrotorDynamics>(graph, "dynamics");
    dynamics.reset(init);

    // Seed the unit-delay on the dynamics→sensor back-edge so tick 0 sensors
    // see the init pose, not a default-constructed TrueState.
    auto& truth_delay = require_block<shared::UnitDelayBlock<TrueState>>(graph, "truth_delay");
    truth_delay.set_held(init);

    auto& ekf = require_block<shared::QuadrotorEkfBlock>(graph, "ekf");
    ekf.initialize(init);

    auto& alt_hold = require_block<shared::AltitudeHoldBlock<NavigationState>>(graph, "alt_hold");
    alt_hold.set_setpoint_m(ekf.origin().alt_m - init.position.z());

    auto& controller = require_block<shared::AttitudePidController>(graph, "att_ctrl");
    AttitudeReference ref;
    ref.set_roll(0.0);
    ref.set_pitch(0.0);
    ref.set_yaw(0.0);
    controller.reference_input().set(ref);

    // --- Topo order (cached once; graph isn't mutated after load) -----
    auto order = graph.topo_order();

    // --- CSV output ---------------------------------------------------
    std::ofstream csv("impulse_response.csv");
    csv << "t_s,roll_deg,pitch_deg,yaw_deg,roll_rate_dps,pitch_rate_dps,pos_z_m,disturbance_nm,"
           "ekf_roll_deg,ekf_pitch_deg,ekf_yaw_deg,ekf_pos_z_m\n";
    csv << std::fixed << std::setprecision(6);

    auto& dist_gen = require_block<sim::WaveformGenerator>(graph, "dist_gen");

    // --- Run loop: 2 minutes, 1 ms steps ------------------------------
    constexpr uint64_t DT_US  = 1000;
    constexpr uint64_t END_US = 120000000;

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "  t(s)   roll(deg)  pitch(deg)  pos_z(m)  dist(N·m)\n";
    std::cout << "-----------------------------------------------------------\n";

    for (uint64_t t = 0; t < END_US; t += DT_US) {
        for (auto* b : order) {
            if (b->is_due(t)) b->update(t);
        }

        const auto& s     = dynamics.output().get();
        Vec3 euler        = s.euler_angles();
        Vec3 omega_dps    = s.angular_velocity * (180.0 / M_PI);
        double dist_nm    = dist_gen.value();

        const auto& ekf_est = ekf.output().get();
        Vec3 ekf_euler      = ekf_est.euler_angles();

        csv << (t / 1e6)
            << "," << (euler.x() * 180.0 / M_PI)
            << "," << (euler.y() * 180.0 / M_PI)
            << "," << (euler.z() * 180.0 / M_PI)
            << "," << omega_dps.x()
            << "," << omega_dps.y()
            << "," << s.position.z()
            << "," << dist_nm
            << "," << (ekf_euler.x() * 180.0 / M_PI)
            << "," << (ekf_euler.y() * 180.0 / M_PI)
            << "," << (ekf_euler.z() * 180.0 / M_PI)
            << "," << ekf_est.position.z()
            << "\n";

        if (t % 1000000 == 0) {
            std::cout << std::setw(6)  << (t / 1e6)
                      << "   roll="  << std::setw(7) << (euler.x() * 180.0 / M_PI)
                      << " pitch=" << std::setw(7) << (euler.y() * 180.0 / M_PI)
                      << " pos_z=" << std::setw(7) << s.position.z()
                      << " dist="  << std::setw(6) << dist_nm
                      << "\n";
        }
    }

    csv.close();
    std::cout << "\nWrote impulse_response.csv\n";

    return 0;
}
