#include <iostream>
#include <string>

#include "path_planning.hpp"
#include "map_reader.hpp"
#include "lqr_controller.hpp"
#include "visualizer.hpp"
#include "ekf.hpp"
#include "simulation.hpp"

int main(int argc, char** argv) {
    using namespace EmbeddedNav;

    // Parse args (need to pass in map file and visualize flag if a png should be made)
    if (argc < 2) {
        std::cout << "Usage: ./nav_stack <map_path> [--visualize]" << std::endl;
        return 1;
    }
    std::string map_path = argv[1];
    bool should_visualize = (argc > 2 && std::string(argv[2]) == "--visualize");

    try {
        // Parse the map and pass it to the planner
        MapData map_data = loadMap(map_path);
        AStarPathPlanner planner(map_data.grid);

        // Run the path planner and visualize if the flag is present
        // The goal and start is included in the .dat file itself (annoying to make into args)
        // This path is already densified and smoothed from the planner
        auto path_data = planner.planPath(map_data.start, map_data.goal);

        if (path_data.path.empty()) {
            std::cout << "No path found" << std::endl;
            return 1;
        }

        // Turn the planner output into an LQR reference trajectory
        auto reference_trajectory = buildReferenceTrajectory(path_data.path, 0.6);

        // Construct simulator
        PoseKalmanConfig kf_config;
        kf_config.Q = Eigen::Matrix3d::Zero();
        kf_config.Q(0,0) = 1e-4;
        kf_config.Q(1,1) = 1e-4;
        kf_config.Q(2,2) = 1e-5;
        kf_config.R = Eigen::Matrix3d::Zero();

        // ~5cm std if units are meters
        kf_config.R(0,0) = 2.5e-3;
        kf_config.R(1,1) = 2.5e-3;
        kf_config.R(2,2) = 1e-3;
        kf_config.P0 = 0.01 * Eigen::Matrix3d::Identity();

        double nominal_v = (reference_trajectory.size() > 1) ? std::max(0.2, reference_trajectory.front().v_ref) : 0.5;
        double dt = 0.1;
        int max_steps = 2500;
        double waypoint_tolerance = 0.12;
        DiffDriveSimulator simulator(reference_trajectory, nominal_v, dt, kf_config, max_steps, waypoint_tolerance);

        // Run simulation
        TrackingSimulationResult tracking_result = simulator.simulateDifferentialDriveTracking();
        std::cout << "planned waypoints: " << path_data.path.size() << std::endl;
        std::cout << "tracked samples: " << tracking_result.true_path.size() << std::endl;

        if (should_visualize) {
            Visualizer visualizer(map_data.grid, path_data.safe_grid, path_data.path, map_data.start, map_data.goal);
            visualizer.plotPathAndGrids();
            visualizer.plotTracking(tracking_result.true_path);
            visualizer.plotTrackingComparison(tracking_result.true_path, tracking_result.measured_path, tracking_result.estimated_path);
        }

    } catch (const std::exception& e) {
        std::cerr << "Load / planning error " << e.what() << std::endl;
        return 1;
    }

    return 0;
}