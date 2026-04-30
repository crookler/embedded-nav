#include <iostream>
#include <string>

#include "path_planning.hpp"
#include "map_reader.hpp"
#include "lqr_controller.hpp"
#include "visualizer.hpp"
#include "ekf.hpp"
#include "simulation.hpp"

// Underlying uncertainty (what is actually added to the robot pose and measurement)
constexpr double POSITION_ODOM_STD = 1e-2;
constexpr double ANGULAR_ODOM_STD = 1e-2;
constexpr double POSITION_MEAS_STD = 3e-2;
constexpr double ANGULAR_MEAS_STD = 1e-2;

// Kalman added odometry and measurement uncertainty (not exactly equal to simulate unknown)
constexpr double KALMAN_POSITION_ODOM_STD = 1e-3;
constexpr double KALMAN_ANGULAR_ODOM_STD = 5e-3;
constexpr double KALMAN_POSITION_MEAS_STD = 3e-3;
constexpr double KALMAN_ANGULAR_MEAS_STD = 1e-3;
constexpr double INITIAL_ESTIMATE_COVARIANCE = 0.01;

// Control constants
constexpr double DELTA_T = 0.1; // Time step discretization
constexpr int MAX_STEPS = 2000; // Max simulation steps

// Map generation and control constants
constexpr double OBSTACLE_THRESHOLD = 0.2; // Probability of obstacle above which to flag it
constexpr int INFLATION_RADIUS = 1; // How much safety to add to obstacles
constexpr int STEPS_PER_SMOOTHING_SPAN = 20; // Steps used to get clean curvature
constexpr double RESAMPLE_SPACING = 0.1; // Distance between densified waypoints
constexpr double WAYPOINT_TOLERANCE = RESAMPLE_SPACING * 1.2; // How close to be to a waypoint before advancing
constexpr double NOMINAL_VELOCITY = 0.5 * (RESAMPLE_SPACING / DELTA_T); // Need nominal velocity for cart to actually know how to move forward (assume that it takes roughly 2 time steps to get between adjacent waypoints but may change with additions)

int main(int argc, char** argv) {
    using namespace EmbeddedNav;

    // Parse args (need to pass in map file and visualize flag if a png should be made)
    if (argc < 2) {
        std::cout << "Usage: ./nav_stack <map_path> [--visualize]" << std::endl;
        return 1;
    }
    std::string map_path = argv[1];
    bool should_visualize = (argc > 2 && std::string(argv[2]) == "--visualize");
    bool use_ekf = true;
    if (argc > 3 && std::string(argv[3]) == "--no-ekf") {
        use_ekf = false;
    }

    try {
        // Parse the map and pass it to the planner
        MapData map_data = loadMap(map_path, OBSTACLE_THRESHOLD);
        AStarPathPlanner planner(map_data.grid, INFLATION_RADIUS, STEPS_PER_SMOOTHING_SPAN, RESAMPLE_SPACING);

        // Run the path planner and visualize if the flag is present
        // The goal and start is included in the .dat file itself (annoying to make into args)
        // This path is already densified and smoothed from the planner
        auto path_data = planner.planPath(map_data.start, map_data.goal);

        if (path_data.path.empty()) {
            std::cout << "No path found" << std::endl;
            return 1;
        }

        // Assemble needed kalman filter matrices then pass everything to simulator
        PoseKalmanConfig kf_config;
        kf_config.Q = Eigen::Matrix3d::Zero();
        kf_config.Q(0,0) = KALMAN_POSITION_ODOM_STD;
        kf_config.Q(1,1) = KALMAN_POSITION_ODOM_STD;
        kf_config.Q(2,2) = KALMAN_ANGULAR_ODOM_STD;

        kf_config.R = Eigen::Matrix3d::Zero();
        kf_config.R(0,0) = KALMAN_POSITION_MEAS_STD;
        kf_config.R(1,1) = KALMAN_POSITION_MEAS_STD;
        kf_config.R(2,2) = KALMAN_ANGULAR_MEAS_STD;
        kf_config.P0 = INITIAL_ESTIMATE_COVARIANCE * Eigen::Matrix3d::Identity();
        
        // Construct simulator with appropriate constants    
        DiffDriveSimulator ekf_simulator(
            path_data.path,
            NOMINAL_VELOCITY,
            DELTA_T,
            kf_config,
            MAX_STEPS,
            WAYPOINT_TOLERANCE,
            POSITION_ODOM_STD,
            ANGULAR_ODOM_STD,
            POSITION_MEAS_STD,
            ANGULAR_MEAS_STD,
            true
        );
        // Also run a no EKF version for comparison in visualization
        DiffDriveSimulator no_ekf_simulator(
            path_data.path,
            NOMINAL_VELOCITY,
            DELTA_T,
            kf_config,
            MAX_STEPS,
            WAYPOINT_TOLERANCE,
            POSITION_ODOM_STD,
            ANGULAR_ODOM_STD,
            POSITION_MEAS_STD,
            ANGULAR_MEAS_STD,
            false
        );
        
        TrackingSimulationResult no_ekf_result = no_ekf_simulator.simulateDifferentialDriveTracking();
        TrackingSimulationResult ekf_result = ekf_simulator.simulateDifferentialDriveTracking();

        // Run simulation
        TrackingSimulationResult tracking_result = ekf_simulator.simulateDifferentialDriveTracking();
        std::cout << "planned waypoints: " << path_data.path.size() << std::endl;
        std::cout << "tracked samples: " << tracking_result.true_path.size() << std::endl;

        if (should_visualize) {
            Visualizer visualizer(map_data.grid, path_data.safe_grid, path_data.path, map_data.start, map_data.goal, OBSTACLE_THRESHOLD);
            visualizer.plotPathAndGrids();
            visualizer.plotTracking(no_ekf_result.true_path);
            visualizer.plotTrackingComparison(ekf_result.true_path, ekf_result.measured_path, ekf_result.estimated_path);
            visualizer.plotErrorMetrics(ekf_result, DELTA_T);
        }

    } catch (const std::exception& e) {
        std::cerr << "Load / planning error " << e.what() << std::endl;
        return 1;
    }

    return 0;
}