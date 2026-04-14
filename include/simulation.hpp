#pragma once

#include <random>

#include "lqr_controller.hpp"
#include "ekf.hpp"

namespace EmbeddedNav {

// Result of the tracking simulation containing the true path, noisy measurements, and EKF estimates for visualization
struct TrackingSimulationResult {
    std::vector<Waypoint> true_path;
    std::vector<Waypoint> measured_path;
    std::vector<Waypoint> estimated_path;
};

class DiffDriveSimulator {
public:
    DiffDriveSimulator(const std::vector<TrajectoryPoint>& reference_trajectory,
                        double nominal_v,
                        double dt,
                        PoseKalmanConfig kf_config,
                        int max_steps,
                        double waypoint_tolerance
                        // TODO: Add in a visualizer arg so that each time step of the simulation can be imaged out
                    );
    
    // Main simulation loop
    // Propagate forward robot state based on determined best LQR control along generated reference trajectory
    // Inject artificial noise with smoothing performed by Extended Kalman Filter
    TrackingSimulationResult simulateDifferentialDriveTracking();
private:
    // Advance ground truth based on determined control action
    RobotPose propagateWithNoise(const DiffDriveControl& control);

    // Generate a noisy measurement of the groud truth position with noise
    RobotPose measureWithNoise();

    // Separate principle allows for LQR to act on believed state and for EKF to minimizize error between believed and true state
    // True state still maintained for artificial injection of noise
    // Believed state is used to propagate uncertainty
    std::vector<TrajectoryPoint> reference_trajectory_;
    int max_steps_;
    double dt_, waypoint_tolerance_;
    std::mt19937 generator_; // Noise generator
    RobotPose true_state_; 
    DifferentialDriveLQRController controller_;
    PoseEKF ekf_;
    std::vector<Waypoint> true_path_, measured_path_, estimated_path_;
    double odom_pos_std_{0.005}, odom_theta_std_{0.002}, meas_pos_std_{0.03}, meas_theta_std_{0.01};
};

}