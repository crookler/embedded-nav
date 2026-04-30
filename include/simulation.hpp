#pragma once

#include <random>

#include "lqr_controller.hpp"
#include "ekf.hpp"

namespace EmbeddedNav {

// Struct to represent the robot's pose (x, y, theta)
struct PoseTracePoint {
    double x{0.0};
    double y{0.0};
    double theta{0.0};
};

// Result of the tracking simulation containing the true path, noisy measurements, and EKF estimates for visualization
struct TrackingSimulationResult {
    std::vector<Waypoint> true_path;
    std::vector<Waypoint> measured_path;
    std::vector<Waypoint> estimated_path;
    // Subset of the reference trajectory that was actively being tracked (e.g. if we didn't finish tracking the whole
    // trajectory within max steps, this will be a subset of the full reference trajectory)
    std::vector<TrajectoryPoint> active_reference_trace; 
    std::vector<PoseTracePoint> true_pose_trace;
    std::vector<PoseTracePoint> measured_pose_trace;
    std::vector<PoseTracePoint> estimated_pose_trace;
};

class DiffDriveSimulator {
public:
    DiffDriveSimulator(const std::vector<Waypoint>& dense_path,
                        double nominal_v,
                        double dt,
                        PoseKalmanConfig kf_config,
                        int max_steps,
                        double waypoint_tolerance,
                        double odom_pos_std, 
                        double odom_theta_std, 
                        double meas_pos_std, 
                        double meas_theta_std,
                        bool use_ekf
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
    double odom_pos_std_, odom_theta_std_, meas_pos_std_, meas_theta_std_;
    bool use_ekf_;
    std::vector<TrajectoryPoint> active_reference_trace_;
    std::vector<PoseTracePoint> true_pose_trace_;
    std::vector<PoseTracePoint> measured_pose_trace_;
    std::vector<PoseTracePoint> estimated_pose_trace_;
};

}