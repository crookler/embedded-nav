#include <random>
#include <iostream>

#include "simulation.hpp"

namespace EmbeddedNav {

// Creates internal EKF and LQR that are guranteed to share the same timestep
DiffDriveSimulator::DiffDriveSimulator(const std::vector<Waypoint>& dense_path,
                        double nominal_v,
                        double dt,
                        PoseKalmanConfig kf_config,
                        int max_steps,
                        double waypoint_tolerance,
                        double odom_pos_std, 
                        double odom_theta_std, 
                        double meas_pos_std, 
                        double meas_theta_std,
                        bool use_ekf)
    : reference_trajectory_(buildReferenceTrajectory(dense_path, nominal_v)), use_ekf_(use_ekf), dt_(dt), controller_(dt, nominal_v), ekf_(dt, true_state_, kf_config),
    max_steps_(max_steps), waypoint_tolerance_(waypoint_tolerance), generator_(std::random_device{}()), odom_pos_std_(odom_pos_std),
    odom_theta_std_(odom_theta_std), meas_pos_std_(meas_pos_std), meas_theta_std_(meas_theta_std),
    true_state_([&]{
          RobotPose p;
          p.x = reference_trajectory_.front().position.x;
          p.y = reference_trajectory_.front().position.y;
          p.theta = reference_trajectory_.front().theta;
          return p;
      }())
{
    // Assume in simulation that robot starts out with perfect knowledge of where it begins (no kidnapped robot issues)
    true_path_.push_back({true_state_.x, true_state_.y});
    measured_path_.push_back({true_state_.x, true_state_.y});
    estimated_path_.push_back({true_state_.x, true_state_.y});
}


// Simulate the robot's motion with some added Gaussian noise to represent real-world uncertainties (e.g. wheel slippage, uneven terrain)
RobotPose DiffDriveSimulator::propagateWithNoise(const DiffDriveControl& control) {
    // Propagate the true state based on the determined control action
    // Add odometry noise based on passed in standard deviations for position and heading
    // Assume zero center Gaussian noise
    // Figure out the exact update then add additional noise according to constant odometry standard deviations
    RobotPose next = controller_.propagate(true_state_, control, dt_);
    std::normal_distribution<double> pos_noise(0.0, odom_pos_std_);
    std::normal_distribution<double> theta_noise(0.0, odom_theta_std_);
    next.x += pos_noise(generator_);
    next.y += pos_noise(generator_);
    next.theta += theta_noise(generator_);
    next.theta = wrapAngle(next.theta);
    return next;
}

// Simulate a noisy measurement of the robot's pose (e.g. from a GPS or camera-based localization system)
RobotPose DiffDriveSimulator::measureWithNoise() {
    // Again assume zero centered noise
    std::normal_distribution<double> pos_noise(0.0, meas_pos_std_);
    std::normal_distribution<double> theta_noise(0.0, meas_theta_std_);
    RobotPose z = true_state_;
    z.x += pos_noise(generator_);
    z.y += pos_noise(generator_);
    z.theta += theta_noise(generator_);
    z.theta = wrapAngle(z.theta);
    return z;
}

// Closed-loop sim:
// start at the first reference point
// track each reference point one by one
// move to the next point once we're close enough
TrackingSimulationResult DiffDriveSimulator::simulateDifferentialDriveTracking() {
    std::size_t target_index = 1;
    int steps = 0;
    RobotPose estimated_state = ekf_.getEstimate(); // Initial estimate (currently defined as perfect knowledge)
    RobotPose measured_state = estimated_state;

    // Main simulation loop that keeps tracking the current target point until we're close enough, then moves on to the next one
    while (target_index < reference_trajectory_.size() && steps < max_steps_) {
        const TrajectoryPoint& target = reference_trajectory_[target_index];
        active_reference_trace_.push_back(target);

        // Compute the control action to take based on the current believed state (either EKF estimate or noisy measurement) and the current target point
        RobotPose feedback_state = use_ekf_ ? estimated_state : measured_state;
        DiffDriveControl control = controller_.computeControl(feedback_state, target);

        // True system evolves with process noise (update the true_state based on the control just computed for the current target)
        true_state_ = propagateWithNoise(control);
        
        // Determine what the measured state is (will be compared to the currently believed state by the Kalman filter to shrink uncertainty)
        measured_state = measureWithNoise();
        
        // Kalman predict and update steps using noisy odometry and measurement
        ekf_.predict(control);
        ekf_.update(measured_state);
        estimated_state = ekf_.getEstimate(); // Pull out the new current EKF estimation to use in logging (will also be used in next iteration)
        
        // Log the true path, measurements, and EKF estimates for visualization
        true_path_.push_back({true_state_.x, true_state_.y});
        measured_path_.push_back({measured_state.x, measured_state.y});
        estimated_path_.push_back({estimated_state.x, estimated_state.y});

        // Also log the full pose (x, y, theta) for error metric visualization later on
        true_pose_trace_.push_back({true_state_.x, true_state_.y, true_state_.theta});
        measured_pose_trace_.push_back({measured_state.x, measured_state.y, measured_state.theta});
        estimated_pose_trace_.push_back({estimated_state.x, estimated_state.y, estimated_state.theta});
        
        // Once the robot is close enough to the cur target point, move on to the next reference point.
        const double dx = true_state_.x - target.position.x;
        const double dy = true_state_.y - target.position.y;
        const double distance = std::sqrt(dx * dx + dy * dy);
        const auto& p1 = reference_trajectory_[target_index - 1].position;
        const auto& p2 = target.position;

        // Vector from current waypoint to next waypoint
        double dx_path = p2.x - p1.x;
        double dy_path = p2.y - p1.y;

        // Vector from current waypoint to robot
        double dx_rob = true_state_.x - p1.x;
        double dy_rob = true_state_.y - p1.y;

        // Update progress using dot product or raw distance (giving additional flexibility in case of overshoot)
        double progress = (dx_rob * dx_path + dy_rob * dy_path) / (dx_path * dx_path + dy_path * dy_path);
        if (distance < waypoint_tolerance_ || progress > 1.0) {
            target_index++;
        }
        steps++;
    }

    TrackingSimulationResult result;
    result.true_path = std::move(true_path_);
    result.measured_path = std::move(measured_path_);
    result.estimated_path = std::move(estimated_path_);
    result.active_reference_trace = std::move(active_reference_trace_);
    result.true_pose_trace = std::move(true_pose_trace_);
    result.measured_pose_trace = std::move(measured_pose_trace_);
    result.estimated_pose_trace = std::move(estimated_pose_trace_);
    return result;
}

}