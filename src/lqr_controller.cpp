#include "lqr_controller.hpp"

#include <algorithm>
#include <cmath>
#include <random>
#include <stdexcept>
#include <iostream>

namespace EmbeddedNav {


/*

GAUSSIAN NOISE HELPERS

*/
// Simulate the robot's motion with some added Gaussian noise to represent real-world uncertainties (e.g. wheel slippage, uneven terrain)
RobotPose propagateWithNoise(
    const DifferentialDriveLQRController& controller,
    const RobotPose& current,
    const DiffDriveControl& control,
    std::mt19937& gen,
    double pos_std,
    double theta_std) 
{
    RobotPose next = controller.propagate(current, control);

    std::normal_distribution<double> pos_noise(0.0, pos_std);
    std::normal_distribution<double> theta_noise(0.0, theta_std);

    next.x += pos_noise(gen);
    next.y += pos_noise(gen);
    next.theta += theta_noise(gen);

    while (next.theta > PI) next.theta -= 2.0 * PI;
    while (next.theta < -PI) next.theta += 2.0 * PI;

    return next;
}

// Simulate a noisy measurement of the robot's pose (e.g. from a GPS or camera-based localization system)
RobotPose measureWithNoise(const RobotPose& true_state, std::mt19937& gen, double meas_pos_std, double meas_theta_std) {
    std::normal_distribution<double> pos_noise(0.0, meas_pos_std);
    std::normal_distribution<double> theta_noise(0.0, meas_theta_std);
    RobotPose z = true_state;
    z.x += pos_noise(gen);
    z.y += pos_noise(gen);
    z.theta += theta_noise(gen);
    while (z.theta > PI) {
        z.theta -= 2.0 * PI;
    }
    while (z.theta < -PI) {
        z.theta += 2.0 * PI;
    }
    return z;
}


/*

EKF HELPERS

*/
// Extended Kalman Filter for estimating the robot's pose based on noisy measurements and control inputs
PoseEKF::PoseEKF(double dt, const RobotPose& initial_pose, const PoseKalmanConfig& config)
    : dt_(dt), P_(config.P0), Q_(config.Q), R_(config.R) {
    x_hat_ << initial_pose.x, initial_pose.y, initial_pose.theta;
}

// Predict the next state based on the current estimate and control input
void PoseEKF::predict(const DiffDriveControl& control) {
    const double x = x_hat_(0);
    const double y = x_hat_(1);
    const double theta = x_hat_(2);
    const double v = control.v;
    const double omega = control.omega;
    // Nonlinear state prediction
    Eigen::Vector3d x_pred;
    x_pred << x + dt_ * v * std::cos(theta), y + dt_ * v * std::sin(theta), wrapAngle(theta + dt_ * omega);
    // Jacobian of motion model wrt state
    Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
    F(0,2) = -dt_ * v * std::sin(theta);
    F(1,2) =  dt_ * v * std::cos(theta);
    x_hat_ = x_pred;
    P_ = F * P_ * F.transpose() + Q_;
}

// Update the state estimate based on a new measurement
void PoseEKF::update(const RobotPose& measurement) {
    Eigen::Vector3d z;
    z << measurement.x, measurement.y, measurement.theta;

    // Measurement model: z = Hx + noise, with H = I
    Eigen::Matrix3d H = Eigen::Matrix3d::Identity();
    Eigen::Vector3d innovation = z - x_hat_;
    innovation(2) = wrapAngle(innovation(2));
    Eigen::Matrix3d S = H * P_ * H.transpose() + R_;
    Eigen::Matrix3d K = P_ * H.transpose() * S.inverse();
    x_hat_ = x_hat_ + K * innovation;
    x_hat_(2) = wrapAngle(x_hat_(2));
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    P_ = (I - K * H) * P_;
}

// Get the current state estimate as a RobotPose struct
RobotPose PoseEKF::getEstimate() const {
    RobotPose pose;
    pose.x = x_hat_(0);
    pose.y = x_hat_(1);
    pose.theta = x_hat_(2);
    return pose;
}

// Keep angle in [-pi, pi] so angular error stays continuous
double PoseEKF::wrapAngle(double angle) const {
    while (angle > PI) angle -= 2.0 * PI;
    while (angle < -PI) angle += 2.0 * PI;
    return angle;
}


/*

LQR CONTROLLER HELPERS

*/
// LQR controller for tracking a reference trajectory with a differential drive robot
DifferentialDriveLQRController::DifferentialDriveLQRController(double dt, double v_ref)
    : dt_(dt), nominal_v_(v_ref) 
{
    A_.setIdentity();
    B_.setZero();
    Q_.setZero();
    R_.setZero();
    P_.setZero();
    K_.setZero();

    A_ << 1.0, 0.0, 0.0,
          0.0, 1.0, dt_ * nominal_v_,
          0.0, 0.0, 1.0;
    B_ << dt_, 0.0,
          0.0, 0.0,
          0.0, dt_;

    // Q says how much we care about each kind of tracking error
    // Here we care the most about lateral error, then heading error
    Q_ << 15.0, 0.0, 0.0,
          0.0, 50.0, 0.0,
          0.0, 0.0, 20.0;
    // Bigger vals of R makes the controller less aggressive
    R_ << 1.0, 0.0,
          0.0, 1.5;
    
    // Initial solution to DARE is valid around a heading of 0 degrees and at speads around nominal velocity
    // TODO: May need to re-compute gain K for controller if system ever gets too far off of these assumptions since dynamics are inherently nonlinear
    solveDARE();
}

// Solve the discrete algebraic Riccati equation (DARE) iteratively to get our dearly beloved value of K:
// K = (R + B^T P B)^(-1) B^T P A
void DifferentialDriveLQRController::solveDARE() {
    Eigen::Matrix3d P = Q_;
    constexpr int max_iterations = 1000;
    constexpr double tolerance = 1e-8;
    for (int i = 0; i < max_iterations; ++i) {
        Eigen::Matrix2d S = R_ + B_.transpose() * P * B_;
        Eigen::Matrix<double, 2, 3> gain_term = S.inverse() * B_.transpose() * P * A_;
        Eigen::Matrix3d P_next = A_.transpose() * P * A_ - A_.transpose() * P * B_ * gain_term + Q_;

        // Stop once the Riccati matrix stops changing meaningfully
        if ((P_next - P).norm() < tolerance) {
            P = P_next;
            break;
        }
        P = P_next;
    }
    P_ = P;
    K_ = (R_ + B_.transpose() * P_ * B_).inverse() * B_.transpose() * P_ * A_;
}

// Keep angle in [-pi, pi] so angular error stays continuous
double DifferentialDriveLQRController::wrapAngle(double angle) const {
    while (angle > PI) {
        angle -= 2.0 * PI;
    }
    while (angle < -PI) {
        angle += 2.0 * PI;
    }
    return angle;
}

DiffDriveControl DifferentialDriveLQRController::computeControl(const RobotPose& current, const TrajectoryPoint& reference) const {
    const double dx = current.x - reference.position.x;
    const double dy = current.y - reference.position.y;

    // Rotate  position error into the reference frame of the target trajectory point (so heading error is decoupled from position error magnitude) 
    const double c = std::cos(reference.theta);
    const double s = std::sin(reference.theta);
    const double e_x =  c * dx + s * dy;
    const double e_y = -s * dx + c * dy;

    // Heading error between the robot and the reference orientation (also wrapped to avoid discontinuities)
    const double e_theta = wrapAngle(current.theta - reference.theta);

    Eigen::Vector3d error;
    error << e_x, e_y, e_theta;

    // TODO: working on exact error for controller currently (known state)
    // Kalman filter needed to translate this to LQG (i.e. work on estimated error)
    Eigen::Vector2d delta_u = -K_ * error;
    DiffDriveControl control;
    control.v = reference.v_ref + delta_u(0);
    control.omega = delta_u(1);

    // Clamp control to reasonable limits (these would correspond to the robot's actual physical limits in a real system)
    control.v = std::clamp(control.v, 0.0, 1.0);
    control.omega = std::clamp(control.omega, -2.0, 2.0);
    return control;
}

// Simulate the nonlinear differential-drive kinematics
RobotPose DifferentialDriveLQRController::propagate(const RobotPose& current, const DiffDriveControl& control) const {
    RobotPose next;
    next.x = current.x + dt_ * control.v * std::cos(current.theta);
    next.y = current.y + dt_ * control.v * std::sin(current.theta);
    next.theta = wrapAngle(current.theta + dt_ * control.omega);
    return next;
}

// Turn the geometric path into something more controller-friendly by adding heading and nominal speed information to each point.
std::vector<TrajectoryPoint> buildReferenceTrajectory(const std::vector<Waypoint>& dense_path, double nominal_speed) {
    if (dense_path.empty()) {
        return {};
    }
    std::vector<TrajectoryPoint> trajectory;
    trajectory.reserve(dense_path.size());
    for (std::size_t i = 0; i < dense_path.size(); ++i) {
        double theta = 0.0;
        // Estimate heading from the next waypoint direction
        if (i + 1 < dense_path.size()) {
            const double dx = dense_path[i + 1].x - dense_path[i].x;
            const double dy = dense_path[i + 1].y - dense_path[i].y;
            theta = std::atan2(dy, dx);
        } else if (!trajectory.empty()) {
            // For the final point, just reuse the previous heading since there's no next point to estimate from
            theta = trajectory.back().theta;
        }
        double v_ref = nominal_speed;
        if (i == dense_path.size() - 1) {
            v_ref = 0.0;
        }
        // Wrap the waypoint with heading and velocity information
        trajectory.push_back({dense_path[i], theta, v_ref});
    }
    return trajectory;
}

// Closed-loop sim:
// start at the first reference point
// track each reference point one by one
// move to the next point once we're close enough
std::vector<Waypoint> simulateDifferentialDriveTracking(const std::vector<TrajectoryPoint>& reference_trajectory, double dt, int max_steps, double waypoint_tolerance) {
    if (reference_trajectory.empty()) {
        return {};
    }
    const double nominal_v = (reference_trajectory.size() > 1) ? std::max(0.2, reference_trajectory.front().v_ref) : 0.5;
    DifferentialDriveLQRController controller(dt, nominal_v);

    // Random number generator for simulating noise in the system
    std::random_device rd;
    std::mt19937 gen(rd());

    // Initialize the true state of the robot at the first reference point (with some small random offset to make it interesting)
    RobotPose true_state;
    true_state.x = reference_trajectory.front().position.x;
    true_state.y = reference_trajectory.front().position.y;
    true_state.theta = reference_trajectory.front().theta;

    // Kalman filter for estimating the robot's pose based on noisy measurements
    RobotPose initial_measurement = true_state;
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

    PoseEKF ekf(dt, initial_measurement, kf_config);
    RobotPose estimated_state = ekf.getEstimate();
    std::vector<Waypoint> tracked_path;
    tracked_path.push_back({estimated_state.x, estimated_state.y});

    std::size_t target_index = 1;
    int steps = 0;
    // Main simulation loop: keep tracking the current target point until we're close enough, then move on to the next one
    while (target_index < reference_trajectory.size() && steps < max_steps) {
        const TrajectoryPoint& target = reference_trajectory[target_index];
        DiffDriveControl control = controller.computeControl(estimated_state, target);
        // True system evolves with process noise
        true_state = propagateWithNoise(controller, true_state, control, gen, 0.005, 0.002);
        // Sensor gives noisy measurement
        RobotPose measured_state = measureWithNoise(true_state, gen, 0.03, 0.01);
        // EKF
        ekf.predict(control);
        ekf.update(measured_state);
        estimated_state = ekf.getEstimate();
        tracked_path.push_back({true_state.x, true_state.y});
        
        // Once the robot is close enough to the cur target point, move on to the next reference point.
        const double dx = true_state.x - target.position.x;
        const double dy = true_state.y - target.position.y;
        const double distance = std::sqrt(dx * dx + dy * dy);
        if (distance < waypoint_tolerance) {
            ++target_index;
        }
        ++steps;
        // we don't want too many slices, now.....
        if (steps < 20 || steps % 50 == 0) {
            std::cout << "true: " << true_state.x << ", " << true_state.y
                    << " measured: " << measured_state.x << ", " << measured_state.y
                    << " estimated: " << estimated_state.x << ", " << estimated_state.y
                    << std::endl;
        }
    }
    return tracked_path;
}

}