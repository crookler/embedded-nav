#include "lqr_controller.hpp"

#include <algorithm>
#include <cmath>
#include <random>
#include <stdexcept>
#include <iostream>

namespace EmbeddedNav {

// Keep angle in [-pi, pi] so angular error stays continuous
double wrapAngle(double angle) {
    while (angle > PI) {
        angle -= 2.0 * PI;
    }
    while (angle < -PI) {
        angle += 2.0 * PI;
    }
    return angle;
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

}