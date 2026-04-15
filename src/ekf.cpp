#include "ekf.hpp"

namespace EmbeddedNav {

// Extended Kalman Filter for estimating the robot's pose based on noisy measurements and control inputs
PoseEKF::PoseEKF(double dt, const RobotPose& initial_pose, const PoseKalmanConfig& config)
    : dt_(dt), P_(config.P0), Q_(config.Q), R_(config.R) 
{
    x_hat_ << initial_pose.x, initial_pose.y, initial_pose.theta;
}

// Predict the next state based on the current estimate and control input
void PoseEKF::predict(const DiffDriveControl& control) {
    const double x = x_hat_(0);
    const double y = x_hat_(1);
    const double theta = x_hat_(2);
    
    // Update estimation by taking a single step according to exact dynamics (not Jacobian for expectation)
    Eigen::Vector3d x_pred;
    RobotPose propagation = DifferentialDriveLQRController::propagate({x, y, theta}, control, dt_); // EKF and LQR guranteed to share same dt_ since both in simulator
    x_pred << propagation.x, propagation.y, propagation.theta;
    x_hat_ = x_pred;

    // Jacobian of above motion model with respect to state
    // System is nonlinear and a need a linearized state update matrix F
    const double v = control.v; // Linear velocity
    const double omega = control.omega; // Angular velocity
    Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
    F(0,2) = -dt_ * v * std::sin(theta); // Update to x wrt theta
    F(1,2) =  dt_ * v * std::cos(theta); // Update to y wrt theta
    P_ = F * P_ * F.transpose() + Q_; // Assume constant odometry noise from diagonal Q
}

// Update the state estimate based on a new measurement
void PoseEKF::update(const RobotPose& measurement) {
    // Assembly measurement vector
    Eigen::Vector3d z;
    z << measurement.x, measurement.y, measurement.theta;

    // Measurement model for simulation is z = h(x) + noise with h(ground_truth) = ground_truth
    // This essentially is just ground truth state perturbed by a single sorce of Gaussian noise
    // Measurement function is just identity since measurement function just returns ground truth pose (pose wrt pose is identity)
    Eigen::Matrix3d H = Eigen::Matrix3d::Identity();
    
    // Measurement residual
    Eigen::Vector3d innovation = z - x_hat_;
    innovation(2) = wrapAngle(innovation(2));
    
    // Residual covariance with constant measurement noise from diagonal R
    Eigen::Matrix3d S = H * P_ * H.transpose() + R_;

    // Near optimal Kalman gain
    Eigen::Matrix3d K = P_ * H.transpose() * S.inverse();

    // Update state expectation based on weighting between odometry and measurement confidence
    x_hat_ = x_hat_ + K * innovation;
    x_hat_(2) = wrapAngle(x_hat_(2));

    // Update covariance estimate
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

}