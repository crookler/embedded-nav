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

}