#pragma once

#include <eigen3/Eigen/Dense>

#include "lqr_controller.hpp"

namespace EmbeddedNav {

struct PoseKalmanConfig {
    // process noise covariance
    Eigen::Matrix3d Q;
    // measurement noise covariance
    Eigen::Matrix3d R;
    // initial covariance
    Eigen::Matrix3d P0;
};

// Extended Kalman Filter Class for estimating the robot's pose based on noisy measurements and control inputs
class PoseEKF {
public:
    PoseEKF(double dt, const RobotPose& initial_pose, const PoseKalmanConfig& config);
    // Predict step (growing noise based on previous uncertainty and noisy odometry)
    void predict(const DiffDriveControl& control);
    // Update step (shrink uncertainty ellipse based on measurement)
    // Assume measurement is given in the form of robot pose in simulation (i.e. no translation needed between measurement and pose)
    void update(const RobotPose& measurement);
    RobotPose getEstimate() const;

private:
    double dt_;
    Eigen::Vector3d x_hat_;
    Eigen::Matrix3d P_;
    Eigen::Matrix3d Q_;
    Eigen::Matrix3d R_;
};

}