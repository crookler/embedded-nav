#pragma once

#include <eigen3/Eigen/Dense>
#include <vector>

#include "path_planning.hpp"

namespace EmbeddedNav {

constexpr double PI = 3.14159265358979323846;

// Current pose of the robot (lateral and rotation)
struct RobotPose {
    double x{0.0};
    double y{0.0};
    double theta{0.0};
};

// Control command for a differential drive robot (linear and angular velocity)
// Just simplifying assumption that control acts linearly and angularly instead of on each wheel since in simulation
struct DiffDriveControl {
    double v{0.0};
    double omega{0.0};
};

// Wrap angle to [-pi, pi] so heading error stays well behaved
double wrapAngle(double angle);

// The idea is:
// 1. planner gives us dense / smooth waypoints
// 2. we wrap them as a reference trajectory with added ideal state information (which the robot tries to match)
// 3. this controller computes (v, omega) commands to minimize difference between state and reference point
class DifferentialDriveLQRController {
public:
    DifferentialDriveLQRController(double dt, double v_ref);

    // Compute a control command (v, omega) that drives the robot toward the given reference trajectory point
    DiffDriveControl computeControl(const RobotPose& current, const TrajectoryPoint& reference) const;
    // Propagate the robot one timestep forward using the nonlinear differential-drive kinematics
    static RobotPose propagate(const RobotPose& current, const DiffDriveControl& control, const double dt);

private:
    void solveDARE();
    double dt_;
    double nominal_v_;

    // Discrete-time linearized tracking-error model
    // System acts on the error as the state (difference between pose and reference point) rather than the pose directly
    // e_{k+1} = A e_k + B delta_u_k
    Eigen::Matrix3d A_;
    Eigen::Matrix<double, 3, 2> B_;

    Eigen::Matrix3d Q_;
    Eigen::Matrix2d R_;
    Eigen::Matrix3d P_;
    Eigen::Matrix<double, 2, 3> K_;
};

}