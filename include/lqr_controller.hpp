#pragma once

#include <vector>
#include <Eigen/Dense>

#include "path_planning.hpp"

namespace EmbeddedNav {

struct TrajectoryPoint {
    double x{0.0};
    double y{0.0};
    double theta{0.0};
    double v_ref{0.0};
};

struct RobotPose {
    double x{0.0};
    double y{0.0};
    double theta{0.0};
};

struct DiffDriveControl {
    double v{0.0};
    double omega{0.0};
};

// The idea is:
// 1. planner gives us waypoints
// 2. we densify them
// 3. we build a reference trajectory from them
// 4. this controller computes (v, omega) commands to follow it
class DifferentialDriveLQRController {
public:
    DifferentialDriveLQRController(double dt, double v_ref);

    // Compute a control command (v, omega) that drives the robot toward the given reference trajectory point
    DiffDriveControl computeControl(const RobotPose& current, const TrajectoryPoint& reference) const;
    // Propagate the robot one timestep forward using the nonlinear differential-drive kinematics
    RobotPose propagate(const RobotPose& current, const DiffDriveControl& control) const;

private:
    void initializeSystem();
    void solveDARE();

    // Wrap angle to [-pi, pi] so heading error stays well behaved
    double wrapAngle(double angle) const;
    double dt_{0.1};
    double nominal_v_{0.5};

    // Discrete-time linearized tracking-error model:
    // e_{k+1} = A e_k + B delta_u_k
    Eigen::Matrix3d A_;
    Eigen::Matrix<double, 3, 2> B_;

    Eigen::Matrix3d Q_;
    Eigen::Matrix2d R_;
    Eigen::Matrix3d P_;
    Eigen::Matrix<double, 2, 3> K_;
};

// Inserting extra waypoints between planner waypoints gives the controller a denser/smoother reference path to follow
std::vector<Waypoint> densifyPath(const std::vector<Waypoint>& path, double spacing = 0.2);

// Convert a dense geometric path into a trajectory with heading and nominal speed
std::vector<TrajectoryPoint> buildReferenceTrajectory(const std::vector<Waypoint>& dense_path, double nominal_speed = 0.5);

std::vector<Waypoint> simulateDifferentialDriveTracking(
    const std::vector<TrajectoryPoint>& reference_trajectory,
    double dt = 0.1,
    int max_steps = 2000,
    double waypoint_tolerance = 0.15);

}