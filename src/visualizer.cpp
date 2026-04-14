#include <cstdio>
#include <iostream>
#include <matplot/matplot.h>
#include <unistd.h>
#include "visualizer.hpp"

namespace EmbeddedNav {

namespace {

class ScopedStderrSilencer {
public:
    ScopedStderrSilencer() {
        fflush(stderr);
        saved_stderr_fd_ = dup(fileno(stderr));
        if (saved_stderr_fd_ == -1) {
            return;
        }
        null_stderr_ = freopen("/dev/null", "w", stderr);
        if (null_stderr_ == nullptr) {
            close(saved_stderr_fd_);
            saved_stderr_fd_ = -1;
        }
    }
    ~ScopedStderrSilencer() {
        if (saved_stderr_fd_ == -1) {
            return;
        }
        fflush(stderr);
        dup2(saved_stderr_fd_, fileno(stderr));
        close(saved_stderr_fd_);
    }
    ScopedStderrSilencer(const ScopedStderrSilencer&) = delete;
    ScopedStderrSilencer& operator=(const ScopedStderrSilencer&) = delete;

private:
    int saved_stderr_fd_ = -1;
    FILE* null_stderr_ = nullptr;
};

void savePlotSilently(const std::string& filename) {
    ScopedStderrSilencer silencer;
    matplot::save(filename);
}

}

Visualizer::Visualizer(const OccupancyGrid& true_grid, 
                const OccupancyGrid& inflated_grid, 
                const std::vector<Waypoint>& planned_path, 
                const Waypoint& start, 
                const Waypoint& goal)
    : true_grid_(true_grid), safe_grid_(inflated_grid), nominal_path_(planned_path), start_(start), goal_(goal)
{}

void Visualizer::plotPathAndGrids() {    
    using namespace matplot;
    ScopedStderrSilencer silencer;

    // Create a figure that is not displayed inherently (true arg)
    // Docker expects a display server which is a pain to get working (so just using a save to png strategy)
    // Wouldn't be an issue in actual deployment on hardware anyway
    auto fig = figure(true);

    const double resolution = true_grid_.resolution();
    const double origin_x = true_grid_.originX();
    const double origin_y = true_grid_.originY();

    hold(on);
    axis(equal);

    // Populate 2D map with obstacle likelihoods (just reshaping row wise single vector to matrix)
    // Will be represented in cell space (not global coordinates)
    for (int row = 0; row < true_grid_.rows(); ++row) {
        for (int column = 0; column < true_grid_.columns(); ++column) {

            const double x = origin_x + column * resolution;
            const double y = origin_y + row * resolution;

            double true_value = true_grid_.getCell(row, column);
            double inflated_value = safe_grid_.getCell(row, column);

            // Unknown cells in the original grid
            if (true_value < 0.0) {
                auto rect = rectangle(x, y, resolution, resolution);
                rect->fill(true);
                rect->color("yellow");
                continue;
            }

            // Show how true obstacles are inflated
            // Only draw gray where the inflated grid has an obstacle but the true grid does not
            if (inflated_value > OBSTACLE_THRESHOLD && true_value <= OBSTACLE_THRESHOLD) {
                auto rect = rectangle(x, y, resolution, resolution);
                rect->fill(true);
                rect->color({0.5, 0.5, 0.5});
            }

            // True obstacles directly from map
            if (true_value > OBSTACLE_THRESHOLD) {
                auto rect = rectangle(x, y, resolution, resolution);
                rect->fill(true);
                rect->color("black");
            }
        }
    }

    // Overlay the cell data and the trajectory
    std::vector<double> waypoints_x, waypoints_y;
    for (const auto& waypoint : nominal_path_) {
        waypoints_x.push_back(waypoint.x);
        waypoints_y.push_back(waypoint.y);
    }
    auto plt = plot(waypoints_x, waypoints_y);

    // Add markers for start and stop to visualization
    auto start_mark = scatter(std::vector<double>{start_.x}, std::vector<double>{start_.y});
    start_mark->marker_style(line_spec::marker_style::square).marker_face_color("g");
    auto goal_mark = scatter(std::vector<double>{goal_.x}, std::vector<double>{goal_.y});
    goal_mark->marker_style(line_spec::marker_style::square).marker_face_color("b");

    // Add conversion ticks (continuous space to cell space)
    std::vector<double> x_ticks;
    for (int column = 0; column <= true_grid_.columns(); column++)
        x_ticks.push_back(origin_x + column * resolution);

    std::vector<double> y_ticks;
    for (int row = 0; row <= true_grid_.rows(); row++)
        y_ticks.push_back(origin_y + row * resolution);

    xticks(x_ticks);
    yticks(y_ticks);
    grid(on); 

    // Save to outputs
    title("Global Path A*");
    xlim({origin_x, origin_x + true_grid_.columns() * resolution});
    ylim({origin_y, origin_y + true_grid_.rows() * resolution});
    save("outputs/trajectory_plot.png");
    std::cout << "Plot saved to outputs/trajectory_plot.png" << std::endl;
}

void Visualizer::plotTracking(const std::vector<Waypoint>& tracked_path) {
    using namespace matplot;
    ScopedStderrSilencer silencer;

    // Create a figure that is not displayed inherently (true arg)
    // Docker expects a display server which is a pain to get working (so just using a save to png strategy)
    // Wouldn't be an issue in actual deployment on hardware anyway
    auto fig = figure(true);

    const double resolution = safe_grid_.resolution();
    const double origin_x = safe_grid_.originX();
    const double origin_y = safe_grid_.originY();

    hold(on);
    axis(equal);
    
    // Draw the grid with inflated obstacles only
    for (int row = 0; row < safe_grid_.rows(); ++row) {
        for (int column = 0; column < safe_grid_.columns(); ++column) {
            if (safe_grid_.isOccupied(row, column)) {
                const double x = origin_x + column * resolution;
                const double y = origin_y + row * resolution;
                auto cell_rectangle = rectangle(x, y, resolution, resolution);
                cell_rectangle->fill(true);
                cell_rectangle->color("black");
            }
        }
    }

    // Convert waypoint structs into x/y arrays for Matplot++
    std::vector<double> planned_x, planned_y;
    for (const auto& waypoint : nominal_path_) {
        planned_x.push_back(waypoint.x);
        planned_y.push_back(waypoint.y);
    }

    std::vector<double> tracked_x, tracked_y;
    for (const auto& waypoint : tracked_path) {
        tracked_x.push_back(waypoint.x);
        tracked_y.push_back(waypoint.y);
    }

    auto planned_plot = plot(planned_x, planned_y);
    planned_plot->line_width(1.5);
    planned_plot->line_style("--");
    planned_plot->display_name("A* path");

    auto tracked_plot = plot(tracked_x, tracked_y);
    tracked_plot->line_width(2.5);
    tracked_plot->line_style("-");
    tracked_plot->display_name("LQR tracked path");

    auto start_mark = scatter(std::vector<double>{start_.x}, std::vector<double>{start_.y});
    start_mark->marker_style(line_spec::marker_style::square);
    start_mark->marker_face_color("g");
    start_mark->display_name("Start");

    auto goal_mark = scatter(std::vector<double>{goal_.x}, std::vector<double>{goal_.y});
    goal_mark->marker_style(line_spec::marker_style::square);
    goal_mark->marker_face_color("b");
    goal_mark->display_name("Goal");

    // Add conversion ticks (continuous space to cell space)
    std::vector<double> x_ticks;
    for (int column = 0; column <= true_grid_.columns(); column++)
        x_ticks.push_back(origin_x + column * resolution);

    std::vector<double> y_ticks;
    for (int row = 0; row <= true_grid_.rows(); row++)
        y_ticks.push_back(origin_y + row * resolution);

    xticks(x_ticks);
    yticks(y_ticks);
    grid(on); 

    // i hate this <- true
    // legend();
    title("A* Path with Noisy Simulation and EKF Tracking");
    xlim({origin_x, origin_x + safe_grid_.columns() * resolution});
    ylim({origin_y, origin_y + safe_grid_.rows() * resolution});
    save("outputs/noisy_lqr_plot.png");
    std::cout << "Plot saved to outputs/noisy_lqr_plot.png" << std::endl;
}

}