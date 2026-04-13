#include <fstream>
#include <iostream>
#include <matplot/matplot.h>

#include "map_reader.hpp"
#include "path_planning.hpp"

namespace EmbeddedNav {

MapPackage loadMap(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("File not found: " + path);
    }

    // Not all of these have to be specified (some defaults are okey but it'd be good if they were all there)
    std::string line;
    Waypoint start{0,0}, goal{0,0};
    int rows = 0, columns = 0;
    double resolution = 1.0, origin_x = 0.0, origin_y = 0.0;

    // Parse the map metadata header
    while (std::getline(file, line)) {
        if (line.find("resolution:") != std::string::npos) {
            sscanf(line.c_str(), "resolution: %lf", &resolution);
        } else if (line.find("start_point:") != std::string::npos) {
            sscanf(line.c_str(), "start_point: %lf %lf", &start.x, &start.y);
        } else if (line.find("goal_point:") != std::string::npos) {
            sscanf(line.c_str(), "goal_point: %lf %lf", &goal.x, &goal.y);
        } else if (line.find("origin:") != std::string::npos) {
            sscanf(line.c_str(), "origin: %lf %lf", &origin_x, &origin_y);
        } else if (line.find("global_map:") != std::string::npos) {
            sscanf(line.c_str(), "global_map: %d %d", &rows, &columns);
            break;
        }
    }

    // Parse out the actual cell values and populate the map
    // The top of the map is read first so need to make sure it is populated in reverse order
    std::vector<double> cells(rows * columns);
    for (int file_row = 0; file_row < rows; file_row++) {
        for (int column = 0; column < columns; column++) {
            // Read value and flip row index
            double value;
            file >> value;
            int grid_row = rows - 1 - file_row;
            cells[grid_row * columns + column] = value;
        }
    }

    // Checks for consistency (would also be caught by occupancy grid)
    if (rows <= 0 || columns <= 0) {
        throw std::runtime_error("Invalid map dimensions in file");
    }
    if (static_cast<int>(cells.size()) != rows * columns) {
        throw std::runtime_error("Map cell count mismatch in file");
    }
    return {OccupancyGrid(rows, columns, resolution, origin_x, origin_y, cells), start, goal};
}

// void visualizeTrajectory(const OccupancyGrid& grid,
//                          const std::vector<Waypoint>& planned_path,
//                          const std::vector<Waypoint>& tracked_path,
//                          const Waypoint& start,
//                          const Waypoint& goal) {
void visualizeTrajectory(const OccupancyGrid& grid, const std::vector<Waypoint>& path, const Waypoint& start, const Waypoint& goal) {
    using namespace matplot;

    // Create a figure that is not displayed inherently (true arg)
    // Docker expects a display server which is a pain to get working (so just using a save to png strategy)
    // Wouldn't be an issue in actual deployment on hardware anyway
    auto fig = figure(true);

    const double resolution = grid.resolution();
    const double origin_x = grid.originX();
    const double origin_y = grid.originY();

    hold(on);
    axis(equal);

    // Populate 2D map with obstacle likelihoods (just reshaping row wise single vector to matrix)
    // Will be represented in cell space (not global coordinates)
    for (int row = 0; row < grid.rows(); ++row) {
        for (int column = 0; column < grid.columns(); ++column) {
            const double x = origin_x + column * resolution;
            const double y = origin_y + row * resolution;
            auto cell_rectangle = rectangle(x, y, resolution, resolution);
            if (grid.getCell(row, column) >= OBSTACLE_THRESHOLD) {
                cell_rectangle->fill(true);
                cell_rectangle->color("black");
            }
        }
    }

    // Overlay the cell data and the trajectory
    std::vector<double> waypoints_x, waypoints_y;
    for (const auto& waypoint : path) {
        waypoints_x.push_back(waypoint.x);
        waypoints_y.push_back(waypoint.y);
    }
    auto plt = plot(waypoints_x, waypoints_y);


    // std::vector<double> planned_x, planned_y;
    // for (const auto& waypoint : planned_path) {
    //     planned_x.push_back(waypoint.x);
    //     planned_y.push_back(waypoint.y);
    // }

    // std::vector<double> tracked_x, tracked_y;
    // for (const auto& waypoint : tracked_path) {
    //     tracked_x.push_back(waypoint.x);
    //     tracked_y.push_back(waypoint.y);
    // }

    // auto planned_plot = plot(planned_x, planned_y);
    // planned_plot->display_name("A* planned path!");

    // auto tracked_plot = plot(tracked_x, tracked_y);
    // tracked_plot->display_name("LQR tracked path!");

    // Add markers for start and stop to visualization
    auto start_mark = scatter(std::vector<double>{start.x}, std::vector<double>{start.y});
    start_mark->marker_style(line_spec::marker_style::square).marker_face_color("g");
    auto goal_mark = scatter(std::vector<double>{goal.x}, std::vector<double>{goal.y});
    goal_mark->marker_style(line_spec::marker_style::square).marker_face_color("b");

    // Save to outputs
    title("Global Path A*");
    save("outputs/trajectory_plot.png");
    std::cout << "Plot saved to trajectory_plot.png" << std::endl;
}

}