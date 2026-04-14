#pragma once
#include <string>
#include <vector>

#include "path_planning.hpp"

namespace EmbeddedNav {

// Return type containing grid and start and goal waypoints (in global coordinates not cell indices)
struct MapData {
    OccupancyGrid grid;
    Waypoint start;
    Waypoint goal;
};

// Load the occupancy grid from the .dat file (including metadata)
MapData loadMap(const std::string& path);

// Save the Matplot++ visualization (generated from path_planning) to a png
// void visualizeTrajectory(const OccupancyGrid& true_grid, const OccupancyGrid& inflated_grid, const std::vector<Waypoint>& path, const Waypoint& start, const Waypoint& goal);

// Draw the obstacle grid, the A* path, and the LQR tracked path
void visualizeTrajectory(const OccupancyGrid& grid,
                         const std::vector<Waypoint>& planned_path,
                         const std::vector<Waypoint>& tracked_path,
                         const Waypoint& start,
                         const Waypoint& goal);
}