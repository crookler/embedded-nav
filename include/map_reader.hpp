#pragma once
#include <string>
#include <vector>

#include "path_planning.hpp"

namespace EmbeddedNav {

// Return type containing grid and start and goal waypoints (in global coordinates not cell indices)
struct MapPackage {
    OccupancyGrid grid;
    Waypoint start;
    Waypoint goal;
};

// Load the occupancy grid from the .dat file (including metadata)
MapPackage loadMap(const std::string& path);

// Save the Matplot++ visualization (generated from path_planning) to a png
void visualizeTrajectory(const OccupancyGrid& grid, const std::vector<Waypoint>& path, const Waypoint& start, const Waypoint& goal);

}