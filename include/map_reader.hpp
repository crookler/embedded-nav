#pragma once
#include <string>

#include "path_planning.hpp"

namespace EmbeddedNav {

// Return type containing grid and start and goal waypoints (in global coordinates not cell indices)
struct MapData {
    OccupancyGrid grid;
    Waypoint start;
    Waypoint goal;
};

// Load the occupancy grid from the .dat file (including metadata)
MapData loadMap(const std::string& path, const double obstacle_threshold);

}