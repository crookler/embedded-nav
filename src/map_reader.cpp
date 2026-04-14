#include <fstream>

#include "map_reader.hpp"
#include "path_planning.hpp"

namespace EmbeddedNav {

MapData loadMap(const std::string& path) {
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

}