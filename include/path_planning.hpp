#pragma once
#include <vector>

namespace EmbeddedNav {

// This is defined in continuous space
// Robot can be in multiple positions per cell (i.e. robot does not just snap between grid cells)
// Way points mark where the robot should navigate to within each cell (and special cases for the starting and stopping pose)
struct Waypoint {
    double x{0.0};
    double y{0.0};
    bool operator==(const Waypoint& other) const { return x==other.x && y==other.y; }
    bool operator!=(const Waypoint& other) const { return !(*this==other); }
};

// Occupancy grid is composed of cells only (with internal resolution converting between world size and grid size)
struct Cell {
    int row{0};
    int column{0};
    bool operator==(const Cell& other) const { return row==other.row && column==other.column; }
    bool operator!=(const Cell& other) const { return !(*this==other); }
};

struct AStarNode {
    double estimated_cost; // Estimated cost of path that passes through this node (including actual cost and heuristic)
    Cell cell;
    bool operator>(const AStarNode& other) const { return estimated_cost> other.estimated_cost; } // Sort A* nodes basd on estimated cost in min heap
};

constexpr double OBSTACLE_THRESHOLD = 0.2;

class OccupancyGrid {
public:
    OccupancyGrid() = default;
    // Will get these fields (stored internally) from .dat file (example from other class in maps)
    // Might make more since to just path source path for .dat file instead (right now assuming this will be decoded by main somewhere)
    OccupancyGrid(int rows, int columns, double resolution, double origin_x, double origin_y, const std::vector<double>& cells);

    // Cells have a value of -1 if unknown and 0 < value < 1 for the probability of being occupied
    // Probability needs to be smoothed by some threshold to determining binary existence of obstacle
    void setCell(int row, int column, double value);
    double getCell(int row, int column) const;
    bool inBounds(int row, int column) const;
    bool isOccupied(int row, int column) const;

    // Convert between grid space and world space
    // Inflation adds extra space around obstacles for safety
    Cell worldToCell(double world_x, double world_y) const;
    Waypoint cellToWorld(const Cell& cell) const;
    OccupancyGrid inflateObstacles(int radius) const;

    // Getters for grid fields (for conversion and encoding)
    // Inlined
    int rows() const { return rows_; }
    int columns() const { return columns_; }
    double resolution() const { return resolution_; }
    double originX() const { return origin_x_; }
    double originY() const { return origin_y_; }

private:
    int rows_{0}, columns_{0};
    double resolution_{0.05}, origin_x_{0.0}, origin_y_{0.0};
    std::vector<double> cells_;
};

// Wrap the returned path and the grid it operated on
struct PlanningData {
    std::vector<Waypoint> path;
    OccupancyGrid safe_grid;
};

// A* path planner
// Could be others like RRT* if we want
// If others, may make sense to just add a path planning function parameter (and just have a planner calls that calls underlying implementation)
class AStarPathPlanner {
public:
    // Pass in the generated grid directly (generate it in main)
    explicit AStarPathPlanner(const OccupancyGrid& grid);

    // Path planner works on a private occupancy grid with private obstacle inflation
    // Once nominal path of grid is constructed and smooted it is then resampled at a desired spacing (creating uniform waypoints)
    void setGrid(const OccupancyGrid& grid);
    void setInflation(const int radius);

    // Main algorithm for planning the path
    // Includes smoothing and densification to make waypoints less jarring and more natural
    PlanningData planPath(const Waypoint& start, const Waypoint& goal);

private:
    // Going with octile heuristic for A* (allowing movement diagonally with higher weighting 1 vs 1.4)
    double octileHeuristic(const Cell& a, const Cell& b) const;

    // Matlab likes B-spline smoothing so use that on the minimal set of control points
    // Resample at specified density (under the hood call to below)
    std::vector<Waypoint> smoothPath(const std::vector<Waypoint>& path) const;

    // Take in path and resample along the path using resample_spacing_
    std::vector<Waypoint> densifyPath(const std::vector<Waypoint>& path) const;

    OccupancyGrid grid_;
    int inflation_radius_{1}, steps_per_span_{20};
    double resample_spacing_{0.1};
};

} 
