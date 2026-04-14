#include <queue>
#include <unordered_map>
#include <cmath>
#include <algorithm>
#include <stdexcept>

#include "path_planning.hpp"

namespace EmbeddedNav {

OccupancyGrid::OccupancyGrid(int rows, int columns, double resolution, double origin_x, double origin_y, const std::vector<double>& cells)
    : rows_(rows), columns_(columns), resolution_(resolution), origin_x_(origin_x), origin_y_(origin_y) 
{
    if (static_cast<int>(cells.size()) != rows_ * columns_)
        throw std::invalid_argument("OccupancyGrid size mismatch betweens cells and reported size");
    cells_ = cells;
}

void OccupancyGrid::setCell(int row, int column, double value) {
    if (inBounds(row, column)) {
        cells_[static_cast<size_t>(row * columns_ + column)] = value;
    }
}

double OccupancyGrid::getCell(int row, int column) const {
    if (!inBounds(row, column)) {
        return -1.0;
    } else {
        return cells_[static_cast<size_t>(row * columns_ + column)];
    }
}

bool OccupancyGrid::inBounds(int row, int column) const {
    return row >= 0 && row < rows_ && column >= 0 && column < columns_;
}

bool OccupancyGrid::isOccupied(int row, int column) const {
    double value = getCell(row, column);
    return (value > OBSTACLE_THRESHOLD) || (value < 0.0);
}

Cell OccupancyGrid::worldToCell(double world_x, double world_y) const {
    return { static_cast<int>((world_y - origin_y_) / resolution_), static_cast<int>((world_x - origin_x_) / resolution_) };
}

Waypoint OccupancyGrid::cellToWorld(const Cell& cell) const {
    return { origin_x_ + (cell.column + 0.5) * resolution_, origin_y_ + (cell.row + 0.5) * resolution_ };
}

OccupancyGrid OccupancyGrid::inflateObstacles(int radius) const {
    // Take the internally stored occupancy grid and make a copy that has all obstacles expanded by a given radius
    OccupancyGrid expanded_grid(rows_, columns_, resolution_, origin_x_, origin_y_, cells_);
    for (int row = 0; row < rows_; row++) {
        for (int column = 0; column < columns_; column++) {
            // If cell is occupied in the original map then set surrounding cells as occupied in outgoing grid
            if (isOccupied(row, column)) {
                for (int row_change = -radius; row_change <= radius; row_change++) {
                    for (int column_change = -radius; column_change <= radius; column_change++) {
                        // Check squared radius for current indices
                        if (row_change*row_change + column_change*column_change <= radius*radius) {
                            expanded_grid.setCell(row+row_change, column+column_change, std::max(expanded_grid.getCell(row+row_change, column+column_change), getCell(row, column))); // Pass max probability of other true obstacles around here (should be smooted anyway but useful if this information needs to be used)
                        }
                    }
                }
            }
        }
    }
    return expanded_grid;
}

AStarPathPlanner::AStarPathPlanner(const OccupancyGrid& grid)
    : grid_(grid)
{}

void AStarPathPlanner::setGrid(const OccupancyGrid& grid) { 
    grid_ = grid; 
}

void AStarPathPlanner::setInflation(const int radius) { 
    inflation_radius_ = radius; 
}

double AStarPathPlanner::octileHeuristic(const Cell& a, const Cell& b) const {
    double row_change = std::abs(a.row-b.row);
    double column_change = std::abs(a.column-b.column);
    // Add weights from cardinal moves but regulate them if a diagonal is taken (i.e. subtract wasted weight)
    return 10.0*(row_change+column_change) + (14.0-20.0)*std::min(row_change,column_change); 
}

// Format row_change, column_change, movement_cost
// Weight diagonals 1:sqrt(2) higher than cardinal directions (10 vs 14)
static const int CellOffsetWeights[8][3] = {
    {-1,  0, 10}, { 1,  0, 10}, { 0, -1, 10}, { 0,  1, 10}, 
    {-1, -1, 14}, {-1,  1, 14}, { 1, -1, 14}, { 1,  1, 14}  
};

// Main A* implementation
// Return a vector of waypoints that can then be the optimal trajectory for LQR controller to attempt to match
PlanningData AStarPathPlanner::planPath(const Waypoint& start, const Waypoint& goal) {
    OccupancyGrid safe_grid = grid_.inflateObstacles(inflation_radius_);
    Cell start_cell = safe_grid.worldToCell(start.x, start.y);
    Cell goal_cell = safe_grid.worldToCell(goal.x,  goal.y);

    // Check to make sure that starting and ending cells are valid (i.e. known and not in obstacle)
    if (safe_grid.isOccupied(start_cell.row, start_cell.column) || safe_grid.isOccupied(goal_cell.row, goal_cell.column)) {
        return {std::vector<Waypoint>(), safe_grid};
    }
    
    // Encode each cell with absolute index (counting along rows)
    // Used as primary key for costs and parent map
    int total_columns = safe_grid.columns(); 
    auto encodeCell = [&](const Cell& cell) -> int64_t {
        return static_cast<int64_t>(cell.row) * total_columns + cell.column;
    };
    auto decodeCell = [&](int64_t key) -> Cell {
        return { static_cast<int>(key / total_columns), static_cast<int>(key % total_columns) };
    };

    // Create open set min heap, running cost (updated as cost improves), and parent map (updated as cost improves) 
    // Create the starting cell and push it on to the open set for evaluation 
    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_set;
    std::unordered_map<int64_t, double> cost_from_start; 
    std::unordered_map<int64_t, int64_t> parent_map;
    int64_t start_key = encodeCell(start_cell);
    cost_from_start[start_key] = 0.0;
    open_set.push({octileHeuristic(start_cell, goal_cell), start_cell}); // Cost to go through start cell is just heuristic since actual start cost is 0

    // Iterate until open set is empty (no more neighbors to process)
    // Break early on first path found (i.e. key matches goal)
    while (!open_set.empty()) {
        // Estimated cost only used in sorting
        auto [current_estimated_cost, current_cell] = open_set.top(); 
        open_set.pop();

        // Check if cell popped from priority queue is the goal 
        // If it is return this first path that made it to the goal
        if (current_cell == goal_cell) {
            // Push back the end and then start reverse tree crawl
            std::vector<Waypoint> optimal_path;
            optimal_path.push_back(goal);
            int64_t current_key = encodeCell(goal_cell);
            current_key = parent_map.at(current_key); 
            
            // Walk backward from goal until starting following parent map
            // Populate optimal path based on the optimal parents found before this
            while (current_key != start_key) {
                Cell path_cell = decodeCell(current_key);
                optimal_path.push_back(safe_grid.cellToWorld(path_cell));
                current_key = parent_map.at(current_key); 
            }
            
            optimal_path.push_back(start); // Push starting cell too (since not included above)
            std::reverse(optimal_path.begin(), optimal_path.end());

            // Smooth out the generated grid path and then uniformly sample it
            return {smoothPath(optimal_path), safe_grid};
        }
        
        // Look at neighbors of current cell and populate open set with any that exist
        // Weight neighbors by actual cost and heuristic
        double current_actual_cost = cost_from_start[encodeCell(current_cell)]; // Actual cost to get to current cell (not including heuristic cost from heuristic to goal)
        for (const auto& offset_weight : CellOffsetWeights) {
            // Go through eight directions
            // Create neighbor based on added cost map above
            Cell neighbor{current_cell.row + offset_weight[0], current_cell.column + offset_weight[1]};
            
            // Ignore invalid neighbor (assuming do not want to travel through unknown if it exists)
            // Should be known map so assume this there is no way of knowing information if it is unknown (i.e. outside of closed environment)
            if (!safe_grid.inBounds(neighbor.row, neighbor.column)) continue;
            double cell_probability = safe_grid.getCell(neighbor.row, neighbor.column);
            if (cell_probability < 0.0 || cell_probability > OBSTACLE_THRESHOLD) continue;

            // Calculate cost of the step
            // Treat all cells as equally valid as long as they are below the threshold
            // TODO: Could incorporate cell_probability here if unsafe paths are selected
            double new_actual_cost = current_actual_cost + offset_weight[2]; // Cost not including heuristic portion
            int64_t neighbor_key = encodeCell(neighbor);
            auto cost_iterator = cost_from_start.find(neighbor_key);
            
            // Update cost of the neighbor if it is cheaper or if there was no cost before
            if (cost_iterator == cost_from_start.end() || new_actual_cost < cost_iterator->second) {
                cost_from_start[neighbor_key] = new_actual_cost;
                parent_map[neighbor_key] = encodeCell(current_cell); // Say current cell is the new / first parent of this neighbor
                double new_estimated_cost = new_actual_cost + octileHeuristic(neighbor, goal_cell); // Say cost of cell is actual cost plus cost from octile heuristic
                open_set.push({new_estimated_cost, neighbor});
            }
        }
    }
    
    // All neighbors searched but the goal was not reached
    // Return empty path on safe grid
    return {std::vector<Waypoint>(), safe_grid};
}

//  Create finely sampled smooth curves using B-spline
// After curves are finely sampled with correct shape then resample the trajectory with the intended width of points
std::vector<Waypoint> AStarPathPlanner::smoothPath(const std::vector<Waypoint>& path) const {
    // Repeat first and last points so spline interpolates endpoints correctly
    // Everything else is just copied directly
    std::vector<Waypoint> path_with_guards;
    path_with_guards.push_back(path.front());
    for (const auto& waypoint : path) {
        path_with_guards.push_back(waypoint);
    }
    path_with_guards.push_back(path.back());

    // Fine sampling of the spline curve
    // Begin populating a separate vector with the curved points
    std::vector<Waypoint> curve;
    curve.push_back(path_with_guards.front());

    for (std::size_t point = 1; point + 2 < path_with_guards.size(); point++) {
        // Points to locally fit the curve other
        const Waypoint& p0 = path_with_guards[point - 1];
        const Waypoint& p1 = path_with_guards[point];
        const Waypoint& p2 = path_with_guards[point + 1];
        const Waypoint& p3 = path_with_guards[point + 2];

        // Fitting this curve to the points (fourth order)
        // b0(t) = (-t³ + 3t² - 3t + 1) / 6
        // b1(t) = ( 3t³ - 6t²       + 4) / 6
        // b2(t) = (-3t³ + 3t² + 3t  + 1) / 6
        // b3(t) = (  t³              ) / 6
        for (int step = 1; step <= steps_per_span_; step++) {
            double t  = static_cast<double>(step) / steps_per_span_;
            double t2 = t * t;
            double t3 = t2 * t;
            double b0 = (-t3 + 3.0*t2 - 3.0*t + 1.0) / 6.0;
            double b1 = ( 3.0*t3 - 6.0*t2 + 4.0)     / 6.0;
            double b2 = (-3.0*t3 + 3.0*t2 + 3.0*t + 1.0) / 6.0;
            double b3 = ( t3)                          / 6.0;

            // Add fine curve points to the newly curved path
            curve.push_back({
                b0*p0.x + b1*p1.x + b2*p2.x + b3*p3.x,
                b0*p0.y + b1*p1.y + b2*p2.y + b3*p3.y
            });
        }
    }

    // After curves are introduced, then resample the path at the desired subsampling
    // Also ensure that the curve ends exactly where it is supposed to
    if (std::hypot(curve.back().x - path.back().x, curve.back().y - path.back().y) > 1e-9) {
        curve.back() = path.back();
    }
    return densifyPath(curve);
}

// Walk along path in increments of the internal resample spacing
// Starting and goal waypoints are still included exactly
std::vector<Waypoint> AStarPathPlanner::densifyPath(const std::vector<Waypoint>& path) const {
    std::vector<Waypoint> dense_path;
    dense_path.push_back(path.front());
    double carry = 0.0; // Leftover distance from the previous segment that should be incorporating in next segmenet to avoid bunching around original points

    // Slice the space between each successive point (including carryout from potential overshoot over current point)
    for (std::size_t point = 0; point + 1 < path.size(); ++point) {
        const Waypoint& a = path[point];
        const Waypoint& b = path[point + 1];
        const double dx = b.x - a.x;
        const double dy = b.y - a.y;
        const double distance = std::sqrt(dx*dx + dy*dy);
        if (distance < 1e-9) continue;
        double remaining = resample_spacing_ - carry;

        // Add waypoints until distance is satisfied
        while (remaining <= distance) {
            const double frac = remaining / distance;
            dense_path.push_back({a.x + frac * dx, a.y + frac * dy});
            remaining += resample_spacing_;
        }

        // Leftover distance within this segment carries into the next
        carry = distance - (remaining - resample_spacing_);
    }

    // Always end exactly at the goal in the case where the dense path does not place an end point close enough to the original
    if (std::hypot(dense_path.back().x - path.back().x, dense_path.back().y - path.back().y) > 1e-9) {
        dense_path.back() = path.back();
    }
    return dense_path;
}

} 
