#include "path_planning.hpp"

namespace EmbeddedNav {

class Visualizer {
public:
    Visualizer(const OccupancyGrid& true_grid, 
                const OccupancyGrid& inflated_grid, 
                const std::vector<Waypoint>& planned_path, 
                const Waypoint& start, 
                const Waypoint& goal,
                const double obstacle_threshold);

    // Plot the smoothed trajectory returned from A* on top of the raw grid and the processed grid
    // Save the Matplot++ visualization (generated from path_planning) to a png
    void plotPathAndGrids();

    // Draw the inflated obstacle grid, the A* path, and the LQR tracked path
    void plotTracking(const std::vector<Waypoint>& tracked_path);
    void plotTrackingComparison(const std::vector<Waypoint>& true_path,
                                const std::vector<Waypoint>& measured_path,
                                const std::vector<Waypoint>& estimated_path);

private:
    OccupancyGrid true_grid_;
    OccupancyGrid safe_grid_;
    std::vector<Waypoint> nominal_path_;
    Waypoint start_, goal_;
    double obstacle_threshold_;
};

}