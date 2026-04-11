#include <iostream>
#include <string>

#include "path_planning.hpp"
#include "map_reader.hpp"

int main(int argc, char** argv) {
    using namespace EmbeddedNav;

    // Parse args (need to pass in map file and visualize flag if a png should be made)
    if (argc < 2) {
        std::cout << "Usage: ./nav_stack <map_path> [--visualize]" << std::endl;
        return 1;
    }
    std::string map_path = argv[1];
    bool should_visualize = (argc > 2 && std::string(argv[2]) == "--visualize");

    try {
        // Parse the map and pass it to the planner
        MapPackage map_pkg = loadMap(map_path);
        AStarPathPlanner planner(map_pkg.grid);

        // Run the path planner and visualize if the flag is present
        // The goal and start is included in the .dat file itself (annoying to make into args)
        auto path = planner.plan_path(map_pkg.start, map_pkg.goal);

        if (path.empty()) {
            std::cout << "No path found" << std::endl;
            return 1;
        }

        if (should_visualize) {
            visualizeTrajectory(map_pkg.grid, path, map_pkg.start, map_pkg.goal);
        }

    } catch (const std::exception& e) {
        std::cerr << "Load / planning error " << e.what() << std::endl;
        return 1;
    }

    return 0;
}