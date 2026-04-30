# embedded-nav
Discrete-Time Navigation Stack
Belle Connaught & Ethan Crook
Carnegie Mellon University
18-675 Spring 2025

## Running Instructions

### Dependencies
The project has all of its dependencies included in the `Dockerfile` at the root of the project for easy replication. Open the root folder in a devcontainer according to these specifications. It is highly recommded to use VS Code with the Dev Containers Extension to take advantage of the `.devcontainer` includes. If it is preferred to install all of the dependencies locally though, everything you need can just be copied straight from the `Dockerfile` (i.e. just run the commands in that file locally).

### Build Configuration
Once in the container or with all of the dependencies installed locally, use the default cmake build location setup in the preset. Run `cmake --preset default` from the container root to configure output directory as `build`. Should only need to run this once per session unless additional libraries added to `CMakeLists.txt`.

After using default cmake present, run `cmake --build --preset default` to populate the binary in the `build` directory. The exectuable can just be run from there. This should be the only command needed after just making changes to source files (i.e. don't need to configure again unless changing libraries). 

### Running Experiments

Once a build is populated in `build`, you can run the binary (called `nav_stack`) from the root of the container by running the following command: 

```bash
./build/nav_stack <map_path> [--visualize]
```

Examples for running on our provided map examples include:

```bash
./build/nav_stack ./maps/test_box.dat [--visualize]
```

```bash
./build/nav_stack ./maps/test_combination.dat [--visualize]
```

Feel free to run the navigation stack on any of the maps included in the `maps` directory. To change starting and ending position for the robot navigation, change the "start_point" and "goal_point" fields in the metadata section. These fields should be global coordinates not cell indices. 

The optional--visualize arguement seen above just relates to if it is desired to see the trajectory and the error graphics. Run without this arguement to assess speed, and run with this arguement to see outputted trajectory and error metrics. If graphics are generated, they will be populated as PNGs in the `outputs` directory generated at the container root. Legends are very annoying with the native `Matplot++`, so please refer to the report or to `visualizer.cpp` directly for indications on what each line and symbol represents (although for qualitative purposes this is self explanatory). 