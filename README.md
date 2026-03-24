# embedded-nav
Discrete-Time Navigation Stack
Belle Connaught & Ethan Crook
Carnegie Mellon University
18-675 Spring 2025

## Running Instructions
Open the folder in a devcontainer and use the default cmake build location setup in the preset (just for version control purposes). 

Run `cmake --preset default` from the container root to configure output directory as `build`. Should only need to run this once unless additional libraries added to `CMakeLists.txt`.

Then run `cmake --build --preset default` to populate binary in `build`. The exectuable can just be run from there. This should be the only command needed after just making changes to source files (i.e. don't need to configure again unless changing libraries). 