cmake_minimum_required(VERSION 3.21)
project(embedded-nav)

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Matplot++ REQUIRED)

add_executable(nav_stack
    src/main.cpp
    src/path_planning.cpp
    src/map_reader.cpp
    src/lqr_controller.cpp
)

target_include_directories(nav_stack PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(nav_stack 
    Eigen3::Eigen
    Matplot++::matplot
)
