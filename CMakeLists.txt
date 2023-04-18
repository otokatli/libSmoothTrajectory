cmake_minimum_required(VERSION 3.10)

project(SmoothTrajectory VERSION 0.1
                         DESCRIPTION "Smooth trajectory generator"
                         LANGUAGES CXX)

set(ENV{EIGEN3_INCLUDE_DIR} /usr/include/eigen3)
find_package( Eigen3 REQUIRED )

add_library(SmoothTrajectoryLib STATIC src/smoothTrajectory.cpp include/smoothTrajectory.h)

target_include_directories(SmoothTrajectoryLib PUBLIC include)
target_include_directories(SmoothTrajectoryLib PUBLIC $ENV{EIGEN3_INCLUDE_DIR})

target_compile_definitions(SmoothTrajectoryLib PUBLIC cxx_std_17)

add_executable(trajectory test/trajectory.cpp)

target_link_libraries(trajectory PUBLIC SmoothTrajectoryLib)