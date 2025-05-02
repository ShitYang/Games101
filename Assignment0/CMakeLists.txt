cmake_minimum_required(VERSION 3.10)
project (Assignment0)

find_package(Eigen3 REQUIRED)
include_directories(EIGEN3_INCLUDE_DIR)

message(WARNING ${EIGEN3_LIBRARIES})

add_executable (Assignment0 main.cpp)
