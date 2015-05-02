cmake_minimum_required(VERSION 2.8.3)
project(epsilon_motion)

find_package(catkin_simple REQUIRED)

catkin_simple()

# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

# C++0x support - not quite the same as final C++11!
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries
# cs_add_libraries(my_lib src/my_lib.cpp)   

# Executables
cs_add_executable(estop_listener src/estop_listener.cpp)
cs_add_executable(lidar_alarm src/lidar_alarm.cpp)
cs_add_executable(dead_reck src/dead_reck.cpp)
cs_add_executable(leave_table src/leave_table.cpp)
# target_link_library(example my_lib)

cs_install()
cs_export()
    
