cmake_minimum_required(VERSION 2.8.3)
project(epsilon_arm)

find_package(catkin_simple REQUIRED)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

find_package(Eigen REQUIRED)
find_package(irb120_ik REQUIRED)

include_directories(${Eigen_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})
catkin_simple()

# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries
# cs_add_libraries(my_lib src/my_lib.cpp)   

# Executables

# cs_add_executable(example src/example.cpp)
cs_add_executable(irb120_int src/irb120_int.cpp)
cs_add_executable(pcd_saver src/pcd_saver.cpp)
cs_add_executable(can_finder src/can_finder.cpp)
# target_link_library(example my_lib)
#target_link_libraries(epsilon_process_pcl ${PCL_LIBRARIES})
#target_link_libraries(epsilon_process_pcl_v2 ${PCL_LIBRARIES})
#target_link_libraries(pcd_save ${PCL_LIBRARIES}) #don't need this?
cs_install()
cs_export()
    
