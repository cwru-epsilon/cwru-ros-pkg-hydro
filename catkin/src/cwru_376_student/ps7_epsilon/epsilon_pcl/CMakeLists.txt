cmake_minimum_required(VERSION 2.8.3)
project(epsilon_pcl)

find_package(catkin_simple REQUIRED)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

find_package(Eigen REQUIRED)

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

#cs_add_executable(abby_simu src/abby_simu.cpp)
#cs_add_executable(abby_simu_v1 src/abby_simu_v1.cpp)

# cs_add_executable(example src/example.cpp)
cs_add_executable(epsilon_process_pcl src/epsilon_process_pcl.cpp)

cs_add_executable(epsilon_process_pcl_v2 src/epsilon_process_pcl_v2.cpp)
cs_add_executable(epsilon_pcd_save src/epsilon_pcd_save.cpp)
cs_add_executable(epsilon_process_pcd src/epsilon_process_pcd.cpp)

#cs_add_executable(epsilon_process_pcd_dev src/epsilon_process_pcd_dev.cpp)
cs_add_executable(epsilon_find_can src/epsilon_find_can.cpp)
# target_link_library(example my_lib)
target_link_libraries(epsilon_process_pcl ${PCL_LIBRARIES})
target_link_libraries(epsilon_process_pcl_v2 ${PCL_LIBRARIES})
#target_link_libraries(pcd_save ${PCL_LIBRARIES}) #don't need this?
cs_install()
cs_export()
    
