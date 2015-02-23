cmake_minimum_required(VERSION 2.8.3)
project(ps4_epsilon)


find_package(catkin_simple REQUIRED)

catkin_simple()

# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
 SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries
# cs_add_libraries(my_lib src/my_lib.cpp)   

# Executables
# cs_add_executable(lidar_alarm_epsilon_v2 src/lidar_alarm_epsilon_v2.cpp)
cs_add_executable(lidar_alarm_epsilon_v3 src/lidar_alarm_epsilon_v3.cpp)
cs_add_executable(estop_listener_epsilon_v2 src/estop_listener_epsilon_v2.cpp)
cs_add_executable(vel_sched_epsilon_v2 src/vel_sched_epsilon_v2.cpp)
# cs_add_executable(vel_scheduler-alaa_v2 src/vel_scheduler-alaa_v2.cpp)

# target_link_library(example my_lib)
 
cs_install()
cs_export()
    
