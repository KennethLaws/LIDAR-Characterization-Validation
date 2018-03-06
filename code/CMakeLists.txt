cmake_minimum_required(VERSION 2.4.6)
set(CMAKE_CXX_FLAGS "-std=c++0x -isystem /opt/ros/groovy/include -pthread -Wall -Wextra")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
include_directories("/home/earthmine/Vishi/real_time_lidar")



add_executable(velodyne_node velodyne_node.cpp VelodyneMonitor.cpp)
target_link_libraries(velodyne_node boost_thread boost_filesystem boost_system rt)
