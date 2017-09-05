cmake_minimum_required(VERSION 2.8.3)
project(serial_com)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  serial
  std_msgs
)
catkin_package(
)

include_directories(
              ${catkin_INCLUDE_DIRS}
)

add_executable(serial_com_read_node 
                src/serial_com_read.cpp
                src/dev_mavlink.cpp
)
target_link_libraries(serial_com_read_node
                ${catkin_LIBRARIES}
)