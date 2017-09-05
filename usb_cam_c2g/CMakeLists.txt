cmake_minimum_required(VERSION 2.8.3)
project(usb_cam_c2g)



find_package( OpenCV REQUIRED )
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
)


catkin_package(CATKIN_DEPENDS 

)


include_directories(
  include/usb_cam_c2g
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  
)


add_executable(usb_cam_c2g_node 
            src/usb_cam_c2g_s2p.cpp)

target_link_libraries(usb_cam_c2g_node
            ${OpenCV_LIBS}
           ${Boost_LIBRARIES})
