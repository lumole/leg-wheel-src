cmake_minimum_required(VERSION 3.0.2)
project(udpmaster_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message
  message_filters
  message_generation
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  config/init.cfg
)
find_package(yaml-cpp REQUIRED)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES udpmaster_pkg
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

find_package(yaml-cpp REQUIRED)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

ADD_EXECUTABLE(master_relay 
  src/master.cpp
  src/udp_master.cpp
)


TARGET_LINK_LIBRARIES(master_relay
    ${catkin_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
)



