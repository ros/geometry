cmake_minimum_required(VERSION 2.8)
project(geometry)

find_package(catkin REQUIRED)
find_package(ROS COMPONENTS
  rostime cpp_common roscpp_serialization roscpp_traits # serialization
  roscpp rosconsole                                     # roscpp
  nav_msgs std_msgs                                     # messages
  )                                             # other                                                                                                  

add_subdirectory(angles)
add_subdirectory(tf)

catkin_package(geometry)
