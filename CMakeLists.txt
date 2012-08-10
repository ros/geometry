cmake_minimum_required(VERSION 2.8)
project(geometry)

find_package(catkin REQUIRED)

catkin_stack()

add_subdirectory(tf)
add_subdirectory(kdl_conversions)
add_subdirectory(tf_conversions)
add_subdirectory(eigen_conversions)

