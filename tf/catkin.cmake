cmake_minimum_required(VERSION 2.8)
project(tf)

include_directories(include)
include_directories(${ROS_INCLUDE_DIRS})

include_directories(/usr/local/include/bullet) # my local install of bullet
find_package(angles)
include_directories(${angles_INCLUDE_DIRS})

add_message_files(
  DIRECTORY msg
  FILES tfMessage.msg)

add_service_files(
  DIRECTORY srv
  FILES FrameGraph.srv)

generate_messages(DEPENDENCIES geometry_msgs std_msgs)

add_library(tf src/tf.cpp src/transform_listener.cpp src/cache.cpp src/transform_broadcaster.cpp)
#target_link_library(tf thread signals)

add_dependencies(tf tf_gencpp)

install_cmake_infrastructure(tf
  VERSION 0.0.0
  INCLUDE_DIRS include
  )
