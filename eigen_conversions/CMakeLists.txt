cmake_minimum_required(VERSION 2.8)
project(eigen_conversions)

find_package(catkin REQUIRED)
catkin_package(LIBRARIES ${PROJECT_NAME}
               INCLUDE_DIRS include
)

find_package(Eigen REQUIRED)
find_package(Orocos-KDL REQUIRED)
find_package(catkin REQUIRED geometry_msgs std_msgs)
include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS}
                           ${geometry_msgs_INCLUDE_DIRS}
                           ${Orocos-KDL_INCLUDE_DIRS}
                           ${std_msgs_INCLUDE_DIRS}
)
include_directories(include)

link_directories(${Orocos-KDL_LIBRARY_DIRS})

add_library(${PROJECT_NAME}
  src/eigen_msg.cpp
  src/eigen_kdl.cpp
  )

target_link_libraries(${PROJECT_NAME}  ${Orocos-KDL_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
