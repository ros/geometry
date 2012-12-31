cmake_minimum_required(VERSION 2.8)
project(tf_conversions)

find_package(Eigen REQUIRED)
find_package(catkin REQUIRED geometry_msgs kdl_conversions tf)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  DEPENDS Eigen
  CATKIN_DEPENDS geometry_msgs kdl_conversions tf
)

include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/tf_kdl.cpp
  src/tf_eigen.cpp
)
add_dependencies(${PROJECT_NAME} geometry_msgs_gencpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

catkin_add_gtest(test_eigen_tf test/test_eigen_tf.cpp)
catkin_add_gtest(test_kdl_tf test/test_kdl_tf.cpp)
target_link_libraries(test_eigen_tf ${PROJECT_NAME})
target_link_libraries(test_kdl_tf ${PROJECT_NAME})

catkin_add_nosetests(test/posemath.py)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
