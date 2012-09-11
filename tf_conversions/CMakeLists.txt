project(tf_conversions)

catkin_project(${PROJECT_NAME} 
  LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS include
  DEPENDS tf)

include_directories(include)

find_package(tf)
include_directories(${tf_INCLUDE_DIRS})

find_package(Eigen REQUIRED)
include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS})

find_package(Orocos-KDL REQUIRED)
include_directories(${Orocos-KDL_INCLUDE_DIRS})
link_directories(${Orocos-KDL_LIBRARY_DIRS})

find_package(kdl_conversions REQUIRED)
include_directories(${kdl_conversions_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/tf_kdl.cpp
  src/tf_eigen.cpp
  )
target_link_libraries(${PROJECT_NAME} ${tf_LIBRARIES} ${Orocos-KDL_LIBRARIES} ${kdl_conversions_LIBRARIES})

catkin_add_gtest(test_eigen_tf test/test_eigen_tf.cpp)
catkin_add_gtest(test_kdl_tf test/test_kdl_tf.cpp)
target_link_libraries(test_eigen_tf ${PROJECT_NAME} )
target_link_libraries(test_kdl_tf ${PROJECT_NAME} )

#add_pyunit(test/posemath.py)
# does CATKIN suppor pyunit?

install(DIRECTORY include/
  DESTINATION include)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib)
