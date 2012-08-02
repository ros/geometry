project(tf_conversions)

catkin_project(${PROJECT_NAME} 
  LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS include)

include_directories(include)

find_package(tf)
include_directories(${tf_INCLUDE_DIRS})

find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

find_package(Eigen REQUIRED)
include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS})

find_package(Orocos-KDL REQUIRED)
include_directories(${Orocos-KDL_INCLUDE_DIRS})
link_directories(${Orocos-KDL_LIBRARY_DIRS})

add_library(${PROJECT_NAME}
  src/tf_kdl.cpp
  src/tf_eigen.cpp
  )
target_link_libraries(${PROJECT_NAME} ${tf_LIBRARIES} ${Orocos-KDL_LIBRARIES})

add_gtest(test_eigen_tf test/test_eigen_tf.cpp)
add_gtest(test_kdl_tf test/test_kdl_tf.cpp)
target_link_libraries(test_eigen_tf ${PROJECT_NAME} )
target_link_libraries(test_kdl_tf ${PROJECT_NAME} )

#add_pyunit(test/posemath.py)
# does CATKIN suppor pyunit?


# for compatibility with old build system
install(FILES manifest.xml
        DESTINATION share/${PROJECT_NAME})
