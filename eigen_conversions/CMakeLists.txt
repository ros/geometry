project(eigen_conversions)

catkin_project(${PROJECT_NAME} 
  LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS include
  DEPENDS tf Orocos-KDL Eigen)

include_directories(include)

find_package(tf)
include_directories(${tf_INCLUDE_DIRS})

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})

find_package(Orocos-KDL REQUIRED)
include_directories(${Orocos-KDL_INCLUDE_DIRS})
link_directories(${Orocos-KDL_LIBRARY_DIRS})

add_library(${PROJECT_NAME}
  src/eigen_msg.cpp
  src/eigen_kdl.cpp
  )

target_link_libraries(${PROJECT_NAME}  ${tf_LIBRARIES} ${Orocos-KDL_LIBRARIES})

# for compatibility with old build system
install(FILES manifest.xml
        DESTINATION share/${PROJECT_NAME})
