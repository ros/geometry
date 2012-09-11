project(kdl_conversions)

catkin_project(${PROJECT_NAME} 
  LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS include
  )

include_directories(include)

find_package(Orocos-KDL REQUIRED)
include_directories(${Orocos-KDL_INCLUDE_DIRS})
link_directories(${Orocos-KDL_LIBRARY_DIRS})

add_library(${PROJECT_NAME}
  src/kdl_msg.cpp
  )

target_link_libraries(${PROJECT_NAME} ${Orocos-KDL_LIBRARIES})

install(DIRECTORY include/
  DESTINATION include)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib)
