project(tf)

catkin_project(${PROJECT_NAME} 
  LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS include
  DEPENDS Boost)

include_directories(include)

find_package(Boost REQUIRED thread signals)
include_directories(SYSTEM ${Boost_INCLUDE_DIR})

find_package(genmsg REQUIRED)
find_package(ROS REQUIRED COMPONENTS roscpp std_msgs geometry_msgs sensor_msgs)
include_directories(${ROS_INCLUDE_DIRS})

add_message_files(DIRECTORY msg FILES tfMessage.msg)
add_service_files(DIRECTORY srv FILES FrameGraph.srv)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)


add_library(${PROJECT_NAME} src/tf.cpp src/transform_listener.cpp src/cache.cpp src/transform_broadcaster.cpp)
target_link_libraries(${PROJECT_NAME} ${ROS_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)

# Debug
add_executable(tf_empty_listener src/empty_listener.cpp)
target_link_libraries(tf_empty_listener ${PROJECT_NAME})

add_executable(tf_echo src/tf_echo.cpp)
target_link_libraries(tf_echo ${PROJECT_NAME})

add_executable(tf_change_notifier src/change_notifier.cpp)
target_link_libraries(tf_change_notifier ${PROJECT_NAME})

add_executable(tf_monitor src/tf_monitor.cpp)
target_link_libraries(tf_monitor ${PROJECT_NAME})

catkin_initialize_tests()

add_gtest(tf_unittest test/tf_unittest.cpp)
target_link_libraries(tf_unittest ${PROJECT_NAME})

add_gtest(test_transform_datatypes test/test_transform_datatypes.cpp)
target_link_libraries(tf_unittest ${PROJECT_NAME})

add_gtest(transform_listener_unittest test/transform_listener_unittest.cpp)
target_link_libraries(transform_listener_unittest ${PROJECT_NAME})
#add_rostest(test/transform_listener_unittest.launch) # how does CATKIN do rostests?

# Disabled because of changes in TransformStorage
#add_gtest_future(tf_unittest_future test/tf_unittest_future.cpp)
#target_link_libraries(tf_unittest_future ${PROJECT_NAME})

add_gtest(test_velocity test/velocity_test.cpp)
target_link_libraries(test_velocity ${PROJECT_NAME})

#add_executable(test_transform_twist test/transform_twist_test.cpp)
#target_link_libraries(test_transform_twist ${PROJECT_NAME})
#add_gtest_build_flags(test_transform_twist)
#add_rostest(test/transform_twist_test.launch)

add_gtest(cache_unittest test/cache_unittest.cpp)
target_link_libraries(cache_unittest ${PROJECT_NAME})

add_gtest(test_message_filter EXCLUDE_FROM_ALL test/test_message_filter.cpp)
target_link_libraries(test_message_filter tf ${Boost_LIBRARIES})

# HOW DOES CATKIN do ROSTEST?
# add_rostest(test/test_message_filter.xml)

### Benchmarking
#add_gtest_future(tf_benchmark test/tf_benchmark.cpp)
#target_link_libraries(tf_benchmark ${PROJECT_NAME})

add_executable(btTest test/quaternion.cpp)

add_gtest(testListener test/testListener.cpp)
target_link_libraries(testListener ${PROJECT_NAME})

# HOW DOES CATKIN do ROSTEST?
# add_rostest(test/test_broadcaster.launch)
                                 
add_executable(testBroadcaster test/testBroadcaster.cpp)
target_link_libraries(testBroadcaster ${PROJECT_NAME})
add_executable(static_transform_publisher src/static_transform_publisher.cpp)
target_link_libraries(static_transform_publisher ${PROJECT_NAME})


find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_PATH})

# # If on Darwin, create a symlink _foo.so -> _foo.dylib, because the
# # MacPorts version of Python won't find _foo.dylib for 'import _foo'
# include(CMakeDetermineSystem)
# if(CMAKE_SYSTEM_NAME MATCHES "Darwin")
#   add_custom_command(OUTPUT ${LIBRARY_OUTPUT_PATH}/_${PROJECT_NAME}_swig.so
#                      COMMAND cmake -E create_symlink  ${LIBRARY_OUTPUT_PATH}/_${PROJECT_NAME}_swig.dylib ${LIBRARY_OUTPUT_PATH}/_${PROJECT_NAME}_swig.so
#                      DEPENDS python_${PROJECT_NAME})
#   add_custom_target(symlink_darwin_lib ALL
#                     DEPENDS ${LIBRARY_OUTPUT_PATH}/_${PROJECT_NAME}_swig.so)
# endif(CMAKE_SYSTEM_NAME MATCHES "Darwin")


# Check for SSE
#check_for_sse()

# Dynamic linking with tf worked OK, except for exception propagation, which failed in the unit test.
# so build with the objects directly instead.

link_libraries(${PYTHON_LIBRARIES})
add_library(pytf_py src/pytf.cpp src/tf.cpp src/transform_listener.cpp src/cache.cpp)
find_package(PythonLibs REQUIRED)
set_target_properties(pytf_py PROPERTIES OUTPUT_NAME tf PREFIX "_" SUFFIX ".so")
# target_link_libraries(pytf_py ${PROJECT_NAME})
# add_compile_flags(pytf_py -g -Wno-missing-field-initializers)
#add_compile_flags(pytf_py ${SSE_FLAGS}) #conditionally adds sse flags if available
# add_pyunit(test/testPython.py) 
# DOES PYUNIT WORK IN CATKIN?
target_link_libraries(pytf_py ${Boost_LIBRARIES})

add_executable(tf_speed_test EXCLUDE_FROM_ALL test/speed_test.cpp)
target_link_libraries(tf_speed_test ${PROJECT_NAME})


install(DIRECTORY include/
  DESTINATION include)

install(TARGETS ${PROJECT_NAME} pytf_py
  DESTINATION lib)

install(TARGETS tf_echo tf_empty_listener tf_change_notifier tf_monitor static_transform_publisher
  DESTINATION bin)

# for compatibility with old build system
install(FILES manifest.xml
        DESTINATION share/${PROJECT_NAME})
