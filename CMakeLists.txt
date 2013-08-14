cmake_minimum_required(VERSION 2.8.3)
project(tf_lookup)

if(USE_ROSBUILD)

  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

  rosbuild_find_ros_package(actionlib_msgs)
  include(${actionlib_msgs_PACKAGE_PATH}/cmake/actionbuild.cmake)
  genaction()

  rosbuild_init()

  set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
  set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

  rosbuild_genmsg()
  rosbuild_gensrv()

  rosbuild_add_executable(tf_lookup
      src/tf_lookup_main.cpp
      src/tf_lookup.cpp
      src/tf_stream.cpp
      src/tf_stream_server.cpp)

else()

  find_package(catkin REQUIRED COMPONENTS roscpp genmsg actionlib_msgs tf)

  add_action_files(DIRECTORY action FILES TfLookup.action TfStream.action)
  add_service_files(FILES lookupTransform.srv)
  add_message_files(FILES Subscription.msg)

  generate_messages(DEPENDENCIES actionlib_msgs geometry_msgs)

  catkin_package(
      INCLUDE_DIRS include
      )

  include_directories(include ${catkin_INCLUDE_DIRS})

  add_executable(tf_lookup
      src/tf_lookup_main.cpp
      src/tf_lookup.cpp
      src/tf_stream.cpp
      src/tf_stream_server.cpp)

  target_link_libraries(tf_lookup ${catkin_LIBRARIES})
  add_dependencies(tf_lookup ${PROJECT_NAME}_gencpp)

  install(TARGETS tf_lookup
     RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
   )

  ## Mark cpp header files for installation
  install(DIRECTORY include/${PROJECT_NAME}/
     DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
     FILES_MATCHING PATTERN "*.h"
  )

endif()
