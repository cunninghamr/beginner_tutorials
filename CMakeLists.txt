cmake_minimum_required(VERSION 2.8.3)
project(beginner_tutorials)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  genmsg
  message_generation
  tf
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_service_files(
  FILES
  SetMsg.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(talker src/talker.cpp src/set_msg_service.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker beginner_tutorials_generate_messages_cpp ${beginner_tutorials_EXPORTED_TARGETS})

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(set_msg_service_test test/set_msg_service_test.launch test/set_msg_service_test.cpp)
  target_link_libraries(set_msg_service_test ${catkin_LIBRARIES})
  add_dependencies(set_msg_service_test talker ${catkin_EXPORTED_TARGETS})
endif()
