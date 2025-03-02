cmake_minimum_required(VERSION 3.0.2)
project(control)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  Params.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

## Declare catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

## Specify additional locations of header files
include_directories(${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include/control)

## Declare C++ executable
add_executable(${PROJECT_NAME} 
  src/main.cpp
  src/file1.cpp
  src/can_handler.cpp
)

## Link libraries to executable
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(zlis42_sender 
src/zlis42_sender.cpp)
target_link_libraries(zlis42_sender ${catkin_LIBRARIES})

add_executable(zlis42_receiver 
src/zlis42_receiver.cpp
src/can_reader.cpp
)
target_link_libraries(zlis42_receiver ${catkin_LIBRARIES})

add_executable(zlac706_sender 
src/zlac706_sender.cpp)
target_link_libraries(zlac706_sender ${catkin_LIBRARIES})

add_executable(zlac706_receiver 
src/zlac706_receiver.cpp
src/can_reader.cpp
)
target_link_libraries(zlac706_receiver ${catkin_LIBRARIES})