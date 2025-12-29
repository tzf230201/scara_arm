cmake_minimum_required(VERSION 3.5)

project(robot_sim)

find_package(ament_cmake REQUIRED)

foreach(dir config launch world)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${dir}")
    install(
      DIRECTORY ${dir}/
      DESTINATION share/${PROJECT_NAME}/${dir}
    )
  endif()
endforeach()

ament_package()
