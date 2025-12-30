cmake_minimum_required(VERSION 3.5)

project(robot_sim)

find_package(ament_cmake REQUIRED)

foreach(dir config launch rviz world)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${dir}")
    install(
      DIRECTORY ${dir}/
      DESTINATION share/${PROJECT_NAME}/${dir}
    )
  endif()
endforeach()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/scripts")
  install(
    PROGRAMS
      scripts/joint_state_mapper
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

ament_package()
