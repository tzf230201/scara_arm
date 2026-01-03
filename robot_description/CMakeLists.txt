cmake_minimum_required(VERSION 3.5)

project(robot_description)

find_package(ament_cmake REQUIRED)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/scripts")
	install(
		PROGRAMS
			scripts/joint_state_mapper
			scripts/pvt_to_joint
		DESTINATION lib/${PROJECT_NAME}
	)
endif()

# Install description assets (folder may be absent in some checkouts)
foreach(dir config launch meshes urdf rviz)
	if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${dir}")
		install(
			DIRECTORY ${dir}/
			DESTINATION share/${PROJECT_NAME}/${dir}
		)
	endif()
endforeach()

ament_package()
