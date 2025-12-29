cmake_minimum_required(VERSION 3.5)

project(robot_description)

find_package(ament_cmake REQUIRED)

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
