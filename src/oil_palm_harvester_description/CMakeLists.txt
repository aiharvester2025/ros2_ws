cmake_minimum_required(VERSION 3.8)
project(oil_palm_harvester_description)
find_package(ament_cmake REQUIRED)
install(DIRECTORY urdf meshes launch config worlds preview DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
	scripts/publish_urdf.py
	DESTINATION share/${PROJECT_NAME}/scripts
)
ament_package()
