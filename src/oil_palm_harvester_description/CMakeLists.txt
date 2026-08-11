cmake_minimum_required(VERSION 3.8)
project(oil_palm_harvester_description)
find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(harvester_kinematic_gazebo_plugin SHARED
	src/harvester_kinematic_gazebo_plugin.cpp
)
target_compile_features(harvester_kinematic_gazebo_plugin PUBLIC cxx_std_14)
target_include_directories(harvester_kinematic_gazebo_plugin PRIVATE
	${gazebo_dev_INCLUDE_DIRS}
	${gazebo_ros_INCLUDE_DIRS}
)
target_link_libraries(harvester_kinematic_gazebo_plugin
	${gazebo_dev_LIBRARIES}
	${gazebo_ros_LIBRARIES}
)
ament_target_dependencies(harvester_kinematic_gazebo_plugin
	rclcpp sensor_msgs geometry_msgs tf2_ros
)

install(DIRECTORY urdf meshes launch config worlds gazebo_model gazebo_model_database preview rviz DESTINATION share/${PROJECT_NAME})
install(TARGETS harvester_kinematic_gazebo_plugin LIBRARY DESTINATION lib)
install(PROGRAMS
	scripts/publish_urdf.py
	scripts/gazebo_joint_state_bridge.py
	scripts/gazebo_base_kinematic_controller.py
	DESTINATION share/${PROJECT_NAME}/scripts
)
ament_package()
