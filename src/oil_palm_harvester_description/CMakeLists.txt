cmake_minimum_required(VERSION 3.8)
project(oil_palm_harvester_description)
find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

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

add_library(harvester_range_sensor_panel SHARED
	src/range_sensor_panel.cpp
	include/oil_palm_harvester_description/range_sensor_panel.hpp
)
set_target_properties(harvester_range_sensor_panel PROPERTIES AUTOMOC ON)
target_compile_features(harvester_range_sensor_panel PUBLIC cxx_std_14)
target_include_directories(harvester_range_sensor_panel PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
)
target_link_libraries(harvester_range_sensor_panel Qt5::Widgets)
ament_target_dependencies(harvester_range_sensor_panel
	rclcpp sensor_msgs std_msgs pluginlib rviz_common
)

pluginlib_export_plugin_description_file(
	rviz_common rviz/range_sensor_panel_plugin.xml
)

install(DIRECTORY urdf meshes launch config worlds gazebo_model gazebo_model_database preview rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(TARGETS
	harvester_kinematic_gazebo_plugin
	harvester_range_sensor_panel
	LIBRARY DESTINATION lib
)
install(PROGRAMS
	scripts/publish_urdf.py
	scripts/gazebo_joint_state_bridge.py
	scripts/gazebo_base_kinematic_controller.py
	scripts/lidar_timestamp_bridge.py
	scripts/camera_view_selector.py
	scripts/camera_lidar_projection.py
	scripts/range_sensor_label_publisher.py
	scripts/range_sensor_calibration.py
	scripts/validate_range_sensor_calibration.py
	scripts/validate_camera_lidar_calibration.py
	DESTINATION share/${PROJECT_NAME}/scripts
)
install(FILES
	scripts/range_sensor_calibration_common.py
	scripts/camera_lidar_calibration_common.py
	DESTINATION share/${PROJECT_NAME}/scripts
)
ament_package()
